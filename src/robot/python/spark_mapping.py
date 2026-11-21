"""Portable encoder/gyro odometry, a small ToF discovery map and scan matching.

No simulator imports, arena geometry or object positions: the pose correction
comes only from matching new ToF hits against the map built so far (SLAM).
Prior knowledge from the rules is allowed: every cube is 6 x 6 x 6 cm and
stands free, walls are long and 10 cm high. Cubes are kept as centre estimates
(DiscoveryMap.cubes, known_cubes()). The grid keeps raw hits for scan
matching; the published map hides the wider smear around a known cube, which
the view draws at its real size instead. Cubes within the robot's reach may
be pushed and stay out of scan matching.
Copy alongside spark_tof.py to the robot. Units: metres, radians, microseconds.
The start pose is an assumption: (0, 0, 0), +x in the starting direction.
"""
import math
import json
from array import array


class Odometry:
    HISTORY = 200  # poses kept for at(); ~2 s at the usual loop rate

    def __init__(self, counts_per_m=8467, gyro_bias=0.0):
        self.x = self.y = self.heading = self.distance = 0.0
        self.counts_per_m = counts_per_m
        self.gyro_bias = gyro_bias
        self._last = None
        # Ring buffer in flat arrays: no per-sample objects that would stay
        # scattered over the RP2040 heap and fragment it.
        n = self.HISTORY
        self._t = array('l', [0] * n)  # microseconds since _t0
        self._x = array('f', [0.0] * n)
        self._y = array('f', [0.0] * n)
        self._h = array('f', [0.0] * n)
        self._t0 = None
        self.clear_history()

    def clear_history(self):
        self._count = 0
        self._next = 0

    def update(self, left, right, gyro_dps, now_us):
        if self._last is not None:
            ll, rr, previous_gyro, previous_us = self._last
            # Caller supplies a monotonic, unwrapped timestamp.
            dt = (now_us - previous_us) / 1000000
            if dt <= 0:
                return
            ds = ((left - ll) + (right - rr)) / (2 * self.counts_per_m)
            turn = ((gyro_dps + previous_gyro) / 2 - self.gyro_bias) * math.pi / 180 * dt
            mid = self.heading + turn / 2
            self.x += ds * math.cos(mid)
            self.y += ds * math.sin(mid)
            self.heading = (self.heading + turn + math.pi) % (2 * math.pi) - math.pi
            self.distance += abs(ds)
        self._last = (left, right, gyro_dps, now_us)
        if self._t0 is None:
            self._t0 = now_us
        i = self._next
        self._t[i] = now_us - self._t0
        self._x[i], self._y[i], self._h[i] = self.x, self.y, self.heading
        self._next = (i + 1) % self.HISTORY
        self._count = min(self._count + 1, self.HISTORY)

    def at(self, captured_us):
        """Estimated pose at capture, interpolated from sensor odometry ONLY.

        Return None outside the history; never assign an old image today's pose.
        The caller must align ESP and Zumo clocks before calling this method.
        """
        n, count = self.HISTORY, self._count
        if not count:
            return None
        t = captured_us - self._t0
        newest = (self._next - 1) % n
        oldest = (self._next - count) % n
        if t < self._t[oldest] or t > self._t[newest]:
            return None
        b = newest
        for k in range(count):
            a = (newest - k) % n
            if self._t[a] <= t:
                if self._t[a] == t:
                    return (self._x[a], self._y[a], self._h[a])
                f = (t - self._t[a]) / (self._t[b] - self._t[a])
                turn = (self._h[b] - self._h[a] + math.pi) % (2 * math.pi) - math.pi
                return (self._x[a] + f * (self._x[b] - self._x[a]),
                        self._y[a] + f * (self._y[b] - self._y[a]), self._h[a] + f * turn)
            b = a


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


_SLOPES = {}


def _segment_distance(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    span = dx * dx + dy * dy
    f = 0.0 if span == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / span))
    return math.sqrt((px - x0 - f * dx) ** 2 + (py - y0 - f * dy) ** 2)


# Known from the rules, not measured: every cube is 6 x 6 x 6 cm.
CUBE = 0.06
CUBE_TALL = 0.08    # a return this high at the face's depth is a wall; cube hits stay below ~7.5 cm
CUBE_STEP = 0.04    # depth step between neighbouring columns that still belongs to one face
CUBE_GAP = 0.05     # neighbour columns must be this much farther (or empty): a cube stands free
CUBE_JOIN = 0.09    # a sighting this close to a known cube is the same cube
CUBE_MEMORY = 8     # averaging weight cap, so a pushed cube can follow its new sightings
CUBE_MAX = 16       # five cubes in the arena; room for a few stray sightings on top
CUBE_STILL = 0.2    # farther than this from the robot a cube was not pushed: fine for scan matching


def cube_sightings(columns, mount_x, fov=math.pi / 4):
    """Runs of columns that look like one free-standing 6 cm cube.

    columns[i] is None or (depth, tan_lateral, tall). The sensor reports the
    strongest return per zone, so a bright cube fills every zone it barely
    touches: the run is up to one zone wider than the cube. Knowing the size,
    the centre is the middle of the run, half a cube behind the nearest face.
    Returns (x, y, first_col, last_col) in the robot frame (x forward, y left).
    """
    size = len(columns)
    zone = 2 * math.tan(fov / 2) / size
    out = []
    i = 0
    while i < size:
        if columns[i] is None:
            i += 1
            continue
        j = i
        while j + 1 < size and columns[j + 1] is not None and abs(columns[j + 1][0] - columns[j][0]) < CUBE_STEP:
            j += 1
        run = columns[i:j + 1]
        near = min(c[0] for c in run)
        far = max(c[0] for c in run)
        # Free-standing: a neighbour on both sides, each empty or clearly farther.
        # A run at the edge of the view may continue outside it: unknown.
        if 0 < i and j < size - 1:
            left, right = columns[i - 1], columns[j + 1]
            free = (left is None or left[0] > far + CUBE_GAP) and (right is None or right[0] > far + CUBE_GAP)
            span = near * (run[-1][1] - run[0][1])
            if free and not any(c[2] for c in run) and span <= CUBE + near * zone + 0.01:
                tan = (run[0][1] + run[-1][1]) / 2
                back = CUBE / 2 / math.sqrt(1 + tan * tan)
                out.append((mount_x + near + back, (near + back) * tan, i, j))
        i = j + 1
    return out


def on_cube(depth, zone):
    """How far from a known cube's centre a hit still lies on that cube:
    half a cube, plus half a zone the bright cube bleeds into."""
    return CUBE / 2 + depth * zone / 2 + 0.015


def seen_through(cube, ray):
    """A beam that passes right through a known cube and ends well behind it:
    the cube is not there any more (pushed away)."""
    ox, oy, ex, ey = ray
    dx, dy = ex - ox, ey - oy
    length = math.sqrt(dx * dx + dy * dy)
    if length == 0:
        return False
    along = ((cube[0] - ox) * dx + (cube[1] - oy) * dy) / length
    if along < 0 or along > length - CUBE:
        return False
    # Within half a cube of the centre the bright cube would have filled that zone.
    return abs((cube[0] - ox) * dy - (cube[1] - oy) * dx) / length < CUBE / 2


class DiscoveryMap:
    # Local drawing window, NOT a known arena boundary. All cells start unknown.
    SIZE = 48
    CELL = 0.05
    MOUNT_X = 0.052
    MOUNT_Z = 0.09  # measured on the team robot: 9 cm above the floor, level
    # Obstacles the robot can hit: above floor noise, below the 10 cm walls' top.
    MIN_HEIGHT = 0.025
    MAX_HEIGHT = 0.13
    # Scan matching: only once the map holds enough confident hits to match against.
    MATCH_MIN_CELLS = 12
    MATCH_MIN_HITS = 3

    def __init__(self, odometry, match=True):
        self.pose = odometry
        self.match = match
        self.cells = array('b', [0] * (self.SIZE * self.SIZE))
        self.rays = []
        self.scan_time = None
        self.scan_pose = None
        self.sequence = None
        self.scans = 0
        self.matched = 0
        self.confident = 0  # cells with a confident hit; matching needs enough of them
        # Known cubes, map frame: [x, y, sightings]. Two sightings or more = confirmed.
        self.cubes = []
        # Rigid transform odometry frame -> map frame, found by scan matching.
        self.correction = (0.0, 0.0, 0.0)
        self._last_odom = None
        self._dirty = True
        # The published grid, kept up to date cell by cell in _mark: building it
        # at publish time costs ~60 ms and ''.join over 2304 cells a 16 KB list.
        self._grid_text = bytearray(b'0' * (self.SIZE * self.SIZE))
        self._scratch = bytearray(self.SIZE * self.SIZE)  # reused mask / grid copy: no 2 KB allocations later

    def corrected(self, pose):
        """Odometry pose -> map pose (what the robot believes after SLAM)."""
        cx, cy, ct = self.correction
        x, y, h = pose
        c, s = math.cos(ct), math.sin(ct)
        return (cx + c * x - s * y, cy + s * x + c * y, _wrap(h + ct))

    def estimate(self):
        return self.corrected((self.pose.x, self.pose.y, self.pose.heading))

    def _evidence(self, x, y):
        """Bilinear map evidence between cell centres: +1 hit .. -0.5 free, 0 unknown.

        Free space counts against a pose too: a wrong heading lays the hits into
        space that earlier scans saw through, which a hit-only score cannot tell.
        """
        half = self.SIZE * self.CELL / 2
        fx = (x + half) / self.CELL - 0.5
        fy = (half - y) / self.CELL - 0.5
        col, row = int(math.floor(fx)), int(math.floor(fy))
        if col < 0 or row < 0 or col >= self.SIZE - 1 or row >= self.SIZE - 1:
            return 0.0
        ax, ay = fx - col, fy - row
        i = row * self.SIZE + col
        c = self.cells
        a, b, d, e = c[i], c[i + 1], c[i + self.SIZE], c[i + self.SIZE + 1]
        v = ((a * (1 - ax) + b * ax) * (1 - ay) + (d * (1 - ax) + e * ax) * ay) / 6
        return v if v > 0 else v / 2

    def _score(self, hits, x, y, h, prior, sigma):
        # _evidence inlined: this runs ~100 times per scan on the RP2040.
        c, s = math.cos(h), math.sin(h)
        n, inv = self.SIZE, 1 / self.CELL
        half = n * self.CELL / 2
        cells = self.cells
        total = 0.0
        for lx, ly in hits:
            fx = (x + c * lx - s * ly + half) * inv - 0.5
            fy = (half - (y + s * lx + c * ly)) * inv - 0.5
            col, row = int(math.floor(fx)), int(math.floor(fy))
            if col < 0 or row < 0 or col >= n - 1 or row >= n - 1:
                continue
            ax, ay = fx - col, fy - row
            i = row * n + col
            v = ((cells[i] * (1 - ax) + cells[i + 1] * ax) * (1 - ay)
                 + (cells[i + n] * (1 - ax) + cells[i + n + 1] * ax) * ay) / 6
            total += v if v > 0 else v / 2
        # Stay near odometry unless the map clearly says otherwise; this keeps a
        # few hits on one straight wall from sliding the pose along that wall.
        dx, dy, dh = x - prior[0], y - prior[1], _wrap(h - prior[2])
        return total - 0.5 * ((dx * dx + dy * dy) / sigma[0] ** 2 + dh * dh / sigma[1] ** 2)

    def _match(self, hits, prior, sigma):
        """Hill-climb the pose that best lays the hits onto known obstacles."""
        best = prior
        score = self._score(hits, *prior, prior, sigma)
        for step, turn in ((0.02, 0.03), (0.01, 0.015), (0.005, 0.0075)):
            improved = True
            while improved:
                improved = False
                x, y, h = best
                for cand in ((x + step, y, h), (x - step, y, h), (x, y + step, h),
                             (x, y - step, h), (x, y, h + turn), (x, y, h - turn)):
                    value = self._score(hits, *cand, prior, sigma)
                    if value > score + 1e-6:
                        best, score, improved = cand, value, True
        # Hits that land on known obstacles after matching; few = no information.
        support = sum(1 for lx, ly in hits if self._evidence(
            best[0] + math.cos(best[2]) * lx - math.sin(best[2]) * ly,
            best[1] + math.sin(best[2]) * lx + math.cos(best[2]) * ly) > 0.3)
        if support < self.MATCH_MIN_HITS:
            return None
        return best

    def _cell(self, x, y):
        half = self.SIZE * self.CELL / 2
        col = int(math.floor((x + half) / self.CELL))
        row = int(math.floor((half - y) / self.CELL))
        if 0 <= col < self.SIZE and 0 <= row < self.SIZE:
            return row * self.SIZE + col
        return None

    def _mark(self, index, delta):
        if index is not None:
            old = self.cells[index]
            new = max(-6, min(6, old + delta))
            self.cells[index] = new
            self.confident += int(new >= 3) - int(old >= 3)
            self._grid_text[index] = 50 if new > 0 else 49 if new < 0 else 48  # '2' hit, '1' free, '0' unknown

    @staticmethod
    def _slopes(size):
        """Per-row height slope and per-column lateral slope; tan() once per size."""
        if size not in _SLOPES:
            _SLOPES[size] = ([math.tan((1 - (row + 0.5) / size * 2) * math.pi / 8) for row in range(size)],
                             [math.tan(((col + 0.5) / size * 2 - 1) * math.pi / 8) for col in range(size)])
        return _SLOPES[size]

    def observe(self, frame, now_us, capture_pose=None):
        """Fuse raw depths at an estimated capture pose; no motor commands.

        capture_pose is ODOMETRY at capture. For stationary callers it can default
        to the current odometry. Moving callers must interpolate odometry at the
        aligned capture timestamp. Scan matching then corrects it against the map.
        """
        if frame['seq'] == self.sequence:
            return
        self.sequence = frame['seq']
        size = int(math.sqrt(frame['resolution']))
        if size not in (4, 8):
            return
        odom = capture_pose if capture_pose is not None else (self.pose.x, self.pose.y, self.pose.heading)
        # Per column the nearest return that hits something robot-high. Reject
        # probable floor (too low) and returns over the walls (too high) using
        # the ASSUMED level mount, not a geometry label.
        row_slope, col_slope = self._slopes(size)
        distance, status, targets = frame['distance_mm'], frame['target_status'], frame['nb_target_detected']
        low, high, mount_z = self.MIN_HEIGHT, self.MAX_HEIGHT, self.MOUNT_Z
        columns = []
        for col in range(size):
            accepted = []
            for row in range(size):
                zone = row * size + col
                if status[zone] != 5 or targets[zone] < 1:
                    continue
                d = distance[zone] / 1000
                estimated_height = mount_z + d * row_slope[row]
                if 0.02 <= d <= 1.5 and low <= estimated_height <= high:
                    accepted.append((d, estimated_height))
            if not accepted:
                # Invalid / above objects / floor: unknown, never free space.
                columns.append(None)
                continue
            depth = min(a[0] for a in accepted)
            # ST reports optical-axis depth, not radial distance. Raw col 0 is RIGHT.
            tan = col_slope[col]
            tall = any(d - depth < CUBE_STEP and h > CUBE_TALL for d, h in accepted)
            columns.append((depth, tan, tall))
        sightings = cube_sightings(columns, self.MOUNT_X)
        # Hits in the robot frame: x forward from the robot origin, y left.
        local, used = [], []
        for col in range(size):
            if columns[col] is not None:
                depth, tan, _ = columns[col]
                local.append((self.MOUNT_X + depth, depth * tan))
                used.append(col)
        on_sighting = [any(g[2] <= col <= g[3] for g in sightings) for col in used]
        pose = self.corrected(odom)
        # How far odometry may be off since the previous scan: grows with the
        # travelled distance and the turned angle (slip, gyro scale error).
        last = self._last_odom or odom
        moved = math.sqrt((odom[0] - last[0]) ** 2 + (odom[1] - last[1]) ** 2)
        turned = abs(_wrap(odom[2] - last[2]))
        self._last_odom = odom
        sigma = (0.004 + 0.1 * moved + 0.01 * turned, 0.01 + 0.3 * turned + 0.5 * moved)
        # Match only what cannot have moved: hits on a cube within the robot's
        # reach (sighted now or known from earlier) stay out of scan matching.
        zone = 2 * math.tan(math.pi / 8) / size
        cs, sn = math.cos(pose[2]), math.sin(pose[2])
        near = [c for c in self.cubes if c[2] >= 2
                and (c[0] - pose[0]) ** 2 + (c[1] - pose[1]) ** 2 < CUBE_STILL ** 2]
        static = []
        for k in range(len(local)):
            lx, ly = local[k]
            if on_sighting[k] and lx * lx + ly * ly < CUBE_STILL ** 2:
                continue
            ex, ey = pose[0] + cs * lx - sn * ly, pose[1] + sn * lx + cs * ly
            reach = on_cube(columns[used[k]][0], zone) + 0.02  # odometry may be off before matching
            if not any((c[0] - ex) ** 2 + (c[1] - ey) ** 2 < reach * reach for c in near):
                static.append(local[k])
        if self.match and static and self.confident >= self.MATCH_MIN_CELLS:
            matched = self._match(static, pose, sigma)
            if matched is not None:
                pose = matched
                # New correction: odometry at capture maps exactly onto the matched pose.
                ct = _wrap(matched[2] - odom[2])
                c, s = math.cos(ct), math.sin(ct)
                self.correction = (matched[0] - (c * odom[0] - s * odom[1]),
                                   matched[1] - (s * odom[0] + c * odom[1]), ct)
                self.matched += 1
        px, py, heading = pose
        cs, sn = math.cos(heading), math.sin(heading)
        ox, oy = px + self.MOUNT_X * cs, py + self.MOUNT_X * sn
        rays = []
        for lx, ly in local:
            ex, ey = px + cs * lx - sn * ly, py + sn * lx + cs * ly
            rays.append([round(ox, 4), round(oy, 4), round(ex, 4), round(ey, 4)])
        self._update_cubes(rays, sightings, pose)
        # Carve free evidence first, then all endpoints: one beam must not erase
        # another beam's hit in the same frame. Endpoint cells are excluded.
        hits = []
        n, inv = self.SIZE, 1 / self.CELL
        half = n * self.CELL / 2
        cells, text = self.cells, self._grid_text
        for ox, oy, ex, ey in rays:
            hit = self._cell(ex, ey)
            hits.append(hit)
            # Same half-cell sampling as _cell() along the beam, inlined in grid units.
            gx, gy = (ox + half) * inv, (half - oy) * inv
            steps = max(1, int(math.ceil(math.sqrt((ex - ox) ** 2 + (ey - oy) ** 2) * inv * 2)))
            sx, sy = (ex - ox) * inv / steps, (oy - ey) * inv / steps
            previous = None
            for i in range(steps):
                col, row = int(math.floor(gx + sx * i)), int(math.floor(gy + sy * i))
                if col < 0 or row < 0 or col >= n or row >= n:
                    previous = None
                    continue
                index = row * n + col
                if index != hit and index != previous:
                    old = cells[index]
                    if old > -6:
                        cells[index] = old - 1
                        if old == 3:
                            self.confident -= 1
                        text[index] = 50 if old > 1 else 49 if old <= 0 else 48
                previous = index
        for hit in set(hits):
            if hit is not None:
                self._mark(hit, 3)
        self.rays = rays
        self.scan_time = now_us / 1000000
        self.scan_pose = [round(px, 4), round(py, 4), round(heading, 5)]
        self.scans += 1
        self._dirty = True

    # ---- Queries: the map as a data structure for driving programs ----------
    # Cell evidence runs -6 (seen through often) .. +6 (hit often); 0 = unknown.
    HIT = 3    # at or above: obstacle (one fresh hit is enough)
    FREE = -1  # at or below: a beam passed through

    def center(self, index):
        """Map-frame centre (x, y) of a cell index."""
        half = self.SIZE * self.CELL / 2
        row, col = divmod(index, self.SIZE)
        return ((col + 0.5) * self.CELL - half, half - (row + 0.5) * self.CELL)

    def value(self, x, y):
        """Evidence of the cell at (x, y); None outside the map window."""
        i = self._cell(x, y)
        return None if i is None else self.cells[i]

    def occupied(self, x, y):
        v = self.value(x, y)
        return v is not None and v >= self.HIT

    def seen_free(self, x, y):
        v = self.value(x, y)
        return v is not None and v <= self.FREE

    def unknown(self, x, y):
        v = self.value(x, y)
        return v is None or self.FREE < v < self.HIT

    def path_clear(self, x0, y0, x1, y1, radius=0.0, ignore=None):
        """True if no obstacle cell lies within radius of the segment.

        Unknown cells count as clear: the ToF cannot see everything, so callers
        must still watch for contact. ignore = (x, y, r): skip obstacles inside
        that circle, e.g. the cube you are about to push.
        """
        n, cell, hit = self.SIZE, self.CELL, self.HIT
        half = n * cell / 2
        reach = radius + cell * 0.71  # cell centre to its farthest corner
        c0 = max(0, int((min(x0, x1) - reach + half) / cell))
        c1 = min(n - 1, int((max(x0, x1) + reach + half) / cell))
        r0 = max(0, int((half - max(y0, y1) - reach) / cell))
        r1 = min(n - 1, int((half - min(y0, y1) + reach) / cell))
        cells = self.cells
        for row in range(r0, r1 + 1):
            base = row * n
            cy = half - (row + 0.5) * cell
            for col in range(c0, c1 + 1):
                if cells[base + col] < hit:
                    continue
                cx = (col + 0.5) * cell - half
                if ignore and (cx - ignore[0]) ** 2 + (cy - ignore[1]) ** 2 <= ignore[2] ** 2:
                    continue
                if _segment_distance(cx, cy, x0, y0, x1, y1) <= reach:
                    return False
        return True

    def _clusters(self, mask, min_cells):
        """8-connected groups of set cells in mask -> [(x, y, cells)].

        x, y is the evidence-weighted centre. One pass with a small stack; no
        per-cell objects, so this stays cheap on the RP2040 heap.
        """
        n = self.SIZE
        out = []
        for start in range(n * n):
            if not mask[start]:
                continue
            mask[start] = 0
            stack = [start]
            count = 0
            wx = wy = weight = 0.0
            while stack:
                i = stack.pop()
                row, col = divmod(i, n)
                w = abs(self.cells[i]) or 1
                cx, cy = self.center(i)
                wx += cx * w
                wy += cy * w
                weight += w
                count += 1
                for dr in (-1, 0, 1):
                    r = row + dr
                    if r < 0 or r >= n:
                        continue
                    for dc in (-1, 0, 1):
                        c = col + dc
                        if 0 <= c < n and mask[r * n + c]:
                            mask[r * n + c] = 0
                            stack.append(r * n + c)
            if count >= min_cells:
                out.append((wx / weight, wy / weight, count))
        return out

    def hits(self):
        """Yield the centre (x, y) of every obstacle cell (a generator: no big list on the heap)."""
        cells, hit = self.cells, self.HIT
        for i in range(len(cells)):
            if cells[i] >= hit:
                yield self.center(i)

    def obstacles(self, min_cells=1):
        """Groups of obstacle cells: [(x, y, cells)] in the map frame, unordered.

        Walls appear as long groups, cubes as one to four cells. The map knows no
        arena geometry; telling the two apart is the caller's job.
        """
        cells, hit = self.cells, self.HIT
        mask = self._scratch
        for i in range(len(cells)):
            mask[i] = 1 if cells[i] >= hit else 0
        return self._clusters(mask, min_cells)

    def frontiers(self, min_cells=2):
        """Groups of seen-free cells that border unknown space: where to look next."""
        c, n, free, hit = self.cells, self.SIZE, self.FREE, self.HIT
        mask = self._scratch
        for i in range(n * n):
            mask[i] = 0
            if c[i] > free:
                continue
            row, col = divmod(i, n)
            if (row and free < c[i - n] < hit or row < n - 1 and free < c[i + n] < hit
                    or col and free < c[i - 1] < hit or col < n - 1 and free < c[i + 1] < hit):
                mask[i] = 1
        return self._clusters(mask, min_cells)

    def forget(self, x, y, radius):
        """Set cells within radius back to unknown, e.g. where you just moved a cube away."""
        n, cell = self.SIZE, self.CELL
        half = n * cell / 2
        for row in range(max(0, int((half - y - radius) / cell)), min(n, int((half - y + radius) / cell) + 1)):
            for col in range(max(0, int((x - radius + half) / cell)), min(n, int((x + radius + half) / cell) + 1)):
                i = row * n + col
                cx, cy = self.center(i)
                if (cx - x) ** 2 + (cy - y) ** 2 <= radius * radius and self.cells[i]:
                    self._mark(i, -self.cells[i])
        # Known cubes there are gone too.
        self.cubes = [c for c in self.cubes if (c[0] - x) ** 2 + (c[1] - y) ** 2 > radius * radius]
        self._dirty = True

    def reset_pose(self, x=0.0, y=0.0, heading=0.0):
        """The robot was put down at a known pose (e.g. the referee's start square).

        Keeps everything the map has learned; only the odometry restarts there.
        """
        p = self.pose
        p.x, p.y, p.heading = x, y, heading
        p.clear_history()
        self.correction = (0.0, 0.0, 0.0)
        self._last_odom = None

    def _update_cubes(self, rays, sightings, pose):
        # Beams through a known cube weaken it (pushed away); then this frame's sightings.
        for cube in list(self.cubes):
            if any(seen_through(cube, r) for r in rays):
                cube[2] = min(cube[2], CUBE_MEMORY) - 1
                if cube[2] <= 0:
                    self.cubes.remove(cube)
        px, py, heading = pose
        cs, sn = math.cos(heading), math.sin(heading)
        for lx, ly, _, _ in sightings:
            x, y = px + cs * lx - sn * ly, py + sn * lx + cs * ly
            best, gap = None, CUBE_JOIN
            for cube in self.cubes:
                d = math.sqrt((cube[0] - x) ** 2 + (cube[1] - y) ** 2)
                if d < gap:
                    best, gap = cube, d
            if best is None:
                # Bounded memory: a single stray sighting makes room first.
                if len(self.cubes) >= CUBE_MAX:
                    self.cubes.remove(min(self.cubes, key=lambda c: c[2]))
                best = [x, y, 1]
                self.cubes.append(best)
            else:
                w = min(best[2], CUBE_MEMORY)
                best[0] = (best[0] * w + x) / (w + 1)
                best[1] = (best[1] * w + y) / (w + 1)
                best[2] += 1

    def _grid_bytes(self):
        """Published map: obstacle cells around a known cube are that cube seen
        too wide (a bright cube fills every ToF zone it touches). Hide them,
        the cube itself is published at its real 6 cm size. Returns a bytearray
        (the scratch buffer when something is hidden: no new 2 KB block)."""
        text = self._grid_text
        known = [c for c in self.cubes if c[2] >= 2]
        if known:
            self._scratch[:] = text
            text = self._scratch
            half = self.SIZE * self.CELL / 2
            reach = CUBE / 2 + self.CELL
            for c in known:
                for dx in (-2, -1, 0, 1, 2):
                    for dy in (-2, -1, 0, 1, 2):
                        index = self._cell(c[0] + dx * self.CELL, c[1] + dy * self.CELL)
                        if index is None or text[index] != 50:
                            continue
                        cx = -half + (index % self.SIZE + 0.5) * self.CELL
                        cy = half - (index // self.SIZE + 0.5) * self.CELL
                        if abs(cx - c[0]) < reach and abs(cy - c[1]) < reach:
                            text[index] = 48
        return text

    def _grid(self):
        return str(self._grid_bytes(), 'ascii')

    def known_cubes(self):
        """Confirmed cubes, map frame: [(x, y), ...]. Use for planning."""
        return [(c[0], c[1]) for c in self.cubes if c[2] >= 2]

    def packet(self, now_us, state, grid=True):
        p = self.pose
        x, y, h = self.estimate()
        packet = {
            'v': 1, 't': round(now_us / 1000000, 4),
            'pose': [round(x, 4), round(y, 4), round(h, 5)],
            'odom': [round(p.x, 4), round(p.y, 4), round(p.heading, 5)],
            'matched': self.matched,
            'distance': round(p.distance, 3), 'bias': round(p.gyro_bias, 3),
            'state': state, 'scans': self.scans,
            'scan_t': round(self.scan_time, 4) if self.scan_time is not None else None,
            'scan_pose': self.scan_pose,
            'rays': self.rays, 'size': self.SIZE, 'cell': self.CELL,
            'cubes': [[round(c[0], 4), round(c[1], 4), c[2]] for c in self.cubes],
        }
        if grid and self._dirty:
            packet['grid'] = self._grid()
            self._dirty = False
        return packet

    def publish(self, now_us, state, grid=True):
        # Plain JSON output also works on hardware; the browser recognises this prefix.
        # grid=False sends a small pose-only update; the map follows on a later call.
        # The grid goes out in pieces of one line: a 2.3 KB string in one block can
        # fail on a fragmented RP2040 heap even with plenty of memory free.
        packet = self.packet(now_us, state, grid=False)
        if not (grid and self._dirty):
            print('@spark-map ' + json.dumps(packet))
            return
        self._dirty = False
        print('@spark-map ' + json.dumps(packet)[:-1] + ', "grid": "', end='')
        chars = self._grid_bytes()
        for i in range(0, len(chars), 256):
            print(str(chars[i:i + 256], 'ascii'), end='')
        print('"}')
