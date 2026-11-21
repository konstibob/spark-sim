"""Portable encoder/gyro odometry, a small ToF discovery map and scan matching.

No simulator imports, arena geometry or object positions: the pose correction
comes only from matching new ToF hits against the map built so far (SLAM).
Copy alongside spark_tof.py to the robot. Units: metres, radians, microseconds.
The start pose is an assumption: (0, 0, 0), +x in the starting direction.
"""
import math
import json
from array import array


class Odometry:
    def __init__(self, counts_per_m=8467, gyro_bias=0.0):
        self.x = self.y = self.heading = self.distance = 0.0
        self.counts_per_m = counts_per_m
        self.gyro_bias = gyro_bias
        self._last = None
        self._history = []

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
        self._history.append((now_us, self.x, self.y, self.heading))
        if len(self._history) > 200:
            self._history.pop(0)

    def at(self, captured_us):
        """Estimated pose at capture, interpolated from sensor odometry ONLY.

        Return None outside the history; never assign an old image today's pose.
        The caller must align ESP and Zumo clocks before calling this method.
        """
        h = self._history
        if not h or captured_us < h[0][0] or captured_us > h[-1][0]:
            return None
        for i in range(len(h) - 1, -1, -1):
            if h[i][0] <= captured_us:
                a = h[i]
                if a[0] == captured_us:
                    return a[1:]
                b = h[i + 1]
                f = (captured_us - a[0]) / (b[0] - a[0])
                turn = (b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi
                return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * turn)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


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
        # Rigid transform odometry frame -> map frame, found by scan matching.
        self.correction = (0.0, 0.0, 0.0)
        self._last_odom = None
        self._dirty = True

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
        c, s = math.cos(h), math.sin(h)
        total = 0.0
        for lx, ly in hits:
            total += self._evidence(x + c * lx - s * ly, y + s * lx + c * ly)
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
        # Hits in the robot frame: x forward from the robot origin, y left.
        local = []
        for col in range(size):
            depth = None
            # Nearest return per column that hits something robot-high. Reject
            # probable floor (too low) and returns over the walls (too high)
            # using the ASSUMED level mount, not a geometry label.
            for row in range(size):
                zone = row * size + col
                d = frame['distance_mm'][zone] / 1000
                if frame['target_status'][zone] != 5 or frame['nb_target_detected'][zone] < 1:
                    continue
                z_slope = math.tan((1 - (row + 0.5) / size * 2) * math.pi / 8)
                estimated_height = self.MOUNT_Z + d * z_slope
                if 0.02 <= d <= 1.5 and self.MIN_HEIGHT <= estimated_height <= self.MAX_HEIGHT:
                    if depth is None or d < depth:
                        depth = d
            if depth is None:
                # Invalid / above objects / floor: unknown, never free space.
                continue
            # ST reports optical-axis depth, not radial distance. Raw col 0 is RIGHT.
            lateral = depth * math.tan(((col + 0.5) / size * 2 - 1) * math.pi / 8)
            local.append((self.MOUNT_X + depth, lateral))
        pose = self.corrected(odom)
        # How far odometry may be off since the previous scan: grows with the
        # travelled distance and the turned angle (slip, gyro scale error).
        last = self._last_odom or odom
        moved = math.sqrt((odom[0] - last[0]) ** 2 + (odom[1] - last[1]) ** 2)
        turned = abs(_wrap(odom[2] - last[2]))
        self._last_odom = odom
        sigma = (0.004 + 0.1 * moved + 0.01 * turned, 0.01 + 0.3 * turned + 0.5 * moved)
        if self.match and local and self.confident >= self.MATCH_MIN_CELLS:
            matched = self._match(local, pose, sigma)
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
        # Carve free evidence first, then all endpoints: one beam must not erase
        # another beam's hit in the same frame. Endpoint cells are excluded.
        hits = []
        for ox, oy, ex, ey in rays:
            hit = self._cell(ex, ey)
            hits.append(hit)
            steps = max(1, int(math.ceil(math.sqrt((ex - ox) ** 2 + (ey - oy) ** 2) / (self.CELL / 2))))
            previous = None
            for i in range(steps):
                f = i / steps
                index = self._cell(ox + (ex - ox) * f, oy + (ey - oy) * f)
                if index is not None and index != hit and index != previous:
                    self._mark(index, -1)
                previous = index
        for hit in set(hits):
            self._mark(hit, 3)
        self.rays = rays
        self.scan_time = now_us / 1000000
        self.scan_pose = [round(px, 4), round(py, 4), round(heading, 5)]
        self.scans += 1
        self._dirty = True

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
        }
        if grid and self._dirty:
            packet['grid'] = ''.join('2' if v > 0 else '1' if v < 0 else '0' for v in self.cells)
            self._dirty = False
        return packet

    def publish(self, now_us, state, grid=True):
        # Plain JSON output also works on hardware; the browser recognises this prefix.
        # grid=False sends a small pose-only update; the map follows on a later call.
        print('@spark-map ' + json.dumps(self.packet(now_us, state, grid)))
