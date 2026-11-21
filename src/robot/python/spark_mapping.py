"""Portable encoder/gyro odometry and a small ToF discovery map.

No simulator imports, arena geometry, object positions, or pose corrections.
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


class DiscoveryMap:
    # Local drawing window, NOT a known arena boundary. All cells start unknown.
    SIZE = 48
    CELL = 0.05
    MOUNT_X = 0.052
    MOUNT_Z = 0.045

    def __init__(self, odometry):
        self.pose = odometry
        self.cells = array('b', [0] * (self.SIZE * self.SIZE))
        self.rays = []
        self.scan_time = None
        self.scan_pose = None
        self.sequence = None
        self.scans = 0
        self._dirty = True

    def _cell(self, x, y):
        half = self.SIZE * self.CELL / 2
        col = int(math.floor((x + half) / self.CELL))
        row = int(math.floor((half - y) / self.CELL))
        if 0 <= col < self.SIZE and 0 <= row < self.SIZE:
            return row * self.SIZE + col
        return None

    def _mark(self, index, delta):
        if index is not None:
            self.cells[index] = max(-6, min(6, self.cells[index] + delta))

    def observe(self, frame, now_us, capture_pose=None):
        """Fuse raw depths at an estimated capture pose; no motor commands.

        For stationary callers capture_pose can default to the current estimate.
        Moving callers must interpolate odometry at the aligned capture timestamp.
        """
        if frame['seq'] == self.sequence:
            return
        self.sequence = frame['seq']
        size = int(math.sqrt(frame['resolution']))
        if size not in (4, 8):
            return
        px, py, heading = capture_pose if capture_pose is not None else (self.pose.x, self.pose.y, self.pose.heading)
        cs, sn = math.cos(heading), math.sin(heading)
        ox, oy = px + self.MOUNT_X * cs, py + self.MOUNT_X * sn
        rays = []
        for col in range(size):
            depth = None
            # The two rows next to the horizon see low objects. Reject probable
            # ground returns using the ASSUMED level mount, not a geometry label.
            for row in (size // 2 - 1, size // 2):
                zone = row * size + col
                d = frame['distance_mm'][zone] / 1000
                if frame['target_status'][zone] != 5 or frame['nb_target_detected'][zone] < 1:
                    continue
                z_slope = math.tan((1 - (row + 0.5) / size * 2) * math.pi / 8)
                estimated_height = self.MOUNT_Z + d * z_slope
                if 0.02 <= d <= 1.5 and 0.015 <= estimated_height <= 0.13:
                    if depth is None or d < depth:
                        depth = d
            if depth is None:
                # Invalid / above objects / floor: unknown, never free space.
                continue
            # ST reports optical-axis depth, not radial distance. Raw col 0 is RIGHT.
            lateral = depth * math.tan(((col + 0.5) / size * 2 - 1) * math.pi / 8)
            ex, ey = ox + cs * depth - sn * lateral, oy + sn * depth + cs * lateral
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

    def packet(self, now_us, state):
        p = self.pose
        packet = {
            'v': 1, 't': round(now_us / 1000000, 4),
            'pose': [round(p.x, 4), round(p.y, 4), round(p.heading, 5)],
            'distance': round(p.distance, 3), 'bias': round(p.gyro_bias, 3),
            'state': state, 'scans': self.scans,
            'scan_t': round(self.scan_time, 4) if self.scan_time is not None else None,
            'scan_pose': self.scan_pose,
            'rays': self.rays, 'size': self.SIZE, 'cell': self.CELL,
        }
        if self._dirty:
            packet['grid'] = ''.join('2' if v > 0 else '1' if v < 0 else '0' for v in self.cells)
            self._dirty = False
        return packet

    def publish(self, now_us, state):
        # Plain JSON output also works on hardware; the browser recognises this prefix.
        print('@spark-map ' + json.dumps(self.packet(now_us, state)))
