"""Drive with the discovery map: one sensor loop, motion primitives, referee-lift handling.

Portable: copy next to spark_mapping.py and spark_tof.py onto the Zumo. Every
primitive keeps sampling, so pose and map stay current while the robot moves.
Map frame = arena frame by the rules: the robot starts at (0, 0), the centre of
the start square, facing +x. Units: metres, radians, seconds.
"""
import gc
import math
import time
from zumo_2040_robot import robot
from spark_mapping import Odometry, DiscoveryMap


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Lifted(Exception):
    """The referee lifted the robot and put it back on the start square.

    Raised from any primitive. Pose is already reset to the start pose; the map
    keeps what it learned. Catch it in the strategy loop and plan again.
    """


class Pilot:
    SAMPLE_MS = 10
    LINE_EVERY_US = 50000
    LIFT_US = 600000  # all line sensors "nothing below" this long, away from tape = lifted
    SENSOR_X = 0.047  # robot origin to the line sensor bar (sensors at -3.5 .. +3.5 cm)
    # Push & Pull tape squares as (inner, outer) half sizes from the centre: 12 cm and 30 cm + 2.5 cm tape.
    TAPE = ((0.06, 0.085), (0.15, 0.175))
    # Smallest motor command that still turns / drives the Zumo on the arena floor (sim-tuned, check on hardware).
    MIN_TURN = 1400
    TURN_MAX = 3000   # fastest turn command
    TURN_RAMP = 1.0   # rad before the target where the turn starts slowing down
    MIN_DRIVE = 1300

    def __init__(self, tof=True, match=True, publish=True):
        self.motors = robot.Motors()
        self.encoders = robot.Encoders()
        self.imu = robot.IMU()
        self.imu.reset()
        self.imu.enable_default()
        self.line = robot.LineSensors()
        self.leds = robot.RGBLEDs()
        self.sensor = None
        if tof:
            from spark_tof import ToFSerial
            self.sensor = ToFSerial()
        self.odom = Odometry()
        self.map = DiscoveryMap(self.odom, match=match)
        self.state = 'Start'
        # @spark-map telemetry for the simulator's robot view (plain print; harmless on hardware).
        self.publish = publish
        # ESP capture clock -> Zumo clock. Equal in the simulator; measure on hardware.
        self.esp_offset_us = 0
        self.lifts = 0
        self.us = 0
        self._clock = time.ticks_us()
        self._start_us = 0
        self._publish_us = -1000000
        self._grid_us = -1000000
        self._line_us = -1000000
        self._dark_since = None
        self._dark_tape = False
        self._frame = None
        self._stale = False
        self.line_values = None  # last LineSensors.read(): small = bright, 1000+ = black tape / lifted

    # ---- clock and pose ----------------------------------------------------
    def calibrate(self, samples=80):
        """Gyro bias while standing still. Call before wait_for_start()."""
        self.motors.off()
        total = 0.0
        for _ in range(samples):
            self.imu.gyro.read()
            total += self.imu.gyro.last_reading_dps[2]
            time.sleep_ms(5)
        self.odom.gyro_bias = total / samples
        self.sample()

    def begin(self):
        """Call right after wait_for_start(): the round clock starts here."""
        self.sample()
        self._start_us = self.us

    def t(self):
        return (self.us - self._start_us) / 1000000

    def remaining(self, limit=120):
        return limit - self.t()

    def pose(self):
        """(x, y, heading) the robot believes, after scan matching."""
        return self.map.estimate()

    def relocate(self, x, y, heading=None):
        """A landmark says where the robot is: move the belief there, keep the map."""
        odom = self.odom
        ct = self.map.correction[2] if heading is None else wrap(heading - odom.heading)
        c, s = math.cos(ct), math.sin(ct)
        self.map.correction = (x - (c * odom.x - s * odom.y), y - (s * odom.x + c * odom.y), ct)

    # ---- the one sensor loop -------------------------------------------------
    def sample(self, look=True, line=False):
        """Read encoders, gyro and line sensors once; update the pose.

        look=True also parses ToF frames into the map (tens of ms each on the
        RP2040). look=False keeps the control loop fast: ToF bytes wait in the
        UART and are thrown away unparsed when looking starts again.
        line=True reads the line sensors every call (tape edges), else every 50 ms.
        """
        self.imu.gyro.read()
        left, right = self.encoders.get_counts()
        tick = time.ticks_us()
        self.us += time.ticks_diff(tick, self._clock)
        self._clock = tick
        self.odom.update(left, right, self.imu.gyro.last_reading_dps[2], self.us)
        if self.sensor is not None and look:
            if self._stale:
                # Bytes that waited in the UART while we were not looking: old frames.
                self._stale = False
                # Read in small pieces: one 4 KB block may not fit a fragmented heap.
                uart, tail = self.sensor.uart, self.sensor._buffer
                while uart.any():
                    piece = uart.read(256)
                    if not piece:
                        break
                    cut = piece.rfind(b"\n")
                    tail = piece[cut + 1:] if cut >= 0 else (tail + piece if len(tail) < 2048 else b"")
                self.sensor._buffer = tail
            for _ in range(4):
                try:
                    frame = self.sensor.read()
                except ValueError:
                    frame = None  # a line cut by a full UART buffer; the next one is whole
                    self.sensor._buffer = b""
                if frame is None:
                    break
                self._frame = frame
            if self._frame is not None:
                captured = self._frame['t_us'] + self.esp_offset_us
                at = self.odom.at(captured)
                if at is not None:
                    self.map.observe(self._frame, captured, at)
        elif self.sensor is not None:
            self._stale = True
        if line or self.us - self._line_us >= self.LINE_EVERY_US:
            self._line_us = self.us
            self.line_values = self.line.read()
            self._check_lift()
        if self.publish and self.us - self._publish_us >= 100000:
            grid = self.us - self._grid_us >= 500000
            if grid:
                gc.collect()  # the grid packet needs ~3 KB in one piece
            self.map.publish(self.us, self.state, grid)
            self._publish_us = self.us
            if grid:
                self._grid_us = self.us

    def on_tape(self, x, y, margin=0.03):
        """Could the line sensors of a robot at this pose see black tape? Push & Pull squares."""
        m = max(abs(x), abs(y))
        for inner, outer in self.TAPE:
            if inner - margin <= m <= outer + margin:
                return True
        return False

    def _check_lift(self):
        # All five sensors read "nothing below" both in the air and on black tape.
        # Lifted = dark for LIFT_US, then floor again, and the sensor bar was not
        # over known tape when it went dark. Never blocks: the program keeps going.
        if min(self.line_values) >= 1000:
            if self._dark_since is None:
                self._dark_since = self.us
                x, y, h = self.pose()
                c, s = math.cos(h), math.sin(h)
                self._dark_tape = any(self.on_tape(x + c * self.SENSOR_X - s * d, y + s * self.SENSOR_X + c * d)
                                      for d in (-0.035, 0.0, 0.035))
            return
        if self._dark_since is None:
            return
        lasted = self.us - self._dark_since
        self._dark_since = None
        if lasted < self.LIFT_US or self._dark_tape:
            return
        # Put down on the start square: restart the pose there, keep the map.
        self.lifts += 1
        self.motors.off()
        self.map.reset_pose()
        self.encoders.get_counts(reset=True)
        self.odom._last = None
        self.sample(False)  # drop the ToF bytes that piled up in the air
        raise Lifted()

    def wait(self, ms, left=0, right=0, look=True):
        self.motors.set_speeds(left, right)
        end = self.us + ms * 1000
        while self.us < end:
            self.sample(look)
            time.sleep_ms(self.SAMPLE_MS)

    def gather(self, x, y, radius, scans=4, timeout_ms=1200):
        """Stand still and collect raw ToF hit points (map frame) within radius of (x, y).

        Finer than the 5 cm map cells: up close one zone is 1-2 cm wide. Use it to
        measure an object before touching it.
        """
        self.motors.off()
        points = []
        seen = self.map.scans
        end = self.us + timeout_ms * 1000
        while self.us < end and self.map.scans - seen < scans:
            before = self.map.scans
            self.sample()
            if self.map.scans != before:
                for _, _, ex, ey in self.map.rays:
                    if (ex - x) ** 2 + (ey - y) ** 2 <= radius * radius:
                        points.append((ex, ey))
            time.sleep_ms(self.SAMPLE_MS)
        return points

    def stop(self, ms=150, look=False):
        """Brake and keep measuring while the robot settles."""
        self.wait(ms, look=look)

    # ---- motion primitives ---------------------------------------------------
    def turn_to(self, heading, tolerance=0.03, max_speed=None, timeout=4.0, look=False):
        """Turn on the spot to a map-frame heading.

        Fast while far off, creeping just above the motors' dead band at the end;
        an overshoot past the target ends the turn (one creep back at most).
        """
        end = self.us + int(timeout * 1000000)
        max_speed = max_speed or self.TURN_MAX
        last = None
        while self.us < end:
            error = wrap(heading - self.pose()[2])
            if abs(error) < tolerance:
                break
            # Sign flip near the target = overshoot (near ±pi it is only the wrap).
            if last is not None and (error > 0) != (last > 0) and abs(error) < 1.5:
                if abs(error) < 3 * tolerance:
                    break
                max_speed = self.MIN_TURN + 300  # came back from an overshoot: creep
            last = error
            power = min(max_speed, self.MIN_TURN + int((max_speed - self.MIN_TURN) * min(1.0, abs(error) / self.TURN_RAMP)))
            if error < 0:
                power = -power
            self.motors.set_speeds(-power, power)
            self.sample(look)
            time.sleep_ms(self.SAMPLE_MS)
        self.stop()

    def face(self, x, y, **kw):
        px, py, _ = self.pose()
        self.turn_to(math.atan2(y - py, x - px), **kw)

    def spin(self, angle, speed=1300):
        """Turn by angle (rad, + = left) at a steady rate while mapping: a ToF scan."""
        turned = 0.0
        last = self.odom.heading
        power = speed if angle > 0 else -speed
        end = self.us + int(abs(angle) / 0.5 * 1000000) + 4000000
        while abs(turned) < abs(angle) and self.us < end:
            self.motors.set_speeds(-power, power)
            self.sample()
            turned += wrap(self.odom.heading - last)
            last = self.odom.heading
            time.sleep_ms(self.SAMPLE_MS)
        self.stop()

    def drive_to(self, x, y, speed=3500, tolerance=0.02, timeout=8.0, until=None, look=False, line=False,
                 back=0.15):
        """Drive to (x, y), steering continuously. Returns True if reached.

        A target less than back (m) behind the robot is reached in reverse instead
        of turning around twice. until() -> True stops early, checked every loop
        (e.g. a tape edge: pass line=True so line_values are fresh each time).
        """
        px, py, h = self.pose()
        dx, dy = x - px, y - py
        sign = -1 if math.sqrt(dx * dx + dy * dy) < back and abs(wrap(math.atan2(dy, dx) - h)) > 2.0 else 1
        if sign > 0:
            self.face(x, y)
        end = self.us + int(timeout * 1000000)
        reached = False
        while self.us < end:
            px, py, h = self.pose()
            if sign < 0:
                h = wrap(h + math.pi)
            dx, dy = x - px, y - py
            distance = math.sqrt(dx * dx + dy * dy)
            # Passed the point sideways counts as reached; no endless circling.
            if distance < tolerance or dx * math.cos(h) + dy * math.sin(h) < 0 and distance < 0.05:
                reached = True
                break
            if until is not None and until():
                break
            error = wrap(math.atan2(dy, dx) - h)
            if abs(error) > 0.7:
                if sign < 0:
                    break  # reversing only for short, straight corrections
                self.face(x, y)
                continue
            power = min(speed, max(self.MIN_DRIVE, int(distance * 9000)))
            steer = int(error * 2500)
            self.motors.set_speeds(sign * power - steer, sign * power + steer)
            self.sample(look, line)
            time.sleep_ms(self.SAMPLE_MS)
        self.stop()
        return reached

    def drive(self, distance, speed=2000, timeout=4.0, look=False):
        """Straight by distance (m), backwards if negative, holding the heading."""
        x0, y0, heading = self.pose()
        sign = 1 if distance > 0 else -1
        end = self.us + int(timeout * 1000000)
        while self.us < end:
            px, py, h = self.pose()
            if math.sqrt((px - x0) ** 2 + (py - y0) ** 2) >= abs(distance):
                break
            steer = int(wrap(heading - h) * 2500)
            self.motors.set_speeds(sign * speed - steer, sign * speed + steer)
            self.sample(look)
            time.sleep_ms(self.SAMPLE_MS)
        self.stop()

    def clear(self, x0, y0, x1, y1, radius=0.08, keep_out=()):
        """No map obstacle and no keep-out circle (x, y, r) within radius of the segment."""
        for kx, ky, kr in keep_out:
            dx, dy = x1 - x0, y1 - y0
            span = dx * dx + dy * dy
            f = 0.0 if span == 0 else max(0.0, min(1.0, ((kx - x0) * dx + (ky - y0) * dy) / span))
            if (x0 + f * dx - kx) ** 2 + (y0 + f * dy - ky) ** 2 < (kr + radius) ** 2:
                return False
        return self.map.path_clear(x0, y0, x1, y1, radius)

    def route(self, x, y, radius=0.08, keep_out=(), allowed=None):
        """Waypoints to (x, y): [] = straight, [(wx, wy)] = one detour, None = no route found.

        allowed(wx, wy) -> False rejects detour points (e.g. too close to a wall).
        Unknown space counts as free; the map only blocks what the ToF has seen.
        """
        px, py, _ = self.pose()
        if self.clear(px, py, x, y, radius, keep_out):
            return []
        dx, dy = x - px, y - py
        length = math.sqrt(dx * dx + dy * dy) or 1
        nx, ny = -dy / length, dx / length
        # Detour candidates: beside the straight line, and on rings around the
        # target (to come at a spot behind an obstacle from the side).
        candidates = []
        for along in (0.5, 0.3, 0.7):
            for side in (0.15, -0.15, 0.25, -0.25, 0.35, -0.35):
                candidates.append((px + dx * along + nx * side, py + dy * along + ny * side))
        for r in (0.22, 0.32):
            for k in range(12):
                a = k * math.pi / 6
                candidates.append((x + r * math.cos(a), y + r * math.sin(a)))
        best = None
        for wx, wy in candidates:
            if allowed is not None and not allowed(wx, wy):
                continue
            cost = math.sqrt((wx - px) ** 2 + (wy - py) ** 2) + math.sqrt((x - wx) ** 2 + (y - wy) ** 2)
            if best is not None and cost >= best[0]:
                continue
            if self.clear(px, py, wx, wy, radius, keep_out) and self.clear(wx, wy, x, y, radius, keep_out):
                best = (cost, wx, wy)
        return None if best is None else [(best[1], best[2])]

    def finish(self, text='Fertig'):
        """Green blink ends the round early (rules: on equal points the shorter time wins)."""
        self.motors.off()
        self.state = text
        self.map.publish(self.us, text)
        for _ in range(4):
            for i in range(6):
                self.leds.set(i, [0, 60, 0])
            self.leds.show()
            time.sleep_ms(250)
            self.leds.off()
            time.sleep_ms(250)
