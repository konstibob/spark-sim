# Name: Challenge 2 · calibration windows
# Strategy: The crew's three 10 s calibration windows (start square, wall, cube) before the round: learns floor and
#   tape per line sensor and the cube threshold of the front sensor (highest wall value + 1, fine levels), then
#   starts with wait_for_start() and looks around once with the learned threshold.
# Strengths / weaknesses: A starting point for challenge2.py, not a strategy. In the simulator the windows only
#   run when "Kalibrierung durchspielen" (Arena-Karte) or --calibrate is on; otherwise it uses the stock values.
# Arena: push-pull
from zumo_2040_robot import robot
import time
import json

# --- Hardware ---
motors = robot.Motors()
display = robot.Display()
button_a = robot.ButtonA()
encoders = robot.Encoders()
line = robot.LineSensors()
prox = robot.ProximitySensors()
leds = robot.RGBLEDs()
imu = robot.IMU()
imu.reset()
imu.enable_default()


def show(*rows):
    """Print up to a few lines on the OLED display."""
    display.fill(0)
    for i, r in enumerate(rows):
        display.text(r, 0, i * 12)
    display.show()


# --- Start: only the crew starts robots, all at the same moment ---
try:
    from spark_start import wait_for_start
except ImportError:
    def wait_for_start():
        """Practice start: press A, the robot moves 5 s later."""
        show("SPARK Zumo", "Challenge 2", "A = start")
        while not button_a.check():
            time.sleep_ms(10)
        for s in (5, 4, 3, 2, 1):
            show("Challenge 2", "", "GO in %d" % s)
            time.sleep(1)

# Only the simulator's spark_start has this (True when its calibration is switched off, to keep test runs short).
# The crew's spark_start has no such attribute: on the robot the windows always run.
try:
    import spark_start
    SIM_SKIP_CALIBRATION = getattr(spark_start, "SIM_SKIP_CALIBRATION", False)
except ImportError:
    SIM_SKIP_CALIBRATION = False

# Tell the crew these distances (scoop to wall / cube at the start of windows 2 and 3, 1 ... 10 cm)
WALL_CM = 5
CUBE_CM = 5

# The crew's fine brightness levels: counts 0 ... 30, one reading takes about 34 ms
FINE = [round(10 * 756.3 ** (i / 29)) for i in range(30)]
prox.brightness_levels = FINE

# Values of the crew's stock Zumo (calibration.md, 21.11.2026): used when a window gives nothing useful
cal = {
    "floor": [57, 36, 30, 37, 43],
    "tape": [961, 726, 565, 954, 1024],
    "threshold": [509, 381, 297, 495, 533],
    "wall_max": 15,
    "cube_threshold": 16,
    "cube_reach_cm": 24,
    "measured": False,
}

COUNTS_PER_CM = 84.67  # Zumo 2040: encoder counts per cm of track
WINDOW_MS = 10000
STOP_BEFORE_MS = 400  # stand still before the window ends


def window(n, name):
    """Wait for the crew's press of A, return when the window started."""
    show("CALIBRATION %d/3" % n, name, "", "crew: A")
    while not button_a.check():
        time.sleep_ms(10)
    show("CALIBRATION %d/3" % n, name, "", "running ...")
    return time.ticks_ms()


def time_left(t0):
    return WINDOW_MS - time.ticks_diff(time.ticks_ms(), t0)


class Crawl:
    """Drive straight at a set speed (cm/s, negative = backwards) with the encoders: a fixed low motor power is
    not a fixed speed, so add power when a track falls behind the distance it should have driven by now."""

    def __init__(self, cm_per_s):
        self.v = cm_per_s
        self.l0, self.r0 = encoders.get_counts()
        self.t0 = time.ticks_ms()
        self.boost = [0.0, 0.0]

    def driven_cm(self):
        l, r = encoders.get_counts()
        return ((l - self.l0) + (r - self.r0)) / 2 / COUNTS_PER_CM

    def update(self):
        t = time.ticks_diff(time.ticks_ms(), self.t0) / 1000
        want = self.v * t * COUNTS_PER_CM
        l, r = encoders.get_counts()
        sign = 1 if self.v >= 0 else -1
        power = []
        for k, got in enumerate((l - self.l0, r - self.r0)):
            behind = (want - got) * sign  # counts behind schedule
            self.boost[k] = max(0.0, min(3000.0, self.boost[k] + 0.6 * behind))
            power.append(sign * int(max(0, min(4000, 300 + 12 * behind + self.boost[k]))))
        motors.set_speeds(power[0], power[1])


def calibrate_line():
    """Window 1, start square: note what the sensors see at the start, then drive 16 cm across both tapes and
    back. Every sensor sees floor and tape; floor = lowest, tape = highest reading."""
    t0 = window(1, "start square")
    start = line.read()
    time.sleep_ms(1000)  # the crew's hand goes away
    low = [1024] * 5
    high = [0] * 5
    for v, until in ((6, 16), (-6, 0)):
        crawl = Crawl(v)
        while time_left(t0) > STOP_BEFORE_MS and (crawl.driven_cm() < until if v > 0 else crawl.driven_cm() > -16 + until):
            crawl.update()
            for i, x in enumerate(line.read()):
                low[i] = min(low[i], x)
                high[i] = max(high[i], x)
        motors.off()
        time.sleep_ms(150)
    motors.off()
    if all(h - l > 200 for l, h in zip(low, high)):
        cal["floor"] = low
        cal["tape"] = high
        cal["threshold"] = [(l + h) // 2 for l, h in zip(low, high)]
    print("CAL line start", list(start))
    print("CAL line floor", cal["floor"], "tape", cal["tape"], "threshold", cal["threshold"])
    while time_left(t0) > 0:
        time.sleep_ms(20)


def back_away(n, name, start_cm):
    """Windows 2 and 3: back away at 2.5 cm/s for the whole window, reading the front sensor (fine levels).
    Returns {cm from the scoop: (FL, FR)}, the highest reading per centimetre."""
    t0 = window(n, name)
    curve = {}
    crawl = Crawl(-2.5)
    while time_left(t0) > STOP_BEFORE_MS:
        crawl.update()
        prox.read()
        fl = prox.front_counts_with_left_leds()
        fr = prox.front_counts_with_right_leds()
        cm = int(start_cm - crawl.driven_cm() + 0.5)
        old = curve.get(cm, (0, 0))
        curve[cm] = (max(old[0], fl), max(old[1], fr))
    motors.off()
    while time_left(t0) > 0:
        time.sleep_ms(20)
    return curve


def calibrate():
    if SIM_SKIP_CALIBRATION:
        print("CAL skipped (simulator): stock values")
        return
    calibrate_line()
    wall = back_away(2, "wall %d cm" % WALL_CM, WALL_CM)
    cube = back_away(3, "cube %d cm" % CUBE_CM, CUBE_CM)
    print("CAL wall", sorted(wall.items()))
    print("CAL cube", sorted(cube.items()))
    if wall:
        # the black wall is darkest right at it and brightest a few cm away: take the highest value of the drive
        cal["wall_max"] = max(max(v) for v in wall.values())
        cal["cube_threshold"] = cal["wall_max"] + 1
    seen = [cm for cm, v in cube.items() if min(v) >= cal["cube_threshold"]]
    cal["cube_reach_cm"] = max(seen) if seen else 0
    cal["measured"] = True
    print("CAL wall max", cal["wall_max"], "cube threshold", cal["cube_threshold"], "cube reaches it up to", cal["cube_reach_cm"], "cm")
    try:
        with open("c2_cal.json", "w") as f:
            json.dump(cal, f)
    except OSError:
        pass


def on_tape():
    """Which line sensors see tape now (learned thresholds)"""
    return [x > t for x, t in zip(line.read(), cal["threshold"])]


def blink_green():
    """Two green blinks end the round early"""
    for _ in range(2):
        for i in range(6):
            leds.set(i, [0, 255, 0])
        leds.show()
        time.sleep_ms(400)
        leds.off()
        time.sleep_ms(400)


def main():
    calibrate()
    show("Challenge 2", "waiting for start", "cube >= %d" % cal["cube_threshold"])
    wait_for_start()
    show("Challenge 2", "Running...")

    # ─── YOUR CODE GOES HERE ───
    # Example: turn once on the spot and report where the front sensor sees a cube (learned threshold).
    print("START tape under the sensors:", on_tape())
    heading = 0.0
    last = time.ticks_ms()
    seen = []
    motors.set_speeds(-1400, 1400)
    while heading < 360:
        prox.read()
        imu.gyro.read()
        now = time.ticks_ms()
        heading += imu.gyro.last_reading_dps[2] * time.ticks_diff(now, last) / 1000
        last = now
        if min(prox.front_counts_with_left_leds(), prox.front_counts_with_right_leds()) >= cal["cube_threshold"]:
            seen.append(int(heading))
    motors.off()
    print("CUBE seen at headings", seen)
    blink_green()
    # ───────────────────────────


# The robot's menu starts this file with import, so the program must run
# at the top level. No "if __name__ == '__main__'": it would never run.
try:
    main()
finally:
    motors.off()  # safety: always switch the motors off
