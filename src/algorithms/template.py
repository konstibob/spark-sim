# Name: Vorlage · eigener Algorithmus
# Strategy: Starting point for a new Challenge 2 strategy: scan, read the discovery map as data, drive.
#   This one only drives up to the nearest thing it saw and stops in front of it.
# Strengths / weaknesses: Shows the whole API in one screen: Pilot (sensor loop, turns, drives, routes,
#   lift handling) and the map queries hits(), obstacles(), frontiers(), path_clear(), forget().
# Sensors: tof
# Arena: push-pull
#
# On the robot: copy spark_pilot.py, spark_mapping.py and spark_tof.py next to this file (Download menu).
from zumo_2040_robot import robot
from spark_pilot import Pilot, Lifted
import math
import time

# Map frame = arena frame: start square centre (0, 0), start direction +x, metres, radians.
pilot = Pilot(match=False)
pilot.calibrate()  # gyro bias while standing still, before the start signal
display = robot.Display()
button_a = robot.ButtonA()

ARENA_HALF = 0.80  # assumption: 160 cm arena


def show(*rows):
    display.fill(0)
    for i, r in enumerate(rows):
        display.text(r, 0, i * 12)
    display.show()


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


def dist(dx, dy):
    return math.sqrt(dx * dx + dy * dy)  # MicroPython has no math.hypot


def main():
    wait_for_start()
    pilot.begin()
    try:
        pilot.state = 'Scan'
        pilot.spin(2 * math.pi - 0.75, 1800)  # 315° + 45° field of view = all around
        x, y, _ = pilot.pose()
        # The map as data: obstacle cells on open floor (walls are at ±0.8 m).
        things = [(hx, hy) for hx, hy in pilot.map.hits()
                  if abs(hx) < ARENA_HALF - 0.07 and abs(hy) < ARENA_HALF - 0.07]
        print('groups', pilot.map.obstacles(), 'frontiers', len(pilot.map.frontiers()))
        if things:
            tx, ty = min(things, key=lambda p: dist(p[0] - x, p[1] - y))
            # Stop 15 cm short, on the line from here.
            d = dist(tx - x, ty - y)
            gx, gy = tx - (tx - x) / d * 0.15, ty - (ty - y) / d * 0.15
            pilot.state = 'Hinfahren'
            for wx, wy in pilot.route(gx, gy) or []:
                pilot.drive_to(wx, wy)
            pilot.drive_to(gx, gy)
            pilot.face(tx, ty)
            print('stopped in front of', round(tx, 2), round(ty, 2))
    except Lifted:
        show("Lifted", "back at start")
    pilot.finish()


try:
    main()
finally:
    pilot.motors.off()
