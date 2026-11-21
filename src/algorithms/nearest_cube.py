# Name: Nächster Würfel → Mitte
# Strategy: Scan 360° with the ToF, take the nearest cube from the discovery map, drive behind it on the line
#   through the start square and push it in. The first cube goes to the centre (3 points), later ones stop
#   completely inside the 30 cm square (1 point). Cubes near a wall are first pushed sideways into open floor.
# Strengths / weaknesses: Uses only the robot's own map (hits(), path_clear(), route()) plus the published
#   arena size; the tape edges correct the last push. No pull: cubes touching a wall or in a corner are
#   skipped. Only the first cube aims for 3 points.
# Sensors: tof
# Arena: push-pull
#
# On the robot: copy spark_pilot.py, spark_mapping.py and spark_tof.py next to this file (Download menu).
from zumo_2040_robot import robot
from spark_pilot import Pilot, Lifted
import math
import time

# Scan matching off: in the simulator encoder + gyro odometry alone stays within 1-2 cm, matching drifted
# up to 8 cm while pushing. Compare both on the real floor before switching it on.
pilot = Pilot(match=False)
pilot.calibrate()  # gyro bias while the robot still stands; before the start signal
display = robot.Display()
button_a = robot.ButtonA()

# --- Assumptions (published geometry; measure on the real arena) ---
ARENA_HALF = 0.80      # 160 cm arena, walls on the inside of this line
ROBOT_MARGIN = 0.09    # robot origin to wall, keeps the blade and the back clear
HOME_HALF = 0.175      # 30 cm square + 2.5 cm tape
CUBE_HALF = 0.03
FRONT = 0.059          # robot origin to the blade edge
SENSOR = 0.047         # robot origin to the line sensors
SMALL_EDGE = 0.085     # outer tape edge of the 12 cm square, from the centre
HOME_EDGE = 0.175      # outer tape edge of the 30 cm square
GAP = 0.08             # blade to cube before each push
BEHIND = CUBE_HALF + FRONT + GAP   # robot origin behind the cube centre before a push
SIDE_GOAL = 0.10       # later cubes: centre this far from the middle (completely inside the 1-point square)
SEGMENT = 0.18         # long pushes in steps of this much cube travel, looking again between them
FINAL = 0.32           # closer than this to the centre: the last push, guided by the tape edges
LAST_SECONDS = 10      # no new push this close to 120 s

tries = []   # (x, y) of failed attempts
placed = []  # (x, y) where we left every cube we pushed in


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


def inside(x, y, margin):
    return abs(x) <= ARENA_HALF - margin and abs(y) <= ARENA_HALF - margin


def in_home(x, y):
    """Whole cube inside the 30 cm square (+ tape), 1 cm to spare: only then it counts."""
    return abs(x) < HOME_HALF - CUBE_HALF - 0.01 and abs(y) < HOME_HALF - CUBE_HALF - 0.01


def measure(cx, cy):
    """Cube centre from the raw ToF hits on its near face, seen from behind it."""
    points = pilot.gather(cx, cy, 0.07)
    if len(points) < 2:
        return None
    fx = sum(p[0] for p in points) / len(points)
    fy = sum(p[1] for p in points) / len(points)
    x, y, _ = pilot.pose()
    d = dist(fx - x, fy - y) or 1
    # The hits lie on the face towards us; the centre is half a cube further.
    return fx + (fx - x) / d * CUBE_HALF, fy + (fy - y) / d * CUBE_HALF


def cubes():
    """Cube candidates: obstacle cells on open floor, away from the walls.

    Cells, not obstacle groups: cubes standing close together merge into one
    group, but each of their cells is still worth a closer look (measure()).
    """
    out = []
    for x, y in pilot.map.hits():
        if not inside(x, y, 0.07):
            continue
        if sum(1 for tx, ty in tries if dist(tx - x, ty - y) < 0.08) >= 2:
            continue
        out.append((x, y))
    return out


def nearest(cands, x, y):
    return min(cands, key=lambda c: dist(c[0] - x, c[1] - y))


def cube_near(x, y, radius=0.12):
    near = [c for c in cubes() if dist(c[0] - x, c[1] - y) < radius]
    return nearest(near, x, y) if near else None


def plan(cx, cy):
    """Start point behind the cube, push direction and kind: 'small', 'home' or 'side'."""
    home = math.atan2(-cy, -cx)
    for offset in (0, 0.5, -0.5, 1.0, -1.0, 1.57, -1.57):
        a = home + offset
        ux, uy = math.cos(a), math.sin(a)
        bx, by = cx - ux * BEHIND, cy - uy * BEHIND
        if not inside(bx, by, ROBOT_MARGIN):
            continue
        if offset == 0:
            return (bx, by), (ux, uy), 'home' if placed else 'small'
        # Away from the wall first; the next round pushes it home.
        if inside(cx + ux * SEGMENT, cy + uy * SEGMENT, CUBE_HALF + 0.03):
            return (bx, by), (ux, uy), 'side'
    return None


def keep_out():
    """Cubes we already scored: never drive through them on the way to the next one."""
    return [(x, y, CUBE_HALF * 1.5 + 0.02) for x, y in placed]


def go_to(x, y, **kw):
    """Drive there around seen obstacles and scored cubes; False if blocked."""
    route = pilot.route(x, y, keep_out=keep_out(), allowed=lambda wx, wy: inside(wx, wy, ROBOT_MARGIN))
    if route is None:
        return False
    for wx, wy in route:
        pilot.drive_to(wx, wy, timeout=5)
    return pilot.drive_to(x, y, **kw)


def push_home(ux, uy, small):
    """Push along u through the centre. The tape edges under the line sensors say
    where the cube is, which encoders cannot while the tracks slip on the push."""
    k = 1 / max(abs(ux), abs(uy))  # a ray through the centre meets a square's edge at edge * k
    seen = {'dark': True, 'edge': None}  # start "dark": standing on tape is no new edge

    def tape():
        v = pilot.line_values
        dark = max(v[1], v[2], v[3]) >= 1000
        rising = dark and not seen['dark']
        seen['dark'] = dark
        if not rising:
            return False
        x, y, _ = pilot.pose()
        along = -(x * ux + y * uy)  # robot origin before the centre, along the push
        r = along - SENSOR
        edge = SMALL_EDGE if abs(r - SMALL_EDGE * k) < abs(r - HOME_EDGE * k) else HOME_EDGE
        # The edge is a landmark: correct the belief along the push, keep it across.
        fix = edge * k + SENSOR - along
        pilot.relocate(x - ux * fix, y - uy * fix)
        seen['edge'] = edge
        return edge == SMALL_EDGE or not small

    stop = CUBE_HALF + FRONT + (0 if small else SIDE_GOAL * k)  # robot origin before the centre at the goal
    pilot.drive_to(-ux * stop, -uy * stop, speed=2400, timeout=8, until=tape, line=True)
    edge = seen['edge']
    if edge == SMALL_EDGE or edge == HOME_EDGE and not small:
        # Sensors on the tape edge: the last few cm by encoder put the cube on its goal.
        rest = edge * k + SENSOR - stop
        if rest > 0.005:
            pilot.drive(rest, speed=2000)
    return edge


def deliver(cx, cy):
    """Bring one cube home in steps: line up, push a bit, back off, look, repeat.

    Returns 'done', 'gone' (nothing there up close: a stale or noisy map cell)
    or 'failed'.
    """
    for step in range(6):
        if pilot.remaining() < LAST_SECONDS:
            return 'failed'
        route = plan(cx, cy)
        if route is None:
            return 'failed'
        (bx, by), (ux, uy), kind = route
        pilot.state = 'Anfahrt'
        x, y, _ = pilot.pose()
        if dist(bx - x, by - y) > 0.04 and not go_to(bx, by, timeout=6):
            return 'failed'
        # Measure from close by: from afar the map only had a coarse cell.
        pilot.face(cx, cy)
        found = measure(cx, cy)
        if found is None:
            pilot.map.forget(cx, cy, 0.05)
            return 'gone' if step == 0 else 'failed'
        cx, cy = found
        (bx, by), (ux, uy), kind = plan(cx, cy) or ((bx, by), (ux, uy), kind)
        x, y, _ = pilot.pose()
        if dist(bx - x, by - y) > 0.03:
            pilot.drive_to(bx, by, timeout=3, speed=2000)
        pilot.face(cx, cy, tolerance=0.025)
        pilot.state = 'Schieben'
        travel = dist(cx, cy) - (0 if kind == 'small' else SIDE_GOAL)
        if kind != 'side' and travel <= FINAL:
            edge = push_home(ux, uy, kind == 'small')
            pilot.map.forget(cx, cy, 0.05)
            pilot.drive(-0.12)
            if edge is not None:
                r = 0 if kind == 'small' else SIDE_GOAL
                placed.append((-ux * r, -uy * r))
                return 'done'
            return 'failed'
        # A straight piece, then look again before the cube can slide off the blade.
        move = SEGMENT if kind == 'side' else min(SEGMENT, travel - FINAL + 0.05)
        pilot.drive(GAP + move, speed=2400)
        pilot.map.forget(cx, cy, 0.05)
        cx, cy = cx + ux * move, cy + uy * move
        pilot.drive(-GAP)
        if kind == 'side':
            return 'done'
    return 'failed'


def main():
    wait_for_start()
    pilot.begin()
    show("Challenge 2", "Running...")
    turn = 1
    scan = True
    empty = 0
    while pilot.remaining() > LAST_SECONDS:
        try:
            if scan:
                pilot.state = 'Scan'
                # 315° covers the full circle with the 45° field of view. Alternate the
                # direction: the gyro's scale error then cancels instead of adding up.
                pilot.spin(turn * (2 * math.pi - 0.75), 1800)
                turn = -turn
                scan = False
            x, y, _ = pilot.pose()
            todo = [c for c in cubes() if not in_home(*c)]
            if not todo:
                empty += 1
                if empty >= 3:
                    break
                # Nothing left in view: look again from another spot.
                go_to(0.3 * (1 if empty % 2 else -1), 0.3 * (1 if empty < 2 else -1), timeout=4)
                scan = True
                continue
            empty = 0
            cx, cy = nearest(todo, x, y)
            result = deliver(cx, cy)
            if result != 'done':
                tries.append((cx, cy))
            # A cube moved or a push failed: the map around it is out of date.
            scan = result != 'gone'
        except Lifted:
            show("Challenge 2", "Lifted: restart")
            scan = True
    pilot.finish()


try:
    main()
finally:
    pilot.motors.off()
