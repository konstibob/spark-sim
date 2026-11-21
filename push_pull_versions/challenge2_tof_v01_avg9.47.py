# Name: Challenge 2 · ToF map & push
# Strategy: 360° scan with the VL53L5CX maps the cubes; the walls keep the pose honest.
#   Four cubes go into a pinwheel of slots in the 17 cm square, the fifth into the 30 cm square.
# Arena: push-pull
# Tests: Enable VL53L5CX, then Run. Headless: npm run runs -- src/examples/challenge2_tof.py --tof
# ─────────────────────────────────────────────────────────────
#  SPARK · Zumo 2040 · Challenge 2 · Push & Pull
#
#  Everything is the robot's own view: its map comes only from its ToF,
#  gyro and encoders. What it knows beforehand is the rules: it starts in
#  the middle of the start square facing the start direction, the arena is
#  160 x 160 cm with walls, the home squares are 12 and 30 cm.
#  Robot frame = world frame at the start: +x ahead, +y left, metres.
#
#  1. Spin in steps, map cubes (ToF returns clearly in front of the
#     expected floor/wall) and fit the walls to correct the pose.
#  2. Per cube: push it along the axes onto a slot's axis line, re-measure
#     it from behind before every push, fix the pose on a side wall,
#     correct it sideways, push it straight into its slot.
#  3. Blink green when nothing is left to do.
# ─────────────────────────────────────────────────────────────
from zumo_2040_robot import robot
import time
import math
import array
import gc

try:
    from spark_tof import ToFSerial
except ImportError:
    ToFSerial = None

DEBUG = False             # True: progress log on the console (costs robot time)

motors = robot.Motors()
display = robot.Display()
button_a = robot.ButtonA()
leds = robot.RGBLEDs()
encoders = robot.Encoders()
imu = robot.IMU()
imu.reset()
imu.enable_default()
imu.gyro.set_full_scale(1000)


def show(*rows):
    display.fill(0)
    for i, r in enumerate(rows):
        display.text(str(r), 0, i * 12)
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


def log(*a):
    if DEBUG:
        print("%6.2f" % (elapsed() / 1000.0), *a)


# --- Constants (metres) ----------------------------------------
WALL = 0.80
CONTACT = 0.0589 + 0.03     # robot origin -> centre of a cube on the blade
HALF = 0.03                 # half a cube
TOF_X = 0.052               # ToF position on the robot
TOF_Z = 0.09
COUNTS_PER_M = 8467
ROT_X = 0.0063              # the robot spins about this point ahead of its origin
TOTAL_MS = 120000
TURN_MIN = 1450             # spinning needs ~1300 before the tracks move
TURN_MAX = 4200
DRIVE_V = 5000
PUSH_V = 3800
PUSH_FINAL_V = 2600
PUSH_END_V = 1500
CONTACT_V = 1300            # touching a cube: slow, a light one flies off
LIMIT = 0.70                # robot origin stays inside +-LIMIT to turn
LIMIT_PUSH = 0.735          # a push may start this close to a wall
CLR = 0.125                 # robot origin <-> cube centre while travelling
STAND = 0.105               # ... while standing behind a cube to push it
LEAD = 0.05                 # run-up behind a cube
STAGE = 0.22                # the last push starts this far out on the slot axis
OBSERVE = 0.23              # robot origin -> cube centre when measuring it
SWEEP_CUBE = 0.064          # a pushed cube passes other cubes this far off
SWEEP_ROBOT = 0.10          # the pushing robot passes other cubes this far off
LAT_TOL = 0.008             # sideways error on the slot axis that gets corrected
MAX_LEG = 0.40              # longest push without re-measuring
WALL_COS = 0.7              # wall points only within 45 deg of square on
WALL_K = 0.3                # wall fix gains (position, heading)
WALL_KH = 0.1

# Pinwheel in the 17 cm square: each cube is pushed in along an axis and stops
# P out (odometry stop, 1 cm to spare), sitting Q to the side (2 cm to spare).
# P - Q keeps the 10 cm blade clear of the neighbouring cube.
P = 0.045
Q = 0.033
SLOTS = [(P, Q, -1.0, 0.0), (-Q, P, 0.0, -1.0), (-P, -Q, 1.0, 0.0), (Q, -P, 0.0, 1.0)]
RINGS = [(0.135, -0.06, -1.0, 0.0), (0.06, 0.135, 0.0, -1.0), (-0.135, 0.06, 1.0, 0.0), (-0.06, -0.135, 0.0, 1.0)]

# 8x8 zone slopes (zone 0 = top right) and where a zone's low rays hit the floor
_H = math.radians(22.5)
SY = [math.tan(((c + 0.5) / 4.0 - 1.0) * _H) for c in range(8)]
SZ = [math.tan((1.0 - (r + 0.5) / 4.0) * _H) for r in range(8)]
FLOOR_D = []
for _r in range(8):
    _s = math.tan((1.0 - (_r + 0.8) / 4.0) * _H)
    FLOOR_D.append(TOF_Z / -_s if _s < -0.01 else 99.0)

# --- Odometry --------------------------------------------------
x = 0.0
y = 0.0
th = 0.0
gyro_bias = 0.0
last_l = 0
last_r = 0
last_t = 0
last_w = 0.0
t0 = 0
HIST = 48
hist_t = [0] * HIST
hist_x = [0.0] * HIST
hist_y = [0.0] * HIST
hist_h = [0.0] * HIST
hist_w = [0.0] * HIST
hist_i = 0


def elapsed():
    return time.ticks_diff(time.ticks_ms(), t0)


def out_of_time():
    return elapsed() > TOTAL_MS - 1200


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def update():
    global x, y, th, last_l, last_r, last_t, last_w, hist_i
    imu.gyro.read()
    now = time.ticks_us()
    dt = time.ticks_diff(now, last_t) * 1e-6
    last_t = now
    if dt <= 0 or dt > 0.2:
        dt = 0.005
    l, r = encoders.get_counts()
    d = ((l - last_l) + (r - last_r)) * 0.5 / COUNTS_PER_M
    last_l, last_r = l, r
    w = math.radians(imu.gyro.last_reading_dps[2] - gyro_bias)
    dth = (w + last_w) * 0.5 * dt
    last_w = w
    mid = th + dth * 0.5
    x += d * math.cos(mid) - ROT_X * (math.cos(th + dth) - math.cos(th))
    y += d * math.sin(mid) - ROT_X * (math.sin(th + dth) - math.sin(th))
    th = wrap(th + dth)
    hist_i = (hist_i + 1) % HIST
    hist_t[hist_i] = now
    hist_x[hist_i] = x
    hist_y[hist_i] = y
    hist_h[hist_i] = th
    hist_w[hist_i] = w


def pose_at(t):
    """Pose and turn rate at ticks_us t (frames arrive ~60 ms after capture)."""
    i = hist_i
    for _ in range(HIST):
        if time.ticks_diff(t, hist_t[i]) >= 0:
            return hist_x[i], hist_y[i], hist_h[i], hist_w[i]
        i = (i - 1) % HIST
    return x, y, th, 9.9


def calibrate_gyro(ms):
    global gyro_bias, last_l, last_r, last_t, last_w
    n = 0
    s = 0.0
    end = time.ticks_add(time.ticks_ms(), ms)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        imu.gyro.read()
        s += imu.gyro.last_reading_dps[2]
        n += 1
        time.sleep_ms(3)
    gyro_bias = s / n if n else 0.0
    last_l, last_r = encoders.get_counts()
    last_t = time.ticks_us()
    last_w = 0.0


# --- ToF: cube map and wall fixes ------------------------------
tof = ToFSerial() if ToFSerial else None
BYTE_US = 10 * 1000000 // 115200
frames_seen = 0
fresh_after = 0             # frames captured before this ticks_us are ignored
cubes = []                  # [x, y, weight, seen, state 0 free/1 busy/2 placed, fails]
wall_k = WALL_K
wall_kh = WALL_KH
busy_near = None            # cube being pushed: keep its sightings out of the map
probe = None                # collects sightings while measuring one cube


def frame_points(f, px, py, ph):
    """Cube points of one frame, plus a wall fix. Each zone expects the nearer of
    floor and wall; a return clearly in front of that is a cube."""
    dist = f["distance_mm"]
    stat = f["target_status"]
    c = math.cos(ph)
    s = math.sin(ph)
    ox = px + TOF_X * c
    oy = py + TOF_X * s
    pts = []
    wd = [0.0] * 8              # wall depth per column
    wk = [0] * 8                # 1 x wall, 2 y wall, 0 corner/too oblique
    for col in range(8):
        dx = c - SY[col] * s
        dy = s + SY[col] * c
        tx = ((WALL if dx > 0 else -WALL) - ox) / dx if abs(dx) > 1e-6 else 99.0
        ty = ((WALL if dy > 0 else -WALL) - oy) / dy if abs(dy) > 1e-6 else 99.0
        n = WALL_COS * math.sqrt(dx * dx + dy * dy)
        if tx < ty:
            wd[col] = tx
            wk[col] = 1 if ty - tx > 0.12 and abs(dx) > n else 0
        else:
            wd[col] = ty
            wk[col] = 2 if tx - ty > 0.12 and abs(dy) > n else 0
    a00 = a02 = a11 = a12 = a22 = b0 = b1 = b2 = 0.0
    nw = 0
    for z in range(16, 64):     # rows 0, 1 look over every wall
        if stat[z] != 5:
            continue
        d = dist[z] * 0.001
        row = z >> 3
        col = z & 7
        bg = min(wd[col], FLOOR_D[row])
        ly = d * SY[col]
        wx = ox + d * c - ly * s
        wy = oy + d * s + ly * c
        if d < bg - 0.04 - 0.04 * d:
            if TOF_Z + d * SZ[row] < 0.07 and abs(wx) < WALL - 0.06 and abs(wy) < WALL - 0.06:
                pts.append((wx, wy, d))
            continue
        k = wk[col]
        if k == 0 or FLOOR_D[row] < wd[col] + 0.05 or abs(d - wd[col]) > 0.08:
            continue
        if k == 1:
            r = (WALL if wx > 0 else -WALL) - wx
            j = py - wy
            a00 += 1
            a02 += j
            b0 += r
        else:
            r = (WALL if wy > 0 else -WALL) - wy
            j = wx - px
            a11 += 1
            a12 += j
            b1 += r
        a22 += j * j
        b2 += j * r
        nw += 1
    if nw >= 4:
        wall_fix(a00, a02, a11, a12, a22, b0, b1, b2)
    return pts


def wall_fix(a00, a02, a11, a12, a22, b0, b1, b2):
    """Least squares for (dx, dy, dth) with a prior; nudge the pose part of the way."""
    global x, y, th
    a00 += 3.0
    a11 += 3.0
    a22 += 0.15
    det = a00 * (a11 * a22 - a12 * a12) - a02 * a11 * a02
    if abs(det) < 1e-9:
        return
    dx = (b0 * (a11 * a22 - a12 * a12) + a02 * (b1 * a12 - a11 * b2)) / det
    dy = (a00 * (b1 * a22 - a12 * b2) - b0 * (-a12 * a02) + a02 * (-b1 * a02)) / det
    dt = (a00 * (a11 * b2 - b1 * a12) + b0 * (-a11 * a02)) / det
    x += clamp(dx * wall_k, -0.015, 0.015)
    y += clamp(dy * wall_k, -0.015, 0.015)
    th = wrap(th + clamp(dt * wall_kh, -0.01, 0.01))


def observe(pts, px, py):
    """Cluster the points and merge them into the map (a face is seen: the centre
    is half a cube further away)."""
    cl = []
    for (qx, qy, d) in pts:
        for c in cl:
            if abs(c[0] / c[2] - qx) < 0.04 and abs(c[1] / c[2] - qy) < 0.04:
                c[0] += qx
                c[1] += qy
                c[2] += 1
                c[3] += d
                break
        else:
            cl.append([qx, qy, 1, d])
    for c in cl:
        n = c[2]
        cx = c[0] / n
        cy = c[1] / n
        dx = cx - px
        dy = cy - py
        r = math.sqrt(dx * dx + dy * dy)
        if r < 0.05:
            continue
        cx += dx / r * HALF
        cy += dy / r * HALF
        w = n / (0.1 + (c[3] / n) ** 2)
        if probe is not None:
            probe.append((cx, cy, w))
        if busy_near and abs(cx - busy_near[0]) < 0.15 and abs(cy - busy_near[1]) < 0.15:
            continue
        best = None
        bd = 0.12
        for cb in cubes:
            e = math.sqrt((cb[0] - cx) ** 2 + (cb[1] - cy) ** 2)
            if e < bd:
                bd = e
                best = cb
        if best is None:
            cubes.append([cx, cy, w, 1, 0, 0])
        elif best[4] == 0:
            tw = best[2] + w
            best[0] = (best[0] * best[2] + cx * w) / tw
            best[1] = (best[1] * best[2] + cy * w) / tw
            best[2] = min(tw, 400.0)
            best[3] += 1


def poll():
    """Odometry step, then every ToF frame that has arrived."""
    global frames_seen
    update()
    if tof is None:
        return
    while True:
        try:
            f = tof.read()
        except Exception:
            return
        if f is None:
            return
        # capture time = arrival minus the line's wire time (no synced clocks needed)
        cap = time.ticks_add(time.ticks_us(), -(len(f["distance_mm"]) * 11 * BYTE_US + 4000))
        if time.ticks_diff(cap, fresh_after) < 0:
            continue
        px, py, ph, pw = pose_at(cap)
        if abs(pw) > 0.35:
            continue            # turning: small timing errors smear the frame
        frames_seen += 1
        observe(frame_points(f, px, py, ph), px, py)


def wait_frames(n, timeout_ms=600):
    global fresh_after
    fresh_after = time.ticks_us()
    want = frames_seen + n
    end = time.ticks_add(time.ticks_ms(), timeout_ms)
    while frames_seen < want and time.ticks_diff(end, time.ticks_ms()) > 0:
        poll()
        time.sleep_ms(3)


def stop(ms):
    motors.set_speeds(0, 0)
    end = time.ticks_add(time.ticks_ms(), ms)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        poll()
        time.sleep_ms(3)


# --- Motion ----------------------------------------------------
def turn_to(target, tol=0.03):
    end = time.ticks_add(time.ticks_ms(), 2500)
    ok = 0
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        poll()
        e = wrap(target - th)
        ok = ok + 1 if abs(e) < tol else 0
        if ok >= 3:
            break
        pe = e - last_w * 0.06          # brake early for the coast
        u = min(TURN_MIN + 4500 * abs(pe), TURN_MAX)
        if pe < 0:
            u = -u
        motors.set_speeds(int(-u), int(u))
        time.sleep_ms(3)
    motors.set_speeds(0, 0)


def drive_to(tx, ty, stop_at=0.012):
    end = time.ticks_add(time.ticks_ms(), 6000)
    # a short way to a point behind: reverse instead of turning around
    back = math.sqrt((tx - x) ** 2 + (ty - y) ** 2) < 0.35 and abs(wrap(math.atan2(ty - y, tx - x) - th)) > 2.2
    while time.ticks_diff(end, time.ticks_ms()) > 0 and not out_of_time():
        poll()
        dx = tx - x
        dy = ty - y
        dist = math.sqrt(dx * dx + dy * dy)
        want = math.atan2(dy, dx)
        if back:
            want = wrap(want + math.pi)
        e = wrap(want - th)
        if abs(e) > 0.5 and dist > max(0.05, stop_at * 2):
            turn_to(want, 0.06)
            continue
        along = dist * math.cos(e)
        if along < stop_at:
            break
        v = clamp(800 + along * 16000, 1000, DRIVE_V)
        u = clamp(e * 5000, -2000, 2000) if dist > 0.04 else 0
        if back:
            motors.set_speeds(int(-v - u), int(-v + u))
        else:
            motors.set_speeds(int(v - u), int(v + u))
        time.sleep_ms(3)
    motors.set_speeds(0, 0)


def push_through(cx, cy, ux, uy, s_end, vend, vmax):
    """Drive along the line through (cx, cy), direction u, until the robot origin is
    s_end past (cx, cy). Steer onto the line only before touching the cube."""
    head = math.atan2(uy, ux)
    s0 = (x - cx) * ux + (y - cy) * uy
    end = time.ticks_add(time.ticks_ms(), int(2500 + (s_end - s0) * 9000))
    last_s = s0
    tick = time.ticks_ms()
    while time.ticks_diff(end, time.ticks_ms()) > 0 and not out_of_time():
        poll()
        rx = x - cx
        ry = y - cy
        s = rx * ux + ry * uy
        cross = ry * ux - rx * uy
        left = s_end - s
        if left <= 0.0:
            break
        want = head
        if s < -CONTACT - 0.025 and abs(cross) > 0.012:
            want = head - clamp(cross * 10.0, -0.3, 0.3)
        v = clamp(vend + (vmax - vend) * left / 0.08, vend, vmax)
        before = -CONTACT - s
        if before > -0.03:
            v = min(v, CONTACT_V + (vmax - CONTACT_V) * clamp((before - 0.015) / 0.06, 0.0, 1.0))
        else:
            v = min(v, CONTACT_V + (-before - 0.03) * 25000)
        u = clamp(wrap(want - th) * 5000, -1500, 1500)
        motors.set_speeds(int(v - u), int(v + u))
        if time.ticks_diff(time.ticks_ms(), tick) > 700:
            if s - last_s < 0.01:
                break           # stalled
            last_s = s
            tick = time.ticks_ms()
        time.sleep_ms(3)
    motors.set_speeds(0, 0)


def back_up(dist):
    sx = x
    sy = y
    head = th
    end = time.ticks_add(time.ticks_ms(), int(1500 + dist * 8000))
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        poll()
        d = math.sqrt((x - sx) ** 2 + (y - sy) ** 2)
        if d >= dist - 0.004:
            break
        v = clamp(900 + (dist - d) * 30000, 1000, 3400)
        u = clamp(wrap(head - th) * 4000, -1000, 1000)
        motors.set_speeds(int(-v - u), int(-v + u))
        time.sleep_ms(3)
    motors.set_speeds(0, 0)


# --- Planning --------------------------------------------------
def seg_dist(ax, ay, bx, by, px, py):
    dx = bx - ax
    dy = by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-9 else clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0.0, 1.0)
    return math.sqrt((ax + t * dx - px) ** 2 + (ay + t * dy - py) ** 2)


def obstacles(skip=-1):
    return [(cb[0], cb[1]) for i, cb in enumerate(cubes) if i != skip and (cb[3] >= 2 or cb[4] != 0)]


def inside(px, py, lim=LIMIT):
    return abs(px) <= lim and abs(py) <= lim


def seg_clear(ax, ay, bx, by, obs, clr):
    for (ox, oy) in obs:
        if seg_dist(ax, ay, bx, by, ox, oy) < clr:
            # starting too close is fine if the segment leads away
            if (ax - ox) ** 2 + (ay - oy) ** 2 < clr * clr and (bx - ax) * (ax - ox) + (by - ay) * (ay - oy) > 0 \
                    and (bx - ox) ** 2 + (by - oy) ** 2 >= clr * clr:
                continue
            return False
    return True


def point_clear(px, py, obs, clr, lim=LIMIT):
    if not inside(px, py, lim):
        return False
    for (ox, oy) in obs:
        if (px - ox) ** 2 + (py - oy) ** 2 < clr * clr:
            return False
    return True


class Blocked(Exception):
    pass


GRID = 0.04
GN = 37                     # grid cells per axis, +-0.72
g_block = bytearray(GN * GN)
g_border = bytearray(GN * GN)
for _i in range(GN):
    for _j in range(GN):
        if not inside((_i - GN // 2) * GRID, (_j - GN // 2) * GRID, LIMIT_PUSH):
            g_border[_i * GN + _j] = 1
g_came = array.array("h", [0] * (GN * GN))
g_queue = array.array("h", [0] * (GN * GN))


def grid_plan(tx, ty, obs):
    """Breadth-first wave on a coarse grid around the cubes, then shortcut it."""
    def cell(px, py):
        return clamp(int(round(px / GRID)) + GN // 2, 0, GN - 1) * GN + clamp(int(round(py / GRID)) + GN // 2, 0, GN - 1)
    c2 = (CLR - 0.01) ** 2
    g_block[:] = g_border
    for k in range(GN * GN):
        g_came[k] = -2
    r = int(CLR / GRID) + 1
    for (ox, oy) in obs:
        ci = int(round(ox / GRID)) + GN // 2
        cj = int(round(oy / GRID)) + GN // 2
        for gx in range(max(0, ci - r), min(GN, ci + r + 1)):
            px = (gx - GN // 2) * GRID - ox
            for gy in range(max(0, cj - r), min(GN, cj + r + 1)):
                py = (gy - GN // 2) * GRID - oy
                if px * px + py * py < c2:
                    g_block[gx * GN + gy] = 1
    start = cell(x, y)
    goal = cell(tx, ty)
    g_block[start] = 0
    g_block[goal] = 0
    g_came[start] = -1
    g_queue[0] = start
    head = 0
    tail = 1
    while head < tail:
        cur = g_queue[head]
        head += 1
        if cur == goal:
            break
        cx = cur // GN
        cy = cur % GN
        for nx in (cx - 1, cx, cx + 1):
            if nx < 0 or nx >= GN:
                continue
            for ny in (cy - 1, cy, cy + 1):
                if ny < 0 or ny >= GN:
                    continue
                n = nx * GN + ny
                if g_block[n] or g_came[n] != -2:
                    continue
                g_came[n] = cur
                g_queue[tail] = n
                tail += 1
    if g_came[goal] == -2:
        return None
    cells = []
    n = goal
    while n != -1:
        cells.append(((n // GN - GN // 2) * GRID, (n % GN - GN // 2) * GRID))
        n = g_came[n]
    cells.reverse()
    cells[-1] = (tx, ty)
    out = []
    px, py = x, y
    k = 0
    while k < len(cells) - 1:
        j = len(cells) - 1
        while j > k + 1 and not seg_clear(px, py, cells[j][0], cells[j][1], obs, CLR - 0.01):
            j -= 1
        out.append(cells[j])
        px, py = cells[j]
        k = j
    return out or [(tx, ty)]


def travel(tx, ty, obs):
    path = [(tx, ty)] if seg_clear(x, y, tx, ty, obs, CLR) else grid_plan(tx, ty, obs)
    if path is None:
        raise Blocked()
    for (px, py) in path:
        drive_to(px, py, 0.02 if (px, py) != (tx, ty) else 0.008)
        if out_of_time():
            return


# --- Pushing ---------------------------------------------------
def look_at(cx, cy, radius=0.06, frames=2):
    """Fresh sightings near (cx, cy) -> weighted mean, or None."""
    global probe, busy_near
    stop(60)
    probe = []
    old = busy_near
    busy_near = (cx, cy)
    wait_frames(frames)
    others = [(cb[0], cb[1]) for cb in cubes if cb[4] == 2]
    sx = sy = sw = 0.0
    for (px, py, w) in probe:
        d2 = (px - cx) ** 2 + (py - cy) ** 2
        if d2 > radius * radius or any((px - ox) ** 2 + (py - oy) ** 2 < d2 for (ox, oy) in others):
            continue
        sx += px * w
        sy += py * w
        sw += w
    probe = None
    busy_near = old
    return (sx / sw, sy / sw) if sw else None


def localize(heading):
    """Face a wall square on; let a few frames pull the pose in hard."""
    global wall_k, wall_kh
    turn_to(heading, 0.04)
    wall_k = 0.6
    wall_kh = 0.5
    stop(40)
    wait_frames(4)
    wall_k = WALL_K
    wall_kh = WALL_KH


def measure_from(i, ux, uy):
    """Stand behind cube i looking along u and measure it. Returns its position
    (the map's if there is no room to stand), or None if it is gone."""
    cb = cubes[i]
    cx = cb[0]
    cy = cb[1]
    od = None
    for d in (OBSERVE, 0.205, 0.18):
        if inside(cx - ux * d, cy - uy * d, LIMIT_PUSH):
            od = d
            break
    if od is None:
        return (cx, cy)
    for attempt in range(2):
        ox = cx - ux * od
        oy = cy - uy * od
        if math.sqrt((x - ox) ** 2 + (y - oy) ** 2) > 0.015:
            travel(ox, oy, obstacles(i) + [(cx, cy)])
        turn_to(math.atan2(uy, ux), 0.03)
        m = look_at(cx, cy) or look_at(cx, cy, 0.09, 4)
        if m is None:
            log("cube %d lost" % i)
            return None
        cx, cy = m
        cb[0] = cx
        cb[1] = cy
        if abs((y - cy) * ux - (x - cx) * uy) < 0.01 or not inside(cx - ux * od, cy - uy * od, LIMIT_PUSH):
            break
    return (cx, cy)


def push_dir(i, ux, uy, dist, vend):
    """Push cube i by dist along u; its sideways position stays."""
    global busy_near
    cb = cubes[i]
    cx = cb[0]
    cy = cb[1]
    s0 = (x - cx) * ux + (y - cy) * uy
    if s0 > -CONTACT - 0.02 or s0 < -OBSERVE - 0.05 or abs((y - cy) * ux - (x - cx) * uy) > 0.02:
        travel(cx - ux * (CONTACT + LEAD), cy - uy * (CONTACT + LEAD), obstacles(i) + [(cx, cy)])
    turn_to(math.atan2(uy, ux), 0.02)
    busy_near = (cx, cy)
    push_through(cx, cy, ux, uy, dist - CONTACT, vend, PUSH_FINAL_V if vend == PUSH_END_V else PUSH_V)
    stop(60)
    s = (x - cx) * ux + (y - cy) * uy + CONTACT
    cb[0] = cx + ux * s
    cb[1] = cy + uy * s
    busy_near = None


def leg(i, ux, uy, tx, ty, vend=1800):
    """Push cube i along u until level with (tx, ty). False if it got lost."""
    for k in range(4):
        c = measure_from(i, ux, uy)
        if c is None:
            return False
        d = (tx - c[0]) * ux + (ty - c[1]) * uy
        if d < 0.006:
            return True
        push_dir(i, ux, uy, min(d, MAX_LEG), vend)
        back_up(OBSERVE - CONTACT)
        if d <= MAX_LEG or out_of_time():
            return True
    return True


def place(i, slot, legs):
    """Legs onto the slot axis, then fix pose and cube sideways, then push it home."""
    gx, gy, ux, uy = slot
    nx = -uy
    ny = ux
    ring = abs(gx) > 0.1 or abs(gy) > 0.1     # 1-point spot: room to spare
    cb = cubes[i]
    log("cube %d (%.2f %.2f) -> (%.3f %.3f)" % (i, cb[0], cb[1], gx, gy))
    cb[4] = 1
    if (cb[0] - gx) * -ux + (cb[1] - gy) * -uy > STAGE + 0.03:
        legs = legs + [(ux, uy, gx - ux * STAGE, gy - uy * STAGE)]
    for (lx, ly, tx, ty) in legs:
        if not leg(i, lx, ly, tx, ty):
            return False
        if out_of_time():
            return True
    loc = ring
    for k in range(1 if ring else 4):
        c = measure_from(i, ux, uy)
        if c is None:
            return False
        if not loc:
            loc = True
            side = 1.0 if x * nx + y * ny > 0 else -1.0
            localize(math.atan2(uy, ux) - side * math.pi / 2)
            continue
        lat = (c[0] - gx) * nx + (c[1] - gy) * ny
        if abs(lat) <= (0.015 if ring else LAT_TOL) or out_of_time():
            break
        sx = -nx if lat > 0 else nx
        sy = -ny if lat > 0 else ny
        if not point_clear(c[0] - sx * (CONTACT + LEAD), c[1] - sy * (CONTACT + LEAD), obstacles(i), STAND, LIMIT_PUSH):
            break
        if not leg(i, sx, sy, c[0] - nx * lat, c[1] - ny * lat, PUSH_END_V):
            return False
    for attempt in range(2):
        d = (gx - cb[0]) * ux + (gy - cb[1]) * uy
        if d > 0.006:
            push_dir(i, ux, uy, d, PUSH_END_V)
        back_up(OBSERVE - CONTACT)
        m = look_at(cb[0], cb[1])
        if not m:
            break
        cb[0], cb[1] = m
        if (gx - m[0]) * ux + (gy - m[1]) * uy < 0.012 or out_of_time():
            break
    log("  placed (%.3f %.3f)" % (cb[0], cb[1]))
    cb[4] = 2
    return True


def leg_ok(ax, ay, bx, by, obs):
    """Can a cube be pushed from a to b? The robot's start point, or None."""
    L = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
    if L < 0.01:
        return (ax, ay)
    ux = (bx - ax) / L
    uy = (by - ay) / L
    px = ax - ux * (CONTACT + LEAD)
    py = ay - uy * (CONTACT + LEAD)
    if not point_clear(px, py, obs, STAND, LIMIT_PUSH):
        return None
    rx = bx - ux * CONTACT
    ry = by - uy * CONTACT
    for (ox, oy) in obs:
        if seg_dist(ax, ay, bx, by, ox, oy) < SWEEP_CUBE or seg_dist(px, py, rx, ry, ox, oy) < SWEEP_ROBOT:
            return None
    return (px, py)


def legs_from(cx, cy, slot, obs):
    """Axis-aligned legs onto the slot axis -> (cost, [(ux, uy, tx, ty)]) or None.
    Along an axis a squared cube stays squared and does not slip on the blade."""
    gx, gy, ux, uy = slot
    nx = -uy
    ny = ux
    a = (cx - gx) * -ux + (cy - gy) * -uy      # how far out along the axis
    l = (cx - gx) * nx + (cy - gy) * ny        # how far to the side
    sx = gx - ux * STAGE
    sy = gy - uy * STAGE
    # out/in to the staging distance then sideways, or (far enough out) sideways first
    opts = [[(cx, cy, sx + nx * l, sy + ny * l), (sx + nx * l, sy + ny * l, sx, sy)]]
    if a > STAGE - 0.04:
        opts.append([(cx, cy, gx - ux * a, gy - uy * a)])
    best = None
    for opt in opts:
        cost = 0.0
        out = []
        for (ax, ay, bx, by) in opt:
            L = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
            if L < 0.01:
                continue
            p = leg_ok(ax, ay, bx, by, obs)
            if p is None:
                cost = None
                break
            cost += (0.6 if out else math.sqrt((p[0] - x) ** 2 + (p[1] - y) ** 2)) + L * 1.6 + 0.4
            out.append(((bx - ax) / L, (by - ay) / L, bx, by))
        if cost is not None and (best is None or cost < best[0]):
            best = (cost, out)
    return best


def route_for(i, slot, obs):
    """(cost, legs) for cube i to this slot, or None. A cube at a wall first gets
    a shallow escape push along it."""
    gx, gy, ux, uy = slot
    if leg_ok(gx - ux * STAGE, gy - uy * STAGE, gx, gy, obs) is None:
        return None
    tail = 1.5 + STAGE * 1.6
    cx, cy = cubes[i][0], cubes[i][1]
    r = legs_from(cx, cy, slot, obs)
    if r is not None:
        return (r[0] + tail, r[1])
    best = None
    for k in range(24):
        ca = math.cos(k * math.pi / 12)
        sa = math.sin(k * math.pi / 12)
        for dist in (0.2, 0.35):
            mx = cx + dist * ca
            my = cy + dist * sa
            if abs(mx) > 0.6 or abs(my) > 0.6 or leg_ok(cx, cy, mx, my, obs) is None:
                continue
            r = legs_from(mx, my, slot, obs)
            if r is not None and (best is None or r[0] + dist * 1.6 + 1.5 + tail < best[0]):
                best = (r[0] + dist * 1.6 + 1.5 + tail, [(ca, sa, mx, my)] + r[1])
    return best


# --- Strategy --------------------------------------------------
slot_used = [False] * 4
ring_used = [False] * 4


def choose():
    """Cheapest (cube, spot, is_slot, legs): slots first, quick ring spots near the end."""
    best = None
    bc = 99.0
    late = elapsed() > TOTAL_MS - 30000
    for pool, used, bonus in ((SLOTS, slot_used, 0.0), (RINGS, ring_used, -1.0 if late else 1.5)):
        for k in range(4):
            if used[k]:
                continue
            for i, cb in enumerate(cubes):
                if cb[4] != 0 or cb[3] < 3 or cb[5] >= 2:
                    continue
                r = route_for(i, pool[k], obstacles(i))
                if r is not None and r[0] + bonus < bc:
                    bc = r[0] + bonus
                    best = (i, k, pool is SLOTS, r[1])
        if best is not None and not late:
            return best
    return best


def scan(steps):
    start = th
    for i in range(steps):
        turn_to(wrap(start + i * 2 * math.pi / steps), 0.05)
        stop(30)
        wait_frames(2)


def finish():
    motors.set_speeds(0, 0)
    for _ in range(2):
        for k in range(6):
            leds.set(k, (0, 255, 0))
        leds.show()
        time.sleep_ms(300)
        leds.off()
        time.sleep_ms(300)


def main():
    global t0, cubes
    wait_for_start()
    t0 = time.ticks_ms()
    show("Challenge 2", "running")
    calibrate_gyro(250)
    scan(10)
    cubes = [cb for cb in cubes if cb[3] >= 2]
    log("scan: %d cubes, mem %d" % (len(cubes), gc.mem_free()))
    rescanned = False
    while not out_of_time():
        pick = choose()
        if pick is None:
            if rescanned or elapsed() > TOTAL_MS - 15000:
                break
            rescanned = True        # something may hide behind a cube: look again
            scan(8)
            continue
        i, k, is_slot, route = pick
        used = slot_used if is_slot else ring_used
        used[k] = True
        try:
            ok = place(i, (SLOTS if is_slot else RINGS)[k], route)
            if not ok:
                del cubes[i]        # not where it should be; the map finds it again
        except Exception as e:      # Blocked or anything else: never end the run
            motors.set_speeds(0, 0)
            log("gave up on cube %d: %r" % (i, e))
            ok = False
            if i < len(cubes):
                cubes[i][4] = 0
                cubes[i][5] += 2 if isinstance(e, Blocked) else 1
        if not ok:
            used[k] = False
        gc.collect()
    finish()


try:
    main()
finally:
    motors.off()
