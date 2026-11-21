# Name: Secret Algo v2 · ToF-Karte & Pinwheel
# Strategy: Like Secret Algo, plus: long pushes split every 30 cm with the cube re-measured before each push,
#   sideways error corrected, the pose pulled in against a wall before the last push, pushes close to walls.
# Sensors: tof
# Arena: push-pull
#
# On the robot: copy spark_tof.py next to this file (Download menu).
# ─────────────────────────────────────────────────────────────
#  SPARK · Zumo 2040 · Challenge 2 · Push & Pull
#
#  World frame: origin = centre of the start square, +x = the direction
#  the robot faces at the start, +y = to its left. Metres, radians.
#  Walls at +-0.80. Home zones: |x|,|y| <= 0.085 (3 pts), <= 0.175 (1 pt).
# ─────────────────────────────────────────────────────────────
from zumo_2040_robot import robot
import time
import math

try:
    from spark_tof import ToFSerial
except ImportError:
    ToFSerial = None

DEBUG = True

# --- Hardware --------------------------------------------------
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


# --- Constants -------------------------------------------------
WALL = 0.80
FRONT = 0.0589              # robot origin -> blade edge
HALF = 0.03                 # half a cube
CONTACT = FRONT + HALF      # robot origin -> cube centre while pushing
TOF_X = 0.052               # sensor position in the robot frame
TOF_Z = 0.09
TAN_HALF = math.tan(math.radians(22.5))
COUNTS_PER_M = 8467
TOTAL_MS = 120000
TURN_MIN = 1450             # spinning in place: below ~1300 the tracks do not move
TURN_MAX = 3600
DRIVE_V = 4000              # travel
PUSH_V = 2800               # pushing
PUSH_END_V = 1500           # last cm of a final push
LIMIT = 0.70                # robot origin stays within +-LIMIT (room to turn at the wall)
LIMIT_PUSH = 0.735          # a push may start this close to a wall (robot parallel to it)
CLR = 0.125                 # robot origin <-> cube centre while travelling
LEAD = 0.05                 # start a push this far behind the cube
STAGE = 0.20                # staging point: this far out on the slot's axis line
OBSERVE = 0.23              # robot origin -> cube centre when re-measuring a cube
SWEEP_CUBE = 0.075          # a pushed cube's path stays this far from other cubes
WALL_COS = 0.7              # wall points only where the ray meets the wall within 45 deg
ROT_X = 0.0063              # spin centre (middle between the axles) ahead of the robot origin
WALL_K = 0.3                # wall fix gain, position
WALL_KH = 0.1               # wall fix gain, heading (the gyro is good)
CONTACT_V = 1300            # speed around first contact
MAX_LEG = 0.30              # longest push without re-measuring the cube
SWEEP_ROBOT = 0.10          # the pushing robot's path stays this far from other cubes

# zone slopes (8x8, ULD order: zone 0 = top right)
SY = [math.tan(((c + 0.5) / 4.0 - 1.0) * math.radians(22.5)) for c in range(8)]
SZ = [math.tan((1.0 - (r + 0.5) / 4.0) * math.radians(22.5)) for r in range(8)]
# depth where a zone's lowest rays meet the floor (99 = never)
FLOOR_D = []
for _r in range(8):
    _s = math.tan((1.0 - (_r + 0.8) / 4.0) * math.radians(22.5))
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


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def update():
    """Integrate encoders (distance) and gyro (heading); remember the pose."""
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
    # the robot spins about the middle between the axles, ROT_X ahead of its origin
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
    """Pose and turn rate at ticks_us t from the history (nearest older entry)."""
    i = hist_i
    for _ in range(HIST):
        if time.ticks_diff(t, hist_t[i]) >= 0:
            return hist_x[i], hist_y[i], hist_h[i], hist_w[i]
        i = (i - 1) % HIST
    i = (hist_i + 1) % HIST
    return hist_x[i], hist_y[i], hist_h[i], 9.9


def calibrate_gyro(ms=300):
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


# --- ToF: frames, cube map, wall fixes ----------------------------
tof = ToFSerial() if ToFSerial else None
BYTE_US = 10 * 1000000 // 115200
frames_seen = 0
fresh_after = 0             # ticks_us: frames captured before this are ignored
cubes = []                  # [x, y, weight, n_seen, state]  state: 0 free, 1 done, 2 skip
wall_fix_on = True
wall_k = 0.3
n_wall = 0
busy_near = None            # (x, y): cube being pushed, do not map points close to it
probe = None                # list: collects cube sightings of fresh frames


def frame_points(f, px, py, ph):
    """Cube points (world) from one frame; also applies a wall fix.

    Background subtraction against the known arena: every zone's expected depth is
    the nearer of the floor (lowest sub-ray, it returns the strongest echo) and the
    wall. A return clearly shorter than that is a cube; one at the wall depth is wall.
    """
    dist = f["distance_mm"]
    stat = f["target_status"]
    c = math.cos(ph)
    s = math.sin(ph)
    ox = px + TOF_X * c
    oy = py + TOF_X * s
    cube_pts = []
    # wall depth per column (perpendicular depth = ray length per unit forward)
    wd = [0.0] * 8
    wk = [0] * 8                    # 1 = x wall, 2 = y wall, 0 = corner/none
    for col in range(8):
        sy = SY[col]
        dx = c - sy * s
        dy = s + sy * c
        tx = ((WALL if dx > 0 else -WALL) - ox) / dx if abs(dx) > 1e-6 else 99.0
        ty = ((WALL if dy > 0 else -WALL) - oy) / dy if abs(dy) > 1e-6 else 99.0
        if tx < ty:
            wd[col] = tx
            wk[col] = 1 if ty - tx > 0.12 and abs(dx) > WALL_COS * math.sqrt(dx * dx + dy * dy) else 0
        else:
            wd[col] = ty
            wk[col] = 2 if tx - ty > 0.12 and abs(dy) > WALL_COS * math.sqrt(dx * dx + dy * dy) else 0
    # normal equations for (dx, dy, dth)
    a00 = a02 = a11 = a12 = a22 = b0 = b1 = b2 = 0.0
    nw = 0
    for z in range(16, 64):         # rows 0, 1 look above every wall
        if stat[z] != 5:
            continue
        d = dist[z] * 0.001
        row = z >> 3
        col = z & 7
        bg = wd[col]
        if FLOOR_D[row] < bg:
            bg = FLOOR_D[row]
        ly = d * SY[col]
        wx = ox + d * c - ly * s
        wy = oy + d * s + ly * c
        if d < bg - 0.04 - 0.04 * d:
            if TOF_Z + d * SZ[row] < 0.07 and abs(wx) < WALL - 0.06 and abs(wy) < WALL - 0.06:
                cube_pts.append((wx, wy, d))
            continue
        k = wk[col]
        if k == 0 or FLOOR_D[row] < wd[col] + 0.05 or abs(d - wd[col]) > 0.08:
            continue
        if k == 1:
            r = (WALL if wx > 0 else -WALL) - wx
            j2 = -(wy - py)
            a00 += 1; a02 += j2; b0 += r
        else:
            r = (WALL if wy > 0 else -WALL) - wy
            j2 = wx - px
            a11 += 1; a12 += j2; b1 += r
        a22 += j2 * j2; b2 += j2 * r
        nw += 1
    if wall_fix_on and nw >= 4:
        wall_fix(a00, 0.0, a02, a11, a12, a22, b0, b1, b2, nw)
    return cube_pts


def wall_fix(a00, a01, a02, a11, a12, a22, b0, b1, b2, nw):
    """Solve the small least-squares problem with a prior and nudge the pose."""
    global x, y, th, n_wall
    lam = 3.0                       # prior: keep unobserved directions
    a00 += lam; a11 += lam; a22 += lam * 0.05
    # 3x3 solve (symmetric), Cramer
    det = (a00 * (a11 * a22 - a12 * a12) - a01 * (a01 * a22 - a12 * a02) + a02 * (a01 * a12 - a11 * a02))
    if abs(det) < 1e-9:
        return
    dx = (b0 * (a11 * a22 - a12 * a12) - a01 * (b1 * a22 - a12 * b2) + a02 * (b1 * a12 - a11 * b2)) / det
    dy = (a00 * (b1 * a22 - a12 * b2) - b0 * (a01 * a22 - a12 * a02) + a02 * (a01 * b2 - b1 * a02)) / det
    dt = (a00 * (a11 * b2 - b1 * a12) - a01 * (a01 * b2 - b1 * a02) + b0 * (a01 * a12 - a11 * a02)) / det
    dx = clamp(dx * wall_k, -0.015, 0.015)
    dy = clamp(dy * wall_k, -0.015, 0.015)
    dt = clamp(dt * WALL_KH, -0.005, 0.005)
    x += dx
    y += dy
    th = wrap(th + dt)
    n_wall += 1


def cluster(pts, radius=0.06):
    """Greedy clusters of cube points -> [(cx, cy, n, mean_d)]."""
    out = []
    for (px, py, d) in pts:
        for cl in out:
            if abs(cl[0] / cl[2] - px) < radius and abs(cl[1] / cl[2] - py) < radius:
                cl[0] += px; cl[1] += py; cl[2] += 1; cl[3] += d
                break
        else:
            out.append([px, py, 1, d])
    return [(cl[0] / cl[2], cl[1] / cl[2], cl[2], cl[3] / cl[2]) for cl in out]


def observe(pts, px, py):
    """Merge cube clusters into the map. The face is seen: the centre is HALF further."""
    for (cx, cy, n, md) in cluster(pts):
        dx = cx - px
        dy = cy - py
        r = math.sqrt(dx * dx + dy * dy)
        if r < 0.05:
            continue
        cx += dx / r * HALF
        cy += dy / r * HALF
        if max(abs(cx), abs(cy)) > WALL - HALF + 0.01:
            continue
        w = n / (0.1 + md * md)
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
            cubes.append([cx, cy, w, 1, 0])
        elif best[4] == 0:
            tw = best[2] + w
            best[0] = (best[0] * best[2] + cx * w) / tw
            best[1] = (best[1] * best[2] + cy * w) / tw
            best[2] = min(tw, 400.0)
            best[3] += 1


last_dbg = 0


def poll():
    """Odometry step + any ToF frame that has arrived."""
    global frames_seen, last_dbg
    update()
    if DEBUG and time.ticks_diff(time.ticks_ms(), last_dbg) >= 1000:
        last_dbg = time.ticks_ms()
        print("est %.1f %.1f %.0f" % (x * 100, y * 100, math.degrees(th)))
    if tof is None:
        return
    while True:
        try:
            f = tof.read()
        except Exception:
            return
        if f is None:
            return
        arrival = time.ticks_us()
        # capture time: wire time of the line + some margin (works without synced clocks)
        cap = time.ticks_add(arrival, -(len(f["distance_mm"]) * 11 * BYTE_US + 4000))
        if time.ticks_diff(cap, fresh_after) < 0:
            continue
        px, py, ph, pw = pose_at(cap)
        if abs(pw) > 0.35:
            continue                # turning: a few ms of timing error smear the frame
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


def idle(ms):
    end = time.ticks_add(time.ticks_ms(), ms)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        poll()
        time.sleep_ms(3)


# --- Motion ----------------------------------------------------
def stop(ms=80):
    motors.set_speeds(0, 0)
    idle(ms)


def turn_to(target, tol=0.03, timeout_ms=2500):
    end = time.ticks_add(time.ticks_ms(), timeout_ms)
    ok = 0
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        poll()
        e = wrap(target - th)
        if abs(e) < tol:
            ok += 1
            if ok >= 3:
                break
        else:
            ok = 0
        # brake early: predicted error after the coast; spinning needs ~1300 to move at all
        pe = e - last_w * 0.06
        u = TURN_MIN + 4500 * abs(pe)
        if u > TURN_MAX:
            u = TURN_MAX
        if pe < 0:
            u = -u
        motors.set_speeds(int(-u), int(u))
        time.sleep_ms(3)
    motors.set_speeds(0, 0)


def drive_to(tx, ty, vmax=None, stop_at=0.012, back=False, timeout_ms=6000):
    """Drive with heading control to (tx, ty). Returns when within stop_at."""
    if vmax is None:
        vmax = DRIVE_V
    end = time.ticks_add(time.ticks_ms(), timeout_ms)
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
        v = clamp(800 + along * 16000, 1000, vmax)
        u = clamp(e * 5000, -2000, 2000) if dist > 0.04 else 0
        if back:
            motors.set_speeds(int(-v + u), int(-v - u))
        else:
            motors.set_speeds(int(v - u), int(v + u))
        time.sleep_ms(3)
    motors.set_speeds(0, 0)
    log("  drive_to %.3f %.3f ended at %.3f %.3f" % (tx, ty, x, y))


def push_through(cx, cy, ux, uy, s_end, vend):
    """Drive along the line through (cx, cy) with direction u until the robot origin
    reaches s_end (measured from (cx, cy) along u). Slow around first contact."""
    head = math.atan2(uy, ux)
    s0 = (x - cx) * ux + (y - cy) * uy
    end = time.ticks_add(time.ticks_ms(), int(2500 + (s_end - s0) * 9000))
    last_s = s0
    last_tick = time.ticks_ms()
    while time.ticks_diff(end, time.ticks_ms()) > 0 and not out_of_time():
        poll()
        rx = x - cx
        ry = y - cy
        s = rx * ux + ry * uy
        cross = -rx * uy + ry * ux          # left of the line > 0
        left = s_end - s
        if left <= 0.0:
            break
        if s < -CONTACT - 0.025 and abs(cross) > 0.006:
            want = head - clamp(cross * 10.0, -0.3, 0.3)
        else:
            want = head                     # touching: hold the heading, never steer the cube
        e = wrap(want - th)
        v = clamp(vend + (PUSH_V - vend) * left / 0.08, vend, PUSH_V)
        if -CONTACT - 0.015 < s < -CONTACT + 0.03:
            v = min(v, CONTACT_V)           # let the cube square up against the blade
        u = clamp(e * 5000, -1500, 1500)
        motors.set_speeds(int(v - u), int(v + u))
        if time.ticks_diff(time.ticks_ms(), last_tick) > 700:
            if s - last_s < 0.01:
                log("push stalled")
                break
            last_s = s
            last_tick = time.ticks_ms()
        time.sleep_ms(3)
    motors.set_speeds(0, 0)


def back_up(dist, vmax=2600):
    """Reverse straight by dist (no turning)."""
    sx = x
    sy = y
    head = th
    end = time.ticks_add(time.ticks_ms(), int(1500 + dist * 8000))
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        poll()
        d = math.sqrt((x - sx) ** 2 + (y - sy) ** 2)
        if d >= dist - 0.004:
            break
        v = clamp(900 + (dist - d) * 15000, 1000, vmax)
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
    ex = ax + t * dx - px
    ey = ay + t * dy - py
    return math.sqrt(ex * ex + ey * ey)


def obstacles(skip=-1):
    return [(cb[0], cb[1]) for i, cb in enumerate(cubes) if i != skip and (cb[3] >= 2 or cb[4] != 0)]


def inside(px, py, lim=LIMIT):
    return abs(px) <= lim and abs(py) <= lim


def seg_clear(ax, ay, bx, by, obs, clr):
    for (ox, oy) in obs:
        if seg_dist(ax, ay, bx, by, ox, oy) < clr:
            # already too close at the start: fine if the segment leads away
            if math.sqrt((ax - ox) ** 2 + (ay - oy) ** 2) < clr and (bx - ax) * (ax - ox) + (by - ay) * (ay - oy) > 0 \
                    and math.sqrt((bx - ox) ** 2 + (by - oy) ** 2) >= clr:
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


def plan(tx, ty, obs):
    """Waypoints from here to (tx, ty) around the obstacles (0, 1 or 2 detours)."""
    if seg_clear(x, y, tx, ty, obs, CLR):
        return [(tx, ty)]
    cands = []
    for (ox, oy) in obs:
        for k in range(8):
            a = k * math.pi / 4
            px = ox + (CLR + 0.04) * math.cos(a)
            py = oy + (CLR + 0.04) * math.sin(a)
            if point_clear(px, py, obs, CLR):
                cands.append((px, py))
    for k in range(8):
        a = k * math.pi / 4 + math.pi / 8
        px = 0.34 * math.cos(a)
        py = 0.34 * math.sin(a)
        if point_clear(px, py, obs, CLR):
            cands.append((px, py))
    best = None
    bl = 99.0
    first = [c for c in cands if seg_clear(x, y, c[0], c[1], obs, CLR)]
    for (px, py) in first:
        if seg_clear(px, py, tx, ty, obs, CLR):
            l = math.sqrt((px - x) ** 2 + (py - y) ** 2) + math.sqrt((tx - px) ** 2 + (ty - py) ** 2)
            if l < bl:
                bl = l
                best = [(px, py), (tx, ty)]
    if best:
        return best
    last = [c for c in cands if seg_clear(c[0], c[1], tx, ty, obs, CLR)]
    for (ax, ay) in first:
        la = math.sqrt((ax - x) ** 2 + (ay - y) ** 2)
        for (bx, by) in last:
            l = la + math.sqrt((bx - ax) ** 2 + (by - ay) ** 2) + math.sqrt((tx - bx) ** 2 + (ty - by) ** 2)
            if l < bl and seg_clear(ax, ay, bx, by, obs, CLR):
                bl = l
                best = [(ax, ay), (bx, by), (tx, ty)]
    return best


def path_len(path):
    px = x
    py = y
    l = 0.0
    for (qx, qy) in path:
        l += math.sqrt((qx - px) ** 2 + (qy - py) ** 2)
        px = qx
        py = qy
    return l


def travel(tx, ty, obs):
    path = plan(tx, ty, obs)
    if path is None:
        log("no path to %.2f %.2f" % (tx, ty))
        path = [(tx, ty)]
    log("  path " + " ".join(["(%.2f %.2f)" % q for q in path]))
    for (px, py) in path:
        drive_to(px, py, stop_at=0.02 if (px, py) != (tx, ty) else 0.008)
        if out_of_time():
            return


# --- Pushing ---------------------------------------------------
def look_at(cx, cy, radius=0.06, frames=3):
    """Fresh sightings near (cx, cy): weighted mean, or None."""
    global probe, busy_near
    stop(60)
    probe = []
    old = busy_near
    busy_near = (cx, cy)
    wait_frames(frames)
    others = [(cb[0], cb[1]) for cb in cubes if cb[4] == 2]
    log("  probe near %.3f %.3f: %s" % (cx, cy, ["%.3f %.3f %.0f" % q for q in probe]))
    sx = sy = sw = 0.0
    for (px, py, w) in probe:
        d2 = (px - cx) ** 2 + (py - cy) ** 2
        if d2 > radius * radius:
            continue
        if any((px - ox) ** 2 + (py - oy) ** 2 < d2 for (ox, oy) in others):
            continue
        sx += px * w
        sy += py * w
        sw += w
    probe = None
    busy_near = old
    if sw == 0:
        return None
    return sx / sw, sy / sw


def observe_dist(cx, cy, ux, uy):
    """How far behind the cube the robot can stand to measure it (None: too close to a wall)."""
    for d in (OBSERVE, 0.205, 0.18):
        if inside(cx - ux * d, cy - uy * d, LIMIT_PUSH):
            return d
    return None


def localize(heading, frames=4):
    """Face a wall square on and let several frames pull the pose in."""
    global wall_k
    turn_to(heading, 0.04)
    wall_k = 0.6
    wait_frames(frames)
    wall_k = WALL_K


def push_leg(i, gx, gy, final):
    """Measure cube i from behind, then push it so its centre ends on (gx, gy).
    Returns False when the cube was not found where it should be."""
    global busy_near
    cb = cubes[i]
    cx = cb[0]
    cy = cb[1]
    L = math.sqrt((gx - cx) ** 2 + (gy - cy) ** 2)
    if L < 0.008:
        return True
    ux = (gx - cx) / L
    uy = (gy - cy) / L
    od = observe_dist(cx, cy, ux, uy)
    if od is not None:
        for attempt in range(3):
            ox = cx - ux * od
            oy = cy - uy * od
            if math.sqrt((x - ox) ** 2 + (y - oy) ** 2) > 0.015:
                travel(ox, oy, obstacles(i) + [(cx, cy)])
            turn_to(math.atan2(uy, ux), 0.03)
            m = look_at(cx, cy)
            if m is None:
                m = look_at(cx, cy, 0.09, 4)
            if m is None:
                log("  cube %d lost near %.3f %.3f" % (i, cx, cy))
                return False
            cx, cy = m
            cb[0], cb[1] = cx, cy
            L = math.sqrt((gx - cx) ** 2 + (gy - cy) ** 2)
            if L < 0.008:
                return True
            ux = (gx - cx) / L
            uy = (gy - cy) / L
            cross = -(x - cx) * uy + (y - cy) * ux
            if final and attempt == 0:
                # sideways error here is sideways error of the cube in the slot
                h = math.atan2(uy, ux)
                hx = h + math.pi / 2
                side = -1.0 if (x * math.cos(hx) + y * math.sin(hx)) < 0 else 1.0
                localize(h - side * math.pi / 2)
                continue
            if abs(cross) < 0.015:
                break
            od = observe_dist(cx, cy, ux, uy)
            if od is None:
                break
        turn_to(math.atan2(uy, ux), 0.02)
    else:
        px = cx - ux * (CONTACT + LEAD)
        py = cy - uy * (CONTACT + LEAD)
        travel(px, py, obstacles(i) + [(cx, cy)])
        turn_to(math.atan2(uy, ux), 0.02)
    busy_near = (cx, cy)
    push_through(cx, cy, ux, uy, L - CONTACT, PUSH_END_V if final else 1800)
    stop(60)
    cb[0] = x + math.cos(th) * CONTACT
    cb[1] = y + math.sin(th) * CONTACT
    busy_near = None
    return True


def lose(i):
    """Forget a cube that was not where it should be; the map finds it again."""
    del cubes[i]


def place(i, slot, route):
    """Push cube i along its route (cube waypoints) into the slot. False if lost."""
    gx, gy, ux, uy = slot
    cb = cubes[i]
    log("cube %d (%.3f %.3f) -> %.3f %.3f via %s" % (i, cb[0], cb[1], gx, gy, ["%.2f %.2f" % q for q in route[:-1]]))
    cb[4] = 1
    # split long legs: the cube drifts on the blade, so re-measure every MAX_LEG
    pts = []
    px, py = cb[0], cb[1]
    for (wx, wy) in route:
        L = math.sqrt((wx - px) ** 2 + (wy - py) ** 2)
        n = int(L / MAX_LEG)
        for k in range(1, n + 1):
            pts.append((px + (wx - px) * k / (n + 1), py + (wy - py) * k / (n + 1)))
        pts.append((wx, wy))
        px, py = wx, wy
    for (wx, wy) in pts[:-1]:
        if not push_leg(i, wx, wy, False):
            lose(i)
            return False
        back_up(OBSERVE - CONTACT)
        if out_of_time():
            return True
    for attempt in range(2):
        if not push_leg(i, gx, gy, True):
            lose(i)
            return False
        back_up(OBSERVE - CONTACT)
        m = look_at(cb[0], cb[1])
        log("  placed at %.3f %.3f, seen %s" % (cb[0], cb[1], m))
        if not m:
            break
        cb[0], cb[1] = m
        short = (gx - m[0]) * ux + (gy - m[1]) * uy
        if short < 0.012 or out_of_time():
            break
    cb[4] = 2
    return True


def leg_ok(ax, ay, bx, by, obs):
    """Can the robot push a cube from a to b? Returns the robot's start point or None."""
    L = math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)
    if L < 0.01:
        return (ax, ay)
    ux = (bx - ax) / L
    uy = (by - ay) / L
    px = ax - ux * (CONTACT + LEAD)
    py = ay - uy * (CONTACT + LEAD)
    if not inside(px, py, LIMIT_PUSH):
        return None
    # cube and robot sweep along the leg: keep clear of every other cube
    rx = bx - ux * CONTACT
    ry = by - uy * CONTACT
    for (ox, oy) in obs:
        if seg_dist(ax, ay, bx, by, ox, oy) < SWEEP_CUBE or seg_dist(px, py, rx, ry, ox, oy) < SWEEP_ROBOT:
            return None
    return (px, py)


def on_axis(cx, cy, slot):
    gx, gy, ux, uy = slot
    off = abs(-(cx - gx) * uy + (cy - gy) * ux)
    along = (cx - gx) * -ux + (cy - gy) * -uy
    return off < 0.015 and STAGE - 0.08 < along < STAGE + 0.15


def route_for(i, slot, obs):
    """(cost, cube waypoints ending on the slot) or None.

    Legs: [detour ->] stage -> slot. The detour leg gets cubes off a wall: a
    shallow push along it, angled inward just enough to keep the robot inside.
    """
    gx, gy, ux, uy = slot
    cx, cy = cubes[i][0], cubes[i][1]
    sx = gx - ux * STAGE
    sy = gy - uy * STAGE
    if leg_ok(sx, sy, gx, gy, obs) is None:
        return None
    tail = 0.25 + STAGE * 1.6
    if on_axis(cx, cy, slot):
        p = leg_ok(cx, cy, gx, gy, obs)
        if p and point_clear(p[0], p[1], obs, CLR, LIMIT_PUSH):
            return (math.sqrt((p[0] - x) ** 2 + (p[1] - y) ** 2) + tail, [(gx, gy)])
    p = leg_ok(cx, cy, sx, sy, obs)
    if p and point_clear(p[0], p[1], obs, CLR, LIMIT_PUSH):
        L = math.sqrt((sx - cx) ** 2 + (sy - cy) ** 2)
        return (math.sqrt((p[0] - x) ** 2 + (p[1] - y) ** 2) + L * 1.6 + 1.0 + tail, [(sx, sy), (gx, gy)])
    best = None
    bc = 99.0
    for k in range(24):
        a = k * math.pi / 12
        ca = math.cos(a)
        sa = math.sin(a)
        for dist in (0.22, 0.38):
            mx = cx + dist * ca
            my = cy + dist * sa
            if abs(mx) > 0.6 or abs(my) > 0.6:
                continue
            p = leg_ok(cx, cy, mx, my, obs)
            if p is None or not point_clear(p[0], p[1], obs, CLR, LIMIT_PUSH):
                continue
            if leg_ok(mx, my, sx, sy, obs) is None:
                continue
            c = math.sqrt((p[0] - x) ** 2 + (p[1] - y) ** 2) + (dist + math.sqrt((sx - mx) ** 2 + (sy - my) ** 2)) * 1.6 + 2.5 + tail
            if c < bc:
                bc = c
                best = (c, [(mx, my), (sx, sy), (gx, gy)])
    return best


# --- Strategy --------------------------------------------------
A = 0.039
SLOTS = [(A, A, -1.0, 0.0), (-A, A, 0.0, -1.0), (-A, -A, 1.0, 0.0), (A, -A, 0.0, 1.0)]
RINGS = [(0.135, -0.06, -1.0, 0.0), (0.06, 0.135, 0.0, -1.0), (-0.135, 0.06, 1.0, 0.0), (-0.06, -0.135, 0.0, 1.0)]
slot_used = [False] * 4
ring_used = [False] * 4


def choose():
    """Best (cube, target, is_slot) or None."""
    best = None
    bc = 99.0
    for pool, used, bonus in ((SLOTS, slot_used, 0.0), (RINGS, ring_used, 1.5)):
        for k in range(4):
            if used[k]:
                continue
            for i, cb in enumerate(cubes):
                if cb[4] != 0 or cb[3] < 3:
                    continue
                r = route_for(i, pool[k], obstacles(i))
                if r is not None and r[0] + bonus < bc:
                    bc = r[0] + bonus
                    best = (i, k, pool is SLOTS, r[1])
        if best is not None and pool is SLOTS:
            return best
    return best


def out_of_time():
    return elapsed() > TOTAL_MS - 1200


def finish():
    motors.set_speeds(0, 0)
    for _ in range(2):
        for k in range(6):
            leds.set(k, (0, 255, 0))
        leds.show()
        time.sleep_ms(300)
        leds.off()
        time.sleep_ms(300)
    log("done, green blink")


# --- Scan ------------------------------------------------------
def scan(steps=10):
    start = th
    for i in range(steps):
        turn_to(wrap(start + i * 2 * math.pi / steps), 0.05)
        stop(30)
        wait_frames(2)


def dump_map():
    for i, cb in enumerate(cubes):
        log("cube %d: %.3f %.3f w=%.0f n=%d s=%d" % (i, cb[0], cb[1], cb[2], cb[3], cb[4]))


def prune():
    """Keep the five best supported cubes (drop single noisy sightings)."""
    global cubes
    cubes = [cb for cb in cubes if cb[3] >= 2 or cb[4] != 0]
    if len(cubes) > 5:
        cubes.sort(key=lambda cb: -cb[2])
        cubes = cubes[:5]


def main():
    wait_for_start()
    global t0
    t0 = time.ticks_ms()
    show("Challenge 2", "scan")
    calibrate_gyro(250)
    scan()
    prune()
    log("pose %.3f %.3f %.1f walls %d frames %d" % (x, y, math.degrees(th), n_wall, frames_seen))
    dump_map()
    while not out_of_time():
        pick = choose()
        if pick is None:
            break
        i, k, is_slot, route = pick
        used = slot_used if is_slot else ring_used
        used[k] = True
        if not place(i, (SLOTS if is_slot else RINGS)[k], route):
            used[k] = False
        # leave the centre before planning the next cube
        log("pose %.3f %.3f %.1f walls %d" % (x, y, math.degrees(th), n_wall))
    dump_map()
    finish()


try:
    main()
finally:
    motors.off()

