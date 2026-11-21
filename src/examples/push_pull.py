# Name: Push & Pull · one-cube demo
# Strategy: Demonstrates going behind cube 1 and pushing it into the 30 cm home square.
# Strengths / weaknesses: Uses the example map's known coordinates, not autonomous cube search.
#   After editing the map, adapt the waypoints. Odometry drifts: calibrate on the real robot.
# Arena: push-pull
from zumo_2040_robot import robot
import math
import time

motors = robot.Motors()
encoders = robot.Encoders()
button = robot.ButtonA()
leds = robot.RGBLEDs()
display = robot.Display()
imu = robot.IMU()
imu.reset()
imu.enable_default()

# Zumo 2040: approximate encoder counts per metre.
COUNTS_PER_M = 8467
x, y, heading = 0.0, 0.0, 0.0
last_l, last_r = encoders.get_counts()
last_t = time.ticks_us()
last_gyro = 0.0

def update():
    global x, y, heading, last_l, last_r, last_t, last_gyro
    imu.read()
    now = time.ticks_us()
    dt = time.ticks_diff(now, last_t) / 1000000
    last_t = now
    l, r = encoders.get_counts()
    dl, dr = (l - last_l) / COUNTS_PER_M, (r - last_r) / COUNTS_PER_M
    last_l, last_r = l, r
    # Gyro heading avoids the large encoder error from skid-steer turns.
    gyro = imu.gyro.last_reading_dps[2]
    turn = (gyro + last_gyro) / 2 * math.pi / 180 * dt
    last_gyro = gyro
    distance = (dl + dr) / 2
    x += distance * math.cos(heading + turn / 2)
    y += distance * math.sin(heading + turn / 2)
    heading += turn

def angle_error(target):
    return (target - heading + math.pi) % (2 * math.pi) - math.pi

def settle():
    # Keep measuring while braking: a single reading after a long sleep misses the turn.
    for sample in range(15):
        time.sleep_ms(10)
        update()

def face(angle):
    deadline = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), deadline) < 6000:
        update()
        error = angle_error(angle)
        if abs(error) < 0.035:
            break
        power = min(2200, max(1600, int(abs(error) * 1900)))
        if error < 0:
            power = -power
        motors.set_speeds(-power, power)
        time.sleep_ms(10)
    motors.off()
    settle()

def go(tx, ty):
    face(math.atan2(ty - y, tx - x))
    deadline = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), deadline) < 12000:
        update()
        distance = math.sqrt((tx - x) ** 2 + (ty - y) ** 2)
        if distance < 0.012:
            break
        correction = int(angle_error(math.atan2(ty - y, tx - x)) * 1600)
        power = min(2200, max(1600, int(distance * 7000)))
        motors.set_speeds(power - correction, power + correction)
        time.sleep_ms(10)
    motors.off()
    settle()
    print("odometry", round(x, 3), round(y, 3))

display.fill(0)
display.text("Push & Pull", 0, 0)
display.text("A: start", 0, 16)
display.text("Example map", 0, 32)
display.show()
while not button.check():
    time.sleep_ms(10)

# Drive around the cube, line up behind it, then push towards home.
# Cube 1 starts at x=43 cm, y=0. All distances are metres.
go(0.0, -0.19)
go(0.57, -0.19)
go(0.57, 0.0)
face(math.pi)
go(0.17, 0.0)
motors.off()
print("Demo finished; check the cube's actual position and score.")

# Green blinking ends the round. No button stops the program.
for flash in range(4):
    for i in range(6):
        leds.set(i, [0, 60, 0])
    leds.show()
    time.sleep_ms(250)
    leds.off()
    time.sleep_ms(250)
