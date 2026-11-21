# Name: Mapping · einfacher Testlauf
# Strategy: A fixed, editable sequence: forwards, turn, backwards. No autonomous exploration.
# Strengths / weaknesses: Only this selected sample sends motor commands. The map observes sensors independently.
#   Encoder/gyro pose and ToF map update while moving; no map-based position correction yet.
# Tests: Load Testskript in the robot view, Run, then A. Needs VL53L5CX enabled.
# Arena: push-pull
from zumo_2040_robot import robot
from spark_tof import ToFSerial
from spark_mapping import Odometry, DiscoveryMap
import time

motors = robot.Motors()
encoders = robot.Encoders()
imu = robot.IMU()
imu.reset()
imu.enable_default()
sensor = ToFSerial()
pose = Odometry()
discovery = DiscoveryMap(pose)
button = robot.ButtonA()

# ESP capture clock -> Zumo clock. Both clocks are aligned in the simulator.
# Synchronize the real clocks / set this offset before mapping during hardware motion.
ESP_CLOCK_OFFSET_US = 0

# Edit only this sequence to try a different motion. Speeds: -6000..6000.
# Nothing here chooses a direction or route based on the discovered map.
# left, right, milliseconds, label
MOVES = [
    (1800, 1800, 1000, 'Geradeaus'),
    (0, 0, 500, 'Pause'),
    (-1800, 1800, 650, 'Links drehen'),
    (1800, 1800, 900, 'Geradeaus'),
    (-1800, -1800, 900, 'Zurueck'),
    (1800, -1800, 650, 'Rechts drehen'),
    (-1800, -1800, 1000, 'Zurueck'),
    (0, 0, 800, 'Stillstand'),
]

last_clock = time.ticks_us()
elapsed = last_clock
last_publish = -1000000
latest = None


def sample(state, mapping):
    global elapsed, last_clock, last_publish, latest
    imu.gyro.read()
    left, right = encoders.get_counts()
    tick = time.ticks_us()
    elapsed += time.ticks_diff(tick, last_clock)
    last_clock = tick
    pose.update(left, right, imu.gyro.last_reading_dps[2], elapsed)
    # Always drain serial, including while waiting. Only actual measurements enter the map.
    for _ in range(4):
        frame = sensor.read()
        if frame is None:
            break
        latest = frame
    if mapping and latest is not None:
        captured = latest['t_us'] + ESP_CLOCK_OFFSET_US
        capture_pose = pose.at(captured)
        if capture_pose is not None:
            discovery.observe(latest, captured, capture_pose)
    if elapsed - last_publish >= 250000:
        discovery.publish(elapsed, state)
        last_publish = elapsed
    time.sleep_ms(10)


try:
    motors.off()
    bias = 0.0
    requested_start = False
    for _ in range(80):
        imu.gyro.read()
        bias += imu.gyro.last_reading_dps[2]
        requested_start = requested_start or button.check()
        time.sleep_ms(5)
    pose.gyro_bias = bias / 80
    # Emit the empty initial map, even if A arrived during calibration.
    sample('Warte auf A', False)
    while not requested_start and not button.check():
        sample('Warte auf A', False)
    for left, right, duration_ms, label in MOVES:
        motors.set_speeds(left, right)
        end = elapsed + duration_ms * 1000
        while elapsed < end:
            sample(label, True)
    motors.off()
    discovery.publish(elapsed, 'Test beendet')
finally:
    motors.off()
