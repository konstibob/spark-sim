# Name: VL53L5CX · ESP serial
# Strategy: Read the ESP32-C6 depth stream while turning slowly to look around.
# Arena: free
# Tests: Enable VL53L5CX above the 3D view, then Run. Distances are in mm.
from spark_tof import ToFSerial
from zumo_2040_robot import robot
import time

sensor = ToFSerial()  # UART0, 115200 baud, proposed JSONL protocol
motors = robot.Motors()

try:
    motors.set_speeds(-700, 700)
    while True:
        frame = sensor.read()
        if frame is not None:
            valid = [d for d, status, count in zip(
                frame["distance_mm"], frame["target_status"], frame["nb_target_detected"]
            ) if status == 5 and count > 0]
            if valid:
                print("nearest_mm:", min(valid))
            else:
                print("No valid target")
        time.sleep_ms(5)
finally:
    motors.off()
