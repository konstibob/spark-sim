# Name: Challenge 2 · team template
# Strategy: The unchanged challenge2.py from spark-hpi/team-green: start block, drive 1 s, motors off.
#   Start here so that your file can be copied 1:1 into the team repo.
# Arena: push-pull
# ─────────────────────────────────────────────────────────────
#  ★ SPARK · Zumo 2040 · Challenge 2
# ─────────────────────────────────────────────────────────────
#  Your program for challenge 2 (see RULES.md).
#
#  Files: challenge1_front.py, challenge1_back.py, challenge2.py,
#  challenge3.py. At the event the crew picks the file in the
#  robot's menu and starts the robots with an infrared signal; they
#  move the moment it arrives. On your own robot: press button A,
#  the robot moves 5 s later.
#
#  Keep wait_for_start() as the first step of main(): that's how
#  the crew starts your robot.
#
#  The Zumo library is already installed on the robot: you only
#  upload your own .py files, nothing else.
#
#  Motor speeds range from -6000 (full reverse) to 6000 (full forward).
# ─────────────────────────────────────────────────────────────

from zumo_2040_robot import robot
import time

# --- Hardware ---
motors = robot.Motors()
display = robot.Display()
button_a = robot.ButtonA()
buzzer = robot.Buzzer()


def show(*rows):
    """Print up to a few lines on the OLED display."""
    display.fill(0)
    for i, r in enumerate(rows):
        display.text(r, 0, i * 12)
    display.show()


# --- Start: only the crew starts robots, all at the same moment ---
# At the event every robot has the crew's start module (spark_start.py):
# the robot moves the moment the crew's infrared start signal arrives,
# at the same time as the others. Without it (on your own robot) this
# falls back to button A: the robot moves 5 s later, once your hand is
# off it. Keep this block as it is.
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


def main():
    wait_for_start()
    show("Challenge 2", "Running...")

    # ─── YOUR CODE GOES HERE ───
    # Example: drive straight for 1 second, then stop.
    # motors.set_speeds(left, right)   values from -6000 to 6000
    motors.set_speeds(3000, 3000)
    time.sleep(1)
    motors.off()
    # ───────────────────────────


# The robot's menu starts this file with import, so the program must run
# at the top level. No "if __name__ == '__main__'": it would never run.
try:
    main()
finally:
    motors.off()  # safety: always switch the motors off
