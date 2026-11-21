# spark_start.py - simulator stand-in for the Crew's start module
#
# Same API as https://makernight.spark-hpi.de/files/spark_start.py, so the team template
# (challenge2.py) runs unchanged:
#
#     from spark_start import wait_for_start
#     wait_for_start()
#
# On the robot, wait_for_start() returns at the Crew's infrared START (no delay), or 5 s after
# button A when practising. The simulator has no IR start light: its start signal (Start / key S,
# the bench and --press A@t) presses button A, so here button A stands for the Crew's START and
# the program goes right away, the moment the rules start the match clock.
#
# Simulator only: SIM_SKIP_CALIBRATION is True unless the simulation plays the crew's three calibration windows
# (Challenge 2, off by default so test runs stay short). A program can skip its windows when it is set:
#
#     import spark_start
#     skip = getattr(spark_start, "SIM_SKIP_CALIBRATION", False)
#
# The crew's spark_start.py has no such attribute, so on the robot getattr gives False and the windows run.
import time
import _spark_rt
from zumo_2040_robot import robot

SIM_SKIP_CALIBRATION = not _spark_rt.calibration_runs()

START_DELAY_MS = 0
PRACTICE_DELAY_MS = 5000


def _show(display, lines):
    if display is None:
        return
    display.fill(0)
    for i, line in enumerate(lines):
        display.text(line, 0, i * 12)
    display.show()


def wait_for_start(delay_ms=START_DELAY_MS, button=True, show=True,
                   practice_delay_ms=PRACTICE_DELAY_MS):
    """Waits for the start signal (button A in the simulator) and returns delay_ms after it."""
    button_a = robot.ButtonA()
    display = robot.Display() if show else None
    _show(display, ["Ready", "waiting for", "the start signal", "", "A = start (sim)"])
    # is_pressed, not check(): a press that is already held when we get here counts too
    while not button_a.is_pressed():
        time.sleep_ms(1)
    if delay_ms > 0:
        time.sleep_ms(delay_ms)
    _show(display, [])
