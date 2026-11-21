// Challenge 2 calibration as the simulator plays it (crew procedure, 21.11.2026). Before the round the crew gives
// the robot three 10 s windows in the real arena: in the start square, facing the black wall, facing a cube. The
// crew places the robot and presses A once per window; after window 3 it sets the robot in the start square and
// the round starts as usual.
//
// In the simulator this is optional and OFF by default: most test runs don't need it. Off, the sim's
// spark_start module sets SIM_SKIP_CALIBRATION = True, so a program can skip its windows and use its built-in
// values; the real spark_start has no such attribute, so on the robot the windows always run.
//
// World and robot runtime both derive every time from calibrationPlan(t0), t0 = the moment the start would
// otherwise have been given (the first A): the robot sees the presses from its own clock (deterministic).
import type { Pose } from "./arenas.ts";
import { PUSH_PULL } from "./push-pull.ts";
import { ZUMO } from "./zumo-spec.ts";

export interface CalibrationSetup {
  /** scoop to the wall at the start of window 2, cm (the crew allows 1 … 10) */
  wallCm: number;
  /** scoop to the cube at the start of window 3, cm (1 … 10) */
  cubeCm: number;
}

export const CALIBRATION_DEFAULT: CalibrationSetup = { wallCm: 5, cubeCm: 5 };
export const CAL_WINDOW_S = 10;
/** the crew's time between the end of a window and the next press (placing the robot) */
export const CAL_GAP_S = 1;
/** from the first A to the start signal */
export const CAL_TOTAL_S = 3 * CAL_WINDOW_S + 3 * CAL_GAP_S;
/** the crew looks whether the robot stands still this long after a window ended */
export const CAL_STILL_CHECK_S = 0.25;

export function validCalibration(v: unknown): v is CalibrationSetup {
  const ok = (n: unknown) => typeof n === "number" && Number.isFinite(n) && n >= 1 && n <= 10;
  return !!v && typeof v === "object" && ok((v as CalibrationSetup).wallCm) && ok((v as CalibrationSetup).cubeCm);
}

export interface CalibrationPlan {
  /** A press of window 1, 2, 3 (s) */
  presses: number[];
  /** end of window 1, 2, 3 */
  ends: number[];
  /** robot placed for window 2, for window 3, back in the start square */
  places: number[];
  /** the round's start signal */
  start: number;
}

export function calibrationPlan(t0: number): CalibrationPlan {
  const step = CAL_WINDOW_S + CAL_GAP_S;
  const presses = [0, 1, 2].map((k) => t0 + k * step);
  const ends = presses.map((t) => t + CAL_WINDOW_S);
  return { presses, ends, places: ends.map((t) => t + CAL_GAP_S / 2), start: t0 + CAL_TOTAL_S };
}

export interface CalibrationPoses {
  /** window 2: facing the wall straight on */
  wall: Pose;
  /** window 3: facing the middle of the calibration cube */
  cubeRobot: Pose;
  cube: Pose;
}

const HALF = PUSH_PULL.size / 2;
const OFFSETS = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4, 0.5, -0.5];
const DIRS = [0, Math.PI / 2, Math.PI, -Math.PI / 2];
/** a corridor this far from every arena cube counts as free (the front fan is wide) */
const ENOUGH = 0.3;

function segmentDistance(px: number, py: number, ax: number, ay: number, bx: number, by: number): number {
  const vx = bx - ax;
  const vy = by - ay;
  const t = Math.max(0, Math.min(1, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)));
  return Math.hypot(px - ax - t * vx, py - ay - t * vy);
}

/**
 * Where the crew puts the robot (and the calibration cube) in windows 2 and 3: along a line towards a wall, with
 * 40 cm free behind the robot and 50 cm behind the cube, as far from the arena's cubes as possible (they stay
 * where they are for the round). Which wall the real crew uses is not published; this is an assumption.
 */
export function calibrationPoses(cubes: readonly Pose[], setup: CalibrationSetup): CalibrationPoses {
  const back = 0.4 + Math.abs(ZUMO.rear); // free space behind the robot's tail
  const wallRobot = HALF - ZUMO.front - setup.wallCm / 100;
  const cubeAt = HALF - 0.5 - PUSH_PULL.cube / 2;
  const cubeRobot = cubeAt - PUSH_PULL.cube / 2 - setup.cubeCm / 100 - ZUMO.front;
  const wall = freeLine(cubes, wallRobot - back);
  const cube = freeLine(cubes, cubeRobot - back);
  const at = (line: { yaw: number; o: number }, a: number): Pose => ({
    x: Math.cos(line.yaw) * a - Math.sin(line.yaw) * line.o,
    y: Math.sin(line.yaw) * a + Math.cos(line.yaw) * line.o,
    yaw: line.yaw,
  });
  return { wall: at(wall, wallRobot), cubeRobot: at(cube, cubeRobot), cube: at(cube, cubeAt) };
}

/** The line (heading towards a wall, sideways offset) whose corridor from the wall back to `to` is freest. */
function freeLine(cubes: readonly Pose[], to: number): { yaw: number; o: number } {
  let pick = { yaw: 0, o: 0, clear: -1 };
  for (const yaw of DIRS)
    for (const o of OFFSETS) {
      const ux = Math.cos(yaw);
      const uy = Math.sin(yaw);
      const clear = Math.min(ENOUGH, ...cubes.map((q) => segmentDistance(q.x, q.y, ux * HALF - uy * o, uy * HALF + ux * o, ux * to - uy * o, uy * to + ux * o)));
      if (clear > pick.clear + 1e-9) pick = { yaw, o, clear };
    }
  return pick;
}
