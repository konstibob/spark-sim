import { arenaById, jitteredStarts } from "../src/shared/arenas.ts";
import { blocksStart, cubeMasses, ZONE, cubePoints, cubesFromSeed, DEFAULT_CUBES, FINE_LEVELS, MEASURED_LINE, MEASURED_PROX, PUSH_PULL, validCubeLayout, type CubeView } from "../src/shared/push-pull.ts";
import { calibrationPlan, calibrationPoses, CAL_TOTAL_S, CALIBRATION_DEFAULT } from "../src/shared/calibration.ts";
import { ZUMO } from "../src/shared/zumo-spec.ts";
import type { Pose } from "../src/shared/arenas.ts";
import { Rules, type RobotView } from "../src/world/rules.ts";
import { World, type Mujoco } from "../src/world/world.ts";

export function checkPushPull(mj: Mujoco, check: (name: string, ok: boolean, detail: string) => void) {
  const arena = arenaById("push-pull");
  const cube = (x: number, y: number, yaw = 0): CubeView => ({ pos: [x, y, 0.05], quat: [Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)] });
  const robot: RobotView = { x: 0, y: 0, z: 0.02, moving: false, out: false, name: "Zumo" };
  check("push-pull geometry", arena.shape.kind === "slab" && arena.shape.w === 1.6 && arena.maxRobots === 1 && arena.timeLimit === 120 && arena.cubes?.length === 5, "160 cm / five cubes / solo / 120 s");
  check("push-pull tape", arena.reflect(0.065, 0) < 0.1 && arena.reflect(0.155, 0) < 0.1 && arena.reflect(0.4, 0) > 0.8, "black outlines, light floor elsewhere");
  check("tape counts", Math.abs(ZONE.small - 0.17) < 1e-12 && Math.abs(ZONE.home - 0.35) < 1e-12, "12 + 2 × 2.5 cm and 30 + 2 × 2.5 cm");
  check("four cubes fit", [-0.03, 0.03].flatMap((x) => [-0.03, 0.03].map((y) => cubePoints(cube(x, y)))).every((n) => n === 3), "four 6 cm cubes within 17 cm");
  {
    const c = ZONE.small / 2 - 0.03;
    const five = [[-c, -c], [-c, c], [c, -c], [c, c]].map(([x, y]) => ({ x, y, yaw: 0 })).concat({ x: 0, y: 0, yaw: Math.PI / 4 });
    const views = five.map((p) => cube(p.x, p.y, p.yaw));
    check("five cubes fit in theory", validCubeLayout(five) && views.every((v) => cubePoints(v) === 3), "four in the corners, one turned 45° in the middle: 15 points");
  }
  check("score at boundaries", cubePoints(cube(0.055, 0)) === 3 && cubePoints(cube(0.056, 0)) === 1 && cubePoints(cube(0.145, 0)) === 1 && cubePoints(cube(0.146, 0)) === 0 && cubePoints(cube(0.175, 0)) === 0, "on the tape / past its outer edge / fully home / sticking out / half out");
  check("rotated cube corners", cubePoints(cube(0.13, 0, Math.PI / 4)) === 1 && cubePoints(cube(0.135, 0, Math.PI / 4)) === 0 && cubePoints(cube(0.03, 0.03, Math.PI / 4)) === 3 && cubePoints(cube(0.05, 0.05, Math.PI / 4)) === 1, "rotation-aware footprint, not centre-only scoring");
  check("map validation", validCubeLayout(DEFAULT_CUBES) && !validCubeLayout(DEFAULT_CUBES.map(() => ({ x: 0, y: 0, yaw: 0 }))) && !validCubeLayout([{ x: 2, y: 0, yaw: 0 }, ...DEFAULT_CUBES.slice(1)]), "reject overlaps and cubes outside walls");
  {
    const layouts = Array.from({ length: 300 }, (_, i) => cubesFromSeed(i + 1));
    const outsideHome = (p: { x: number; y: number }) => Math.max(Math.abs(p.x), Math.abs(p.y)) > PUSH_PULL.home / 2 + PUSH_PULL.tape;
    check("seeded cubes valid", layouts.every((l) => validCubeLayout(l) && !l.some(blocksStart) && l.every(outsideHome)), "300 seeds: five cubes, inside walls, no overlap, outside home");
    check("seeded cubes reproducible", JSON.stringify(cubesFromSeed(42)) === JSON.stringify(cubesFromSeed(42)) && JSON.stringify(cubesFromSeed(42)) !== JSON.stringify(cubesFromSeed(43)), "same seed = same layout, next seed differs");
    const near = layouts.flat().filter((p) => Math.max(Math.abs(p.x), Math.abs(p.y)) > PUSH_PULL.size / 2 - 0.06).length;
    check("seeded cubes reach the walls", near > 50, `${near} of 1500 cubes within 3 cm of a wall`);
  }
  {
    const draws = Array.from({ length: 4000 }, (_, i) => cubeMasses(i + 1));
    const heavy = draws.map((m) => m.filter((v) => v === PUSH_PULL.heavy).length);
    const two = heavy.filter((n) => n === 2).length / draws.length;
    const valid = draws.every((m, i) => m.length === 5 && m.every((v) => v === PUSH_PULL.heavy || v === PUSH_PULL.light) && (heavy[i] === 2 || heavy[i] === 3));
    const everyPosition = [0, 1, 2, 3, 4].every((k) => draws.some((m) => m[k] === PUSH_PULL.heavy) && draws.some((m) => m[k] === PUSH_PULL.light));
    check("cube masses per seed", valid && everyPosition && Math.abs(two - PUSH_PULL.twoHeavy) < 0.03 && JSON.stringify(cubeMasses(7)) === JSON.stringify(cubeMasses(7)), `2 heavy in ${(two * 100).toFixed(1)} % of 4000 seeds, else 3; any cube can be heavy`);
  }
  check("fixed referee start", jitteredStarts(arena, 1, 12)[0].x === 0 && jitteredStarts(arena, 1, 32)[0].yaw === 0, "no random movement outside the 12 cm square");
  const scoring = new Rules(arena, 1);
  scoring.signal(0.2);
  const packed = [-0.03, 0.03].flatMap((x) => [-0.03, 0.03].map((y) => cube(x, y))).concat(cube(0.12, 0));
  scoring.step(0.3, 0.1, [robot], packed);
  check("immediate A and 13 points", scoring.state.phase === "running" && Math.abs(scoring.state.t - 0.1) < 1e-8 && scoring.state.scores[0] === 13, "no sumo countdown; four small + one home");
  scoring.step(120.2, 119.9, [{ ...robot, x: 0.1 }], packed); // moved: one long step must not count as stuck
  scoring.step(121, 0.8, [robot], []);
  check("120 s ends and freezes score", scoring.state.ended && scoring.state.scores[0] === 13 && scoring.state.t <= 120.001, scoring.state.reason);
  const blink = new Rules(arena, 1); blink.signal(0);
  blink.step(1, 1, [{ ...robot, green: true }], [cube(0.1, 0)]);
  check("steady green does not finish", !blink.state.ended, "requires blinking");
  blink.step(1.25, 0.25, [robot]); blink.step(1.5, 0.25, [{ ...robot, green: true }]);
  check("green blinking finishes", blink.state.ended, blink.state.reason);
  const rescue = new Rules(arena, 1); rescue.signal(0); rescue.step(1, 1, [robot], packed); rescue.referee("rescue");
  check("rescue penalty", rescue.state.scores[0] === 12 && !rescue.state.ended, "first rescue −1");
  rescue.referee("rescue"); check("second rescue ends", rescue.state.ended && rescue.state.scores[0] === 12, rescue.state.reason);
  {
    // standing still: warning at 5 s, freed at 10 s (−1), stuck again ends the round
    const stuck = new Rules(arena, 1); stuck.signal(0);
    const evs: string[] = [];
    let freed = 0;
    for (let k = 1; k <= 2400 && !stuck.state.ended; k++) {
      evs.push(...stuck.step(k * 0.01, 0.01, [{ ...robot, yaw: 0 }], packed).map((e) => e.text));
      if (stuck.takeFree() !== null) freed = k * 0.01;
    }
    const warnAt = evs.findIndex((e) => e.includes("has not moved for 5 s"));
    check("stuck warning then free", warnAt >= 0 && Math.abs(freed - 10) < 0.02, `freed at ${freed.toFixed(2)} s`);
    check("stuck twice ends", stuck.state.ended && stuck.state.reason === "stuck a second time" && stuck.state.scores[0] === 12, `${stuck.state.reason}, t=${stuck.state.t.toFixed(2)} s, ${stuck.state.scores[0]} points`);
    // turning on the spot is not stuck
    const spin = new Rules(arena, 1); spin.signal(0);
    for (let k = 1; k <= 1500; k++) spin.step(k * 0.01, 0.01, [{ ...robot, yaw: k * 0.01 }], packed);
    check("spinning is not stuck", !spin.state.rescues && (spin.state.stuck ?? 0) < 1, `stuck ${(spin.state.stuck ?? 0).toFixed(2)} s`);
  }

  const threshold = World.calibrateProx(mj, arena);
  {
    const w = new World(mj, arena, [{ x: 0.65, y: 0.2, yaw: 0 }], { proxThreshold: threshold, autoPressA: 0 });
    w.robots[0].cmd = [3000, 3000]; w.advanceTo(2_000_000);
    check("black walls contain robot", !w.robots[0].out && w.snapshot(1).robots[0].pos[0] < 0.8, `x=${w.snapshot(1).robots[0].pos[0].toFixed(3)} m`);
    // still pushing against the wall 10 s after the start: the referee lifts it (line sensors see nothing) and
    // sets it down in the start square, facing the start direction
    w.robots[0].cmd = [0, 0];
    w.advanceTo(10_900_000);
    const air = w.snapshot(1).robots[0].pos;
    const line = w.readLine(w.robots[0], true);
    check("freed robot is carried", !!w.robots[0].held && Math.hypot(air[0], air[1]) < 0.01 && line.every((v) => v >= 1000), `at (${air[0].toFixed(3)}, ${air[1].toFixed(3)}) z=${air[2].toFixed(3)}, line ${line.join(",")}`);
    w.advanceTo(12_000_000);
    const p = w.snapshot(1).robots[0];
    {
      // a cube already lies in the start square: the referee sets the robot down next to it
      const layout = DEFAULT_CUBES.map((c) => ({ ...c }));
      layout[0] = { x: 0.02, y: 0, yaw: 0 };
      const busy = new World(mj, arena, [{ x: 0.5, y: 0.5, yaw: 0 }], { proxThreshold: threshold, cubes: layout, autoPressA: 0 });
      busy.free(0);
      busy.advanceTo(1_500_000);
      const q = busy.snapshot(1).robots[0].pos;
      const c = busy.snapshot(1).cubes![0].pos;
      check("freed next to a cube in the start square", Math.hypot(q[0] - c[0], q[1] - c[1]) > 0.1 && Math.hypot(q[0], q[1]) < 0.2, `robot (${q[0].toFixed(3)}, ${q[1].toFixed(3)}), cube (${c[0].toFixed(3)}, ${c[1].toFixed(3)})`);
      busy.dispose();
    }
    check("stuck at wall is freed into the start square", w.rules.state.rescues === 1 && w.rules.state.penalties === 1 && !w.robots[0].held && Math.hypot(p.pos[0], p.pos[1]) < 0.01 && Math.abs(p.yaw) < 1, `rescues ${w.rules.state.rescues}, at (${p.pos[0].toFixed(3)}, ${p.pos[1].toFixed(3)}) yaw ${p.yaw.toFixed(1)}°`);
    w.dispose();
  }
  {
    const layout = DEFAULT_CUBES.map((p) => ({ ...p })); layout[0] = { x: 0.2, y: 0, yaw: 0 };
    const w = new World(mj, arena, [{ x: 0, y: 0, yaw: 0 }], { proxThreshold: threshold, cubes: layout, autoPressA: 0 });
    const before = w.snapshot(1).cubes![0].pos[0];
    const reflective = w.readProx(w.robots[0], false);
    w.robots[0].cmd = [1800, 1800]; w.advanceTo(1_800_000);
    const after = w.snapshot(1).cubes![0].pos[0];
    check("physical cube pushing", after > before + 0.05, `${before.toFixed(3)} → ${after.toFixed(3)} m`);
    const modelMass = w.cubeBodies.map((b) => w.m.body_mass[b]);
    check("cube masses in the model", modelMass.every((v, i) => Math.abs(v - cubeMasses(1)[i]) < 1e-9) && w.snapshot(1).cubes!.every((c, i) => c.mass === modelMass[i]), modelMass.map((v) => `${Math.round(v * 1000)} g`).join(" "));
    check("reflective cube sensed", Math.max(...reflective) > 0, reflective.join(","));
    w.dispose();
    const wall = new World(mj, arena, [{ x: 0.58, y: 0.2, yaw: 0 }], { proxThreshold: threshold });
    const dark = wall.readProx(wall.robots[0], false);
    check("cube brighter than black wall", Math.max(reflective[2], reflective[3]) > Math.max(dark[2], dark[3]), `cube ${reflective[2]}/${reflective[3]}, wall ${dark[2]}/${dark[3]}`);
    wall.dispose();
  }

  checkMeasuredSensors(mj, check, threshold);
  checkCalibrationCrew(mj, check, threshold);
}

/** The fitted sensor models against the crew's stock Zumo (calibration_stock_zumo.csv) */
function checkMeasuredSensors(mj: Mujoco, check: (name: string, ok: boolean, detail: string) => void, threshold: number) {
  const arena = arenaById("push-pull");
  const parked: Pose[] = [{ x: -0.7, y: -0.7, yaw: 0 }, { x: -0.7, y: 0.7, yaw: 0 }, { x: 0.7, y: -0.7, yaw: 0 }, { x: 0.7, y: 0.7, yaw: 0 }];
  const scenes = {
    wall: { cubes: [{ x: -0.7, y: 0, yaw: 0 }, ...parked], pose: (cm: number): Pose => ({ x: PUSH_PULL.size / 2 - ZUMO.front - cm / 100, y: 0, yaw: 0 }) },
    cube: { cubes: [{ x: -0.2, y: 0, yaw: 0 }, ...parked], pose: (cm: number): Pose => ({ x: -0.2 - PUSH_PULL.cube / 2 - ZUMO.front - cm / 100, y: 0, yaw: 0 }) },
  };
  for (const kind of ["wall", "cube"] as const) {
    const w = new World(mj, arena, [scenes[kind].pose(5)], { proxThreshold: threshold, cubes: scenes[kind].cubes });
    let fineOff = 0;
    const defaults: number[] = [];
    MEASURED_PROX[kind].fine.forEach((m, cm) => {
      w.place(0, scenes[kind].pose(cm));
      const fine = w.readProx(w.robots[0], false, FINE_LEVELS);
      fineOff = Math.max(fineOff, Math.abs((fine[2] + fine[3]) / 2 - (m[0] + m[1]) / 2));
      defaults.push(Math.max(...w.readProx(w.robots[0], false).slice(2, 4)));
    });
    w.dispose();
    check(`${kind} fine levels as measured`, fineOff <= 1.5, `largest difference ${fineOff} counts over 0 … 24 cm`);
    if (kind === "wall") check("black wall never reads 6", Math.max(...defaults) <= 5 && defaults[0] === 0, `default counts ${defaults.join("")} (0 with the scoop touching)`);
    else check("cube reads 6 up to 22 cm", defaults.slice(0, 23).every((v) => v === 6), `default counts ${defaults.join("")}`);
  }
  {
    const w = new World(mj, arena, [{ x: 0.4, y: 0.35, yaw: 0 }], { seed: 3 });
    const mean = (n = 10) => { const s = [0, 0, 0, 0, 0]; for (let k = 0; k < n; k++) w.readLine(w.robots[0], true).forEach((v, i) => (s[i] += v / n)); return s.map(Math.round); };
    const floor = mean();
    w.place(0, { x: PUSH_PULL.small / 2 + PUSH_PULL.tape / 2 - ZUMO.lineSensors[0][0], y: 0, yaw: 0 });
    const tape = mean();
    w.dispose();
    const lo = Math.min(...MEASURED_LINE.floor) * 0.6, hi = Math.max(...MEASURED_LINE.floor) * 1.5;
    check("line sensors floor and tape as measured", floor.every((v) => v >= lo && v <= hi) && tape.every((v) => v >= Math.min(...MEASURED_LINE.tape) * 0.7), `floor ${floor.join(",")} (measured ${MEASURED_LINE.floor.join(",")}), tape ${tape.join(",")} (measured ${MEASURED_LINE.tape.join(",")})`);
  }
}

/** The crew's calibration windows (src/shared/calibration.ts) played by the world, without a program */
function checkCalibrationCrew(mj: Mujoco, check: (name: string, ok: boolean, detail: string) => void, threshold: number) {
  const arena = arenaById("push-pull");
  {
    let worst = Infinity;
    for (let seed = 1; seed <= 300; seed++) {
      const cubes = cubesFromSeed(seed);
      const p = calibrationPoses(cubes, CALIBRATION_DEFAULT);
      const inside = [p.wall, p.cubeRobot, p.cube].every((q) => Math.abs(q.x) < 0.8 && Math.abs(q.y) < 0.8);
      const clear = Math.min(...cubes.flatMap((c) => [p.wall, p.cubeRobot, p.cube].map((q) => Math.hypot(c.x - q.x, c.y - q.y))));
      worst = Math.min(worst, inside ? clear : -1);
    }
    check("calibration spots clear of the cubes", worst > 0.15, `300 seeds: robot and calibration cube at least ${(worst * 100).toFixed(0)} cm from every arena cube`);
  }
  const w = new World(mj, arena, [arena.starts[0]], { proxThreshold: threshold, autoPressA: 0.2, calibration: { wallCm: 3, cubeCm: 7 } });
  const plan = calibrationPlan(0.2);
  const poses = calibrationPoses(arena.cubes!, { wallCm: 3, cubeCm: 7 });
  const at = () => { const r = w.snapshot(0).robots[0]; return { x: r.pos[0], y: r.pos[1] }; };
  const near = (p: { x: number; y: number }, q: Pose) => Math.hypot(p.x - q.x, p.y - q.y) < 0.01;
  w.advanceTo(300_000);
  w.robots[0].cmd = [1500, 1500]; // driving in window 1 (after its A) is allowed
  w.advanceTo(9_000_000);
  w.robots[0].cmd = [0, 0];
  w.advanceTo(Math.round((plan.places[0] + 0.3) * 1e6));
  const wall = at();
  w.advanceTo(Math.round((plan.places[1] + 0.3) * 1e6));
  const cube = at();
  const calCube = w.snapshot(0).calCube;
  w.robots[0].cmd = [-800, -800]; // still driving when window 3 ends: the crew notices
  w.advanceTo(Math.round((plan.ends[2] + 0.3) * 1e6));
  w.robots[0].cmd = [0, 0];
  w.advanceTo(Math.round((plan.start - 0.1) * 1e6));
  const home = at();
  const waiting = w.rules.state.phase === "waiting" && w.snapshot(0).calCube === null;
  w.advanceTo(Math.round((plan.start + 0.5) * 1e6));
  const texts = w.events.map((e) => e.text);
  check("calibration: robot placed for each window", near(wall, poses.wall) && near(cube, poses.cubeRobot) && !!calCube && Math.hypot(calCube.pos[0] - poses.cube.x, calCube.pos[1] - poses.cube.y) < 0.01 && near(home, arena.starts[0]), `wall (${wall.x.toFixed(3)}, ${wall.y.toFixed(3)}), cube window (${cube.x.toFixed(3)}, ${cube.y.toFixed(3)}), back (${home.x.toFixed(3)}, ${home.y.toFixed(3)})`);
  check("calibration: start after the windows", waiting && w.rules.state.signal !== null && Math.abs(w.rules.state.signal - (0.2 + CAL_TOTAL_S)) < 1e-9 && w.rules.state.phase === "running", `start signal at ${w.rules.state.signal} s`);
  check("calibration: driving in a window is no false start", !texts.some((t) => /False start/.test(t)) && texts.some((t) => /window 3: the robot still drives/.test(t)), texts.filter((t) => /alibration|False/.test(t)).join(" | "));
  w.dispose();
}
