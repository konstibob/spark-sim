import { arenaById, jitteredStarts } from "../src/shared/arenas.ts";
import { blocksStart, cubeMasses, ZONE, cubePoints, cubesFromSeed, DEFAULT_CUBES, PUSH_PULL, validCubeLayout, type CubeView } from "../src/shared/push-pull.ts";
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
  check("score at boundaries", cubePoints(cube(0.055, 0)) === 3 && cubePoints(cube(0.056, 0)) === 1 && cubePoints(cube(0.175, 0)) === 1 && cubePoints(cube(0.176, 0)) === 0, "on the tape / past its outer edge / exactly half / under half");
  check("rotated cube area", cubePoints(cube(0.175, 0, Math.PI / 4)) === 1 && cubePoints(cube(0.176, 0, Math.PI / 4)) === 0 && cubePoints(cube(0.03, 0.03, Math.PI / 4)) === 3 && cubePoints(cube(0.05, 0.05, Math.PI / 4)) === 1, "rotation-aware clipping, not centre-only scoring");
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
  scoring.step(120.2, 119.9, [robot], packed);
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

  const threshold = World.calibrateProx(mj, arena);
  {
    const w = new World(mj, arena, [{ x: 0.65, y: 0.2, yaw: 0 }], { proxThreshold: threshold, autoPressA: 0 });
    w.robots[0].cmd = [3000, 3000]; w.advanceTo(2_000_000);
    check("black walls contain robot", !w.robots[0].out && w.snapshot(1).robots[0].pos[0] < 0.8, `x=${w.snapshot(1).robots[0].pos[0].toFixed(3)} m`);
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
}
