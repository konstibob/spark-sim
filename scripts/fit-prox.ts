// Fits PROX_RESPONSE (src/shared/push-pull.ts) to the crew's measurements (MEASURED_PROX): a robot facing the
// black wall / a cube straight on, every centimetre from the scoop, front sensor, fine levels. Rewrites the
// PROX_RESPONSE block in place and prints sim vs measured counts.
//
//   npx tsx scripts/fit-prox.ts
import loadMujoco from "mujoco";
import { readFileSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { arenaById, type Pose } from "../src/shared/arenas.ts";
import { FINE_LEVELS, MEASURED_PROX, PROX_RESPONSE, PUSH_PULL } from "../src/shared/push-pull.ts";
import { ZUMO } from "../src/shared/zumo-spec.ts";
import { World } from "../src/world/world.ts";

const mj = await loadMujoco();
const arena = arenaById("push-pull");
const T = World.calibrateProx(mj, arena);
const gap = ZUMO.front - ZUMO.proxSensors[1].pos[0]; // receiver behind the scoop's edge
const cm = MEASURED_PROX.wall.fine.map((_, i) => i);
// A count c with the levels L (ascending, n of them) means level × signal ≥ 1 for L[n−c] … L[n−1] only:
// signal in [1 / L[n−c], 1 / L[n−c−1]). FL and FR together span the union of their steps.
function band(levels: readonly number[], pair: readonly [number, number]): [number, number] {
  const n = levels.length;
  const lo = Math.min(...pair);
  const hi = Math.max(...pair);
  return [lo >= n ? 1 / levels[0] : lo === 0 ? 0 : 1 / levels[n - lo], hi >= n ? Infinity : 1 / levels[n - hi - 1]];
}
// Aim at the middle of the range both readings allow; where the default and the fine reading contradict each
// other (they do by one step at a few distances), follow the fine one.
function target(kind: "wall" | "cube", c: number): number {
  const m = MEASURED_PROX[kind];
  const fine = band(FINE_LEVELS, m.fine[c]);
  const both = band(ZUMO.proxLevels, m.default[c]);
  let [lo, hi] = [Math.max(fine[0], both[0]), Math.min(fine[1], both[1])];
  if (lo >= hi) [lo, hi] = fine;
  if (lo === 0) return hi / 1.5;
  if (hi === Infinity) return lo * 1.5;
  return Math.sqrt(lo * hi);
}

// the other cubes in far corners, out of the front sensor's view
const half = PUSH_PULL.size / 2;
const parked: Pose[] = [
  { x: -0.7, y: -0.7, yaw: 0 },
  { x: -0.7, y: 0.7, yaw: 0 },
  { x: 0.7, y: -0.7, yaw: 0 },
  { x: 0.7, y: 0.7, yaw: 0 },
];
const cubeX = -0.2;
const scenes = {
  wall: { cubes: [{ x: -0.7, y: 0, yaw: 0 }, ...parked], pose: (d: number): Pose => ({ x: half - ZUMO.front - d, y: 0, yaw: 0 }) },
  cube: { cubes: [{ x: cubeX, y: 0, yaw: 0 }, ...parked], pose: (d: number): Pose => ({ x: cubeX - PUSH_PULL.cube / 2 - ZUMO.front - d, y: 0, yaw: 0 }) },
};

PROX_RESPONSE.knots.splice(0, Infinity, ...cm.map((c) => c / 100 + gap));
for (const kind of ["wall", "cube"] as const) PROX_RESPONSE[kind].splice(0, Infinity, ...cm.map((c) => Math.log10(target(kind, c))));

const worlds = Object.fromEntries(
  (["wall", "cube"] as const).map((kind) => [kind, new World(mj, arena, [scenes[kind].pose(0.05)], { proxThreshold: T, cubes: scenes[kind].cubes })]),
) as Record<"wall" | "cube", World>;
const signal = (kind: "wall" | "cube", c: number) => {
  const w = worlds[kind];
  w.place(0, scenes[kind].pose(c / 100));
  const s = w.proxSignals(w.robots[0])[1];
  return (s[0] + s[1]) / 2 / T;
};

for (let iter = 0; iter < 60; iter++) {
  let worst = 0;
  for (const kind of ["wall", "cube"] as const) {
    const err = cm.map((c) => Math.log10(target(kind, c) / signal(kind, c)));
    err.forEach((e, i) => (PROX_RESPONSE[kind][i] += 0.8 * e));
    worst = Math.max(worst, ...err.map(Math.abs));
  }
  if (worst < 0.002) break;
}

const counts = (kind: "wall" | "cube", c: number, levels: readonly number[]) => {
  const w = worlds[kind];
  w.place(0, scenes[kind].pose(c / 100));
  const v = w.readProx(w.robots[0], false, levels);
  return `${v[2]}/${v[3]}`;
};
for (const kind of ["wall", "cube"] as const) {
  console.log(`${kind}: cm  sim default · measured  |  sim fine · measured`);
  for (const c of cm) {
    const m = MEASURED_PROX[kind];
    console.log(`  ${String(c).padStart(2)}  ${counts(kind, c, ZUMO.proxLevels)} · ${m.default[c].join("/")}  |  ${counts(kind, c, FINE_LEVELS)} · ${m.fine[c].join("/")}`);
  }
}
for (const w of Object.values(worlds)) w.dispose();

const fmt = (a: number[], digits: number) => `[${a.map((v) => v.toFixed(digits)).join(", ")}]`;
const file = fileURLToPath(new URL("../src/shared/push-pull.ts", import.meta.url));
const src = readFileSync(file, "utf8");
const block = `export const PROX_RESPONSE = {
  knots: ${fmt(PROX_RESPONSE.knots, 4)},
  wall: ${fmt(PROX_RESPONSE.wall, 3)},
  cube: ${fmt(PROX_RESPONSE.cube, 3)},
};`;
const next = src.replace(/export const PROX_RESPONSE = \{[\s\S]*?\n\};/, block);
if (next === src && !src.includes(block)) throw new Error("PROX_RESPONSE block not found");
writeFileSync(file, next);
console.log("PROX_RESPONSE written to src/shared/push-pull.ts");
process.exit(0);
