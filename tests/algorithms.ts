// Every program in src/algorithms/ (the "Challenge 2 · Algorithmen" group in the program menu) plays one full
// Push & Pull round on the example layout: no Python error, the template rules kept, and the algorithms keep at
// least the points they scored when they were added. Deterministic per seed.
//   npx tsx tests/algorithms.ts
import assert from "node:assert/strict";
import { readdirSync, readFileSync } from "node:fs";
import { join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { DEFAULT_CUBES } from "../src/shared/push-pull.ts";
import { needsToF, RUN_START_S } from "../src/shared/runs.ts";
import { runMatch } from "./match.ts";

const dir = join(dirname(fileURLToPath(import.meta.url)), "../src/algorithms");
// Points on DEFAULT_CUBES, seed 7, measured when the algorithm was added. A drop is a regression.
const MIN_POINTS: Record<string, number> = { "nearest_cube.py": 3, "secret_algo.py": 5 };

for (const file of readdirSync(dir).filter((f) => f.endsWith(".py")).sort()) {
  const code = readFileSync(join(dir, file), "utf8");
  // Same rules as the team template, so the file can go 1:1 into team-green as challenge2.py.
  assert.match(code, /from spark_start import wait_for_start/, `${file}: keeps the template start block`);
  assert.match(code, /def main\(\):\n(?:\s*#.*\n)*\s+wait_for_start\(\)/, `${file}: wait_for_start() first in main()`);
  assert.match(code, /finally:\n\s+\S*motors\.off\(\)/, `${file}: motors.off() in finally`);
  assert.doesNotMatch(code, /__name__/, `${file}: main() runs at the top level`);
  assert.doesNotMatch(code, /math\.hypot\(/, `${file}: MicroPython has no math.hypot`);
  assert(needsToF(code), `${file}: header declares "# Sensors: tof"`);

  const r = await runMatch({
    programs: [code],
    names: [file],
    arena: "push-pull",
    seconds: RUN_START_S + 120.1,
    seed: 7,
    presses: [{ button: "A", t: RUN_START_S }],
    cubes: DEFAULT_CUBES.map((p) => ({ ...p })),
    tof: [true],
  });
  assert.equal(r.errors[0], undefined, `${file}: Python error\n${r.errors[0]}`);
  const score = r.match.scores[0] ?? 0;
  assert(score >= (MIN_POINTS[file] ?? 0), `${file}: ${score} points, expected at least ${MIN_POINTS[file] ?? 0}`);
  console.log(`ok   ${file}: ${score} points in ${r.match.t.toFixed(1)} s (${r.match.reason || "time limit"})`);
}
