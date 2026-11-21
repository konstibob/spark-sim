// Many Push & Pull runs of one program in parallel child processes, with a score histogram: the command-line
// twin of the "Runs" grid in the app. Same base seed = same runs (seeds, cube layouts, noise) as in the grid.
//
//   npx tsx tests/runs.ts prog.py                       # 16 runs, base seed 1, each run its own cube layout
//   npx tsx tests/runs.ts prog.py --runs 50 --base-seed 7
//   npx tsx tests/runs.ts prog.py --map layout.json     # one fixed layout (Arena-Karte export), seeds vary the noise
//   npx tsx tests/runs.ts prog.py --calibrate 5,5       # the crew's calibration windows first (scoop to wall, cube in cm)
//   options: --seconds 120 (round length; the run stops 0.2 s start + this)  --jobs N  --json out.json
//            --tof (VL53L5CX on; automatic for programs whose header says "# Sensors: tof")
import { fork, type ChildProcess } from "node:child_process";
import { readFileSync, writeFileSync } from "node:fs";
import { cpus } from "node:os";
import { basename } from "node:path";
import { fileURLToPath } from "node:url";
import { validCubeLayout } from "../src/shared/push-pull.ts";
import { needsToF, RUN_START_S, runCubes, runSeed, shortReason, summarizeRuns, type RunResult } from "../src/shared/runs.ts";
import type { Pose } from "../src/shared/arenas.ts";
import { CALIBRATION_DEFAULT, validCalibration, type CalibrationSetup } from "../src/shared/calibration.ts";
import { runMatch } from "./match.ts";

interface Job {
  index: number;
  seed: number;
  file: string;
  seconds: number;
  cubes: Pose[];
  tof: boolean;
  calibration: CalibrationSetup | null;
}

if (process.argv.includes("--child")) {
  process.on("message", async (job: Job) => {
    const r = await runMatch({
      programs: [readFileSync(job.file, "utf8")],
      names: [basename(job.file, ".py")],
      arena: "push-pull",
      seconds: RUN_START_S + job.seconds + 0.1,
      seed: job.seed,
      presses: [{ button: "A", t: RUN_START_S }],
      cubes: job.cubes,
      tof: [job.tof],
      calibration: job.calibration,
    });
    const m = r.match;
    const res: RunResult = {
      index: job.index,
      seed: job.seed,
      score: m.scores[0] ?? 0,
      t: m.t,
      reason: m.reason || "time limit",
      small: (m.cubeScores ?? []).filter((v) => v === 3).length,
      home: (m.cubeScores ?? []).filter((v) => v === 1).length,
      penalties: m.penalties ?? 0,
      rescues: m.rescues ?? 0,
      error: r.errors[0],
    };
    process.send!(res);
  });
} else {
  await main();
}

async function main() {
  const args = process.argv.slice(2);
  const opt = (k: string, d: string) => {
    const i = args.indexOf(`--${k}`);
    return i >= 0 ? args[i + 1] : d;
  };
  const file = args.find((a, i) => a.endsWith(".py") && !args[i - 1]?.startsWith("--"));
  if (!file) {
    console.error("usage: npx tsx tests/runs.ts prog.py [--runs 16] [--base-seed 1] [--map layout.json] [--seconds 120] [--jobs N] [--json out.json] [--tof]");
    process.exit(2);
  }
  const runs = Number(opt("runs", "16"));
  const base = Number(opt("base-seed", "1"));
  const seconds = Number(opt("seconds", "120"));
  const jobsN = Number(opt("jobs", String(Math.max(1, Math.min(runs, cpus().length - 1)))));
  const mapFile = opt("map", "");
  const tof = args.includes("--tof") || needsToF(readFileSync(file, "utf8"));
  let calibration: CalibrationSetup | null = null;
  const calAt = args.indexOf("--calibrate");
  if (calAt >= 0) {
    const value = args[calAt + 1];
    const [wallCm, cubeCm = wallCm] = /^\d/.test(value ?? "") ? value.split(",").map(Number) : [CALIBRATION_DEFAULT.wallCm, CALIBRATION_DEFAULT.cubeCm];
    calibration = { wallCm, cubeCm };
    if (!validCalibration(calibration)) throw new Error("--calibrate wall,cube: scoop distances 1 … 10 cm");
  }
  let fixed: Pose[] | null = null;
  if (mapFile) {
    const map = JSON.parse(readFileSync(mapFile, "utf8"));
    if (map.arena !== "push-pull" || map.units !== "metres" || !validCubeLayout(map.cubes)) throw new Error("Invalid Push & Pull map; export a layout from Arena-Karte (metres).");
    fixed = map.cubes;
  }
  const jobs: Job[] = Array.from({ length: runs }, (_, index) => {
    const seed = runSeed(base, index);
    return { index, seed, file, seconds, cubes: runCubes(seed, fixed), tof, calibration };
  });
  console.error(`${runs} runs of ${basename(file)} · base seed ${base} · ${fixed ? `layout from ${mapFile}` : "own cube layout per run"} · max ${seconds} s each${calibration ? ` · calibration windows first (wall ${calibration.wallCm} cm, cube ${calibration.cubeCm} cm)` : ""} · ${jobsN} processes${tof ? " · VL53L5CX on" : ""}`);
  const t0 = performance.now();
  const results = await runPool(jobs, jobsN);
  console.error(`done in ${((performance.now() - t0) / 1000).toFixed(0)} s\n`);

  console.log("run  seed        points  small  home  pen  time     end");
  for (const r of results)
    console.log(
      `${String(r.index + 1).padStart(3)}  ${String(r.seed).padEnd(10)}  ${String(r.score).padStart(6)}  ${String(r.small).padStart(5)}  ${String(r.home).padStart(4)}  ${String(r.penalties).padStart(3)}  ${r.t.toFixed(1).padStart(6)} s  ${shortReason(r.reason)}${r.error ? `  ERROR ${r.error.split("\n").pop()}` : ""}`,
    );

  const s = summarizeRuns(results);
  const most = Math.max(1, ...s.histogram.map(([, c]) => c));
  console.log("\npoints  runs");
  for (const [score, count] of [...s.histogram].reverse())
    console.log(`${String(score).padStart(6)}  ${"█".repeat(Math.round((count / most) * 40))}${count ? ` ${count}` : ""}`);
  console.log(`\nmean ${s.mean.toFixed(2)} · median ${s.median} · best ${s.best?.score} (run ${(s.best?.index ?? 0) + 1}, seed ${s.best?.seed}) · worst ${s.worst?.score} (run ${(s.worst?.index ?? 0) + 1}, seed ${s.worst?.seed})`);
  console.log(`end: ${s.reasons.map(([r, c]) => `${shortReason(r)} ${c}`).join(" · ")}`);
  const errors = results.filter((r) => r.error).length;
  if (errors) console.log(`${errors} run(s) with a Python error`);
  const json = opt("json", "");
  if (json) writeFileSync(json, JSON.stringify({ file, base, runs, seconds, map: mapFile || null, results, summary: s }, null, 1));
}

function runPool(jobs: Job[], n: number): Promise<RunResult[]> {
  const queue = [...jobs];
  const results: RunResult[] = [];
  const self = fileURLToPath(import.meta.url);
  return new Promise((resolve) => {
    let running = 0;
    const startChild = () => {
      // a fresh process every few runs keeps memory flat (each run loads its own MuJoCo + MicroPython)
      const child: ChildProcess = fork(self, ["--child"], { stdio: ["ignore", "ignore", "inherit", "ipc"] });
      let done = 0;
      let current: Job | null = null;
      let retired = false;
      const next = () => {
        current = null;
        if (!queue.length || done >= 10) {
          retired = true;
          child.kill();
          running--;
          if (queue.length) {
            running++;
            startChild();
          } else if (running === 0) resolve(results.sort((a, b) => a.index - b.index));
          return;
        }
        current = queue.shift()!;
        child.send(current);
      };
      const record = (r: RunResult) => {
        results.push(r);
        done++;
        process.stderr.write(`\r  ${results.length}/${jobs.length}`);
        if (results.length === jobs.length) process.stderr.write("\n");
      };
      child.on("message", (r: RunResult) => {
        record(r);
        next();
      });
      // a crashed child: count its run as an error instead of waiting forever
      child.on("exit", (code) => {
        if (retired) return;
        if (current) record({ index: current.index, seed: current.seed, score: 0, t: 0, reason: "crashed", small: 0, home: 0, penalties: 0, rescues: 0, error: `child exited with ${code}` });
        retired = true;
        running--;
        if (queue.length) {
          running++;
          startChild();
        } else if (running === 0) resolve(results.sort((a, b) => a.index - b.index));
      });
      next();
    };
    for (let i = 0; i < Math.min(n, jobs.length); i++) {
      running++;
      startChild();
    }
  });
}
