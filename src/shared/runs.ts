// Many solo runs of one program at once (the "Runs" grid in the app and tests/runs.ts on the command line).
// Both derive the same seeds and cube layouts from a base seed, so a run in the grid and the same run on the
// command line are the same deterministic match.
import type { Pose } from "./arenas.ts";
import { cubesFromSeed } from "./push-pull.ts";

/** The program's header asks for the VL53L5CX ("# Sensors: tof"): runs switch it on by themselves. */
export function needsToF(code: string): boolean {
  for (const line of code.split("\n")) {
    if (!line.startsWith("#")) break;
    const m = line.match(/^#\s*Sensors:\s*(.*)$/i);
    if (m) return /\btof\b/i.test(m[1]);
  }
  return false;
}

/** Grid sizes the app offers: 1, 1×2, 2×2, 2×3, 3×3, 4×4 */
export const RUN_COUNTS = [1, 2, 4, 6, 9, 16] as const;

/** The start signal (button A) of every run, like "Auto start" and the bench */
export const RUN_START_S = 0.2;

/** Seed of run i: the same formula as the test bench, so seeds line up there too */
export function runSeed(baseSeed: number, index: number): number {
  return (baseSeed + index * 7919) >>> 0;
}

/** Cube layout of a run: its own layout from its seed, or the given fixed layout (Arena-Karte / --map) */
export function runCubes(seed: number, fixed?: Pose[] | null): Pose[] {
  return fixed ? fixed.map((p) => ({ ...p })) : cubesFromSeed(seed);
}

export interface RunResult {
  index: number;
  seed: number;
  score: number;
  /** match clock when it ended (s since the start signal) */
  t: number;
  reason: string;
  small: number;
  home: number;
  penalties: number;
  rescues: number;
  error?: string;
}

export interface RunSummary {
  n: number;
  mean: number;
  median: number;
  best: RunResult | null;
  worst: RunResult | null;
  /** [score, how many runs] for every score from the lowest to the highest, gaps included */
  histogram: [number, number][];
  /** end reason → how many runs */
  reasons: [string, number][];
}

export function summarizeRuns(results: RunResult[]): RunSummary {
  const n = results.length;
  const scores = results.map((r) => r.score).sort((a, b) => a - b);
  const mean = n ? scores.reduce((s, v) => s + v, 0) / n : 0;
  const median = n ? (n % 2 ? scores[(n - 1) / 2] : (scores[n / 2 - 1] + scores[n / 2]) / 2) : 0;
  // best: most points, then the shorter time (the official tie-break); worst: fewest points
  const ranked = [...results].sort((a, b) => b.score - a.score || a.t - b.t);
  const histogram: [number, number][] = [];
  if (n) for (let s = Math.min(0, scores[0]); s <= Math.max(0, scores[n - 1]); s++) histogram.push([s, scores.filter((v) => v === s).length]);
  const reasons = new Map<string, number>();
  for (const r of results) reasons.set(r.reason, (reasons.get(r.reason) ?? 0) + 1);
  return {
    n,
    mean,
    median,
    best: ranked[0] ?? null,
    worst: ranked[n - 1] ?? null,
    histogram,
    reasons: [...reasons].sort((a, b) => b[1] - a[1]),
  };
}

/** Short label of an end reason for charts and tiles */
export function shortReason(reason: string): string {
  if (/green/.test(reason)) return "green blink";
  if (/second time/.test(reason)) return "stuck twice";
  if (/left the arena/.test(reason)) return "left arena";
  if (/touched/.test(reason)) return "touched";
  if (/time limit|^$/.test(reason)) return "120 s";
  return reason;
}
