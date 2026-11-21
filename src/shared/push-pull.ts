// Challenge 2 uses metres, with the origin at the centre of the start square.
import type { Pose } from "./arenas.ts";

export const PUSH_PULL = {
  size: 1.6,
  wallHeight: 0.1,
  wallThickness: 0.015,
  small: 0.12,
  home: 0.3,
  cube: 0.06,
  tape: 0.025, // measured; the tape lies outside small/home and counts as part of the square
  heavy: 0.14, // weighed: the heavy cube
  light: 0.04, // weighed: the light cube
  /** assumed: how likely the referee uses 2 heavy + 3 light cubes; otherwise 3 heavy + 2 light */
  twoHeavy: 0.75,
  /** EST: the ESP32-C6 Pico board mounted on the Zumo, on top of ZUMO.mass (weigh the whole robot) */
  robotExtra: 0.008,
  friction: 0.35, // assumed
} as const;

/** Example positions, NOT the unknown referee layout. Same layout for every seed. */
export const DEFAULT_CUBES: readonly Pose[] = [
  { x: 0.43, y: 0, yaw: 0 },
  { x: 0.48, y: 0.49, yaw: 0 },
  { x: -0.39, y: 0.46, yaw: 0 },
  { x: -0.52, y: -0.3, yaw: 0 },
  { x: 0.24, y: -0.53, yaw: 0 },
];

export interface CubeView {
  pos: [number, number, number];
  quat: [number, number, number, number]; // MuJoCo: w, x, y, z
  /** kg */
  mass?: number;
}

/** Uniform [0, 1) numbers, reproducible per seed. */
function seeded(seed: number) {
  let s = seed >>> 0 || 1;
  return () => {
    s = (Math.imul(s ^ (s >>> 15), 0x2c1b3c6d) + 0x6d2b79f5) >>> 0;
    return (s >>> 8) / 16777216;
  };
}

/**
 * Masses of the five cubes for one match: 2 heavy + 3 light (PUSH_PULL.twoHeavy), else 3 heavy + 2 light,
 * in random order. Reproducible per match seed, independent of the layout (a hand-placed map gets them too).
 */
export function cubeMasses(seed: number): number[] {
  const rnd = seeded(Math.imul(seed, 0x9e3779b1) ^ 0x5bd1e995);
  const heavy = rnd() < PUSH_PULL.twoHeavy ? 2 : 3;
  const out = [0, 1, 2, 3, 4].map((i) => (i < heavy ? PUSH_PULL.heavy : PUSH_PULL.light));
  for (let i = out.length - 1; i > 0; i--) {
    const j = Math.floor(rnd() * (i + 1));
    [out[i], out[j]] = [out[j], out[i]];
  }
  return out;
}

export function validCubeLayout(value: unknown): value is Pose[] {
  if (!Array.isArray(value) || value.length !== 5) return false;
  const limit = PUSH_PULL.size / 2;
  return value.every((p) => {
    if (!p || ![p.x, p.y, p.yaw].every((v) => typeof v === "number" && Number.isFinite(v))) return false;
    const r = PUSH_PULL.cube / 2 * (Math.abs(Math.cos(p.yaw)) + Math.abs(Math.sin(p.yaw)));
    return Math.abs(p.x) + r <= limit && Math.abs(p.y) + r <= limit;
  }) && value.every((p, i) => value.slice(i + 1).every((q) => {
    // Separating axis test for the two rotated square footprints. Touching edges are legal.
    const r = PUSH_PULL.cube / 2;
    return [p.yaw, p.yaw + Math.PI / 2, q.yaw, q.yaw + Math.PI / 2].some((a) => {
      const radius = (yaw: number) => r * (Math.abs(Math.cos(yaw - a)) + Math.abs(Math.sin(yaw - a)));
      return Math.abs((p.x - q.x) * Math.cos(a) + (p.y - q.y) * Math.sin(a)) >= radius(p.yaw) + radius(q.yaw) - 1e-8;
    });
  }));
}

/** The robot stands in the start square: a cube there would make the start impossible. */
export function blocksStart(p: Pose): boolean {
  return cubeFootprint({ pos: [p.x, p.y, 0.05], quat: [Math.cos(p.yaw / 2), 0, 0, Math.sin(p.yaw / 2)] }).some(([x, y]) => Math.abs(x) < 0.075 && Math.abs(y) < 0.065);
}

/**
 * A random but reproducible layout: same seed, same five cubes (browser, bench and CLI).
 * Cubes start outside the 30 cm home square (its tape included), may touch walls and sit in corners,
 * and keep at least 1 cm to each other. Yaw is free (a square repeats every 90°).
 */
export function cubesFromSeed(seed: number): Pose[] {
  const rnd = seeded(seed);
  const P = PUSH_PULL;
  const r = P.cube / 2;
  const gap = 0.01;
  const out: Pose[] = [];
  for (let tries = 0; out.length < 5 && tries < 10_000; tries++) {
    const yaw = rnd() * Math.PI / 2;
    const reach = r * (Math.cos(yaw) + Math.sin(yaw)); // half the footprint's bounding box
    const limit = P.size / 2 - reach;
    const p = { x: (rnd() * 2 - 1) * limit, y: (rnd() * 2 - 1) * limit, yaw };
    if (Math.max(Math.abs(p.x), Math.abs(p.y)) < P.home / 2 + P.tape + reach + gap) continue;
    if (out.some((q) => Math.hypot(p.x - q.x, p.y - q.y) < P.cube * Math.SQRT2 + gap)) continue;
    out.push(p);
  }
  return out;
}

type Point = [number, number];
const cross = (a: Point, b: Point, c: Point) => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);

/** Convex projection of all eight corners: also works when a cube tips over. */
export function cubeFootprint(cube: CubeView): Point[] {
  const [w, x, y, z] = cube.quat;
  const r = PUSH_PULL.cube / 2;
  const pts: Point[] = [];
  for (const dx of [-r, r]) for (const dy of [-r, r]) for (const dz of [-r, r]) pts.push([
    cube.pos[0] + (1 - 2 * (y * y + z * z)) * dx + 2 * (x * y - z * w) * dy + 2 * (x * z + y * w) * dz,
    cube.pos[1] + 2 * (x * y + z * w) * dx + (1 - 2 * (x * x + z * z)) * dy + 2 * (y * z - x * w) * dz,
  ]);
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const half = (list: Point[]) => {
    const hull: Point[] = [];
    for (const p of list) {
      while (hull.length >= 2 && cross(hull[hull.length - 2], hull[hull.length - 1], p) <= 0) hull.pop();
      hull.push(p);
    }
    return hull.slice(0, -1);
  };
  return [...half(pts), ...half([...pts].reverse())];
}

const area = (pts: Point[]) => Math.abs(pts.reduce((sum, p, i) => {
  const q = pts[(i + 1) % pts.length];
  return sum + p[0] * q[1] - p[1] * q[0];
}, 0)) / 2;

/** Scoring zones: the inner size plus the tape on both sides (a cube on the tape still counts). */
export const ZONE = { small: PUSH_PULL.small + 2 * PUSH_PULL.tape, home: PUSH_PULL.home + 2 * PUSH_PULL.tape } as const;

export function cubePoints(cube: CubeView): 0 | 1 | 3 {
  const footprint = cubeFootprint(cube);
  // A cube lifted off the board / stacked above another is not lying in the square.
  if (cube.pos[2] > 0.02 + PUSH_PULL.cube || cube.pos[2] < 0) return 0;
  if (footprint.every(([x, y]) => Math.abs(x) <= ZONE.small / 2 + 1e-7 && Math.abs(y) <= ZONE.small / 2 + 1e-7)) return 3;
  let clipped = footprint;
  for (const axis of [0, 1]) for (const sign of [-1, 1]) {
    const input = clipped;
    clipped = [];
    input.forEach((a, i) => {
      const b = input[(i + 1) % input.length];
      const da = ZONE.home / 2 - sign * a[axis];
      const db = ZONE.home / 2 - sign * b[axis];
      if (da >= 0) clipped.push(a);
      if ((da >= 0) !== (db >= 0)) {
        const t = da / (da - db);
        clipped.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
      }
    });
  }
  return area(clipped) + 1e-10 >= area(footprint) / 2 ? 1 : 0;
}
