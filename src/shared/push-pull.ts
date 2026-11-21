// Challenge 2 uses metres, with the origin at the centre of the start square.
import type { Pose } from "./arenas.ts";

export const PUSH_PULL = {
  size: 1.6,
  wallHeight: 0.1,
  wallThickness: 0.015,
  small: 0.12,
  home: 0.3,
  cube: 0.06,
  tape: 0.01, // assumed: rules do not specify the tape width
  mass: 0.06, // assumed: tune against the real cubes
  friction: 0.35,
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

export function cubePoints(cube: CubeView): 0 | 1 | 3 {
  const footprint = cubeFootprint(cube);
  // A cube lifted off the board / stacked above another is not lying in the square.
  if (cube.pos[2] > 0.02 + PUSH_PULL.cube || cube.pos[2] < 0) return 0;
  if (footprint.every(([x, y]) => Math.abs(x) <= PUSH_PULL.small / 2 + 1e-7 && Math.abs(y) <= PUSH_PULL.small / 2 + 1e-7)) return 3;
  let clipped = footprint;
  for (const axis of [0, 1]) for (const sign of [-1, 1]) {
    const input = clipped;
    clipped = [];
    input.forEach((a, i) => {
      const b = input[(i + 1) % input.length];
      const da = PUSH_PULL.home / 2 - sign * a[axis];
      const db = PUSH_PULL.home / 2 - sign * b[axis];
      if (da >= 0) clipped.push(a);
      if ((da >= 0) !== (db >= 0)) {
        const t = da / (da - db);
        clipped.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
      }
    });
  }
  return area(clipped) + 1e-10 >= area(footprint) / 2 ? 1 : 0;
}
