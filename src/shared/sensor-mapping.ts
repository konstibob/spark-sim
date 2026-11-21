import type { DiscoveryPacket } from "./discovery.ts";
import { tofSlope, VL53L5CX, type ToFFrame } from "./vl53l5cx.ts";

type Pose = [number, number, number];
type Ray = [number, number, number, number];
/** Deliberately excludes simulator capture origin/rotation and object identities. */
export type MappingFrame = Pick<ToFFrame, "capturedAtUs" | "sequence" | "distanceMm" | "targetStatus" | "targetsDetected">;
const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

/** Prior knowledge from the rules, not a measurement: every cube is 6 × 6 × 6 cm, walls are long and 10 cm high. */
export const CUBE = 0.06;
/** A return this high at the face's depth belongs to a wall; cube hits stay below ~7.5 cm from the 9 cm mount. */
const CUBE_TALL = 0.08;
/** Depth step between neighbouring columns that still belongs to the same face. */
const CUBE_STEP = 0.04;
/** Neighbour columns must be this much farther (or empty): a cube stands free, a wall continues. */
const CUBE_GAP = 0.05;
/** A sighting this close to a known cube is the same cube. */
const CUBE_JOIN = 0.09;
/** Averaging weight cap, so a pushed cube can follow its new sightings. */
const CUBE_MEMORY = 8;
/** Five cubes in the arena; room for a few stray sightings on top. */
const CUBE_MAX = 16;

/** One column of a ToF frame: nearest robot-high return, its lateral slope, and whether a wall-high return backs it. */
export type ToFColumn = { depth: number; tan: number; tall: boolean } | null;
/** Cube centre in the robot frame (x forward from the robot origin, y left) and the columns it covers. */
export interface CubeSighting { x: number; y: number; first: number; last: number }
/** Map-frame cube belief: centre x, y and how often it was seen. */
export type CubeEstimate = [number, number, number];

/** Runs of columns that look like one free-standing 6 cm cube.
 * The sensor reports the strongest return per zone, so a bright cube fills every zone it barely
 * touches: the run is up to one zone wider than the cube. Knowing the size, the centre is the
 * middle of the run, half a cube behind the nearest face. */
export function cubeSightings(columns: ToFColumn[], mountX: number, fovRad: number): CubeSighting[] {
  const size = columns.length;
  const zone = 2 * Math.tan(fovRad / 2) / size; // zone width per metre of depth
  const out: CubeSighting[] = [];
  for (let i = 0; i < size;) {
    if (!columns[i]) { i++; continue; }
    let j = i;
    while (j + 1 < size && columns[j + 1] && Math.abs(columns[j + 1]!.depth - columns[j]!.depth) < CUBE_STEP) j++;
    const run = columns.slice(i, j + 1) as NonNullable<ToFColumn>[];
    const near = Math.min(...run.map((c) => c.depth)), far = Math.max(...run.map((c) => c.depth));
    // Free-standing: a neighbour on both sides, each empty or clearly farther. A run at the edge of the view is unknown.
    const open = (k: number) => !columns[k] || columns[k]!.depth > far + CUBE_GAP;
    const span = near * (run[run.length - 1].tan - run[0].tan);
    if (i > 0 && j < size - 1 && open(i - 1) && open(j + 1) && !run.some((c) => c.tall) && span <= CUBE + near * zone + 0.01) {
      const tan = (run[0].tan + run[run.length - 1].tan) / 2;
      const back = CUBE / 2 / Math.hypot(1, tan);
      out.push({ x: mountX + near + back, y: (near + back) * tan, first: i, last: j });
    }
    i = j + 1;
  }
  return out;
}

/** Fold one map-frame sighting into the known cubes: same cube when close, else a new one. */
export function addCube(cubes: CubeEstimate[], x: number, y: number): CubeEstimate {
  let best: CubeEstimate | null = null, gap = CUBE_JOIN;
  for (const c of cubes) {
    const d = Math.hypot(c[0] - x, c[1] - y);
    if (d < gap) { best = c; gap = d; }
  }
  if (!best) {
    // Bounded memory: a single stray sighting makes room first.
    if (cubes.length >= CUBE_MAX) cubes.splice(cubes.findIndex((c) => c[2] === Math.min(...cubes.map((k) => k[2]))), 1);
    best = [x, y, 1];
    cubes.push(best);
    return best;
  }
  const w = Math.min(best[2], CUBE_MEMORY);
  best[0] = (best[0] * w + x) / (w + 1);
  best[1] = (best[1] * w + y) / (w + 1);
  best[2]++;
  return best;
}

/** A beam that ends well behind a known cube's centre and passes right through it: the cube is not there any more. */
export function seenThrough(cube: CubeEstimate, [ox, oy, ex, ey]: Ray): boolean {
  const dx = ex - ox, dy = ey - oy, length = Math.hypot(dx, dy);
  const along = ((cube[0] - ox) * dx + (cube[1] - oy) * dy) / length;
  if (along < 0 || along > length - CUBE) return false;
  // Within half a cube of the centre the bright cube would have filled that zone.
  return Math.abs((cube[0] - ox) * dy - (cube[1] - oy) * dx) / length < CUBE / 2;
}

/** Passive inspector estimate for programs that do not publish their own map.
 * Sensor inputs only; no world pose, arena geometry, motor commands or randomness.
 * Uses the same odometry and occupancy conventions as portable spark_mapping.py.
 * This fallback does not perform scan matching and is labelled separately in the UI.
 */
export class SensorMapper {
  private readonly size = 48;
  private readonly cell = 0.05;
  private readonly cells = new Int8Array(this.size ** 2);
  private pose: Pose = [0, 0, 0];
  private last: { t: number; enc: [number, number]; gyro: number } | null = null;
  private biasTotal = 0;
  private biasCount = 0;
  private bias = 0;
  private calibrating = true;
  private distance = 0;
  private sequence = -1;
  private scans = 0;
  private scanTime: number | null = null;
  private scanPose: Pose | null = null;
  private rays: Ray[] = [];
  private grid = "0".repeat(this.size ** 2);
  private readonly cubes: CubeEstimate[] = [];

  get sampledAtUs() { return this.last?.t ?? -Infinity; }

  updateOdometry(t: number, enc: [number, number], gyro: number) {
    const last = this.last;
    if (last && t <= last.t) return;
    // Estimate startup bias only while both encoders report no movement.
    // Once moving, keep that calibration; never infer heading from physics pose.
    if (last && (enc[0] !== last.enc[0] || enc[1] !== last.enc[1])) this.calibrating = false;
    if (this.calibrating) {
      this.biasTotal += gyro;
      this.bias = this.biasTotal / ++this.biasCount;
    }
    if (last) {
      const ds = ((enc[0] - last.enc[0]) + (enc[1] - last.enc[1])) / (2 * 8467);
      const turn = ((gyro + last.gyro) / 2 - this.bias) * Math.PI / 180 * (t - last.t) / 1e6;
      const mid = this.pose[2] + turn / 2;
      this.pose = [this.pose[0] + ds * Math.cos(mid), this.pose[1] + ds * Math.sin(mid), wrap(this.pose[2] + turn)];
      this.distance += Math.abs(ds);
    }
    this.last = { t, enc, gyro };
  }

  observe(frame: MappingFrame) {
    if (frame.sequence === this.sequence || frame.capturedAtUs !== this.sampledAtUs) return;
    const size = Math.sqrt(frame.distanceMm.length);
    if (size !== 4 && size !== 8) return;
    this.sequence = frame.sequence;
    const [x, y, heading] = this.pose;
    const c = Math.cos(heading), s = Math.sin(heading);
    const ox = x + VL53L5CX.mount[0] * c, oy = y + VL53L5CX.mount[0] * s;
    const columns: ToFColumn[] = [];
    for (let col = 0; col < size; col++) {
      const accepted: [number, number][] = [];
      for (let row = 0; row < size; row++) {
        const zone = row * size + col;
        if (frame.targetStatus[zone] !== 5 || frame.targetsDetected[zone] < 1) continue;
        const d = frame.distanceMm[zone] / 1000;
        const height = VL53L5CX.mount[2] + d * tofSlope(zone, size)[2];
        // Reject probable floor and above-wall returns. Invalid means unknown.
        if (d >= 0.02 && d <= 1.5 && height >= 0.025 && height <= 0.13) accepted.push([d, height]);
      }
      if (!accepted.length) { columns.push(null); continue; }
      const depth = Math.min(...accepted.map(([d]) => d));
      const tall = accepted.some(([d, h]) => d - depth < CUBE_STEP && h > CUBE_TALL);
      columns.push({ depth, tan: tofSlope(col, size)[1], tall });
    }
    const rays: Ray[] = [];
    const sightings = cubeSightings(columns, VL53L5CX.mount[0], VL53L5CX.fovDeg * Math.PI / 180);
    for (const column of columns) {
      if (!column) continue;
      const forward = VL53L5CX.mount[0] + column.depth, lateral = column.depth * column.tan;
      rays.push([ox, oy, x + c * forward - s * lateral, y + s * forward + c * lateral]);
    }
    // Beams that pass through a known cube remove it (pushed away); then this frame's sightings.
    for (const cube of [...this.cubes]) {
      if (!rays.some((r) => seenThrough(cube, r))) continue;
      cube[2] = Math.min(cube[2], CUBE_MEMORY) - 1;
      if (cube[2] <= 0) this.cubes.splice(this.cubes.indexOf(cube), 1);
    }
    for (const g of sightings) addCube(this.cubes, x + c * g.x - s * g.y, y + s * g.x + c * g.y);
    const hits = new Set<number>();
    // Free evidence before all endpoints, so crossing beams preserve this scan's hits.
    for (const [ox, oy, ex, ey] of rays) {
      const hit = this.index(ex, ey);
      if (hit !== null) hits.add(hit);
      const steps = Math.max(1, Math.ceil(Math.hypot(ex - ox, ey - oy) / (this.cell / 2)));
      let previous: number | null = null;
      for (let i = 0; i < steps; i++) {
        const index = this.index(ox + (ex - ox) * i / steps, oy + (ey - oy) * i / steps);
        if (index !== null && index !== hit && index !== previous) this.mark(index, -1);
        previous = index;
      }
    }
    for (const hit of hits) this.mark(hit, 3);
    this.grid = this.published();
    this.rays = rays;
    this.scanPose = [...this.pose];
    this.scanTime = frame.capturedAtUs / 1e6;
    this.scans++;
  }

  packet(): DiscoveryPacket {
    return {
      v: 1, source: "sensors", t: this.sampledAtUs / 1e6,
      pose: [...this.pose], distance: this.distance, bias: this.bias,
      state: "Sensor-Schätzung", size: this.size, cell: this.cell,
      scans: this.scans, scan_t: this.scanTime, scan_pose: this.scanPose,
      rays: this.rays, grid: this.grid,
      cubes: this.cubes.map(([x, y, n]) => [x, y, n]),
    };
  }

  private index(x: number, y: number): number | null {
    const half = this.size * this.cell / 2;
    const col = Math.floor((x + half) / this.cell), row = Math.floor((half - y) / this.cell);
    return col >= 0 && row >= 0 && col < this.size && row < this.size ? row * this.size + col : null;
  }

  /** Obstacle cells around a known cube are that cube seen too wide: hidden, the cube is drawn at its real size. */
  private published(): string {
    const grid = Array.from(this.cells, (v) => v > 0 ? "2" : v < 0 ? "1" : "0");
    const half = this.size * this.cell / 2, reach = CUBE / 2 + this.cell;
    for (const [x, y, seen] of this.cubes) {
      if (seen < 2) continue;
      for (let dx = -2; dx <= 2; dx++) for (let dy = -2; dy <= 2; dy++) {
        const index = this.index(x + dx * this.cell, y + dy * this.cell);
        if (index === null || grid[index] !== "2") continue;
        const cx = -half + (index % this.size + 0.5) * this.cell, cy = half - (Math.floor(index / this.size) + 0.5) * this.cell;
        if (Math.abs(cx - x) < reach && Math.abs(cy - y) < reach) grid[index] = "0";
      }
    }
    return grid.join("");
  }

  private mark(index: number, delta: number) {
    this.cells[index] = Math.max(-6, Math.min(6, this.cells[index] + delta));
  }
}
