import type { DiscoveryPacket } from "./discovery.ts";
import { tofSlope, VL53L5CX, type ToFFrame } from "./vl53l5cx.ts";

type Pose = [number, number, number];
type Ray = [number, number, number, number];
/** Deliberately excludes simulator capture origin/rotation and object identities. */
export type MappingFrame = Pick<ToFFrame, "capturedAtUs" | "sequence" | "distanceMm" | "targetStatus" | "targetsDetected">;
const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

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
    const rays: Ray[] = [];
    for (let col = 0; col < size; col++) {
      let depth = Infinity;
      for (let row = 0; row < size; row++) {
        const zone = row * size + col;
        if (frame.targetStatus[zone] !== 5 || frame.targetsDetected[zone] < 1) continue;
        const d = frame.distanceMm[zone] / 1000;
        const height = VL53L5CX.mount[2] + d * tofSlope(zone, size)[2];
        // Reject probable floor and above-wall returns. Invalid means unknown.
        if (d >= 0.02 && d <= 1.5 && height >= 0.025 && height <= 0.13) depth = Math.min(depth, d);
      }
      if (!Number.isFinite(depth)) continue;
      const lateral = depth * tofSlope(col, size)[1];
      const forward = VL53L5CX.mount[0] + depth;
      rays.push([ox, oy, x + c * forward - s * lateral, y + s * forward + c * lateral]);
    }
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
    this.grid = Array.from(this.cells, (v) => v > 0 ? "2" : v < 0 ? "1" : "0").join("");
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
    };
  }

  private index(x: number, y: number): number | null {
    const half = this.size * this.cell / 2;
    const col = Math.floor((x + half) / this.cell), row = Math.floor((half - y) / this.cell);
    return col >= 0 && row >= 0 && col < this.size && row < this.size ? row * this.size + col : null;
  }

  private mark(index: number, delta: number) {
    this.cells[index] = Math.max(-6, Math.min(6, this.cells[index] + delta));
  }
}
