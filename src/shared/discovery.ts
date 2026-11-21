// This boundary accepts estimates from robot output or the passive sensor mapper.
// Never pass WorldSnapshot, capture origin/rotation, arena geometry or true poses
// into the robot-view renderer. Sensor fallback is always labelled as such.
export interface DiscoveryPacket {
  v: 1;
  /** Absent = the robot program's own output; sensors = inspector fallback. */
  source?: "sensors";
  t: number;
  /** Robot's belief in its own map frame (odometry after scan-matching correction). */
  pose: [number, number, number];
  /** Encoder/gyro odometry alone, before correction; absent in older output. */
  odom?: [number, number, number];
  /** Scans whose pose scan matching corrected. */
  matched?: number;
  distance: number;
  bias: number;
  state: string;
  size: number;
  cell: number;
  scans: number;
  scan_t: number | null;
  scan_pose?: [number, number, number] | null;
  rays: [number, number, number, number][];
  grid?: string;
}

const finite = (n: unknown): n is number => typeof n === "number" && Number.isFinite(n);

/** Validate untrusted print output. No execution or HTML interpretation. */
export function parseDiscovery(line: string): DiscoveryPacket | null {
  if (!line.startsWith("@spark-map ") || line.length > 16_384) return null;
  try {
    const p = JSON.parse(line.slice(11)) as DiscoveryPacket;
    if (p.source !== undefined) return null; // program output cannot impersonate the fallback
    if (p.v !== 1 || !finite(p.t) || p.t < 0 || !Array.isArray(p.pose) || p.pose.length !== 3 || !p.pose.every(finite)) return null;
    if (!finite(p.distance) || p.distance < 0 || !finite(p.bias) || typeof p.state !== "string" || p.state.length > 80) return null;
    if (!Number.isInteger(p.size) || p.size < 4 || p.size > 80 || !finite(p.cell) || p.cell < 0.01 || p.cell > 0.5) return null;
    if (!Number.isInteger(p.scans) || p.scans < 0 || !(p.scan_t === null || finite(p.scan_t) && p.scan_t >= 0 && p.scan_t <= p.t + 0.001)) return null;
    if (!Array.isArray(p.rays) || p.rays.length > 64 || !p.rays.every((r) => Array.isArray(r) && r.length === 4 && r.every(finite))) return null;
    if (p.odom !== undefined && (!Array.isArray(p.odom) || p.odom.length !== 3 || !p.odom.every(finite))) return null;
    if (p.matched !== undefined && (!Number.isInteger(p.matched) || p.matched < 0 || p.matched > p.scans)) return null;
    if (p.scan_pose !== undefined && p.scan_pose !== null && (!Array.isArray(p.scan_pose) || p.scan_pose.length !== 3 || !p.scan_pose.every(finite))) return null;
    if (p.grid !== undefined && (typeof p.grid !== "string" || p.grid.length !== p.size ** 2 || /[^012]/.test(p.grid))) return null;
    return p;
  } catch { return null; }
}

export interface DiscoveryScan {
  t: number;
  /** Robot's estimate at capture (odometry + scan matching), never physics pose. */
  pose: [number, number, number];
  /** Measured rays expressed in the estimated world frame, metres. */
  rays: [number, number, number, number][];
}

export function discoveryScans(packets: DiscoveryPacket[]): DiscoveryScan[] {
  const out: DiscoveryScan[] = [];
  let last = -1;
  for (const p of packets) {
    if (p.scan_t === null || !p.scan_pose || p.scans === last) continue;
    last = p.scans;
    out.push({ t: p.scan_t, pose: p.scan_pose, rays: p.rays });
  }
  return out;
}

/** Stdout can split in the middle of JSON at ANY VM/world sync. */
export class RobotOutputLines {
  private pending = "";
  feed(text: string): string[] {
    this.pending += text;
    const lines = this.pending.split("\n");
    this.pending = lines.pop()!;
    if (this.pending.length > 32_768) {
      lines.push(this.pending);
      this.pending = "";
    }
    return lines.map((s) => s.replace(/\r$/, ""));
  }
  flush(): string {
    const rest = this.pending;
    this.pending = "";
    return rest;
  }
}
