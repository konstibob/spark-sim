/** ST DS13754 rev 13 / UM2884 rev 7. See docs/VL53L5CX.md for sources and model limits. */
export const VL53L5CX = {
  fovDeg: 45, // horizontal AND vertical detection volume, not the diagonal 65° marketing figure
  minMm: 20,
  maxMm: 4000,
  // Front mounting, metres in the chassis frame (x forward, y left, z up; z from the ground).
  // Height measured: 9 cm on the robot. Forward offset is still assumed.
  mount: [0.052, 0, 0.09] as const,
  samplesPerAxis: 3,
} as const;

/** ULD order: zone 0 sees the top RIGHT of the scene (Rx lens flips both axes).
 * Rows here go top to bottom in scene space; increasing raw row goes downward. */
export function tofSlope(zone: number, size: number, dx = 0.5, dy = 0.5): [number, number, number] {
  const half = VL53L5CX.fovDeg * Math.PI / 360;
  const col = zone % size;
  const row = Math.floor(zone / size);
  return [1, Math.tan(((col + dx) / size * 2 - 1) * half), Math.tan((1 - (row + dy) / size * 2) * half)];
}

export function tofColor(mm: number): string {
  // Comma syntax is understood by both browser CSS and THREE.Color.setStyle().
  return `hsl(${Math.round(35 + 205 * Math.min(1, mm / VL53L5CX.maxMm))}, 80%, 53%)`;
}

export interface ToFFrame {
  /** Simulation microseconds at capture, shared by UI and program. */
  capturedAtUs: number;
  sequence: number;
  distanceMm: number[];
  targetStatus: number[];
  targetsDetected: number[];
  /** Capture pose, so old measurements do not move with the robot between frames. */
  origin: number[];
  rotation: number[];
}

export interface ToFSnapshot {
  enabled: boolean;
  ranging: boolean;
  resolution: number; // 16 or 64 zones
  frequencyHz: number;
  frame: ToFFrame | null;
}
