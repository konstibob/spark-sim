import { tofSlope, VL53L5CX, type ToFFrame, type ToFSnapshot } from "../shared/vl53l5cx.ts";

type Hit = { distance: number; reflectance: number } | null;

/** Single strongest return per zone, continuous mode. Independent of rendering and arena rules. */
export class ToFSensor {
  enabled = false;
  ranging = false;
  resolution = 64;
  frequencyHz = 15;
  frame: ToFFrame | null = null;
  private nextAtUs = Infinity;
  private sequence = 0;
  private consumed = 0;

  enable(on: boolean, now: number) {
    this.enabled = on;
    this.resolution = 64;
    this.frequencyHz = 15;
    this.start(on, now);
  }

  start(on: boolean, now: number) {
    this.ranging = on;
    this.frame = null;
    this.consumed = this.sequence;
    this.nextAtUs = on ? now + 1e6 / this.frequencyHz : Infinity;
  }

  /** Returns -1 absent, -2 busy, -3 invalid setting; 1 success. */
  command(action: number, value: number, now: number): number {
    if (!this.enabled) return -1;
    if (action === 0) return 1;
    if (action === 1) { // ULD power-on defaults
      this.start(false, now);
      this.resolution = 16;
      this.frequencyHz = 1;
    } else if (action === 2 || action === 3) {
      if (this.ranging) return -2;
      if (action === 2) {
        if (value !== 16 && value !== 64) return -3;
        this.resolution = value;
        this.frequencyHz = Math.min(this.frequencyHz, value === 64 ? 15 : 60);
      } else {
        if (!Number.isInteger(value) || value < 1 || value > (this.resolution === 64 ? 15 : 60)) return -3;
        this.frequencyHz = value;
      }
      this.frame = null;
    } else if (action === 4 || action === 5) {
      this.start(action === 4, now);
    } else if (action === 6) {
      return this.ranging && this.frame !== null && this.frame.sequence !== this.consumed ? 1 : 0;
    } else if (action === 7) {
      if (!this.ranging || !this.frame) return 0;
      this.consumed = this.frame.sequence;
    } else return -3;
    return 1;
  }

  /** Called on physics steps, never from snapshot() or reads: polling cannot create new frames. */
  due(now: number): boolean {
    return this.enabled && this.ranging && now + 1e-6 >= this.nextAtUs;
  }

  tick(now: number, origin: number[], rotation: number[], cast: (direction: number[]) => Hit, noise: (sigma: number) => number) {
    if (!this.due(now)) return;
    this.nextAtUs += 1e6 / this.frequencyHz;
    const size = Math.sqrt(this.resolution);
    const distanceMm: number[] = [], targetStatus: number[] = [], targetsDetected: number[] = [];
    const samples = VL53L5CX.samplesPerAxis;
    for (let zone = 0; zone < this.resolution; zone++) {
      const returns: { mm: number; strength: number }[] = [];
      for (let row = 0; row < samples; row++) for (let col = 0; col < samples; col++) {
        const slope = tofSlope(zone, size, (col + 0.5) / samples, (row + 0.5) / samples);
        const length = Math.hypot(...slope);
        const hit = cast(slope.map((v) => v / length));
        if (!hit || hit.reflectance <= 0) continue;
        // The chip reports perpendicular depth, not the longer radial ray length.
        const mm = hit.distance / length * 1000;
        if (mm > VL53L5CX.maxMm) continue;
        const strength = hit.reflectance / Math.max(0.02, hit.distance) ** 2;
        const bucket = returns.find((b) => Math.abs(b.mm - mm) < 30);
        if (bucket) {
          bucket.mm = (bucket.mm * bucket.strength + mm * strength) / (bucket.strength + strength);
          bucket.strength += strength;
        } else returns.push({ mm, strength });
      }
      const best = returns.sort((a, b) => b.strength - a.strength)[0];
      const valid = best && best.mm >= VL53L5CX.minMm;
      // Estimated repeatability only; this is NOT a fit of ST's accuracy tolerance to a Gaussian.
      const measured = valid ? Math.round(Math.max(20, Math.min(4000, best.mm + noise(3 + best.mm * 0.01)))) : 0;
      distanceMm.push(measured);
      targetStatus.push(valid ? 5 : best ? 4 : 255);
      targetsDetected.push(best ? 1 : 0);
    }
    this.frame = { sequence: ++this.sequence, capturedAtUs: now, distanceMm, targetStatus, targetsDetected, origin, rotation };
  }

  snapshot(): ToFSnapshot {
    return { enabled: this.enabled, ranging: this.ranging, resolution: this.resolution, frequencyHz: this.frequencyHz, frame: this.frame };
  }
}
