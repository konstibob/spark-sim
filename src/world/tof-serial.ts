import type { ToFFrame } from "../shared/vl53l5cx.ts";

/** Proposed ESP32-C6 → RP2040 protocol; ASCII JSONL, UART0, 8N1, 115200 baud.
 * This models wire time + bounded RX buffering, not an existing ESP firmware. */
export class ToFSerialBridge {
  baud = 115200;
  capacity = 4096;
  private packets: { text: string; start: number; read: number }[] = [];
  private txUntil = 0;

  reset() { this.packets = []; this.txUntil = 0; }

  configure(baud: number, capacity: number) {
    this.baud = Math.max(1200, Math.min(3_000_000, Math.round(baud) || 115200));
    this.capacity = Math.max(256, Math.min(16384, Math.round(capacity) || 4096));
    this.reset();
  }

  publish(frame: ToFFrame, resolution: number) {
    const text = JSON.stringify({ type: "vl53l5cx", seq: frame.sequence, t_us: frame.capturedAtUs, resolution,
      distance_mm: frame.distanceMm, target_status: frame.targetStatus, nb_target_detected: frame.targetsDetected }) + "\n";
    // Drop a whole NEW packet on overflow: keep already buffered lines intact.
    if (text.length + this.packets.reduce((n, p) => n + p.text.length - p.read, 0) > this.capacity) return;
    const start = Math.max(frame.capturedAtUs, this.txUntil);
    this.packets.push({ text, start, read: 0 });
    this.txUntil = start + text.length * 10e6 / this.baud;
  }

  available(now: number): number {
    return this.packets.reduce((n, p) => n + Math.max(0, Math.min(p.text.length, Math.floor((now - p.start) * this.baud / 10e6)) - p.read), 0);
  }

  read(now: number, count: number): number[] {
    const bytes: number[] = [];
    while (this.packets.length && bytes.length < count) {
      const p = this.packets[0];
      const end = Math.min(p.text.length, Math.floor((now - p.start) * this.baud / 10e6), p.read + count - bytes.length);
      while (p.read < end) bytes.push(p.text.charCodeAt(p.read++));
      if (p.read === p.text.length) this.packets.shift();
      else break;
    }
    return bytes;
  }
}
