import type { DiscoveryPacket } from "../shared/discovery.ts";

type Point = [number, number];

/** Draw only what the portable Python mapper reports. No world/arena dependency. */
export class DiscoveryView {
  private canvas = document.getElementById("discovery-map") as HTMLCanvasElement;
  private mode: "map" | "scan" = "map";
  private packet: DiscoveryPacket | null = null;
  private path: Point[] = [];
  private time = 0;

  constructor() {
    document.querySelectorAll<HTMLButtonElement>("[data-discovery-mode]").forEach((button) => {
      button.onclick = () => {
        this.mode = button.dataset.discoveryMode as "map" | "scan";
        document.querySelectorAll("[data-discovery-mode]").forEach((b) => b.setAttribute("aria-pressed", String(b === button)));
        this.draw();
      };
    });
    this.draw();
  }

  render(packet: DiscoveryPacket | null, path: Point[], time: number) {
    this.packet = packet;
    this.path = path;
    this.time = time;
    const status = document.getElementById("discovery-status")!;
    const pose = document.getElementById("discovery-pose")!;
    if (!packet) {
      status.textContent = "Noch keine Schätzung · Testskript laden, Run, dann A.";
      pose.textContent = "x —   y —   θ —";
    } else {
      status.textContent = `${packet.state} · ${packet.scans} Scans`;
      const [x, y, yaw] = packet.pose;
      pose.textContent = `x ${(x * 100).toFixed(1)} cm · y ${(y * 100).toFixed(1)} cm · θ ${(yaw * 180 / Math.PI).toFixed(1)}°`;
    }
    const meta = document.getElementById("discovery-meta")!;
    const age = packet?.scan_t === null || packet?.scan_t === undefined ? null : Math.max(0, time - packet.scan_t);
    meta.textContent = packet
      ? `Weg ${(packet.distance * 100).toFixed(0)} cm · letzter Scan ${age === null ? "—" : `${age.toFixed(1)} s alt`} · ${Math.round(packet.cell * 100)}-cm-Raster`
      : "Start = (0, 0, 0°) angenommen · Umgebung unbekannt";
    this.draw();
  }

  private draw() {
    const c = this.canvas.getContext("2d")!;
    const w = this.canvas.width;
    const p = this.packet;
    c.fillStyle = "#111b2b";
    c.fillRect(0, 0, w, w);
    const extent = p ? p.size * p.cell / 2 : 1.2;
    const scale = (w - 32) / (2 * extent);
    const point = (x: number, y: number): Point => {
      if (this.mode === "scan" && p) {
        const dx = x - p.pose[0], dy = y - p.pose[1];
        const yaw = p.pose[2];
        return [w / 2 - (-Math.sin(yaw) * dx + Math.cos(yaw) * dy) * scale, w * 0.76 - (Math.cos(yaw) * dx + Math.sin(yaw) * dy) * scale];
      }
      return [w / 2 + x * scale, w / 2 - y * scale];
    };
    const line = (a: Point, b: Point) => { c.beginPath(); c.moveTo(...a); c.lineTo(...b); c.stroke(); };
    if (this.mode === "map") {
      if (p?.grid) {
        const step = p.cell * scale;
        for (let i = 0; i < p.grid.length; i++) {
          const value = p.grid[i];
          if (value === "0") continue;
          const x = 16 + i % p.size * step, y = 16 + Math.floor(i / p.size) * step;
          c.fillStyle = value === "2" ? "#f5ab61" : "#243d54";
          c.fillRect(x, y, step + 0.3, step + 0.3);
        }
      }
      c.strokeStyle = "#344357"; c.lineWidth = 1;
      for (let v = -1; v <= 1.01; v += 0.2) {
        line(point(v, -extent), point(v, extent));
        line(point(-extent, v), point(extent, v));
      }
      c.strokeStyle = "#64a9ff"; c.lineWidth = 2;
      c.beginPath();
      this.path.forEach(([x, y], i) => { const [px, py] = point(x, y); if (i) c.lineTo(px, py); else c.moveTo(px, py); });
      c.stroke();
      c.strokeStyle = "#8e9eaf";
      line(point(-0.025, 0), point(0.025, 0)); line(point(0, -0.025), point(0, 0.025));
    }
    const fresh = p?.scan_t !== null && p?.scan_t !== undefined && this.time - p.scan_t < 0.9;
    if (p && fresh) {
      for (const [ox, oy, ex, ey] of p.rays) {
        c.strokeStyle = "#70d6c388"; c.lineWidth = 1;
        line(point(ox, oy), point(ex, ey));
        c.fillStyle = "#ffbd78";
        const [x, y] = point(ex, ey); c.beginPath(); c.arc(x, y, 3, 0, Math.PI * 2); c.fill();
      }
    }
    if (p) {
      const [x, y] = point(p.pose[0], p.pose[1]);
      c.save(); c.translate(x, y); c.rotate(this.mode === "scan" ? -Math.PI / 2 : -p.pose[2]);
      c.fillStyle = "#70aeff"; c.strokeStyle = "#d8eaff"; c.lineWidth = 1.5;
      c.beginPath(); c.moveTo(11, 0); c.lineTo(-7, -7); c.lineTo(-4, 0); c.lineTo(-7, 7); c.closePath(); c.fill(); c.stroke(); c.restore();
    }
    c.fillStyle = "#aab9ca"; c.font = "11px monospace";
    c.fillText(this.mode === "map" ? "+y ↑  ·  +x →" : "Roboter blickt nach oben", 15, 17);
    if (!p || this.mode === "scan" && !fresh) {
      c.textAlign = "center"; c.fillStyle = "#aab9ca";
      c.fillText(!p ? "Umgebung noch unbekannt" : "Kein aktueller Scan", w / 2, w / 2 - 20);
      c.textAlign = "left";
    }
    c.strokeStyle = "#aab9ca"; c.lineWidth = 2;
    line([w - 20 - 0.2 * scale, w - 18], [w - 20, w - 18]);
    c.fillText("20 cm", w - 70, w - 25);
  }
}
