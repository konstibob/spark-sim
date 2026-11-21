import type { DiscoveryPacket } from "../shared/discovery.ts";

type Point = [number, number];
type Pose = [number, number, number];

/** Zumo 2040 footprint, metres (≈ 10 × 10 cm) */
const BODY = 0.1;
/** VL53L5CX: 45° field of view; the cone is drawn this far when there is no hit */
const FOV = Math.PI / 4;
const CONE = 0.45;
/** smallest map window, metres: a fresh map still shows the robot large enough */
const MIN_SPAN = 0.8;

const turned = (a: number) => Math.abs(Math.atan2(Math.sin(a), Math.cos(a)));

/** Draw sensor-derived estimates only. No world/arena dependency. */
export class DiscoveryView {
  private canvas = document.getElementById("discovery-map") as HTMLCanvasElement;
  private mode: "map" | "robot" = "map";
  private packet: DiscoveryPacket | null = null;
  private path: Point[] = [];
  private time = 0;
  private tofEnabled = false;

  constructor() {
    document.querySelectorAll<HTMLButtonElement>("[data-discovery-mode]").forEach((button) => {
      button.onclick = () => {
        this.mode = button.dataset.discoveryMode as "map" | "robot";
        document.querySelectorAll("[data-discovery-mode]").forEach((b) => b.setAttribute("aria-pressed", String(b === button)));
        this.draw();
      };
    });
    this.draw();
  }

  render(packet: DiscoveryPacket | null, path: Point[], time: number, tofEnabled = false) {
    this.packet = packet;
    this.path = path;
    this.time = time;
    this.tofEnabled = tofEnabled;
    const status = document.getElementById("discovery-status")!;
    const pose = document.getElementById("discovery-pose")!;
    if (!packet) {
      status.textContent = tofEnabled ? "Warte auf erste Sensormessung …" : "VL53L5CX aktivieren, um die Umgebung zu kartieren.";
      pose.textContent = "x —   y —   θ —";
    } else {
      const slam = packet.matched === undefined ? "" : ` · SLAM ${packet.matched}/${packet.scans} korrigiert`;
      const source = packet.source === "sensors" ? "" : "Programmkarte · ";
      status.textContent = `${source}${packet.state} · ${packet.scans} Scans${slam}${packet.source === "sensors" && !tofEnabled ? " · ToF aus" : ""}`;
      const [x, y, yaw] = packet.pose;
      pose.textContent = `x ${(x * 100).toFixed(1)} cm · y ${(y * 100).toFixed(1)} cm · θ ${(yaw * 180 / Math.PI).toFixed(1)}°`;
    }
    const meta = document.getElementById("discovery-meta")!;
    const age = packet?.scan_t === null || packet?.scan_t === undefined ? null : Math.max(0, time - packet.scan_t);
    const odom = packet?.odom;
    const shift = packet && odom
      ? ` · Korrektur ggü. Odometrie ${(Math.hypot(packet.pose[0] - odom[0], packet.pose[1] - odom[1]) * 100).toFixed(1)} cm / ${(turned(packet.pose[2] - odom[2]) * 180 / Math.PI).toFixed(1)}°`
      : "";
    meta.textContent = packet
      ? `Weg ${(packet.distance * 100).toFixed(0)} cm · letzter Scan ${age === null ? "—" : `${age.toFixed(1)} s alt`}${shift} · ${Math.round(packet.cell * 100)}-cm-Raster`
      : "Start = (0, 0, 0°) angenommen · Umgebung unbekannt";
    document.getElementById("discovery-note")!.textContent = packet && packet.source !== "sensors"
      ? "Eigene Kartenausgabe des Python-Programms. Das Mapping-Beispiel korrigiert Encoder + Gyro durch Scan Matching. Keine echten Kartenpositionen."
      : "Automatische Sensor-Schätzung aus Encoder, Gyro und ToF, unabhängig vom Fahrprogramm. Ursprung beim Aktivieren angenommen; keine echten Kartenpositionen, kein Scan Matching. Eine eigene Programmkarte hat Vorrang.";
    this.draw();
  }

  /** Map mode frames everything the robot knows so far: observed cells, path, pose. Grows, never jumps back. */
  private frame(p: DiscoveryPacket): { cx: number; cy: number; span: number } {
    let x0 = Math.min(0, p.pose[0]), x1 = Math.max(0, p.pose[0]), y0 = Math.min(0, p.pose[1]), y1 = Math.max(0, p.pose[1]);
    const add = (x: number, y: number) => { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); };
    for (const [x, y] of this.path) add(x, y);
    if (p.grid) {
      const half = p.size * p.cell / 2;
      for (let i = 0; i < p.grid.length; i++) {
        if (p.grid[i] === "0") continue;
        const x = -half + (i % p.size + 0.5) * p.cell, y = half - (Math.floor(i / p.size) + 0.5) * p.cell;
        add(x - p.cell / 2, y - p.cell / 2); add(x + p.cell / 2, y + p.cell / 2);
      }
    }
    const pad = 0.12;
    return { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, span: Math.max(MIN_SPAN, x1 - x0 + 2 * pad, y1 - y0 + 2 * pad) };
  }

  private draw() {
    const c = this.canvas.getContext("2d")!;
    const w = this.canvas.width;
    const p = this.packet;
    c.setTransform(1, 0, 0, 1, 0, 0);
    c.fillStyle = "#111b2b";
    c.fillRect(0, 0, w, w);
    const inner = w - 24;
    // world (metres, +y up) -> canvas. Map: north up, framed to the explored area. Robot: robot centred, facing up.
    let scale: number, span: number;
    if (this.mode === "robot" || !p) {
      span = 1.2;
      scale = inner / span;
      c.translate(w / 2, this.mode === "robot" ? w * 0.68 : w / 2);
      c.scale(scale, -scale);
      if (p) {
        c.rotate(Math.PI / 2 - p.pose[2]);
        c.translate(-p.pose[0], -p.pose[1]);
      }
    } else {
      const f = this.frame(p);
      span = f.span;
      scale = inner / span;
      c.translate(w / 2, w / 2);
      c.scale(scale, -scale);
      c.translate(-f.cx, -f.cy);
    }
    const px = 1 / scale; // one canvas pixel in metres
    const line = (a: Point, b: Point) => { c.beginPath(); c.moveTo(...a); c.lineTo(...b); c.stroke(); };

    // the robot's map: observed free space and obstacle hypotheses (unknown stays background)
    if (p?.grid) {
      const half = p.size * p.cell / 2;
      for (let i = 0; i < p.grid.length; i++) {
        const value = p.grid[i];
        if (value === "0") continue;
        c.fillStyle = value === "2" ? "#f5ab61" : "#243d54";
        c.fillRect(-half + i % p.size * p.cell, half - (Math.floor(i / p.size) + 1) * p.cell, p.cell + px * 0.4, p.cell + px * 0.4);
      }
    }
    // reference grid: 10 cm, 20 cm once the window is large
    const step = span > 1.6 ? 0.2 : 0.1;
    const reach = 1.4 + (p ? Math.hypot(p.pose[0], p.pose[1]) : 0) + span;
    c.strokeStyle = "#34435755"; c.lineWidth = px;
    for (let v = -Math.ceil(reach / step) * step; v <= reach; v += step) {
      line([v, -reach], [v, reach]);
      line([-reach, v], [reach, v]);
    }
    // start pose, the frame's origin
    c.strokeStyle = "#8e9eaf"; c.lineWidth = 1.5 * px;
    line([-0.025, 0], [0.025, 0]); line([0, -0.025], [0, 0.025]);
    // estimated path
    if (this.path.length > 1) {
      c.strokeStyle = "#64a9ff"; c.lineWidth = 2 * px;
      c.beginPath();
      this.path.forEach(([x, y], i) => (i ? c.lineTo(x, y) : c.moveTo(x, y)));
      c.stroke();
    }
    const fresh = p?.scan_t !== null && p?.scan_t !== undefined && this.time - p.scan_t < 0.9;
    if (p) {
      // where the sensor is looking now: the cone, then this frame's hits
      const [x, y, yaw] = p.pose;
      const sx = x + 0.052 * Math.cos(yaw), sy = y + 0.052 * Math.sin(yaw);
      const depth = fresh && p.rays.length ? Math.max(...p.rays.map(([ox, oy, ex, ey]) => Math.hypot(ex - ox, ey - oy))) : CONE;
      const r = Math.max(0.15, Math.min(1.5, depth / Math.cos(FOV / 2)));
      c.fillStyle = "#70d6c31c"; c.strokeStyle = "#70d6c366"; c.lineWidth = px;
      c.beginPath(); c.moveTo(sx, sy); c.arc(sx, sy, r, yaw - FOV / 2, yaw + FOV / 2); c.closePath(); c.fill(); c.stroke();
      if (fresh) {
        for (const [ox, oy, ex, ey] of p.rays) {
          c.strokeStyle = "#70d6c388"; c.lineWidth = px;
          line([ox, oy], [ex, ey]);
          c.fillStyle = "#ffbd78";
          c.beginPath(); c.arc(ex, ey, 3 * px, 0, Math.PI * 2); c.fill();
        }
      }
      // odometry alone, where the robot would think it is without the map correction
      if (p.odom && (Math.hypot(p.odom[0] - x, p.odom[1] - y) > 0.01 || turned(p.odom[2] - yaw) > 0.02)) {
        this.body(c, p.odom, px, true);
      }
      this.body(c, p.pose, px, false);
    }
    // labels in canvas pixels
    c.setTransform(1, 0, 0, 1, 0, 0);
    c.fillStyle = "#aab9ca"; c.font = "11px monospace";
    c.fillText(this.mode === "map" ? "+y ↑  ·  +x →  ·  + = Start" : "Roboter blickt nach oben", 12, 17);
    if (p?.odom && this.mode === "map") {
      c.setLineDash([3, 3]); c.strokeStyle = "#aab9ca"; c.lineWidth = 1;
      c.strokeRect(12, w - 24, 9, 9); c.setLineDash([]);
      c.fillText("nur Odometrie", 26, w - 16);
    }
    if (!p) {
      c.textAlign = "center";
      c.fillText(this.tofEnabled ? "Warte auf Sensormessung …" : "VL53L5CX aktivieren", w / 2, w / 2 - 30);
      c.textAlign = "left";
    }
    const bar = (step === 0.2 ? 0.2 : 0.1) * scale;
    c.strokeStyle = "#aab9ca"; c.lineWidth = 2;
    c.beginPath(); c.moveTo(w - 14 - bar, w - 14); c.lineTo(w - 14, w - 14); c.stroke();
    c.textAlign = "right";
    c.fillText(step === 0.2 ? "20 cm" : "10 cm", w - 14, w - 21);
    c.textAlign = "left";
  }

  /** The robot to scale: body square and heading wedge. `ghost` = dashed outline for the odometry-only pose. */
  private body(c: CanvasRenderingContext2D, [x, y, yaw]: Pose, px: number, ghost: boolean) {
    c.save();
    c.translate(x, y);
    c.rotate(yaw);
    // never smaller than ~14 px, so a zoomed-out map still shows where the robot is
    const k = Math.max(1, 14 * px / BODY);
    c.scale(k, k);
    const h = BODY / 2;
    c.lineWidth = 1.5 * px / k;
    if (ghost) {
      c.setLineDash([3 * px / k, 3 * px / k]);
      c.strokeStyle = "#aab9ca";
      c.strokeRect(-h, -h, BODY, BODY);
      c.beginPath(); c.moveTo(h, 0); c.lineTo(0, 0); c.stroke();
    } else {
      c.fillStyle = "#3a88fecc"; c.strokeStyle = "#d8eaff";
      c.fillRect(-h, -h, BODY, BODY); c.strokeRect(-h, -h, BODY, BODY);
      c.fillStyle = "#d8eaff";
      c.beginPath(); c.moveTo(h * 0.95, 0); c.lineTo(-h * 0.2, h * 0.55); c.lineTo(-h * 0.2, -h * 0.55); c.closePath(); c.fill();
    }
    c.restore();
  }
}
