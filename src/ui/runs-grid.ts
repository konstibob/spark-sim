// "Runs": the program of the editor robot N times at once (1, 2, 4, 6, 9 or 16), each run its own world.
// Every run gets its own cube layout from its seed (or the Arena-Karte layout for all), the tiles show a live
// top view, the side column the results as they come in (mean, median, best/worst, a points histogram).
// Same base seed = same runs as `npm run runs -- prog.py --base-seed N` on the command line.
import { arenaById, type Arena, type Pose } from "../shared/arenas.ts";
import type { CalibrationSetup } from "../shared/calibration.ts";
import type { WorldSnapshot, WorldTweaks } from "../shared/protocol.ts";
import { cubePoints, PUSH_PULL } from "../shared/push-pull.ts";
import { RUN_START_S, runCubes, runSeed, shortReason, summarizeRuns, type RunResult } from "../shared/runs.ts";
import { STUCK_WARN_S } from "../world/rules.ts";
import { MatchHost, type RobotConfig } from "./session.ts";

export interface RunsHost {
  /** the program to run (the robot in the editor) */
  robot(): RobotConfig;
  arena(): string;
  tweaks(): WorldTweaks;
  /** the Arena-Karte layout */
  layout(): Pose[];
  /** the Arena-Karte calibration setting (null = skipped) */
  calibration(): CalibrationSetup | null;
  speed(): number;
  /** load one run into the main 3D view (closes the grid) */
  open(run: { index: number; seed: number; cubes: Pose[]; robot: RobotConfig; arena: string; tweaks: WorldTweaks; calibration: CalibrationSetup | null }): void;
  /** the grid takes over the stage: stop the main match */
  stopMain(): void;
  log(text: string): void;
}

/** columns × rows per run count */
const GRID: Record<number, [number, number]> = { 1: [1, 1], 2: [2, 1], 4: [2, 2], 6: [3, 2], 9: [3, 3], 16: [4, 4] };
/** Push & Pull points range: −3 (stuck −1, left the arena −2, no cube) … 15 (five cubes in the small square) */
const MIN_POINTS = -3;
const MAX_POINTS = 15;

interface Run {
  index: number;
  seed: number;
  cubes: Pose[];
  host: MatchHost | null;
  snap: WorldSnapshot | null;
  trail: [number, number][];
  lastTrailT: number;
  error?: string;
  result: RunResult | null;
  tile: HTMLButtonElement;
  canvas: HTMLCanvasElement;
  head: HTMLElement;
  score: HTMLElement;
  foot: HTMLElement;
}

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls?: string, text?: string) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

const css = (name: string) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

export class RunsGrid {
  readonly el = el("div", "runs hidden");
  private tiles = el("div", "runs-tiles");
  private results = el("aside", "runs-results");
  private status = el("span", "runs-status");
  private seedInput = Object.assign(el("input"), { type: "number", min: "1", value: "1", title: "Base seed: run i uses seed + i·7919 (like the bench and npm run runs)" });
  private layoutBtn = el("button", "btn toggle", "Own layout per run");
  private runs: Run[] = [];
  private robot: RobotConfig | null = null;
  private arena: Arena | null = null;
  private tweaksUsed: WorldTweaks | null = null;
  private calibrationUsed: CalibrationSetup | null = null;
  private count = 16;
  private bg: HTMLCanvasElement | null = null;
  private frame = 0;
  private token = 0;
  private highlight: number | null = null;

  constructor(private host: RunsHost) {
    const bar = el("div", "runs-bar");
    const again = el("button", "btn", "↻ Again");
    again.title = "Run the same program again with this seed and layout setting";
    again.onclick = () => this.start(this.count);
    const close = el("button", "btn icon", "✕");
    close.title = "Close the runs and go back to the 3D view";
    close.setAttribute("aria-label", "Close the runs");
    close.onclick = () => this.close();
    this.layoutBtn.setAttribute("aria-pressed", "true");
    this.layoutBtn.title = "On: every run gets its own random cube layout from its seed. Off: all runs use the Arena-Karte layout (the seeds only vary noise and motors)";
    this.layoutBtn.onclick = () => {
      const on = this.layoutBtn.getAttribute("aria-pressed") !== "true";
      this.layoutBtn.setAttribute("aria-pressed", String(on));
      this.layoutBtn.textContent = on ? "Own layout per run" : "Arena-Karte layout";
    };
    const seedLabel = el("label", "runs-seed");
    seedLabel.append(el("span", "label", "Seed"), this.seedInput);
    bar.append(el("b", undefined, "Runs"), this.status, el("i", "runs-fill"), seedLabel, this.layoutBtn, again, close);
    const body = el("div", "runs-body");
    body.append(this.tiles, this.results);
    this.el.append(bar, body);
    new ResizeObserver(() => this.sizeCanvases()).observe(this.tiles);
  }

  get active() {
    return !this.el.classList.contains("hidden");
  }

  /** Start `count` runs of the editor robot's program on the current arena. */
  start(count: number) {
    this.dispose();
    this.host.stopMain();
    const token = ++this.token;
    this.count = GRID[count] ? count : 16;
    this.robot = this.host.robot();
    this.arena = arenaById(this.host.arena());
    this.tweaksUsed = { ...this.host.tweaks() };
    const cal = this.host.calibration();
    this.calibrationUsed = cal ? { ...cal } : null;
    this.bg = null;
    this.highlight = null;
    const base = Math.max(1, Math.floor(Number(this.seedInput.value)) || 1);
    this.seedInput.value = String(base);
    const fixed = this.layoutBtn.getAttribute("aria-pressed") === "true" ? null : this.host.layout();
    const [cols, rows] = GRID[this.count];
    this.tiles.style.gridTemplateColumns = `repeat(${cols}, minmax(0, 1fr))`;
    this.tiles.style.gridTemplateRows = `repeat(${rows}, minmax(0, 1fr))`;
    this.tiles.replaceChildren();
    this.runs = Array.from({ length: this.count }, (_, index) => {
      const seed = runSeed(base, index);
      const tile = el("button", "run-tile");
      tile.title = `Run ${index + 1}, seed ${seed}: click to watch it in the 3D view`;
      const canvas = el("canvas");
      const head = el("div", "rt-head");
      const score = el("b", "rt-score", "0");
      const foot = el("div", "rt-foot", "starting…");
      head.append(el("span", undefined, `#${index + 1} · ${seed}`), score);
      tile.append(canvas, head, foot);
      this.tiles.append(tile);
      const run: Run = { index, seed, cubes: runCubes(seed, fixed), host: null, snap: null, trail: [], lastTrailT: -1, result: null, tile, canvas, head, score, foot };
      tile.onclick = () => this.open(run);
      return run;
    });
    this.el.classList.remove("hidden");
    this.sizeCanvases();
    this.renderResults();
    this.host.log(
      `▦ ${this.count} runs · ${this.robot.name} · ${this.arena.name} · base seed ${base} · ${fixed ? "Arena-Karte layout" : "own cube layout per run"} · same as: npm run runs -- prog.py --runs ${this.count} --base-seed ${base}${fixed ? " --map layout.json" : ""}`,
    );
    for (const run of this.runs) {
      const h = new MatchHost({
        snapshot: (s) => {
          if (token !== this.token || run.result) return;
          run.snap = s;
          const r = s.robots[0];
          if (r && s.t - run.lastTrailT > 0.1) {
            run.trail.push([r.pos[0], r.pos[1]]);
            run.lastTrailT = s.t;
          }
          if (s.match.ended) this.finish(run, s);
        },
        robot: (m) => {
          if (token !== this.token) return;
          if ((m.type === "done" && m.error) || m.type === "fatal") run.error = m.type === "fatal" ? m.message : m.error;
        },
        error: (message) => {
          if (token === this.token) run.error = message;
        },
      });
      run.host = h;
      void h.load({
        arena: this.arena.id,
        robots: [{ ...this.robot }],
        seed: run.seed,
        poses: this.arena.starts.slice(0, 1),
        autoPressA: RUN_START_S,
        run: true,
        tweaks: this.tweaksUsed,
        cubes: run.cubes,
        calibration: this.calibrationUsed,
      });
      h.setSpeed(this.host.speed());
    }
    cancelAnimationFrame(this.frame);
    const loop = () => {
      if (token !== this.token) return;
      this.draw();
      this.frame = requestAnimationFrame(loop);
    };
    this.frame = requestAnimationFrame(loop);
  }

  setSpeed(speed: number) {
    for (const r of this.runs) r.host?.setSpeed(speed);
  }

  close() {
    this.dispose();
    this.el.classList.add("hidden");
  }

  private dispose() {
    this.token++;
    cancelAnimationFrame(this.frame);
    for (const r of this.runs) {
      r.host?.dispose();
      r.host = null;
    }
  }

  private open(run: Run) {
    if (!this.robot || !this.arena || !this.tweaksUsed) return;
    const what = { index: run.index, seed: run.seed, cubes: run.cubes.map((p) => ({ ...p })), robot: this.robot, arena: this.arena.id, tweaks: this.tweaksUsed, calibration: this.calibrationUsed };
    this.close();
    this.host.open(what);
  }

  private finish(run: Run, s: WorldSnapshot) {
    const m = s.match;
    run.result = {
      index: run.index,
      seed: run.seed,
      score: m.scores[0] ?? 0,
      t: m.t,
      reason: m.reason || "time limit",
      small: (m.cubeScores ?? []).filter((v) => v === 3).length,
      home: (m.cubeScores ?? []).filter((v) => v === 1).length,
      penalties: m.penalties ?? 0,
      rescues: m.rescues ?? 0,
      error: run.error,
    };
    run.host?.dispose(); // free the CPU for the runs still going
    run.host = null;
    this.renderResults();
  }

  private sizeCanvases() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    for (const r of this.runs) {
      const w = Math.max(1, Math.round(r.canvas.clientWidth * dpr));
      const h = Math.max(1, Math.round(r.canvas.clientHeight * dpr));
      if (r.canvas.width !== w || r.canvas.height !== h) {
        r.canvas.width = w;
        r.canvas.height = h;
      }
    }
  }

  /** the arena floor (tape and all) once, sampled from the arena's own colours */
  private background(): HTMLCanvasElement {
    if (this.bg) return this.bg;
    const a = this.arena!;
    const e = a.extent;
    const n = 320;
    const c = el("canvas");
    c.width = c.height = n;
    const ctx = c.getContext("2d")!;
    const img = ctx.createImageData(n, n);
    for (let j = 0; j < n; j++)
      for (let i = 0; i < n; i++) {
        const col = a.color(e.x0 + ((i + 0.5) / n) * (e.x1 - e.x0), e.y1 - ((j + 0.5) / n) * (e.y1 - e.y0));
        const k = (j * n + i) * 4;
        img.data[k] = col[0];
        img.data[k + 1] = col[1];
        img.data[k + 2] = col[2];
        img.data[k + 3] = 255;
      }
    ctx.putImageData(img, 0, 0);
    this.bg = c;
    return c;
  }

  private draw() {
    if (!this.arena) return;
    const e = this.arena.extent;
    const bg = this.background();
    const fg = css("--fg") || "#11053b";
    const accent = css("--link") || "#3a88fe";
    const ok = css("--ok") || "#0b7d5d";
    const warn = css("--warn") || "#b45309";
    for (const run of this.runs) {
      const c = run.canvas;
      const ctx = c.getContext("2d");
      if (!ctx || c.width < 4) continue;
      // room for the text line above and below the arena
      const pad = Math.max(3, Math.round(c.width * 0.02));
      // small tiles (CSS container query hides most text): the arena gets the whole height
      const text = c.clientHeight > 120 ? Math.round(16 * Math.min(window.devicePixelRatio || 1, 2)) : pad;
      const size = Math.max(4, Math.min(c.width - 2 * pad, c.height - 2 * text));
      const ox = (c.width - size) / 2;
      const oy = (c.height - size) / 2;
      const sx = size / (e.x1 - e.x0);
      const X = (x: number) => ox + (x - e.x0) * sx;
      const Y = (y: number) => oy + (e.y1 - y) * sx;
      ctx.clearRect(0, 0, c.width, c.height);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(bg, ox, oy, size, size);
      // black walls
      ctx.strokeStyle = "#111217";
      ctx.lineWidth = Math.max(2, PUSH_PULL.wallThickness * sx);
      ctx.strokeRect(ox - ctx.lineWidth / 2, oy - ctx.lineWidth / 2, size + ctx.lineWidth, size + ctx.lineWidth);
      const snap = run.snap;
      // cubes: green = 3 points, amber = 1 point, white = none
      const cubes = snap?.cubes ?? run.cubes.map((p) => ({ pos: [p.x, p.y, 0.05] as [number, number, number], quat: [Math.cos(p.yaw / 2), 0, 0, Math.sin(p.yaw / 2)] as [number, number, number, number] }));
      for (const cube of cubes) {
        const pts = cubePoints(cube);
        const [w, , , z] = cube.quat;
        ctx.save();
        ctx.translate(X(cube.pos[0]), Y(cube.pos[1]));
        ctx.rotate(-2 * Math.atan2(z, w));
        const s = PUSH_PULL.cube * sx;
        ctx.fillStyle = pts === 3 ? ok : pts === 1 ? warn : "#f4f6fb";
        ctx.strokeStyle = fg;
        ctx.lineWidth = 1;
        ctx.fillRect(-s / 2, -s / 2, s, s);
        ctx.strokeRect(-s / 2, -s / 2, s, s);
        ctx.restore();
      }
      // trail and robot
      if (run.trail.length > 1) {
        ctx.strokeStyle = accent;
        ctx.globalAlpha = 0.45;
        ctx.lineWidth = Math.max(1, c.width / 260);
        ctx.beginPath();
        run.trail.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y))));
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
      const r = snap?.robots[0];
      const rx = r ? r.pos[0] : this.arena.starts[0].x;
      const ry = r ? r.pos[1] : this.arena.starts[0].y;
      const yaw = r ? Math.atan2(2 * (r.quat[0] * r.quat[3] + r.quat[1] * r.quat[2]), 1 - 2 * (r.quat[2] ** 2 + r.quat[3] ** 2)) : this.arena.starts[0].yaw;
      const b = 0.098 * sx;
      ctx.save();
      ctx.translate(X(rx), Y(ry));
      ctx.rotate(-yaw);
      ctx.fillStyle = accent;
      ctx.strokeStyle = fg;
      ctx.lineWidth = 1;
      ctx.fillRect(-b / 2, -b / 2, b, b);
      ctx.strokeRect(-b / 2, -b / 2, b, b);
      ctx.fillStyle = fg;
      ctx.fillRect(b / 2 - Math.max(2, b * 0.18), -b / 2, Math.max(2, b * 0.18), b); // the blade: front
      ctx.restore();
      // the text around it
      const m = snap?.match;
      const score = run.result?.score ?? m?.scores[0] ?? 0;
      run.score.textContent = String(score);
      let foot = "starting…";
      let tone = "";
      if (run.result) {
        foot = `${run.result.t.toFixed(1)} s · ${shortReason(run.result.reason)}`;
        tone = "done";
      } else if (m) {
        const stuck = m.stuck ?? 0;
        foot = m.phase === "waiting" ? (m.calWindow ? `calibration ${m.calWindow}/3` : "waiting for A") : `${m.t.toFixed(1)} s${stuck >= STUCK_WARN_S ? ` · STUCK ${Math.floor(stuck)} s` : ""}${m.rescues ? " · freed once" : ""}`;
        tone = stuck >= STUCK_WARN_S ? "stuck" : "";
      }
      if (run.error) {
        foot += " · Python error";
        tone = "error";
      }
      run.foot.textContent = foot;
      run.foot.dataset.tone = tone;
      run.tile.classList.toggle("hl", this.highlight !== null && run.result?.score === this.highlight);
    }
    const done = this.runs.filter((r) => r.result).length;
    const t = Math.max(0, ...this.runs.map((r) => r.result?.t ?? r.snap?.match.t ?? 0));
    this.status.textContent = done === this.runs.length ? `all ${done} done` : `${done}/${this.runs.length} done · ${t.toFixed(0)} / ${this.arena.timeLimit || "∞"} s`;
  }

  private renderResults() {
    const results = this.runs.flatMap((r) => (r.result ? [r.result] : []));
    const s = summarizeRuns(results);
    const box = this.results;
    box.replaceChildren();
    box.append(el("div", "rr-title", `Results · ${results.length}/${this.runs.length}`));
    if (!results.length) {
      box.append(el("p", "muted", "Each run ends at 120 s, with a green blink, stuck twice or leaving the arena. Results appear here as they finish."));
      return;
    }
    const stats = el("div", "rr-stats");
    const stat = (label: string, value: string) => {
      const d = el("div");
      d.append(el("b", undefined, value), el("span", undefined, label));
      return d;
    };
    stats.append(stat("mean", s.mean.toFixed(1)), stat("median", String(s.median)), stat("best", String(s.best?.score ?? "–")), stat("worst", String(s.worst?.score ?? "–")));
    box.append(stats);
    box.append(this.chart(results, s.mean));
    const list = el("div", "rr-list");
    const jump = (label: string, r: RunResult | null) => {
      if (!r) return;
      const b = el("button", "link-btn", `${label}: #${r.index + 1} · ${r.score} pts · ${r.t.toFixed(1)} s`);
      b.title = `Watch run ${r.index + 1} (seed ${r.seed}) in the 3D view`;
      b.onclick = () => this.open(this.runs[r.index]);
      list.append(b);
    };
    jump("best", s.best);
    jump("worst", s.worst);
    list.append(el("div", "muted", s.reasons.map(([r, c]) => `${shortReason(r)} ${c}`).join(" · ")));
    const errors = results.filter((r) => r.error).length;
    if (errors) list.append(el("div", "rr-error", `${errors} run(s) with a Python error`));
    box.append(list);
  }

  /** Histogram: how many runs ended with each number of points (−3 … 13) */
  private chart(results: RunResult[], mean: number): SVGSVGElement {
    const NS = "http://www.w3.org/2000/svg";
    const svgEl = <K extends keyof SVGElementTagNameMap>(tag: K, attrs: Record<string, string | number>) => {
      const e = document.createElementNS(NS, tag);
      for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v));
      return e;
    };
    const lo = Math.min(MIN_POINTS, ...results.map((r) => r.score));
    const hi = Math.max(MAX_POINTS, ...results.map((r) => r.score));
    const values = Array.from({ length: hi - lo + 1 }, (_, i) => lo + i);
    const counts = values.map((v) => results.filter((r) => r.score === v).length);
    const most = Math.max(1, ...counts);
    const W = 200, H = 126, top = 14, base = 92, left = 2, gap = 2;
    const step = (W - 2 * left) / values.length;
    const bw = step - gap;
    const svg = svgEl("svg", { viewBox: `0 0 ${W} ${H}`, class: "rr-chart", role: "img", "aria-label": `Points per run: ${values.flatMap((v, i) => (counts[i] ? [`${v} points ${counts[i]}×`] : [])).join(", ")}` });
    svg.append(svgEl("line", { x1: 0, x2: W, y1: base + 0.5, y2: base + 0.5, class: "rr-axis" }));
    values.forEach((v, i) => {
      const x = left + i * step + gap / 2;
      const g = svgEl("g", { class: "rr-bar" });
      const title = svgEl("title", {});
      title.textContent = `${v} points: ${counts[i]} run${counts[i] === 1 ? "" : "s"}`;
      g.append(title, svgEl("rect", { x: x - gap / 2, y: top - 4, width: step, height: base - top + 4, class: "rr-hit" }));
      if (counts[i]) {
        const h = Math.max(3, ((base - top) * counts[i]) / most);
        const r = Math.min(4, bw / 2, h);
        const y = base - h;
        g.append(svgEl("path", { d: `M${x},${base} V${y + r} Q${x},${y} ${x + r},${y} H${x + bw - r} Q${x + bw},${y} ${x + bw},${y + r} V${base} Z`, class: "rr-fill" }));
        const label = svgEl("text", { x: x + bw / 2, y: y - 3, class: "rr-count" });
        label.textContent = String(counts[i]);
        g.append(label);
      }
      if ((v - lo) % 2 === 0) {
        const t = svgEl("text", { x: x + bw / 2, y: base + 11, class: "rr-tick" });
        t.textContent = String(v);
        g.append(t);
      }
      g.addEventListener("mouseenter", () => (this.highlight = counts[i] ? v : null));
      g.addEventListener("mouseleave", () => (this.highlight = null));
      svg.append(g);
    });
    const mx = left + (mean - lo) * step + step / 2;
    svg.append(svgEl("line", { x1: mx, x2: mx, y1: top - 6, y2: base, class: "rr-mean" }));
    const ml = svgEl("text", { x: Math.min(W - 24, Math.max(24, mx)), y: H - 2, class: "rr-mean-label" });
    ml.textContent = `⌀ ${mean.toFixed(1)} points`;
    svg.append(ml);
    return svg;
  }
}
