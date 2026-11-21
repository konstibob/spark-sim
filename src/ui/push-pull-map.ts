import type { Pose } from "../shared/arenas.ts";
import type { WorldSnapshot } from "../shared/protocol.ts";
import { blocksStart, cubeFootprint, cubePoints, cubesFromSeed, DEFAULT_CUBES, PUSH_PULL, validCubeLayout, type CubeView } from "../shared/push-pull.ts";

/** Editable start map; the live view always comes from physics, never a second simulation. */
export class PushPullMap {
  readonly dialog = document.createElement("dialog");
  private canvas!: HTMLCanvasElement;
  private inputs: HTMLInputElement[][] = [];
  private status!: HTMLElement;
  private error!: HTMLElement;
  private editable = true;
  private zoom = false;
  private snap: WorldSnapshot | null = null;
  private drag: number | null = null;
  private selected = 0;
  private draft: Pose[] | null = null;
  private controls!: HTMLFieldSetElement;
  private seedInput!: HTMLInputElement;

  constructor(private layout: () => Pose[], private seed: () => number | null, private change: (poses: Pose[], seed: number | null) => void, private referee: (action: "rescue" | "touch" | "finish") => void) {
    this.dialog.className = "pp-dialog";
    this.dialog.setAttribute("aria-label", "Challenge 2 · Arena-Karte");
    this.dialog.innerHTML = `
      <header class="pp-heading"><div><span class="label">CHALLENGE 2</span><h2>Push & Pull</h2></div><button class="btn" data-close aria-label="Karte schließen">Schließen</button></header>
      <div class="pp-specs"><span><b>160 × 160</b> cm Arena</span><span><b>30 × 30</b> cm Zuhause</span><span><b>12 × 12</b> cm Start</span><span><b>5 × 6</b> cm Würfel</span></div>
      <div class="pp-layout"><div class="pp-map-wrap"><canvas width="640" height="640" tabindex="0" aria-label="Arena-Karte. Würfel vor dem Lauf ziehen oder Koordinaten rechts eingeben."></canvas><button class="btn" data-zoom>Zentrum vergrößern</button><span class="hint">Oben = +y · rechts = +x · Start in Richtung +x</span></div>
      <div class="pp-settings"><p data-status></p><fieldset><legend>Startaufstellung · Koordinaten in cm</legend><table><thead><tr><th>Würfel</th><th>x</th><th>y</th><th>°</th></tr></thead><tbody></tbody></table><div class="pp-actions pp-seed"><label>Seed <input type="number" min="1" max="999999" step="1" data-seed aria-label="Seed der Würfelaufstellung"></label><button class="btn" data-random title="Neue zufällige Aufstellung; derselbe Seed ergibt immer dieselbe">Neuer Seed</button></div><div class="pp-actions"><button class="btn" data-default>Beispiel</button><button class="btn" data-export>JSON speichern</button><button class="btn" data-import>JSON laden</button><input type="file" accept=".json,application/json" hidden data-file></div></fieldset>
      <p class="pp-error" role="status" data-error></p>
      <p>Würfel vor dem Lauf ziehen. Gleiche Aufstellung für jeden Versuch; <b>Reset</b> setzt sie zurück. Die offiziellen Würfelpositionen sind noch unbekannt.</p>
      <p>Das 2,5-cm-Band zählt zur Fläche. <b>3 Punkte</b> vollständig im kleinen Quadrat samt Band (17 × 17 cm). <b>1 Punkt</b> mindestens halb im großen Quadrat samt Band (35 × 35 cm). Vier Würfel passen mit Luft hinein: <b>13 Punkte</b>; theoretisch fünf (einer um 45° gedreht): 15.</p>
      <details><summary>Modell & Schiedsrichter</summary><p>Schwarze Wände: 10 cm hoch. Band außen an den Zielquadraten. Band 2,5 cm. Würfel 140 g (schwer) und 40 g (leicht): pro Lauf zufällig 2 schwere + 3 leichte (75 %) oder 3 schwere + 2 leichte (25 %); die Gewichte stehen im Log und während des Laufs auf der Karte. Roboter +8 g für das ESP32-C6-Board (geschätzt). Annahme: Reibwert 0,35. IR-Werte sind Näherungen. Keine Greifvorrichtung; Schieben mit der Zumo-Schaufel.</p><p>Start sofort mit A; 120 Sekunden. Zwei grüne Blinkimpulse im Abstand von 0,1–2 s beenden den Lauf. Frühstarts werden gemeldet; den offiziellen Neustart beurteilst du selbst. Feststecken prüft die Sim automatisch: nach 5 s ohne Bewegung (unter 3 cm, unter 30° Drehung) eine Warnung, nach 10 s hebt der Schiedsrichter den Roboter hoch (1 s in der Luft, Liniensensoren sehen nichts) und setzt ihn ins Startquadrat in Startrichtung (−1; liegt dort ein Würfel, an die nächste freie Stelle daneben); beim zweiten Mal endet der Lauf.</p><div class="pp-actions" data-referee><button class="btn" data-action="rescue">Befreien −1</button><button class="btn" data-action="touch">Berührt −1 & Ende</button><button class="btn" data-action="finish">Grün bestätigt</button></div><p>Nach „Befreien“ Karte schließen und Roboter mit Shift+Ziehen versetzen. Zweites Befreien beendet den Lauf.</p></details>
      </div></div>`;
    document.body.append(this.dialog);
    this.canvas = this.dialog.querySelector("canvas")!;
    this.status = this.dialog.querySelector("[data-status]")!;
    this.error = this.dialog.querySelector("[data-error]")!;
    this.controls = this.dialog.querySelector("fieldset")!;
    const rows = this.dialog.querySelector("tbody")!;
    for (let i = 0; i < 5; i++) {
      const row = document.createElement("tr");
      const label = document.createElement("th"); label.textContent = String(i + 1); row.append(label);
      const inputs = ["x", "y", "Winkel"].map((axis, k) => {
        const cell = document.createElement("td");
        const input = document.createElement("input");
        input.type = "number"; input.step = k === 2 ? "1" : "0.5"; input.min = k === 2 ? "-360" : "-77"; input.max = k === 2 ? "360" : "77";
        input.setAttribute("aria-label", `Würfel ${i + 1} ${axis}`);
        input.onchange = () => {
          const poses = this.layout().map((p) => ({ ...p }));
          const values = this.inputs[i].map((v) => v.valueAsNumber);
          poses[i] = { x: values[0] / 100, y: values[1] / 100, yaw: values[2] * Math.PI / 180 };
          this.commit(poses);
        };
        cell.append(input); row.append(cell); return input;
      });
      this.inputs.push(inputs); rows.append(row);
    }
    const button = (selector: string, action: () => void) => { this.dialog.querySelector<HTMLButtonElement>(selector)!.onclick = action; };
    button("[data-close]", () => this.dialog.close());
    button("[data-zoom]", () => { this.zoom = !this.zoom; this.dialog.querySelector("[data-zoom]")!.textContent = this.zoom ? "Gesamte Arena" : "Zentrum vergrößern"; this.draw(); });
    button("[data-default]", () => this.commit(DEFAULT_CUBES.map((p) => ({ ...p }))));
    this.seedInput = this.dialog.querySelector<HTMLInputElement>("[data-seed]")!;
    this.seedInput.onchange = () => {
      const n = Math.round(this.seedInput.valueAsNumber);
      if (Number.isFinite(n) && n >= 1) this.commit(cubesFromSeed(n), n);
      else this.syncInputs();
    };
    button("[data-random]", () => { const n = 1 + Math.floor(Math.random() * 999_999); this.commit(cubesFromSeed(n), n); });
    button("[data-export]", () => {
      const blob = new Blob([JSON.stringify({ version: 1, arena: "push-pull", units: "metres", cubes: this.layout() }, null, 2)], { type: "application/json" });
      const a = document.createElement("a"); a.href = URL.createObjectURL(blob); a.download = "push-pull-map.json"; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    });
    const file = this.dialog.querySelector<HTMLInputElement>("[data-file]")!;
    button("[data-import]", () => file.click());
    file.onchange = async () => {
      try { const value = JSON.parse(await file.files![0].text()); this.commit(value.cubes); }
      catch { this.error.textContent = "Diese JSON-Datei konnte nicht gelesen werden."; }
      file.value = "";
    };
    this.dialog.querySelectorAll<HTMLButtonElement>("[data-action]").forEach((b) => { b.onclick = () => this.referee(b.dataset.action as "rescue" | "touch" | "finish"); });
    this.dialog.addEventListener("click", (e) => { if (e.target === this.dialog) { const r = this.dialog.getBoundingClientRect(); if (e.clientX < r.left || e.clientX > r.right || e.clientY < r.top || e.clientY > r.bottom) this.dialog.close(); } });
    const point = (e: PointerEvent) => {
      const r = this.canvas.getBoundingClientRect(); const extent = this.zoom ? 0.22 : 0.94;
      return { x: ((e.clientX - r.left) / r.width * 2 - 1) * extent, y: (1 - (e.clientY - r.top) / r.height * 2) * extent };
    };
    this.canvas.onpointerdown = (e) => {
      if (!this.editable) return;
      const p = point(e); const poses = this.layout();
      const i = poses.findIndex((q) => Math.hypot(p.x - q.x, p.y - q.y) < 0.055);
      if (i < 0) return;
      this.drag = this.selected = i; this.draft = poses.map((q) => ({ ...q })); this.canvas.setPointerCapture(e.pointerId);
    };
    this.canvas.onpointermove = (e) => {
      if (this.drag === null || !this.draft) return;
      const p = point(e); const old = this.draft[this.drag];
      const radius = PUSH_PULL.cube / 2 * (Math.abs(Math.cos(old.yaw)) + Math.abs(Math.sin(old.yaw)));
      const clamp = (n: number) => Math.max(-0.8 + radius, Math.min(0.8 - radius, Math.round(n * 200) / 200));
      this.draft[this.drag] = { ...old, x: clamp(p.x), y: clamp(p.y) }; this.draw();
    };
    this.canvas.onpointerup = () => { if (this.draft) this.commit(this.draft); this.drag = null; this.draft = null; this.draw(); };
    this.canvas.onpointercancel = () => { this.drag = null; this.draft = null; this.draw(); };
  }

  /** seed: the layout came from cubesFromSeed(seed); null = placed by hand / example / file */
  private commit(poses: unknown, seed: number | null = null) {
    if (!this.editable) return;
    if (!validCubeLayout(poses)) { this.error.textContent = "Fünf Würfel nötig: innerhalb der Wände, ohne Überlappung."; this.draw(); return; }
    // The actual robot occupies the small square at the start.
    if (poses.some(blocksStart)) {
      this.error.textContent = "Die Startfläche des Roboters bitte frei lassen."; this.draw(); return;
    }
    this.error.textContent = ""; this.snap = null; this.change(poses.map((p) => ({ ...p })), seed); this.syncInputs(); this.draw();
  }

  open() { this.syncInputs(); this.dialog.showModal(); this.draw(); }
  update(snap: WorldSnapshot, editable: boolean) {
    this.snap = snap; this.editable = editable; this.controls.disabled = !editable;
    if (this.dialog.open) this.draw();
    this.dialog.querySelectorAll<HTMLButtonElement>("[data-action]").forEach((b) => { b.disabled = snap.match.phase !== "running" || snap.match.ended; });
  }
  private syncInputs() {
    const seed = this.seed();
    this.seedInput.value = seed === null ? "" : String(seed);
    this.seedInput.placeholder = seed === null ? "eigene" : "";
    this.layout().forEach((p, i) => { [p.x * 100, p.y * 100, p.yaw * 180 / Math.PI].forEach((v, k) => { this.inputs[i][k].value = String(+v.toFixed(2)); }); });
  }
  private draw() {
    const ctx = this.canvas.getContext("2d")!; const W = this.canvas.width; const E = this.zoom ? 0.22 : 0.94;
    const scale = W / (2 * E); const px = (v: number) => W / 2 + v * scale; const py = (v: number) => W / 2 - v * scale;
    ctx.fillStyle = "#e9eef5"; ctx.fillRect(0, 0, W, W);
    ctx.fillStyle = "#fcfcfc"; ctx.fillRect(px(-0.8), py(0.8), 1.6 * scale, 1.6 * scale);
    ctx.strokeStyle = "#dce3ec"; ctx.lineWidth = 1;
    for (let n = -0.8; n < 0.81; n += 0.1) { ctx.beginPath(); ctx.moveTo(px(n), py(-0.8)); ctx.lineTo(px(n), py(0.8)); ctx.moveTo(px(-0.8), py(n)); ctx.lineTo(px(0.8), py(n)); ctx.stroke(); }
    ctx.strokeStyle = "#181a22"; ctx.lineWidth = PUSH_PULL.wallThickness * scale;
    ctx.strokeRect(px(-0.8) - ctx.lineWidth / 2, py(0.8) - ctx.lineWidth / 2, 1.6 * scale + ctx.lineWidth, 1.6 * scale + ctx.lineWidth);
    for (const [size, fill] of [[0.3, "#dbeafe"], [0.12, "#b9eedc"]] as const) {
      ctx.fillStyle = fill; ctx.fillRect(px(-size / 2), py(size / 2), size * scale, size * scale);
      ctx.strokeStyle = "#171922"; ctx.lineWidth = PUSH_PULL.tape * scale;
      ctx.strokeRect(px(-size / 2 - PUSH_PULL.tape / 2), py(size / 2 + PUSH_PULL.tape / 2), (size + PUSH_PULL.tape) * scale, (size + PUSH_PULL.tape) * scale);
    }
    ctx.fillStyle = "#4c6077"; ctx.font = `${this.zoom ? 18 : 16}px sans-serif`; ctx.textAlign = "center";
    if (!this.zoom) { ctx.fillText("160 cm", W / 2, py(0.86)); ctx.save(); ctx.translate(px(-0.87), W / 2); ctx.rotate(-Math.PI / 2); ctx.fillText("160 cm", 0, 0); ctx.restore(); }
    ctx.textAlign = "left"; ctx.fillText("30 × 30 cm + Band · 1 P", px(0.185), py(0.185));
    ctx.fillText("12 × 12 cm + Band · 3 P", px(0.095), py(-0.11));
    const cubes = this.draft ? this.draft.map((p) => ({ pos: [p.x, p.y, 0.05] as [number, number, number], quat: [Math.cos(p.yaw / 2), 0, 0, Math.sin(p.yaw / 2)] as [number, number, number, number] })) : this.snap?.cubes ?? this.layout().map((p) => ({ pos: [p.x, p.y, 0.05] as [number, number, number], quat: [Math.cos(p.yaw / 2), 0, 0, Math.sin(p.yaw / 2)] as [number, number, number, number] }));
    cubes.forEach((cube, i) => {
      const pts = cubeFootprint(cube); ctx.beginPath(); pts.forEach(([x, y], k) => k ? ctx.lineTo(px(x), py(y)) : ctx.moveTo(px(x), py(y))); ctx.closePath();
      ctx.fillStyle = cubePoints(cube) === 3 ? "#75d6ad" : "#e5edf7"; ctx.strokeStyle = i === this.selected ? "#2474e7" : "#56708c"; ctx.lineWidth = 2; ctx.fill(); ctx.stroke();
      ctx.fillStyle = "#162c47"; ctx.font = `bold ${this.zoom ? 20 : 16}px sans-serif`; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(String(i + 1), px(cube.pos[0]), py(cube.pos[1]));
      // masses are drawn per run (seed), so only shown once it runs
      const mass = (cube as CubeView).mass;
      if (!this.editable && mass) { ctx.font = `${this.zoom ? 15 : 12}px sans-serif`; ctx.fillText(`${Math.round(mass * 1000)} g`, px(cube.pos[0]), py(cube.pos[1] - PUSH_PULL.cube * 0.75) + (this.zoom ? 0 : 6)); }
    });
    const robot = this.snap?.robots[0]; const [rx, ry] = robot?.pos ?? [0, 0]; const yaw = (robot?.yaw ?? 0) * Math.PI / 180;
    ctx.save(); ctx.translate(px(rx), py(ry)); ctx.rotate(-yaw); ctx.fillStyle = "#3483f5"; ctx.fillRect(-0.05 * scale, -0.049 * scale, 0.1 * scale, 0.098 * scale); ctx.fillStyle = "#ffffff"; ctx.beginPath(); ctx.moveTo(0.045 * scale, 0); ctx.lineTo(0.005 * scale, -0.024 * scale); ctx.lineTo(0.005 * scale, 0.024 * scale); ctx.fill(); ctx.restore();
    ctx.textBaseline = "alphabetic";
    this.status.textContent = this.editable ? `Aufstellung bearbeiten · ${this.seed() === null ? "eigene Aufstellung" : `Seed ${this.seed()}`}` : `${this.snap?.match.phase === "over" ? "Ergebnis" : "Live-Positionen"} · ${this.snap?.match.scores[0] ?? 0} Punkte · ${(this.snap?.match.t ?? 0).toFixed(1)} / 120 s`;
  }
}
