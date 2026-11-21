// Match rules per arena mode. Runs inside the world on every physics step, so the browser, the test bench and
// headless Node runs judge identically. The same functions are meant to judge real matches from camera
// tracking later (positions in, events and scores out).
//
// Like a real bout, sumo, King of the Hill and shuttle start with a start signal (the referee: every robot's
// button A is pressed). Robots may move 5 s after it ("go"); the match clock and the time limit count from go.
// The race has no signal: its laps are timed per robot from its first crossing of the line.

import type { Arena } from "../shared/arenas.ts";
import { cubePoints, type CubeView } from "../shared/push-pull.ts";

export type Phase = "waiting" | "countdown" | "running" | "over";

export interface MatchState {
  mode: Arena["mode"];
  /** waiting for the start signal · countdown (5 s) · running · over */
  phase: Phase;
  /** sim time of the start signal (s); null = not given yet */
  signal: number | null;
  /** seconds left until go (countdown phase) */
  countdown: number;
  /** match clock: seconds since go */
  t: number;
  timeLimit: number;
  ended: boolean;
  /** per robot: sumo 1 = winner, race completed laps, koth seconds in the zone, shuttle points */
  scores: number[];
  /** per robot short status line */
  detail: string[];
  /** winning slot, null = draw or still running */
  winner: number | null;
  /** why the match ended */
  reason: string;
  /** race: lap times per robot (s) */
  laps: number[][];
  /** robots that are out (fell off / left the ring) */
  out: boolean[];
  cubeScores?: number[];
  penalties?: number;
  rescues?: number;
  /** Push & Pull: seconds the robot has not really moved (the referee frees it at STUCK_S) */
  stuck?: number;
}

export interface RuleEvent {
  kind: "out" | "lap" | "point" | "end" | "warn" | "info";
  slot?: number;
  text: string;
}

export interface RobotView {
  x: number;
  y: number;
  z: number;
  out: boolean;
  /** motors commanded non-zero */
  moving: boolean;
  name: string;
  green?: boolean;
  /** heading (rad); lets the stuck rule tell turning on the spot from standing still */
  yaw?: number;
}

/** rules art. 8: robots may start operating 5 s after the start signal */
export const START_DELAY_S = 5;

/**
 * Push & Pull, "The robot is stuck for 10 seconds": the referee frees it once (−1 point); stuck again ends the
 * round. The page does not define "stuck"; here it means the robot stayed within STUCK_MOVE_M of one spot and
 * turned less than STUCK_TURN_RAD for STUCK_S seconds, whether its motors run or not (pushing against a wall,
 * wedged on a cube, or standing still). Turning on the spot is not stuck.
 */
export const STUCK_S = 10;
export const STUCK_WARN_S = 5;
export const STUCK_MOVE_M = 0.03;
export const STUCK_TURN_RAD = (30 * Math.PI) / 180;

// Extra per-mode scoring, registered by the private competition overlay (koth, shuttle). The core handles
// sumo / race / free; any other mode is dispatched to its registered handler, which keeps the Challenge-1
// scoring out of the public build.
export interface RuleContext {
  arena: Arena;
  robots: RobotView[];
  state: MatchState;
  running: boolean;
  dt: number;
  events: RuleEvent[];
  /** per-match scratch array (e.g. the shuttle's last base), reset with the match */
  scratch: number[];
  end(winner: number | null, reason: string): void;
  leader(): number | null;
}
export type RuleHandler = (ctx: RuleContext) => void;
const ruleHandlers: Record<string, RuleHandler> = {};
export function registerRuleHandler(mode: string, handler: RuleHandler): void {
  ruleHandlers[mode] = handler;
}

export class Rules {
  state: MatchState;
  private prev: { x: number; y: number }[] = [];
  private lastBase: number[] = [];
  private lapStart: (number | null)[] = [];
  private visited: Set<number>[] = [];
  private shortcutWarned: boolean[] = [];
  private falseStart: boolean[] = [];
  private go: number | null;
  private greenOn = false;
  private greenAt: number | null = null;
  private stuckAnchor: { x: number; y: number; yaw: number } | null = null;
  private stuckWarned = false;
  /** slot the referee should free (set down elsewhere), taken by the world with takeFree() */
  private freeRequest: number | null = null;

  constructor(
    private arena: Arena,
    n: number,
  ) {
    const race = arena.mode === "race" || arena.mode === "free";
    this.go = race ? 0 : null;
    this.state = {
      mode: arena.mode,
      phase: race ? "running" : "waiting",
      signal: null,
      countdown: 0,
      t: 0,
      timeLimit: arena.timeLimit,
      ended: false,
      scores: new Array(n).fill(0),
      detail: new Array(n).fill(""),
      winner: null,
      reason: "",
      laps: Array.from({ length: n }, () => []),
      out: new Array(n).fill(false),
      ...(arena.mode === "push-pull" ? { cubeScores: [], penalties: 0, rescues: 0 } : {}),
    };
    this.lastBase = new Array(n).fill(-1);
    this.lapStart = new Array(n).fill(null);
    this.visited = Array.from({ length: n }, () => new Set<number>());
    this.shortcutWarned = new Array(n).fill(false);
    this.falseStart = new Array(n).fill(false);
  }

  /** The start signal at sim time t (s). Only the first one counts. */
  signal(t: number): RuleEvent[] {
    const s = this.state;
    if (s.signal !== null || this.go !== null || s.ended) return [];
    s.signal = t;
    const delay = this.arena.mode === "push-pull" ? 0 : START_DELAY_S;
    this.go = t + delay;
    s.phase = delay ? "countdown" : "running";
    s.countdown = delay;
    return [{ kind: "info", text: delay ? `Start signal — the robots may move in ${delay} s` : "Start signal — button A, go immediately!" }];
  }

  /** A robot that was out is back in the arena (moved there by hand). */
  revive(slot: number) {
    if (!this.state.ended && this.state.out[slot]) this.state.out[slot] = false;
  }

  /** Advance by dt seconds with the current robot positions. Returns events for the console. */
  step(t: number, dt: number, robots: RobotView[], cubes: CubeView[] = []): RuleEvent[] {
    const s = this.state;
    const ev: RuleEvent[] = [];
    if (s.ended) return ev;
    const a = this.arena;
    const go = this.go;
    const running = go !== null && t >= go;
    if (go === null) {
      s.phase = "waiting";
    } else if (!running) {
      s.phase = "countdown";
      s.countdown = go - t;
    } else {
      if (s.phase !== "running") {
        s.phase = "running";
        s.countdown = 0;
        if (s.signal !== null) ev.push({ kind: "info", text: "Go!" });
      }
      s.t = t - go;
    }

    // Out of the arena: always reported, in every mode
    robots.forEach((r, i) => {
      if (r.out && !s.out[i]) {
        s.out[i] = true;
        ev.push({ kind: "out", slot: i, text: `${r.name} is out (${t.toFixed(2)} s)` });
      }
    });

    switch (a.mode) {
      case "push-pull": {
        s.cubeScores = cubes.map(cubePoints);
        s.scores[0] = s.cubeScores.reduce((sum, v) => sum + v, 0) - (s.penalties ?? 0);
        const small = s.cubeScores.filter((v) => v === 3).length;
        const home = s.cubeScores.filter((v) => v === 1).length;
        const r = robots[0];
        if (r && running && !s.ended) this.judgeStuck(ev, r, dt);
        const stuck = (s.stuck ?? 0) >= STUCK_WARN_S ? ` · STUCK ${Math.floor(s.stuck!)} s` : "";
        s.detail[0] = `${small} small · ${home} home · ${s.penalties ?? 0} penalty${stuck}`;
        if (r?.moving && !running && !this.falseStart[0]) {
          this.falseStart[0] = true;
          ev.push({ kind: "warn", slot: 0, text: "False start: motors moved before A. Officially: restart with −1; a second false start scores 0. Reset this practice run." });
        }
        if (running && s.out[0]) {
          s.penalties = (s.penalties ?? 0) + 2;
          s.scores[0] -= 2;
          this.end(ev, 0, "left the arena (−2 points)");
        }
        const green = !!r?.green;
        if (running && green && !this.greenOn) {
          if (this.greenAt !== null && t - this.greenAt >= 0.1 && t - this.greenAt <= 2) this.end(ev, 0, "finished — green blink signal");
          this.greenAt = t;
        }
        this.greenOn = green;
        break;
      }
      case "sumo":
        robots.forEach((r, i) => {
          if (r.moving && !running && !this.falseStart[i]) {
            this.falseStart[i] = true;
            ev.push({
              kind: "warn",
              slot: i,
              text: `${r.name}: false start — moved ${s.signal === null ? "before the start signal" : `${(t - s.signal).toFixed(2)} s after the start signal`} (rules art. 8: wait ${START_DELAY_S} s)`,
            });
          }
        });
        {
          const alive = robots.map((_, i) => i).filter((i) => !s.out[i]);
          s.detail = robots.map((_, i) => (s.out[i] ? "OUT" : this.falseStart[i] ? "FALSE START" : ""));
          if (robots.length > 1 && alive.length <= 1) {
            const w = alive.length === 1 ? alive[0] : null;
            if (w !== null) s.scores[w] = 1;
            this.end(ev, w, w === null ? "both out" : `${robots[w].name} pushed the rest out`);
          }
        }
        break;

      case "race": {
        const f = a.finish!;
        const cps = a.checkpoints ?? [];
        const need = a.laps ?? 3;
        robots.forEach((r, i) => {
          cps.forEach((c, k) => r.x >= c.x0 && r.x <= c.x1 && r.y >= c.y0 && r.y <= c.y1 && this.visited[i].add(k));
          const p = this.prev[i];
          if (p && !s.out[i] && s.laps[i].length < need && p.x < f.x && r.x >= f.x && r.y >= f.y0 && r.y <= f.y1) {
            const start = this.lapStart[i];
            if (start === null) {
              this.lapStart[i] = t; // flying start: each robot's clock runs from its first crossing
              ev.push({ kind: "lap", slot: i, text: `${r.name} crossed the start line — timing` });
            } else if (this.visited[i].size < cps.length) {
              if (!this.shortcutWarned[i]) ev.push({ kind: "warn", slot: i, text: `${r.name} crossed the line without driving the whole loop — no lap` });
              this.shortcutWarned[i] = true;
            } else {
              const lap = t - start;
              s.laps[i].push(lap);
              s.scores[i] = s.laps[i].length;
              this.lapStart[i] = t;
              this.shortcutWarned[i] = false;
              const total = s.laps[i].reduce((x, y) => x + y, 0);
              ev.push({
                kind: "lap",
                slot: i,
                text: `${r.name} lap ${s.laps[i].length}: ${lap.toFixed(2)} s${s.laps[i].length >= need ? ` — finished in ${total.toFixed(2)} s` : ""}`,
              });
            }
            this.visited[i].clear();
          }
          const best = s.laps[i].length ? Math.min(...s.laps[i]) : null;
          const total = s.laps[i].reduce((x, y) => x + y, 0);
          s.detail[i] = s.out[i]
            ? "OUT"
            : s.laps[i].length >= need
              ? `FINISHED · ${total.toFixed(2)} s`
              : `LAP ${s.laps[i].length}/${need}${best !== null ? ` · BEST ${best.toFixed(2)} s` : ""}`;
        });
        // the race is over when every robot still on the track has finished; the fastest total time wins
        const racing = robots.map((_, i) => i).filter((i) => !s.out[i]);
        if (racing.length === 0) this.end(ev, this.raceLeader(), "everyone left the track");
        else if (racing.every((i) => s.laps[i].length >= need)) {
          const w = this.raceLeader();
          this.end(ev, w, w === null ? "same time" : `${robots[w].name} has the fastest ${need} laps`);
        }
        break;
      }

      case "free":
        s.detail = robots.map((_, i) => (s.out[i] ? "OUT" : ""));
        break;

      default: {
        // koth, shuttle, … scoring lives in the competition overlay and is run here if registered
        const handler = ruleHandlers[a.mode];
        if (handler)
          handler({
            arena: a,
            robots,
            state: s,
            running,
            dt,
            events: ev,
            scratch: this.lastBase,
            end: (winner, reason) => this.end(ev, winner, reason),
            leader: () => this.leader(),
          });
        break;
      }
    }

    if (!s.ended && running && s.timeLimit > 0 && s.t >= s.timeLimit) {
      if (a.mode === "sumo") this.end(ev, null, "time limit — no decision");
      else if (a.mode === "race") this.end(ev, this.raceLeader(), "time limit");
      else this.end(ev, this.leader(), "time limit");
    }
    this.prev = robots.map((r) => ({ x: r.x, y: r.y }));
    return ev;
  }

  /** Slot with the highest score, null on a tie */
  referee(action: "rescue" | "touch" | "finish"): RuleEvent[] {
    const s = this.state;
    const ev: RuleEvent[] = [];
    if (s.mode !== "push-pull" || s.phase !== "running" || s.ended) return ev;
    if (action === "rescue") this.rescue(ev, "Referee rescue (−1 point). Shift+drag the robot free; a second rescue ends the round.");
    else {
      if (action === "touch") { s.penalties = (s.penalties ?? 0) + 1; s.scores[0] -= 1; }
      this.end(ev, 0, action === "finish" ? "finished — referee confirmed green signal" : "touched robot or cube (−1 point)");
    }
    return ev;
  }

  /** Take the pending "free the robot" request (the world sets the robot down elsewhere). */
  takeFree(): number | null {
    const slot = this.freeRequest;
    this.freeRequest = null;
    return slot;
  }

  private judgeStuck(ev: RuleEvent[], r: RobotView, dt: number) {
    const s = this.state;
    const yaw = r.yaw ?? 0;
    const a = this.stuckAnchor;
    const turned = a ? Math.abs(Math.atan2(Math.sin(yaw - a.yaw), Math.cos(yaw - a.yaw))) : 0;
    if (!a || Math.hypot(r.x - a.x, r.y - a.y) > STUCK_MOVE_M || turned > STUCK_TURN_RAD) {
      this.stuckAnchor = { x: r.x, y: r.y, yaw };
      this.stuckWarned = false;
      s.stuck = 0;
      return;
    }
    s.stuck = (s.stuck ?? 0) + dt;
    if (s.stuck >= STUCK_WARN_S && !this.stuckWarned) {
      this.stuckWarned = true;
      ev.push({ kind: "warn", slot: 0, text: `${r.name} has not moved for ${STUCK_WARN_S} s — at ${STUCK_S} s the referee frees it (${s.rescues ? "second time: round over" : "−1 point"})` });
    }
    if (s.stuck >= STUCK_S) {
      this.stuckAnchor = null;
      s.stuck = 0;
      const first = !s.rescues;
      this.rescue(ev, `${r.name} stuck for ${STUCK_S} s: the referee frees it (−1 point). Stuck a second time ends the round.`);
      if (first) this.freeRequest = 0;
    }
  }

  /** "We free it, once at most, and it costs you 1 point. If it gets stuck a second time, your round ends." */
  private rescue(ev: RuleEvent[], text: string) {
    const s = this.state;
    if (s.rescues) {
      this.end(ev, 0, "stuck a second time");
      return;
    }
    s.rescues = 1;
    s.penalties = (s.penalties ?? 0) + 1;
    s.scores[0] -= 1;
    ev.push({ kind: "warn", slot: 0, text });
  }

  private leader(): number | null {
    const sc = this.state.scores;
    const best = Math.max(...sc);
    const who = sc.map((v, i) => (v === best ? i : -1)).filter((i) => i >= 0);
    return who.length === 1 ? who[0] : null;
  }

  /** Race: most laps, then the shortest total time of those laps; null on a tie or without laps */
  private raceLeader(): number | null {
    const s = this.state;
    const most = Math.max(...s.laps.map((l) => l.length));
    if (most === 0) return null;
    const total = (i: number) => s.laps[i].reduce((x, y) => x + y, 0);
    const cand = s.laps.map((l, i) => (l.length === most ? i : -1)).filter((i) => i >= 0);
    const best = Math.min(...cand.map(total));
    const who = cand.filter((i) => total(i) === best);
    return who.length === 1 ? who[0] : null;
  }

  private end(ev: RuleEvent[], winner: number | null, reason: string) {
    const s = this.state;
    s.ended = true;
    s.phase = "over";
    s.winner = winner;
    s.reason = reason;
    ev.push({ kind: "end", slot: winner ?? undefined, text: winner === null ? `Match over: draw (${reason})` : `Match over: ${reason}` });
  }
}

// The Challenge-1 scoring (koth, shuttle) lives in the private competition overlay
// (overlay/rules.ts) and registers itself via registerRuleHandler().
