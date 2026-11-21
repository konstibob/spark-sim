# AGENTS.md

Context for coding agents working in this repo. Read the README for the full challenge description;
this file is the short version plus the rules for how to work here.

## What this repo is

A fork of the SPARK Zumo 2040 simulator (real MicroPython in WebAssembly + MuJoCo physics in the browser).
Team Green uses it to develop the program for **SPARK Maker Night, Challenge 2 "Push & Pull"**
(https://makernight.spark-hpi.de/en/docs/challenge-2/). The code that runs at the event is `challenge2.py`
in the private team repo **spark-hpi/team-green**. This repo is only the simulator; its job is to tell us
whether a `challenge2.py` would score well on the real arena.

## The challenge in one paragraph

Solo Zumo 2040 starts in the 12 cm tape square at the centre of a ~160 × 160 cm arena (light floor,
**black** 10 cm walls, black tape squares of 12 cm and 30 cm). Five reflective 6 cm cubes stand in the same
(unpublished) spots for every team. 2 minutes. A cube completely inside the 12 cm square = 3 points, completely
inside the 30 cm square = 1 point; the 2.5 cm tape counts as part of each square, so 4 cubes fit (13 points,
15 in theory). Green blinking ends the round early; on equal points the
shorter time wins. Penalties: early move −1 (twice = 0), stuck 10 s −1 (twice ends), leaving the arena −2,
touching −1. No radio on the robot, no remote control, only shop parts, max 500 g.

## Rules for agents

- **Do not bring up the time, the clock or how close the code freeze is unless the user asks about it.**
  The deadlines are in the README; don't use them to push priorities or add urgency to answers.
- Any MicroPython you write for the robot must run unchanged in `team-green`: keep the template's start block
  (`from spark_start import wait_for_start` + `wait_for_start()` first in `main()`), `motors.off()` in
  `finally`, top-level `main()` call (no `if __name__ == "__main__"`), only `zumo_2040_robot` + MicroPython
  stdlib. `src/examples/challenge2_template.py` is the unchanged team template.
- In the simulator, `spark_start.wait_for_start()` returns the moment button A is pressed (the sim's start
  signal stands in for the Crew's IR START). On the real robot the IR START also has no delay; button A there
  is the practice start with 5 s delay.
- Measured: tape 2.5 cm, cubes 140 g (heavy) / 40 g (light), mix drawn per seed. Still assumptions: cube friction
  0.35, IR reflectance cube 0.95 / wall 0.04 / tape 0.04 / floor 0.9, ESP board +8 g. Say so when a result
  depends on the assumptions. Teammates are measuring the real sensors; measured numbers go into `src/shared/push-pull.ts`, `src/shared/zumo-spec.ts`
  and `src/world/world.ts`.
- Stuck rule in the sim: within 3 cm and under 30° of turning for 10 s = stuck. The first time costs −1 and the
  referee lifts the robot (1 s, line sensors read 1024) and puts it back in the start square in the start
  direction; the second time ends the round. A good program notices the lift and localises again from the start pose.
- Don't recommend the ESP32 bridge for the event: an ESP32 with Wi-Fi/Bluetooth is forbidden by the rules.
- Keep the simulator deterministic per seed. A new random source must come from the seed.

## Commands

```bash
npm install
npm run dev -- --host 127.0.0.1        # http://127.0.0.1:5173/?arena=push-pull
npm test                               # world checks incl. Push & Pull scoring (tests/headless.ts)
npm run typecheck
# Headless run, same physics and MicroPython as the browser:
npx tsx tests/match.ts src/examples/challenge2_template.py --arena push-pull --press A@0.2 --seconds 121 --seed 7
npx tsx tests/match.ts prog.py --arena push-pull --press A@0.2 --seconds 121 --cube-seed 42   # seeded layout
npx tsx tests/match.ts prog.py --arena push-pull --press A@0.2 --seconds 121 --map layout.json # Arena-Karte export
SPARK_TRACE=1 npx tsx tests/match.ts ...                                                      # positions + sensors per second
# Many runs in parallel + score histogram (same seeds as the app's Runs grid):
npm run runs -- prog.py --runs 16 --base-seed 1            # each run its own cube layout
npm run runs -- prog.py --runs 50 --map layout.json        # one fixed layout, seeds vary the noise
```

Run `npm test` and `npm run typecheck` before you say a change works.

## Where things are

| Path | What |
|---|---|
| `src/shared/push-pull.ts` | Challenge 2 geometry, cube defaults, seeded layouts, scoring (`cubePoints`) |
| `src/shared/arenas.ts` | Arena registry; `push-pull` arena (reflectance, MJCF walls) |
| `src/world/rules.ts` | Rules per mode; Push & Pull start, 120 s limit, green blink, penalties, stuck (`STUCK_*`) |
| `src/world/world.ts` | Physics, motors, sensors (prox reflectance per material), lockstep scheduler |
| `src/robot/python/` | Simulated `zumo_2040_robot`, `spark_start`, MicroPython runtime shims |
| `src/examples/` | Example programs, auto-listed in the UI (header comments `# Name:` etc.) |
| `src/ui/bench.ts`, `bench-panel.ts` | Parallel headless match pool (test bench) |
| `src/shared/runs.ts` | Seeds, layouts and result summary shared by the Runs grid and `tests/runs.ts` |
| `src/ui/runs-grid.ts` | ▦ Runs: N live worlds in a grid, results column with a points histogram |
| `src/ui/push-pull-map.ts` | Arena-Karte: cube layout editor, seed, referee actions |
| `src/ui/viewer.ts` | three.js 3D view |
| `tests/push-pull.ts` | Push & Pull checks (part of `npm test`) |
| `docs/ARCHITECTURE.md` | How world, robot workers and lockstep fit together |

## Git workflow

Work in the fork `konstibob/spark-sim`; PRs go into its `main`, never into upstream `spark-hpi/spark-sim`.
`gh pr create --repo konstibob/spark-sim --base main`.
