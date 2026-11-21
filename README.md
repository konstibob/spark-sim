# SPARK Sim

An open-source **Zumo 2040 simulator**: write the same program you copy onto the real robot and run
it in **real MicroPython** (a custom WebAssembly build with the RP2040's heap limit) against **MuJoCo 3D
physics** of the Zumo — tracks, blade, battery sag, realistic sensor timing — in the browser.

Made by the SPARK robotics community. MIT-licensed — self-host it, fork it, use it in your own event.

> **Accuracy — this is a simulation, not the real robot.** The physics (the MuJoCo model and the motor,
> battery and sensor constants) are tuned to *approximate* a real Zumo 2040, but a real robot behaves
> differently: surface, wear, build tolerances, battery state and sensor noise all vary, and some effects
> are modelled only roughly. Use the simulator to develop and sanity-check your code, then **always validate
> on real hardware**. Results here are approximate and come with no guarantee of accuracy — don't rely on
> them for anything safety- or outcome-critical.

## Quick start

```bash
npm install          # .npmrc sets legacy-peer-deps, ignore-scripts and exact versions
npm run dev          # http://localhost:5173
npm test             # world checks (physics, sensors, rules)
npm run typecheck
```

Headless match without a browser (deterministic per seed):

```bash
npx tsx tests/match.ts src/examples/sumo_starter.py --arena dohyo --press A@0.2 --seed 7
```

## Challenge 2: Push & Pull

Open `http://127.0.0.1:5173/?arena=push-pull` after `npm run dev -- --host 127.0.0.1`.
The arena has a **160 × 160 cm clear floor**, **10 cm black walls**, centred **30 × 30 cm**
and **12 × 12 cm** tape squares, and five reflective **6 cm cubes** with MuJoCo collisions.
The tape marks the outside of each square; the labelled dimensions are the clear scoring area.

- **Arena-Karte** opens the scaled map. Drag cubes or enter coordinates in cm (origin at the
  centre, +x right, +y up); zoom into the centre. Save/load a layout as JSON. Changes persist in
  this browser; Run and Reset use the same starting cube positions. Example positions are
  illustrative: the referee's actual layout is unknown.
- Select the **Push & Pull · one-cube demo** example, then **Start** (or Run, then hardware A).
  A starts immediately, with **no sumo countdown**. The demo drives behind the first example
  cube and pushes it into the big square. It uses known example waypoints and encoder/gyro
  odometry; adapt it for another layout. Your own MicroPython uses the normal robot APIs.
- Scoring is live, from each cube's rotated footprint. 12 and 30 cm are inner sizes and the
  2.5 cm tape around them counts: **3 points** completely within the small square including its
  tape (17 × 17 cm), otherwise **1 point** for at least half within the big square including its
  tape (35 × 35 cm). Four cubes fit with room to spare (**13 points**); five fit in theory, the
  middle one turned 45° (**15**). The score freezes after **120 seconds**
  or two green LED pulses 0.1–2 seconds apart. A steady green LED does not finish a round.
- **Test bench** runs the current program alone on this same saved layout across seeds and
  reports average scores; each result can be replayed.
- Referee actions are in the map's **Modell & Schiedsrichter** section: first rescue −1,
  second rescue ends the round, touching −1 and ends the round. Leaving the arena costs −2
  and ends it. Judge the 10 seconds stuck yourself. Early motor commands produce a warning;
  official first-false-start restarts and second-false-start disqualification are manual.

Measured: **2.5 cm tape width**, heavy cubes **140 g**, light cubes **40 g**. Every match draws from its
seed either **2 heavy + 3 light (75 %)** or **3 heavy + 2 light (25 %)**, in random order; the console
lists them at load and the map shows them during the run. The Zumo carries an ESP32-C6 board
(**+8 g, estimated**). Still assumed: **cube friction 0.35** and approximate IR reflectance (cube 0.95,
black wall 0.04). These values are in `src/shared/push-pull.ts` and `src/world/world.ts`. The default robot pushes with its blade; it has no pulling attachment.

Headless demo (same physics and MicroPython as the browser):

```bash
npx tsx tests/match.ts src/examples/push_pull.py --arena push-pull --press A@0.2 --seconds 121 --seed 7
# Optional: use a JSON layout exported from Arena-Karte:
npx tsx tests/match.ts your_program.py --arena push-pull --map push-pull-map.json --press A@0.2 --seconds 121
```

## Self-hosting

```bash
npm run build        # -> dist/
```

Serve `dist/` with a static server that sends the **cross-origin isolation** headers
`Cross-Origin-Opener-Policy: same-origin` and `Cross-Origin-Embedder-Policy: require-corp` — the
simulator runs real MicroPython in Web Workers and needs `SharedArrayBuffer`, so a plain `file://` page
or a server without these headers will not work.

A tiny zero-dependency server that sets them is included:

```bash
node deploy/serve-practice.mjs        # serves ./dist-practice on http://localhost:8099
# or point it at your build:  node deploy/serve-practice.mjs 8099 ./dist
```

## What it can do

- **Same code as the robot.** Real MicroPython with the `zumo_2040_robot` API (Pololu's semantics); every
  bytecode costs RP2040 time and every sensor read its real duration, so loop rates match the robot.
- **Arenas with rules:** a mini-sumo dohyo, a line-following track and a free practice table.
- **Example programs** to start from (sumo, line follower, hello).
- **Optional VL53L5CX + ESP32-C6:** enable **VL53L5CX** above the 3D view for an 8×8 depth image,
  visible module, field grid and measured points on every arena. Includes a proposed UART JSONL bridge
  and MicroPython example. [Specifications, serial contract and simulation limits](docs/VL53L5CX.md).
- **Test bench:** many headless matches in parallel → win rate with a confidence interval; watch any match again.
- **Debugging:** a timeline to scrub every frame, telemetry strip charts, a plotter for your `print`s, a 3D
  sensor view, errors marked in the editor, autocompletion and in-app docs for the robot API.
- **Download** your program for the real robot (as `challenge1.py` … `challenge3.py` or `main.py`); your code is saved in the browser.

## Layout

| Folder | Content |
|---|---|
| `src/shared/` | protocol world ↔ robots, Zumo numbers, the arena registry (`arenas.ts`) |
| `src/world/` | MuJoCo model, motor/battery model, sensors, lockstep, the rules dispatch (`rules.ts`) |
| `src/robot/` | MicroPython runtime and the simulated `zumo_2040_robot` hardware modules |
| `src/ui/` | 3D view and pixel filter, test bench, telemetry, editor helpers, docs, the brand hooks (`brand.ts`) |
| `src/bots/` | the bot registry; the example programs live in `src/examples/` |
| `vendor/micropython/` | the custom MicroPython WebAssembly build |
| `tests/` | world checks, headless match runner |

## How it fits together

```
Browser tab
├── UI (main.ts)        editor, 3D view, dock (console · telemetry · test bench · docs)
├── World worker        MuJoCo + motors/battery + sensors + rules, in time order
└── Robot worker × N    one MicroPython each, blocks on every request to the world
        └─ SharedArrayBuffer (lockstep) ─┘
```

## Extending it

Arenas, scoring, bots, docs and branding are **registries**. A deployment can add its own content without
touching the core — e.g. `registerArena(id, factory)` (`src/shared/arenas.ts`),
`registerRuleHandler(mode, fn)` (`src/world/rules.ts`) and `setBrand(...)` (`src/ui/brand.ts`). This is how
an event can keep its competition arenas, opponent bots and theme in a private layer while the core stays open.

## Workflow

We work in the fork [konstibob/spark-sim](https://github.com/konstibob/spark-sim). All branches and pull
requests go into its `main` — not into the upstream [spark-hpi/spark-sim](https://github.com/spark-hpi/spark-sim).

```bash
git remote -v                     # origin = konstibob/spark-sim, upstream = spark-hpi/spark-sim
git push -u origin HEAD           # push your branch to the fork
gh pr create --repo konstibob/spark-sim --base main
git pull upstream main            # pick up upstream changes when needed
```

## Licence

MIT — see [LICENSE](LICENSE). Third-party components and fonts are listed in [NOTICE](NOTICE). The code
in `src/robot/python/zumo_2040_robot/robot.py` is adapted from
[pololu/zumo-2040-robot](https://github.com/pololu/zumo-2040-robot) (MIT, © Pololu Corporation), and
`vendor/micropython` is built from MicroPython (MIT).
