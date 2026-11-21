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

## What we're working on: SPARK Maker Night · Challenge 2 "Push & Pull"

We use this fork to develop and test our program for
[Challenge 2](https://makernight.spark-hpi.de/en/docs/challenge-2/) of the SPARK Maker Night (HPI).
The code that actually runs at the event is `challenge2.py` in our team repo
[spark-hpi/team-green](https://github.com/spark-hpi/team-green) (private). This repo is only the simulator.

**Round:** Sat 21.11., 11:00, Project Studio. **Code freeze Sat 10:30** (what is on `main` of
team-green then gets flashed). One of us brings the robot to the shop **by 10:40**, otherwise we don't start.
Be at the arena when our team is called, or we get 0 points.

### The task

"Your robot starts in the middle, finds the reflective cubes and pushes them back home." One robot alone,
one round of **2 minutes**.

- **Arena:** about 160 × 160 cm, light floor, **black** walls 10 cm high (unlike Challenge 1).
- **Home:** two black tape squares in the centre, 30 cm and, inside it, 12 cm. The robot starts in the
  12 cm square; the referee lines up every robot facing the same direction.
- **Cubes:** five reflective 6 × 6 × 6 cm cubes, in the **same spots for every team** (the layout is not
  published). Not all five fit into the small square.
- **Calibration phase** before the round: tune the robot to what its sensors read at a cube and at the wall.

### Scoring

| Where a cube lies at the end | Points |
|---|---|
| completely inside the 12 cm square | 3 |
| at least half inside the 30 cm square | 1 |
| anywhere else | 0 |

Four cubes fit exactly into the small square, so the maximum is **13 points**. The round ends after
2 minutes, or when the robot **blinks green** to say it is done (we have to program that). Equal points:
**the shorter time wins**, so finishing early with a green blink is worth it. Overall ranking:
1st place gets 8 points, 8th place 1.

| Event | Consequence |
|---|---|
| Robot moves too early | Start repeated, −1. A second time: round over, 0 points |
| Stuck for 10 s | Freed once, −1. A second time ends the round |
| Robot leaves the arena | Round over, −2 |
| We touch the robot or a cube | Round over, −1 |

Only the referee touches robot and cubes. No end-program button.

### Hardware

Pololu [Zumo 2040](https://www.pololu.com/product/5010) (RP2040, MicroPython, 75:1 HP motors, wide-angle
white front IR LEDs): 2 motors with encoders, 5 down-facing line sensors, IR proximity (front/left/right,
0–6 counts, sees things up to ~30 cm, ~9 ms per `prox.read()`), IMU, 128 × 64 OLED, buttons A/B/C,
6 RGB LEDs, buzzer, 4 × AA NiMH. Weight limit **500 g**, no fixed size limit for Push & Pull
(the referee decides).

Optional parts from the SPARK shop go on I2C0 (GP4 SDA, GP5 SCL) or the two free pins GP28/GP29:
VL53L4CD (1 distance, 1.2 m), VCNL4200 (IR distance 1.5 m), **VL53L5CX** (8 × 8 ToF, 4 m, uploads
its firmware over I2C at every start, a few seconds), SEN0628 (8 × 8 ToF, 60°), BNO085 IMU,
9 g servo + PCA9685 driver, IR LED pair. Soldering only in the Hardware Studio (closed Sat 00:00–07:00).

**Not allowed:** any radio on the robot (*"an ESP32 with Wi-Fi or Bluetooth"*), electronics that are not
from the shop, remote control of any kind, batteries other than 4 × AA NiMH. A script checks the code for
radio and remote control at check-in.

### Team repo rules (spark-hpi/team-green)

- `challenge2.py` in the **top level**, exactly this name. Extra modules also top level (subfolders are not
  copied). Never `main.py` or `boot.py`.
- **Keep the start block**: `from spark_start import wait_for_start` and `wait_for_start()` as the first
  step of `main()`. The crew copies its own `spark_start.py` onto the robot when flashing. Without it
  (practising at home) the template falls back to button A + 5 s countdown.
- `motors.off()` in the `finally` block. The file runs at top level (the menu imports it): no
  `if __name__ == "__main__"`.
- Commit directly to `main`. Other branches, unmerged PRs, USB sticks don't count. At the freeze the repo
  goes read-only and `main` is tagged `submission`; check that tag right after.
- The robot API cheat sheet is `API.md` in the team repo. Test with Thonny + the Pololu plugin or `mpremote`.
- Tips from the crew's test runs: calibrate line sensors on the real floor (white ≈ 23–50, thin black tape
  ≈ 250–325), and `time.sleep_ms(5)` in fast loops.

### Open questions for the crew

1. **Start signal.** The Challenge 2 page says the referee presses **button A** and the robot must start
   on it ("not the IR signal"). The rules page and the template say the crew's **IR signal** starts the
   robot via `spark_start`. Ask which one applies, and whether there is a 5 s delay.
2. **ESP32 bridge.** Our VL53L5CX experiment assumes an ESP32-C6 between sensor and Zumo. The C6 has
   Wi-Fi/Bluetooth, and an ESP is not a shop part, so it most likely fails check-in. The VL53L5CX from the
   shop is meant to go straight onto I2C0.

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

## Simulating Challenge 2: Push & Pull

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
- **Challenge 2 · team template** is the unchanged `challenge2.py` from team-green. The simulator ships its
  own `spark_start.py`, so the template's start block works here: `wait_for_start()` returns the moment
  button A (Start, key S, the bench, `--press A@t`) is pressed, like the Crew's IR start, with no 5 s practice
  delay. Write your program on top of the template and it copies 1:1 into the team repo.
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
  and ends it. **Stuck is judged automatically:** if the robot stays within 3 cm and turns less
  than 30° for 5 s the console warns; at 10 s the referee frees it (−1 point, set down on the nearest
  clear spot towards the centre, facing the centre); stuck a second time ends the round. The rules
  page does not define "stuck" or where a freed robot goes, so these are our assumptions
  (`STUCK_*` in `src/world/rules.ts`, `World.free` in `src/world/world.ts`). Early motor commands produce a warning;
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
