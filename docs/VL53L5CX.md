# VL53L5CX + ESP32-C6 depth bridge

Enable **VL53L5CX** in the toolbar above the 3D view, or at the top of the selected
robot's Inspector. This attaches the sensor to that robot on **every arena**, including
registered overlay arenas. The setting follows the robot across Run, Reset, arena
changes and browser reloads. The Inspector shows the live 8×8 depth image; **Sensors**
toggles the 3D field grid and measured points. The module itself remains visible.
Recorded frames include capture time, depth, validity and capture pose.

The assumed setup is VL53L5CX → I²C → ESP32-C6 → UART → Zumo 2040. The actual
breakout and existing ESP firmware are unspecified. UART0, 115200 baud, 8N1 and the
JSONL messages below are **a proposed bridge contract**, not a claim of compatibility
with an arbitrary breakout, ESP firmware or third-party Python sensor driver.

## Verified hardware requirements

Checked against ST's primary sources on 2026-11-21:

| Property | Hardware specification |
|---|---|
| Sensing | Direct ToF; 940 nm VCSEL and SPAD receiver |
| Detection volume | Nominal 45° horizontal × 45° vertical; advertised 65° diagonal |
| Range | 20–4000 mm; maximum depends strongly on conditions |
| Physical package | 6.4 × 3.0 × 1.5 mm |
| Supply | AVDD 2.8 or 3.3 V; IOVDD 1.8, 2.8 or 3.3 V |
| Host bus | I²C 400 kHz–1 MHz; ST address 0x52 (7-bit address 0x29) |
| Temperature | −30…85 °C |

These are **chip requirements**; a breakout may contain regulators and level shifters.
Check its schematic before connecting power or ESP/Zumo logic.
Source: [ST DS13754 rev 13, §§2.1–2.6](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf).

| ULD behavior | Requirement |
|---|---|
| Resolution | 4×4 / 16 zones or 8×8 / 64 zones |
| Frame frequency | 1–60 Hz at 4×4; **1–15 Hz at 8×8** |
| Power-on defaults | 4×4 at 1 Hz; this experiment explicitly selects 8×8 at 15 Hz |
| Initialization | Host uploads approximately 84 KB firmware before ranging |
| Configuration | Stop ranging before changing settings |
| Validity | Check `nb_target_detected` and `target_status`; 5 = valid, 255 = no target |
| Target capacity | ULD supports configuring 1–4 returns per zone |

Source: [ST UM2884 rev 7, §§3.3–5.5](https://www.st.com/resource/en/user_manual/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-a-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf).

The returned distance is **perpendicular depth along the optical axis**, already
corrected on the chip, rather than the longer radial distance to an off-axis zone.
A perpendicular flat wall therefore has equal depth in all zones.
Source: [ST employee explanation](https://community.st.com/mems-sensors-48/for-vl53l5cx-what-does-resultsdata-distance-mm-zonenum-exactly-mean-7106).

Raw zone 0 sees the **top right of the scene** because the Rx lens flips the image.
Raw rows progress downward in scene space; columns progress right to left.
The Inspector reverses the columns to display the scene as viewed from behind the sensor:

```text
Top of scene:     7  6  5  4  3  2  1  0
                 15 14 13 12 11 10  9  8
                 ...
Bottom of scene: 63 62 61 60 59 58 57 56
```

Source: [UM2884 §2.2](https://www.st.com/resource/en/user_manual/um2884-a-guide-to-using-the-vl53l5cx-multizone-timeofflight-ranging-sensor-with-a-wide-field-of-view-ultra-lite-driver-uld-stmicroelectronics.pdf).

## Simulation model and boundaries

- Front mounting: optical origin `(52, 0, 90)` mm in the chassis frame (origin at
  ground level). The 90 mm height is measured on the robot; the 52 mm forward offset
  is still an **assumption**. The sensor sits above the 6 cm cubes, so cubes show up
  only in the lower rows. Looking along +x; +y points left, +z up. Placement, yaw,
  pitch and roll all transform the sensor. Mount position is shared between physics and the 3D model.
- Captures use simulation time at 15 Hz with 1 ms physics quantization. Reading the
  data or rendering snapshots never triggers extra captures. Pausing freezes capture.
- Each zone samples a 3×3 angular area against actual MuJoCo collision geometry:
  floor, arena, walls, obstacles, other robots. Occlusion is handled by first hits.
  Sub-rays are grouped within 30 mm; reflectance / radial-distance² weights select
  a single strongest depth cluster. Sampling, clustering and weighting are estimates.
- Reports axis depth in integer millimetres. No hit / beyond 4 m produces distance
  0, count 0, status 255. Below 20 mm produces invalid status 4 and distance 0.
  Status 4 here is a conservative simulation convention for the unspecified near field.
- Repeatability noise uses an **estimated** sigma of `3 mm + 1% of distance`, scaled
  by World → sensor noise. Zero noise makes geometric tests exact. Its separate
  random stream avoids changing other sensors' noise or robot parameters.
- Points use the capture pose, so stale measurements stay in their captured world
  positions while the robot moves. The 350 mm 3D field grid is only a viewing aid;
  it is not the sensor's maximum range or an additional physical object.
  At this level mount, the lower four rows look down at the floor, while the upper
  four look above the horizon. No rows are disabled. The scene hides the parts of
  the reference grid below the floor or behind objects. The Inspector's 64 cells
  and the measured points show actual returns; the wire grid itself is not a depth image.
- **Not a validated optical digital twin**: no photon histogram, ambient IR/sunlight,
  material-dependent detection threshold, glass transparency, multipath, crosstalk,
  inter-zone blur, temperature drift, multi-target output, cover-glass calibration,
  rolling/sequential sub-frame acquisition, I²C firmware upload timing or ESP CPU timing.
  The carrier and ESP add no collision geometry; only Push & Pull adds an estimated 8 g for the
  ESP board (`PUSH_PULL.robotExtra`). All rays use existing
  collision geometry, whose detail can differ from decorative meshes.

For context, ST's 8×8 continuous-mode range table gives a typical dark range of
1.3 m for a gray inner-zone target, compared with 4 m for a white target; the latter
is not a universal guaranteed distance. Actual conditions need hardware measurement.
Source: [DS13754 §§7.2.1–7.2.4](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf).

## Proposed ESP serial contract

One ASCII JSON object followed by LF (`\n`) per sensor frame. Distances, status and
counts retain ULD ordering. The following illustrates the fields; real arrays contain
exactly 64 entries (16 if configured to 4×4):

```text
{"type":"vl53l5cx","seq":1,"t_us":67000,"resolution":64,"distance_mm":[...],"target_status":[...],"nb_target_detected":[...]}\n
```

The ESP should configure continuous mode, 64 zones, 15 Hz and one strongest target;
copy a complete ready ULD result into each message. `t_us` is the ESP capture clock,
not the Zumo receive clock. These clocks are equal in the simulation; real hardware
needs clock synchronization if absolute cross-device timing is required.

The simulated receiver accounts for 10 wire bits per ASCII byte. Messages become
readable incrementally, not instantaneously at capture. It bounds queued unread data
to 4096 bytes by default and drops whole new packets on overflow, preserving buffered
line boundaries. Sequence gaps indicate lost packets. This is a chosen bridge policy;
physical RP2040 UART overrun behavior can differ. Configuring a different simulated
baud assumes matching firmware at the other end. The model does not emulate baud
mismatch, voltage faults or ESP boot messages. Commands from Zumo to ESP are not defined.

Use `machine.UART(0, 115200, timeout=0, rxbuf=4096)` directly, or the portable helper
at [`src/robot/python/spark_tof.py`](../src/robot/python/spark_tof.py):

```python
from spark_tof import ToFSerial
import time

sensor = ToFSerial()
while True:
    frame = sensor.read()  # None until a complete JSON line arrives
    if frame is not None:
        for zone, mm in enumerate(frame["distance_mm"]):
            if frame["target_status"][zone] == 5 and frame["nb_target_detected"][zone]:
                print(zone, mm)
    time.sleep_ms(5)
```

`ToFSerial` needs copying to the real Zumo and matching ESP firmware. It is not part
of Pololu's stock library. The simulator's UART0 supports `any`, `read`, `readline`
and `readinto`; UART1 remains unconnected. Reads return currently buffered bytes
after the first-byte timeout; full MicroPython inter-character read semantics and
hardware pin mapping are not modeled. The helper accumulates partial lines itself.

Try **Examples → VL53L5CX · ESP serial** (slowly turns the robot), or:

```bash
npx tsx tests/match.ts src/examples/tof_serial.py --arena free --seconds 2 --tof
npm run test:tof
```

## Still needed for matching your physical experiment

Measure the final mount angle and forward offset (height: 90 mm, measured); identify the breakout's power requirements;
provide an ESP output sample to replace the provisional JSON contract; record
known-distance walls/targets under intended lighting to calibrate detection and noise.
No physical sensor was connected during this implementation.
