// Analytic optics, real MuJoCo geometry, serial wire time and the actual MicroPython runtime.
import assert from "node:assert/strict";
import loadMujoco from "mujoco";
import { ARENAS, arenaById } from "../src/shared/arenas.ts";
import { NOMINAL, OP, ST, C, makeChannelBuffers, openChannel } from "../src/shared/protocol.ts";
import { tofSlope } from "../src/shared/vl53l5cx.ts";
import { DEFAULT_CUBES } from "../src/shared/push-pull.ts";
import { ToFSensor } from "../src/world/vl53l5cx.ts";
import { ToFSerialBridge } from "../src/world/tof-serial.ts";
import { World } from "../src/world/world.ts";
import { runMatch } from "./match.ts";

const identity = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const noNoise = () => 0;
function wall(sensor: ToFSensor, now: number, depth = 1) {
  sensor.tick(now, [0, 0, 0], identity,
    (dir) => ({ distance: depth / dir[0], reflectance: 0.88 }), noNoise);
  return sensor.frame;
}

// A plane has constant axis depth even in the corners (radial range would be wrong).
const sensor = new ToFSensor();
sensor.enable(true, 0);
assert.equal(wall(sensor, 66_000), null, "no frame before first integration period");
assert(wall(sensor, 67_000)!.distanceMm.every((d) => d === 1000), "wall must be flat across all 64 zones");
assert.equal(sensor.frame!.sequence, 1);
assert.equal(sensor.command(6, 0, 67_000), 1);
sensor.command(7, 0, 67_000);
assert.equal(sensor.command(6, 0, 67_000), 0, "reading consumes readiness");
const old = sensor.frame;
for (let t = 68_000; t <= 133_000; t += 1000) wall(sensor, t, 0.5);
assert.equal(sensor.frame, old, "polling cannot invent fresh frames");
wall(sensor, 134_000, 0.5);
assert(sensor.frame!.distanceMm.every((d) => d === 500));
assert(old!.distanceMm.every((d) => d === 1000), "recorded frames stay immutable");
assert.equal(sensor.command(2, 16, 134_000), -2, "no live configuration");
sensor.command(5, 0, 134_000);
assert.equal(sensor.command(2, 16, 134_000), 1);
assert.equal(sensor.command(3, 60, 134_000), 1);
sensor.command(4, 0, 134_000);
wall(sensor, 151_000);
assert.equal(sensor.frame!.distanceMm.length, 16);
sensor.command(5, 0, 151_000);
sensor.command(2, 64, 151_000);
assert.equal(sensor.command(3, 60, 151_000), -3, "8×8 cannot run at 60 Hz");

for (const depth of [0.01, 4.1]) {
  sensor.enable(true, 0);
  wall(sensor, 67_000, depth);
  assert.deepEqual(sensor.frame!.distanceMm, new Array(64).fill(0));
  assert(sensor.frame!.targetStatus.every((s) => s === (depth < 0.02 ? 4 : 255)));
}
sensor.enable(true, 0);
sensor.tick(67_000, [0, 0, 0], identity, () => null, noNoise);
assert(sensor.frame!.targetsDetected.every((n) => n === 0));
sensor.enable(true, 0);
// Only top-right in scene space has a target: raw zone 0, as in ST's orientation diagram.
sensor.tick(67_000, [0, 0, 0], identity, (d) => d[1] < -0.3 && d[2] > 0.3
  ? { distance: 0.5 / d[0], reflectance: 0.88 } : null, noNoise);
assert.equal(sensor.frame!.targetStatus[0], 5);
assert.equal(sensor.frame!.targetStatus[7], 255);
assert.equal(sensor.frame!.targetStatus[56], 255);
assert(tofSlope(0, 8)[1] < 0 && tofSlope(0, 8)[2] > 0);

// At 115200 baud, even the first byte takes ten bits to arrive. Partial reads preserve JSONL.
sensor.enable(true, 0);
wall(sensor, 67_000);
const serial = new ToFSerialBridge();
serial.publish(sensor.frame!, 64);
assert.equal(serial.available(67_000), 0);
assert.equal(serial.available(67_080), 0);
assert.equal(serial.available(67_087), 1);
const partial = String.fromCharCode(...serial.read(68_000, 200));
assert(!partial.includes("\n"));
const rest = String.fromCharCode(...serial.read(200_000, 4096));
const parsed = JSON.parse(partial + rest);
assert.equal(parsed.resolution, 64);
assert.deepEqual(parsed.distance_mm, new Array(64).fill(1000));
assert.equal(serial.available(200_000), 0);
serial.configure(1200, 4096);
serial.publish(sensor.frame!, 64);
assert(serial.available(100_000) < 5, "low baud changes transport latency");
serial.configure(115200, 256);
serial.publish(sensor.frame!, 64);
assert.equal(serial.available(1e6), 0, "overflow drops a whole packet");
console.log("ok   optical geometry, limits, zone order, cadence, configuration, serial timing and buffering");

const mj = await loadMujoco();
// No arena-specific sensor gating, including dynamically registered overlay maps.
try { const entry = "../overlay/register-world.ts"; await import(entry); } catch { /* optional */ }
for (const makeArena of Object.values(ARENAS)) {
  const arena = makeArena();
  const w = new World(mj, arena, [arena.starts[0]], { tof: [true], tweaks: { ...NOMINAL, noise: 0 } });
  w.judging = false;
  w.advanceTo(1_000_000);
  const snapshot = w.snapshot(1).robots[0].tof!;
  assert.equal(snapshot.frame!.distanceMm.length, 64, arena.id);
  assert.equal(snapshot.frame!.sequence, 15, `${arena.id} must have exactly 15 captures per second`);
  const before = snapshot.frame;
  w.snapshot(1); w.snapshot(1);
  assert.equal(w.snapshot(1).robots[0].tof!.frame, before, "rendering never resamples");
  w.setToF(0, false);
  assert.equal(w.snapshot(1).robots[0].tof!.frame, null);
  assert.equal(w.robots[0].tofSerial.available(1e6), 0);
  w.dispose();
  console.log(`ok   ${arena.id}: 64 zones, 15 Hz, disabling clears readings`);
}

// Add test geometry through the same arena interface used by arbitrary future maps.
const free = arenaById("free");
// Reproduce the half-empty image in an open arena: upwards sees air, downwards sees floor.
// This must be scene geometry, never a hard-coded mask that skips 32 zones.
{
  const w = new World(mj, free, [{ x: 0, y: 0, yaw: 0 }], { tof: [true], tweaks: { ...NOMINAL, noise: 0 } });
  w.judging = false;
  w.advanceTo(67_000);
  const frame = w.robots[0].tof.frame!;
  assert.deepEqual(frame.targetStatus.slice(0, 32), new Array(32).fill(255), "upper half sees open air");
  assert.deepEqual(frame.targetStatus.slice(32), new Array(32).fill(5), "all lower 32 zones measure the floor");
  assert(frame.distanceMm.slice(32).every((d) => d >= 20 && d <= 4000));
  assert(frame.distanceMm[60] < frame.distanceMm[36], "steeper downward zones see nearer floor");
  console.log("ok   open table: upper 32 zones see air, lower 32 zones measure floor; no ignored rows");
  w.dispose();
}
const arena = { ...free, id: "tof-test-wall", mjcf: () => free.mjcf() +
  '<geom name="wall" type="box" pos="0.137 0 0.3" size="0.005 1 0.3" group="0"/>' };
{
  const w = new World(mj, arena, [{ x: 0, y: 0, yaw: 0 }], { tof: [true], tweaks: { ...NOMINAL, noise: 0 } });
  w.judging = false;
  w.advanceTo(67_000);
  const frame = w.snapshot(1).robots[0].tof!.frame!;
  assert(frame.distanceMm.every((d) => Math.abs(d - 80) <= 1), `MuJoCo wall: ${frame.distanceMm}`);
  const ch = openChannel(makeChannelBuffers());
  w.attach(0, ch);
  ch.f64.set([w.tUs, 7]);
  Atomics.store(ch.ctl, C.Op, OP.ToF);
  Atomics.store(ch.ctl, C.State, ST.Request);
  w.pump(new Int32Array(new SharedArrayBuffer(4)), w.tUs, 1);
  assert.equal(ch.f64[1], 1);
  assert.deepEqual(Array.from(ch.f64.slice(7, 71)), frame.distanceMm, "UI and robot share the captured frame");
  w.detach(0);
  w.place(0, { x: 0, y: 0, yaw: Math.PI });
  w.advanceTo(134_000);
  assert.equal(w.robots[0].tof.frame!.targetsDetected[27], 0, "turning away loses the wall");
  w.dispose();
}

// Near wall must occlude a farther one. A small target off the centre ray still contributes to its zone.
{
  const block = { ...free, id: "tof-occlusion", mjcf: () => free.mjcf() +
    '<geom name="far" type="box" pos="0.3 0 0.3" size="0.005 1 0.3" group="0"/>' +
    '<geom name="near" type="box" pos="0.137 -0.032 0.097" size="0.005 0.003 0.003" group="0"/>' };
  const w = new World(mj, block, [{ x: 0, y: 0, yaw: 0 }], { tof: [true], tweaks: { ...NOMINAL, noise: 0 } });
  w.advanceTo(67_000);
  const frame = w.robots[0].tof.frame!;
  assert(frame.distanceMm[0] < 100, `near target visible in raw zone 0: ${frame.distanceMm[0]}`);
  assert(frame.distanceMm[7] > 200, "top-left zone sees the farther wall");
  // Tilt the physical robot, not the viewer: downward optical axis now sees the floor.
  const r = w.robots[0];
  const angle = Math.PI / 6;
  w.d.qpos.set([Math.cos(angle / 2), 0, Math.sin(angle / 2), 0], r.freeQpos + 3);
  mj.mj_forward(w.m, w.d);
  w.setToF(0, true);
  w.advanceTo(134_000);
  assert(w.robots[0].tof.frame!.distanceMm.some((d) => d > 20 && d < 100));
  w.dispose();
}
console.log("ok   MuJoCo wall depth, shared robot/UI frame, yaw, tilt, off-centre target and occlusion");

{
  const cubes = DEFAULT_CUBES.map((p) => ({ ...p }));
  cubes[0] = { x: 0.2, y: 0, yaw: 0 };
  const w = new World(mj, arenaById("push-pull"), [{ x: 0, y: 0, yaw: 0 }], {
    cubes, tof: [true], tweaks: { ...NOMINAL, noise: 0 },
  });
  w.judging = false;
  w.advanceTo(67_000);
  const frame = w.robots[0].tof.frame!;
  assert.equal(frame.targetStatus[27], 5, "sensor sees the physical Push & Pull cube");
  assert(Math.abs(frame.distanceMm[27] - 118) <= 2, `cube face should be 118 mm from the mount: ${frame.distanceMm[27]}`);
  w.dispose();
  console.log("ok   Push & Pull cube geometry produces the expected depth");
}

const program = `from spark_tof import ToFSerial
from zumo_2040_robot import robot
import time
sensor = ToFSerial()
leds = robot.RGBLEDs()
leds.set(0, [0, 20, 0])
leds.show()
frames = []
start = time.ticks_ms()
while len(frames) < 10 and time.ticks_diff(time.ticks_ms(), start) < 1500:
    frame = sensor.read()
    if frame is not None:
        assert len(frame['distance_mm']) == 64
        assert len(frame['target_status']) == 64
        assert len(frame['nb_target_detected']) == 64
        assert time.ticks_us() > frame['t_us'] + 20000
        assert not frames or frame['seq'] > frames[-1]
        frames.append(frame['seq'])
    time.sleep_ms(2)
print('received', len(frames))
`;
const result = await runMatch({ programs: [program], arena: "free", seconds: 2, tof: [true] });
assert.equal(result.errors[0], undefined, result.errors[0]);
assert(result.log.some((l) => l.includes("received 10")), result.log.join("\n"));
const absent = await runMatch({ programs: ["from spark_tof import ToFSerial\nimport time\ns=ToFSerial()\ntime.sleep_ms(200)\nprint(s.read())"], arena: "free", seconds: 1 });
assert.equal(absent.errors[0], undefined);
assert(absent.log.some((l) => l.includes("None")));
console.log("ok   real MicroPython: 10 ordered serial frames after wire latency; disconnected bridge returns None");
