// Regression: enabling ToF must populate both map views without special Python prints.
import assert from "node:assert/strict";
import loadMujoco from "mujoco";
import { arenaById } from "../src/shared/arenas.ts";
import type { DiscoveryPacket } from "../src/shared/discovery.ts";
import type { RobotSnapshot } from "../src/shared/protocol.ts";
import { World } from "../src/world/world.ts";
import { SensorMapper } from "../src/shared/sensor-mapping.ts";
import { Recorder } from "../src/ui/recorder.ts";

const mj = await loadMujoco();
const arena = arenaById("push-pull");
const world = new World(mj, arena, [arena.starts[0]], { seed: 7, tof: [true] });
try {
  // No robot program at all: the sensor image already has data in preview mode.
  world.advanceTo(200_000);
  const robot = world.snapshot(1).robots[0] as RobotSnapshot & { discovery?: DiscoveryPacket };
  assert(robot.tof?.frame?.distanceMm.some((v) => v > 0), "fixture has live depth measurements");
  assert(robot.discovery, "live ToF must provide a map estimate without @spark-map program output");
  assert(robot.discovery.scans > 0, "sensor frames reach the mapper");
  assert(robot.discovery.grid?.includes("2"), "observed obstacles appear on the map");
  assert(robot.discovery.grid?.includes("1"), "observed free space appears on the map");
  assert.deepEqual(robot.discovery.pose.slice(0, 2), [0, 0], "stationary preview stays at assumed origin");
  const frozen = structuredClone(robot.discovery);
  world.robots[0].cmd = [1800, 1800];
  world.advanceTo(1_200_000);
  const moved = world.snapshot(1).robots[0].discovery!;
  assert(moved.distance > 0.05 && moved.pose[0] > 0.05, "map pose follows encoder-measured travel without Python sensor reads");
  assert.deepEqual(robot.discovery, frozen, "recorded estimates are immutable while the robot moves");
  world.setToF(0, false);
  assert.equal(world.snapshot(1).robots[0].discovery, undefined, "disabled sensor produces no fresh map");
  world.advanceTo(1_400_000);
  world.setToF(0, true);
  world.advanceTo(1_600_000);
  assert(world.snapshot(1).robots[0].discovery!.distance > moved.distance, "toggling ToF preserves the odometry frame");
  console.log("ok   ordinary ToF preview produces a sensor-derived pose, free space and obstacles");
} finally {
  world.dispose();
}

// The mapper's input contains measurements only; synthetic depths exercise optics.
const mapper = new SensorMapper();
mapper.updateOdometry(0, [0, 0], 3);
mapper.updateOdometry(1_000_000, [8467, 8467], 3);
assert(Math.abs(mapper.packet().pose[0] - 1) < 1e-9);
assert.equal(mapper.packet().pose[2], 0, "stationary startup bias is removed");
const frame = { capturedAtUs: 1_000_000, sequence: 1,
  distanceMm: new Array(64).fill(0), targetStatus: new Array(64).fill(255), targetsDetected: new Array(64).fill(0) };
frame.distanceMm[27] = 400; frame.targetStatus[27] = 5; frame.targetsDetected[27] = 1;
mapper.observe(frame);
const ray = mapper.packet().rays[0];
assert(Math.abs(ray[2] - 1.452) < 1e-9, "off-axis data is axial depth, transformed by estimated pose");
assert(ray[3] < ray[1], "raw leftmost column points scene-right");
const mapped = mapper.packet();
mapper.observe(frame);
assert.equal(mapper.packet().scans, mapped.scans, "snapshot/render polling does not repeat observations");
mapper.observe({ ...frame, sequence: 2, capturedAtUs: 500_000 });
assert.equal(mapper.packet().scans, mapped.scans, "old image cannot be assigned the current pose");

const history = new Recorder();
history.addSensorDiscovery(0, { ...mapped, t: 1 });
history.addDiscovery(0, { ...mapped, source: undefined, t: 2, pose: [9, 8, 0], grid: undefined });
history.addSensorDiscovery(0, { ...mapped, t: 3 });
assert.equal(history.discoveryAt(0, 1.5)!.source, "sensors", "replay uses fallback before program telemetry exists");
assert.deepEqual(history.discoveryAt(0, 3)!.pose, [9, 8, 0], "program's own estimate takes precedence");
assert.equal(history.discoveryAt(0, 3)!.grid, undefined, "program maps never inherit fallback grids");
assert.deepEqual(history.discoveryPath(0, 3), [[9, 8]], "paths from different map frames stay separate");
assert.equal(history.discoveryAt(1, 3), null, "sensor fallback stays within its robot slot");
history.clear();
assert.equal(history.discoveryAt(0, 3), null, "reset removes both map sources");
console.log("ok   sensor geometry, capture alignment, map-source precedence and replay/reset");

// Extra inspector observations must not change the program's sensors, UART or physics.
const observed = new World(mj, arena, [arena.starts[0]], { seed: 19, tof: [true] });
const control = new World(mj, arena, [arena.starts[0]], { seed: 19, tof: [true] });
try {
  control.robots[0].discovery = null;
  for (const w of [observed, control]) w.robots[0].cmd = [1500, 1800];
  for (let t = 20_000; t <= 1_000_000; t += 20_000) {
    observed.advanceTo(t);
    observed.snapshot(1); // deliberately observe frequently
  }
  control.advanceTo(1_000_000);
  const { discovery, ...actual } = observed.snapshot(1).robots[0];
  const expected = control.snapshot(1).robots[0];
  delete expected.discovery;
  assert(discovery!.scans > 0);
  assert.deepEqual(actual, expected, "mapper leaves all other telemetry and physics unchanged");
  assert.deepEqual(observed.readImu(observed.robots[0]), control.readImu(control.robots[0]), "program IMU noise stream is untouched");
  assert.deepEqual(observed.robots[0].tofSerial.read(1_000_000, 4096), control.robots[0].tofSerial.read(1_000_000, 4096), "mapper never consumes program UART data");
} finally {
  observed.dispose(); control.dispose();
}
console.log("ok   passive mapping leaves physics, program sensor noise and UART bytes unchanged");
