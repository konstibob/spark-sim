import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { discoveryScans, parseDiscovery, RobotOutputLines, type DiscoveryPacket } from "../src/shared/discovery.ts";
import { Recorder } from "../src/ui/recorder.ts";
import { runMatch } from "./match.ts";

const packet: DiscoveryPacket = {
  v: 1, t: 1, pose: [0.1, 0, 0], distance: 0.1, bias: 0.2, state: "scan",
  size: 4, cell: 0.05, scans: 1, scan_t: 1, rays: [[0, 0, 0.2, 0]], grid: "0000001002000000",
};
const line = "@spark-map " + JSON.stringify(packet);
const stream = new RobotOutputLines();
const decoded: string[] = [];
for (const byte of line + "\nhello\n") decoded.push(...stream.feed(byte));
assert.deepEqual(parseDiscovery(decoded[0]), packet, "JSON survives byte-at-a-time stdout flushes");
assert.equal(decoded[1], "hello");
assert.equal(parseDiscovery('@spark-map {"v":1}'), null);
assert.equal(parseDiscovery("@spark-map " + JSON.stringify({ ...packet, grid: "bad" })), null);
assert.equal(parseDiscovery("@spark-map " + JSON.stringify({ ...packet, scan_t: 5 })), null);

const recorder = new Recorder();
recorder.addDiscovery(0, packet);
recorder.addDiscovery(0, { ...packet, t: 2, pose: [0.2, 0, 0], grid: undefined });
recorder.addDiscovery(0, { ...packet, t: 3, grid: "2222222222222222" });
assert.equal(recorder.discoveryAt(0, 0.9), null, "no future estimate before first telemetry");
assert.equal(recorder.discoveryAt(1, 2), null, "robot slots stay isolated");
assert.equal(recorder.discoveryAt(0, 2.5)!.grid, packet.grid, "replay keeps old map, never future discoveries");
assert.equal(recorder.discoveryPath(0, 1.5).length, 1);
recorder.clear();
assert.equal(recorder.discoveryAt(0, 10), null, "reset removes map and pose");
const captured = { ...packet, scan_pose: [0.09, 0, 0] as [number, number, number] };
assert.equal(discoveryScans([captured, { ...captured, t: 2 }]).length, 1, "scan export deduplicates received images");
assert.deepEqual(discoveryScans([captured])[0].pose, captured.scan_pose, "export uses pose at capture, not current pose");
console.log("ok   fragmented telemetry, validation, map history, slot isolation and reset");

// Real WASM MicroPython / UART / physics: no pre-A discoveries, then changing
// estimated headings and observed map cells. The bot only gets hardware APIs.
const code = readFileSync("src/examples/discovery.py", "utf8");
const waiting = await runMatch({ programs: [code], arena: "push-pull", tof: [true], seed: 7, seconds: 2 });
assert(!waiting.errors[0], waiting.errors[0]);
const waitingPackets = waiting.log.map((l) => parseDiscovery(l.replace(/^\[[^\]]*\] /, ""))).filter((p): p is DiscoveryPacket => !!p);
assert(waitingPackets.length > 0 && waitingPackets.every((p) => p.scans === 0 && p.distance === 0), "no discoveries or driving before A");
const result = await runMatch({ programs: [code], arena: "push-pull", tof: [true], seed: 7,
  seconds: 16, presses: [{ button: "A", t: 0.2 }], stopWhenDecided: false });
assert(!result.errors[0], result.errors[0]);
const packets = result.log.map((l) => parseDiscovery(l.replace(/^\[[^\]]*\] /, ""))).filter((p): p is DiscoveryPacket => !!p);
assert.equal(packets.length, result.log.filter((l) => l.includes("@spark-map ")).length, "every emitted packet must pass validation");
assert(packets.length > 20, `expected telemetry, got ${packets.length}`);
assert(packets[0].scans === 0 && !packets[0].grid?.includes("2"), "first telemetry starts with an unknown map");
assert(packets.some((p) => p.t < 3 && p.scans > 0), "early A during gyro calibration is retained");
assert(packets.some((p) => p.grid?.includes("2")), "sensor observations create obstacle hypotheses");
assert(packets.some((p) => Math.abs(p.pose[2]) > 0.5), "gyro heading changes during scan");
assert(packets.some((p) => p.distance > 0.05), "test includes actual encoder-measured travel");
assert(!result.out[0], "short exploration remains in the arena");
const last = packets.at(-1)!;
assert.equal(last.state, 'Test beendet', 'the sample finishes its fixed movement sequence');
assert(packets.some((p) => p.state === 'Zurueck'), 'the editable test includes reverse travel');
assert(packets.some((p) => p.state === 'Geradeaus' && p.scans > 0), 'mapping updates during movement');
const truth = result.robots[0];
const error = Math.hypot(last.pose[0] - truth.pos[0], last.pose[1] - truth.pos[1]);
const angle = last.pose[2] - truth.yaw * Math.PI / 180;
const angleError = Math.abs(Math.atan2(Math.sin(angle), Math.cos(angle)));
assert(error < 0.08, `short-run position estimate error ${error} m`);
assert(angleError < 0.35, `short-run heading error ${angleError} rad (including telemetry age)`);
console.log(`ok   MicroPython discovery: ${packets.length} packets, ${last.scans} scans, ${last.distance} m travel, ${(error * 100).toFixed(1)} cm / ${(angleError * 180 / Math.PI).toFixed(1)}° final error including telemetry age`);
