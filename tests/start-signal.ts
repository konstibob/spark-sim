import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { runMatch } from "./match.ts";

// The real failure: referee Start occurs before ButtonA even exists. A short
// pulse disappears during imports/calibration; the later wait loop never sees it.
const lateButton = `from zumo_2040_robot import robot
import time
time.sleep_ms(600)
a = robot.ButtonA()
while not a.check():
    time.sleep_ms(10)
print("STARTED")
motors = robot.Motors()
motors.set_speeds(1800, 1800)
time.sleep_ms(200)
motors.off()
time.sleep_ms(150)
print("SECOND", a.check())
`;

for (const delay of [0, 200, 600]) for (const start of [{ liveStartAt: 0 }, { presses: [{ button: "A" as const, t: 0.2 }] }]) {
  const r = await runMatch({ programs: [lateButton.replace('sleep_ms(600)', `sleep_ms(${delay})`)], arena: "push-pull", seed: 800395275, seconds: 1.5, ...start });
  assert(!r.errors[0], r.errors[0]);
  assert(r.log.some((s) => s.endsWith("STARTED")), `Start was lost during initialization (${JSON.stringify(start)}): ${r.log.join("\n")}`);
  assert(r.robots[0].pos[0] > 0.005, "program must actually drive after Start");
  assert(r.log.some((s) => s.endsWith("SECOND False")), "a start must not remain held or create a second press");
  console.log(`ok   ButtonA after ${delay} ms receives ${'liveStartAt' in start ? 'live referee Start' : 'scheduled auto start'} exactly once`);
}

const waiting = await runMatch({ programs: [lateButton], arena: "push-pull", seed: 800395275, seconds: 1.2 });
assert(!waiting.log.some((s) => s.endsWith("STARTED")), "Run without Start must still wait for A");
console.log("ok   Run without Start keeps the robot waiting");

const demo = await runMatch({ programs: [readFileSync('src/examples/push_pull.py', 'utf8')],
  arena: 'push-pull', seed: 800395275, seconds: 1, liveStartAt: 0 });
assert(!demo.errors[0], demo.errors[0]);
assert(demo.robots[0].enc.some((count) => Math.abs(count) > 10), 'the actual Push & Pull demo must move after an immediate Start');
console.log('ok   original Push & Pull demo moves after immediate referee Start');
