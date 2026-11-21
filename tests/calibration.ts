// Challenge 2 calibration with real programs (part of npm test): the crew's three windows before the start, the
// simulator-only skip flag, the fine brightness levels. World-only checks are in tests/push-pull.ts.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { CAL_TOTAL_S } from "../src/shared/calibration.ts";
import { runMatch } from "./match.ts";

const example = readFileSync("src/examples/challenge2_calibration.py", "utf8");
const at = (log: string[], text: string) => {
  const line = log.find((s) => s.includes(text));
  return line ? Number(line.slice(1, line.indexOf("s "))) : NaN;
};

{
  const r = await runMatch({ programs: [example], arena: "push-pull", seed: 7, seconds: 2, presses: [{ button: "A", t: 0.2 }], calibration: { wallCm: 5, cubeCm: 5 } });
  assert(!r.errors[0], r.errors[0]);
  const max = r.log.find((s) => s.includes("CAL wall max"));
  assert(max, `no calibration result: ${r.log.join("\n")}`);
  const [, wall, cube] = max.match(/wall max (\d+) cube threshold (\d+)/)!.map(Number);
  assert(wall >= 13 && wall <= 17 && cube === wall + 1, max);
  const line = r.log.find((s) => s.includes("CAL line floor"))!;
  assert(/floor \[\d{2}, /.test(line) && !line.includes("tape [961, 726"), `window 1 learned nothing: ${line}`);
  assert(!r.log.some((s) => /still drives|False start/.test(s)), r.log.filter((s) => /world/.test(s)).join("\n"));
  const start = at(r.log, "START tape");
  assert(Math.abs(start - (0.2 + CAL_TOTAL_S)) < 0.05, `program started at ${start} s`);
  console.log(`ok   calibration windows: ${max.slice(max.indexOf("CAL"))}, round from ${start.toFixed(2)} s`);
}

{
  const r = await runMatch({ programs: [example], arena: "push-pull", seed: 7, seconds: 0.5, presses: [{ button: "A", t: 0.2 }] });
  assert(!r.errors[0], r.errors[0]);
  assert(r.log.some((s) => s.includes("CAL skipped (simulator)")), r.log.join("\n"));
  assert(Math.abs(at(r.log, "START tape") - 0.2) < 0.05, r.log.join("\n"));
  console.log("ok   calibration off (default): SIM_SKIP_CALIBRATION, the round starts at once");
}

{
  // the UI's Start (live) begins the crew; every press reaches a program that polls A
  const presses = `from zumo_2040_robot import robot
import time
import spark_start
a = robot.ButtonA()
print("SKIP", spark_start.SIM_SKIP_CALIBRATION)
for k in range(4):
    while not a.check():
        time.sleep_ms(10)
    print("PRESS", k)
`;
  const r = await runMatch({ programs: [presses], arena: "push-pull", seed: 3, seconds: 2, liveStartAt: 1, calibration: { wallCm: 2, cubeCm: 2 } });
  assert(r.log.some((s) => s.endsWith("SKIP False")), r.log.join("\n"));
  const times = [0, 1, 2, 3].map((k) => at(r.log, `PRESS ${k}`));
  assert(times.every((t, k) => Math.abs(t - (1 + k * 11)) < 0.05), `presses at ${times.join(", ")}`);
  assert(Math.abs((r.match.signal ?? 0) - (1 + CAL_TOTAL_S)) < 0.01, `start signal at ${r.match.signal}`);
  console.log(`ok   live Start with calibration: A at ${times.map((t) => t.toFixed(2)).join(", ")} s`);
}

{
  // the crew's 30 fine levels: counts above Pololu's 6 and about 34 ms per reading
  const fine = `from zumo_2040_robot import robot
import time
prox = robot.ProximitySensors()
prox.brightness_levels = [round(10 * 756.3 ** (i / 29)) for i in range(30)]
t = time.ticks_us()
prox.read()
print("FINE", prox.front_counts_with_left_leds(), prox.front_counts_with_right_leds(), time.ticks_diff(time.ticks_us(), t))
`;
  const r = await runMatch({ programs: [fine], arena: "push-pull", seed: 3, seconds: 0.3, poses: [{ x: 0.43 - 0.03 - 0.05 - 0.0589, y: 0, yaw: 0 }] });
  const [fl, fr, us] = r.log.find((s) => s.includes("FINE"))!.split("FINE ")[1].split(" ").map(Number);
  assert(fl >= 18 && fr >= 18 && fl <= 30 && us > 30_000 && us < 40_000, `counts ${fl}/${fr}, ${us} µs`);
  console.log(`ok   fine levels: cube 5 cm ahead reads ${fl}/${fr} of 30 in ${(us / 1000).toFixed(1)} ms`);
}
process.exit(0);
