// Record only robot-derived scans for an external SLAM experiment.
// npx tsx scripts/record-discovery.ts .context/robot-scans.jsonl [--seed 7] [--truth .context/truth.json]
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname } from 'node:path';
import { runMatch } from '../tests/match.ts';
import { discoveryScans, parseDiscovery, type DiscoveryPacket } from '../src/shared/discovery.ts';

const args = process.argv.slice(2);
const output = args[0];
if (!output) throw new Error('Usage: record-discovery.ts output.jsonl [--seed N] [--truth evaluation-only.json]');
const value = (flag: string) => args.includes(flag) ? args[args.indexOf(flag) + 1] : undefined;
const result = await runMatch({ programs: [readFileSync('src/examples/discovery.py', 'utf8')], arena: 'push-pull', tof: [true],
  seconds: 16, seed: Number(value('--seed') ?? 7), presses: [{ button: 'A', t: 0.2 }], stopWhenDecided: false });
if (result.errors[0]) throw new Error(result.errors[0]);
const packets = result.log.map((l) => parseDiscovery(l.replace(/^\[[^\]]*\] /, ''))).filter((p): p is DiscoveryPacket => !!p);
const scans = discoveryScans(packets);
if (scans.length < 3) throw new Error(`Only ${scans.length} scans recorded`);
mkdirSync(dirname(output), { recursive: true });
writeFileSync(output, scans.map((s) => JSON.stringify(s)).join('\n') + '\n');
console.log(`${scans.length} sensor-derived scans written to ${output}; no physics poses or arena geometry included.`);
// Explicit separate audit file: never part of the SLAM input. It is read AFTER estimation to score results.
const truthFile = value('--truth');
if (truthFile) {
  mkdirSync(dirname(truthFile), { recursive: true });
  const r = result.robots[0];
  writeFileSync(truthFile, JSON.stringify({ purpose: 'evaluation only', seed: Number(value('--seed') ?? 7),
    final_pose: [r.pos[0], r.pos[1], r.yaw * Math.PI / 180], simulation_t: result.t }, null, 2));
}
