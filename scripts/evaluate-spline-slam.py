"""Run UNMODIFIED C2SR spline-slam on exported ToF scans, outside the browser.

python scripts/evaluate-spline-slam.py scans.jsonl --source /path/to/spline-slam --output result.json [--truth evaluation-only.json]
Requires numpy/scipy. No upstream source is bundled or patched. Does not control the robot.
"""
import argparse
import importlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('scans', type=Path)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, default=Path('spline-result.json'))
    p.add_argument('--truth', type=Path, help='Optional separate audit, used only after all SLAM computation')
    p.add_argument('--levels', type=int, choices=[1, 3], default=3, help='Coarse-to-fine levels, as in upstream example (default 3)')
    args = p.parse_args()
    sys.path.insert(0, str(args.source.resolve()))
    import numpy as np
    core = importlib.import_module('spline_slam.core')
    Surface = importlib.import_module('spline_slam.basics').CubicSplineSurface
    revision = subprocess.run(['git', '-C', str(args.source), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    scans = [json.loads(line) for line in args.scans.read_text().splitlines() if line.strip()]
    if not scans:
        raise ValueError('No sensor scans')
    settings = dict(knot_space=0.05, surface_size=np.array([8., 8.]), logodd_occupied=1., logodd_free=.1,
                    logodd_min_free=-25., logodd_max_occupied=25.)
    matchers, mappers = [], []
    for level in range(args.levels):
        config = dict(settings, knot_space=.05 * 2.5 ** (args.levels - level - 1))
        surface = Surface(**config)
        matchers.append(core.ScanMatching(surface, **config))
        mappers.append(core.Mapping(surface, **config))
    baseline = scans[0]['pose']
    estimate = np.array(baseline, dtype=float)
    previous = estimate.copy()
    for matcher in matchers:
        matcher.pose = estimate.copy()
    trajectory = []
    skipped, matched, failure = 0, 0, None
    start = time.perf_counter()

    class Scan:
        # The upstream core accepts occupied/free point sets. Preserve the eight
        # sparse ToF columns; do NOT invent a dense laser scan or treat misses as free.
        def __init__(self, record):
            pose = record['pose']
            cs, sn = math.cos(pose[2]), math.sin(pose[2])
            rotation = np.array([[cs, sn], [-sn, cs]])
            origin = np.array(pose[:2])
            occupied, free = [], []
            for ox, oy, ex, ey in record['rays']:
                a = rotation @ (np.array([ox, oy]) - origin)
                b = rotation @ (np.array([ex, ey]) - origin)
                occupied.append(b)
                length = np.linalg.norm(b - a)
                for d in np.arange(0.03, max(.03, length - .03), .05):
                    free.append(a + (b - a) * d / length)
            self.occupied = np.array(occupied).T if occupied else np.empty((2, 0))
            self.free = np.array(free).T if free else np.empty((2, 0))

        def get_occupied_pts(self):
            return self.occupied

        def get_free_pts(self):
            return self.free

    for i, record in enumerate(scans):
        odom = np.array(record['pose'])
        cs, sn = math.cos(previous[2]), math.sin(previous[2])
        delta = np.array([[cs, sn], [-sn, cs]]) @ (odom[:2] - previous[:2])
        cs, sn = math.cos(estimate[2]), math.sin(estimate[2])
        prediction = estimate.copy()
        prediction[:2] += np.array([[cs, -sn], [sn, cs]]) @ delta
        prediction[2] += angle(odom[2] - previous[2])
        previous = odom
        sensor = Scan(record)
        try:
            if i >= 3 and sensor.occupied.shape[1] >= 3:
                # Unmodified upstream least-squares scan matching.
                for matcher in matchers:
                    matcher.update_localization(sensor, prediction)
                    prediction = matcher.pose.copy()
                estimate = prediction
                matched += 1
            else:
                estimate = prediction
                for matcher in matchers:
                    matcher.pose = estimate.copy()
                skipped += 1
            if not np.isfinite(estimate).all() or np.max(np.abs(estimate[:2])) > 2:
                raise ValueError('Estimate diverged beyond the evaluation window')
            if sensor.occupied.shape[1]:
                for mapper in mappers:
                    mapper.update_map(sensor, estimate)
            trajectory.append(dict(t=record['t'], odometry=record['pose'], spline=estimate.tolist(), hits=int(sensor.occupied.shape[1])))
        except (ValueError, IndexError, np.linalg.LinAlgError) as e:
            failure = dict(scan=i, t=record['t'], error=str(e))
            break
    report = dict(source='https://github.com/C2SR/spline-slam', revision=revision,
                  input=str(args.scans), frames=len(scans), processed=len(trajectory), matched=matched, skipped=skipped,
                  seconds=time.perf_counter() - start, status='failed' if failure else 'completed', failure=failure,
                  settings=dict(finest_knot_space_m=.05, surface_size_m=[8, 8], resolution_levels=args.levels), trajectory=trajectory,
                  note='Sparse forward ToF experiment, no generated laser beams; completed does not imply improved localization.')
    # Audit truth is loaded only here, after the algorithm has finished.
    if args.truth and trajectory and failure is None:
        truth = json.loads(args.truth.read_text())['final_pose']
        def errors(pose):
            return dict(position_cm=100 * math.hypot(pose[0] - truth[0], pose[1] - truth[1]), heading_deg=abs(math.degrees(angle(pose[2] - truth[2]))))
        report['final_errors'] = dict(odometry=errors(trajectory[-1]['odometry']), spline=errors(trajectory[-1]['spline']))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'trajectory'}, indent=2))


if __name__ == '__main__':
    main()
