"""Analytic tests for the portable estimator; no physics or simulator pose available."""
import sys
import math
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/robot/python'))
from spark_mapping import Odometry, DiscoveryMap


def frame(depth=400, sequence=1):
    distances, status, counts = [0] * 64, [255] * 64, [0] * 64
    # Near-horizon upper row, scene right (raw ordering).
    distances[27], status[27], counts[27] = depth, 5, 1
    return dict(seq=sequence, resolution=64, distance_mm=distances,
                target_status=status, nb_target_detected=counts)


class MappingTests(unittest.TestCase):
    def test_odometry_uses_counts_and_corrected_gyro(self):
        p = Odometry(counts_per_m=1000, gyro_bias=3)
        p.update(0, 0, 3, 0)
        p.update(1000, 1000, 3, 1000000)
        self.assertAlmostEqual(p.x, 1)
        self.assertAlmostEqual(p.y, 0)
        p.update(1000, 1000, 183, 2000000)
        self.assertAlmostEqual(p.heading, math.pi / 2)
        p.update(2000, 2000, -177, 3000000)
        self.assertAlmostEqual(p.x, 1)
        self.assertAlmostEqual(p.y, 1)

    def test_initial_map_and_invalid_returns_stay_unknown(self):
        m = DiscoveryMap(Odometry())
        self.assertEqual(set(m.packet(0, 'wait')['grid']), {'0'})
        f = frame()
        f['target_status'][27] = 255
        m.observe(f, 1000000)
        self.assertFalse(any(m.cells))
        self.assertEqual(m.rays, [])

    def test_delayed_scan_uses_interpolated_odometry(self):
        p = Odometry(counts_per_m=1000)
        p.update(0, 0, 0, 0)
        p.update(200, 200, 0, 1000000)
        self.assertAlmostEqual(p.at(500000)[0], 0.1)
        self.assertIsNone(p.at(-1))
        self.assertIsNone(p.at(1000001))
        m = DiscoveryMap(p)
        m.observe(frame(), 500000, p.at(500000))
        self.assertAlmostEqual(m.rays[0][0], 0.152)
        self.assertAlmostEqual(m.rays[0][2], 0.552)
        self.assertAlmostEqual(m.scan_pose[0], 0.1)

    def test_moved_object_is_cleared_only_after_observing_old_space(self):
        m = DiscoveryMap(Odometry())
        m.observe(frame(), 1000000)
        old = m._cell(*m.rays[0][2:])
        self.assertGreater(m.cells[old], 0)
        # A miss cannot establish that the old object disappeared.
        invalid = frame(sequence=2)
        invalid['target_status'][27] = 255
        m.observe(invalid, 1100000)
        self.assertGreater(m.cells[old], 0)
        # Repeated valid farther returns provide free-space evidence at the old location.
        for seq in range(3, 12):
            m.observe(frame(depth=800, sequence=seq), 1000000 + seq * 100000)
        self.assertLess(m.cells[old], 0)
        self.assertGreater(m.cells[m._cell(*m.rays[0][2:])], 0)

    def test_axis_depth_and_raw_right_column(self):
        m = DiscoveryMap(Odometry())
        m.observe(frame(), 1000000)
        ox, oy, x, y = m.rays[0]
        self.assertAlmostEqual(ox, 0.052)
        self.assertAlmostEqual(x, 0.452)
        self.assertLess(y, oy)
        self.assertGreater(m.cells[m._cell(x, y)], 0)
        self.assertLess(m.cells[m._cell(ox + 0.15, oy)], 0)
        before = list(m.cells)
        m.observe(frame(), 1100000)
        self.assertEqual(before, list(m.cells), 'same frame must not accumulate evidence')

    def test_endpoint_uses_estimated_pose(self):
        p = Odometry()
        p.x, p.y, p.heading = 0.1, -0.2, math.pi / 2
        m = DiscoveryMap(p)
        m.observe(frame(), 1000000)
        ox, oy, x, y = m.rays[0]
        self.assertAlmostEqual(ox, 0.1)
        self.assertAlmostEqual(oy, -0.148)
        self.assertAlmostEqual(y, 0.252)
        self.assertGreater(x, ox)

    def test_probable_ground_return_is_not_obstacle(self):
        m = DiscoveryMap(Odometry())
        f = frame()
        f['target_status'][27] = 255
        f['distance_mm'][35] = 910
        f['target_status'][35] = 5
        f['nb_target_detected'][35] = 1
        m.observe(f, 1000000)
        self.assertFalse(any(m.cells))


if __name__ == '__main__':
    unittest.main()
