"""Analytic tests for the portable estimator; no physics or simulator pose available."""
import io
import json
import sys
import math
import unittest
from contextlib import redirect_stdout
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


def room_frame(x, y, heading, sequence, walls=(-0.45, 0.55, -0.35, 0.4)):
    """Synthetic 8x8 frame of a walled box seen from a true pose (test fixture only)."""
    left, right, bottom, top = walls
    distances, status, counts = [0] * 64, [255] * 64, [0] * 64
    sx, sy = x + DiscoveryMap.MOUNT_X * math.cos(heading), y + DiscoveryMap.MOUNT_X * math.sin(heading)
    for col in range(8):
        tan_a = math.tan(((col + 0.5) / 8 * 2 - 1) * math.pi / 8)
        a = heading + math.atan(tan_a)
        dx, dy = math.cos(a), math.sin(a)
        hits = [t for t in ((left - sx) / dx if dx < 0 else (right - sx) / dx if dx > 0 else None,
                            (bottom - sy) / dy if dy < 0 else (top - sy) / dy if dy > 0 else None) if t]
        depth = min(hits) * math.cos(math.atan(tan_a))  # optical-axis depth, like the sensor
        zone = 3 * 8 + col
        distances[zone], status[zone], counts[zone] = round(depth * 1000), 5, 1
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
        # Bottom row from the 9 cm mount meets the floor about 25 cm ahead.
        f['distance_mm'][59] = 251
        f['target_status'][59] = 5
        f['nb_target_detected'][59] = 1
        m.observe(f, 1000000)
        self.assertFalse(any(m.cells))


    def test_scan_matching_corrects_drift(self):
        # Turn, drive, turn back, drive. Odometry over-counts every turn by 9 % and has
        # 15 % track slip; the room walls let the matcher pull the pose back.
        def run(match):
            odom = Odometry()
            m = DiscoveryMap(odom, match=match)
            x = y = heading = 0.0
            moves = [(0, 0.06)] * 30 + [(0.01, 0)] * 15 + [(0, -0.05)] * 20 + [(0.01, 0)] * 10
            for seq, (forward, turn) in enumerate(moves, 1):
                heading += turn
                x += forward * math.cos(heading)
                y += forward * math.sin(heading)
                odom.heading += turn + 0.09 * abs(turn)
                odom.x += forward * 1.15 * math.cos(odom.heading)
                odom.y += forward * 1.15 * math.sin(odom.heading)
                m.observe(room_frame(x, y, heading, seq), seq * 66000, (odom.x, odom.y, odom.heading))
            ex, ey, eh = m.estimate()
            return m, math.hypot(ex - x, ey - y), abs(math.atan2(math.sin(eh - heading), math.cos(eh - heading)))

        _, odom_xy, odom_heading = run(False)
        m, slam_xy, slam_heading = run(True)
        self.assertGreater(odom_heading, math.radians(8), 'fixture must actually drift')
        self.assertGreater(odom_xy, 0.03, 'fixture must actually drift')
        self.assertGreater(m.matched, 40)
        # Measured: 14.4 deg -> 1.5 deg, 6.1 cm -> 3.1 cm. Position stays about
        # half a 5 cm cell off; heading is where matching helps most.
        self.assertLess(slam_heading, odom_heading / 5)
        self.assertLess(slam_xy, odom_xy * 0.6)

    def test_matching_waits_for_a_map(self):
        m = DiscoveryMap(Odometry())
        m.observe(room_frame(0, 0, 0, 1), 1000000, (0.03, 0.02, 0.1))
        self.assertEqual(m.matched, 0, 'first scan has nothing to match against')
        self.assertEqual(m.correction, (0.0, 0.0, 0.0))


class MapQueryTests(unittest.TestCase):
    """The map as a data structure for driving programs."""

    def mark(self, m, x, y, value=3):
        m._mark(m._cell(x, y), value)

    def test_hits_and_groups(self):
        m = DiscoveryMap(Odometry())
        self.mark(m, 0.40, 0.0)
        self.mark(m, 0.45, 0.0)   # neighbour cell: same group
        self.mark(m, -0.30, 0.30)
        hits = list(m.hits())
        self.assertEqual(len(hits), 3)
        groups = sorted(m.obstacles(), key=lambda g: -g[2])
        self.assertEqual([g[2] for g in groups], [2, 1])
        self.assertAlmostEqual(groups[0][0], 0.45, places=3)  # cells centred at 0.425 and 0.475
        self.assertEqual(len(m.obstacles(min_cells=2)), 1)
        self.assertTrue(m.occupied(0.41, -0.01))  # same cell as (0.40, 0.0)
        self.assertTrue(m.unknown(0.0, 0.5))
        self.assertIsNone(m.value(5, 5))

    def test_path_clear_radius_and_ignore(self):
        m = DiscoveryMap(Odometry())
        self.mark(m, 0.30, 0.10)
        self.assertTrue(m.path_clear(0, 0, 0.6, 0))  # 10 cm beside the line
        self.assertFalse(m.path_clear(0, 0, 0.6, 0, radius=0.08))
        self.assertTrue(m.path_clear(0, 0, 0.6, 0, radius=0.08, ignore=(0.3, 0.1, 0.05)))
        self.assertTrue(m.path_clear(0, 0, 0.2, 0, radius=0.08), 'segment ends before the obstacle')

    def test_frontiers_follow_seen_free_space(self):
        m = DiscoveryMap(Odometry())
        self.assertEqual(m.frontiers(), [])
        m.observe(frame(), 1000000)
        self.assertTrue(m.seen_free(0.25, -0.02))
        self.assertGreater(len(m.frontiers(min_cells=1)), 0)

    def test_forget_returns_cells_to_unknown(self):
        m = DiscoveryMap(Odometry())
        for _ in range(4):
            self.mark(m, 0.4, 0.0)
        self.assertEqual(m.confident, 1)
        m.forget(0.4, 0.0, 0.05)
        self.assertTrue(m.unknown(0.4, 0.0))
        self.assertEqual(m.confident, 0)
        self.assertEqual(set(m.packet(0, 'x')['grid']), {'0'})

    def test_grid_text_stays_in_step_with_cells(self):
        odom = Odometry()
        m = DiscoveryMap(odom)
        for seq in range(1, 30):
            odom.heading = seq * 0.2
            m.observe(room_frame(0, 0, odom.heading, seq), seq * 66000)
        expected = ''.join('2' if v > 0 else '1' if v < 0 else '0' for v in m.cells)
        self.assertEqual(m.packet(0, 'x')['grid'], expected)

    def test_publish_prints_the_grid_in_pieces_of_one_line(self):
        m = DiscoveryMap(Odometry())
        self.mark(m, 0.4, 0.0)
        out = io.StringIO()
        with redirect_stdout(out):
            m.publish(1000000, 'scan')
            m.publish(1100000, 'scan')  # nothing new: pose only
        first, second = out.getvalue().splitlines()
        self.assertTrue(first.startswith('@spark-map '))
        packet = json.loads(first[11:])
        self.assertEqual(packet['grid'].count('2'), 1)
        self.assertEqual(len(packet['grid']), m.SIZE ** 2)
        self.assertNotIn('grid', json.loads(second[11:]))

    def test_odometry_history_ring_wraps(self):
        p = Odometry(counts_per_m=1000)
        n = Odometry.HISTORY
        for k in range(n + 50):
            p.update(k, k, 0, k * 1000)
        self.assertIsNone(p.at(10 * 1000), 'oldest samples were overwritten')
        x, y, h = p.at((n + 40) * 1000 + 500)
        self.assertAlmostEqual(x, (n + 40.5) / 1000, places=5)
        p.clear_history()
        self.assertIsNone(p.at((n + 40) * 1000))

    def test_reset_pose_keeps_the_map(self):
        p = Odometry()
        m = DiscoveryMap(p)
        self.mark(m, 0.4, 0.0)
        p.x, p.y, p.heading = 0.3, -0.2, 1.0
        m.correction = (0.01, 0.0, 0.02)
        m.reset_pose()
        self.assertEqual(m.estimate(), (0.0, 0.0, 0.0))
        self.assertTrue(m.occupied(0.4, 0.0))


if __name__ == '__main__':
    unittest.main()
