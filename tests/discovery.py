"""Analytic tests for the portable estimator; no physics or simulator pose available."""
import sys
import math
import unittest
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/robot/python'))
from spark_mapping import Odometry, DiscoveryMap, cube_sightings, CUBE


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


def cube_frame(cube_mm=400, wall_mm=900, cube_cols=(3, 4), sequence=1):
    """A bright cube in columns 3-4 (low row 5) in front of a far wall (row 4, every column)."""
    distances, status, counts = [0] * 64, [255] * 64, [0] * 64
    for col in range(8):
        distances[32 + col], status[32 + col], counts[32 + col] = wall_mm, 5, 1
        if col in cube_cols:
            distances[40 + col], status[40 + col], counts[40 + col] = cube_mm, 5, 1
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

    def test_cube_sighting_uses_known_size(self):
        t = lambda col: math.tan(((col + 0.5) / 8 * 2 - 1) * math.pi / 8)
        wall = [(1.2, t(c), False) for c in range(8)]
        # Two columns at 40 cm, the bright cube bled into both: centre between them, 3 cm behind the face.
        columns = list(wall)
        columns[3], columns[4] = (0.4, t(3), False), (0.4, t(4), False)
        (x, y, first, last), = cube_sightings(columns, 0.052)
        self.assertAlmostEqual(x, 0.052 + 0.4 + CUBE / 2)
        self.assertAlmostEqual(y, 0)
        self.assertEqual((first, last), (3, 4))
        # Too wide for a 6 cm cube at that distance: a wall piece or a robot, not a cube.
        wide = list(wall)
        for c in range(2, 7):
            wide[c] = (0.4, t(c), False)
        self.assertEqual(cube_sightings(wide, 0.052), [])
        # A wall-high return at the same depth: wall, even when narrow.
        tall = list(columns)
        tall[3] = (0.4, t(3), True)
        self.assertEqual(cube_sightings(tall, 0.052), [])
        # Touching the edge of the view: it may continue outside, unknown.
        edge = list(wall)
        edge[0], edge[1] = (0.4, t(0), False), (0.4, t(1), False)
        self.assertEqual(cube_sightings(edge, 0.052), [])
        # Flush against a wall: no free gap behind, unknown.
        flush = [(0.43, t(c), False) for c in range(8)]
        flush[3], flush[4] = columns[3], columns[4]
        self.assertEqual(cube_sightings(flush, 0.052), [])

    def test_known_cube_is_mapped_at_its_size(self):
        m = DiscoveryMap(Odometry(), match=False)
        m.observe(cube_frame(sequence=1), 1000000)
        self.assertEqual(m.known_cubes(), [], 'one sighting is only a candidate')
        m.observe(cube_frame(sequence=2), 1100000)
        (x, y), = m.known_cubes()
        self.assertAlmostEqual(x, 0.482, places=3)
        self.assertAlmostEqual(y, 0, places=3)
        packet = m.packet(1100000, 'scan')
        self.assertEqual(packet['cubes'], [[0.482, 0.0, 2]])
        # Scan matching keeps the raw hit; the published map shows the cube, not its 5 cm cell.
        cell = m._cell(0.452, 0.02)
        self.assertGreater(m.cells[cell], 0)
        self.assertEqual(packet['grid'][cell], '0')
        self.assertEqual(packet['grid'][m._cell(*m.rays[0][2:])], '2', 'the wall stays on the map')

    def test_pushed_cube_is_forgotten_after_seeing_through(self):
        m = DiscoveryMap(Odometry(), match=False)
        for seq in (1, 2):
            m.observe(cube_frame(sequence=seq), seq * 100000)
        self.assertEqual(len(m.known_cubes()), 1)
        # The cube is gone: the same columns now reach the wall behind it.
        for seq in range(3, 10):
            m.observe(cube_frame(cube_cols=(), sequence=seq), seq * 100000)
        self.assertEqual(m.cubes, [])


if __name__ == '__main__':
    unittest.main()
