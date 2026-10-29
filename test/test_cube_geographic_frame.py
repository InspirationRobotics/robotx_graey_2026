import math
import unittest
from robotx_graey_2026.api.navigation.continuous_estimator import ContinuousEstimator
from robotx_graey_2026.api.navigation.waypoint_logic import geo_to_ne,ne_to_geo


class CubeGeographicFrameTests(unittest.TestCase):
    def estimator(self):
        e=ContinuousEstimator();e.imu(1.,(1,0,0,0),(0,0,0))
        e.filter.x[:2]=[1.2,2.3];e.sync(1.)
        e.map.align(32.9235839,-117.0385754,(0.,0.),0.)
        return e

    def test_different_origins_register_odometry_at_same_geographic_position(self):
        e=self.estimator();geo=e.map.to_geo(e.position)
        e.set_origin((32.9235833,-117.0385061,215.98))
        status=e.status(1.)
        self.assertEqual(status['coordinate_frame'],'cube_ned')
        self.assertEqual(status['map_transform']['translation_ne'],(0.,0.))
        expected=geo_to_ne(*geo,e.origin)
        for a,b in zip(status['position_ned'][:2],expected):self.assertAlmostEqual(a,b,places=7)
        self.assertGreater(math.dist(e.position[:2],expected),6.)
        cube_geo=ne_to_geo(*status['position_ned'][:2],e.origin)
        for a,b in zip(cube_geo,geo):self.assertAlmostEqual(a,b,places=9)
        # GUI/Jetson target inverse now uses this same Cube origin.
        from robotx_graey_2026.api.navigation.geographic_frame import GeographicFrame
        frame=GeographicFrame.restore(status['map_transform'])
        for a,b in zip(frame.gps_to_local(*geo),expected):self.assertAlmostEqual(a,b,places=7)

    def test_manual_reset_anchor_is_converted_and_geographic_position_preserved(self):
        e=self.estimator();e.set_origin((32.9235833,-117.0385061,215.98))
        before=e.cube_position(e.position);e.align_known_heading(1.,90.)
        status=e.status(1.)
        for a,b in zip(status['position_ned'],before):self.assertAlmostEqual(a,b,places=7)
        for a,b in zip(status['heading_reset']['anchor'],before):self.assertAlmostEqual(a,b,places=7)
        self.assertEqual(status['map_transform']['local_to_ne_deg'],0.)

    def test_late_cube_origin_changes_only_output_registration(self):
        e=self.estimator();raw=e.filter.x.copy()
        self.assertEqual(e.status(1.)['coordinate_frame'],'fixed_local')
        e.set_origin((32.9235833,-117.0385061,215.98))
        self.assertTrue((raw==e.filter.x).all())
        self.assertTrue(e.status(1.)['cube_origin_registered'])
        p=e.cube_position(e.position)
        e.set_origin((32.9235833,-117.0385061,220.))
        self.assertEqual(e.cube_position(e.position),p)

if __name__=='__main__':unittest.main()
