"""Regression coverage for removal of automatic alignment fusion gating."""
import unittest
from robotx_graey_2026.api.navigation.continuous_estimator import ContinuousEstimator
from robotx_graey_2026.api.navigation.bridge_estimator import yaw

class EKFFusionTests(unittest.TestCase):
    def test_gps_and_dvl_dropouts_do_not_start_or_clear_a_manual_reference(self):
        e=ContinuousEstimator();e.imu(1.,(1,0,0,0),(0,0,0));e.align_known_heading(1.,90.)
        e.begin_guided_alignment('unrequested')
        for i in range(1,21):
            t=1+i*.1;e.imu(t,(1,0,0,0),(0,0,0));e.dvl(t,(0,0,0),False)
        self.assertTrue(e.manual_north);self.assertFalse(e.alignment_collecting)
        self.assertAlmostEqual(yaw(e.orientation()),90.,places=5)
        self.assertIsNone(e.north_alignment.candidate)

    def test_gps_is_fused_without_automatic_heading_candidate(self):
        e=ContinuousEstimator()
        for i in range(1,31):
            t=i*.2;e.gps.clock.offset=0.;e.gps.clock.uncertainty=.001;e.gps.clock.updated=t
            e.imu(t,(1,0,0,0),(0,0,0));e.pressure_depth(t,0.)
            e.observe_gps(t,dict(cube_fix_time_us=int(t*1e6),lat=33.,lon=-117.,fix=3,accuracy=.8))
        self.assertTrue(e.status(t)['estimator_health']['gps_position_fused'])
        self.assertFalse(e.heading_validated);self.assertIsNone(e.heading_reset)
        self.assertIsNone(e.north_alignment.candidate)
        self.assertEqual(e.yaw_offset_deg,0.)

if __name__=='__main__':unittest.main()
