import math
import unittest
import numpy as np
from robotx_graey_2026.api.navigation.continuous_estimator import ContinuousEstimator
from robotx_graey_2026.api.navigation.bridge_estimator import yaw
from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun


class CubeHeadingTests(unittest.TestCase):
    def test_manual_true_heading_is_applied_without_gps_or_dvl_and_retained(self):
        from robotx_graey_2026.api.navigation.bridge_estimator import yaw_rotate
        e=ContinuousEstimator();q=yaw_rotate((1,0,0,0),30.)
        e.imu(1.,q,(0,0,0));before=e.position[:]
        e.align_known_heading(1.,90.)
        self.assertAlmostEqual(yaw(e.orientation()),90.,places=6)
        self.assertTrue(e.manual_north);self.assertTrue(e.heading_validated)
        np.testing.assert_allclose(e.position,before)
        e.begin_guided_alignment('next')
        self.assertTrue(e.heading_validated);self.assertFalse(e.north_alignment.active)
        e.imu(1.1,q,(0,0,0))
        self.assertAlmostEqual(yaw(e.orientation()),90.,places=6)
        for i in range(12):
            t=1.1+i*.2;e.pressure_depth(t,0.);e.imu(t,q,(0,0,0))
        e.gps.clock.offset=0.;e.gps.clock.updated=t;e.gps.clock.uncertainty=.001
        e.observe_gps(t,dict(cube_fix_time_us=3300000,lat=33.,lon=-117.,fix=3,accuracy=.8))
        self.assertTrue(e.status(3.31)['estimator_health']['gps_position_fused'])
        self.assertAlmostEqual(e.map.angle,0.)

    def test_manual_heading_before_sensor_initialization_captures_first_vn(self):
        from robotx_graey_2026.api.navigation.bridge_estimator import yaw_rotate
        e=ContinuousEstimator();result=e.align_known_heading(0.,270.)
        self.assertIn('awaiting',result);self.assertFalse(e.heading_validated)
        e.imu(1.,yaw_rotate((1,0,0,0),60.),(0,0,0))
        self.assertTrue(e.manual_north)
        self.assertAlmostEqual(yaw(e.orientation())%360,270.,places=6)

    def test_manual_north_bypasses_alignment_motion_leg(self):
        r=GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic',alignment_corridor_confirmed=True),'run',0.)
        r.stage='ARM'
        b=dict(manual_north=True,heading_reference_validated=True)
        o=dict(fresh=True,mode=4,armed=True,bridge=b,local=(2.,3.,0.),yaw=0.,speed=0.,depth=0.)
        self.assertEqual(r.step(.1,o),[])
        self.assertEqual(r.stage,'TRANSIT')
        self.assertFalse(r.alignment_collecting if hasattr(r,'alignment_collecting') else False)

    def test_corrected_yaw_preserves_position_body_velocity_and_covariance(self):
        for angle in (40.,90.,-120.,179.):
            e=ContinuousEstimator()
            for i in range(101):
                t=i*.05;e.imu(t,(1,0,0,0),(0,0,0));e.dvl(t,(.2,.03,0),True)
            before=np.array(e.position);body=e.velocity();P=e.filter.P.copy()
            e.correct_heading(angle,5.)
            np.testing.assert_allclose(e.position,before,atol=1e-9)
            np.testing.assert_allclose(e.velocity(),body,atol=1e-9)
            np.testing.assert_allclose(np.linalg.eigvalsh(e.filter.P),np.linalg.eigvalsh(P),atol=1e-9)
            self.assertAlmostEqual((yaw(e.orientation())-angle+180)%360-180,0.,places=8)
            e.imu(5.1,(1,0,0,0),(0,0,0))
            self.assertAlmostEqual((yaw(e.orientation())-angle+180)%360-180,0.,places=8)
            self.assertEqual(e.heading_reset['revision'],1)

    def test_cached_guided_hold_and_heading_rotate_with_reference_once(self):
        r=GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic',alignment_corridor_confirmed=True),'run',0.)
        r.start=(0.,0.,0.);r.align_start=(0.,0.,0.);r.guided_target=(10.,0.,0.)
        r.heading=r.align_heading=0.
        reset=dict(revision=1,offset_deg=90.,anchor=[10.,0.,0.])
        r.apply_heading_reset(reset)
        np.testing.assert_allclose(r.guided_target,(10.,0.,0.),atol=1e-9)
        np.testing.assert_allclose(r.align_start,(10.,-10.,0.),atol=1e-9)
        self.assertAlmostEqual(r.heading,math.pi/2)
        r.apply_heading_reset(reset)
        self.assertAlmostEqual(r.heading,math.pi/2)
        np.testing.assert_allclose(r.align_start,(10.,-10.,0.),atol=1e-9)


if __name__=='__main__':unittest.main()
