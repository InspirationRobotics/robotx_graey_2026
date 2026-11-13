import math
import unittest
from robotx_graey_2026.api.navigation.prediction_policy import cube_prediction_allowed, new_dvl_measurement
from robotx_graey_2026.api.navigation.relative_estimator import RelativeEstimator
from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun


class PredictionAlignmentTests(unittest.TestCase):
    def test_prediction_requires_live_healthy_cube_and_unbroken_frame(self):
        b=dict(coordinate_frame='fixed_local', initialized=True, enabled=True,
               configuration_verified=True, heading_continuity_lost=False,
               inputs=dict(vn=True), published_monotonic=10., pressure_sample=dict(received=10.))
        self.assertTrue(cube_prediction_allowed(b,10.1,True))
        self.assertFalse(cube_prediction_allowed(b,10.1,False))
        self.assertFalse(cube_prediction_allowed(b,10.4,True))
        for field,value in [('enabled',False),('configuration_verified',False),
                            ('heading_continuity_lost',True),('fault','clock regressed')]:
            self.assertFalse(cube_prediction_allowed(dict(b,**{field:value}),10.1,True))

    def test_long_dvl_gap_does_not_latch_or_reset_local_frame(self):
        e=RelativeEstimator();e.initialized=True
        def update(t,valid):
            e.imu(t,(1.,0.,0.,0.),(0.,0.,0.));e.pressure_depth(t,0.)
            e.dvl(t,(.1,0.,0.),valid);return e.status(t)
        update(10.,True);self.assertTrue(new_dvl_measurement(e,10.,None))
        self.assertFalse(new_dvl_measurement(e,10.,10.))
        for i in range(1,161):
            update(10.+i*.05,False)
            self.assertFalse(new_dvl_measurement(e,10.+i*.05,None))
        self.assertFalse(e.fault);self.assertGreater(e.horizontal_variance,25.)
        before=e.orientation()
        for i in range(1,12):update(18.+i*.05,True)
        self.assertIsNone(e.dvl_gap_start);self.assertFalse(e.fault)
        self.assertEqual(e.orientation(),before)
        self.assertTrue(new_dvl_measurement(e,18.55,10.))
        self.assertGreater(e.horizontal_variance,25.) # Velocity does not erase position uncertainty.

