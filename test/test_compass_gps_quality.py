import unittest
from robotx_graey_2026.api.navigation.compass_gps_quality import CompassGPSQuality


def fix(t, **changes):
    return dict(dict(cube_fix_time_us=int((t+1)*1e6),lat=32.,lon=-117.,fix=4,
                     speed_mps=.2,speed_accuracy_mps=.1,accuracy=1.),**changes)


class CompassGPSQualityTests(unittest.TestCase):
    def setUp(self):
        self.q=CompassGPSQuality();self.q.begin(0.)

    def test_weak_motion_is_guidance_not_an_abort_or_a_heading_claim(self):
        self.q.observe(.1,fix(.1))
        self.assertTrue(self.q.capture_allowed(.2))
        self.assertIn('weak',self.q.reason)
        self.assertIn('Unavailable',self.q.status()['gsf_yaw_confidence'])
        self.q.observe(.3,fix(.3,speed_mps=.8))
        self.assertIn('convergence still required',self.q.reason)

    def test_impossible_speed_is_immediate_even_if_uncertainty_also_bad(self):
        self.q.observe(.1,fix(.1,speed_mps=32.,speed_accuracy_mps=8.))
        self.assertTrue(self.q.fault)
        self.q.observe(.2,fix(.2))
        self.assertFalse(self.q.capture_allowed(.2))

    def test_large_position_jump_taints_attempt(self):
        self.q.observe(.1,fix(.1))
        self.q.observe(.3,fix(.3,lat=32.001))
        self.assertIn('position jumped',self.q.fault)

    def test_brief_loss_pauses_capture_without_reset_loop(self):
        self.q.observe(.1,fix(.1))
        self.q.observe(.3,fix(.3,fix=1))
        self.assertFalse(self.q.capture_allowed(.4))
        self.q.observe(.5,fix(.5))
        self.assertTrue(self.q.capture_allowed(.5))
        self.assertFalse(self.q.fault)
        self.assertEqual(self.q.interruptions,1)

    def test_sustained_uncertainty_and_missing_measurements_reject(self):
        self.q.observe(.1,fix(.1,speed_accuracy_mps=.9))
        self.q.check(1.2)
        self.assertTrue(self.q.fault)
        self.q.begin(2.)
        self.q.check(3.6);self.assertFalse(self.q.fault)
        self.q.check(4.7);self.assertTrue(self.q.fault)

    def test_duplicate_packets_cannot_refresh_fix_and_clock_reversal_rejects(self):
        self.q.observe(.1,fix(.1))
        for t in (.3,1.,1.7,2.8):self.q.observe(t,fix(.1))
        self.assertTrue(self.q.fault)
        self.q.begin(3.);self.q.observe(3.1,fix(3.1))
        self.q.observe(3.2,fix(3.))
        self.assertIn('backwards',self.q.fault)

    def test_invalid_values_are_not_usable(self):
        for field,value in (('accuracy',float('nan')),('speed_mps',None),('lat',91.),
                            ('speed_accuracy_mps',0.),('cube_fix_time_us',0)):
            self.q.begin(0.);self.q.observe(.1,fix(.1,**{field:value}))
            self.assertFalse(self.q.capture_allowed(.2))
