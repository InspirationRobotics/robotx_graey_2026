import unittest
from robotx_graey_2026.api.navigation.gps_quality import GPSQuality


class GPSQualityTests(unittest.TestCase):
    def fix(self, north=0., accuracy=1.):
        return dict(lat=33+north/111320, lon=-117., accuracy=accuracy, fix=3)

    def test_stationary_drift_and_rejected_anchor(self):
        q = GPSQuality()
        q.motion(0., (0, 0, 0), True)
        self.assertTrue(q.check(0., self.fix())[0])
        for i in range(1, 31):
            t = i/10
            q.motion(t, (0, 0, 0), True)
            good, reason = q.check(t, self.fix(t))
            if t > 2.01:
                self.assertFalse(good)
                self.assertIn('DVL', reason)
        self.assertTrue(q.check(3., self.fix(.1))[0])

    def test_real_motion_is_not_mistaken_for_drift(self):
        q = GPSQuality()
        for i in range(101):
            t = i/10
            q.motion(t, (1, 0, 0), True)
            self.assertTrue(q.check(t, self.fix(t))[0])

    def test_stale_invalid_and_nonfinite_motion(self):
        q = GPSQuality()
        self.assertFalse(q.check(0, self.fix())[0])
        q.motion(0, (0, 0, 0), True)
        self.assertFalse(q.check(1, self.fix())[0])
        q.motion(1, (0, 0, 0), False)
        self.assertFalse(q.check(1, self.fix())[0])
        q.motion(1, (float('nan'), 0, 0), True)
        self.assertFalse(q.check(1, self.fix())[0])

    def test_log_reacquisition_accuracy_is_rejected_despite_3d_fix(self):
        q = GPSQuality()
        q.motion(0, (0, 0, 0), True)
        for accuracy in (0, 71.127, 110.910, 170.737, float('nan')):
            self.assertFalse(q.check(0, self.fix(accuracy=accuracy))[0])
