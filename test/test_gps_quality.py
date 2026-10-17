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

    def test_dropout_does_not_forget_rejected_geographic_jump(self):
        q=GPSQuality();q.motion(0.,(0.,0.,0.),True)
        self.assertTrue(q.check(0.,self.fix())[0])
        q.motion(.1,(),False)
        q.motion(.2,(0.,0.,0.),True)
        self.assertFalse(q.check(.2,self.fix(8.))[0])
        self.assertTrue(q.check(.2,self.fix(.1))[0])

    def test_stationary_slow_drift_does_not_walk_through_window_expiry(self):
        q=GPSQuality()
        rejected=False
        for i in range(600):
            t=i*.1;q.motion(t,(0.,0.,0.),True)
            good,reason=q.check(t,self.fix(t*.08))
            if t>30:
                self.assertFalse(good);rejected=True
        self.assertTrue(rejected)
        self.assertLess(len(q.anchors),30)

    def test_confident_gps_velocity_inconsistent_with_dvl_is_rejected(self):
        q=GPSQuality();q.motion(0.,(0.,0.,0.),True)
        f=self.fix();f.update(speed_mps=2.,speed_accuracy_mps=.05)
        self.assertFalse(q.check(0.,f)[0])
        f.update(speed_mps=.02,speed_accuracy_mps=.4)
        self.assertTrue(q.check(0.,f)[0])  # Position-only, not a north measurement.

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
