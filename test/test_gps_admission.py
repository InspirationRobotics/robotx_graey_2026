import unittest
from robotx_graey_2026.api.navigation.gps_admission import CubeClock, GPSAdmission
from robotx_graey_2026.api.navigation.continuous_supervisor import ContinuousSupervisor
from robotx_graey_2026.api.navigation.continuous_supervisor import Observation


class AdmissionTests(unittest.TestCase):
    def make(self):
        g = GPSAdmission(dwell=.2)
        g.clock.request(10_000_000_000)
        self.assertTrue(g.clock.reply(10_000_000_000, 5_010_000_000, 10_020_000_000))
        return g

    def feed(self, g, t, north=0., imu=True, stamp=None):
        g.quality.motion(t, (0., 0., 0.), True)
        return g.check(t, dict(cube_fix_time_us=int((t-5)*1e6) if stamp is None else stamp,
                       lat=33+north/111320, lon=-117., accuracy=1., fix=3), imu)

    def test_qualification_duplicates_stale_and_reboot(self):
        g = self.make()
        for t in (10.1, 10.2, 10.3):
            self.feed(g, t)
        self.assertTrue(self.feed(g, 10.4))
        self.assertFalse(self.feed(g, 10.5, stamp=5_400_000))
        self.assertFalse(g.status(10.9)['qualified'])
        self.assertFalse(self.feed(g, 11., stamp=1))
        self.assertTrue(g.fault)
        self.assertFalse(self.feed(g, 11.1))

    def test_gps_jump_and_imu_loss_never_forward(self):
        g = self.make()
        for i in range(10):
            self.feed(g, 10.1+i*.1)
        self.assertFalse(self.feed(g, 11.1, north=50.))
        self.assertFalse(g.status(11.1)['qualified'])
        self.assertFalse(self.feed(g, 11.2, imu=False))
        self.assertFalse(self.feed(g, 11.3))
        self.assertFalse(g.status(11.3)['qualified'])

    def test_delayed_fix_expires_by_measurement_age(self):
        g = self.make()
        for i in range(5):
            t = 10.1+i*.1
            self.feed(g, t, stamp=round((t-5-.25)*1e6))
        self.assertTrue(g.status(10.5)['qualified'])
        status = g.status(10.61)
        self.assertFalse(status['qualified'])
        self.assertGreater(status['cube_fix_age_upper_s'], .35)
        self.assertIn('stale', status['reason'])
        self.assertFalse(self.feed(g, 10.62))
        self.assertFalse(g.status(10.62)['qualified'])

    def test_lost_clock_mapping_revokes_admission(self):
        g = self.make()
        for i in range(5): self.feed(g, 10.1+i*.1)
        self.assertTrue(g.status(10.5)['qualified'])
        g.clock.offset = None
        self.assertFalse(g.status(10.51)['qualified'])

    def test_timing_requires_current_matched_low_latency_reply(self):
        c = CubeClock()
        self.assertIsNone(c.age(100, 10.))
        c.request(10_000_000_000)
        self.assertFalse(c.reply(9, 5_000_000_000, 10_020_000_000))
        self.assertFalse(c.reply(10_000_000_000, 5_000_000_000, 10_500_000_000))
        g = self.make()
        self.assertIsNone(g.clock.age(9_000_000, 10.1))
        self.assertIsNone(g.clock.age(9_000_000, 14.))

    def test_long_pause_requires_requalification(self):
        g = self.make()
        for i in range(5):
            self.feed(g, 10.1+i*.1)
        self.assertTrue(g.status(10.5)['qualified'])
        self.assertFalse(self.feed(g, 11.))

    def test_stationary_jump_does_not_age_into_acceptance(self):
        g = self.make()
        for i in range(5):
            self.feed(g, 10.1+i*.1)
        for i in range(6, 200):
            now = 10+i*.1
            g.clock.updated = now  # keep the test clock synchronized
            self.assertFalse(self.feed(g, now, north=50.))

    def test_continuous_supervisor_never_switches_or_realigns(self):
        s = ContinuousSupervisor(active=True, depth_dwell=.1)
        o = Observation(healthy=True, depth=0., depth_valid=True, intent='SURFACE',
                        intent_fresh=True, run_id='one', gps_good=True,
                        configuration_verified=True, heading_allowed=True)
        s.step(0., o)
        self.assertEqual(s.step(.2, o), [])
        self.assertTrue(s.allowed)
        o.intent = 'DIVE'
        self.assertEqual(s.step(.3, o), [('capture_reference', None)])
        o.reference_ready = o.reference_saved = True
        o.gps_good = False
        for i in range(4, 300):
            o.depth = .5
            self.assertEqual(s.step(i*.1, o), [])
            self.assertTrue(s.allowed)
        self.assertEqual(s.snapshot(o)['fusion_policy'], 'continuous')
        o.healthy = False
        s.step(30., o)
        self.assertFalse(s.allowed)

    def test_uncommissioned_does_not_allow_navigation(self):
        s = ContinuousSupervisor(active=True, depth_dwell=0.)
        o = Observation(healthy=True, depth=0., depth_valid=True, gps_good=True,
                        intent='SURFACE', intent_fresh=True)
        self.assertEqual(s.step(1., o), [])
        self.assertFalse(s.allowed)
