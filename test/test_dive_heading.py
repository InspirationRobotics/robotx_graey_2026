import math
import unittest
from robotx_graey_2026.api.navigation.dive_heading import DiveHeadingCheck
from test_supervisor_logic import Rig


class HeadingTests(unittest.TestCase):
    def setUp(self):
        self.check = DiveHeadingCheck(reference_validated=True)

    def sample(self, t, course=0., yaw=0., velocity=(.7, 0., 0.), **changes):
        data = dict(surface=True, gps_good=True,
            gps=dict(received=t, receiver_time_us=int((t+1)*1e6), course_deg=course,
                     speed_mps=.7, speed_accuracy_mps=.03),
            vn=dict(received=t, yaw=yaw, rate=0.), cube=dict(received=t, yaw=yaw, rate=0.),
            dvl=dict(received=t, velocity=velocity, valid=True))
        data.update(changes)
        return self.check.evaluate(t, **data)

    def series(self, **changes):
        return [self.sample(i*.2, **changes) for i in range(21)][-1]

    def test_straight_motion_needs_dwell_then_passes(self):
        self.assertFalse(self.sample(0)['allowed'])
        r = self.series()
        self.assertEqual(r['state'], 'Passed')
        self.assertFalse(r['correction_applied'])

    def test_wraparound(self):
        self.assertEqual(self.series(course=359, yaw=1)['state'], 'Passed')

    def test_true_north_conversion_is_explicit(self):
        self.check = DiveHeadingCheck(reference_validated=True, vn_to_true=12, cube_to_true=12)
        self.assertEqual(self.series(course=12)['state'], 'Passed')

    def test_unvalidated_frame_blocks_even_stationary(self):
        self.check = DiveHeadingCheck()
        self.assertFalse(self.sample(0, velocity=(0, 0, 0))['allowed'])

    def test_stationary_strafe_reverse_and_vertical_motion_skip_course(self):
        for v in [(0, 0, 0), (.7, .5, 0), (-.7, 0, 0), (.7, 0, .3)]:
            self.assertEqual(self.sample(0, velocity=v)['state'], 'Skipped')

    def test_bad_gps_skips_but_invalid_attitude_does_not(self):
        self.assertTrue(self.sample(0, gps_good=False)['allowed'])
        self.assertFalse(self.sample(0, vn=None)['allowed'])
        self.assertFalse(self.sample(0, cube=dict(received=0, yaw=90, rate=0))['allowed'])

    def test_turning_and_uncertain_course_skip(self):
        self.assertEqual(self.sample(0, vn=dict(received=0, yaw=0, rate=10))['state'], 'Skipped')
        for acc in [None, 0., .5, math.nan]:
            gps=dict(received=0, receiver_time_us=1, course_deg=0, speed_mps=.7, speed_accuracy_mps=acc)
            self.assertEqual(self.sample(0, gps=gps)['state'], 'Skipped')

    def test_stale_future_invalid_dvl_and_skew(self):
        for d in [dict(received=-1, valid=True, velocity=(.7, 0, 0)),
                  dict(received=1, valid=True, velocity=(.7, 0, 0)),
                  dict(received=0, valid=False, velocity=(.7, 0, 0))]:
            self.assertFalse(self.sample(0, dvl=d)['allowed'])
        self.assertEqual(self.sample(0, vn=dict(received=-.3, yaw=0, rate=0))['state'], 'Skipped')

    def test_mismatch_cannot_be_bypassed_by_stopping_or_gps_dropout(self):
        self.assertEqual(self.series(course=90)['state'], 'Blocked')
        self.assertFalse(self.sample(4.2, velocity=(0, 0, 0))['allowed'])
        self.assertFalse(self.sample(4.4, gps_good=False)['allowed'])
        for i in range(21):r = self.sample(4.6+i*.2)
        self.assertEqual(r['state'], 'Passed')

    def test_frozen_fix_cannot_qualify_by_repeated_polling(self):
        for i in range(30):
            t=i*.2
            gps=dict(received=t, receiver_time_us=1, course_deg=0, speed_mps=.7, speed_accuracy_mps=.03)
            r=self.sample(t,gps=gps)
        self.assertFalse(r['allowed'])

    def test_interrupted_window_and_nan(self):
        for i in range(12):self.sample(i*.2)
        self.assertFalse(self.sample(5)['allowed'])
        self.assertFalse(self.sample(5.2,yaw=math.nan)['allowed'])


class DiveGateTests(unittest.TestCase):
    def test_completed_gate_survives_descent_and_ascent_hysteresis(self):
        r=Rig().run(5);r.o.intent='DIVE';r.run(2)
        self.assertTrue(r.s.allowed)
        r.o.depth=.45;r.o.heading_allowed=False;r.run(.1)
        self.assertTrue(r.s.surface)
        self.assertTrue(r.s.allowed)
        r.o.intent='UNDERWATER';r.run(3)
        r.o.depth=.1;r.run(3)
        self.assertTrue(r.s.surface)
        self.assertTrue(r.s.allowed)
        r.o.intent='SURFACE';r.run(.1)
        self.assertFalse(r.s.dive_heading_approved)

    def test_disagreement_blocks_capture_alignment_and_switch(self):
        r=Rig().run(5)
        r.o.intent='DIVE';r.o.heading_allowed=False;r.o.heading_reason='Heading disagreement'
        r.run(5)
        self.assertEqual(r.commands,[2])
        self.assertFalse(r.o.reference_ready)
        self.assertFalse(r.o.alignment_id)
        self.assertFalse(r.s.allowed)
        self.assertEqual(r.s.reason,'Heading disagreement')
        r.o.heading_allowed=True;r.run(2)
        self.assertEqual(r.commands,[2,1])
        self.assertTrue(r.s.allowed)

    def test_unexpected_submergence_still_selects_underwater_but_blocks_permission(self):
        r=Rig().run(5)
        r.o.intent='DIVE';r.o.heading_allowed=False;r.o.depth=.8;r.run(5)
        self.assertEqual(r.commands,[2,1])
        self.assertFalse(r.s.allowed)

    def test_underwater_intent_cannot_bypass_surface_gate(self):
        r=Rig().run(5)
        r.o.intent='UNDERWATER';r.o.heading_allowed=False;r.run(5)
        self.assertEqual(r.commands,[2])
        self.assertFalse(r.s.allowed)

    def test_new_run_does_not_inherit_heading_approval_underwater(self):
        r=Rig().run(5);r.o.intent='DIVE';r.run(2)
        r.o.depth=.8;r.o.intent='UNDERWATER';r.run(3)
        self.assertTrue(r.s.allowed)
        r.o.run_id='new';r.run(.2)
        self.assertFalse(r.s.allowed)
