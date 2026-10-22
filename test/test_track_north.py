import math
import unittest
from robotx_graey_2026.api.navigation.track_north import TrackNorth


class TrackTests(unittest.TestCase):
    def feed(self, a, angle=73, scale=1, accuracy=.3, sideways=.1):
        cs,sn=math.cos(math.radians(angle)),math.sin(math.radians(angle))
        for i in range(260):
            t=i*.1;x=t*.4;y=t*sideways
            north,east=scale*(cs*x-sn*y),scale*(sn*x+cs*y)
            fix=dict(cube_fix_time_us=i*100000+1,lat=32+north/111320,
                lon=-117+east/(111320*math.cos(math.radians(32))),accuracy=accuracy)
            self.assertFalse(a.observe(t,fix,(.4,sideways,0),(0,0,0),(1,0,0,0),(x,y,0)))
        return fix

    def test_no_passive_alignment_and_no_acceptance_before_complete_stop(self):
        a=TrackNorth();f=self.feed(a);self.assertIsNone(a.candidate)
        a.begin();f=self.feed(a)
        self.assertAlmostEqual(a.candidate['offset_deg'],73,places=4)
        self.assertIsNone(a.offset)
        a.stop(True)
        f['cube_fix_time_us']+=100000
        self.assertTrue(a.observe(26,f,(0,0,0),(0,0,0),(1,0,0,0),(10.4,2.6,0)))
        self.assertAlmostEqual(a.offset,73,places=4)

    def test_bad_scale_uncertainty_abort_and_rejection_never_accept(self):
        for scale,accuracy in ((2.,.3),(1,3)):
            a=TrackNorth();a.begin();self.feed(a,scale=scale,accuracy=accuracy)
            self.assertIsNone(a.candidate);a.stop(True);self.assertFalse(a.approved)
        for stop in (False,):
            a=TrackNorth();a.begin();self.feed(a);a.stop(stop)
            self.assertIsNone(a.candidate);self.assertIsNone(a.offset)
        a=TrackNorth();a.begin();self.feed(a);a.reject('GPS jump')
        a.stop(True);self.assertFalse(a.approved)

    def test_wrapped_offset_and_noisy_gps_track(self):
        a=TrackNorth();a.begin();self.feed(a,angle=190)
        self.assertAlmostEqual(a.candidate['offset_deg'],-170,places=4)

    def test_duplicate_does_not_accumulate_and_clock_reversal_resets(self):
        a=TrackNorth();a.begin()
        f=dict(cube_fix_time_us=100,lat=32,lon=-117,accuracy=.3)
        for i in range(30):a.observe(i*.1,f,(0,0,0),(0,0,0),(1,0,0,0),(0,0,0))
        self.assertEqual(len(a.rows),1)
        f['cube_fix_time_us']=90
        a.observe(3,f,(0,0,0),(0,0,0),(1,0,0,0),(0,0,0))
        self.assertEqual(len(a.rows),0)

    def test_five_metre_track_tolerates_metric_gps_disagreement(self):
        a=TrackNorth();a.begin()
        for i in range(110):
            x=i*.04; n=1.3*x
            a.observe(i*.1,dict(cube_fix_time_us=i*100000+1,lat=32+n/111320,lon=-117,accuracy=.8),(.4,0,0),(0,0,0),(1,0,0,0),(x,0,0))
        self.assertIsNotNone(a.candidate)
        self.assertAlmostEqual(a.candidate['offset_deg'],0.)
        self.assertAlmostEqual(a.candidate['baseline_m'],4.36)
