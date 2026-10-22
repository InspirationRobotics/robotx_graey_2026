"""Deterministic closed-loop maneuver/track tests (not vehicle dynamics proof)."""
import math
import unittest
from robotx_graey_2026.api.navigation.north_maneuver import NorthManeuver
from robotx_graey_2026.api.navigation.track_north import TrackNorth


def simulate(offset=73, bad_gps=False, dropout=False, cancel=False):
    a=TrackNorth();a.begin();m=NorthManeuver(0.,10.)
    v=x=0.;dt=.05;last=None
    for i in range(1,1201):
        t=i*dt;x+=v*dt
        eligible=not (dropout and t>8)
        if i%4==0 and eligible:
            angle=math.radians(offset)
            # Small GPS noise plus a fixed translation: translation must not
            # become a yaw correction. DVL stays in the relative VN frame.
            gn=x*math.cos(angle)+2+.02*math.sin(t)
            ge=x*math.sin(angle)-3+.02*math.cos(t)
            if bad_gps and t>10:ge+=8
            last=dict(cube_fix_time_us=i*50000,lat=32+gn/111320,
                lon=-117+ge/(111320*math.cos(math.radians(32))),accuracy=.3)
            a.observe(t,last,(v,0,0),(0,0,0),(1,0,0,0),(x,0,0))
        cmd,_=m.step(t,dict(control_ok=True,abort=cancel and t>8,fresh=True,
            depth=0,yaw=0,rate=0,speed=abs(v),forward=v,sideways=0,
            dvl_valid=True,gps_qualified=eligible,gps_speed=abs(v),candidate_ready=a.candidate is not None))
        v+=(cmd*.002-v)*dt*2
        if m.stage in m.terminal:
            a.stop(m.stage=='COMPLETE')
            if a.approved:
                last['cube_fix_time_us']+=200000
                a.observe(t+.2,last,(0,0,0),(0,0,0),(1,0,0,0),(x,0,0))
            return m,a,x
    return m,a,x


class LoopTests(unittest.TestCase):
    def test_known_offsets_complete_near_ten_metres_without_compass(self):
        for offset in (0,73,-170):
            m,a,x=simulate(offset)
            self.assertEqual(m.stage,'COMPLETE',m.reason)
            self.assertTrue(9<=x<=12,x)
            self.assertLess(abs(a.offset-offset),1)

    def test_distorted_gps_dropout_and_cancel_never_publish_offset(self):
        for args in (dict(bad_gps=True),dict(dropout=True),dict(cancel=True)):
            m,a,x=simulate(**args)
            self.assertEqual(m.stage,'ABORTED')
            self.assertIsNone(a.offset)
            self.assertLess(x,12)
