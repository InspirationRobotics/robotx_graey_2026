import math
import unittest
from unittest.mock import Mock
from robotx_graey_2026.api.navigation.surface_control import surface_velocity,SurfaceLeg
from robotx_graey_2026.api.pixhawk.mavlink import Link


class SurfaceControlTests(unittest.TestCase):
    def test_translate_while_slewing_without_turn_only_stage(self):
        c=SurfaceLeg();last=0.
        for i in range(20):
            v,heading=c.step(i*.1,(0,0),(0,5),(0,0),(0,0),0.,0.)
            self.assertGreater(v[1],0.)
            self.assertLessEqual(abs(heading-last),math.radians(1.00001));last=heading
            self.assertEqual(c.state['stage'],'tracking')

    def test_near_target_reduces_lookahead_and_freezes_yaw(self):
        c=SurfaceLeg()
        c.step(0.,(0,0),(5,0),(0,0),(0,0),0.,.2)
        _,before=c.step(.1,(0,0),(5,0),(3.5,0),(0,0),0.,.2)
        far=c.state.copy()
        _,after=c.step(.2,(0,0),(5,0),(4.3,.1),(0,0),0.,.2)
        self.assertEqual(before,after)
        self.assertLess(c.state['lookahead_m'],far['lookahead_m'])
        self.assertLess(c.state['yaw_rate_limit_deg_s'],far['yaw_rate_limit_deg_s'])

    def test_cross_track_feedback_returns_to_original_line(self):
        c=SurfaceLeg()
        for i in range(12):v,h=c.step(i*.1,(0,0),(5,0),(1,.2),(0,0),0.,0.)
        self.assertGreater(v[0],0);self.assertLess(v[1],0)
        self.assertLessEqual(math.hypot(*v[:2]),.200001)
        self.assertEqual(v[2],0.)
        v,h=c.step(1.2,(0,0),(5,0),(1,.2),(0,0),0.,math.pi)
        self.assertEqual(c.state['stage'],'tracking');self.assertGreater(v[0],0.)
    def test_surface_deadband_never_requests_continuous_ascent_or_descent(self):
        for depth in (-.1,0.,.03,.08,.2,1.):
            n,e,d=surface_velocity((10.,-10.,0.),(0.,0.,0.),(0.,0.,0.),depth)
            self.assertLessEqual(math.hypot(n,e),.200001)
            self.assertTrue(-.05<=d<=0.)
            if depth<=.03:self.assertEqual(d,0.)
            else:self.assertLess(d,0.)

    def test_overshoot_returns_to_endpoint_instead_of_stalling_beyond_it(self):
        c=SurfaceLeg()
        for i in range(12):
            v,h=c.step(i*.1,(0,0),(5,0),(6,0),(0,0),0.,math.pi)
        self.assertLess(v[0],0.)
        self.assertAlmostEqual(v[1],0.)

    def test_closed_loop_legs_converge_with_turns_cross_current_and_overshoot(self):
        for target,position,yaw in (((4,2),[0.,0.],math.pi),
                                    ((-3,1),[0.,0.],0.),((4,0),[5.,0.],math.pi)):
            c=SurfaceLeg();velocity=[0.,0.];aligned=False
            for i in range(1600):
                v,heading=c.step(i*.1,(0,0),target,position,velocity,0.,yaw)
                error=(heading-yaw+math.pi)%(2*math.pi)-math.pi
                yaw+=max(-.03,min(.03,error))
                for axis in (0,1):
                    velocity[axis]+=.25*(v[axis]-velocity[axis])
                    position[axis]+=.1*velocity[axis]+(0. if axis==0 else .001)
                if c.state['stage']=='tracking':aligned=True
                self.assertLessEqual(math.hypot(*v[:2]),.200001)
                self.assertLessEqual(v[2],0.)
                if math.dist(position,target)<.1:break
            self.assertTrue(aligned)
            self.assertLess(math.dist(position,target),.1)

    def test_depth_correction_is_continuous_bounded_and_independent_of_horizontal_target(self):
        previous=0.
        for depth in (.03,.030001,.04,.06,.08,.2,1.):
            _,_,d=surface_velocity((0.,0.,0.),(0.,0.,0.),(0.,0.,0.),depth)
            self.assertAlmostEqual(d,-min(.05,max(0.,depth-.03)*.5))
            self.assertLessEqual(d,previous);previous=d

    def test_near_target_brakes_and_bad_input_rejected(self):
        n,e,d=surface_velocity((0.,0.,0.),(0.,0.,0.),(.1,-.1,0.),0.)
        self.assertLess(n,0);self.assertGreater(e,0)
        with self.assertRaises(ValueError):surface_velocity((0.,0.),(0.,0.),(0.,0.),math.nan)

    def test_transport_ignores_all_position_fields_and_rejects_down(self):
        link=Link.__new__(Link);link.send=Mock()
        link.surface_velocity(.1,.1,-.03,0.)
        args=link.send.call_args.args
        self.assertEqual(args[5],0b100111000111)
        self.assertEqual(args[6:9],(0,0,0))
        self.assertEqual(args[9:12],(.1,.1,-.03))
        with self.assertRaises(ValueError):link.surface_velocity(0,0,.01,0)
