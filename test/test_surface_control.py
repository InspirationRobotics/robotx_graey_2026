import math
import unittest
from unittest.mock import Mock
from robotx_graey_2026.api.navigation.surface_control import surface_velocity
from robotx_graey_2026.api.pixhawk.mavlink import Link


class SurfaceControlTests(unittest.TestCase):
    def test_surface_deadband_never_requests_continuous_ascent_or_descent(self):
        for depth in (-.1,0.,.03,.08,.2,1.):
            n,e,d=surface_velocity((10.,-10.,0.),(0.,0.,0.),(0.,0.,0.),depth)
            self.assertLessEqual(math.hypot(n,e),.200001)
            self.assertTrue(-.05<=d<=0.)
            if depth<=.03:self.assertEqual(d,0.)
            else:self.assertLess(d,0.)

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
