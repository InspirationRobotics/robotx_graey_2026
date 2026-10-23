import unittest
from robotx_graey_2026.api.navigation.surface_control import gentle_horizontal
class GentleSurfaceTest(unittest.TestCase):
    def test_ramp_and_vertical_unchanged(self):
        v=(0.,0.,0.)
        for _ in range(25):
            n=gentle_horizontal((.2,0.,-.03),v,.1)
            self.assertLessEqual(n[0]-v[0],.00400001)
            self.assertEqual(n[2],-.03)
            v=n
        self.assertAlmostEqual(v[0],.1)
    def test_long_gap_does_not_jump(self):
        self.assertAlmostEqual(gentle_horizontal((.2,0.,0.),(0.,0.,0.),10.)[0],.008)
