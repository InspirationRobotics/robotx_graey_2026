import math
import unittest
from robotx_graey_2026.api.navigation.land_north import LandNorth


class LandNorthTests(unittest.TestCase):
    def steady(self, heading=30.):
        n=LandNorth()
        q=(math.cos(math.radians(heading)/2),0.,0.,math.sin(math.radians(heading)/2))
        for i in range(51):n.observe(i*.05,q,(0.,0.,0.))
        return n,q

    def test_capture_without_gps_dvl_or_depth_and_follow_turn(self):
        n,q=self.steady()
        self.assertAlmostEqual(n.capture(2.5,90.)['offset_deg'],60.)
        for i in range(1,21):
            h=30+i*.5
            q=(math.cos(math.radians(h)/2),0.,0.,math.sin(math.radians(h)/2))
            n.observe(2.5+i*.05,q,(0.,0.,math.radians(10)))
        self.assertAlmostEqual(n.current(3.5)['current_heading_deg'],100.)

    def test_gap_and_orientation_jump_invalidate(self):
        n,q=self.steady();n.capture(2.5,90.)
        self.assertIsNone(n.current(3.))
        n,q=self.steady();n.capture(2.5,90.)
        n.observe(2.55,(1.,0.,0.,0.),(0.,0.,0.))
        self.assertIsNone(n.current(2.55))

    def test_missing_motion_tilt_and_invalid_heading_rejected(self):
        with self.assertRaises(ValueError):LandNorth().capture(0.,90.)
        n,q=self.steady()
        for heading in (True,float('nan'),360.,-1):
            with self.assertRaises(ValueError):n.capture(2.5,heading)
        n.observe(2.55,q,(0.,0.,.2))
        with self.assertRaises(ValueError):n.capture(2.55,90.)
        n=LandNorth()
        for i in range(51):n.observe(i*.05,(0.,1.,0.,0.),(0.,0.,0.))
        with self.assertRaises(ValueError):n.capture(2.5,90.)
