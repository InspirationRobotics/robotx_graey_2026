import math
import unittest
from robotx_graey_2026.api.navigation.north_alignment import NorthAlignment
from robotx_graey_2026.api.navigation.bridge_estimator import BridgeEstimator, rotate, yaw_rotate, yaw
from robotx_graey_2026.api.navigation.vn_protocol import packet, checked, transact, relative_heading


class NorthTests(unittest.TestCase):
    def test_delayed_five_hz_gps_aligns_despite_between_packet_expiry(self):
        e=BridgeEstimator(auto_north_alignment=True)
        e.set_origin((33.,-117.,0.))
        for i in range(320):
            t=100.+i*.05
            e.gps.clock.offset,e.gps.clock.uncertainty,e.gps.clock.updated=90.,.005,t
            e.imu(t,(1.,0.,0.,0.),(0.,0.,0.))
            e.pressure_depth(t,0.)
            e.dvl(t,(1.,0.,0.),True)
            if i%4==0:
                e.observe_gps(t,dict(cube_fix_time_us=round((t-90.-.25)*1e6),
                    lat=33.+(t-100.)/111320.,lon=-117.,alt=0.,accuracy=.5,fix=3,
                    course_deg=0.,speed_mps=1.,speed_accuracy_mps=.05,received=t))
            e.status(t)
        self.assertTrue(e.heading_validated,e.north_alignment.status())
        self.assertTrue(e.initialized)

    def fix(self, t, course=90., speed=1., sigma=.05):
        return dict(cube_fix_time_us=int(t*1e6), speed_mps=speed,
                    course_deg=course, speed_accuracy_mps=sigma)

    def test_vector_alignment_including_sideways_motion(self):
        for velocity in ((1., 0., 0.), (0., 1., 0.), (-1., 0., 0.)):
            align = NorthAlignment()
            q = yaw_rotate((1., 0., 0., 0.), 30.)
            world = rotate(q, velocity)
            for i in range(30):
                align.observe(i*.2, self.fix(i*.2), world, (0., 0., 0.), q)
            self.assertIsNotNone(align.offset)
            expected = (90.-math.degrees(math.atan2(world[1], world[0]))+180)%360-180
            self.assertAlmostEqual(align.offset, expected)
            self.assertTrue(align.status()['aligned'])
            # A later disagreement must never realign the live navigation frame.
            align.observe(20., self.fix(20., course=0.), world, (0., 0., 0.), q)
            self.assertAlmostEqual(align.offset, expected)

    def test_stationary_uncertain_turning_and_duplicate_samples_never_align(self):
        for speed, sigma, rates, stamp in ((0., .05, (0.,0.,0.), None),
                (1., .5, (0.,0.,0.), None), (1., .05, (0.,0.,1.), None),
                (1., .05, (0.,0.,0.), 1.)):
            a = NorthAlignment()
            for i in range(100):
                a.observe(i*.2, self.fix(stamp if stamp is not None else i*.2, speed=speed, sigma=sigma),
                          (speed, 0., 0.), rates, (1.,0.,0.,0.))
            self.assertIsNone(a.offset)

    def test_inconsistent_samples_and_gaps_reset_collection(self):
        a = NorthAlignment()
        for i in range(50):
            a.observe(i, self.fix(i), (1.,0.,0.), (0.,0.,0.), (1.,0.,0.,0.))
        self.assertIsNone(a.offset)
        a = NorthAlignment()
        for i in range(100):
            a.observe(i*.2, self.fix(i*.2, course=90.+(20 if i%2 else 0)),
                      (1.,0.,0.), (0.,0.,0.), (1.,0.,0.,0.))
        self.assertIsNone(a.offset)

    def test_estimator_requires_alignment_then_anchors_in_corrected_frame(self):
        e = BridgeEstimator(auto_north_alignment=True)
        e.set_origin((33., -117., 0.))
        def step(i, moving):
            t = 100.+i*.05
            e.gps.clock.offset, e.gps.clock.uncertainty, e.gps.clock.updated = 90., .005, t
            e.imu(t, (1.,0.,0.,0.), (0.,0.,0.))
            e.pressure_depth(t, 0.)
            e.dvl(t, (1. if moving else 0.,0.,0.), True)
            if i%4 == 0:
                distance = max(0., (i-140)*.05)
                fix = dict(cube_fix_time_us=int((t-90.-.1)*1e6), lat=33.,
                    lon=-117.+distance/(111320.*math.cos(math.radians(33.))),
                    alt=0., accuracy=.5, fix=3, received=t, course_deg=90.,
                    speed_mps=1. if moving else 0., speed_accuracy_mps=.05)
                e.observe_gps(t, fix)
            return e.status(t)
        for i in range(140):
            s = step(i, False)
        self.assertFalse(s['healthy'])
        self.assertFalse(e.initialized)
        for i in range(140, 320):
            s = step(i, True)
        self.assertTrue(s['healthy'], s)
        self.assertTrue(s['north_alignment']['aligned'])
        self.assertAlmostEqual(yaw(e.orientation()), 90.)
        self.assertLess(abs(e.position[0]), .01)
        # A restart cannot inherit yesterday's relative heading alignment.
        self.assertIsNone(BridgeEstimator(auto_north_alignment=True).north_alignment.offset)


class ProtocolTests(unittest.TestCase):
    def test_checksum_validation(self):
        self.assertEqual(checked(packet('VNRRG,35,1,1,1,1')), ['VNRRG','35','1','1','1','1'])
        with self.assertRaises(ValueError):
            checked(b'$VNRRG,35,1,1,1,1*00\r\n')

    def test_error_response_and_write_mismatch(self):
        class Port:
            def __init__(self, reply): self.reply = reply
            def write(self, unused): pass
            def readline(self): return packet(self.reply)
        with self.assertRaises(OSError):
            transact(Port('VNERR,04'), 'VNRRG,35')
        with self.assertRaises(OSError):
            transact(Port('VNWRG,35,1,0,1,1'), 'VNWRG,35,1,1,1,1')

    def test_relative_mode_preserves_filter_settings_and_verifies(self):
        class Port:
            state = '1,0,0,1'
            def write(self, command):
                f = checked(command)
                if f[0] == 'VNWRG': self.state = ','.join(f[2:])
                self.reply = packet(','.join(f[:2])+','+self.state)
            def readline(self): return self.reply
        self.assertEqual(relative_heading(Port()), ['1','1','0','1'])
