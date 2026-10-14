import math
import unittest
from robotx_graey_2026.api.navigation.bridge_estimator import BridgeEstimator, yaw_rotate
from robotx_graey_2026.api.navigation.mission_reference import MissionReference


class EstimatorTests(unittest.TestCase):
    def setUp(self):
        self.e = BridgeEstimator(heading_validated=True)
        self.e.set_origin((33., -117., 0.))
        self.t = 100.

    def step(self, depth=0., gps=True, north=0., speed=0., heading=0., course=0., gps_speed=None):
        self.t += .05
        e, t = self.e, self.t
        e.gps.clock.offset, e.gps.clock.uncertainty, e.gps.clock.updated = 90., .005, t
        e.imu(t, yaw_rotate((1., 0., 0., 0.), heading), (0., 0., 0.))
        e.pressure_depth(t, depth)
        e.dvl(t, (speed, 0., 0.), True)
        good = False
        if gps and round((t-100)*20) % 4 == 0:
            fix = dict(cube_fix_time_us=int((t-90.-.1)*1e6), lat=33.+north/111320.,
                       lon=-117., accuracy=.5, fix=3, alt=0., received=t,
                       speed_mps=speed if gps_speed is None else gps_speed,
                       course_deg=course, speed_accuracy_mps=.05)
            good = e.observe_gps(t, fix)
        e.status(t)
        return good

    def acquire(self):
        for _ in range(130):
            self.step()
        self.assertTrue(self.e.status(self.t)['healthy'])

    def test_surface_dwell_and_depth_immediate_rejection(self):
        self.acquire()
        p = tuple(self.e.position)
        for _ in range(40):
            self.assertFalse(self.step(depth=.2, north=1.))
        self.assertEqual(self.e.position[:2], list(p[:2]))
        self.assertFalse(self.e.gps.status(self.t)['qualified'])
        self.assertTrue(self.e.status(self.t)['healthy'])
        for _ in range(30):
            self.assertFalse(self.step(depth=0.))
        self.assertFalse(self.e.gps_eligible(self.t))
        for _ in range(85):
            self.step()
        self.assertTrue(self.e.gps.status(self.t)['qualified'])

    def test_underwater_keeps_odometry_but_not_gps(self):
        self.acquire()
        start = self.e.position[0]
        for _ in range(200):
            self.assertFalse(self.step(depth=.5, speed=.2, north=50.))
        self.assertAlmostEqual(self.e.position[0]-start, 2., places=5)
        self.assertTrue(self.e.status(self.t)['healthy'])
        self.assertFalse(self.e.status(self.t)['surfaced'])
        self.assertAlmostEqual(self.e.distance_travelled, 2., places=5)

    def test_position_correction_does_not_increment_distance(self):
        self.acquire()
        start = self.e.position[0]
        accepted = False
        for _ in range(4):
            accepted |= self.step(north=.8)
        self.assertTrue(accepted)
        self.assertGreater(self.e.position[0], start)
        self.assertLessEqual(self.e.position[0]-start, .100001)
        self.assertEqual(self.e.distance_travelled, 0.)

    def test_pressure_loss_and_large_jump(self):
        self.acquire()
        self.e.pressure_depth(self.t, math.nan)
        self.assertFalse(self.e.status(self.t)['healthy'])
        for _ in range(120):
            self.step(north=50.)
        self.assertFalse(self.e.gps.status(self.t)['qualified'])
        self.assertAlmostEqual(self.e.position[0], 0.)

    def test_dvl_loss_latches_without_zero_velocity(self):
        self.acquire()
        self.e.dvl(self.t+.05, (), False)
        self.e.status(self.t+.6)
        self.t += .6
        for _ in range(50):
            self.step()
        self.assertTrue(self.e.fault)
        self.assertFalse(self.e.status(self.t)['healthy'])

    def test_brief_invalid_dvl_withholds_then_recovers_without_inventing_motion(self):
        self.acquire()
        e=self.e
        position=list(e.position)
        variance=e.horizontal_variance
        self.t+=.05
        e.imu(self.t,(1.,0.,0.,0.),(0.,0.,0.))
        e.pressure_depth(self.t,0.)
        e.dvl(self.t,(),False)
        self.assertFalse(e.status(self.t)['healthy'])
        self.assertFalse(e.fault)
        self.step(gps=False,speed=0.)
        self.assertFalse(e.status(self.t)['healthy'])
        self.assertEqual(e.position,position)
        self.step(gps=False,speed=0.)
        self.assertTrue(e.status(self.t)['healthy'])
        self.assertFalse(e.fault)
        self.assertGreater(e.horizontal_variance,variance)
        self.assertEqual(e.dvl_dropouts,1)

    def test_dvl_absent_without_invalid_packet_still_latches(self):
        self.acquire()
        self.e.imu(self.t+.51,(1.,0.,0.,0.),(0.,0.,0.))
        self.e.pressure_depth(self.t+.51,0.)
        self.assertIn('DVL continuity',self.e.status(self.t+.51)['fault'])

    def test_inconsistent_recovery_velocities_cannot_reenable_output(self):
        self.acquire()
        e=self.e
        e.dvl(self.t+.05,(),False)
        for i in range(1,5):
            now=self.t+.05+i*.05
            e.imu(now,(1.,0.,0.,0.),(0.,0.,0.))
            e.pressure_depth(now,0.)
            e.dvl(now,((1. if i%2 else -1.),0.,0.),True)
            self.assertFalse(e.status(now)['healthy'])

    def test_vn_failure_during_dvl_recovery_is_not_cleared(self):
        self.acquire()
        self.e.dvl(self.t+.05,(),False)
        self.e.status(self.t+.51)
        self.t+=.51
        for _ in range(10):self.step(gps=False)
        self.assertTrue(self.e.fault)

    def test_no_heading_correction_at_rest(self):
        self.acquire()
        for _ in range(150):
            self.step(heading=10.)
        self.assertEqual(self.e.heading_bias, 0.)

    def test_heading_and_velocity_updates_need_sustained_forward_motion(self):
        # Exercise the motion update independently of the geographic trajectory.
        e = self.e
        sample = dict(q=yaw_rotate((1., 0., 0., 0.), 10.),
                      raw_velocity=(1., 0., 0.), rates=(0., 0., 0.))
        fix = dict(speed_mps=1.1, course_deg=0., speed_accuracy_mps=.05)
        for i in range(14):
            e.correct_motion(i*.2, fix, sample)
        self.assertEqual(e.heading_bias, 0.)
        for i in range(14, 30):
            e.correct_motion(i*.2, fix, sample)
        self.assertLess(e.heading_bias, 0.)
        self.assertEqual(e.velocity_bias[1], 0.)
        sample['q'] = (1., 0., 0., 0.)
        e.correct_motion(6., fix, sample)
        self.assertGreater(e.velocity_bias[0], 0.)

    def test_reference_accepts_explicit_cube_timestamp(self):
        self.acquire()
        ref = MissionReference.capture(run_id='x', vehicle_epoch='y', gps=self.e.last_fix,
            pose=dict(ned=[0, 0, 0], received=self.t), now=self.t,
            captured_utc='test', surface_confirmed=True, gps_qualified=True, ekf_healthy=True)
        self.assertEqual(ref.gps_time_us, self.e.last_fix['cube_fix_time_us'])

    def test_origin_change_is_a_fault(self):
        self.acquire()
        self.assertFalse(self.e.set_origin((33.1, -117., 0.)))
        self.assertFalse(self.e.status(self.t)['healthy'])

    def test_geodetic_altitude_update_does_not_move_pressure_frame(self):
        self.acquire()
        before = list(self.e.position)
        lat, lon, alt = self.e.origin
        self.assertTrue(self.e.set_origin((lat, lon, alt-.17)))
        self.assertEqual(self.e.position, before)
        self.assertTrue(self.e.status(self.t)['healthy'])
