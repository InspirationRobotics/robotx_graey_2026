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

    def recovery_fixture(self):
        self.acquire()
        self.e.north_offset = 30.
        self.e.heading_bias = 2.
        for _ in range(25):
            self.t += .05
            self.e.imu(self.t,(1.,0.,0.,0.),(0.,0.,0.))
            self.e.pressure_depth(self.t,0.)
            self.e.dvl(self.t,(),False)
        for _ in range(45):
            self.step(gps=False)
        self.assertIn('DVL continuity lost',self.e.fault)

    def test_explicit_dvl_recovery_retains_heading_and_requalifies_gps(self):
        self.recovery_fixture()
        before = self.e.orientation()
        self.e.recover_dvl(self.t)
        self.assertEqual(self.e.orientation(),before)
        self.assertEqual((self.e.north_offset,self.e.heading_bias),(30.,2.))
        self.assertFalse(self.e.initialized)
        self.assertFalse(self.e.status(self.t)['healthy'])
        self.assertIsNone(self.e.last_fix)
        self.assertFalse(self.e.gps.status(self.t)['qualified'])
        for _ in range(100):
            self.step(north=12.)
        self.assertTrue(self.e.status(self.t)['healthy'])
        self.assertAlmostEqual(self.e.position[0],12.,places=2)

    def test_dvl_recovery_rejects_vn_loss_even_after_updates_return(self):
        self.recovery_fixture()
        self.e.imu(self.t,(0.,0.,0.,0.),(0.,0.,0.))
        for _ in range(45):self.step(gps=False)
        with self.assertRaisesRegex(ValueError,'VN continuity'):
            self.e.recover_dvl(self.t)

    def test_dvl_recovery_rejects_submerged_and_moving(self):
        self.recovery_fixture()
        self.e.pressure_depth(self.t,.5)
        with self.assertRaisesRegex(ValueError,'floating surface'):
            self.e.recover_dvl(self.t)
        for _ in range(50):self.step(gps=False,speed=.2)
        with self.assertRaisesRegex(ValueError,'stationary'):
            self.e.recover_dvl(self.t)

    def test_dvl_recovery_does_not_clear_unrelated_fault(self):
        self.recovery_fixture()
        self.e.fault='Cube restarted; revalidate navigation frame'
        with self.assertRaisesRegex(ValueError,'only available'):
            self.e.recover_dvl(self.t)

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

    def test_brief_invalid_dvl_coasts_then_recovers_without_reset(self):
        self.acquire()
        e=self.e
        position=list(e.position)
        variance=e.horizontal_variance
        self.t+=.05
        e.imu(self.t,(1.,0.,0.,0.),(0.,0.,0.))
        e.pressure_depth(self.t,0.)
        e.dvl(self.t,(),False)
        self.assertTrue(e.status(self.t)['healthy'])
        self.assertTrue(e.status(self.t)['dvl_continuity']['prediction_active'])
        self.assertFalse(e.fault)
        self.step(gps=False,speed=0.)
        self.assertTrue(e.status(self.t)['healthy'])
        self.assertEqual(e.position,position)
        for _ in range(5):self.step(gps=False,speed=0.)
        self.assertTrue(e.status(self.t)['healthy'])
        self.assertFalse(e.fault)
        self.assertGreater(e.horizontal_variance,variance)
        self.assertEqual(e.dvl_dropouts,1)
        self.assertEqual(e.status(self.t)['dvl_continuity']['state'],'TRACKING')

    def test_dvl_absent_without_invalid_packet_still_latches(self):
        self.acquire()
        self.e.imu(self.t+1.01,(1.,0.,0.,0.),(0.,0.,0.))
        self.e.pressure_depth(self.t+1.01,0.)
        self.assertIn('DVL continuity',self.e.status(self.t+1.01)['fault'])

    def test_inconsistent_recovery_velocities_cannot_reenable_output(self):
        self.acquire()
        e=self.e
        e.dvl(self.t+.05,(),False)
        for i in range(1,5):
            now=self.t+.05+i*.05
            e.imu(now,(1.,0.,0.,0.),(0.,0.,0.))
            e.pressure_depth(now,0.)
            e.dvl(now,((1. if i%2 else -1.),0.,0.),True)
            self.assertTrue(e.status(now)['dvl_continuity']['recovering'])

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

    def test_stationary_dvl_blocks_slow_gps_drift_without_freezing_movement(self):
        self.acquire()
        for _ in range(50):self.step()
        self.assertTrue(self.e.status(self.t)['stationary_hold']['active'])
        before=tuple(self.e.position[:2]);correction=self.e.last_correction
        for i in range(200):self.step(north=i*.005)
        self.assertEqual(tuple(self.e.position[:2]),before)
        self.assertEqual(self.e.velocity(),(0.,0.,0.))
        self.assertEqual(self.e.last_correction,correction)
        self.assertTrue(self.e.status(self.t)['healthy'])
        self.assertTrue(self.e.status(self.t)['gps_admission']['stationary_validated_recently'])
        self.assertFalse(self.e.status(self.t)['gps_admission']['applied_recently'])
        self.step(gps=False,speed=.2)
        self.assertFalse(self.e.status(self.t)['stationary_hold']['active'])
        self.assertGreater(self.e.position[0],before[0])

    def test_stationary_hold_releases_on_invalid_dvl_or_stale_vn(self):
        self.acquire()
        for _ in range(50):self.step()
        self.e.dvl(self.t+.05,(),False)
        self.assertIsNone(self.e.stationary_anchor)
        for _ in range(60):self.step()
        self.assertTrue(self.e.status(self.t)['stationary_hold']['active'])
        self.e.status(self.t+.6)
        self.assertIsNone(self.e.stationary_anchor)

    def test_stationary_hold_never_prevents_first_gps_anchor(self):
        for _ in range(100):self.step(gps=False)
        self.assertFalse(self.e.initialized)
        self.assertIsNone(self.e.stationary_anchor)
        for _ in range(100):self.step(north=3.)
        self.assertTrue(self.e.initialized)
        self.assertAlmostEqual(self.e.position[0],3.,places=2)

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

    def test_underwater_prediction_tracks_last_world_velocity_through_rotation(self):
        self.acquire()
        for _ in range(10):self.step(gps=False,depth=.5,speed=.4)
        e=self.e;start=e.position[0];before=self.t
        for i in range(1,5):
            self.t=before+i*.05
            e.imu(self.t,yaw_rotate((1.,0.,0.,0.),90.),(0.,0.,0.))
            e.pressure_depth(self.t,.5);e.dvl(self.t,(),False)
            s=e.status(self.t)
            self.assertTrue(s['healthy']);self.assertFalse(e.motion_valid)
        self.assertAlmostEqual(e.position[0]-start,.08)
        self.assertAlmostEqual(e.velocity()[1],-.4)
        self.assertTrue(e.heading_validated)
        self.assertGreater(e.missing_distance_bound,0.)

    def test_repeated_dropouts_cannot_erase_unknown_displacement_budget(self):
        self.acquire()
        for cycle in range(20):
            self.t+=.05
            self.e.imu(self.t,(1.,0.,0.,0.),(0.,0.,0.))
            self.e.pressure_depth(self.t,0.);self.e.dvl(self.t,(),False)
            for _ in range(6):self.step(gps=False)
            if self.e.fault:break
        self.assertTrue(self.e.fault)
        self.assertGreaterEqual(self.e.missing_distance_bound,.75)

    def test_coasting_never_supplies_a_gps_or_north_measurement(self):
        self.acquire();self.e.dvl(self.t+.05,(),False)
        self.e.apply_gps(self.t+.05,dict(cube_fix_time_us=1))
        self.assertEqual(self.e.event['decision'],'deferred')
        self.assertEqual(self.e.heading_bias,0.)

    def test_gross_dvl_speed_spike_is_rejected_without_teleport(self):
        self.acquire();before=tuple(self.e.position)
        self.e.dvl(self.t+.05,(100.,0.,0.),True)
        self.assertFalse(self.e.motion_valid)
        self.assertEqual(tuple(self.e.position),before)
        self.assertFalse(self.e.fault)

    def test_geodetic_altitude_update_does_not_move_pressure_frame(self):
        self.acquire()
        before = list(self.e.position)
        lat, lon, alt = self.e.origin
        self.assertTrue(self.e.set_origin((lat, lon, alt-.17)))
        self.assertEqual(self.e.position, before)
        self.assertTrue(self.e.status(self.t)['healthy'])
