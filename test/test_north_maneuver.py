import unittest
from robotx_graey_2026.api.navigation.north_maneuver import NorthManeuver


class ManeuverTests(unittest.TestCase):
    def setUp(self):
        self.run=NorthManeuver(0.,5.)
        self.o=dict(control_ok=True,fresh=True,depth=0.,yaw=40.,rate=0.,
            speed=.6,forward=.6,dvl_valid=True,gps_qualified=True,gps_speed=.6,candidate_ready=False)

    def step(self,t,**fields):
        return self.run.step(t,dict(self.o,**fields))

    def test_nominal_stops_and_requires_three_seconds_of_stationary_checks(self):
        for i in range(1,121):
            x,r=self.step(i*.05)
            self.assertGreaterEqual(x,0);self.assertEqual(r,0)
        self.run.distance=4.6
        x,r=self.step(6.05,candidate_ready=True,speed=.1,forward=.1)
        self.assertLess(x,0);self.assertEqual(self.run.stage,'SETTLE')
        for i in range(122,184):self.step(i*.05,candidate_ready=True,speed=0.,forward=0.,gps_speed=0.)
        self.assertEqual(self.run.stage,'COMPLETE')
        self.assertEqual(self.run.thrust,0)

    def test_settling_brakes_body_velocity_with_bounded_commands(self):
        self.run.distance=4.6
        x,r=self.step(.05,candidate_ready=True,forward=.6,sideways=-.4)
        self.assertEqual(x,-300);self.assertEqual(self.run.lateral_command,300)

    def test_kill_cancel_takeover_stale_and_depth_never_command_movement(self):
        for field in (dict(control_ok=False),dict(abort=True),dict(fresh=False),dict(depth=.3)):
            self.setUp();self.assertEqual(self.step(.05,**field),(0,0))
            self.assertEqual(self.run.stage,'ABORTED')

    def test_dropout_neutral_then_recovers_without_a_new_heading_reference(self):
        self.step(.05)
        self.assertEqual(self.step(.1,dvl_valid=False),(0,0))
        self.assertGreaterEqual(self.step(.15)[0],0)
        self.assertEqual(self.run.heading,40.)

    def test_excessive_motion_and_travel_abort(self):
        for fields in (dict(yaw=80.),dict(speed=1.1),dict(rate=16.)):
            self.setUp();self.step(.05);self.step(.1,**fields)
            self.assertEqual(self.run.stage,'ABORTED')
        self.setUp()
        for i in range(1,350):self.step(i*.05)
        self.assertEqual(self.run.stage,'ABORTED')

    def test_candidate_never_stops_run_early_or_extends_it_when_missing(self):
        for i in range(1,100):self.step(i*.05,candidate_ready=True)
        self.assertEqual(self.run.stage,'MOVE')
        for i in range(100,350):self.step(i*.05,candidate_ready=False)
        self.assertEqual(self.run.stage,'ABORTED')

    def test_sustained_sensor_loss_aborts_without_accepting_heading(self):
        for i in range(1,120):self.step(i*.05,dvl_valid=False)
        self.assertEqual(self.run.stage,'ABORTED')

    def test_stationary_gps_wandering_cannot_finish(self):
        self.run.distance=4.6
        self.step(.05,candidate_ready=True)
        for i in range(2,120):self.step(i*.05,speed=0.,gps_speed=.4,candidate_ready=True)
        self.assertEqual(self.run.stage,'SETTLE')

    def test_rotation_uses_relative_heading_not_new_north_offset(self):
        self.step(.05,yaw=359.)
        x,r=self.step(.1,yaw=1.)
        self.assertLess(r,0)

    def test_bad_corridor_and_timing_rejected(self):
        for value in (None,True,0,2.9,11,float('nan')):
            with self.assertRaises(ValueError):NorthManeuver(0.,value)
        self.step(.4)
        self.assertEqual(self.run.stage,'ABORTED')
