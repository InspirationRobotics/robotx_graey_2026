import copy
import math
import unittest
from robotx_graey_2026.api.navigation.waypoint_logic import WaypointRun, ne_to_geo, geo_to_ne, validate_plan
from robotx_graey_2026.api.gui.waypoint_api import WaypointAPI


class WaypointTests(unittest.TestCase):
    def setUp(self):
        self.origin = (32.92357, -117.03855, 0.)
        lat, lon = ne_to_geo(1., 0., self.origin)
        self.plan = dict(lat=lat, lon=lon, depth_m=.5, map='geographic')
        self.run = WaypointRun(self.plan, 'run', 0.)
        self.now = 0.
        self.o = dict(fresh=True, surface_measurement=dict(ready=True), mode=19, armed=False, origin=self.origin, local=(0.,0.,0.),
                      speed=0., depth=0., yaw=0., **{'global': self.origin[:2]},
                      supervisor=dict(run_id='run', navigation_ready=True, surfaced=True,
                      fusion_policy='continuous', gps_qualified_input=True, confirmed_source='Continuous fusion', reference_current=True,
                      reference=dict(reference_id='ref', cube_ned=(0.,0.,0.))))

    def step(self):
        self.now += .1
        return self.run.step(self.now, self.o)

    def dive(self):
        self.step()
        self.o['supervisor']['confirmed_source'] = 'Underwater'
        self.step()
        self.assertEqual(self.run.stage, 'GUIDED')
        self.assertEqual(self.step()[0][0], 'guided')
        self.o['mode'] = 4
        self.step()
        self.assertEqual(self.step()[0][0], 'arm')
        self.o['armed'] = True
        self.step()
        self.assertEqual(self.run.stage, 'DIVE')

    def settle(self, local, depth):
        self.o.update(local=local, depth=depth)
        for _ in range(13): self.step()

    def test_full_sequence_and_underwater_intent_through_ascent(self):
        self.dive()
        self.settle((0.,0.,.5), .5)
        self.assertEqual(self.run.stage, 'TRANSIT')
        # Realistic gradual travel avoids the discontinuity guard.
        for i in range(1,11):
            self.o['local'] = (i/10.,0.,.5)
            self.step()
        self.settle((1.,0.,.5), .5)
        self.assertEqual(self.run.stage, 'SURFACE')
        self.assertEqual(self.run.phase, 'UNDERWATER')
        self.settle((1.,0.,0.), 0.)
        self.assertEqual(self.run.stage, 'DISARM')
        self.o['armed'] = False
        self.step()
        self.assertEqual(self.run.phase, 'SURFACE')
        self.o['supervisor']['gps_qualified_input'] = True
        self.step()
        self.assertEqual(self.run.stage, 'COMPLETE')

    def test_already_armed_start_does_not_send_arm(self):
        self.o['armed'] = True
        self.step()
        self.o['supervisor']['confirmed_source'] = 'Underwater'
        self.step()
        self.assertEqual(self.step()[0][0], 'guided')
        self.o['mode'] = 4
        self.assertEqual(self.step(), [])
        self.assertEqual(self.step(), [])
        self.assertEqual(self.run.stage, 'DIVE')

    def test_geographic_target_is_not_rotated_by_vehicle_heading(self):
        # A southeast point stays southeast regardless of the nose direction.
        for degrees in (0.,90.,108.,180.,270.):
            with self.subTest(degrees=degrees):
                self.setUp()
                lat,lon=ne_to_geo(-1.,3.,self.origin)
                self.run=WaypointRun(dict(self.plan,lat=lat,lon=lon),'run',0.)
                self.o['yaw']=math.radians(degrees)
                self.dive()
                self.assertAlmostEqual(self.run.target[0],-1.,places=6)
                self.assertAlmostEqual(self.run.target[1],3.,places=6)
                self.assertAlmostEqual(self.run.heading,math.radians(degrees))

    def test_dvl_gap_pauses_while_supervisor_still_ready_then_resumes(self):
        self.dive();self.run.enter('TRANSIT',self.now,'Traveling')
        self.o.update(local=(.1,0.,.5),depth=.5,navigation_hold_safe=True,
                      dvl_continuity=dict(recovering=True))
        target=self.run.target;first=self.step()
        self.assertEqual(first[0][1][:3],(.1,0.,.5))
        for _ in range(10):self.step()
        self.assertEqual(self.run.stage,'TRANSIT')
        self.o['dvl_continuity']['recovering']=False
        resumed=self.step()
        self.assertEqual(resumed[0][1][:3],target)
        self.assertFalse(self.run.snapshot()['navigation_pause']['active'])

    def test_dvl_recovery_cannot_extend_deadline_or_override_pilot(self):
        self.dive();self.o.update(navigation_hold_safe=True,dvl_continuity=dict(recovering=True))
        self.step()
        for _ in range(17):self.step()
        self.assertEqual(self.run.stage,'ABORTED')
        self.o['dvl_continuity']['recovering']=False
        self.assertFalse(any(a=='target' for a,_ in self.step()))
        self.setUp();self.dive()
        self.o.update(navigation_hold_safe=True,dvl_continuity=dict(recovering=True),armed=False)
        self.assertEqual(self.step(),[])
        self.assertTrue(self.run.pilot_takeover)

    def test_surface_measurement_required_before_prepare(self):
        self.o['surface_measurement']=dict(ready=False,reason='Surface measurement 3/20 s')
        self.step();self.assertEqual(self.run.stage,'WAIT_SURFACE')
        self.assertIn('3/20',self.run.reason)

    def test_changing_cube_pressure_offset_aborts_before_permission_hold(self):
        self.o.update(local=(0.,0.,.66),depth=.009)
        self.dive()
        self.o.update(local=(0.,0.,.45),depth=.2,navigation_hold_safe=True)
        self.o['supervisor']['navigation_ready']=False
        actions=self.step()
        self.assertEqual(self.run.stage,'ABORTED')
        self.assertIn('vertical reference changed',self.run.reason)
        self.assertEqual(actions,[('manual',None)])
        self.assertFalse(any(a=='target' for a,_ in self.step()))

    def test_pressure_overshoot_aborts_even_if_cube_tracks_it(self):
        self.dive()
        self.o.update(local=(0.,0.,.7),depth=.7)
        actions=self.step()
        self.assertEqual(self.run.stage,'ABORTED')
        self.assertIn('Pressure depth exceeded',self.run.reason)
        self.assertEqual(actions,[('manual',None)])

    def test_fixed_nonzero_vertical_offset_remains_supported(self):
        self.o.update(local=(0.,0.,.66),depth=.009)
        self.dive()
        self.o.update(local=(0.,0.,1.151),depth=.5)
        self.step()
        self.assertEqual(self.run.stage,'DIVE')
        self.assertAlmostEqual(self.run.vertical_offset_error,0.)

    def test_pilot_disarm_during_armed_preparation_never_rearms(self):
        self.o['armed'] = True
        self.step()
        self.o['armed'] = False
        self.assertEqual(self.step(), [])
        self.assertEqual(self.run.stage, 'ABORTED')
        self.assertEqual(self.step(), [])

    def test_armed_nonpilot_mode_rejected(self):
        self.o.update(armed=True, mode=4)
        self.assertEqual(self.step(), [])
        self.assertEqual(self.run.stage, 'ABORTED')

    def test_faults_never_continue_guided_targets(self):
        for fault in ('fresh', 'origin', 'source', 'reference', 'jump', 'deadline', 'abort'):
            with self.subTest(fault=fault):
                self.setUp(); self.dive()
                if fault == 'fresh': self.o['fresh'] = False
                if fault == 'origin': self.o['origin'] = (33.,-117.,0.)
                if fault == 'source': self.o['supervisor']['fusion_policy'] = 'invalid'
                if fault == 'reference': self.o['supervisor']['reference']['reference_id'] = 'changed'
                if fault == 'jump': self.o['local'] = (8.,0.,0.)
                if fault == 'deadline': self.now += 2
                if fault == 'abort': self.o['abort'] = True
                actions = self.step()
                self.assertEqual(self.run.stage, 'ABORTED')
                self.assertEqual(actions[0][0], 'manual')

    def test_brief_permission_interruption_holds_then_resumes_original_target(self):
        self.dive()
        self.run.enter('TRANSIT',self.now,'Traveling underwater')
        self.o.update(local=(.2,0.,.5),depth=.5,navigation_hold_safe=True)
        self.o['supervisor']['navigation_ready']=False
        self.o['supervisor']['reason']='Unavailable: Navigation bridge'
        actions=self.step()
        self.assertEqual(self.run.stage,'TRANSIT')
        self.assertEqual(actions[0][0],'target')
        self.assertEqual(actions[0][1][:3],(.2,0.,.5))
        self.o['local']=(.21,0.,.5)
        self.assertEqual(self.step(),[])  # Fixed hold, not a moving target.
        self.o['supervisor']['navigation_ready']=True
        actions=self.step()
        self.assertEqual(actions[0][1][:3],self.run.target)
        self.assertFalse(self.run.snapshot()['navigation_pause']['active'])

    def test_stale_supervisor_hold_expires_without_extending_bad_navigation(self):
        self.dive();self.o.update(fresh=False,navigation_hold_safe=True)
        self.assertEqual(self.step()[0][0],'target')
        for _ in range(9):actions=self.step()
        self.assertEqual(self.run.stage,'ABORTED')
        self.assertEqual(self.run.reason,'Fresh navigation telemetry for this run unavailable')

    def test_independent_hold_cannot_bypass_real_fault_or_frame_change(self):
        for fault in ('origin','policy','reference','watchdog','supervisor','pilot','jump','ekf'):
            with self.subTest(fault=fault):
                self.setUp();self.dive();self.o['navigation_hold_safe']=True
                self.o['supervisor']['navigation_ready']=False
                if fault=='origin':self.o['origin']=(34.,-117.,0.)
                if fault=='policy':self.o['supervisor']['fusion_policy']='other'
                if fault=='reference':self.o['supervisor']['reference_current']=False
                if fault=='watchdog':self.o['failure']='Control watchdog or ESC permission unavailable'
                if fault=='supervisor':self.o['supervisor']['state']='Fault'
                if fault=='pilot':self.o['mode']=19
                if fault=='jump':self.o['local']=(20.,0.,0.)
                if fault=='ekf':self.o['navigation_hold_safe']=False
                self.step()
                self.assertEqual(self.run.stage,'ABORTED')

    def test_pilot_takeover_not_overridden(self):
        self.dive(); self.o['mode'] = 2
        self.assertEqual(self.step(), [])
        self.assertTrue(self.run.pilot_takeover)
        self.o['mode'] = 4
        self.assertEqual(self.step(), [])

    def test_pilot_mode_change_before_final_disarm_is_respected(self):
        self.dive()
        self.run.enter('DISARM', self.now, 'Surface reached')
        self.o['mode'] = 19
        self.assertEqual(self.step(), [])
        self.assertEqual(self.run.stage, 'ABORTED')
        self.assertTrue(self.run.pilot_takeover)

    def test_pilot_mode_change_during_arm_confirmation(self):
        self.dive()
        self.run.enter('ARM', self.now, 'Waiting for arm confirmation')
        self.o['mode'] = 19
        self.assertEqual(self.step(), [])
        self.assertEqual(self.run.stage, 'ABORTED')

    def test_altitude_metadata_update_preserves_pressure_reference(self):
        self.dive()
        self.o['origin'] = (*self.origin[:2], 15.)
        self.step()
        self.assertEqual(self.run.stage, 'DIVE')
        self.assertEqual(self.run.surface_down, 0.)

    def test_lost_heartbeat_is_not_pilot_takeover(self):
        self.dive();self.o.update(fresh=False, armed=None, mode=None)
        self.assertEqual(self.step()[0][0], 'manual')
        self.assertEqual(self.run.stage, 'ABORTED')
        self.assertFalse(self.run.pilot_takeover)
        self.now += 1
        self.assertEqual(self.step()[0][0], 'manual')

    def test_wrong_source_before_arm(self):
        self.step(); self.o['supervisor']['confirmed_source'] = 'Underwater'; self.step()
        self.o['supervisor']['fusion_policy'] = 'invalid'
        self.step(); self.assertEqual(self.run.stage, 'ABORTED')

    def test_coordinates_and_invalid_inputs(self):
        for n,e in ((1,2),(-12,4),(0,0)):
            actual = geo_to_ne(*ne_to_geo(n,e,self.origin),self.origin)
            self.assertLess(math.dist(actual,(n,e)), 1e-7)
        for field,value in (('lat',True),('lon',math.nan),('depth_m',2),('map','unknown')):
            with self.assertRaises(ValueError): validate_plan(dict(self.plan, **{field:value}))

    def test_pressure_zero_need_not_equal_local_down_zero(self):
        self.o['local']=(0.,0.,7.1);self.o['depth']=.1
        self.dive()
        self.assertAlmostEqual(self.run.surface_down,7.)
        self.assertAlmostEqual(self.run.target[2],7.5)

    def test_review_token_expiry_motion_and_duplicate(self):
        api = WaypointAPI(clock=lambda:self.now)
        streams = {k:dict(age=0.,data=v) for k,v in dict(
            heartbeat=dict(armed=False), supervisor=dict(configuration_verified=True,mode='Active',surfaced=True),
            local=dict(ned=[0.,0.,0.]),cube_origin=dict(lat=self.origin[0],lon=self.origin[1],alt=0.)).items()}
        nav = dict(session=1,streams=streams,cube_estimate=dict(valid=True,lat=self.origin[0],lon=self.origin[1],heading_deg=0.))
        p = api.preview(self.plan,nav)
        item, duplicate = api.consume(p['plan_id'],nav)
        self.assertFalse(duplicate)
        altitude_update = copy.deepcopy(nav)
        altitude_update['streams']['cube_origin']['data']['alt'] = 15.
        self.assertFalse(api.consume(p['plan_id'], altitude_update)[1])
        horizontal_update = copy.deepcopy(nav)
        horizontal_update['streams']['cube_origin']['data']['lat'] += .000001
        with self.assertRaises(ValueError): api.consume(p['plan_id'], horizontal_update)
        self.now=31.
        with self.assertRaises(ValueError): api.consume(p['plan_id'],nav)
        self.now=0.; changed=copy.deepcopy(nav);changed['session']=2
        with self.assertRaises(ValueError): api.consume(p['plan_id'],changed)
        item['run_id']='started'
        self.assertTrue(api.consume(p['plan_id'],changed)[1])


if __name__ == '__main__': unittest.main()
