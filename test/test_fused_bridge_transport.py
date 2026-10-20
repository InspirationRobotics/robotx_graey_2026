"""Production adapter callbacks with mocked transport; no vehicle access."""
import ast
import json
import math
import statistics
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from robotx_graey_2026.api.navigation.bridge_estimator import BridgeEstimator, yaw
from robotx_graey_2026.api.navigation.odometry_profile import PROFILE
from robotx_graey_2026.api.navigation.automatic_recovery import AutomaticRecovery
from robotx_graey_2026.api.navigation.land_north import LandNorth
from robotx_graey_2026.api.navigation.cube_north import CubeNorth,COMPASS_PARAMS,YAW_PARAMS


class TransportTests(unittest.TestCase):
    def test_passive_capture_while_armed_then_automatic_safe_restoration(self):
        from test_cube_north import CubeNorthTests
        fixture=CubeNorthTests();n=CubeNorth();n.start(0,fixture.params());fixture.completed(n)
        for i in range(31):
            t=1+i*.1;n.observe(t,fixture.params(t,1),90.,30.,True)
        self.assertEqual(n.phase,'captured')
        self.b.cube_north=n;self.b.land_north=LandNorth();self.b.output_permitted=False
        self.b.params=fixture.params(4.,1)
        self.b.control_state=dict(vehicle_fresh=True,armed=True,sa_active=False,esc_commanded_on=True)
        self.b.control_received=4.
        self.b.finish_compass_reference(4.)
        self.assertEqual(n.phase,'captured');self.b.link.send.assert_not_called()
        self.b.control_state.update(armed=False,sa_active=True,esc_commanded_on=False)
        self.b.control_received=4.1
        self.b.est.imu_at=4.1
        self.b.finish_compass_reference(4.1)
        self.assertEqual(n.phase,'restoring')
        self.assertAlmostEqual(self.b.land_north.reference['offset_deg'],60.)
        self.b.link.command.assert_not_called();self.b.link.set_mode.assert_not_called()
        self.b.control_received=5.2
        self.b.finish_compass_reference(5.2)
        self.assertEqual(self.b.link.send.call_count,8)  # four params retried pending ACK
        n.observe(5.3,fixture.params(5.3),0.,0.,False)
        self.assertEqual(n.phase,'ready')

    def test_compass_parameter_polling_does_not_add_load_to_normal_navigation(self):
        self.b.sync_clock=Mock();self.b.request()
        names=[c.args[3].decode() for c in self.b.link.send.call_args_list if c.args[0]=='param_request_read_send']
        self.assertEqual(names,list(PROFILE))
        self.b.link.send.reset_mock();self.b.output_permitted=False;self.b.request()
        names=[c.args[3].decode() for c in self.b.link.send.call_args_list if c.args[0]=='param_request_read_send']
        self.assertTrue(set(COMPASS_PARAMS).issubset(names))

    def test_compass_workflow_is_maintenance_only_and_restores_external_yaw(self):
        self.b.cube_north=CubeNorth();self.b.land_north=LandNorth()
        self.b.output_permitted=False;self.b.get_logger=Mock(return_value=Mock())
        self.b.compass_health=(10.,True);self.b.raw_gps_health=(10.,True)
        self.b.params={k:(10.,v) for k,v in PROFILE.items()}
        self.b.params.update({k:(10.,v) for k,v in dict(COMPASS_ENABLE=1,COMPASS_USE=1,COMPASS_DEV_ID=123,COMPASS_LEARN=0).items()})
        result=self.b.compass_action('compass_start',NS())
        self.assertTrue(result.success,result.message)
        calls=self.b.link.send.call_args_list
        self.assertEqual({c.args[3] for c in calls},{k.encode() for k in (*YAW_PARAMS,'COMPASS_LEARN')})
        self.b.link.command.assert_not_called();self.b.link.set_mode.assert_not_called()
        self.b.params.update({k:(10.,1) for k in YAW_PARAMS})
        self.b.params['COMPASS_LEARN']=(10.,3)
        self.b.cube_north.observe(10.,self.b.params,90.,30.,True)
        self.b.cube_north.message(10.,'CompassLearn: Finished')
        self.b.params['COMPASS_LEARN']=(10.,0)
        for i in range(31):
            t=10+i*.1
            self.b.params={k:(t,v[1]) for k,v in self.b.params.items()}
            self.b.cube_north.observe(t,self.b.params,90.,30.,True)
        self.clock.now=13.
        self.b.est.imu_at=13.
        result=self.b.compass_action('compass_capture',NS())
        self.assertTrue(result.success,result.message)
        self.assertAlmostEqual(self.b.land_north.reference['offset_deg'],60.)
        self.assertEqual(self.b.cube_north.phase,'restoring')
        self.assertFalse(self.b.output_permitted)
        self.b.compass_action.__func__.__globals__['require_maintenance'].side_effect=ValueError('Keep killed')
        self.b.link.send.reset_mock()
        self.assertFalse(self.b.compass_action('compass_cancel',NS()).success)
        self.b.link.send.assert_not_called()

    def test_land_capture_then_prepare_retains_north_without_land_depth_or_dvl(self):
        self.b.land_north=LandNorth()
        for i in range(51):self.b.land_north.observe(7.5+i*.05,(1.,0.,0.,0.),(0.,0.,0.))
        self.b.output_permitted=False
        self.b.get_logger=Mock(return_value=Mock())
        request=NS(parameters=[NS(name='true_heading_deg',value=NS(type=3,double_value=90.))])
        response=self.b.align_land(request,NS(result=NS()))
        self.assertTrue(response.result.successful,response.result.reason)
        self.assertFalse(self.b.output_permitted)
        self.b.params={k:(10.,v) for k,v in PROFILE.items()}
        self.b.pressures=deque((8.+i*.1,1000.) for i in range(21))
        response=self.b.prepare(NS(data=True),NS())
        self.assertTrue(response.success,response.message)
        self.assertTrue(self.b.est.heading_validated)
        self.assertEqual(self.b.est.north_offset,90.)
        self.assertFalse(self.b.est.initialized)

    def test_known_heading_service_requires_maintenance_and_explicit_double(self):
        self.b.est.align_known_heading=Mock(return_value='captured')
        self.b.est.north_alignment=Mock(status=Mock(return_value={}))
        self.b.get_logger=Mock(return_value=Mock())
        request=NS(parameters=[NS(name='true_heading_deg',value=NS(type=3,double_value=90.))])
        response=NS(result=NS())
        self.b.align_heading(request,response)
        self.assertTrue(response.result.successful)
        self.b.est.align_known_heading.assert_called_once_with(10.,90.)
        self.b.est.align_known_heading.reset_mock()
        self.b.align_heading.__func__.__globals__['require_maintenance'].side_effect=ValueError('Keep killed')
        self.b.align_heading(request,response)
        self.assertFalse(response.result.successful)
        self.b.est.align_known_heading.assert_not_called()

    def test_known_heading_service_rejects_missing_prepare_and_wrong_type(self):
        self.b.est.align_known_heading=Mock()
        response=NS(result=NS())
        for enabled,parameters in ((False,[NS(name='true_heading_deg',value=NS(type=3,double_value=90.))]),
                                   (True,[]),(True,[NS(name='true_heading_deg',value=NS(type=2))])):
            self.b.output_permitted=enabled
            self.b.align_heading(NS(parameters=parameters),response)
            self.assertFalse(response.result.successful)
        self.b.est.align_known_heading.assert_not_called()

    def setUp(self):
        path = Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/fused_nav_bridge.py'
        tree = ast.parse(path.read_text())
        tree.body = [n for n in tree.body if isinstance(n, ast.ClassDef)]
        self.clock = NS(now=10.)
        ns = dict(SteadyNode=object, math=math, json=json, String=NS, PROFILE=PROFILE,
                  yaw=yaw, time=NS(monotonic=lambda: self.clock.now),
                  statistics=statistics, BridgeEstimator=BridgeEstimator, require_maintenance=Mock(), AutomaticRecovery=AutomaticRecovery, COMPASS_PARAMS=COMPASS_PARAMS)
        exec(compile(tree, str(path), 'exec'), ns)
        self.b = ns['FusedNavBridge'].__new__(ns['FusedNavBridge'])
        self.b.est = BridgeEstimator()
        self.b.auto_recovery=AutomaticRecovery()
        self.b.control_state={}
        self.b.control_received=None
        self.b.stamps, self.b.params = {}, {}
        self.b.boot_ms = self.b.last_sent = None
        self.b.start_checked = True
        self.b.output_permitted = True
        self.b.reset_counter = 0
        self.b.pressures = deque(maxlen=80)
        self.b.vn_calibration = (10.,dict(phase='idle'))
        self.b.settings = dict(enabled=True, surface_pressure_hpa=1000., water_density=1000.)
        self.b.link, self.b.pub = Mock(), Mock()
        self.b.instance = 'test'
        self.b.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(self.clock.now*1e9)))
        self.b.est.gps.clock.offset = 0.
        self.b.est.gps.clock.uncertainty = 0.
        self.b.est.gps.clock.updated = 10.

    def pressure(self, stamp, system=1):
        m = NS(time_boot_ms=stamp, press_abs=1000.49,
               get_srcSystem=lambda: system, get_srcComponent=lambda: 1)
        self.b.consume('SCALED_PRESSURE3', m)

    def test_pressure_requires_clock_freshness_calibration_and_cube_sender(self):
        self.pressure(10000, system=42)
        self.assertIsNone(self.b.est.depth_at)
        self.pressure(10000)
        self.assertAlmostEqual(self.b.est.depth, .005, places=4)
        self.clock.now = 11.
        self.pressure(10100)
        self.assertIsNone(self.b.est.depth_at)
        self.b.settings['surface_pressure_hpa'] = 0.
        self.pressure(11000)
        self.assertIsNone(self.b.est.depth_at)

    def test_old_queued_dvl_is_not_freshened(self):
        self.b.dvl(NS(data=json.dumps(dict(stamp_ns=9000000000, valid=True,
                                          velocity=[0, 0, 0], sensor_time=1))))
        self.assertFalse(self.b.est.motion_valid)

    def test_air_barometers_cannot_replace_water_pressure_or_qualify_submerged_gps(self):
        self.b.settings['surface_pressure_hpa']=992.7
        m=NS(time_boot_ms=10000,press_abs=979.,get_srcSystem=lambda:1,get_srcComponent=lambda:1)
        for kind in ('SCALED_PRESSURE','SCALED_PRESSURE2'):
            self.b.consume(kind,m)
        self.assertIsNone(self.b.est.depth_at)
        m.press_abs=992.7+1000*9.80665*.7/100
        self.b.consume('SCALED_PRESSURE3',m)
        self.assertAlmostEqual(self.b.est.depth,.7)
        self.assertFalse(self.b.est.gps_eligible(10.))
        self.assertEqual(self.b.pressure_sample['source'],'SCALED_PRESSURE3')

    def test_explicit_pressure_source_is_used_without_fallback(self):
        self.b.settings['pressure_message']='SCALED_PRESSURE2'
        m=NS(time_boot_ms=10000,press_abs=1049.03325,get_srcSystem=lambda:1,get_srcComponent=lambda:1)
        self.b.consume('SCALED_PRESSURE3',m)
        self.assertIsNone(self.b.est.depth_at)
        self.b.consume('SCALED_PRESSURE2',m)
        self.assertAlmostEqual(self.b.est.depth,.5)

    def test_nonfinite_pressure_blocks_output_without_crashing_status(self):
        for pressure in (math.nan, math.inf):
            self.clock.now += .1
            stamp = round(self.clock.now*1000)
            m = NS(time_boot_ms=stamp, press_abs=pressure,
                   get_srcSystem=lambda: 1, get_srcComponent=lambda: 1)
            self.b.consume('SCALED_PRESSURE3', m)
            self.b.tick()
            status = json.loads(self.b.pub.publish.call_args.args[0].data)
            self.assertIsNone(status['pressure_sample']['hpa'])
            self.assertIsNone(status['depth_m'])
            self.assertFalse(status['healthy'])
            self.b.link.odometry.assert_not_called()

    def test_profile_readback_expires_and_inhibits_output(self):
        self.b.params = {k: (10., v) for k, v in PROFILE.items()}
        self.assertTrue(self.b.configured(10.))
        self.assertFalse(self.b.configured(13.))
        self.b.tick()
        self.b.link.odometry.assert_not_called()
        self.b.link.command.assert_not_called()
        self.b.link.send.assert_not_called()

    def test_cube_restart_latches_fault(self):
        self.pressure(10000)
        self.pressure(100)
        self.assertIn('restarted', self.b.est.fault)

    def test_armed_start_latches_fault_and_missing_heartbeat_blocks_output(self):
        self.b.start_checked = False
        self.b.params = {k: (10., v) for k, v in PROFILE.items()}
        self.b.est.status = lambda now: dict(healthy=not self.b.est.fault)
        self.b.tick()
        self.b.link.odometry.assert_not_called()
        self.b.consume('HEARTBEAT', NS(base_mode=128, get_srcSystem=lambda: 1, get_srcComponent=lambda: 1))
        self.assertIn('started while armed', self.b.est.fault)
        self.b.tick()
        self.b.link.odometry.assert_not_called()
        self.b.consume('HEARTBEAT', NS(base_mode=0, get_srcSystem=lambda: 1, get_srcComponent=lambda: 1))
        self.assertTrue(self.b.est.fault)  # Must restart; no automatic recovery.

    def test_prepare_requires_confirmed_surface_and_restarts_alignment_with_reset_counter(self):
        self.b.params = {k:(10.,v) for k,v in PROFILE.items()}
        self.b.pressures.extend((8.+i*.1,1000.+i*.001) for i in range(21))
        self.b.est.set_origin((33.,-117.,0.))
        self.b.est.initialized = True
        self.b.est.position = [20.,10.,.5]
        old = self.b.est
        response = self.b.prepare(NS(data=False),NS())
        self.assertFalse(response.success)
        self.assertIs(self.b.est,old)
        response = self.b.prepare(NS(data=True),NS())
        self.assertTrue(response.success,response.message)
        self.assertEqual(self.b.est.origin,(33.,-117.,0.))
        self.assertFalse(self.b.est.initialized)
        self.assertIsNone(self.b.est.north_alignment.offset)
        self.assertEqual(self.b.reset_counter,1)
        self.assertTrue(self.b.output_permitted)
        self.b.tick()
        self.b.link.odometry.assert_not_called()
        self.b.link.command.assert_not_called()

    def test_prepare_rejects_unstable_pressure_or_failed_calibration(self):
        self.b.params = {k:(10.,v) for k,v in PROFILE.items()}
        for phase,spread in (('failed',0.),('collecting',0.),('idle',20.)):
            self.b.vn_calibration = (10.,dict(phase=phase))
            self.b.pressures = deque((8.+i*.1,1000.+spread*i/20) for i in range(21))
            response = self.b.prepare(NS(data=True),NS())
            self.assertFalse(response.success)
            self.assertEqual(self.b.reset_counter,0)

    def test_profile_never_selects_native_gps(self):
        for slot in (1, 2, 3):
            for axis in ('POSXY', 'VELXY', 'POSZ', 'YAW'):
                self.assertEqual(PROFILE[f'EK3_SRC{slot}_{axis}'], 6)

    def test_fast_receive_polling_does_not_flood_odometry(self):
        self.b.params = {k: (10., v) for k, v in PROFILE.items()}
        self.b.est.status = lambda now: dict(healthy=True)
        self.b.tick()
        self.clock.now = 10.01
        self.b.tick()
        self.assertEqual(self.b.link.odometry.call_count, 1)
        self.clock.now = 10.06
        self.b.tick()
        self.assertEqual(self.b.link.odometry.call_count, 2)
