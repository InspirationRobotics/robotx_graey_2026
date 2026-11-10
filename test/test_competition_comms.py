"""Offline durable competition reporting and measured telemetry tests."""
import tempfile
import unittest
from types import SimpleNamespace
from pathlib import Path
from robotx_graey_2026.api.comms.competition_wifi import CompetitionSender
from robotx_graey_2026.api.comms.competition_telemetry import CompetitionTelemetry
from robotx_graey_2026.api.comms import competition_codec as rx

def sync(sender,run=70000,now=0,state='RUNNING'):
    sender.receive({'ocs_info':{'graey_run':dict(run_id=run,state=state)}},now)

def status(sender,now=0,depth=1.23):
    sender.update_status(now=now,depth_m=depth,robot_state=rx.STATE_AUTO,lat=1.2806,lon=103.8557,
        spd_mps=.4,hdg_deg=90,roll_deg=1,pitch_deg=2,alt_m=5.9,task=3)

class TestOutbox(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'outbox.sqlite3'
        self.sender = CompetitionSender(self.path,wall=lambda:1790000000.)
        self.addCleanup(lambda:self.sender.close())

    def record(self):
        sync(self.sender)
        return self.sender.survey('survey-1',1.2806,103.8557,[1,2,1])

    def receipt(self,seq,verdict,kind='pipeline_survey'):
        self.sender.receive({'ocs_receipt':dict(vehicle_seq=seq,kind=kind,verdict=verdict)},0)

    def test_requires_official_run(self):
        with self.assertRaises(ValueError):self.sender.survey('s',1,2,[1])

    def test_u32_run_without_truncation(self):
        self.record()
        self.assertEqual(self.sender.run,70000)

    def test_no_transmission_until_surface_known(self):
        self.record();status(self.sender)
        self.assertEqual(self.sender.due(0),[])

    def test_underwater_records_but_sends_nothing(self):
        self.record();status(self.sender);self.sender.set_surface(False,0)
        self.assertEqual(self.sender.due(0),[])
        self.assertEqual(self.sender.status(0)['pending'],1)

    def test_two_hz_heartbeat_measured_depth(self):
        sync(self.sender);self.sender.set_surface(True,0);status(self.sender)
        self.assertEqual(self.sender.due(0)[0]['heartbeat']['depth_m'],1.23)
        self.assertEqual(self.sender.due(.49),[])
        self.assertEqual(len(self.sender.due(.5)),1)

    def test_surface_and_telemetry_expire(self):
        sync(self.sender);status(self.sender);self.sender.set_surface(True,0)
        self.assertEqual(self.sender.due(2),[])
        self.sender.set_surface(True,2)
        self.assertEqual(self.sender.due(2),[])

    def test_executor_jitter_does_not_accumulate_heartbeat_drift(self):
        self.sender.set_surface(True,0)
        status(self.sender)
        self.sender.due(0)
        self.sender.due(.53)
        self.assertEqual(len(self.sender.due(1.01)),1)

    def test_stable_observation_id_is_idempotent(self):
        seq = self.record()
        self.assertEqual(self.sender.survey('survey-1',1.2806,103.8557,[1,2,1]),seq)
        self.assertEqual(self.sender.status(0)['pending'],1)
        with self.assertRaises(ValueError):self.sender.survey('survey-1',1.2806,103.8557,[2])

    def test_wrong_receipt_kind_cannot_clear_observation(self):
        seq = self.record();self.receipt(seq,'acknowledged','resource_delivery')
        self.assertEqual(self.sender.status(0)['pending'],1)

    def test_publish_is_not_broker_ack(self):
        seq = self.record();self.receipt(seq,'published')
        self.assertEqual(self.sender.status(0)['pending'],1)
        self.receipt(seq,'acknowledged');self.receipt(seq,'published')
        self.assertEqual(self.sender.status(0)['broker_acknowledged'],1)

    def test_rejected_observation_remains_visible(self):
        seq = self.record();self.receipt(seq,'dropped')
        self.assertEqual(self.sender.status(0)['rejected'],1)
        self.assertEqual(self.sender.db.execute('SELECT count(*) FROM events').fetchone()[0],1)

    def test_restart_preserves_report_and_forgets_surface_and_remote(self):
        self.record();self.sender.set_surface(True,0)
        original = self.sender.due(0)[0]
        self.sender.close();self.sender = CompetitionSender(self.path)
        self.assertEqual(self.sender.due(0),[])
        sync(self.sender);self.sender.set_surface(True,0)
        self.assertEqual(self.sender.due(0),[original])

    def test_other_run_is_quarantined(self):
        self.record();sync(self.sender,run=70001);self.sender.set_surface(True,0)
        self.assertEqual(self.sender.due(0),[])
        self.assertEqual(self.sender.status(0)['previous_run_pending'],1)

    def test_reconnect_needs_run_confirmation(self):
        self.record();self.sender.set_surface(True,0);self.sender.disconnected()
        self.assertEqual(self.sender.due(0),[])
        sync(self.sender)
        self.assertEqual(len(self.sender.due(0)),1)

    def test_remote_confirmation_expires(self):
        self.record();self.sender.set_surface(True,6)
        self.assertEqual(self.sender.due(6),[])

    def test_invalid_observations_do_not_enter_outbox(self):
        sync(self.sender)
        for segments in ([],[0],[True],[3],list(range(9))):
            with self.assertRaises(ValueError):self.sender.survey('s',1,2,segments)
        with self.assertRaises(ValueError):self.sender.resource('r',0,1)
        self.assertEqual(self.sender.status(0)['pending'],0)

    def test_resource_has_official_task_and_colors(self):
        sync(self.sender);self.sender.resource('r',1,4);self.sender.set_surface(True,0)
        body = self.sender.due(0)[0]['graey_report']['report']['resource_delivery']
        self.assertEqual(body,dict(task='TASK_INFRA_SURVEY_REPAIR',resource_color='COLOR_RED',delivery_circle_color='COLOR_ANY'))

class TestTelemetry(unittest.TestCase):
    def setUp(self):
        self.sender = CompetitionSender(':memory:')
        self.addCleanup(self.sender.close)
        self.telemetry = CompetitionTelemetry(lambda:100.)
        self.sample = dict(cube_estimate=dict(valid=True,lat=1.28,lon=103.85,heading_deg=90),streams={
            'bridge':dict(age=.1,data=dict(surfaced=True,navigation_valid=True,depth_m=.3)),
            'control':dict(age=.1,data=dict(sa_active=False,esc_commanded_on=True,autonomous=True,armed=True,vehicle_fresh=True)),
            'attitude':dict(age=.1,data=dict(rpy=[0,0,90])),
            'local':dict(age=.1,data=dict(velocity=[.3,.4,0]))})
        self.telemetry.view.snapshot = lambda:self.sample
        self.telemetry.hae,self.telemetry.hae_at = 5.9,100

    def test_measured_fields_produce_heartbeat(self):
        self.telemetry.update(self.sender,100)
        self.assertIsNone(self.telemetry.reason)
        self.assertEqual(self.sender.heartbeat_body['spd_mps'],.5)
        self.assertEqual(self.sender.heartbeat_body['depth_m'],.3)

    def test_missing_hae_suppresses_heartbeat(self):
        self.telemetry.hae_at = None
        self.telemetry.update(self.sender,100)
        self.assertIsNone(self.sender.status_at)
        self.assertIn('ellipsoid',self.telemetry.reason)

    def test_control_kill_reports_commanded_permission(self):
        self.sample['streams']['control']['data']['esc_commanded_on'] = False
        self.telemetry.update(self.sender,100)
        self.assertEqual(self.sender.heartbeat_body['state'],'STATE_KILLED')

    def test_stale_bridge_immediately_pauses_transmission(self):
        self.telemetry.update(self.sender,100)
        self.sample['streams']['bridge']['age'] = 1
        self.telemetry.update(self.sender,100)
        self.assertIsNone(self.sender.surface)
        self.assertIsNone(self.sender.status_at)

    def test_bad_depth_suppresses_heartbeat(self):
        self.sample['streams']['bridge']['data']['depth_m'] = None
        self.telemetry.update(self.sender,100)
        self.assertIsNone(self.sender.status_at)

    def gps(self,fix=3,source=1,stamp=123,alt_ellipsoid=5900):
        return SimpleNamespace(get_srcSystem=lambda:source,get_srcComponent=lambda:1,
            fix_type=fix,time_usec=stamp,alt_ellipsoid=alt_ellipsoid,lat=12806000,lon=1038557000,
            alt=10000,eph=100,satellites_visible=15,h_acc=500,v_acc=1000)

    def test_repeated_gps_timestamp_does_not_refresh_hae(self):
        self.telemetry.consume('GPS_RAW_INT',self.gps())
        self.telemetry.clock = lambda:102.
        self.telemetry.consume('GPS_RAW_INT',self.gps())
        self.assertEqual(self.telemetry.hae_at,100.)

    def test_lost_gps_fix_invalidates_hae_even_with_same_timestamp(self):
        self.telemetry.consume('GPS_RAW_INT',self.gps())
        self.telemetry.consume('GPS_RAW_INT',self.gps(fix=1))
        self.assertIsNone(self.telemetry.hae_at)

    def test_foreign_vehicle_gps_is_ignored(self):
        self.telemetry.consume('GPS_RAW_INT',self.gps(source=200,alt_ellipsoid=999000))
        self.assertEqual(self.telemetry.hae,5.9)

    def test_missing_hae_extension_does_not_become_zero_altitude(self):
        self.telemetry.consume('GPS_RAW_INT',self.gps(alt_ellipsoid=0))
        self.assertIsNone(self.telemetry.hae_at)

    def test_stale_cube_control_invalidates_state(self):
        self.sample['streams']['control']['data']['vehicle_fresh'] = False
        self.telemetry.update(self.sender,100)
        self.assertIn('Cube control',self.telemetry.reason)

    def test_task_claim_expires_and_manual_takeover_clears_reported_task(self):
        self.telemetry.set_task('TASK_INFRA_SURVEY_REPAIR')
        self.telemetry.update(self.sender,100.)
        self.assertEqual(self.sender.heartbeat_body['current_task'],'TASK_INFRA_SURVEY_REPAIR')
        self.sample['streams']['control']['data']['autonomous']=False
        self.telemetry.update(self.sender,100.)
        self.assertEqual(self.sender.heartbeat_body['current_task'],'TASK_NONE')
        self.sample['streams']['control']['data']['autonomous']=True
        self.telemetry.hae_at=102.1
        self.telemetry.update(self.sender,102.1)
        self.assertEqual(self.sender.heartbeat_body['current_task'],'TASK_NONE')


class TestCurrentMissionHeartbeat(unittest.TestCase):
    setUp = TestTelemetry.setUp
    def test_active_armed_mission_is_auto_in_guided_or_manual_mode(self):
        self.sample['streams']['control']['data'].update(autonomous=False,mission_claimed=True)
        self.telemetry.set_task('  task_infra_survey_repair  ')
        self.telemetry.update(self.sender,100.)
        self.assertEqual(self.sender.heartbeat_body['state'],'STATE_AUTO')
        self.assertEqual(self.sender.heartbeat_body['current_task'],'TASK_INFRA_SURVEY_REPAIR')
        self.sample['streams']['control']['data']['armed']=False
        self.telemetry.update(self.sender,100.)
        self.assertEqual(self.sender.heartbeat_body['state'],'STATE_MANUAL')
        self.assertEqual(self.sender.heartbeat_body['current_task'],'TASK_NONE')

    def test_invalid_task_ignored_without_stand_down(self):
        self.telemetry.set_task('TASK_INFRA_SURVEY_REPAIR')
        with self.assertRaises(ValueError):self.telemetry.set_task('typo')
        self.telemetry.update(self.sender,100.)
        self.assertEqual(self.sender.heartbeat_body['current_task'],'TASK_INFRA_SURVEY_REPAIR')
