import tempfile
import time
import unittest

from robotx_graey_2026.api.comms.mission_radio import (
    Assembler, KIND_REQUEST, encode, inspect_chunk, revision)
from robotx_graey_2026.api.comms import competition_codec as rx
from robotx_graey_2026.api.comms.competition_telemetry import CompetitionTelemetry
from robotx_graey_2026.api.comms.radio_mission_node import (
    BeaconGate, HeartbeatCapture, MissionBackend, ResourceRadio, StartRepeat, accept_run_start,
    build_heartbeat, validate_mission)
from robotx_graey_2026.api.comms.run_gate import RunGate
from robotx_graey_2026.api.gui.laptop.radio_client import RadioMissionClient


MISSION=dict(kind='waypoints',competition=True,points=[
    dict(kind='waypoint',lat=1.2801,lon=103.8552,name='Waypoint 1'),
    dict(kind='waypoint',lat=1.2802,lon=103.8553,name='Waypoint 2')])


class ProtocolTests(unittest.TestCase):
    def test_out_of_order_duplicate_safe_round_trip(self):
        value=dict(op='prepare',mission=MISSION,padding='x'*1200)
        chunks=encode(value,17,KIND_REQUEST)
        self.assertGreater(len(chunks),1)
        a=Assembler()
        self.assertIsNone(a.add(chunks[-1]))
        self.assertIsNone(a.add(chunks[-1]))
        complete=None
        for chunk in chunks[:-1]:complete=a.add(chunk) or complete
        self.assertEqual(complete,(KIND_REQUEST,17,value))

    def test_corruption_and_header_bounds_are_rejected(self):
        chunks=encode(dict(op='status'),9,KIND_REQUEST)
        bad=bytearray(chunks[0]);bad[-1]^=1
        with self.assertRaisesRegex(ValueError,'checksum'):
            Assembler().add(bad)
        with self.assertRaises(ValueError):inspect_chunk(b'x')

    def test_revision_is_canonical(self):
        self.assertEqual(revision({'b':2,'a':1}),revision({'a':1,'b':2}))


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.calls=[]
        class Fake(MissionBackend):
            def _post(inner,body):
                self.calls.append(body)
                return {'ok':True,'plan_id':'plan-1'} if body['action']=='preview' else {'ok':True,'run_id':'run-1'}
        self.backend=Fake(self.temp.name)

    def tearDown(self):self.temp.cleanup()

    def test_prepare_stores_exact_revision_then_uses_checked_gui_api(self):
        result=self.backend.handle(dict(op='prepare',mission=MISSION))
        self.assertEqual(result['revision'],revision(validate_mission(MISSION)))
        self.assertEqual([c['action'] for c in self.calls],['preview','start'])
        self.assertTrue(self.calls[-1]['competition'])
        self.assertEqual(self.backend.status()['active']['run_id'],'run-1')

    def test_survey_competition_is_forwarded(self):
        mission=dict(kind='survey',competition=True,points=MISSION['points'])
        self.backend.handle(dict(op='prepare',mission=mission))
        self.assertEqual(self.calls,[dict(action='survey_start',points=MISSION['points'],
                                          dry=False,competition=True)])

    def test_invalid_mission_never_reaches_backend(self):
        with self.assertRaises(ValueError):self.backend.handle(dict(op='prepare',mission={}))
        self.assertEqual(self.calls,[])

    def test_terminal_status_releases_the_active_slot(self):
        self.backend.handle(dict(op='prepare',mission=MISSION))
        self.backend.note_waypoint(dict(run_id='run-1',stage='COMPLETE'))
        self.assertIsNone(self.backend.active)

    def test_ip_prepared_competition_mission_is_claimed_for_rfd_runstart(self):
        self.backend.note_waypoint(dict(run_id='ip-run',stage='WAIT_RUN_START',competition=True,
            plan=dict(kind='radio_route',operator_kind='surface_route')))
        self.assertEqual(self.backend.active['source'],'ip')
        self.assertEqual(self.backend.active['kind'],'surface_route')
        event,ack=accept_run_start(250,190,43,rx.pack_run_start(run_id=12),4,
                                   self.backend.active,1.)
        self.assertEqual(event['run_id'],12)
        self.assertIsNotNone(ack)

    def test_noncompetition_ip_mission_is_not_claimed(self):
        self.backend.note_waypoint(dict(run_id='local',stage='GUIDED',competition=False,
                                        plan=dict(kind='radio_route')))
        self.assertIsNone(self.backend.active)

    def test_ip_survey_is_claimed_and_terminal_result_releases_it(self):
        status=dict(run_id='survey-ip',running=True,competition=True)
        self.backend.note_survey(status)
        self.assertEqual(self.backend.active['kind'],'survey')
        self.backend.note_survey(dict(status,running=False,result='DONE'))
        self.assertIsNone(self.backend.active)

    def test_recording_status_drops_topic_catalogue_for_rfd(self):
        self.backend._request=lambda path,body=None:dict(session={'active':True},topics=[{'name':'/large'}])
        self.assertEqual(self.backend.handle(dict(op='recording_status')),
                         dict(session={'active':True}))


class RunStartTests(unittest.TestCase):
    def test_early_start_is_not_consumed(self):
        payload=rx.pack_run_start(run_id=7)
        self.assertEqual(accept_run_start(250,190,43,payload,9,None,1.),(None,None))

    def test_prepared_mission_confirms_and_acks_exact_frame(self):
        active=dict(competition=True,run_id='local')
        event,ack=accept_run_start(250,190,43,rx.pack_run_start(run_id=7),9,active,1.)
        self.assertEqual((event['run_id'],event['source']),(7,'rfd'))
        self.assertEqual(rx.unpack_ack(ack),dict(msg_id=rx.RUN_START,seq=9,src=rx.NODE_OCS))

    def test_any_run_id_is_acked_once_a_mission_waits(self):
        # 13 Nov 2026: the run ID is ignored. Every RunStart for a waiting mission is ACKed,
        # so fleet_link stops resending it on the shared air.
        active=dict(competition=True,run_id='se',se_mission=True,robocommand_run_id=7,state='WAIT_RUN_START',ready=True,status_at=1.9)
        event,ack=accept_run_start(250,190,43,rx.pack_run_start(run_id=8),10,active,2.)
        self.assertEqual(event['run_id'],8)
        self.assertIsNotNone(ack)
        self.assertEqual(accept_run_start(250,190,43,rx.pack_run_start(run_id=8),10,None,2.),(None,None))
        self.assertEqual(accept_run_start(255,190,43,rx.pack_run_start(run_id=8),10,active,2.),(None,None))


    def test_se_ack_requires_fresh_surface_wait_and_is_scoped_to_attempt(self):
        active=dict(competition=True,se_mission=True,run_id='local',state='ARM',ready=False,status_at=1.)
        receive=lambda:accept_run_start(250,190,43,rx.pack_run_start(run_id=0),12,active,2.)
        self.assertEqual(receive(),(None,None))
        active.update(state='WAIT_RUN_START',ready=True)
        self.assertEqual(receive(),(None,None))
        active.update(status_at=1.9)
        event,ack=receive()
        self.assertEqual(event['mission_run_id'],'local')
        self.assertEqual(rx.unpack_ack(ack)['src'],rx.NODE_OCS)
        active.update(state='FORWARD',admitted=True)
        self.assertEqual(receive(),(None,ack))
        active.update(se_mission=False,robocommand_run_id=1)
        self.assertIsNone(receive()[1])

class ResourceRadioTests(unittest.TestCase):
    REQUEST=dict(observation_id='chris-simple-dive:run-1',resource_color=4,delivery_circle_color=3)

    def test_colour_report_is_rxl_resource_delivery_for_task_two(self):
        r=ResourceRadio()
        self.assertTrue(r.offer(self.REQUEST))
        self.assertEqual(rx.unpack_resource_delivery(r.due(0.,True)),
                         dict(task=3,resource_color=4,delivery_circle_color=3))

    def test_repeats_are_bounded_and_wait_for_a_live_heartbeat(self):
        r=ResourceRadio();r.offer(self.REQUEST)
        self.assertIsNone(r.due(0.,False))                 # antenna under / no GPS yet
        times=[t/10 for t in range(0,600) if r.due(t/10,True) is not None]
        self.assertEqual(len(times),ResourceRadio.SENDS)
        self.assertLessEqual(times[-1]-times[0],ResourceRadio.WINDOW_S)
        self.assertTrue(all(b-a>=ResourceRadio.PERIOD_S-1e-9 for a,b in zip(times,times[1:])))

    def test_same_observation_is_queued_once_new_one_restarts(self):
        r=ResourceRadio()
        self.assertTrue(r.offer(self.REQUEST))
        r.due(0.,True)
        self.assertFalse(r.offer(dict(self.REQUEST)))      # the mission republishes at 2 Hz
        self.assertEqual(r.status()['sent'],1)
        self.assertTrue(r.offer(dict(self.REQUEST,observation_id='chris-simple-dive:run-2')))
        self.assertEqual(r.status()['sent'],0)
        with self.assertRaises(ValueError):r.offer(dict(observation_id='x',resource_color=0,
                                                        delivery_circle_color=1))


class StartRepeatTests(unittest.TestCase):
    EVENT=dict(run_id=4464,remote_run_id=4464,remote_state='RUNNING',run_confirmed=True,source='rfd',
               received_monotonic=10.,published_monotonic=10.)

    def test_repeats_fresh_while_the_same_mission_waits(self):
        r=StartRepeat();r.accepted(self.EVENT,'se-1',10.)
        for now in (10.5,11.,30.):
            e=r.due(dict(run_id='se-1',state='WAIT_RUN_START'),now)
            self.assertEqual((e['run_id'],e['received_monotonic'],e['published_monotonic']),(4464,now,now))

    def test_stops_once_the_mission_moves_on_ends_or_is_another(self):
        for active in (dict(run_id='se-1',state='FORWARD'),None,dict(run_id='se-2',state='WAIT_RUN_START')):
            r=StartRepeat();r.accepted(self.EVENT,'se-1',10.)
            self.assertIsNone(r.due(active,11.))
            self.assertIsNone(r.due(dict(run_id='se-1',state='WAIT_RUN_START'),11.5))   # gone for good

    def test_gives_up_after_its_limit(self):
        r=StartRepeat();r.accepted(self.EVENT,'se-1',0.)
        self.assertIsNone(r.due(dict(run_id='se-1',state='WAIT_RUN_START'),StartRepeat.LIMIT_S+1))


class RunGateIgnoringRunIdTests(unittest.TestCase):
    def sample(self,run,state='RUNNING',now=1.):
        return dict(run_id=run,remote_run_id=run,remote_state=state,run_confirmed=state=='RUNNING',
                    published_monotonic=now)

    def test_first_runstart_admits_and_nothing_later_revokes(self):
        gate=RunGate(True,match_run=False)
        gate.update(self.sample(5),1.);self.assertTrue(gate.admit(1.,True))
        for later in (self.sample(6,now=2.),self.sample(5,'ENDED',now=3.)):
            gate.update(later,later['published_monotonic'])
            self.assertFalse(gate.revoked)
            self.assertTrue(gate.admit(later['published_monotonic'],True))

    def test_default_gate_still_matches_runs(self):
        gate=RunGate(True)
        gate.update(self.sample(5),1.);gate.admit(1.,True)
        gate.update(self.sample(6,now=2.),2.)
        self.assertTrue(gate.revoked)


class HeartbeatTests(unittest.TestCase):
    def sample(self, **changes):
        value=dict(robot_state=rx.STATE_AUTO,lat=1.2806,lon=103.8557,spd_mps=.4,
                   hdg_deg=90.,roll_deg=1.,pitch_deg=2.,alt_m=5.9,task=3)
        value.update(changes)
        return value

    def test_uuv_auto_heartbeat_is_complete_and_truthful(self):
        value=rx.unpack_heartbeat(build_heartbeat(self.sample()))
        self.assertEqual((value['vehicle'],value['robot_state'],value['task']),
                         (rx.NODE_UUV,rx.STATE_AUTO,3))
        self.assertEqual((value['pos_valid'],value['att_valid'],value['alt_valid']),(True,True,True))
        self.assertAlmostEqual(value['alt_m'],5.9)

    def test_incomplete_heartbeat_is_suppressed_with_a_specific_reason(self):
        with self.assertRaisesRegex(ValueError,'alt_m'):
            build_heartbeat(self.sample(alt_m=None))

    def test_capture_survives_every_navigation_surface_state(self):
        # Regression: the bridge reports surfaced=True/False once depth is valid.
        # The capture must accept that exactly as the WiFi sender does.
        for surfaced in (None,True,False):
            telemetry=CompetitionTelemetry(time.monotonic);capture=HeartbeatCapture()
            telemetry.put('bridge',dict(published_monotonic=time.monotonic(),
                                        surfaced=surfaced,depth_m=.2))
            telemetry.update(capture,time.monotonic())
            self.assertIs(capture.surface,surfaced)
            self.assertIsNone(capture.value)  # no Cube position yet: withheld, not crashed
            self.assertIsNotNone(telemetry.reason)

    def test_private_status_is_change_driven_and_not_a_second_2hz_stream(self):
        gate=BeaconGate();event=dict(event='radio_status',ok=True)
        self.assertTrue(gate.due(event,0.,heartbeat_available=True))
        self.assertFalse(gate.due(event,.5,heartbeat_available=True))
        self.assertFalse(gate.due(event,4.99,heartbeat_available=True))
        self.assertTrue(gate.due(event,5.,heartbeat_available=True))
        self.assertTrue(gate.due(dict(event='radio_status',ok=False),5.1,True))

    def test_private_route_presence_is_one_hz_when_heartbeat_is_withheld(self):
        gate=BeaconGate();event=dict(event='radio_status',ok=False)
        self.assertTrue(gate.due(event,0.,heartbeat_available=False))
        self.assertFalse(gate.due(event,.99,heartbeat_available=False))
        self.assertTrue(gate.due(event,1.,heartbeat_available=False))


class LaptopClientTests(unittest.TestCase):
    class Tunnel:
        def __init__(self,ptype,payload,system=43,component=201):
            self.payload_type=ptype;self.payload_length=len(payload);self.payload=payload
            self.target_system=0;self.target_component=0
            self.system,self.component=system,component
        def get_srcSystem(self):return self.system
        def get_srcComponent(self):return self.component

    def test_decoded_standard_uuv_heartbeat_establishes_link_and_telemetry(self):
        now=[10.]
        client=RadioMissionClient(start=False,clock=lambda:now[0]);self.addCleanup(client.close)
        payload=rx.pack_heartbeat(vehicle=rx.NODE_UUV,robot_state=rx.STATE_AUTO,
            flight_phase=0,pos_valid=True,att_valid=True,alt_valid=True,
            lat=1.28,lon=103.85,spd_mps=.2,hdg_deg=90,roll_deg=0,pitch_deg=0,alt_m=2,task=3)
        self.assertTrue(client.consume(self.Tunnel(rx.payload_type(rx.HEARTBEAT),payload)))
        self.assertTrue(client.status()['connected'])
        self.assertEqual(client.status()['heartbeat']['robot_state'],rx.STATE_AUTO)

    def test_foreign_tunnel_does_not_claim_rfd_connection(self):
        client=RadioMissionClient(start=False);self.addCleanup(client.close)
        self.assertFalse(client.consume(self.Tunnel(0x9999,b'x')))
        self.assertFalse(client.status()['connected'])

    def test_request_crosses_the_tunnel_and_waits_for_matching_ack(self):
        client=RadioMissionClient(start=False)
        self.addCleanup(client.close)
        client.last_seen=time.monotonic()

        class Mav:
            def __init__(inner):inner.assembler=Assembler()
            def tunnel_send(inner,target_system,target_component,payload_type,length,payload):
                self.assertEqual((target_system,target_component,payload_type), (43,201,0x9001))
                complete=inner.assembler.add(bytes(payload)[:length])
                if complete is None:return
                kind,request_id,body=complete
                self.assertEqual((kind,body),(KIND_REQUEST,dict(op='status')))
                with client.condition:
                    client.responses[request_id]=dict(ok=True,result=dict(active=None))
                    client.condition.notify_all()
        class Connection:
            mav=Mav()
            def close(inner):pass
        client.connection=Connection()
        self.assertEqual(client.request(dict(op='status'),timeout=1.)['result'],dict(active=None))

    def test_no_ack_never_claims_the_mission_was_confirmed(self):
        client=RadioMissionClient(start=False)
        self.addCleanup(client.close)
        client.last_seen=time.monotonic()
        class Mav:
            calls=0
            def tunnel_send(inner,*args):inner.calls+=1
        class Connection:
            mav=Mav()
            def close(inner):pass
        client.connection=Connection()
        with self.assertRaisesRegex(ValueError,'not confirmed'):
            client.request(dict(op='status'),timeout=.15)
        self.assertGreater(Connection.mav.calls,0)


if __name__=='__main__':unittest.main()
