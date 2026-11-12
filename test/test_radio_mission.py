import tempfile
import time
import unittest

from robotx_graey_2026.api.comms.mission_radio import (
    Assembler, KIND_REQUEST, encode, inspect_chunk, revision)
from robotx_graey_2026.api.comms import competition_codec as rx
from robotx_graey_2026.api.comms.radio_mission_node import (
    BeaconGate, MissionBackend, accept_run_start, build_heartbeat, validate_mission)
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


class RunStartTests(unittest.TestCase):
    def test_early_start_is_not_consumed(self):
        payload=rx.pack_run_start(run_id=7)
        self.assertEqual(accept_run_start(250,190,43,payload,9,None,1.),(None,None))

    def test_prepared_mission_confirms_and_acks_exact_frame(self):
        active=dict(competition=True,run_id='local')
        event,ack=accept_run_start(250,190,43,rx.pack_run_start(run_id=7),9,active,1.)
        self.assertEqual((event['run_id'],event['source']),(7,'rfd'))
        self.assertEqual(rx.unpack_ack(ack),dict(msg_id=rx.RUN_START,seq=9,src=rx.NODE_UUV))

    def test_different_run_revokes_but_is_not_acked(self):
        active=dict(competition=True,robocommand_run_id=7)
        event,ack=accept_run_start(250,190,43,rx.pack_run_start(run_id=8),10,active,2.)
        self.assertEqual(event['run_id'],8)
        self.assertIsNone(ack)


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
