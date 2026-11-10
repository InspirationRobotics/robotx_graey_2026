"""Offline GRAEY bytes -> actual Ekko callbacks / ground -> OCS protobuf path.

Requires the local rx26_uav/rx26_ocs integration checkouts beside GRAEY. Never
starts a ROS node, vehicle connection, MQTT client loop, or real RoboCommand.
"""
import ast
import dataclasses
import json
from pathlib import Path
import socket
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

from robotx_graey_2026.api.comms import sub_link as sl
from robotx_graey_2026.api.comms.mesh_link import tunnel_args
from test_sub_link import heartbeat

ROOT = Path(__file__).resolve().parents[2]
OCS = ROOT / 'rx26_ocs'
UAV = ROOT / 'rx26_uav'
ASV = ROOT / 'rx26_asv'


def methods(path, classname, names, globals):
    """Exercise production callback bodies without importing ROS on Windows."""
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == classname)
    selected = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    if {n.name for n in selected} != set(names):
        raise AssertionError('production callback disappeared')
    module = ast.Module(body=[ast.ClassDef(name='Callbacks', bases=[], keywords=[],
                                         body=selected, decorator_list=[])], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(path), 'exec'), globals)
    return globals['Callbacks']


class Task2IntegrationTests(unittest.TestCase):
    def setUp(self):
        if not (OCS.is_dir() and UAV.is_dir()):
            self.skipTest('local Ekko/OCS integration checkouts are required')
        for path in (OCS, UAV/'uav_common', UAV/'uav_groundstation', ASV/'crusader_link'):
            sys.path.insert(0, str(path))
            self.addCleanup(lambda p=str(path): sys.path.remove(p))
        from robotx_graey_2026.api.pixhawk.mavlink import mavutil
        from rx_bridge.bridge import Bridge
        from rx_bridge.config import load
        from rx_bridge.proto import rx_reports_pb2
        from fleet.sub_receiver import forward_tunnel
        from fleet import sub_link as ground_codec
        from uav_common import sub_link as ekko_codec
        self.mavutil = mavutil
        self.proto = rx_reports_pb2
        self.forward = forward_tunnel
        self.assertEqual(sl.pack_sub(heartbeat(), run_id=7, session_id=15, sequence=1),
                         ekko_codec.pack_sub(heartbeat(), run_id=7, session_id=15, sequence=1))
        # Working-tree line endings may differ on Windows; catalogue source and
        # packed wire bytes must agree independently of checkout EOL conversion.
        self.assertEqual((UAV/'uav_common/uav_common/sub_link.py').read_text(),
                         (OCS/'fleet/sub_link.py').read_text())
        self.assertEqual(Path(sl.__file__).read_text(), (OCS/'fleet/sub_link.py').read_text())
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        cfg = dataclasses.replace(load(OCS/'bridge.toml'),
                                  seq_store=Path(self.temp.name)/'seq.json',
                                  wire_log=Path(self.temp.name)/'wire')
        self.bridge = Bridge(cfg, log=lambda s: None)
        self.addCleanup(self.bridge.wire.close)
        self.bridge.machine.on_connect()
        self.bridge.machine.on_course(object())
        self.bridge.machine.on_declared(1)
        self.bridge.machine.on_run_start(1, 7)
        self.published = []
        self.bridge.rc.publish = lambda topic, payload, qos: self.published.append((topic, payload, qos))
        String = lambda **kw: types.SimpleNamespace(**kw)
        callbacks = methods(UAV/'uav_fcu/uav_fcu/telemetry_bridge.py', 'TelemetryBridge',
                            ['_sub_received', '_sub_delivery_begun'],
                            dict(json=json, String=String, sub_link=ekko_codec,
                                 boat_link=types.SimpleNamespace(body=lambda m: bytes(m.payload)[:m.payload_length])))
        self.ekko = callbacks()
        self.ekko._sub = ekko_codec.Receiver()
        self.ekko._sub_pubs = {kind: Mock() for kind in ('sub', 'pipeline_survey', 'resource_delivery')}
        self.envelopes = []
        self.ekko._sub_ocs_pub = Mock()
        self.ekko._sub_ocs_pub.publish.side_effect = lambda msg: self.envelopes.append(json.loads(msg.data))
        self.ekko.get_logger = lambda: Mock()
        self.ground = ground_codec.Receiver()

    def wire(self, ptype, payload, source_system=43):
        encoder = self.mavutil.mavlink.MAVLink(None, srcSystem=source_system, srcComponent=191)
        raw = encoder.tunnel_encode(*tunnel_args(ptype, payload)).pack(encoder)
        return self.mavutil.mavlink.MAVLink(None).parse_buffer(raw)[0]

    def submit(self, envelope):
        return self.bridge._on_tcp_frame(json.dumps(envelope).encode())

    def report(self, index=-1):
        msg = self.proto.RxReport()
        msg.ParseFromString(self.published[index][1])
        return msg

    def test_status_survey_colors_reach_ekko_and_ocs_with_uuv_attribution(self):
        fixtures = [
            (sl.PAYLOAD_SUB, sl.pack_sub(heartbeat(), run_id=7, session_id=15, sequence=1)),
            (sl.PAYLOAD_PIPELINE_SURVEY, sl.pack_pipeline_survey(heartbeat()['position'],
                ['PIPELINE_SEGMENT_INTACT', 'PIPELINE_SEGMENT_DAMAGED', 'PIPELINE_SEGMENT_INTACT'],
                run_id=7, session_id=15, sequence=2)),
            (sl.PAYLOAD_RESOURCE_REQUEST, sl.pack_resource_request('COLOR_BLUE', 'COLOR_RED',
                run_id=7, session_id=15, sequence=3))]
        for ptype, payload in fixtures:
            self.ekko._sub_received(self.wire(ptype, payload))
            self.assertEqual(self.submit(self.envelopes[-1]), 'UUV1')
        reports = [self.report(i) for i in range(3)]
        self.assertEqual([r.WhichOneof('body') for r in reports],
                         ['heartbeat', 'pipeline_survey', 'resource_delivery'])
        self.assertTrue(all(r.vehicle_id == 'UUV1' and r.team_id == 'ASTA' for r in reports))
        self.assertEqual(reports[0].heartbeat.current_task, 3)
        self.assertEqual(list(reports[1].pipeline_survey.segments), [1, 2, 1])
        self.assertEqual(reports[2].resource_delivery.resource_color, 3)
        self.assertEqual(reports[2].resource_delivery.delivery_circle_color, 1)
        self.assertEqual(self.ekko._sub.resource['resource_delivery']['resource_color'], 'COLOR_BLUE')

    def test_direct_ground_and_ekko_relay_publish_only_one_copy(self):
        payload = sl.pack_resource_request('COLOR_BLUE', 'COLOR_RED', run_id=7, session_id=15, sequence=3)
        msg = self.wire(sl.PAYLOAD_RESOURCE_REQUEST, payload)
        self.ekko._sub_received(msg)
        queue = Mock()
        queue.publish_event.side_effect = lambda envelope: self.submit(envelope)
        self.forward(msg, self.ground, queue)
        self.submit(self.envelopes[-1])
        self.assertEqual(len(self.published), 1)
        self.assertEqual(self.report().vehicle_id, 'UUV1')

    def test_delivery_echo_requires_explicit_mission_event_and_keeps_both_colors(self):
        payload = sl.pack_resource_request('COLOR_BLUE', 'COLOR_RED', run_id=7, session_id=15, sequence=3)
        self.ekko._sub_received(self.wire(sl.PAYLOAD_RESOURCE_REQUEST, payload))
        self.assertEqual(len(self.envelopes), 1)
        self.submit(self.envelopes[0])
        self.ekko._sub_delivery_begun(types.SimpleNamespace(data=json.dumps(
            dict(run_id=7, session_id=15, sequence=3))))
        self.assertEqual(len(self.envelopes), 2)
        self.submit(self.envelopes[-1])
        report = self.report()
        self.assertEqual(report.vehicle_id, 'UAV1')
        self.assertEqual(report.resource_delivery.resource_color, 3)
        self.assertEqual(report.resource_delivery.delivery_circle_color, 1)

    def test_old_run_and_malformed_packet_do_not_publish(self):
        payload = sl.pack_resource_request('COLOR_BLUE', 'COLOR_RED', run_id=6, session_id=15, sequence=3)
        self.ekko._sub_received(self.wire(sl.PAYLOAD_RESOURCE_REQUEST, payload))
        self.submit(self.envelopes[-1])
        self.assertEqual(self.published, [])
        count = len(self.envelopes)
        self.ekko._sub_received(self.wire(sl.PAYLOAD_RESOURCE_REQUEST, payload[:-1]))
        self.assertEqual(len(self.envelopes), count)

    def test_event_retry_after_failed_tcp_write_and_heartbeat_does_not_replace_it(self):
        from uav_groundstation.ocs_link import OcsLink, FrameReader
        link = OcsLink('127.0.0.1', 1, log=lambda m: None)
        sock = Mock()
        sock.recv.side_effect = socket.timeout
        sock.sendall.side_effect = OSError('offline fixture')
        link._sock = sock
        event = {'report': {'vehicle_id': 'UUV1', 'resource_delivery': {'resource_color': 'COLOR_BLUE'}}}
        link.publish_event(event)
        link.publish({'heartbeat': {'current_task': 'TASK_INFRA_SURVEY_REPAIR'}})
        self.assertFalse(link._pump())
        self.assertEqual(len(link._events), 1)
        sock.sendall.side_effect = None
        self.assertTrue(link._pump())
        reader = FrameReader()
        actual = json.loads(reader.feed(sock.sendall.call_args.args[0])[0])
        self.assertEqual(actual, event)
        self.assertEqual(len(link._events), 0)
        self.assertIsNotNone(link._pending)

    def test_actual_ekko_ocs_callback_routes_uuv_and_uav_reports_to_correct_links(self):
        from uav_groundstation import heartbeat_core
        callbacks = methods(UAV/'uav_groundstation/uav_groundstation/ocs_client_node.py',
                            'OcsClient', ['_on_sub_report', '_on_delivery_ended', '_build'],
                            dict(json=json, heartbeat_core=heartbeat_core,
                                 rfc3339=lambda: '2026-10-31T00:00:00Z'))
        client = callbacks()
        client.p = dict(team_id='ASTA', vehicle_id='UAV1', fake_telemetry=False,
                        geoid_separation_m=0, airborne_alt_m=1)
        client._delivery_task_active = False
        def cache(**values):
            result = Mock()
            result.get.return_value = types.SimpleNamespace(**values)
            return result
        client._pose = cache(latitude=32.9, longitude=-117., ground_speed=1.,
                             heading=123., altitude_amsl=10., altitude_rel=5.)
        client._status = cache(mode='AUTO', armed=True)
        client._att = cache(roll=0., pitch=0.)
        client._flight = cache(valid=True, landed_state=2)
        client._quiet_reason = client._phase_source = None
        client.link, client.sub_link = Mock(), Mock()
        client.get_logger = lambda: Mock()
        packet = sl.unpack_resource_request(sl.pack_resource_request(
            'COLOR_BLUE', 'COLOR_RED', run_id=7, session_id=15, sequence=3))
        envelope = sl.ocs_envelope(packet)
        client._on_sub_report(types.SimpleNamespace(data=json.dumps(envelope)))
        client.sub_link.publish_event.assert_called_once()
        client.link.publish_event.assert_not_called()
        self.assertFalse(client._delivery_task_active)
        self.assertEqual(client._build(10)['heartbeat']['current_task'], 'TASK_NONE')
        client._on_sub_report(types.SimpleNamespace(data=json.dumps(sl.ocs_envelope(packet, 'UAV1'))))
        client.link.publish_event.assert_called_once()
        self.assertTrue(client._delivery_task_active)
        self.assertEqual(client._build(10)['heartbeat']['current_task'], 'TASK_INFRA_SURVEY_REPAIR')
        client._on_delivery_ended(types.SimpleNamespace(data=''))
        self.assertFalse(client._delivery_task_active)
        self.assertEqual(client._build(10)['heartbeat']['current_task'], 'TASK_NONE')

    def test_graey_producer_callbacks_publish_only_supplied_observations(self):
        callbacks = methods(ROOT/'robotx_graey_2026/robotx_graey_2026/api/comms/sub_link_node.py',
                            'SubLinkNode', ['_input', '_tick'],
                            dict(json=json, time=types.SimpleNamespace(monotonic=lambda: 10),
                                 String=lambda **kw: types.SimpleNamespace(**kw)))
        node = callbacks()
        node.sender = sl.Sender(run_id=7, session_id=15)
        node.mesh = Mock()
        node.mesh.poll.return_value = []
        node.received = Mock()
        node.get_logger = lambda: Mock()
        node._tick()
        node.mesh.send_tunnel.assert_not_called()
        node._input('status', json.dumps(heartbeat()))
        node._input('pipeline_survey', json.dumps(dict(active_buoy_position=heartbeat()['position'],
                     segments=['PIPELINE_SEGMENT_INTACT', 'PIPELINE_SEGMENT_DAMAGED'])))
        node._input('resource_request', json.dumps(dict(resource_color='COLOR_BLUE',
                                                       delivery_circle_color='COLOR_RED')))
        node._tick()
        self.assertEqual([c.args[0] for c in node.mesh.send_tunnel.call_args_list], list(sl.SUB_TYPES))
        for call in node.mesh.send_tunnel.call_args_list:
            self.ekko._sub_received(self.wire(*call.args))
            self.submit(self.envelopes[-1])
        self.assertEqual(len(self.published), 3)

    def test_invalid_queued_event_does_not_block_later_heartbeat(self):
        from uav_groundstation.ocs_link import OcsLink
        link = OcsLink('127.0.0.1', 1, log=lambda m: None)
        sock = Mock()
        sock.recv.side_effect = socket.timeout
        link._sock = sock
        link.publish_event({'bad': object()})
        link.publish({'heartbeat': {}})
        self.assertTrue(link._pump())
        self.assertEqual(len(link._events), 0)
        self.assertTrue(link._pump())
        sock.sendall.assert_called_once()

    def test_idle_then_task2_initiation_follow_the_actual_run(self):
        from rx_bridge.runstate import RunMachine
        self.bridge.machine = RunMachine()
        self.bridge.machine.on_connect()
        self.bridge.machine.on_course(object())
        self.bridge.machine.on_declared(1)
        idle = sl.unpack_sub(sl.pack_sub(heartbeat('TASK_NONE'),
                                        run_id=0, session_id=15, sequence=1))
        self.submit(sl.ocs_envelope(idle))
        self.assertEqual(self.report().heartbeat.current_task, 1)
        self.bridge.machine.on_run_start(1, 7)
        active = sl.unpack_sub(sl.pack_sub(heartbeat(), run_id=7, session_id=15, sequence=2))
        self.submit(sl.ocs_envelope(active))
        self.assertEqual(self.report().heartbeat.current_task, 3)
        self.assertEqual(self.report().vehicle_id, 'UUV1')

    def test_underwater_restart_then_surface_reports_reach_ekko_and_ground(self):
        path = Path(self.temp.name)/'underwater-outbox.json'
        sender = sl.Sender(run_id=7, session_id=15, outbox_path=path)
        sender.set_submerged(True)
        sender.survey_complete(heartbeat()['position'], ['PIPELINE_SEGMENT_DAMAGED', 'PIPELINE_SEGMENT_INTACT'])
        sender.resource_observed('COLOR_BLUE', 'COLOR_RED')
        self.assertEqual(sender.due(100), [])
        restored = sl.Sender(outbox_path=path)
        self.assertEqual(restored.due(500), [])
        restored.set_submerged(False)
        queue = Mock()
        queue.publish_event.side_effect = self.submit
        for ptype, payload in restored.due(500):
            msg = self.wire(ptype, payload)
            self.ekko._sub_received(msg)
            self.submit(self.envelopes[-1])
            self.forward(msg, self.ground, queue)
        self.assertEqual(len(self.published), 2)
        self.assertEqual(list(self.report(0).pipeline_survey.segments), [2, 1])
        self.assertEqual(self.report(1).resource_delivery.resource_color, 3)
        self.assertEqual(self.report(1).resource_delivery.delivery_circle_color, 1)

    def test_boat_and_sub_ignore_each_others_application_packets(self):
        if not ASV.is_dir():
            self.skipTest('local Crusader checkout is required')
        from crusader_link import tunnel_link as tl
        from crusader_link.ekko_map import EkkoMap
        callbacks = methods(ASV/'crusader_link/crusader_link/rfd_link_node.py',
                            'RfdLinkNode', ['_heard'],
                            dict(time=types.SimpleNamespace(monotonic=lambda: 99), tl=tl,
                                 RadioFrame=types.SimpleNamespace(DIR_RX=2)))
        boat = callbacks()
        boat.p = dict(source_system=42, uav_sysid=200)
        boat._record = Mock()
        boat._ekko = EkkoMap()
        boat._last_rx, boat._quiet_warned = 5, True
        fixtures = [(sl.PAYLOAD_SUB, sl.pack_sub(heartbeat(), run_id=7, session_id=15, sequence=1)),
                    (sl.PAYLOAD_PIPELINE_SURVEY, sl.pack_pipeline_survey(heartbeat()['position'],
                      ['PIPELINE_SEGMENT_INTACT'], run_id=7, session_id=15, sequence=2)),
                    (sl.PAYLOAD_RESOURCE_REQUEST, sl.pack_resource_request('COLOR_BLUE', 'COLOR_RED',
                      run_id=7, session_id=15, sequence=3))]
        for src in (43, 200):
            for ptype, payload in fixtures:
                boat._heard(self.wire(ptype, payload, source_system=src))
                self.assertIsNone(boat._ekko.hear(ptype, payload))
        self.assertFalse(boat._ekko.heard())
        self.assertEqual(boat._last_rx, 5)
        self.assertTrue(boat._quiet_warned)
        self.assertEqual(boat._record.call_count, 6)  # Diagnostics still hear the shared mesh.
        sub = sl.Receiver()
        self.assertIsNone(sub.hear(tl.PAYLOAD_BOAT, tl.pack_boat(32.9, -117., 1)))
        self.assertIsNone(sub.status)
        self.assertIsNone(sub.resource)

    def test_uuv_heartbeat_expires_in_relay_queue_but_resource_does_not(self):
        from uav_groundstation.ocs_link import OcsLink
        link = OcsLink('127.0.0.1', 1, log=lambda m: None, heartbeat_max_age_s=1)
        sock = Mock()
        sock.recv.side_effect = socket.timeout
        link._sock = sock
        with patch('uav_groundstation.ocs_link.time.monotonic', return_value=0):
            link.publish({'report': {'heartbeat': {}}})
            link.publish_event({'report': {'resource_delivery': {'resource_color': 'COLOR_BLUE'}}})
        with patch('uav_groundstation.ocs_link.time.monotonic', return_value=100):
            self.assertTrue(link._pump())
            self.assertTrue(link._pump())
        sock.sendall.assert_called_once()  # Resource retained; no stale heartbeat is revived.


if __name__ == '__main__':
    unittest.main()
