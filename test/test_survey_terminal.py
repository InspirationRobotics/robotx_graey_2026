"""tools/survey_run.py terminal mode (SE + the QGC plan on the Cube) on the semantic gateway.

The gateway only publishes kinds in FCU_RX_KINDS and RosTransport refuses a consumer that doesn't declare its
kinds, so survey_run must declare RC_CHANNELS and the two mission-download replies, and those replies must
arrive in order (a dropped item breaks the download)."""
import importlib.util
import json
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from pymavlink.dialects.v20 import ardupilotmega as mav

from robotx_graey_2026.api.pixhawk.ros_link import FCU_RX_KINDS, ORDERED_RX_KINDS, RosTransport, mav_topic

ROOT = Path(__file__).parents[1]


def load_survey_run():
    spec = importlib.util.spec_from_file_location('survey_run_under_test', ROOT/'tools/survey_run.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fake_ros():
    string_type = type('String', (), {'__init__': lambda self, data='': setattr(self, 'data', data)})
    module = Mock(String=string_type)
    return string_type, patch.dict(sys.modules, {'std_msgs': Mock(msg=module), 'std_msgs.msg': module})


def gateway_payload(message, system=3, component=1):
    """What telemetry_gateway publishes on /graey/fcu/mav/<kind>."""
    return json.dumps(dict(received_monotonic=time.monotonic(), kind=message.get_type(), source_system=system,
                           source_component=component, sequence=0, flightmode=None, fields=message.to_dict()))


class TransportFcu:
    """survey_run's Fcu with the ROS node replaced: replies go through a real RosTransport."""

    def __init__(self, survey, replies):
        self.String, context = fake_ros()
        with context:
            self.node = Mock()
            self.transport = RosTransport(self.node, survey.COMPONENT, 3, kinds=survey.Fcu.KINDS)
        self.replies, self.sent = replies, []
        self.survey = survey

    def send(self, name, *args):
        self.sent.append(name)
        for message in self.replies.pop(name, []):
            self.transport.receive_kind(message.get_type(), self.String(data=gateway_payload(message)))

    def recv(self, types, timeout):
        return self.survey.Fcu.recv(self, types, timeout)

    @property
    def link(self):
        return self.transport


class SurveyTerminalTests(unittest.TestCase):
    def test_declared_kinds_are_published_by_the_gateway_and_mission_replies_are_lossless(self):
        survey = load_survey_run()
        self.assertEqual(set(survey.Fcu.KINDS), {'RC_CHANNELS', 'MISSION_COUNT', 'MISSION_ITEM_INT'})
        self.assertLessEqual(set(survey.Fcu.KINDS), FCU_RX_KINDS)
        self.assertLessEqual({'MISSION_COUNT', 'MISSION_ITEM_INT'}, ORDERED_RX_KINDS)
        _, context = fake_ros()
        with context:
            node = Mock()
            RosTransport(node, survey.COMPONENT, 3, rx_depth=200, kinds=survey.Fcu.KINDS)
        topics = {call.args[1]: call.args[3] for call in node.create_subscription.call_args_list}
        self.assertEqual(set(topics), {mav_topic(k) for k in survey.Fcu.KINDS} | {mav_topic('HEARTBEAT')})
        self.assertEqual(topics[mav_topic('MISSION_ITEM_INT')], 20)      # ordered queue, not latest-only

    def test_read_plan_returns_the_qgc_waypoints_after_home(self):
        survey = load_survey_run()
        item = lambda seq, lat, lon, cmd=16: mav.MAVLink_mission_item_int_message(
            survey.COMPONENT, 3, seq, 0, cmd, 0, 1, 0, 0, 0, 0, int(lat*1e7), int(lon*1e7), 0, 0)
        fcu = TransportFcu(survey, {
            'mission_request_list_send': [mav.MAVLink_mission_count_message(255, 0, 4, 0)],
        })
        items = [item(0, 1.29, 103.85), item(1, 1.2806, 103.8557), item(2, 0, 0), item(3, 1.2810, 103.8560)]
        pending = iter(items)
        original = fcu.send

        def send(name, *args):
            if name == 'mission_request_int_send':
                fcu.replies[name] = [next(pending)]
            original(name, *args)
        fcu.send = send
        points = survey.read_plan(fcu)
        self.assertEqual(points, [(1.2806, 103.8557), (1.281, 103.856)])     # home and the 0,0 item are skipped
        self.assertEqual(fcu.sent[-1], 'mission_ack_send')


if __name__ == '__main__':
    unittest.main()
