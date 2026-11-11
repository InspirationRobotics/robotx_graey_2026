"""gui_node reads the gateway's summaries (Crusader's gcs_bridge pattern, Nov 11): pose, attitude and arm/mode from
/graey/fcu/pose|attitude|status, each with its true arrival time; the raw stream keeps a short queue."""
import ast
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

from pymavlink.dialects.v20 import ardupilotmega as mav

from robotx_graey_2026.api.gui.navigation_view import NavigationView
from robotx_graey_2026.api.pixhawk.mavlink import mavutil
from robotx_graey_2026.api.pixhawk.ros_link import RosTransport

GUI = Path(__file__).parents[1]/'robotx_graey_2026/api/gui/gui_node.py'


def gui_names(*names):
    """SummaryFrame, SUMMARY_KINDS, RAW_RX_DEPTH from gui_node without importing ROS."""
    tree = ast.parse(GUI.read_text())
    keep = [n for n in tree.body if (isinstance(n, ast.ClassDef) and n.name in names)
            or (isinstance(n, ast.Assign) and any(getattr(t, 'id', None) in names for t in n.targets))]
    ns = {}
    exec(compile(ast.Module(body=keep, type_ignores=[]), str(GUI), 'exec'), ns)
    return ns


def summary(message):
    """What the gateway publishes on /graey/fcu/<name>: the decoded fields."""
    return json.loads(json.dumps(message.to_dict()))


class SummaryTests(unittest.TestCase):
    def setUp(self):
        self.ns = gui_names('SummaryFrame', 'SUMMARY_KINDS', 'RAW_RX_DEPTH')
        self.now = [100.0]
        self.view = NavigationView(clock=lambda: self.now[0])
        self.Frame = self.ns['SummaryFrame']

    def test_pose_attitude_and_heartbeat_reach_the_view_with_their_true_age(self):
        pose = mav.MAVLink_global_position_int_message(5000, int(1.2807711e7), int(103.85464e7), 1000, -500,
                                                        10, 20, 0, 28657)
        att = mav.MAVLink_attitude_message(5000, 0.1, -0.05, 1.5, 0, 0, 0)
        hb = mav.MAVLink_heartbeat_message(mav.MAV_TYPE_SUBMARINE, mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                           mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mav.MAV_MODE_FLAG_SAFETY_ARMED,
                                           4, 4, 3)
        for kind, message in (('GLOBAL_POSITION_INT', pose), ('ATTITUDE', att), ('HEARTBEAT', hb)):
            self.view.consume(kind, self.Frame(kind, summary(message)), received=99.7)
        snap = self.view.snapshot()
        streams = snap['streams']
        self.assertAlmostEqual(streams['qgc_global']['age'], 0.3, places=6)
        self.assertTrue(streams['qgc_global']['fresh'])
        self.assertAlmostEqual(streams['qgc_global']['data']['lat'], 1.2807711, places=6)
        self.assertAlmostEqual(streams['qgc_global']['data']['heading_deg'], 286.57, places=2)
        self.assertAlmostEqual(streams['attitude']['data']['rpy'][2], 85.94, places=1)
        self.assertEqual(streams['heartbeat']['data'], dict(armed=True, mode_id=4))
        self.assertEqual(mavutil.mode_string_v10(self.Frame('HEARTBEAT', summary(hb))), 'GUIDED')

    def test_an_old_summary_stays_old(self):
        att = mav.MAVLink_attitude_message(5000, 0., 0., 0., 0, 0, 0)
        self.view.consume('ATTITUDE', self.Frame('ATTITUDE', summary(att)), received=97.0)
        self.assertFalse(self.view.snapshot()['streams']['attitude']['fresh'])

    def test_summaries_cover_the_display_kinds_and_raw_queue_fits_the_freshness_window(self):
        self.assertEqual(set(self.ns['SUMMARY_KINDS'].values()), {'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT'})
        self.assertLessEqual(self.ns['RAW_RX_DEPTH'], 0.25*235)     # the gateway's ~235 frames/s on Graey

    def test_ros_transport_queue_depth_is_optional_and_defaults_unchanged(self):
        msgs = Mock(String=Mock())                  # no ROS needed: only the subscription call is checked
        with patch.dict(sys.modules, {'std_msgs': Mock(msg=msgs), 'std_msgs.msg': msgs}):
            for depth, want in ((None, 200), (50, 50)):
                node = Mock()
                RosTransport(node, 195, 1) if depth is None else RosTransport(node, 195, 1, depth)
                self.assertEqual(node.create_subscription.call_args[0][3], want)


if __name__ == '__main__':
    unittest.main()
