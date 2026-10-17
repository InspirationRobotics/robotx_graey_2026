"""Execute production HTTP operation helpers with process/ROS boundaries mocked."""
import ast
from pathlib import Path
import queue
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock


class WaypointGuiTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/gui/gui_node.py'
        tree=ast.parse(path.read_text())
        tree.body=[n for n in tree.body if isinstance(n,ast.FunctionDef)
                   and n.name in ('waypoint_request','any_mission_running','mission_stop')]
        self.item=dict(plan=dict(map='geographic'),run_id=None)
        self.ns=dict(navigation=NS(snapshot=lambda:dict(streams={})),
            waypoint_api=NS(consume=lambda *args:(self.item,False)),
            waypoint_capabilities=lambda:dict(enabled=False,simulation=False),
            waypoint_process=None,waypoint_launch=None,waypoint_aborts=queue.Queue(),
            calibration_jobs=None,
            north_process=None,north_aborts=queue.Queue(),
            pids_for=lambda name:[],MISSION_EXECUTABLES=('waypoint_mission',),
            subprocess=NS(Popen=Mock()),link=Mock())
        exec(compile(tree,str(path),'exec'),self.ns)

    def test_client_cannot_enable_live_execution(self):
        with self.assertRaisesRegex(ValueError,'not enabled'):
            self.ns['waypoint_request'](dict(action='start',plan_id='p',commissioned=True,simulation=True))
        self.ns['subprocess'].Popen.assert_not_called()

    def test_live_pool_target_cannot_bypass_registration(self):
        self.ns['waypoint_capabilities']=lambda:dict(enabled=True,simulation=False)
        self.item['plan']['map']='pool'
        with self.assertRaisesRegex(ValueError,'survey required'):
            self.ns['waypoint_request'](dict(action='start',plan_id='p'))

    def test_pending_child_blocks_second_mission_before_pid_discovery(self):
        self.ns['waypoint_process']=NS(poll=lambda:None)
        self.assertTrue(self.ns['any_mission_running']())

    def test_alignment_child_excludes_waypoint_and_stop_requests_abort(self):
        self.ns['north_process']=NS(poll=lambda:None)
        self.assertTrue(self.ns['any_mission_running']())
        self.assertTrue(self.ns['mission_stop']()[0])
        self.assertTrue(self.ns['north_aborts'].get_nowait())
        self.ns['link'].disarm.assert_not_called()

    def test_legacy_stop_cancels_starting_waypoint_without_disarm_rearm_race(self):
        self.ns.update(waypoint_launch=dict(run_id='r'),waypoint_process=NS(poll=lambda:None))
        self.assertTrue(self.ns['mission_stop']()[0])
        self.assertEqual(self.ns['waypoint_aborts'].get_nowait(),'r')
        self.ns['link'].disarm.assert_not_called()
        self.assertTrue(self.ns['waypoint_request'](dict(action='abort',run_id='r'))['ok'])

    def test_duplicate_start_returns_original_run_without_new_process(self):
        self.item['run_id']='original'
        self.ns['waypoint_api'].consume=lambda *args:(self.item,True)
        self.assertEqual(self.ns['waypoint_request'](dict(action='start',plan_id='p'))['run_id'],'original')
        self.ns['subprocess'].Popen.assert_not_called()

    def test_stop_after_gui_restart_uses_reported_waypoint_run(self):
        self.ns['pids_for']=lambda name:[1] if name=='waypoint_mission' else []
        self.assertFalse(self.ns['mission_stop']()[0])
        self.ns['link'].disarm.assert_not_called()
        self.ns['navigation'].snapshot=lambda:dict(streams=dict(waypoint=dict(data=dict(run_id='active'))))
        self.assertTrue(self.ns['mission_stop']()[0])
        self.assertEqual(self.ns['waypoint_aborts'].get_nowait(),'active')
