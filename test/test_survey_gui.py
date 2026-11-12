"""Ruth's survey run as a GUI mission (Nov 11): gui_node's survey actions and SurveyLauncher, ROS mocked."""
import ast
import json
import os
from pathlib import Path
import queue
import signal
import tempfile
import time
import uuid
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from robotx_graey_2026.api.gui.survey_launch import SurveyLauncher, validate_points
from robotx_graey_2026.api.gui.mission_store import validate_mission

# start buoy and end buoy ~11 m apart, a path point before them
POINTS = [dict(lat=1.2812, lon=103.8544), dict(lat=1.28125, lon=103.8545), dict(lat=1.28125, lon=103.8546)]


class FakeProcess:
    def __init__(self):
        self.code, self.signals = None, []

    def poll(self):
        return self.code

    @property
    def returncode(self):
        return self.code

    def send_signal(self, sig):
        self.signals.append(sig)


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.proc = FakeProcess()
        self.calls = []

        def popen(argv, **kw):
            self.calls.append((argv, kw))
            return self.proc
        self.launcher = SurveyLauncher(self.dir.name, popen=popen)

    def tearDown(self):
        self.dir.cleanup()

    def test_points_need_start_and_end_buoys_apart(self):
        with self.assertRaisesRegex(ValueError, 'at least 2'):
            validate_points(POINTS[:1])
        with self.assertRaisesRegex(ValueError, '1 m apart'):
            validate_points([POINTS[0], POINTS[1], dict(POINTS[1])])
        with self.assertRaisesRegex(ValueError, 'Invalid'):
            validate_points([POINTS[0], dict(lat='x', lon=1)])
        self.assertEqual(validate_points(POINTS)[-1], [1.28125, 103.8546])

    def test_start_runs_survey_run_in_gui_mode_from_the_source_tree(self):
        reply = self.launcher.start(POINTS, dry=True)
        argv, kw = self.calls[0]
        self.assertTrue(argv[2].replace('\\','/').endswith('tools/survey_run.py'))
        self.assertEqual(json.loads(argv[argv.index('--points')+1]), validate_points(POINTS))
        self.assertEqual(argv[argv.index('--run-id')+1], reply['run_id'])
        self.assertIn('--dry', argv)
        self.assertNotIn('--competition',argv)
        self.assertTrue(kw['env']['PYTHONPATH'].startswith(kw['cwd']))
        self.assertTrue(self.launcher.running() and self.launcher.owns(reply['run_id']))
        self.assertFalse(self.launcher.owns('other'))
        with self.assertRaisesRegex(ValueError, 'already running'):
            self.launcher.start(POINTS)

    def test_stop_signals_and_snapshot_reports_result(self):
        run_id = self.launcher.start(POINTS)['run_id']
        with self.assertRaisesRegex(ValueError, 'run ID'):
            self.launcher.stop('wrong')
        self.launcher.stop(run_id)
        self.assertEqual(self.proc.signals, [getattr(signal,'SIGUSR1',signal.SIGTERM)])
        Path(self.launcher.launch['log']).write_text('a\nSURVEY STOPPED\n')
        self.proc.code = 2
        snap = self.launcher.snapshot()
        self.assertEqual((snap['running'], snap['result'], snap['log'][-1]), (False, 'STOPPED', 'SURVEY STOPPED'))
        self.assertFalse(self.launcher.owns(run_id))

    def test_dry_must_be_boolean(self):
        with self.assertRaisesRegex(ValueError, 'boolean'):
            self.launcher.start(POINTS, dry='yes')
        self.assertEqual(self.calls, [])

    def test_saved_survey_mission_round_trips(self):
        mission = validate_mission(dict(kind='survey', points=[dict(p, kind='waypoint') for p in POINTS]))
        self.assertEqual((mission['kind'], len(mission['points'])), ('survey', 3))
        with self.assertRaisesRegex(ValueError, 'at least 2'):
            validate_mission(dict(kind='survey', points=[dict(POINTS[0], kind='waypoint')]))


class GuiSurveyActionTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).parents[1]/'robotx_graey_2026/api/gui/gui_node.py'
        tree = ast.parse(path.read_text())
        tree.body = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                     and n.name in ('waypoint_request', 'any_mission_running', 'mission_stop')]
        self.item = dict(plan=dict(map='geographic'), run_id=None)
        self.survey = Mock()
        self.survey.running.return_value = False
        self.survey.owns.side_effect = lambda run_id: run_id == 'survey-1'
        self.control = dict(online=True, vehicle_fresh=True, inhibit=None, sa_active=False, esc_commanded_on=True)
        self.ns = dict(navigation=NS(snapshot=lambda: dict(streams={})),
                       waypoint_api=NS(consume=lambda *args: (self.item, False)),
                       waypoint_capabilities=lambda: dict(enabled=False, simulation=False),
                       waypoint_process=None, waypoint_launch=None, waypoint_aborts=queue.Queue(),
                       calibration_jobs=None, north_process=None, north_aborts=queue.Queue(),
                       pids_for=lambda name: [], MISSION_EXECUTABLES=('waypoint_mission',),
                       subprocess=NS(Popen=Mock(), STDOUT=-2), link=Mock(), survey=self.survey,
                       control_bridge=NS(snapshot=lambda: self.control),
                       json=json, os=os, time=time, uuid=uuid, Path=Path)
        exec(compile(tree, str(path), 'exec'), self.ns)
        self.request = self.ns['waypoint_request']

    def test_live_survey_needs_commissioning_and_control_permission(self):
        with self.assertRaisesRegex(ValueError, 'commissioning'):
            self.request(dict(action='survey_start', points=POINTS))
        self.ns['waypoint_capabilities'] = lambda: dict(enabled=True, simulation=False)
        self.control['sa_active'] = True
        with self.assertRaisesRegex(ValueError, 'Control watchdog'):
            self.request(dict(action='survey_start', points=POINTS))
        self.survey.start.assert_not_called()
        self.control['sa_active'] = False
        self.request(dict(action='survey_start', points=POINTS))
        self.survey.start.assert_called_once_with(POINTS, False, False)

    def test_dry_survey_skips_motion_checks_and_preserves_competition_flag(self):
        self.request(dict(action='survey_start', points=POINTS, dry=True))
        self.survey.start.assert_called_once_with(POINTS, True, False)
        self.survey.start.reset_mock()
        self.request(dict(action='survey_start', points=POINTS, dry=True, competition=True))
        self.survey.start.assert_called_once_with(POINTS, True, True)

    def test_survey_refused_while_another_mission_runs(self):
        self.ns['waypoint_process'] = NS(poll=lambda: None)
        with self.assertRaisesRegex(ValueError, 'Another mission'):
            self.request(dict(action='survey_start', points=POINTS, dry=True))
        self.ns['waypoint_process'] = None
        self.survey.running.return_value = True
        with self.assertRaisesRegex(ValueError, 'Another mission'):
            self.request(dict(action='survey_start', points=POINTS, dry=True))

    def test_running_survey_blocks_other_starts_but_not_its_own_legs(self):
        self.survey.running.return_value = True
        self.ns['waypoint_capabilities'] = lambda: dict(enabled=True, simulation=False)
        with self.assertRaisesRegex(ValueError, 'Another mission'):
            self.request(dict(action='start', plan_id='p'))
        self.ns['subprocess'].Popen.reset_mock()
        with tempfile.TemporaryDirectory() as logs, patch.dict(os.environ, GRAEY_WAYPOINT_LOG_DIR=logs):
            self.assertTrue(self.request(dict(action='start', plan_id='p', survey_run_id='survey-1'))['ok'])
        self.ns['subprocess'].Popen.assert_called_once()

    def test_stop_and_abort_reach_the_survey(self):
        self.survey.stop.return_value = dict(ok=True)
        self.request(dict(action='survey_stop', run_id='survey-1'))
        self.request(dict(action='abort', run_id='survey-1'))
        self.assertEqual(self.survey.stop.call_count, 2)
        self.survey.running.return_value = True
        self.survey.launch = dict(run_id='survey-1')
        self.assertTrue(self.ns['mission_stop']()[0])
        self.ns['link'].disarm.assert_not_called()


if __name__ == '__main__':
    unittest.main()
