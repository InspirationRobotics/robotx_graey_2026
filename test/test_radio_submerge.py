"""Radio submerge test: dive legs, timeout, takeover, the CSV log and the mission runner's listening."""
import ast
import json
import math
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from robotx_graey_2026.api.gui.mission_store import MissionStore, validate_mission
from robotx_graey_2026.api.navigation import radio_submerge as rs
from robotx_graey_2026.api.navigation.radio_submerge import (RadioLog, RadioSubmergeRun, LEGS,
    radio_log_path, read_radio_rows, write_radio_header, append_radio_rows, tap_drain)
from robotx_graey_2026.api.navigation.simple_sequence import SimpleSequenceRun
from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun

SURFACE_HPA = 1013.25
FOOT_HPA = rs.DIVE_M*1000*rs.GRAVITY/100       # pressure added by 1 ft of fresh water


def obs(**changes):
    o = dict(fresh=True, local=(1., 2., .1), yaw=.5, speed=0., mode=4, armed=True, pressure_hpa=SURFACE_HPA)
    o.update(changes)
    return o


def armed_run():
    m = RadioSubmergeRun({}, 'test', 0.)
    o = obs()
    m.step(.1, o)        # GUIDED already -> ARM
    m.step(.2, o)        # armed -> capture the start, first leg
    return m, o


class TestDiveLegs(unittest.TestCase):
    def test_three_dives_follow_the_ramp_then_disarm(self):
        m, o = armed_run()
        self.assertEqual(m.stage, 'DOWN_1')
        self.assertEqual((m.start, m.surface_down, m.start_pressure_hpa), ((1., 2., .1), .1, SURFACE_HPA))
        stages, deepest, t = ['DOWN_1'], -math.inf, .2
        while m.stage in LEGS and t < 150:
            t = round(t+.1, 1)
            actions = m.step(t, o)
            if m.stage != stages[-1]:
                stages.append(m.stage)
            if actions and actions[0][0] == 'guided_orient':
                x, y, z, heading = actions[0][1]
                self.assertEqual((x, y, heading), (1., 2., .5))     # holds position and heading
                deepest = max(deepest, z)
                o['local'] = (1., 2., z)                            # the Cube follows exactly
        self.assertEqual(stages, [*LEGS, 'DISARM'])
        self.assertAlmostEqual(deepest, .1+rs.DIVE_M)
        self.assertLess(t, rs.TIMEOUT_S)
        self.assertEqual(m.step(t+1, o), [('disarm', None)])
        o['armed'] = False
        self.assertEqual(m.step(t+2, o), [('manual', None)])
        self.assertEqual(m.stage, 'COMPLETE')

    def test_target_moves_one_centimetre_per_half_second(self):
        m, o = armed_run()
        self.assertAlmostEqual(m.step(5.2, o)[0][1][2], .1+.10)    # 5 s at 2 cm/s
        self.assertAlmostEqual(m.step(5.7, o)[0][1][2], .1+.11)    # one boat line (0.5 s) later

    def test_leg_waits_for_the_ramp_even_if_already_deep(self):
        m, o = armed_run()
        o['local'] = (1., 2., .1+rs.DIVE_M)
        for t in range(1, 15):
            m.step(.2+t, o)
        self.assertEqual(m.stage, 'DOWN_1')
        for t in range(16, 18):
            m.step(.2+t, o)
        self.assertEqual(m.stage, 'UP_1')

    def test_no_arming_without_a_pressure_reading(self):
        m = RadioSubmergeRun({}, 'test', 0.)
        o = obs(mode=0, armed=False, pressure_hpa=None)
        self.assertEqual(m.step(.1, o), [])
        self.assertEqual(m.stage, 'GUIDED')
        self.assertIn('pressure', m.reason)
        o['pressure_hpa'] = SURFACE_HPA
        self.assertEqual(m.step(.2, o), [('guided', None)])


class TestStops(unittest.TestCase):
    def test_timeout_disarms_then_manual_even_without_telemetry(self):
        m, o = armed_run()
        o['fresh'] = False                       # stuck and blind: the timeout must still fire
        m.step(.2+rs.TIMEOUT_S-.1, o)
        self.assertEqual(m.stage, 'DOWN_1')
        self.assertEqual(m.step(.2+rs.TIMEOUT_S, o), [('disarm', None)])
        self.assertEqual(m.stage, 'TIMEOUT')
        self.assertEqual(m.step(.2+rs.TIMEOUT_S+.2, o), [])               # 0.5 s between disarms
        self.assertEqual(m.step(.2+rs.TIMEOUT_S+.6, o), [('disarm', None)])
        o.update(armed=False)
        self.assertEqual(m.step(.2+rs.TIMEOUT_S+.7, o), [('manual', None)])
        o.update(mode=rs.MODE_MANUAL)
        self.assertEqual(m.step(.2+rs.TIMEOUT_S+.8, o), [])
        self.assertEqual(m.stage, 'ABORTED')
        self.assertTrue(m.timed_out)
        self.assertFalse(m.takeover_requested)
        self.assertTrue(m.snapshot()['timed_out'])

    def test_pilot_takeover_during_a_dive(self):
        m, o = armed_run()
        o['mode'] = rs.MODE_MANUAL
        self.assertEqual(m.step(1., o), [])
        self.assertEqual(m.stage, 'ABORTED')
        self.assertTrue(m.pilot_takeover)

    def test_operator_stop_asks_for_manual(self):
        m, o = armed_run()
        o['abort'] = True
        self.assertEqual(m.step(1., o), [('manual', None)])
        self.assertEqual(m.stage, 'ABORTED')

    def test_disarm_while_holding_for_pressure_is_not_rearmed(self):
        m = RadioSubmergeRun({}, 'test', 0.)
        o = obs()
        m.step(.1, o)
        o['pressure_hpa'] = None
        self.assertEqual(m.step(.2, o), [('guided_orient', (1., 2., .1, .5))])
        o['armed'] = False
        self.assertEqual(m.step(.3, o), [])
        self.assertEqual(m.stage, 'ABORTED')


class TestRadioLog(unittest.TestCase):
    def setUp(self):
        self.run = SimpleNamespace(stage='DOWN_1', start_pressure_hpa=SURFACE_HPA, surface_down=.1)
        self.log = RadioLog(0.)

    def test_boat_line_logged_with_raw_and_relative_depth(self):
        self.log.pressure_sample(SURFACE_HPA+FOOT_HPA, 1.)
        row = self.log.boat_line(b'the boat is sending this msg 7', 1.1, self.run, .1+rs.DIVE_M)
        self.assertEqual((row['kind'], row['msg_number'], row['stage']), ('heard', 7, 'DOWN_1'))
        self.assertAlmostEqual(row['relative_depth_m'], rs.DIVE_M, places=3)
        self.assertAlmostEqual(row['raw_depth_m'], (SURFACE_HPA+FOOT_HPA)*100/(1000*rs.GRAVITY), places=3)
        self.assertAlmostEqual(row['cube_relative_m'], rs.DIVE_M, places=3)
        self.assertEqual(self.log.summary(2.)['deepest_heard_relative_m'], row['relative_depth_m'])

    def test_other_text_and_bad_numbers_are_ignored(self):
        for payload in (b'test 1 from crusader', b'the boat is sending this msg x', b''):
            self.assertIsNone(self.log.boat_line(payload, 1., self.run, None))
        self.assertEqual(self.log.received, 0)
        self.assertEqual(self.log.rows(), [])

    def test_stale_pressure_is_left_blank(self):
        self.log.pressure_sample(SURFACE_HPA, 1.)
        row = self.log.boat_line(b'the boat is sending this msg 1', 1.6, self.run, None)
        self.assertIsNone(row['raw_depth_m'])
        self.assertIsNone(row['relative_depth_m'])
        self.assertIsNone(self.log.summary(2.)['deepest_heard_relative_m'])

    def test_depth_rows_every_half_second(self):
        for t in (0., .2, .4, .5, .9, 1.):
            self.log.tick(t, self.run, None)
        self.assertEqual([r['t_s'] for r in self.log.rows()], [0., .5, 1.])
        self.assertEqual(self.log.rows(), [])


class TestLogFile(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.path = radio_log_path(self.folder.name, 'a'*32)

    def test_run_id_cannot_name_another_file(self):
        for bad in ('../x', 'A'*32, 'test', 'a'*31, None):
            with self.assertRaises(ValueError):
                radio_log_path(self.folder.name, bad)

    def test_write_append_and_read_from_a_row_count(self):
        write_radio_header(self.path)
        log = RadioLog(0.)
        run = SimpleNamespace(stage='UP_1', start_pressure_hpa=None, surface_down=None)
        for t in (0., .5, 1.):
            log.tick(t, run, None)
        append_radio_rows(self.path, log.rows())
        rows, total = read_radio_rows(self.path)
        self.assertEqual((len(rows), total), (3, 3))
        self.assertEqual(rows[0]['raw_depth_m'], '')            # unknown stays blank
        rows, total = read_radio_rows(self.path, after=2)
        self.assertEqual(([r['t_s'] for r in rows], total), (['1.0'], 3))
        with self.path.open('a', encoding='utf-8') as f:        # a write caught halfway
            f.write('2026-11-10T00:00:00,1.5,depth')
        self.assertEqual(read_radio_rows(self.path)[1], 3)
        for bad in (-1, True, '1'):
            with self.assertRaises(ValueError):
                read_radio_rows(self.path, after=bad)

    def test_saved_mission_round_trip(self):
        self.assertEqual(validate_mission(dict(kind='radio_submerge')), dict(kind='radio_submerge', competition=False))
        store = MissionStore(self.folder.name)
        store.save('radio', dict(kind='radio_submerge'))
        self.assertEqual(store.get('radio')['kind'], 'radio_submerge')


class Msg:
    def __init__(self, system, component, **fields):
        self.system, self.component = system, component
        self.__dict__.update(fields)

    def get_srcSystem(self):
        return self.system

    def get_srcComponent(self):
        return self.component


class TestRadioRunner(unittest.TestCase):
    """radio_submerge_mission.RadioSubmergeMission on a stand-in for the unchanged WaypointMission:
    the real base reads plan_json while it starts, builds its logic from the plan kind (it knows
    simple_sequence, not radio_submerge), drains frames to its own handler and publishes the
    logic snapshot."""

    def make(self, plan, frames):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        params = dict(plan_json=json.dumps(plan), run_id='e'*32,
                      result_file=str(Path(folder.name)/('waypoint-'+'e'*32+'.json')))
        self.logger = Mock()

        class WaypointMission:
            def __init__(self):
                cfg = {k: self.get_parameter(k).value for k in ('plan_json', 'run_id', 'result_file')}
                plan = json.loads(cfg['plan_json'])
                self.logic = (SimpleSequenceRun if plan.get('kind') == 'simple_sequence'
                              else GuidedSurfaceRun)(plan, cfg['run_id'], 0.)
                self.result_file = cfg['result_file']
                self.link = SimpleNamespace(drain=lambda handler: [handler(k, m) for k, m in frames])
                self.seen, self.status = [], None

            def get_parameter(self, name):
                return SimpleNamespace(value=params[name])

            def get_logger(this):
                return self.logger

            def observation(self, now):
                return dict(fresh=True)

            def tick(self):
                self.link.drain(lambda kind, m: self.seen.append(kind))    # the base's own handler
                self.status = self.logic.snapshot()

        source = Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/radio_submerge_mission.py'
        cls = next(n for n in ast.parse(source.read_text(encoding='utf-8')).body
                   if isinstance(n, ast.ClassDef) and n.name == 'RadioSubmergeMission')
        namespace = dict(vars(rs), WaypointMission=WaypointMission, SimpleSequenceRun=SimpleSequenceRun,
                         SimpleNamespace=SimpleNamespace, json=json, math=math, time=time, Path=Path)
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
        return namespace['RadioSubmergeMission']()

    def frames(self, boat_payload=None):
        boat = list(b'the boat is sending this msg 5')
        return [('SCALED_PRESSURE3', Msg(1, 1, press_abs=SURFACE_HPA+FOOT_HPA/2)),
                ('LOCAL_POSITION_NED', Msg(1, 1, z=.25, time_boot_ms=1000)),
                ('TUNNEL', Msg(42, 191, payload_type=33022, payload=boat+[0]*(128-len(boat)), payload_length=len(boat))
                 if boat_payload is None else Msg(42, 191, payload_type=33022, **boat_payload)),
                ('TUNNEL', Msg(200, 191, payload_type=33022, payload=boat, payload_length=len(boat))),   # Ekko
                ('TUNNEL', Msg(42, 191, payload_type=32768, payload=[1, 2], payload_length=2))]          # boat map

    def test_builds_the_radio_logic_and_reads_the_real_plan_afterwards(self):
        node = self.make(dict(kind='radio_submerge', mode='surface'), [])
        self.assertIsInstance(node.logic, RadioSubmergeRun)
        self.assertEqual(node.logic.run_id, 'e'*32)
        self.assertEqual(json.loads(node.get_parameter('plan_json').value)['kind'], 'radio_submerge')
        self.assertTrue(node.radio_csv.exists())

    def test_refuses_any_other_mission(self):
        with self.assertRaises(ValueError):
            self.make(dict(kind='simple_sequence', mode='surface'), [])

    def test_boat_lines_and_pressure_reach_the_csv_and_status(self):
        node = self.make(dict(kind='radio_submerge', mode='surface'), self.frames())
        node.logic.stage, node.logic.surface_down, node.logic.start_pressure_hpa = 'DOWN_2', .1, SURFACE_HPA
        node.tick()
        self.assertEqual(len(node.seen), 5)                       # the base still saw every frame
        self.assertEqual(node.observation(time.monotonic())['pressure_hpa'], SURFACE_HPA+FOOT_HPA/2)
        node.tick()                                                # rows are written at the next tick
        rows, total = read_radio_rows(node.radio_csv)
        heard = [r for r in rows if r['kind'] == 'heard']
        self.assertEqual(len(heard), 1)                            # the 2nd tick's line is written next tick
        self.assertEqual((heard[0]['msg_number'], heard[0]['stage'], heard[0]['cube_z_m']), ('5', 'DOWN_2', '0.25'))
        self.assertAlmostEqual(float(heard[0]['relative_depth_m']), rs.DIVE_M/2, places=3)
        self.assertEqual(node.status['radio']['received'], 1)     # status is one tick behind the file
        self.assertIsNone(node.status['radio']['log_error'])

    def test_a_logging_failure_never_hides_a_frame_from_the_mission(self):
        node = self.make(dict(kind='radio_submerge', mode='surface'), self.frames(boat_payload=dict(payload_length=4)))
        node.tick()
        node.tick()
        self.assertEqual(len(node.seen), 10)
        self.assertIn('payload', node.status['radio']['log_error'])
        self.logger.error.assert_called_once()                     # logged once, not every frame


class TestTapDrain(unittest.TestCase):
    def test_tap_sees_each_frame_before_the_handler_and_failures_are_reported(self):
        order, errors = [], []
        link = SimpleNamespace(drain=lambda handler: [handler(k, None) for k in ('A', 'B')])
        def tap(kind, m):
            order.append('tap '+kind)
            if kind == 'B':
                raise KeyError('bad frame')
        tap_drain(link, tap, errors.append)
        link.drain(lambda kind, m: order.append('mission '+kind))
        self.assertEqual(order, ['tap A', 'mission A', 'tap B', 'mission B'])
        self.assertEqual(len(errors), 1)


if __name__ == '__main__':
    unittest.main()
