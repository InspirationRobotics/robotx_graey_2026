"""Exercise the production mission timer before it can enter route execution."""
import ast
import json
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from robotx_graey_2026.api.comms.run_gate import RunGate


class ExecutionReached(Exception):pass


class TestCompetitionMissionGate(unittest.TestCase):
    def make_node(self,competition):
        source=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/waypoint_mission.py'
        tree=ast.parse(source.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='WaypointMission')
        tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick')
        namespace=dict(time=SimpleNamespace(monotonic=lambda:0.),json=json,MODE_GUIDED=4,MODE_POSHOLD=16,
            String=lambda **kwargs:SimpleNamespace(**kwargs),Bool=lambda **kwargs:SimpleNamespace(**kwargs),
            GuidedSurfaceRun=type('GuidedSurfaceRun',(),{}),CompetitionRouteRun=type('CompetitionRouteRun',(),{}))
        exec(compile(ast.Module(body=[tick],type_ignores=[]),str(source),'exec'),namespace)
        node=SimpleNamespace(sequence=0,run_gate=RunGate(competition),competition_hold=None,
            intent_pub=Mock(),pub=Mock(),auto_pub=Mock(),send_guided_waypoint=Mock(),
            link=Mock(),surface_measurement_status={})
        node.logic=SimpleNamespace(stage='ARM',terminal={'ABORTED','COMPLETE'},phase='SURFACE_NAV',
            run_id='local',plan={'mode':'surface'},snapshot=lambda:dict(stage='ARM'),
            step=Mock(side_effect=ExecutionReached()),abort=Mock())
        node.observation=lambda now:dict(fresh=True,armed=True,mode=4,failure='',
                                        navigation_hold_safe=True,local=(1.,2.,.3))
        return node,lambda:namespace['tick'](node)

    def test_competition_holds_without_entering_route(self):
        node,tick=self.make_node(True)
        tick()
        node.logic.step.assert_not_called()
        node.send_guided_waypoint.assert_not_called()
        node.link.set_mode.assert_called_once_with(16)
        status=json.loads(node.pub.publish.call_args.args[0].data)
        self.assertEqual((status['stage'],status['control_mode'],status['competition_ready']),
                         ('WAIT_RUN_START','POSHOLD',False))

    def test_poshold_is_reported_ready_and_waits(self):
        node,tick=self.make_node(True)
        node.observation=lambda now:dict(fresh=True,armed=True,mode=16,failure='',
                                        navigation_hold_safe=True,local=(1.,2.,.3))
        tick()
        status=json.loads(node.pub.publish.call_args.args[0].data)
        self.assertTrue(status['competition_ready'])
        node.auto_pub.publish.assert_called_once()

    def test_verified_start_unlocks_production_execution(self):
        node,tick=self.make_node(True)
        node.observation=lambda now:dict(fresh=True,armed=True,mode=16,failure='',
                                        navigation_hold_safe=True,local=(1.,2.,.3))
        node.run_gate.update(dict(run_confirmed=True,remote_state='RUNNING',run_id=70000),0.)
        tick()
        self.assertEqual(json.loads(node.pub.publish.call_args.args[0].data)['stage'],'STARTING')
        node.link.set_mode.assert_called_with(4)
        node.observation=lambda now:dict(fresh=True,armed=True,mode=4,failure='',
                                        navigation_hold_safe=True,local=(1.,2.,.3))
        with self.assertRaises(ExecutionReached):tick()
        self.assertEqual(node.run_gate.run_id,70000)

    def test_local_mission_executes_without_ocs(self):
        node,tick=self.make_node(False)
        with self.assertRaises(ExecutionReached):tick()

    def test_stale_run_does_not_unlock_route(self):
        node,tick=self.make_node(True)
        node.observation=lambda now:dict(fresh=True,armed=True,mode=16,failure='',
                                        navigation_hold_safe=True,local=(1.,2.,.3))
        node.run_gate.update(dict(run_confirmed=True,remote_state='RUNNING',run_id=7),-3.)
        tick();node.logic.step.assert_not_called()

    def test_chris_report_is_completion_only_and_repeated_idempotently(self):
        source=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/waypoint_mission.py'
        tree=ast.parse(source.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='WaypointMission')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef)
                    and n.name=='publish_resource_report')
        namespace=dict(json=json,String=lambda **kwargs:SimpleNamespace(**kwargs))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        node=SimpleNamespace(resource_last=float('-inf'),resource_pub=Mock(),logic=SimpleNamespace(
            stage='SURFACE',resource_request=dict(observation_id='chris-simple-dive:run',
                resource_color=3,delivery_circle_color=1,sequence=['BLUE','RED'])))
        gate=SimpleNamespace(competition=True,run_id=42)
        publish=lambda now:namespace['publish_resource_report'](node,now,gate)
        self.assertIsNone(publish(1.));node.resource_pub.publish.assert_not_called()
        node.logic.stage='COMPLETE';status=publish(2.)
        self.assertEqual(status['sequence'],['BLUE','RED'])
        payload=json.loads(node.resource_pub.publish.call_args.args[0].data)
        self.assertEqual(payload,dict(observation_id='chris-simple-dive:run',
            resource_color=3,delivery_circle_color=1))
        publish(2.2);self.assertEqual(node.resource_pub.publish.call_count,1)
        publish(2.5);self.assertEqual(node.resource_pub.publish.call_count,2)

    def test_chris_practice_result_is_not_sent_to_robocommand(self):
        source=Path(__file__).parents[1]/'robotx_graey_2026/api/navigation/waypoint_mission.py'
        tree=ast.parse(source.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='WaypointMission')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef)
                    and n.name=='publish_resource_report')
        namespace=dict(json=json,String=lambda **kwargs:SimpleNamespace(**kwargs))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        node=SimpleNamespace(resource_last=float('-inf'),resource_pub=Mock(),logic=SimpleNamespace(
            stage='COMPLETE',resource_request=dict(observation_id='practice',resource_color=1,
                delivery_circle_color=2,sequence=['RED','GREEN'])))
        report=namespace['publish_resource_report'](node,1.,SimpleNamespace(competition=False,run_id=None))
        node.resource_pub.publish.assert_not_called()
        self.assertIn('no current RoboCommand run',report['report_state'])
