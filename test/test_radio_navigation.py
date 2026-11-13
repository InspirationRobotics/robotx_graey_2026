import ast
import math
import unittest
from pathlib import Path

from robotx_graey_2026.api.navigation.dive_spin import (
    DiveSpinRun, NOTE_OPERATOR, NOTE_PILOT, dive_target, random_resource_request)
from robotx_graey_2026.api.navigation.radio_route import (
    RadioRouteRun, mission_plan, route_points)


def observation(**changes):
    value=dict(fresh=True,failure='',abort=False,local=(0.,0.,0.),yaw=0.,speed=0.,
               mode=19,armed=False,altitude=None,bridge={})
    value.update(changes)
    return value


class DiveSpinTests(unittest.TestCase):
    def take_vehicle(self,run,o,expected='DESCEND'):
        run.step(.1,o)
        self.assertEqual(run.stage,'GUIDED')
        self.assertEqual(run.step(.2,o)[0][0],'guided')
        o['mode']=4;run.step(.3,o)
        self.assertEqual(run.stage,'ARM')
        self.assertEqual(run.step(.4,o)[0][0],'arm')
        o['armed']=True;run.step(.5,o)
        self.assertEqual(run.stage,expected)

    def test_pool_dive_takes_guided_then_arms_at_current_pose(self):
        run=DiveSpinRun(dict(kind='dive_spin',mode='surface',map='local',
                             dive=dict(depth_m=1.)), 'pool',0.)
        o=observation(local=(2.,3.,.4))
        self.take_vehicle(run,o)
        self.assertEqual(run.hold,(2.,3.,.4))
        actions=run.step(.6,o)
        self.assertEqual(actions[0][0],'guided_orient')
        self.assertAlmostEqual(actions[0][1][2],1.4)

    def test_chris_simple_dive_moves_one_metre_before_descending(self):
        run=DiveSpinRun(dict(kind='dive_spin',operator_kind='dive_here',mode='surface',map='local',
                             dive=dict(depth_m=1.)), 'chris',0.)
        o=observation(local=(2.,3.,.4),yaw=math.pi/2)
        self.take_vehicle(run,o,expected='FORWARD')
        self.assertAlmostEqual(run.forward_target[0],2.)
        self.assertAlmostEqual(run.forward_target[1],4.)
        self.assertAlmostEqual(run.forward_target[2],.4)
        o['local']=run.forward_target
        run.step(.6,o);run.step(1.7,o)
        self.assertEqual(run.stage,'DESCEND')
        self.assertEqual(run.hold,run.forward_target)

    def test_chris_resource_sequence_uses_robotx_two_color_order(self):
        values=iter((2,0))
        request=random_resource_request('run-7',lambda limit:next(values))
        self.assertEqual(request,dict(observation_id='chris-simple-dive:run-7',
            resource_color=3,delivery_circle_color=1,sequence=['BLUE','RED']))

    def test_dvl_target_is_bounded_and_near_floor_surfaces(self):
        run=DiveSpinRun(dict(kind='dive_spin',mode='surface',map='local',
            dive=dict(altitude_m=3.5,max_depth_m=5.)), 'dvl',0.)
        run.hold=(0.,0.,0.)
        self.assertAlmostEqual(run.descent_target((0.,0.,0.),6.),2.5)
        near=DiveSpinRun(dict(kind='dive_spin',mode='surface',map='local',
            dive=dict(altitude_m=3.5,max_depth_m=5.)), 'near',0.)
        near.hold=(0.,0.,0.)
        self.assertIsNone(near.descent_target((0.,0.,0.),2.))

    def test_operator_abort_disarms_and_requests_mission_done_kill(self):
        run=DiveSpinRun(dict(kind='dive_spin',mode='surface',map='local',
                             dive=dict(depth_m=1.)), 'abort',0.)
        o=observation();self.take_vehicle(run,o);o['abort']=True
        actions=run.step(.6,o)
        self.assertEqual(run.stage,'ABORTED')
        self.assertEqual(run.note,NOTE_OPERATOR)
        self.assertTrue(run.kill_requested)
        self.assertEqual(actions[0][0],'disarm')

    def test_pilot_takeover_is_never_countermanded_or_killed(self):
        run=DiveSpinRun(dict(kind='dive_spin',mode='surface',map='local',
                             dive=dict(depth_m=1.)), 'pilot',0.)
        o=observation();self.take_vehicle(run,o);o['mode']=19
        self.assertEqual(run.step(.6,o),[])
        self.assertEqual(run.stage,'ABORTED')
        self.assertEqual(run.note,NOTE_PILOT)
        self.assertFalse(run.kill_requested)

    def test_dive_limits_reject_unbounded_values(self):
        for value in (dict(depth_m=5.1),dict(altitude_m=.4,max_depth_m=1.),
                      dict(altitude_m=3.5,max_depth_m=5.1)):
            with self.assertRaises(ValueError):dive_target(**value)


class RadioRouteTests(unittest.TestCase):
    def test_operator_mission_identity_survives_plan_conversion(self):
        plan=mission_plan(dict(kind='full',operator_kind='dock_pipeline_dock',
            points=[[1.28,103.85],[1.2801,103.8501]],
            dive=dict(altitude_m=3.5,max_depth_m=5.)))
        self.assertEqual((plan['kind'],plan['operator_kind']),
                         ('radio_route','dock_pipeline_dock'))
        dive=mission_plan(dict(kind='dive',operator_kind='pool_dive',
                               dive=dict(depth_m=1.)))
        self.assertEqual(dive['operator_kind'],'pool_dive')

    def test_full_route_returns_over_points_then_to_captured_dock(self):
        points=[[1.28,103.85],[1.2801,103.8501],[1.2802,103.8502]]
        plan=mission_plan(dict(kind='full',points=points,
            dive=dict(altitude_m=3.5,max_depth_m=5.)))
        run=RadioRouteRun(plan,'full',0.)
        run.begin(.1,observation(raw_gps=(1.2799,103.8499)),(0.,0.,0.),0.)
        self.assertEqual(run.legs,route_points(points)+route_points(points)[-2::-1]+
                         [(1.2799,103.8499)])
        self.assertEqual(run.stage,'ORIENT')

    def test_route_bounds_and_operator_kind_are_enforced(self):
        with self.assertRaises(ValueError):route_points([])
        with self.assertRaises(ValueError):route_points([[0.,0.]])
        with self.assertRaises(ValueError):mission_plan(dict(kind='route',operator_kind='other',
                                                              points=[[1.,1.]]))


class MissionDoneRecoveryTests(unittest.TestCase):
    def test_mission_done_uses_existing_selected_source_recovery_procedures(self):
        tree=ast.parse((Path(__file__).parents[1]/'robotx_graey_2026/api/pixhawk/kill_switch.py').read_text())
        values={target.id:ast.literal_eval(node.value) for node in tree.body
                if isinstance(node,ast.Assign) for target in node.targets
                if isinstance(target,ast.Name) and target.id in ('RF_RESETTABLE','TETHER_RESETTABLE')}
        self.assertIn('MISSION_DONE',values['RF_RESETTABLE'])
        self.assertIn('MISSION_DONE',values['TETHER_RESETTABLE'])


if __name__=='__main__':unittest.main()
