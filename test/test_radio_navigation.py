import ast
import math
import unittest
from pathlib import Path

from robotx_graey_2026.api.navigation.dive_spin import (
    DiveSpinRun, NOTE_OPERATOR, NOTE_PILOT, dive_target, fixed_resource_request)
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

    def test_chris_resource_sequence_is_fixed_in_robotx_two_color_order(self):
        request=fixed_resource_request('run-7',['blue','RED'])
        self.assertEqual(request,dict(observation_id='chris-simple-dive:run-7',
            resource_color=3,delivery_circle_color=1,sequence=['BLUE','RED']))
        self.assertEqual(fixed_resource_request('r',['ANY','GREEN'])['resource_color'],4)
        for bad in (['RED','ANY'],['PINK','RED'],['RED'],'RED,BLUE',None):
            with self.assertRaises(ValueError):fixed_resource_request('r',bad)

    def test_chris_dive_reports_only_the_colours_in_its_plan(self):
        plan=dict(kind='dive_spin',operator_kind='dive_here',mode='surface',map='local',
                  dive=dict(depth_m=4.))
        self.assertIsNone(DiveSpinRun(dict(plan,colors=[]),'none',0.).resource_request)
        self.assertIsNotNone(DiveSpinRun(plan,'legacy-gui',0.).resource_request)
        run=DiveSpinRun(dict(plan,colors=['GREEN','BLUE']),'set',0.)
        self.assertEqual(run.resource_request['sequence'],['GREEN','BLUE'])

    def test_no_spin_plan_goes_straight_back_up_at_depth(self):
        run=DiveSpinRun(dict(kind='dive_spin',mode='surface',map='local',spin=False,
                             dive=dict(depth_m=1.)), 'nospin',0.)
        o=observation(local=(0.,0.,0.))
        self.take_vehicle(run,o)
        o['local']=(0.,0.,1.)
        run.step(1.,o);run.step(2.2,o)
        self.assertEqual(run.stage,'SURFACE')
        self.assertEqual(run.spin_step,0)
        self.assertEqual(run.target,run.hold)

    SE=dict(kind='dive_spin',operator_kind='dive_here',mode='surface',map='local',competition=True,
            spin=False,hold_s=10.,report_hold_s=60.,dive=dict(depth_m=1.))

    def dive_to_surface(self,plan=None):
        """The SE plan to the moment it holds at depth; returns (run, o). Time is self.t."""
        run=DiveSpinRun(dict(plan or self.SE),'se',0.)
        o=observation(local=(0.,0.,0.))
        self.take_vehicle(run,o,expected='FORWARD')
        self.t=.5
        o['local']=run.forward_target;self.go(run,o,5,'DESCEND')
        o['local']=(run.hold[0],run.hold[1],1.);self.go(run,o,5,'HOLD')
        return run,o

    def go(self,run,o,seconds,until):
        end=self.t+seconds
        while self.t<end and run.stage!=until:
            self.t+=.1;run.step(self.t,o)

    def up(self,run,o,until='REPORT'):
        self.go(run,o,12,'SURFACE');o['local']=run.hold;self.go(run,o,2,until)

    def test_se_dive_holds_at_depth_then_surfaces(self):
        run,o=self.dive_to_surface()
        self.assertEqual((run.stage,run.phase),('HOLD','UNDERWATER'))
        held=self.t
        self.go(run,o,12,'SURFACE')
        self.assertEqual(run.stage,'SURFACE')
        self.assertAlmostEqual(self.t-held,10.,delta=.15)

    def test_se_dive_reports_armed_at_the_surface_then_disarms(self):
        run,o=self.dive_to_surface()
        self.go(run,o,12,'SURFACE');o['local']=run.hold
        actions=[];end=self.t+2
        while self.t<end and run.stage!='REPORT':
            self.t+=.1;actions=run.step(self.t,o)
        self.assertEqual((run.stage,actions,run.phase),('REPORT',[('manual',None)],'SURFACE_NAV'))
        o['mode']=19
        self.assertEqual(run.step(self.t+.1,o),[('neutral',None)])
        o['report_done']=True
        self.assertEqual(run.step(self.t+.2,o),[('disarm',None)])
        self.assertEqual(run.stage,'DISARM')
        o['armed']=False;run.step(self.t+.3,o)
        self.assertEqual(run.stage,'COMPLETE')
        self.assertTrue(run.kill_requested)

    def test_report_hold_gives_up_and_disarms(self):
        run,o=self.dive_to_surface();self.up(run,o)
        o['mode']=19;self.go(run,o,61,'DISARM')
        self.assertEqual(run.stage,'DISARM')
        self.assertIn('not confirmed',run.reason)

    def test_report_stage_lets_a_pilot_take_over(self):
        for change in (dict(armed=False),dict(mode=0)):
            run,o=self.dive_to_surface();self.up(run,o)
            o.update(change)
            self.assertEqual(run.step(self.t+.1,o),[])
            self.assertEqual(run.stage,'ABORTED')
            self.assertFalse(run.kill_requested)

    def test_report_stage_waits_for_manual_then_calls_guided_a_takeover(self):
        run,o=self.dive_to_surface();self.up(run,o)
        self.assertEqual(o['mode'],4)                      # the Cube has not switched yet
        self.assertEqual(run.step(self.t+1.,o),[('manual',None)])
        run.step(self.t+4.,o)
        self.assertEqual(run.stage,'ABORTED')

    def test_without_report_hold_the_dive_disarms_on_surfacing(self):
        run,o=self.dive_to_surface(dict(self.SE,report_hold_s=0.))
        self.up(run,o,'DISARM')
        self.assertEqual(run.stage,'DISARM')

    def test_hold_and_report_times_are_bounded(self):
        for bad in (dict(hold_s=-1),dict(hold_s=500),dict(report_hold_s=True),dict(report_hold_s=1e9)):
            with self.assertRaises(ValueError):DiveSpinRun(dict(self.SE,**bad),'x',0.)

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
