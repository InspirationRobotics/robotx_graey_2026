import math
import unittest
from robotx_graey_2026.api.navigation.geographic_frame import GeographicFrame, rotation
from robotx_graey_2026.api.navigation.relative_estimator import RelativeEstimator
from robotx_graey_2026.api.navigation.bridge_estimator import yaw_rotate
from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun
from robotx_graey_2026.api.navigation.surface_control import SurfaceLeg, surface_velocity
from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo


def simulate(offset=73., gps_jump=False, dropout=False, abort=False):
    e=RelativeEstimator();e.set_origin((33.,-117.,200.))
    p=[0.,0.];v=[0.,0.];heading=0.;cmd=(0.,0.,0.);desired=0.;mode=19;armed=False
    target=(6.,2.);lat,lon=ne_to_geo(*target,(33.,-117.))
    run=None;leg=SurfaceLeg();begun=False;stages=[];previous=None
    for i in range(6000):
        t=100+i*.05
        rate=max(-.15,min(.15,((desired-heading+math.pi)%(2*math.pi)-math.pi)*2))
        heading+=rate*.05
        for axis in (0,1):
            v[axis]+=(cmd[axis]-v[axis])*.15;p[axis]+=v[axis]*.05
        body=rotation(v,-math.degrees(heading))
        e.gps.clock.offset=90.;e.gps.clock.uncertainty=.005;e.gps.clock.updated=t
        e.imu(t,yaw_rotate((1.,0.,0.,0.),math.degrees(heading)),(0.,0.,rate))
        e.pressure_depth(t,0.);e.dvl(t,(*body,0.),not(dropout and i>280))
        if i%4==0:
            ne=rotation(p,offset)
            if gps_jump and i>280:ne=(ne[0]+15.,ne[1])
            glat,glon=ne_to_geo(*ne,(33.,-117.))
            e.observe_gps(t,dict(cube_fix_time_us=int((t-90-.05)*1e6),received=t,
                lat=glat,lon=glon,alt=200.,accuracy=1.2,fix=3,
                speed_mps=math.hypot(*v),speed_accuracy_mps=.4,course_deg=0.))
        b=e.status(t)
        b['heading_vn']=dict(rate=math.degrees(rate));b['configuration_verified']=True
        if i==140:
            run=GuidedSurfaceRun(dict(lat=lat,lon=lon,map='geographic',mode='surface',alignment_corridor_confirmed=True),'run',t)
        if run is None:continue
        if run.stage=='ALIGN_MOVE' and not begun:e.north_alignment.begin();begun=True
        if run.stage=='ALIGN_ACCEPT' and e.north_alignment.active and not e.north_alignment.approved:e.north_alignment.stop(True)
        s=dict(run_id='run',reference_current=True,reference=dict(reference_id='ref'),navigation_ready=b['healthy'],
            surfaced=True,fusion_policy='continuous',state='Ready')
        current=e.gps.status(t)['qualified'] and e.last_correction is not None and t-e.last_correction<.35
        o=dict(fresh=b['healthy'],failure='',abort=abort and i>280,supervisor=s,bridge=b,
            armed=armed,mode=mode,local=(*e.position[:2],0.),velocity_ned=(*v,0.),depth=0.,yaw=heading,
            speed=math.hypot(*v),surface_gps_current=current,navigation_hold_safe=b['healthy'],
            surface_measurement=dict(ready=True),dvl_continuity=b['dvl_continuity'],
            **{'global':(glat,glon)})
        if run.stage!=previous:stages.append(run.stage);previous=run.stage
        for action,payload in run.step(t,o):
            if action=='guided':mode=4
            elif action=='arm':armed=True
            elif action=='disarm':armed=False;cmd=(0.,0.,0.)
            elif action=='manual':mode=19;cmd=(0.,0.,0.)
            elif action=='align_velocity':cmd=(*payload[:2],0.);desired=payload[2]
            elif action=='stop':cmd=(0.,0.,0.);desired=payload[0]
            elif action=='target':
                if run.stage=='TRANSIT':cmd,desired=leg.step(t,run.start,run.target,o['local'],o['velocity_ned'],0.,heading)
                else:cmd=surface_velocity(payload,o['local'],o['velocity_ned'],0.);desired=payload[3]
        if run.stage in run.terminal:return run,e,p,stages
    return run,e,p,stages


class GeographicTests(unittest.TestCase):
    def test_preflight_does_not_enter_guided_on_one_good_gps_sample(self):
        run=GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic',alignment_corridor_confirmed=True),'r',0.)
        o=dict(fresh=True,supervisor=dict(run_id='r',navigation_ready=True,reference_current=True,reference=dict(reference_id='ref')),
               local=(0.,0.,0.),depth=0.,yaw=0.,mode=19,armed=True,surface_gps_current=True,
               surface_measurement=dict(ready=False,reason='GPS accuracy 11.282 m'),**{'global':(33.,-117.)})
        for i in range(50):
            self.assertEqual(run.step((i+1)*.1,o),[])
            self.assertEqual(run.stage,'WAIT_SURFACE')
        o['surface_measurement']=dict(ready=True)
        run.step(5.1,o)
        self.assertEqual(run.stage,'GUIDED')
        self.assertEqual(run.step(5.2,o),[('guided',None)])
        o['mode']=4
        self.assertEqual(run.step(5.3,o),[('stop',(0.,))])
        actions=run.step(5.5,o)
        self.assertEqual(run.stage,'CHECK_SURFACE')
        self.assertEqual(actions[0][0],'target')

    def test_relative_imu_gap_requires_revalidation_before_north_exists(self):
        e=RelativeEstimator();e.imu(1.,(1,0,0,0),(0,0,0));e.initialized=True
        e.imu(2.,(1,0,0,0),(0,0,0))
        self.assertTrue(e.heading_continuity_lost)
        self.assertFalse(e.status(2.)['local_navigation_ready'])

    def test_alignment_finishes_at_target_without_controller_exception(self):
        leg=SurfaceLeg()
        velocity,heading=leg.step(1.,(0,0),(0,0),(0,0),(.1,0),0.,1.)
        self.assertLess(velocity[0],0.)
        self.assertEqual(heading,1.)

    def test_round_trip_and_cardinal_axes_with_height_ignored(self):
        for angle in (0,90,-73,179):
            f=GeographicFrame();f.align(33.,-117.,(4.,2.,900.),angle)
            for xy in ((0.,0.),(3.,-8.),(20.,20.)):
                restored=f.to_local(f.to_map(xy))
                self.assertLess(math.dist(xy,restored),1e-9)
            f.correct_angle(1.,(4.,2.))
            self.assertLess(math.hypot(*f.to_map((4.,2.))),1e-9)
        f=GeographicFrame();f.align(33.,-117.,(0,0),90)
        self.assertAlmostEqual(f.to_map((1,0))[1],1.)
        self.assertAlmostEqual(f.to_map((0,1))[0],-1.)

    def test_full_guided_alignment_transit_stop_at_reported_1_2m_accuracy(self):
        for offset in (0.,73.,-120.):
            run,e,p,stages=simulate(offset)
            self.assertEqual(run.stage,'COMPLETE',(run.reason,stages,e.north_alignment.status()))
            self.assertIn('ALIGN_MOVE',stages);self.assertIn('ALIGN_ACCEPT',stages)
            self.assertLess(math.dist(rotation(p,offset),(6.,2.)),.55)
            self.assertLess(abs((e.map.angle-offset+180)%360-180),2.)
            self.assertEqual(e.north_offset,0.);self.assertEqual(e.heading_bias,0.)
            self.assertLess(math.dist(e.position[:2],p),.05)

    def test_jump_dropout_and_abort_stop_without_guessing_north(self):
        for args in (dict(gps_jump=True),dict(dropout=True),dict(abort=True)):
            run,e,p,stages=simulate(**args)
            self.assertEqual(run.stage,'ABORTED',(args,run.reason,stages))
            self.assertIsNone(e.map.angle)

    def test_no_corridor_no_mission(self):
        with self.assertRaisesRegex(ValueError,'corridor'):
            GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic'),'r',0.)
