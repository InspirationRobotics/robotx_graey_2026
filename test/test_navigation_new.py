import math
import unittest
import numpy as np
from robotx_graey_2026.api.navigation.horizontal_ekf import HorizontalEKF
from robotx_graey_2026.api.navigation.continuous_estimator import ContinuousEstimator
from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun


class NavigationNewTests(unittest.TestCase):
    def test_each_single_and_paired_sensor_outage_keeps_estimate(self):
        for survivors in [('vn',),('dvl',),('gps',),('vn','dvl'),('gps','vn'),('gps','dvl')]:
            e=HorizontalEKF();e.vn(0,0,0)
            for i in range(1,201):
                t=i*.05
                if 'vn' in survivors:e.vn(t,0,0)
                if 'dvl' in survivors:e.dvl(t,.1,0)
                if 'gps' in survivors:e.gps(t,(t*.1,0),1.)
                e.predict(t)
            self.assertTrue(e.health(10)['usable'],survivors)
            self.assertFalse(e.health(10)['all_sensors_lost'])
            self.assertTrue(np.linalg.eigvalsh(e.P).min()>0)
        self.assertTrue(e.health(13)['all_sensors_lost'])
        e.vn(13,0,0)
        self.assertTrue(e.health(13)['usable'])

    def test_dvl_return_does_not_integrate_missing_time_twice(self):
        e=ContinuousEstimator()
        for i in range(401):
            t=i*.05;e.imu(t,(1,0,0,0),(0,0,0));e.pressure_depth(t,0)
            e.dvl(t,(.1,0,0),not 2<t<12)
            self.assertFalse(e.status(t)['fault'])
        self.assertAlmostEqual(e.position[0],2.,delta=.1)
        self.assertTrue(e.status(20)['estimator_health']['sensors']['dvl'])

    def test_gps_updates_without_dvl_or_vn_and_rejects_jump(self):
        e=ContinuousEstimator();e.imu(0,(1,0,0,0),(0,0,0))
        e.map.align(33.,-117.,(0,0),0)
        e.gps.clock.offset=0.;e.gps.clock.uncertainty=0.
        for i in range(1,61):
            t=i*.1;e.gps.clock.updated=t;e.pressure_depth(t,0.)
            fix=dict(cube_fix_time_us=int(t*1e6),lat=33.+t*.1/111320,lon=-117.,fix=3,accuracy=1.)
            e.observe_gps(t,fix)
        self.assertTrue(e.status(6)['estimator_health']['usable'])
        self.assertTrue(e.status(6)['estimator_health']['sensors']['gps'])
        before=e.position[:]
        e.observe_gps(6.1,dict(fix,cube_fix_time_us=6100000,lat=34.))
        self.assertLess(math.dist(e.position,before),.1)
        self.assertEqual(e.event['decision'],'rejected')

    def test_body_axes_rotate_at_cardinal_headings(self):
        for angle in (0,math.pi/2,math.pi,-math.pi/2):
            e=HorizontalEKF()
            for i in range(101):
                t=i*.05;e.vn(t,angle,0);e.dvl(t,.2,0)
            np.testing.assert_allclose(e.x[2:4],[.2*math.cos(angle),.2*math.sin(angle)],atol=.005)

    def test_sensor_dropouts_do_not_abort_but_total_loss_does(self):
        r=GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic',alignment_corridor_confirmed=True),'r',0.)
        r.stage='TRANSIT';r.align_start=(0,0,0);r.align_heading=0.;r.heading=0.
        o=dict(fresh=True,armed=True,mode=0,local=(2.,0.,0.),velocity_ned=(.1,0,0),
            yaw=0.,speed=.1,bridge=dict(estimator_health=dict(usable=True,all_sensors_lost=False)))
        for i in range(1,100):
            actions=r.step(i*.1,o)
            if actions:self.assertEqual(actions[0][0],'align_velocity')
            self.assertEqual(r.stage,'TRANSIT')
        o['local']=(10.,0,0);r.step(10.,o)
        self.assertEqual(r.stage,'TRANSIT_CHECK')
        self.assertTrue(r.sample_check_ready)
        o['bridge']['estimator_health']['all_sensors_lost']=True
        r.step(10.1,o);self.assertEqual(r.stage,'ABORTED')

    def test_existing_map_still_starts_with_ten_metre_leg(self):
        r=GuidedSurfaceRun(dict(lat=33.0001,lon=-117.,mode='surface',map='geographic',alignment_corridor_confirmed=True),'r',0.)
        r.stage='TRANSIT';r.align_start=(0,0,0);r.align_heading=0.;r.heading=0.;r.surface_down=0.
        e=ContinuousEstimator();e.map.align(33.,-117.,(0,0),0.)
        o=dict(fresh=True,armed=True,mode=0,local=(2.,0,0),velocity_ned=(.1,0,0),yaw=0.,speed=.1,
               bridge=dict(map_transform=e.map.snapshot(),estimator_health=dict(usable=True,all_sensors_lost=False)))
        self.assertEqual(r.step(.1,o)[0][0],'align_velocity')
        self.assertFalse(r.geographic_tracking)
        o['local']=(10.,0,0);r.step(.2,o)
        self.assertTrue(r.alignment_complete)
        r.step(.4,o);self.assertTrue(r.geographic_tracking)

    def test_no_metre_stops_after_registration(self):
        r=GuidedSurfaceRun(dict(lat=33.0001,lon=-117.,mode='surface',map='geographic',alignment_corridor_confirmed=True),'r',0.)
        r.stage='TRANSIT';r.align_start=(0,0,0);r.align_heading=0.;r.heading=0.;r.surface_down=0.;r.alignment_complete=True
        e=ContinuousEstimator();e.map.align(33.,-117.,(0,0),0)
        for i in range(1,50):
            o=dict(fresh=True,armed=True,mode=0,local=(i*.1,0,0),velocity_ned=(.1,0,0),yaw=0.,speed=.1,
                bridge=dict(map_transform=e.map.snapshot(),estimator_health=dict(usable=True,all_sensors_lost=False)))
            actions=r.step(i*.1,o)
            if actions:self.assertEqual(actions[0][0],'target')
            self.assertEqual(r.stage,'TRANSIT')


if __name__=='__main__':unittest.main()
