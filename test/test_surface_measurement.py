import unittest
from robotx_graey_2026.api.navigation.surface_measurement import SurfaceMeasurement


class SurfaceTests(unittest.TestCase):
    def sample(self,t,north=0.,accuracy=1.):
        return dict(healthy=True,surfaced=True,gps_eligible=True,position_ned=[0.,0.,0.],
                    gps_observation=dict(received=t,cube_fix_time_us=round(t*1e6),lat=33.+north/111320,lon=-117.,accuracy=accuracy),
                    gps_admission=dict(qualified=True),heading_dvl=dict(received=t,valid=True,velocity=[0.,0.,0.]),
                    heading_vn=dict(received=t,rate=0.))

    def test_waits_full_window_and_measures_scatter(self):
        s=SurfaceMeasurement()
        for i in range(101):
            t=i*.2;r=s.update(t,self.sample(t,north=.1*(i%2)))
            self.assertEqual(r['ready'],i==100)
        self.assertLess(r['scatter_radius_m'],.1)

    def test_repeated_receiver_timestamp_does_not_manufacture_samples(self):
        s=SurfaceMeasurement();b=self.sample(0.)
        for i in range(201):
            t=i*.1;b['gps_observation']['received']=t;b['heading_vn']['received']=t;b['heading_dvl']['received']=t
            self.assertFalse(s.update(t,b)['ready'])

    def test_jump_rejection_motion_and_recovery_reset_window(self):
        for fault in ('accuracy','invalid','moving','dvl_recovery','gps_rejection'):
            with self.subTest(fault=fault):
                s=SurfaceMeasurement()
                for i in range(101):r=s.update(i*.2,self.sample(i*.2))
                self.assertTrue(r['ready'])
                b=self.sample(20.2)
                if fault=='accuracy':b['gps_observation']['accuracy']=5.
                if fault=='invalid':b['heading_dvl']['valid']=False
                if fault=='moving':b['heading_dvl']['velocity']=[.1,0.,0.]
                if fault=='dvl_recovery':b['dvl_continuity']=dict(recovering=True)
                if fault=='gps_rejection':b['gps_admission']['qualified']=False
                self.assertFalse(s.update(20.2,b)['ready'])
                self.assertFalse(s.update(20.4,self.sample(20.4))['ready'])

    def test_stable_but_unknown_bias_can_pass(self):
        s=SurfaceMeasurement()
        for i in range(101):r=s.update(i*.2,self.sample(i*.2,north=10.))
        self.assertTrue(r['ready'])  # Stability does not establish absolute accuracy.

    def test_wandering_stationary_gps_never_qualifies(self):
        s=SurfaceMeasurement()
        for i in range(200):
            t=i*.2;r=s.update(t,self.sample(t,north=t*.2))
            self.assertFalse(r['ready'])

    def test_turn_rate_units_and_raw_velocity_are_used(self):
        s=SurfaceMeasurement()
        for i in range(101):
            t=i*.2;b=self.sample(t);b['heading_vn']['rate']=1.
            r=s.update(t,b)
        self.assertTrue(r['ready'])  # 1 degree/s, not radians/s.
        b=self.sample(20.2);b['heading_dvl']['raw_velocity']=[.1,0.,0.]
        self.assertFalse(s.update(20.2,b)['ready'])
