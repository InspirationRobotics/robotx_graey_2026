import unittest
from robotx_graey_2026.api.navigation.continuous_estimator import ContinuousEstimator
from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun
from robotx_graey_2026.api.navigation.geographic_frame import GeographicFrame

class ManualOnlyTests(unittest.TestCase):
    def test_gps_fuses_without_heading_qualification_and_does_not_fit_north(self):
        e=ContinuousEstimator();e.imu(1.,(1,0,0,0),(0,0,0))
        e.gps.clock.offset=0.;e.gps.clock.updated=1.;e.gps.clock.uncertainty=.001
        e.north_alignment.begin()
        self.assertTrue(e.apply_gps(1.,dict(cube_fix_time_us=1000000,lat=33.,lon=-117.,fix=3,accuracy=.8)))
        self.assertFalse(e.heading_validated);self.assertIsNone(e.heading_reset)
        self.assertTrue(e.status(1.)['estimator_health']['gps_position_fused'])
        e.begin_guided_alignment('run')
        self.assertFalse(e.alignment_collecting)

    def test_registered_mission_without_alignment_corridor_enters_orientation(self):
        r=GuidedSurfaceRun(dict(lat=33.,lon=-117.,mode='surface',map='geographic'),'run',0.)
        r.stage='ARM'
        frame=GeographicFrame();frame.align(33.,-117.,(0.,0.),0.)
        r.step(.1,dict(fresh=True,mode=4,armed=True,bridge=dict(map_transform=frame.snapshot()),local=(0.,0.,0.),yaw=0.,speed=0.,depth=0.))
        self.assertEqual(r.stage,'ORIENT')

if __name__=='__main__':unittest.main()
