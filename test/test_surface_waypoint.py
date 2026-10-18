import unittest
import test_waypoint as fixtures
from robotx_graey_2026.api.navigation.waypoint_logic import WaypointRun, validate_plan
from robotx_graey_2026.api.navigation.continuous_supervisor import ContinuousSupervisor, Observation


class SurfaceWaypointTests(unittest.TestCase):
    def setup_surface(self):
        fixture=fixtures.WaypointTests();fixture.setUp()
        plan=dict(fixture.plan);plan.pop('mode');plan.pop('depth_m')
        fixture.run=WaypointRun(plan,'run',0.)
        fixture.o.update(surface_gps_current=True,surface_gps_at_target=False)
        fixture.step();fixture.step();fixture.step()
        fixture.o['mode']=4;fixture.step();fixture.step()
        fixture.o['armed']=True;fixture.step()
        self.assertEqual(fixture.run.stage,'TRANSIT')
        self.assertEqual(fixture.run.phase,'SURFACE_NAV')
        self.assertEqual(fixture.run.target[2],0.)
        return fixture

    def test_default_never_dives_and_requires_gps_arrival(self):
        f=self.setup_surface()
        for i in range(1,11):
            f.o['local']=(i/10,0.,0.);f.step()
        for _ in range(15):f.step()
        self.assertEqual(f.run.stage,'TRANSIT')
        f.o['surface_gps_at_target']=True
        for _ in range(15):f.step()
        self.assertEqual(f.run.stage,'DISARM')
        f.o['armed']=False;f.step();f.step()
        self.assertEqual(f.run.stage,'COMPLETE')

    def test_short_gps_outage_preserves_target_and_recovery(self):
        f=self.setup_surface();target=f.run.target
        f.o['surface_gps_current']=False
        for _ in range(50):f.step()
        self.assertEqual(f.run.stage,'TRANSIT')
        self.assertEqual(f.run.target,target)
        f.o['surface_gps_current']=True;f.step()
        self.assertAlmostEqual(f.run.last_surface_gps,f.now)

    def test_outage_time_and_distance_bounds_yield_control(self):
        f=self.setup_surface();f.o['surface_gps_current']=False
        for _ in range(102):f.step()
        self.assertEqual(f.run.stage,'ABORTED')

    def test_circular_travel_cannot_bypass_distance_bound(self):
        import math
        f=self.setup_surface();f.o['surface_gps_current']=False
        for i in range(1,65):
            f.o['local']=(.3*math.sin(i*.3),.3*(1-math.cos(i*.3)),0.)
            f.step()
        self.assertEqual(f.run.stage,'ABORTED')
        f=self.setup_surface();f.o['surface_gps_current']=False
        for i in range(1,34):
            f.o['local']=(i*.1,0.,0.);f.step()
        self.assertEqual(f.run.stage,'ABORTED')

    def test_default_overrides_old_depth_and_dive_is_explicit(self):
        self.assertEqual(validate_plan(dict(lat=32.,lon=-117.,map='geographic',depth_m=.5))['depth_m'],0.)

    def test_supervisor_captures_surface_reference_and_tolerates_gps_only_loss(self):
        s=ContinuousSupervisor(active=True)
        o=Observation(healthy=True,depth=0.,depth_valid=True,surface_authoritative=True,
            surface_observed=True,configuration_verified=True,intent='SURFACE_NAV',
            intent_fresh=True,run_id='surface',gps_good=True,heading_allowed=True)
        self.assertEqual(s.step(0.,o),[('capture_reference',None)])
        o.reference_ready=o.reference_saved=True;s.step(.1,o)
        self.assertTrue(s.allowed)
        o.gps_good=False;s.step(.2,o);self.assertTrue(s.allowed)
        o.healthy=False;s.step(.3,o);self.assertFalse(s.allowed)
