import unittest
import test_bridge_estimator as fixtures
from robotx_graey_2026.api.navigation.automatic_recovery import AutomaticRecovery


class AutomaticRecoveryTests(unittest.TestCase):
    def setup_fault(self):
        f=fixtures.EstimatorTests();f.setUp();f.recovery_fixture()
        self.f=f;self.a=AutomaticRecovery()
        self.c=dict(vehicle_fresh=True,armed=False,sa_active=True,esc_commanded_on=False,
                    autonomous=False,mission_claimed=False,mode='MANUAL')

    def step(self,**kw):
        self.f.step(**kw)
        return self.a.step(self.f.t,self.f.e,self.c,self.f.t)

    def test_auto_reanchors_with_fresh_qualified_gps_preserving_north(self):
        self.setup_fault();before=self.f.e.orientation();triggered=0
        for _ in range(180):triggered+=int(self.step(north=12.))
        self.assertEqual(triggered,1)
        self.assertTrue(self.a.complete)
        self.assertTrue(self.f.e.status(self.f.t)['healthy'])
        self.assertEqual(self.f.e.orientation(),before)
        self.assertAlmostEqual(self.f.e.position[0],12.,places=2)
        self.assertEqual(self.a.count,1)

    def test_no_recovery_while_armed_or_mission_active(self):
        for key,value in (('armed',True),('sa_active',False),('esc_commanded_on',True),
                          ('autonomous',True),('mission_claimed',True),('mode','GUIDED')):
            self.setup_fault();self.c[key]=value
            for _ in range(100):self.assertFalse(self.step())
            self.assertIn('DVL continuity lost',self.f.e.fault)

    def test_submerged_bad_gps_vn_loss_and_unrelated_fault_do_not_recover(self):
        for fault in ('submerged','bad_gps','vn','other','stale_control','gps_clock'):
            self.setup_fault()
            if fault=='vn':self.f.e.heading_continuity_lost=True
            if fault=='other':self.f.e.fault='Cube origin changed'
            if fault=='gps_clock':self.f.e.gps.fault='Cube GPS clock regressed'
            for _ in range(120):
                self.f.step(depth=.5 if fault=='submerged' else 0.,gps=fault!='bad_gps')
                self.assertFalse(self.a.step(self.f.t,self.f.e,self.c,self.f.t-1 if fault=='stale_control' else self.f.t))
            self.assertTrue(self.f.e.fault)

    def test_gps_jump_never_becomes_recovery_anchor(self):
        self.setup_fault()
        for i in range(120):self.assertFalse(self.step(north=0. if i<50 else 50.))
        self.assertFalse(self.a.complete)

    def test_arm_during_reanchor_pauses_completion(self):
        self.setup_fault()
        for _ in range(120):
            if self.step(north=10.):break
        self.assertTrue(self.a.pending)
        self.c['armed']=True
        for _ in range(100):self.step(north=10.)
        self.assertTrue(self.a.pending)
        self.assertFalse(self.a.complete)
