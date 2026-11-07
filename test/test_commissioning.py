import unittest
import time
from robotx_graey_2026.api.navigation.commissioning import commission_reason


class CommissionTests(unittest.TestCase):
    def setUp(self):
        self.controller = dict(vehicle_fresh=True, armed=False, sa_active=True,
            esc_commanded_on=False, mission_claimed=False, autonomous=False)
        self.bridge = dict(healthy=True, navigation_valid=True, configuration_verified=True,
            transmitting=True, heading_reference_validated=True, depth_m=.01,
            gps_admission=dict(qualified=True))

    def reason(self, **overrides):
        args = dict(controller=self.controller, bridge=self.bridge, cube_healthy=True,
                    pose=dict(ned=[1.,2.,0.]), heading=dict(allowed=True), surface=True, intent=None)
        args.update(overrides)
        return commission_reason(**args)

    def test_healthy_disarmed_request_only(self):
        self.assertEqual(self.reason(), '')
        for key, value in (('armed',True), ('vehicle_fresh',False), ('sa_active',False),
                           ('esc_commanded_on',True), ('mission_claimed',True), ('autonomous',True)):
            with self.subTest(key=key):
                self.assertTrue(self.reason(controller=dict(self.controller, **{key:value})))

    def test_all_navigation_conditions_remain_required(self):
        for key in ('healthy','navigation_valid','configuration_verified','transmitting','heading_reference_validated'):
            with self.subTest(key=key):
                self.assertTrue(self.reason(bridge=dict(self.bridge, **{key:False})))
        for overrides in (dict(cube_healthy=False), dict(pose=None), dict(heading=dict(allowed=False)),
                dict(surface=False), dict(intent={'run_id':'busy'}),
                dict(pose=dict(ned=[0.,float('nan'),0.])),
                dict(bridge=dict(self.bridge, depth_m=.5)),
                dict(bridge=dict(self.bridge, gps_admission=dict(qualified=False)))):
            self.assertTrue(self.reason(**overrides))

    def test_local_prepare_allows_drift_without_gps_qualification(self):
        bridge=dict(self.bridge,coordinate_frame='fixed_local',local_navigation_ready=True,
                    heading_reference_validated=False,depth_m=.25,gps_admission=dict(qualified=False),
                    gps_observation=dict(received=time.monotonic(),fix=3,lat=32.,lon=-117.))
        self.assertEqual(self.reason(bridge=bridge), '')
        for key in ('healthy','navigation_valid','configuration_verified','transmitting'):
            self.assertTrue(self.reason(bridge=dict(bridge,**{key:False})))
        self.assertTrue(self.reason(bridge=dict(bridge,depth_m=float('nan'))))
        self.assertTrue(self.reason(bridge=bridge,cube_healthy=False))
        self.assertTrue(self.reason(bridge=bridge,surface=False))
        self.assertTrue(self.reason(bridge=dict(bridge,gps_observation={})))
