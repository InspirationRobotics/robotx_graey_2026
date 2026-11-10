import unittest
from types import SimpleNamespace
from robotx_graey_2026.api.pixhawk.gateway_core import GatewayPolicy


def message(mid,component=198,**fields):
    return SimpleNamespace(get_msgId=lambda:mid,get_srcComponent=lambda:component,**fields)


class TestGatewayPolicy(unittest.TestCase):
    def setUp(self):
        self.policy=GatewayPolicy()
        self.control=dict(vehicle_fresh=True,sa_active=False,esc_commanded_on=True,inhibit=False,armed=True,published_monotonic=0.)
        self.policy.update_control(self.control,0)

    def test_motion_stops_on_stale_or_killed_controller(self):
        self.assertTrue(self.policy.admit(message(84),198,0)[0])
        self.assertFalse(self.policy.admit(message(84),198,1)[0])
        self.policy.update_control(dict(self.control,sa_active=True),0)
        self.assertFalse(self.policy.admit(message(84),198,0)[0])

    def test_readers_cannot_drive(self):
        self.assertFalse(self.policy.admit(message(84,195),195,0)[0])
        self.assertTrue(self.policy.admit(message(20,195),195,0)[0])

    def test_navigation_owns_odometry(self):
        self.assertTrue(self.policy.admit(message(331,197),197,0)[0])
        self.assertFalse(self.policy.admit(message(331,198),198,0)[0])

    def test_emergency_disarm_survives_stale_control(self):
        self.assertTrue(self.policy.admit(message(76,194,command=400,param1=0),194,99)[0])
        self.assertFalse(self.policy.admit(message(76,194,command=400,param1=1),194,99)[0])

    def test_esc_command_exclusive_to_controller(self):
        self.assertTrue(self.policy.admit(message(76,194,command=183),194,99)[0])
        self.assertFalse(self.policy.admit(message(76,198,command=183),198,0)[0])

    def test_param_writes_and_foreign_components_rejected(self):
        self.assertFalse(self.policy.admit(message(23,197),197,0)[0])
        self.assertFalse(self.policy.admit(message(20,250),250,0)[0])

    def test_force_arm_cannot_bypass_fcu_prearm_checks(self):
        self.assertTrue(self.policy.admit(message(76,198,command=400,param1=1,param2=0),198,0)[0])
        self.assertFalse(self.policy.admit(message(76,198,command=400,param1=1,param2=21196),198,0)[0])
