"""Execute real pilot callbacks with transport mocked; no vehicle commands."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import TestCase
from unittest.mock import Mock


class PilotOwnershipTests(TestCase):
    def setUp(self):
        path=Path(__file__).parents[1]/'robotx_graey_2026/api/pixhawk/kill_switch.py'
        tree=ast.parse(path.read_text())
        tree.body=[n for n in tree.body if isinstance(n,(ast.ClassDef,ast.FunctionDef))]
        ns=dict(Node=object,PWM_MIN=800,PWM_MAX=2200,CMD_HOLD_S=3.,MODE_RESEND_S=1.,
                MODE_MANUAL=19,MODE_STABILIZE=0,AUTO_MODES={'GUIDED','AUTO'},
                mavutil=NS(mavlink=NS(MAV_CMD_COMPONENT_ARM_DISARM=400)))
        exec(compile(tree,str(path),'exec'),ns)
        self.s=ns['KillSwitch'].__new__(ns['KillSwitch'])
        self.now=10.
        s=self.s
        s.now=lambda:self.now
        s.link=Mock();s.get_logger=Mock(return_value=Mock())
        s.vehicle_fresh=Mock(return_value=True)
        s.source='RF';s.inhibit=None;s.sa_active=False;s.revision=0
        s.mode='MANUAL';s.armed=True;s.arm_ch=11;s.mode_ch=7;s.high=1500
        s.rc={11:1000,7:1000};s.armed_req=False;s.mode_req=19;s.mode_t=0.
        s.auto_until=0.;s.cmd_until=13.
        s.report_rf=Mock();s.refresh_sa=Mock();s.reset_rf_if_safe=Mock()
        s.rf_alive=Mock(return_value=True);s.selected_alive=Mock(return_value=True)
        s.set_esc_power=Mock()

    def test_static_disarm_switch_and_old_retry_do_not_fight_guided(self):
        for source in ('RF','TETHER'):
            self.s.source=source;self.s.mode='GUIDED';self.s.cmd_until=13.
            self.s.tick()
            self.s.link.command.assert_not_called()
            self.s.link.set_mode.assert_not_called()
            self.assertEqual(self.s.cmd_until,0.)

    def test_old_arm_retry_does_not_force_manual_during_mission(self):
        self.s.mode='GUIDED';self.s.armed=False;self.s.armed_req=True
        self.s.do_arm(1900,True)
        self.s.link.set_mode.assert_not_called();self.s.link.command.assert_not_called()

    def test_claim_cancels_prior_retries_before_guided_confirmation(self):
        self.s.on_auto(NS(data=True))
        self.assertEqual(self.s.cmd_until,0.)
        self.s.do_arm(1000,True)
        self.s.link.command.assert_not_called();self.s.link.set_mode.assert_not_called()

    def test_new_pilot_disarm_edge_retries_but_never_arms(self):
        self.s.mode='GUIDED';self.s.armed_req=True
        self.s.do_arm(1000,True)
        self.s.link.command.assert_called_with(400,0)
        self.now+=.2;self.s.do_arm(1000,True)
        self.assertEqual(self.s.link.command.call_count,2)
        self.s.link.command.reset_mock();self.s.do_arm(1900,True)
        self.s.link.command.assert_not_called();self.s.link.set_mode.assert_not_called()

    def test_sa_kill_still_overrides_claim_and_guided(self):
        for source in ('RF','TETHER'):
            self.s.source=source;self.s.mode='GUIDED';self.s.sa_active=True
            self.s.auto_until=20.;self.s.inhibit=None
            self.s.tick()
            self.s.set_esc_power.assert_called_with(False)
            self.s.link.command.assert_called_with(400,0)

    def test_manual_connection_loss_is_not_bypassed_by_claim(self):
        self.s.auto_until=20.;self.s.selected_alive.return_value=False
        self.s.tick()
        self.assertEqual(self.s.inhibit,'RF_LOST')
        self.s.link.command.assert_called_with(400,0)
