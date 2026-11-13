"""Exercise the actual watchdog methods without ROS or a vehicle connection."""
import ast
from pathlib import Path
import unittest
from unittest.mock import Mock


class ConnectionPolicyTests(unittest.TestCase):
    def watchdog(self, mode='GUIDED', source='RF'):
        path = Path(__file__).parents[1]/'robotx_graey_2026/api/pixhawk/kill_switch.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and
                   any(isinstance(m, ast.FunctionDef) and m.name == 'tick' for m in n.body))
        cls.bases = [ast.Name(id='object', ctx=ast.Load())]
        cls.body = [m for m in cls.body if isinstance(m, ast.FunctionDef) and m.name in ('tick', 'autonomous')]
        tree.body = [cls]
        ns = dict(AUTO_MODES={'GUIDED', 'AUTO', 'SURFACE'})
        exec(compile(ast.fix_missing_locations(tree), str(path), 'exec'), ns)
        node = ns[cls.name]()
        for name in ('report_rf', 'refresh_sa', 'reset_rf_if_safe', 'stop_outputs', 'set_esc_power',
                     'cancel_arm_request', 'latch', 'do_mode', 'do_arm'):
            setattr(node, name, Mock())
        node.vehicle_fresh = Mock(return_value=True)
        node.selected_alive = Mock(return_value=False)
        node.rf_alive = Mock(return_value=False)
        node.sa_active = False
        node.inhibit = None
        node.source = source
        node.mode = mode
        return node

    def test_guided_survives_wifi_or_rf_loss(self):
        for source in ('RF', 'TETHER'):
            node = self.watchdog(source=source)
            node.tick()
            node.stop_outputs.assert_not_called()
            node.latch.assert_not_called()
            node.set_esc_power.assert_called_once_with(True)

    def test_manual_loss_and_physical_kill_still_stop(self):
        node = self.watchdog(mode='MANUAL')
        node.tick()
        node.latch.assert_called_once_with('RF_LOST')
        node.stop_outputs.assert_called_once()
        node = self.watchdog()
        node.sa_active = True
        node.tick()
        node.stop_outputs.assert_called_once_with(force_manual=True)

    def test_cube_link_loss_is_not_wifi_loss(self):
        node = self.watchdog()
        node.vehicle_fresh.return_value = False
        node.tick()
        node.latch.assert_called_once_with('FC_LOST')
        node.stop_outputs.assert_called_once()
