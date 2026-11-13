"""The deleted always-running SE mission cannot return through core wiring."""
import ast
from pathlib import Path
import unittest


ROOT=Path(__file__).parents[1]


class HeadlessMissionRemovalTests(unittest.TestCase):
    def test_se_executor_is_off_unless_explicitly_enabled(self):
        tree=ast.parse((ROOT/'launch/core.launch.py').read_text(encoding='utf-8'))
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)]
        switch='rc_simple_dive_enabled'
        arg,=[c for c in calls if getattr(c.func,'id','')=='DeclareLaunchArgument'
              and ast.literal_eval(c.args[0])==switch]
        self.assertEqual(ast.literal_eval(next(k.value for k in arg.keywords if k.arg=='default_value')),'false')
        node,=[c for c in calls if getattr(c.func,'id','')=='Node'
               and any(k.arg=='executable' and ast.literal_eval(k.value)=='waypoint_mission' for k in c.keywords)]
        condition=next(k.value for k in node.keywords if k.arg=='condition')
        self.assertEqual(ast.dump(condition),ast.dump(ast.parse(
            "IfCondition(LaunchConfiguration('rc_simple_dive_enabled'))",mode='eval').body))

    def test_headless_recorder_and_watchdog_lease_are_absent(self):
        navigation=ROOT/'robotx_graey_2026/api/navigation'
        self.assertFalse((navigation/'onboard_recording.py').exists())
        watchdog=(ROOT/'robotx_graey_2026/api/pixhawk/kill_switch.py').read_text(encoding='utf-8')
        self.assertNotIn('/graey/control_watchdog/onboard_run',watchdog)
        self.assertNotIn('onboard_claimed',watchdog)

    def test_operator_selected_chris_mission_is_retained(self):
        mission=(ROOT/'robotx_graey_2026/api/navigation/dive_spin.py').read_text(encoding='utf-8')
        gui=(ROOT/'tools/missions.js').read_text(encoding='utf-8')
        self.assertIn('DiveSpinRun',mission)
        self.assertIn('dive_here',gui)

    def test_rfd_mission_endpoint_is_opt_in(self):
        core=(ROOT/'launch/core.launch.py').read_text(encoding='utf-8')
        self.assertIn("DeclareLaunchArgument('radio_mission_enabled',default_value='false'",core)
        self.assertIn("IfCondition(LaunchConfiguration('radio_mission_enabled'))",core)


if __name__=='__main__':unittest.main()
