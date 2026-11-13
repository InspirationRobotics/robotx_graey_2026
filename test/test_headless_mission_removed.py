"""The deleted always-running SE mission cannot return through core wiring."""
from pathlib import Path
import unittest


ROOT=Path(__file__).parents[1]


class HeadlessMissionRemovalTests(unittest.TestCase):
    def test_core_does_not_launch_a_waypoint_mission(self):
        core=(ROOT/'launch/core.launch.py').read_text(encoding='utf-8')
        self.assertNotIn("executable='waypoint_mission'",core)
        self.assertNotIn('rc_simple_dive_enabled',core)

    def test_headless_helpers_and_watchdog_lease_are_absent(self):
        navigation=ROOT/'robotx_graey_2026/api/navigation'
        self.assertFalse((navigation/'rc_start.py').exists())
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
