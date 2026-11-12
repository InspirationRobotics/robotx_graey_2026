import unittest
from pathlib import Path

from robotx_graey_2026.api.pixhawk.identity import (
    COMPANION_COMPONENT, COMPANION_SYSTEM, FCU_COMPONENT, FCU_SYSTEM)


class IdentityTests(unittest.TestCase):
    def test_published_fleet_allocation(self):
        self.assertEqual((FCU_SYSTEM, FCU_COMPONENT), (3, 1))
        self.assertEqual((COMPANION_SYSTEM, COMPANION_COMPONENT), (43, 201))
        self.assertEqual(len({FCU_SYSTEM, COMPANION_SYSTEM}), 2)

    def test_competition_overlay_matches_identity_and_minimal_qgc_profile(self):
        path = Path(__file__).parents[1] / 'params/graey_4.7_rfd_competition.parm'
        values = {}
        for line in path.read_text(encoding='utf-8').splitlines():
            line = line.split('#', 1)[0].strip()
            if line:
                key, value = line.split()
                values[key] = int(value)
        self.assertEqual(values['SYSID_THISMAV'], FCU_SYSTEM)
        self.assertEqual({name: values[name] for name in
                          ('MAV2_EXTRA1', 'MAV2_POSITION', 'MAV2_EXT_STAT')},
                         {'MAV2_EXTRA1': 1, 'MAV2_POSITION': 1, 'MAV2_EXT_STAT': 1})
        self.assertEqual({name: values[name] for name in
                          ('MAV2_EXTRA2', 'MAV2_EXTRA3', 'MAV2_RAW_SENS', 'MAV2_RC_CHAN')},
                         {'MAV2_EXTRA2': 0, 'MAV2_EXTRA3': 0,
                          'MAV2_RAW_SENS': 0, 'MAV2_RC_CHAN': 0})


if __name__ == '__main__':
    unittest.main()
