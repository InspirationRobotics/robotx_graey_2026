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
        self.assertEqual(values['MAV_SYSID'], FCU_SYSTEM)
        self.assertEqual({name: values[name] for name in
                          ('MAV1_EXTRA1', 'MAV1_POSITION', 'MAV1_EXT_STAT')},
                         {'MAV1_EXTRA1': 1, 'MAV1_POSITION': 1, 'MAV1_EXT_STAT': 1})
        self.assertEqual({name: values[name] for name in
                          ('MAV1_ADSB', 'MAV1_EXTRA2', 'MAV1_EXTRA3', 'MAV1_PARAMS',
                           'MAV1_RAW_CTRL', 'MAV1_RAW_SENS', 'MAV1_RC_CHAN')},
                         {'MAV1_ADSB': 0, 'MAV1_EXTRA2': 0, 'MAV1_EXTRA3': 0,
                          'MAV1_PARAMS': 0, 'MAV1_RAW_CTRL': 0,
                          'MAV1_RAW_SENS': 0, 'MAV1_RC_CHAN': 0})
        self.assertFalse(any(name.startswith(('SERIAL', 'BRD_SER')) for name in values))


if __name__ == '__main__':
    unittest.main()
