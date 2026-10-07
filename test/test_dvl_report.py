import unittest
from robotx_graey_2026.api.navigation.dvl_report import validate_report


class DVLReportTests(unittest.TestCase):
    def test_valid_velocity_and_covariance_scale_together(self):
        result = validate_report(dict(vx=1, vy=2, vz=3, velocity_valid=True,
            covariance=[[1, 0, 0], [0, 2, 0], [0, 0, 3]]), 2)
        self.assertEqual(result[0], [2, 4, 6])
        self.assertEqual(result[3][2][2], 12)

    def test_malformed_reports_rejected_before_partial_publish(self):
        good = dict(vx=1, vy=2, vz=3, velocity_valid=True)
        for report in (None, [], dict(vx=1), dict(good, vx=float('nan')),
                       dict(good, velocity_valid='false'), dict(good, covariance=[1]),
                       dict(good, covariance=[[1, 0, 0], [0, -1, 0], [0, 0, 1]])):
            with self.subTest(report=report):
                with self.assertRaises((ValueError, TypeError, KeyError)):
                    validate_report(report, 1)

    def test_nonvelocity_and_no_bottom_lock_are_distinct(self):
        self.assertIsNone(validate_report({'type': 'position'}, 1))
        self.assertFalse(validate_report(dict(vx=0, vy=0, vz=0, velocity_valid=False), 1)[2])
