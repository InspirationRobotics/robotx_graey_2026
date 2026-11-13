"""The GUI's VN calibration needs all six faces: gravity_face tells them apart."""
import unittest

from robotx_graey_2026.api.navigation.vn_calibration import gravity_face


class GravityFaceTests(unittest.TestCase):
    def test_coverage_requires_distinct_gravity_faces(self):
        faces = [gravity_face(p) for p in ((0, 0, 0), (0, 0, 180), (0, 90, 0), (0, -90, 0), (0, 0, 90), (0, 0, -90))]
        self.assertEqual(set(faces), set(range(6)))

    def test_tilted_between_faces_is_none(self):
        self.assertIsNone(gravity_face((0, 45, 45)))


if __name__ == '__main__':
    unittest.main()
