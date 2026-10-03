import random
import tempfile
import unittest
from pathlib import Path

from robotx_graey_2026.api.navigation.mission_reference import MissionReference


class MissionReferenceTests(unittest.TestCase):
    def inputs(self):
        return dict(run_id='task2-dive1', vehicle_epoch='cube-frame-A',
                    gps=dict(lat=32.7, lon=-117.2, alt=4.2, accuracy=1.2,
                             fix=3, receiver_time_us=1000, received=10),
                    pose=dict(ned=(125, -20, .12), received=10.1),
                    now=10.2, captured_utc='2026-10-02T20:00:00Z',
                    surface_confirmed=True, gps_qualified=True, ekf_healthy=True)

    def test_dive_point_is_zero_without_changing_cube_coordinates(self):
        ref = MissionReference.capture(**self.inputs())
        self.assertEqual(ref.from_cube((125, -20, .12), vehicle_epoch='cube-frame-A'), (0, 0, 0))
        self.assertEqual(ref.to_cube((0, 0, 2), vehicle_epoch='cube-frame-A'), (125, -20, 2.12))

    def test_transform_round_trip_random_targets(self):
        ref = MissionReference.capture(**self.inputs())
        rng = random.Random(42)
        for _ in range(1000):
            target = tuple(rng.uniform(-100, 100) for _ in range(3))
            actual = ref.from_cube(ref.to_cube(target, vehicle_epoch='cube-frame-A'),
                                   vehicle_epoch='cube-frame-A')
            for a, b in zip(target, actual):
                self.assertAlmostEqual(a, b)

    def test_stale_or_unqualified_capture_rejected(self):
        for key in ('surface_confirmed', 'gps_qualified', 'ekf_healthy'):
            data = self.inputs(); data[key] = False
            with self.subTest(key=key), self.assertRaises(ValueError):
                MissionReference.capture(**data)
        for received in (1, 11, float('nan')):
            data = self.inputs(); data['gps']['received'] = received
            with self.subTest(received=received), self.assertRaises(ValueError):
                MissionReference.capture(**data)

    def test_accuracy_and_time_validation(self):
        for key, value in [('accuracy', 0), ('accuracy', float('nan')),
                           ('accuracy', 20), ('fix', 1), ('receiver_time_us', 0)]:
            data = self.inputs(); data['gps'][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                MissionReference.capture(**data)

    def test_reboot_or_unverified_frame_cannot_reuse_reference(self):
        ref = MissionReference.capture(**self.inputs())
        for epoch in ('cube-frame-B', '', None):
            with self.subTest(epoch=epoch), self.assertRaises(ValueError):
                ref.to_cube((0, 0, 0), vehicle_epoch=epoch)

    def test_persistence_retains_marker_and_frame(self):
        ref = MissionReference.capture(**self.inputs())
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'reference.json'
            ref.save(path)
            restored = MissionReference.load_history(path)
            self.assertEqual(restored, ref)
            self.assertEqual(len(list(Path(folder).iterdir())), 1)


if __name__ == '__main__':
    unittest.main()
