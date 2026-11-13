"""Camera IDs live in api/vision/camera.py and cam_pair / bottom_recorder use them (depthai/cv2 stubbed)."""
import importlib
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).parents[1]
FRONT, DOWN = '14442C1031B3BFD200', '14442C10C1B6BED200'


def fake_depthai(mxids):
    device = type('Device', (), {'getAllAvailableDevices': staticmethod(
        lambda: [NS(getMxId=lambda m=m: m) for m in mxids])})
    return NS(Device=device, UsbSpeed=NS(SUPER=NS(name='SUPER'), HIGH=NS(name='HIGH')), Pipeline=MagicMock())


class CameraToolTests(unittest.TestCase):
    def run_with(self, mxids, fn):
        with patch.dict(sys.modules, {'depthai': fake_depthai(mxids), 'cv2': MagicMock()}):
            sys.modules.pop('robotx_graey_2026.api.vision.camera', None)
            camera = importlib.import_module('robotx_graey_2026.api.vision.camera')
            try:
                return fn(camera)
            finally:                                       # leave no stub camera behind for other tests
                sys.modules.pop('robotx_graey_2026.api.vision.camera', None)
                sys.modules.pop('robotx_graey_2026.api.vision.bottom_recorder', None)
                vars(importlib.import_module('robotx_graey_2026.api.vision')).pop('camera', None)

    def test_camera_names_come_from_one_table(self):
        def check(camera):
            self.assertEqual((camera.FRONT, camera.DOWN), (FRONT, DOWN))
            self.assertEqual([camera.camera_name(m) for m in (FRONT, DOWN, 'ABCDEF123456')],
                             ['FRONT', 'DOWN', '3456'])
            spec = importlib.util.spec_from_file_location('cam_pair_under_test', ROOT/'tools/cam_pair.py')
            cam_pair = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cam_pair)
            self.assertIs(cam_pair.NAMES, camera.CAMERAS)
        self.run_with([], check)

    def test_find_camera_picks_by_mxid(self):
        self.run_with([FRONT, DOWN], lambda camera: self.assertEqual(camera.find_camera(DOWN).getMxId(), DOWN))
        self.run_with([FRONT], lambda camera: self.assertIsNone(camera.find_camera(DOWN)))

    def test_bottom_recorder_defaults_to_the_down_camera(self):
        def check(camera):
            recorder = importlib.import_module('robotx_graey_2026.api.vision.bottom_recorder')
            with tempfile.TemporaryDirectory() as folder, \
                    patch.object(sys, 'argv', ['bottom_recorder', '--directory', folder]):
                with self.assertRaisesRegex(RuntimeError, DOWN):
                    recorder.main()                              # no OAKs attached: names the DOWN MxId
        self.run_with([], check)


if __name__ == '__main__':
    unittest.main()
