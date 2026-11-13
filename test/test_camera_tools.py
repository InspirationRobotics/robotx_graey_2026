"""Camera IDs live in api/vision/camera.py and the camera tools use them (depthai/cv2 stubbed: no OAK here)."""
import importlib
import importlib.util
import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = Path(__file__).parents[1]
FRONT, DOWN = '14442C1031B3BFD200', '14442C10C1B6BED200'


class FakeQueue:
    def __init__(self, frame):
        self.frame = frame

    def tryGet(self):
        return NS(getFrame=lambda: self.frame) if self.frame is not None else None


class FakeDevice:
    """dai.Device(pipeline, info, speed): fails at SUPER for MxIds in `usb2_only`."""
    usb2_only = set()

    def __init__(self, pipeline, info, speed):
        if speed.name == 'SUPER' and info.getMxId() in self.usb2_only:
            raise RuntimeError('X_LINK_DEVICE_NOT_FOUND after booting')
        self.speed = speed

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def getUsbSpeed(self):
        return self.speed

    def getConnectedCameras(self):
        return []

    def getOutputQueue(self, name, *_):
        return FakeQueue(np.ones((4, 4)) if name == 'depth' else np.zeros((4, 4, 3)))


def fake_depthai(mxids):
    speed = NS(SUPER=NS(name='SUPER'), HIGH=NS(name='HIGH'))
    device = type('Device', (FakeDevice,), {})
    device.getAllAvailableDevices = staticmethod(lambda: [NS(getMxId=lambda m=m: m) for m in mxids])
    return NS(Device=device, UsbSpeed=speed, Pipeline=MagicMock())


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CameraToolTests(unittest.TestCase):
    def run_with(self, mxids, fn):
        dai = fake_depthai(mxids)
        with patch.dict(sys.modules, {'depthai': dai, 'cv2': MagicMock()}):
            sys.modules.pop('robotx_graey_2026.api.vision.camera', None)
            camera = importlib.import_module('robotx_graey_2026.api.vision.camera')
            camera.build_pipeline = lambda **kw: object()
            try:
                return fn(camera, dai)
            finally:                                       # leave no stub camera behind for other tests
                sys.modules.pop('robotx_graey_2026.api.vision.camera', None)
                sys.modules.pop('robotx_graey_2026.api.vision.bottom_recorder', None)
                vars(importlib.import_module('robotx_graey_2026.api.vision')).pop('camera', None)

    def test_camera_names_come_from_one_table(self):
        def check(camera, _):
            self.assertEqual((camera.FRONT, camera.DOWN), (FRONT, DOWN))
            self.assertEqual([camera.camera_name(m) for m in (FRONT, DOWN, 'ABCDEF123456')],
                             ['FRONT', 'DOWN', '3456'])
            cam_pair = load('tools/cam_pair.py', 'cam_pair_under_test')
            self.assertIs(cam_pair.NAMES, camera.CAMERAS)
        self.run_with([], check)

    def test_cam_test_names_each_camera_and_passes_when_both_work(self):
        def check(camera, _):
            cam_test = load('tools/cam_test.py', 'cam_test_under_test')
            cam_test.GRAB_S = 0.02
            out = io.StringIO()
            with redirect_stdout(out):
                status = cam_test.main()
            self.assertEqual(status, 0)
            self.assertIn('2 OAK device(s) found: DOWN, FRONT', out.getvalue())
            self.assertIn(f'=== DOWN {DOWN}', out.getvalue())
        self.run_with([FRONT, DOWN], check)

    def test_cam_test_fails_on_a_missing_camera_and_flags_usb2_only(self):
        def check(camera, dai):
            dai.Device.usb2_only = {FRONT}
            cam_test = load('tools/cam_test.py', 'cam_test_under_test')
            cam_test.GRAB_S = 0.02
            out = io.StringIO()
            with redirect_stdout(out):
                status = cam_test.main()
            self.assertEqual(status, 1)                          # DOWN is not plugged in
            self.assertIn('not found: DOWN', out.getvalue())
            self.assertIn('USB 2 ONLY', out.getvalue())          # FRONT passed only at HIGH
        self.run_with([FRONT], check)

    def test_bottom_recorder_defaults_to_the_down_camera(self):
        def check(camera, _):
            recorder = importlib.import_module('robotx_graey_2026.api.vision.bottom_recorder')
            with tempfile.TemporaryDirectory() as folder, \
                    patch.object(sys, 'argv', ['bottom_recorder', '--directory', folder]):
                with self.assertRaisesRegex(RuntimeError, DOWN):
                    recorder.main()                              # no OAKs attached: names the DOWN MxId
        self.run_with([], check)


if __name__ == '__main__':
    unittest.main()
