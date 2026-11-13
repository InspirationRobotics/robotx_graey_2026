#!/usr/bin/env python3
"""OAK-D pipeline, camera IDs and the MJPEG view, shared by tools/cam_pair.py and
api/vision/bottom_recorder.py.

Only ONE process may hold a camera at a time.
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import depthai as dai

# 16:9 ON PURPOSE, and it is worth ~5 degrees of view. The IMX378 does not support
# THE_800_P, so depthai falls back to a 1920x1080 ISP. Preview keeps aspect ratio by
# default, so a 16:10 preview like 640x400 gets its WIDTH centre-cropped by 10% and
# the vehicle flies at about 63.5 deg HFOV instead of the lens's 69. Matching the ISP
# aspect uses the full sensor width for nothing. Horizontal stops there - 69 deg is
# the lens, and the OAK-D W that had 150 deg is dead. The remaining view is VERTICAL:
# a 4:3 sensor mode would give 55 deg instead of 42, but needs setIspScale to avoid
# the NvMap exhaustion in documentation section 4.7.
W, H = 640, 360

# Graey's two OAK-Ds by MxId (stable hardware ids), so a camera is always picked and labelled the same
# way: tools/cam_pair.py (DOWN left, FRONT right) and api/vision/bottom_recorder.py.
FRONT, DOWN = '14442C1031B3BFD200', '14442C10C1B6BED200'
CAMERAS = {FRONT: 'FRONT', DOWN: 'DOWN'}


def camera_name(mxid):
    """'FRONT', 'DOWN', or the last four characters of an unknown camera's MxId."""
    return CAMERAS.get(mxid, mxid[-4:])


def find_camera(mxid):
    """The attached OAK with this MxId, or None (unplugged, or held by another process)."""
    return next((d for d in dai.Device.getAllAvailableDevices() if d.getMxId() == mxid), None)


_frame = None
_lock = threading.Lock()
_streams = 0                    # open /stream connections: a browser view or the recorder


def viewing():
    """True while anything reads /stream (the GUI's camera view, or a recording)."""
    with _lock:
        return _streams > 0

PAGE = b"""<html><head><title>Graey OAK-D</title></head>
<body style="margin:0;background:#111;text-align:center">
<img src="/stream" style="max-width:100%"></body></html>"""


def publish_frame(frame):
    """Encode a frame and hand it to the MJPEG server."""
    global _frame
    ok, buf = cv2.imencode('.jpg', frame)
    if ok:
        with _lock:
            _frame = buf.tobytes()


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return
        if self.path != '/stream':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()
        global _streams
        with _lock:
            _streams += 1
        try:
            while True:
                with _lock:
                    data = _frame
                if data:
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
                time.sleep(0.08)
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            with _lock:
                _streams -= 1

    def log_message(self, *a):
        pass                                        # silence per-request logging


def serve_mjpeg(port=8080):
    threading.Thread(target=lambda: ThreadingHTTPServer(
        ('0.0.0.0', port), _Handler).serve_forever(), daemon=True).start()


def build_pipeline(fps=10, with_depth=True):
    p = dai.Pipeline()
    cam = p.createColorCamera()
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    cam.setPreviewSize(W, H)
    cam.setInterleaved(False)
    cam.setFps(fps)
    xr = p.createXLinkOut()
    xr.setStreamName('rgb')
    cam.preview.link(xr.input)
    if not with_depth:
        return p

    mono = []
    for socket in (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C):
        m = p.createMonoCamera()
        m.setBoardSocket(socket)
        m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        m.setFps(fps)
        mono.append(m)
    st = p.createStereoDepth()
    st.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    st.setLeftRightCheck(True)
    st.setDepthAlign(dai.CameraBoardSocket.CAM_A)   # depth pixels line up with RGB
    st.setOutputSize(W, H)
    mono[0].out.link(st.left)
    mono[1].out.link(st.right)
    xd = p.createXLinkOut()
    xd.setStreamName('depth')
    st.depth.link(xd.input)
    return p

