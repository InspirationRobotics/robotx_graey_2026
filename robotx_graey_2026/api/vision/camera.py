#!/usr/bin/env python3
"""OAK-D pipeline plus the MJPEG debug view, shared by pole_tracker and oak_view.

Only ONE process may hold the camera at a time - the node and the tool cannot
run together.
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import depthai as dai
import numpy as np

W, H = 640, 400

_frame = None
_lock = threading.Lock()

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


def intrinsics(dev):
    """(fx, cx) for the colour camera at our preview size."""
    K = np.array(dev.readCalibration().getCameraIntrinsics(
        dai.CameraBoardSocket.CAM_A, W, H))
    return K[0][0], K[0][2]


def range_bearing(depth, box, fx, cx):
    """Median depth in the middle of a box -> (forward_m, right_m), or None.

    Stereo writes zero wherever it found no match, so those pixels are dropped
    rather than averaged - averaging them pulls every range toward zero.
    """
    x1, y1, x2, y2 = box
    mx, my = (x1 + x2) // 2, (y1 + y2) // 2
    cw, ch = max(2, (x2 - x1) // 4), max(2, (y2 - y1) // 4)
    vals = depth[max(0, my - ch):my + ch, max(0, mx - cw):mx + cw]
    vals = vals[vals > 0]
    if vals.size <= 20:
        return None
    z = float(np.median(vals)) / 1000.0
    if not 0.2 < z < 20.0:
        return None
    return z, (mx - cx) * z / fx
