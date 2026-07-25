#!/usr/bin/env python3
"""OAK-D + YOLO pole viewer -> http://<jetson-ip>:8080

Runs bestPoles724.pt (YOLO11n: red/white poles) on the Orin GPU against the
OAK-D RGB preview, samples OAK-D stereo depth per detection, and overlays
forward/right distance in meters. Step 1 of CV-assisted prequal.
Only one process may hold the camera. Ctrl-C to stop."""
import threading
import time

import cv2
import depthai as dai
import numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ultralytics import YOLO

MODEL = '/root/robotx_ws/src/robotx_graey_2026/models/bestPoles724.pt'
W, H = 640, 400
CONF = 0.4

frame_jpg = None
lock = threading.Lock()

PAGE = b"""<html><head><title>Graey poles</title></head>
<body style="margin:0;background:#111;text-align:center">
<img src="/stream" style="max-width:100%">
</body></html>"""


def grab():
    global frame_jpg
    model = YOLO(MODEL)
    p = dai.Pipeline()

    cam = p.createColorCamera()
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    cam.setPreviewSize(W, H)
    cam.setInterleaved(False)
    cam.setFps(10)

    monoL = p.createMonoCamera()
    monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoL.setFps(10)
    monoR = p.createMonoCamera()
    monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoR.setFps(10)

    stereo = p.createStereoDepth()
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(W, H)
    monoL.out.link(stereo.left)
    monoR.out.link(stereo.right)

    xr = p.createXLinkOut()
    xr.setStreamName('rgb')
    cam.preview.link(xr.input)
    xd = p.createXLinkOut()
    xd.setStreamName('depth')
    stereo.depth.link(xd.input)

    with dai.Device(p) as dev:
        print('USB speed:', dev.getUsbSpeed())
        calib = dev.readCalibration()
        K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, W, H))
        fx, cx = K[0][0], K[0][2]
        print(f'intrinsics fx={fx:.1f} cx={cx:.1f}')
        qr = dev.getOutputQueue('rgb', 4, False)
        qd = dev.getOutputQueue('depth', 4, False)
        depth = None
        while True:
            d = qd.tryGet()
            if d is not None:
                depth = d.getFrame()
            frame = qr.get().getCvFrame()
            res = model.predict(frame, conf=CONF, verbose=False, device=0)[0]
            for b in res.boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                cls = res.names[int(b.cls[0])]
                col = (0, 0, 255) if cls == 'red' else (255, 255, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
                label = f'{cls} {float(b.conf[0]):.2f}'
                if depth is not None:
                    cw = max(2, (x2 - x1) // 4)
                    ch = max(2, (y2 - y1) // 4)
                    mx, my = (x1 + x2) // 2, (y1 + y2) // 2
                    roi = depth[max(0, my - ch):my + ch, max(0, mx - cw):mx + cw]
                    vals = roi[roi > 0]
                    if vals.size > 20:
                        z = float(np.median(vals)) / 1000.0
                        x_m = (mx - cx) * z / fx
                        label += f'  fwd {z:.2f}m  right {x_m:+.2f}m'
                cv2.putText(frame, label, (x1, max(15, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            ok, buf = cv2.imencode('.jpg', frame)
            if ok:
                with lock:
                    frame_jpg = buf.tobytes()


class Handler(BaseHTTPRequestHandler):
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
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()
        try:
            while True:
                with lock:
                    data = frame_jpg
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
        pass


if __name__ == '__main__':
    threading.Thread(target=grab, daemon=True).start()
    print('serving on http://0.0.0.0:8080')
    ThreadingHTTPServer(('0.0.0.0', 8080), Handler).serve_forever()
