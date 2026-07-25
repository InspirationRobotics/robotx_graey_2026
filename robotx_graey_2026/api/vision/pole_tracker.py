#!/usr/bin/env python3
"""Red-pole tracker: OAK-D + YOLO11 -> filtered body-frame position.

Publishes:
  /graey/pole/visible   std_msgs/Bool             stable detection?
  /graey/pole           geometry_msgs/PointStamped  x=forward, y=right, z=0 (m)

Detection flicker is absorbed by requiring the pole in HITS_NEEDED of the last
WINDOW frames, and by median-filtering range/bearing over the recent history.

Also serves a live debug view at http://<jetson-ip>:8080 and, when
save_dir is set, writes a frame every save_period seconds while detecting -
that pile of images is the retraining dataset for our real water/camera.
"""
import os
import threading
import time
from collections import deque

import cv2
import depthai as dai
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ultralytics import YOLO

W, H = 640, 400
frame_jpg = None
jlock = threading.Lock()

PAGE = b"""<html><head><title>Graey pole tracker</title></head>
<body style="margin:0;background:#111;text-align:center">
<img src="/stream" style="max-width:100%"></body></html>"""


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
                with jlock:
                    d = frame_jpg
                if d:
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(d)))
                    self.end_headers()
                    self.wfile.write(d)
                    self.wfile.write(b'\r\n')
                time.sleep(0.08)
        except (ConnectionResetError, BrokenPipeError):
            pass

    def log_message(self, *a):
        pass


class PoleTracker(Node):
    def __init__(self):
        super().__init__('pole_tracker')
        p = self.declare_parameter
        p('model', '/root/robotx_ws/src/robotx_graey_2026/models/bestPoles724.pt')
        p('target_class', 'red')
        p('conf', 0.4)
        p('window', 10)
        p('hits_needed', 5)
        p('save_dir', '')
        p('save_period', 0.5)

        g = self.get_parameter
        self.model_path = g('model').value
        self.target = g('target_class').value
        self.conf = g('conf').value
        self.window = g('window').value
        self.hits = g('hits_needed').value
        self.save_dir = g('save_dir').value
        self.save_period = g('save_period').value
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

        self.pub_vis = self.create_publisher(Bool, '/graey/pole/visible', 10)
        self.pub_pos = self.create_publisher(PointStamped, '/graey/pole', 10)

        self.seen = deque(maxlen=self.window)
        self.fwd = deque(maxlen=self.window)
        self.rgt = deque(maxlen=self.window)
        self.last_save = 0.0
        self.visible = False
        self.last_report = 0.0

        threading.Thread(target=self.camera_loop, daemon=True).start()
        threading.Thread(target=lambda: ThreadingHTTPServer(
            ('0.0.0.0', 8080), Handler).serve_forever(), daemon=True).start()
        self.get_logger().info('pole_tracker starting; debug view on :8080')

    def camera_loop(self):
        global frame_jpg
        model = YOLO(self.model_path)
        p = dai.Pipeline()
        cam = p.createColorCamera()
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam.setPreviewSize(W, H)
        cam.setInterleaved(False)
        cam.setFps(10)
        mL = p.createMonoCamera()
        mL.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mL.setFps(10)
        mR = p.createMonoCamera()
        mR.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mR.setFps(10)
        st = p.createStereoDepth()
        st.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        st.setLeftRightCheck(True)
        st.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        st.setOutputSize(W, H)
        mL.out.link(st.left)
        mR.out.link(st.right)
        xr = p.createXLinkOut()
        xr.setStreamName('rgb')
        cam.preview.link(xr.input)
        xd = p.createXLinkOut()
        xd.setStreamName('depth')
        st.depth.link(xd.input)

        with dai.Device(p) as dev:
            self.get_logger().info(f'OAK-D USB speed {dev.getUsbSpeed()}')
            K = np.array(dev.readCalibration().getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A, W, H))
            fx, cx = K[0][0], K[0][2]
            qr = dev.getOutputQueue('rgb', 4, False)
            qd = dev.getOutputQueue('depth', 4, False)
            depth = None
            while rclpy.ok():
                d = qd.tryGet()
                if d is not None:
                    depth = d.getFrame()
                frame = qr.get().getCvFrame()
                raw = frame.copy()
                res = model.predict(frame, conf=self.conf, verbose=False, device=0)[0]

                best = None
                for b in res.boxes:
                    if res.names[int(b.cls[0])] != self.target:
                        continue
                    if best is None or float(b.conf[0]) > float(best.conf[0]):
                        best = b

                hit = False
                if best is not None and depth is not None:
                    x1, y1, x2, y2 = map(int, best.xyxy[0].tolist())
                    mx, my = (x1 + x2) // 2, (y1 + y2) // 2
                    cw, ch = max(2, (x2 - x1) // 4), max(2, (y2 - y1) // 4)
                    roi = depth[max(0, my - ch):my + ch, max(0, mx - cw):mx + cw]
                    vals = roi[roi > 0]
                    if vals.size > 20:
                        z = float(np.median(vals)) / 1000.0
                        if 0.2 < z < 20.0:
                            hit = True
                            self.fwd.append(z)
                            self.rgt.append((mx - cx) * z / fx)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    lbl = f'{self.target} {float(best.conf[0]):.2f}'
                    if hit:
                        lbl += f'  fwd {self.fwd[-1]:.2f}m  right {self.rgt[-1]:+.2f}m'
                    cv2.putText(frame, lbl, (x1, max(15, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                self.seen.append(1 if hit else 0)
                self.publish(frame)

                if self.save_dir and hit and time.time() - self.last_save > self.save_period:
                    self.last_save = time.time()
                    cv2.imwrite(os.path.join(
                        self.save_dir, f'pole_{int(time.time()*1000)}.jpg'), raw)

                ok, buf = cv2.imencode('.jpg', frame)
                if ok:
                    with jlock:
                        frame_jpg = buf.tobytes()

    def publish(self, frame):
        stable = sum(self.seen) >= self.hits
        self.pub_vis.publish(Bool(data=stable))
        txt = 'SEARCHING'
        if stable and self.fwd:
            f = float(np.median(self.fwd))
            r = float(np.median(self.rgt))
            m = PointStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.point.x, m.point.y, m.point.z = f, r, 0.0
            self.pub_pos.publish(m)
            txt = f'LOCK  fwd {f:.2f}m  right {r:+.2f}m'
            if time.time() - self.last_report > 1.0:
                self.last_report = time.time()
                self.get_logger().info(txt)
        elif not stable:
            self.fwd.clear()
            self.rgt.clear()
        if stable != self.visible:
            self.visible = stable
            self.get_logger().info(f'pole visible: {stable}')
        cv2.putText(frame, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if stable else (0, 200, 255), 2)


def main():
    rclpy.init()
    node = PoleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
