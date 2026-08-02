#!/usr/bin/env python3
"""Red-pole tracker: OAK-D + YOLO11 -> filtered body-frame position.

Publishes:
  /graey/pole/visible   std_msgs/Bool               stable detection?
  /graey/pole           geometry_msgs/PointStamped  x=forward, y=right, z=0 (m)

Detection flicker is absorbed by requiring the pole in HITS of the last WINDOW
frames, then median-filtering range and bearing over that history.

Debug view at http://<jetson-ip>:8080. With save_dir set, writes a frame every
save_period seconds while detecting - that pile is the retraining set from our
own water and camera.
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
from ultralytics import YOLO

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.vision import camera


class PoleTracker(Node):
    def __init__(self):
        super().__init__('pole_tracker')
        p = self.declare_parameter
        p('model', '/root/robotx_ws/src/robotx_graey_2026/models/bestPoles724.pt')
        p('target_class', 'red')
        p('conf', 0.4)
        p('window', 10)
        p('hits_needed', 5)                         # N-of-WINDOW frames to call it stable
        p('save_dir', '')
        p('save_period', 0.5)

        g = self.get_parameter
        self.model_path = g('model').value
        self.target = g('target_class').value
        self.conf = g('conf').value
        self.hits = g('hits_needed').value
        self.save_dir = g('save_dir').value
        self.save_period = g('save_period').value
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

        self.pub_vis = self.create_publisher(Bool, '/graey/pole/visible', 10)
        self.pub_pos = self.create_publisher(PointStamped, '/graey/pole', 10)

        window = g('window').value
        self.seen = deque(maxlen=window)
        self.fwd = deque(maxlen=window)
        self.rgt = deque(maxlen=window)
        self.last_save = 0.0
        self.last_report = 0.0
        self.visible = False

        self.stop = threading.Event()
        self.cam_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.cam_thread.start()
        camera.serve_mjpeg()
        self.get_logger().info('pole_tracker starting; debug view on :8080')

    def destroy_node(self):
        """Let the camera thread finish its current inference and exit on its own.

        Killing it mid-predict force-unwinds a thread parked inside torch's C++,
        which ends in std::terminate and a 30 s hang on Ctrl-C.
        """
        self.stop.set()
        self.cam_thread.join(timeout=5.0)
        super().destroy_node()

    def best_box(self, res):
        """Highest-confidence box of the target class, or None."""
        best = None
        for b in res.boxes:
            if res.names[int(b.cls[0])] != self.target:
                continue
            if best is None or float(b.conf[0]) > float(best.conf[0]):
                best = b
        return best

    def camera_loop(self):
        model = YOLO(self.model_path)
        with dai.Device(camera.build_pipeline()) as dev:
            self.get_logger().info(f'OAK-D USB speed {dev.getUsbSpeed()}')
            fx, cx = camera.intrinsics(dev)
            qr = dev.getOutputQueue('rgb', 4, False)
            qd = dev.getOutputQueue('depth', 4, False)
            depth = None
            while rclpy.ok() and not self.stop.is_set():
                d = qd.tryGet()
                if d is not None:
                    depth = d.getFrame()
                pkt = qr.tryGet()
                if pkt is None:
                    time.sleep(0.005)               # never block - a blocking depthai
                    continue                        # call cannot be interrupted
                frame = pkt.getCvFrame()
                raw = frame.copy()                  # the saved copy carries no overlay
                best = self.best_box(
                    model.predict(frame, conf=self.conf, verbose=False, device=0)[0])

                hit = False
                if best is not None:
                    box = list(map(int, best.xyxy[0].tolist()))
                    rb = camera.range_bearing(depth, box, fx, cx) if depth is not None else None
                    if rb:
                        hit = True
                        self.fwd.append(rb[0])
                        self.rgt.append(rb[1])
                    cv2.rectangle(frame, tuple(box[:2]), tuple(box[2:]), (0, 0, 255), 2)
                    lbl = f'{self.target} {float(best.conf[0]):.2f}'
                    if hit:
                        lbl += f'  fwd {rb[0]:.2f}m  right {rb[1]:+.2f}m'
                    cv2.putText(frame, lbl, (box[0], max(15, box[1] - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                self.seen.append(1 if hit else 0)
                self.publish(frame)

                if self.save_dir and hit and time.time() - self.last_save > self.save_period:
                    self.last_save = time.time()
                    cv2.imwrite(os.path.join(
                        self.save_dir, f'pole_{int(time.time() * 1000)}.jpg'), raw)
                camera.publish_frame(frame)

    def publish(self, frame):
        stable = sum(self.seen) >= self.hits
        self.pub_vis.publish(Bool(data=stable))
        txt = 'SEARCHING'
        if stable and self.fwd:
            f, r = float(np.median(self.fwd)), float(np.median(self.rgt))
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
            self.fwd.clear()                        # do not median across a lost lock
            self.rgt.clear()
        if stable != self.visible:
            self.visible = stable
            self.get_logger().info(f'pole visible: {stable}')
        cv2.putText(frame, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if stable else (0, 200, 255), 2)


def main():
    run(PoleTracker)
