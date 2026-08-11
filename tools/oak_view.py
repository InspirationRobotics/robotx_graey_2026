#!/usr/bin/env python3
"""OAK-D viewer -> http://<jetson-ip>:8080   (Ctrl-C to stop)

    python3 tools/oak_view.py            plain RGB, no depth
    python3 tools/oak_view.py --yolo     + pole detection with range/bearing

Only one process may hold the camera, so stop pole_tracker first.
Needs the workspace sourced:  source /root/robotx_ws/install/setup.bash
"""
import sys
import time

import cv2
import depthai as dai

from robotx_graey_2026.api.vision import camera

MODEL = '/root/robotx_ws/src/robotx_graey_2026/models/pole_daynight_640x360_v2.pt'
CONF = 0.4


def annotate(frame, model, depth, fx, cx):
    res = model.predict(frame, conf=CONF, verbose=False, device=0)[0]
    for b in res.boxes:
        box = list(map(int, b.xyxy[0].tolist()))
        cls = res.names[int(b.cls[0])]
        col = (0, 0, 255) if cls == 'red' else (255, 255, 255)
        cv2.rectangle(frame, tuple(box[:2]), tuple(box[2:]), col, 2)
        label = f'{cls} {float(b.conf[0]):.2f}'
        rb = camera.range_bearing(depth, box, fx, cx) if depth is not None else None
        if rb:
            label += f'  fwd {rb[0]:.2f}m  right {rb[1]:+.2f}m'
        cv2.putText(frame, label, (box[0], max(15, box[1] - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)


def main():
    use_yolo = '--yolo' in sys.argv
    model = None
    if use_yolo:
        from ultralytics import YOLO                # slow import - skip when unused
        model = YOLO(MODEL)

    camera.serve_mjpeg()
    print('serving on http://0.0.0.0:8080')
    with dai.Device(camera.build_pipeline(fps=10 if use_yolo else 15,
                                          with_depth=use_yolo)) as dev:
        print('USB speed:', dev.getUsbSpeed())
        fx, cx = camera.intrinsics(dev)
        qr = dev.getOutputQueue('rgb', 4, False)
        qd = dev.getOutputQueue('depth', 4, False) if use_yolo else None
        depth = None
        while True:
            if qd is not None:
                d = qd.tryGet()
                if d is not None:
                    depth = d.getFrame()
            pkt = qr.tryGet()
            if pkt is None:
                time.sleep(0.005)                   # never block - Ctrl-C cannot
                continue                            # interrupt a depthai call
            frame = pkt.getCvFrame()
            if model is not None:
                annotate(frame, model, depth, fx, cx)
            camera.publish_frame(frame)


if __name__ == '__main__':
    main()
