#!/usr/bin/env python3
"""Both OAK cameras side by side -> http://<jetson-ip>:8080   (Ctrl-C to stop)

    python3 tools/cam_pair.py

Normally started by the launcher (GUI Camera tab, or POST :8096/camera/start). Only one
process may hold a camera: to run it by hand, stop the launcher's copy first
(GUI Camera stop, or curl -X POST http://127.0.0.1:8096/camera/stop).

Both frames are composited into ONE image rather than served as two streams,
because camera.serve_mjpeg() holds a single global frame. hconcat is free next to
JPEG encoding and it keeps the shared module untouched.

USB SPEED IS PINNED TO HIGH DELIBERATELY. The downward camera will not boot at
SUPER on its current port - depthai flashes it, then waits for it to re-enumerate
on a SuperSpeed link that never trains and gives up with X_LINK_DEVICE_NOT_FOUND.
HIGH carries 640x360 RGB from both with room to spare (~83 Mbps each against ~300
practical), so nothing is lost until the port is sorted out.
"""
import time

import cv2
import depthai as dai

from robotx_graey_2026.api.vision import camera

SPEED = dai.UsbSpeed.HIGH
FPS = 15

# Stable hardware ids (api/vision/camera.py), so left/right never swaps between runs.
# An unknown camera still shows up, labelled with the tail of its MxId.
NAMES = camera.CAMERAS


def label(frame, text):
    for colour, weight in (((0, 0, 0), 4), ((255, 255, 255), 1)):
        cv2.putText(frame, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    colour, weight)     # black halo first - readable on any scene
    return frame


def open_all():
    devs = dai.Device.getAllAvailableDevices()
    if not devs:
        print('no OAK devices - check cabling, and that nothing else holds the cameras')
        return []
    devs.sort(key=lambda d: NAMES.get(d.getMxId(), 'zz' + d.getMxId()))

    opened = []
    for info in devs:
        mxid = info.getMxId()
        name = camera.camera_name(mxid)
        try:
            dev = dai.Device(
                camera.build_pipeline(fps=FPS, with_depth=False), info, SPEED)
        except Exception as e:
            print(f'{name} ({mxid}) FAILED to open: {e}')
            continue                    # one dead camera must not hide the other
        print(f'{name} ({mxid}) open at {dev.getUsbSpeed().name}')
        opened.append((name, dev, dev.getOutputQueue('rgb', 2, False)))
    return opened


def main():
    opened = open_all()
    if not opened:
        return

    camera.serve_mjpeg()
    print(f'serving {len(opened)} camera(s) on http://0.0.0.0:8080')

    last = [None] * len(opened)         # hold the newest frame per camera, so one
    sent = 0.0                          # camera stalling never freezes the other
    try:
        while True:
            fresh = False
            for i, (_, _, q) in enumerate(opened):
                pkt = q.tryGet()
                if pkt is not None:
                    last[i] = pkt.getCvFrame()
                    fresh = True
            # Encoding is the cost. Re-encoding the same frames, encoding faster than the
            # cameras run, or encoding with nobody reading /stream (GUI view or recorder)
            # shows nobody anything new (Crusader: pictures only while someone watches).
            now = time.monotonic()
            if (any(f is None for f in last) or not fresh or now - sent < 1.0 / FPS
                    or not camera.viewing()):
                time.sleep(0.005)       # never block - Ctrl-C cannot interrupt
                continue                # a depthai call
            sent = now
            camera.publish_frame(cv2.hconcat(
                [label(f.copy(), n) for f, (n, _, _) in zip(last, opened)]))
            time.sleep(0.005)
    finally:
        # Closing explicitly matters here: this camera has twice left a crash dump
        # behind when a HIGH-speed session ended without a clean teardown.
        for _, dev, _ in opened:
            dev.close()


if __name__ == '__main__':
    main()
