#!/usr/bin/env python3
"""Plain OAK-D viewer -> http://<jetson-ip>:8080  (Ctrl-C to stop).
640x400 / 15 fps while the camera negotiates USB2.
Only one process may hold the camera at a time."""
import threading
import time
import cv2
import depthai as dai
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

frame_jpg = None
lock = threading.Lock()

PAGE = b"""<html><head><title>Graey OAK-D</title></head>
<body style="margin:0;background:#111;text-align:center">
<img src="/stream" style="max-width:100%">
</body></html>"""


def grab():
    global frame_jpg
    p = dai.Pipeline()
    cam = p.createColorCamera()
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    cam.setPreviewSize(640, 400)
    cam.setInterleaved(False)
    cam.setFps(15)
    xout = p.createXLinkOut()
    xout.setStreamName('rgb')
    cam.preview.link(xout.input)

    with dai.Device(p) as dev:
        print('USB speed:', dev.getUsbSpeed())
        q = dev.getOutputQueue('rgb', 4, False)
        while True:
            ok, buf = cv2.imencode('.jpg', q.get().getCvFrame())
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
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
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
                time.sleep(0.06)
        except (ConnectionResetError, BrokenPipeError):
            pass

    def log_message(self, *a):
        pass


if __name__ == '__main__':
    threading.Thread(target=grab, daemon=True).start()
    print('serving on http://0.0.0.0:8080')
    ThreadingHTTPServer(('0.0.0.0', 8080), Handler).serve_forever()
