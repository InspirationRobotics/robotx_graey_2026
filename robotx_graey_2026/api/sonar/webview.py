#!/usr/bin/env python3
"""Serve the radar view as MJPEG, so it can be watched from a laptop browser.

The Jetson has no screen, so cv2.imshow has nothing to draw on. This is the
same trick vision/camera.py already uses for the OAK-D view, and it is copied
rather than imported because that module pulls in depthai at import time - the
sonar package deliberately depends on nothing heavier than numpy and opencv,
which is what lets it run on a laptop with no sub attached.

Default port is 8081, NOT 8080, so this and the camera view can be open in two
browser tabs at once.

    from . import webview
    webview.serve(8081)
    ...
    webview.publish(render(perception, state="SEARCH"))
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

_frame = None
_lock = threading.Lock()

# Fills the window, keeping the aspect ratio. The frame is rendered large to
# begin with, so on a laptop this is at or near native size rather than a blurry
# stretch of a small image.
PAGE = b"""<html><head><title>Graey sonar</title></head>
<body style="margin:0;background:#161412;height:100vh;display:flex;
align-items:center;justify-content:center">
<img src="/stream" style="width:100vw;height:100vh;object-fit:contain"></body></html>"""

# OpenCV's default is already 95. 98 trims the last of the ringing around thin
# text and one-pixel rings, and on a tether the extra bytes cost nothing. Most of
# the sharpness gain came from rendering larger with heavier fonts, not this.
_QUALITY = [cv2.IMWRITE_JPEG_QUALITY, 98]


def publish(frame):
    """Encode a BGR image and hand it to the server."""
    global _frame
    ok, buf = cv2.imencode('.jpg', frame, _QUALITY)
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
                # A sweep takes seconds, not milliseconds, so there is no point
                # spinning fast here - this only controls how quickly a newly
                # opened browser tab picks up the frame already rendered.
                time.sleep(0.2)
        except (ConnectionResetError, BrokenPipeError):
            pass

    def log_message(self, *a):
        pass                                        # silence per-request logging


def serve(port=8081):
    threading.Thread(target=lambda: ThreadingHTTPServer(
        ('0.0.0.0', port), _Handler).serve_forever(), daemon=True).start()
