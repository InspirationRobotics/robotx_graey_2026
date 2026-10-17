#!/usr/bin/env python3
"""SIM ONLY: the bottom camera, drawn from the simulated world (Ruth, Oct 16).

Serves http://0.0.0.0:8080/stream the way tools/cam_pair.py does on Graey: one
MJPEG picture, DOWN camera on the left, FRONT on the right, 640x360 each, so the
mission's box counter (api/vision/led_boxes.py) reads it unchanged.

The down picture is a pinhole view straight down from where the simulator says
the camera really is (SIM_STATE: position and full attitude; the camera sits
CAM_FWD_M ahead of the DVL, i.e. 0.2 m behind the sonar). It draws only what the
box counter needs: a dark floor with a little noise, the pipe as a grey line, and
each light box as a dark body with its LED glowing in its colour
(SIM_BOX_COLOURS, one per box in order along the pipe). No murk, glare or
reflections: this tests the mission's logic, not whether colour detection works
in real water (the Oct 15 footage does that). The FRONT half is left dark.
"""
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

W, H = 640, 360
HFOV = math.radians(float(os.environ.get("SIM_CAM_HFOV", "55")))   # in water (narrower than in air)
F = (W / 2) / math.tan(HFOV / 2)
CAM_FWD_M = 0.22
PIPE_D = 0.0889
BOX_L, BOX_W, BOX_H = 0.20, 0.15, 0.15
LED_R = 0.07                        # glow radius at the box top, m (real LED boxes filled 5-38% of the picture)
BGR = {"red": (40, 40, 255), "green": (60, 255, 60), "blue": (255, 120, 40), "yellow": (40, 230, 255)}

_frame, _lock = None, threading.Lock()


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b'<html><body style="margin:0;background:#000"><img src="/stream" style="width:100%"></body></html>')
            return
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _lock:
                    f = _frame
                if f is not None:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(f)).encode() + b"\r\n\r\n" + f + b"\r\n")
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *a):
        pass


class SimCamera:
    """get_sonar() returns the current SimSonar (its SIM_STATE and world)."""

    def __init__(self, get_sonar, boxes, centre_line, fps=10.0, port=8080):
        self.get_sonar, self.fps, self.port = get_sonar, fps, port
        cols = os.environ.get("SIM_BOX_COLOURS", "red green blue yellow").split()
        self.boxes = [(np.asarray(c, float), np.asarray(u, float), cols[i % len(cols)]) for i, (c, u) in enumerate(boxes)]
        pts = []
        cl = np.asarray(centre_line, float)
        for a, b in zip(cl, cl[1:]):
            n = max(2, int(np.linalg.norm(b - a) / 0.02))
            pts.append(a + (b - a) * np.linspace(0, 1, n)[:, None])
        self.pipe = np.vstack(pts)
        self.rng = np.random.default_rng(3)

    def start(self):
        threading.Thread(target=lambda: ThreadingHTTPServer(("0.0.0.0", self.port), _Handler).serve_forever(),
                         daemon=True).start()
        threading.Thread(target=self._loop, daemon=True).start()
        return self

    def _pose(self):
        s = self.get_sonar()
        m = s._state if s is not None else None
        if m is None:
            return None
        lat0, lon0 = s.home
        n = math.radians(m.lat_int / 1e7 - lat0) * 6378137.0
        e = math.radians(m.lon_int / 1e7 - lon0) * 6378137.0 * math.cos(math.radians(lat0))
        w, x, y, z = m.q1, m.q2, m.q3, m.q4
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        cam = np.array([n, e, -m.alt]) + r @ np.array([CAM_FWD_M, 0.0, 0.0])
        return cam, r

    @staticmethod
    def _project(P, cam, r):
        """NED points -> pixel u, v and distance down the camera's axis (body z)."""
        b = (r.T @ (np.atleast_2d(P) - cam).T).T          # body: x forward, y right, z down
        z = b[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = W / 2 + F * b[:, 1] / z
            v = H / 2 - F * b[:, 0] / z                   # forward is up in the picture
        return u, v, z

    def render(self):
        img = np.full((H, W, 3), (22, 18, 10), np.uint8)
        noise = self.rng.normal(0, 4, (H, W, 1)).astype(np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        pose = self._pose()
        if pose is not None:
            cam, r = pose
            u, v, z = self._project(self.pipe, cam, r)
            ok = z > 0.15
            for i in range(len(u) - 1):
                if ok[i] and ok[i + 1]:
                    t = max(1, int(PIPE_D * F / z[i]))
                    cv2.line(img, (int(u[i]), int(v[i])), (int(u[i + 1]), int(v[i + 1])), (70, 76, 72), t)
            for c, d, colour in self.boxes:
                across = np.cross(d, (0.0, 0.0, 1.0))
                top = c - np.array([0.0, 0.0, BOX_H / 2])         # NED: up is -z
                corners = [top + sx * BOX_L / 2 * d + sy * BOX_W / 2 * across
                           for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
                cu, cv_, cz = self._project(np.array(corners), cam, r)
                if np.any(cz <= 0.1):
                    continue
                cv2.fillPoly(img, [np.stack([cu, cv_], 1).astype(np.int32)], (48, 50, 50))
                lu, lv, lz = self._project(top, cam, r)
                rad = int(LED_R * F / lz[0])
                glow = np.zeros_like(img)
                cv2.circle(glow, (int(lu[0]), int(lv[0])), int(rad * 2.2), BGR[colour], -1)
                glow = cv2.GaussianBlur(glow, (0, 0), max(1.0, rad * 0.5))
                img = cv2.max(img, glow)
                cv2.circle(img, (int(lu[0]), int(lv[0])), max(1, rad // 3), (235, 235, 235), -1)
        img = cv2.GaussianBlur(img, (3, 3), 0)
        cv2.putText(img, "DOWN (sim)", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        front = np.full((H, W, 3), (8, 8, 8), np.uint8)
        cv2.putText(front, "FRONT (not simulated)", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1)
        return cv2.hconcat([img, front])

    def _loop(self):
        global _frame
        while True:
            t = time.monotonic()
            try:
                ok, buf = cv2.imencode(".jpg", self.render(), [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with _lock:
                        _frame = buf.tobytes()
            except Exception as ex:                       # keep the sonar sim alive whatever happens here
                print(f"sim_camera: {ex}", flush=True)
            time.sleep(max(0.0, 1.0 / self.fps - (time.monotonic() - t)))
