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

The repair magnet (Ruth, Oct 22) is drawn where it shows in the real picture
(led_boxes.MAGNET_PX), MAG_DOWN_M under the lens on that pixel's line of sight. A RED
light whose LED comes within REPAIR_M of it turns GREEN for 5 s, then flashes BLUE,
RED, GREEN (1 s on, 1 s off) for 30 s, then stays GREEN, as the handbook's light does.

SIM_CAM_LOOK=real (Ruth, Oct 27): closer to the real down picture, for camera_follow's pipe finder
(api/vision/pipe_line.py), which never passes the dim grey line above: an olive floor with blotches
fixed to the floor, the white pipe, each box a clear case with pale edges, far things fading into the
water. Still no glare, caustics or murk. SIM_DECOY="off from to" adds a bright pipe-like stripe off m
beside the pipe, from..to m along it, and hides the pipe there: a confident camera line that is NOT the
pipe, for camera_follow's disagreement rule.
"""
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from robotx_graey_2026.api.vision.led_boxes import MAGNET_PX

W, H = 640, 360
HFOV = math.radians(float(os.environ.get("SIM_CAM_HFOV", "55")))   # in water (narrower than in air)
F = (W / 2) / math.tan(HFOV / 2)
CAM_FWD_M = 0.22
PIPE_D = 0.0889
BOX_L, BOX_W, BOX_H = 0.20, 0.15, 0.15
LED_R = 0.07                        # glow radius at the box top, m (real LED boxes filled 5-38% of the picture)
BGR = {"red": (40, 40, 255), "green": (60, 255, 60), "blue": (255, 120, 40), "yellow": (40, 230, 255)}
MAG_DOWN_M, MAG_R_PX = 0.149, 49    # the magnet under the lens (Ruth's ruler) and its size in the picture
REPAIR_M = 0.04                     # the LED this close to the magnet: repaired
# the magnet in the body frame, on the line of sight of its pixel
MAG_BODY = np.array([-(MAGNET_PX[1] - H / 2) / F, (MAGNET_PX[0] - W / 2) / F, 1.0]) * MAG_DOWN_M
LOOK = os.environ.get("SIM_CAM_LOOK", "cartoon")
WATER = np.array([96.0, 118.0, 92.0])       # BGR far things fade to (the Oct 24 lake's olive)
FLOOR = np.array([74.0, 96.0, 80.0])        # the floor, blotches +-BLOTCH on it
BLOTCH = np.array([20.0, 22.0, 18.0])
PIPE_C = np.array([215.0, 228.0, 222.0])    # the white pipe
FADE_M = 4.0                                # half the contrast gone this far from the lens
DECOY = [float(v) for v in os.environ.get("SIM_DECOY", "").split()] or None

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
        self.hidden = np.zeros(len(self.pipe), bool)
        self.decoy = None
        if DECOY:                           # beside the pipe, at its height, the pipe hidden there
            off, s0, s1 = DECOY
            s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.pipe, axis=0), axis=1))])
            tan = np.gradient(self.pipe[:, :2], axis=0)
            side = np.column_stack([-tan[:, 1], tan[:, 0], np.zeros(len(tan))])
            side /= np.maximum(np.linalg.norm(side, axis=1, keepdims=True), 1e-9)
            self.hidden = (s >= s0) & (s <= s1)
            self.decoy = self.pipe[self.hidden] + off * side[self.hidden]
            print(f"sim_camera: decoy {off} m beside the pipe from {s0} to {s1} m along it", flush=True)
        self.rng = np.random.default_rng(3)
        self.repaired = {}                  # box index -> when the magnet reached its RED light

    def _colour(self, i, colour):
        """The colour a light shows now (None: off between flashes)."""
        if i not in self.repaired:
            return colour
        dt = time.monotonic() - self.repaired[i]
        if dt < 5.0 or dt >= 35.0:
            return "green"
        k, on = divmod(dt - 5.0, 2.0)
        return ("blue", "red", "green")[int(k) % 3] if on < 1.0 else None

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

    def _floor(self, cam, r):
        """SIM_CAM_LOOK=real: the floor under each pixel (worked out on a 160x90 grid), its blotches fixed to
        the floor so they slide past as the sub moves, fading into the water with distance."""
        s = self.get_sonar()
        u, v = np.meshgrid((np.arange(160) + 0.5) * 4, (np.arange(90) + 0.5) * 4)
        w = np.stack([-(v - H / 2) / F, (u - W / 2) / F, np.ones_like(u)], -1) @ r.T     # NED rays
        t = (s.depth - cam[2]) / np.maximum(w[..., 2], 1e-3)
        n, e = cam[0] + t * w[..., 0], cam[1] + t * w[..., 1]
        tex = (0.6 * np.sin(n * 7.1 + 1.3 * np.sin(e * 3.3)) * np.sin(e * 6.3 + 1.1 * np.sin(n * 2.9))
               + 0.4 * np.sin(n * 2.3 + e * 1.7) * np.sin(e * 2.9 - n * 1.1))
        tex = cv2.resize(tex.astype(np.float32), (W, H))[..., None]
        k = cv2.resize(np.exp(-t * math.log(2) / FADE_M).astype(np.float32), (W, H))[..., None]
        return WATER + (FLOOR + tex * BLOTCH - WATER) * k

    @staticmethod
    def _faded(c, z):
        return tuple(float(x) for x in WATER + (c - WATER) * math.exp(-max(z, 0.0) * math.log(2) / FADE_M))

    def render(self):
        real = LOOK == "real"
        pose = self._pose()
        if real and pose is not None:
            img = self._floor(*pose)
            img = np.clip(img + self.rng.normal(0, 3, (H, W, 1)), 0, 255).astype(np.uint8)
        else:
            img = np.full((H, W, 3), (22, 18, 10), np.uint8)
            noise = self.rng.normal(0, 4, (H, W, 1)).astype(np.int16)
            img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        if pose is not None:
            cam, r = pose
            u, v, z = self._project(self.pipe, cam, r)
            ok = (z > 0.15) & ~self.hidden
            for i in range(len(u) - 1):
                if ok[i] and ok[i + 1]:
                    t = max(1, int(PIPE_D * F / z[i]))
                    cv2.line(img, (int(u[i]), int(v[i])), (int(u[i + 1]), int(v[i + 1])),
                             self._faded(PIPE_C, z[i]) if real else (70, 76, 72), t, cv2.LINE_AA if real else cv2.LINE_8)
            if self.decoy is not None:
                du, dv, dz = self._project(self.decoy, cam, r)
                for i in range(len(du) - 1):
                    if dz[i] > 0.15 and dz[i + 1] > 0.15:
                        cv2.line(img, (int(du[i]), int(dv[i])), (int(du[i + 1]), int(dv[i + 1])),
                                 self._faded(PIPE_C, dz[i]), max(1, int(PIPE_D * F / dz[i])), cv2.LINE_AA)   # just like it
            magnet = cam + r @ MAG_BODY
            for i, (c, d, colour) in enumerate(self.boxes):
                across = np.cross(d, (0.0, 0.0, 1.0))
                top = c - np.array([0.0, 0.0, BOX_H / 2])         # NED: up is -z
                if colour == "red" and i not in self.repaired and np.linalg.norm(magnet - top) < REPAIR_M:
                    self.repaired[i] = time.monotonic()
                    print(f"sim_camera: the magnet reached light {i + 1} - GREEN", flush=True)
                colour = self._colour(i, colour)
                corners = [top + sx * BOX_L / 2 * d + sy * BOX_W / 2 * across
                           for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
                cu, cv_, cz = self._project(np.array(corners), cam, r)
                if np.any(cz <= 0.1):
                    continue
                poly = np.stack([cu, cv_], 1).astype(np.int32)
                if real:                        # a clear case: the floor shows through darker, its edges pale
                    case = img.copy()
                    cv2.fillPoly(case, [poly], (60, 70, 66))
                    img = cv2.addWeighted(case, 0.45, img, 0.55, 0)
                    cv2.polylines(img, [poly], True, self._faded(np.array([175.0, 190.0, 185.0]), cz[0]), 2, cv2.LINE_AA)
                else:
                    cv2.fillPoly(img, [poly], (48, 50, 50))
                if colour is None:
                    continue
                lu, lv, lz = self._project(top, cam, r)
                rad = int(LED_R * F / lz[0])
                glow = np.zeros_like(img)
                cv2.circle(glow, (int(lu[0]), int(lv[0])), int(rad * 2.2), BGR[colour], -1)
                glow = cv2.GaussianBlur(glow, (0, 0), max(1.0, rad * 0.5))
                img = cv2.max(img, glow)
                cv2.circle(img, (int(lu[0]), int(lv[0])), max(1, int(rad * 0.8)), (252, 252, 252), -1)   # washed-out core
            cv2.circle(img, (int(MAGNET_PX[0]), int(MAGNET_PX[1])), MAG_R_PX, (28, 28, 28), -1)   # the magnet
            cv2.circle(img, (int(MAGNET_PX[0]), int(MAGNET_PX[1])), MAG_R_PX, (140, 140, 140), 2)
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
