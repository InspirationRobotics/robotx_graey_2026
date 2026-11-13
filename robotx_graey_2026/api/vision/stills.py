"""Stills for computer-vision data: each camera's own frames, saved as they arrive.

Chris, 12 Nov 2026. The old camera recording copied the :8080 stream: both cameras glued
into one picture with a name drawn across each. For training data he wants what the Jetson
gets from each camera, untouched - so this saves the frames tools/cam_pair.py holds, BEFORE
it labels and joins them, at whatever size they arrive (640x360 today; change the pipeline
and the stills follow).

It runs inside cam_pair.py, the only program that holds the cameras, and is switched by a
small file the radio_status node keeps fresh (the operator's Record button on the laptop
page). That file also carries the depth, the DVL altitude and the heading of that moment, so
every still's line in frames.jsonl says where the sub was.

    request  /dev/shm/graey_stills.json          radio_status -> cam_pair, about 5 a second
    status   /dev/shm/graey_stills_status.json   cam_pair -> radio_status, once a second

/dev/shm is memory, so neither file wears the SD card. Both programs run in the graey
container, and time.monotonic() is the kernel's clock, the same in both.

offer() NEVER raises: the missions steer by this program's stream, and a full disk or a bad
request must cost stills, not the stream.
"""
import json
import os
import time

REQUEST = '/dev/shm/graey_stills.json'
STATUS = '/dev/shm/graey_stills_status.json'
READ_S = 0.3            # how often the request file is looked at
STALE_S = 3.0           # a request not refreshed for this long is off: radio_status died or was stopped
STATUS_S = 1.0          # how often the status file is written
QUALITY = 95            # JPEG quality of a still (cv2's own default, and what the stream uses)


def write_json(path, data):
    """Replace a small JSON file in one step, so a reader never sees half of it."""
    tmp = f'{path}.{os.getpid()}.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f)
    os.replace(tmp, path)


def read_json(path):
    try:
        with open(path) as f:
            data = json.load(f)
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


class StillWriter:
    def __init__(self, request=REQUEST, status=STATUS, clock=time.monotonic, wall=time.time, encode=None):
        self.request_path, self.status_path = request, status
        self.clock, self.wall, self.encode = clock, wall, encode
        self.req = None                 # the request as last read, or None for off
        self.read_t = self.status_t = -1e9
        self.seen = {}                  # camera -> the frame object last offered
        self.fresh_t = {}               # camera -> when it last gave a NEW frame
        self.kept = {}                  # camera -> the frame object last saved (never the same one twice)
        self.save_t = {}                # camera -> when its last still was saved
        self.count = {}                 # camera -> the newest file number in the current folder
        self.dir = None
        self.error = None

    def _encode(self, frame):
        if self.encode is None:         # here, not at import: this file is also read on a laptop without cv2
            import cv2
            self.encode = lambda f: cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
        ok, buf = self.encode(frame)
        if not ok:
            raise ValueError('the JPEG encoder refused the frame')
        return bytes(buf)

    def _read_request(self, now):
        req = read_json(self.request_path)
        on = bool(req and req.get('on') and req.get('dir')
                  and 0 <= now - float(req.get('t', -1e9)) <= STALE_S)
        self.req = req if on else None
        if on and req['dir'] != self.dir:           # a new session folder: number on from what it holds, so
            os.makedirs(req['dir'], exist_ok=True)  # this program restarting mid-session overwrites nothing
            self.dir, self.count = req['dir'], {}
            for file in os.listdir(self.dir):
                name, _, number = file[:-4].rpartition('_')
                if file.endswith('.jpg') and number.isdigit():
                    self.count[name] = max(self.count.get(name, 0), int(number))

    def _save(self, name, frame, now):
        n = self.count.get(name, 0) + 1
        file = f'{name}_{n:06d}.jpg'
        with open(os.path.join(self.dir, file), 'wb') as f:
            f.write(self._encode(frame))
        row = dict(file=file, camera=name, t=round(now, 3), wall=round(self.wall(), 3),
                   width=int(frame.shape[1]), height=int(frame.shape[0]))
        row.update(self.req.get('nav') or {})       # depth, altitude, heading ... as radio_status last had them
        with open(os.path.join(self.dir, 'frames.jsonl'), 'a') as f:
            f.write(json.dumps(row) + '\n')
        self.count[name] = n

    def offer(self, names, frames):
        """The newest frame of every camera (None for one that has not sent any yet). Call it
        whenever any camera has a new one."""
        try:
            now = self.clock()
            for name, frame in zip(names, frames):
                if frame is not None and frame is not self.seen.get(name):
                    self.seen[name], self.fresh_t[name] = frame, now
            if now - self.read_t >= READ_S:
                self.read_t = now
                self._read_request(now)
            if self.req is not None:
                every = 1.0 / max(0.1, float(self.req.get('fps', 3.0)))
                for name, frame in zip(names, frames):          # each camera on its own clock: they
                    if (frame is not None and frame is not self.kept.get(name)      # do not send together
                            and now - self.save_t.get(name, -1e9) >= every):
                        self.save_t[name] = now                 # before the save: a failing disk is retried
                        self._save(name, frame, now)            # once an interval, not every loop
                        self.kept[name] = frame
                        self.error = None
            if now - self.status_t >= STATUS_S:
                self.status_t = now
                write_json(self.status_path, dict(
                    t=now, cameras={n: round(now - t, 2) for n, t in self.fresh_t.items()},
                    saving=self.req is not None, dir=self.dir, in_dir=sum(self.count.values()),
                    error=self.error))
        except Exception as ex:                     # never the stream's problem (see the header)
            self.error = f'{type(ex).__name__}: {ex}'
