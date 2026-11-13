"""The operator's two Record switches: camera stills and sonar sweeps, owned by the sub.

Chris, 12 Nov 2026. The laptop page asks for a recorder ON or OFF (over the radio, or WiFi
when there is some); THIS keeps it that way, so a lost link or a dive changes nothing. It
opens no camera and no sonar: it asks the programs that hold them, like the GUI's Record tab.

  camera   tools/cam_pair.py holds both OAKs (started through the launcher on 8096). While
           the camera switch is on, cam_pair saves each camera's own frames as JPEG stills
           (api/vision/stills.py). An older cam_pair that cannot is noticed after a few
           seconds, and the :8080 stream's joined picture is saved instead ("composite"),
           so a run always leaves pictures.
  sonar    tools/sonar_map.py (launcher, then its own Record on 8095): every sweep with the
           pose and DVL altitude at each ping, in sonar_data/rec_<time>_radio-<session>/.

One SESSION folder per stretch of recording, in ~/robotx_ws/recordings/radio_<date>_<time>/:

  session.json        when, what, and the sonar recording's folder name
  nav.jsonl           depth, DVL altitude, heading, position ... five times a second, stamped
                      with time.monotonic() - the clock the stills and the sonar pings carry,
                      so depth and altitude can be looked up for any picture or ping
  camera/             <CAMERA>_<number>.jpg and frames.jsonl (one line per still, with the
                      depth, altitude and heading of that moment)

What is wanted survives this node restarting, but not a reboot: after a power cycle nothing
records until the operator asks again.
"""
import json
import os
import shutil
import threading
import time
import urllib.error
import urllib.request

from robotx_graey_2026.api.vision import stills

LAUNCHER, SONAR, CAMERA = 8096, 8095, 8080      # tools/gui_sonar_launcher.py, the sonar tool's page, cam_pair's stream
MIN_FREE_GIB = 2.0          # nothing is recorded below this: the boot device is the SD card
STILLS_FPS = 3.0            # per camera. 15 a second would be near-identical pictures and 5 GB an hour
NAV_S = 0.2                 # nav.jsonl and the stills request: 5 a second
POLL_S = 2.0                # how often the camera and sonar tools are asked how they are
RETRY_S = 10.0              # between attempts to start a tool that is not running
CAMERA_FRESH_S = 2.5        # a camera whose newest frame is older than this is not delivering
PING_FRESH_S = 30.0         # the sonar is pinging if a new sweep began this recently (a 20 m sweep is slow)
FALLBACK_AFTER_S = 8.0      # stills asked for this long and cam_pair is not saving: save the stream instead
BOOT_ID = '/proc/sys/kernel/random/boot_id'


def http(port, path, body=None, timeout=3.0):
    """JSON (or text) from one of the Jetson's own tools."""
    with urllib.request.urlopen(urllib.request.Request(f'http://127.0.0.1:{port}{path}', data=body),
                                timeout=timeout) as r:
        data = r.read(8 << 20)
    try:
        return json.loads(data)
    except ValueError:
        return data.decode(errors='replace')


def read_boot_id():
    try:
        with open(BOOT_ID) as f:
            return f.read().strip()
    except OSError:
        return None


def mjpeg_frames(url, stop, timeout=5.0):
    """JPEG after JPEG from an MJPEG stream until stop is set or the stream ends."""
    with urllib.request.urlopen(url, timeout=timeout) as s:
        buf = b''
        while not stop.is_set():
            chunk = s.read(32768)
            if not chunk:
                return
            buf += chunk
            while True:
                a = buf.find(b'\xff\xd8')
                z = buf.find(b'\xff\xd9', a + 2) if a >= 0 else -1
                if z < 0:
                    buf = buf[a:] if a >= 0 else buf[-1:]
                    break
                yield buf[a:z + 2]
                buf = buf[z + 2:]


class RecordControl:
    def __init__(self, root, nav, *, http=http, clock=time.monotonic, wall=time.time,
                 free=lambda path: shutil.disk_usage(path).free, boot_id=read_boot_id,
                 request=stills.REQUEST, status=stills.STATUS, frames=mjpeg_frames, log=print):
        self.root, self.nav, self.http, self.clock, self.wall = root, nav, http, clock, wall
        self.free, self.request_path, self.status_path, self.frames, self.log = free, request, status, frames, log
        self.boot = boot_id()
        self.lock = threading.Lock()
        self.want = {'camera': False, 'sonar': False}
        self.want_path = os.path.join(root, '.radio_record_want.json')
        saved = stills.read_json(self.want_path)
        if saved and self.boot and saved.get('boot') == self.boot:      # this node restarted mid-run
            self.want = {k: bool(saved.get(k)) for k in self.want}
        self.session = None             # the session folder's name while one is open
        self.nav_file = None
        self.t = dict(nav=-1e9, poll=-1e9, cam_start=-1e9, sonar_start=-1e9, sonar_rec=-1e9, free=-1e9,
                      cam_want=None, sweep=-1e9)
        self.free_gib = None
        self.cam = dict(service=False, ages={}, saving=False, count=0, error=None)
        self.sonar = dict(service=False, sweep=None, rec=None, mine=False, error=None, folder=None)
        self.fallback = dict(thread=None, stop=None, count=0, t=-1e9)
        self.said = {}
        self.closed = threading.Event()

    # ------------------------------------------------------------------ what the operator asked
    def set(self, camera=None, sonar=None):
        with self.lock:
            for key, value in (('camera', camera), ('sonar', sonar)):
                if value is not None:
                    self.want[key] = bool(value)
            want = dict(self.want)
        try:
            os.makedirs(self.root, exist_ok=True)
            stills.write_json(self.want_path, dict(want, boot=self.boot))
        except OSError as ex:
            self._say('want', f'record: could not save what was asked for ({ex})')

    def _say(self, key, text):
        """Log a state once, not every pass."""
        if self.said.get(key) != text:
            self.said[key] = text
            if text:
                self.log(text)

    def _ask(self, port, path, body=None):
        try:
            return self.http(port, path, body)
        except (OSError, ValueError, urllib.error.URLError):
            return None

    # ------------------------------------------------------------------ the session folder
    def _open_session(self, want):
        name = time.strftime('radio_%Y%m%d_%H%M%S', time.localtime(self.wall()))
        folder = os.path.join(self.root, name)
        os.makedirs(os.path.join(folder, 'camera'), exist_ok=True)
        self.session = name
        self.cam['count'] = self.fallback['count'] = 0
        self.sonar['folder'] = None
        self.nav_file = open(os.path.join(folder, 'nav.jsonl'), 'a')
        self._write_session(started=time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.wall())),
                            t0_monotonic=round(self.clock(), 3), boot=self.boot, asked=want)
        self.log(f'record: session {name} opened')

    def _write_session(self, **more):
        path = os.path.join(self.root, self.session, 'session.json')
        data = stills.read_json(path) or {}
        data.update(more)
        with open(path, 'w') as f:
            json.dump(data, f, indent=1)

    def _close_session(self):
        self.nav_file.close()
        self.nav_file = None
        self._write_session(ended=time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.wall())),
                            stills=self.cam['count'], sonar_folder=self.sonar['folder'])
        self.log(f'record: session {self.session} closed, {self.cam["count"]} stills')
        self.session = None

    # ------------------------------------------------------------------ camera
    def _camera(self, now, want, poll):
        cam = self.cam
        if poll:
            st = self._ask(LAUNCHER, '/status')
            cam['service'] = bool(st and (st.get('camRunning') or st.get('camUp')))
            if want and not cam['service'] and now - self.t['cam_start'] >= RETRY_S:
                self.t['cam_start'] = now
                reply = self._ask(LAUNCHER, '/camera/start', b'')
                self._say('cam_start', 'record: starting the cameras' if reply else
                          'record: the camera launcher (port 8096) is not answering')
        status = stills.read_json(self.status_path)
        live = bool(status and 0 <= now - float(status.get('t', -1e9)) <= CAMERA_FRESH_S)
        cam['ages'] = {n: a + (now - status['t']) for n, a in (status.get('cameras') or {}).items()} if live else {}
        folder = os.path.join(self.root, self.session, 'camera') if self.session else None
        cam['saving'] = bool(live and want and status.get('saving') and status.get('dir') == folder)
        if poll and folder:                         # counted from the folder, whoever wrote the pictures
            cam['count'] = sum(1 for f in os.scandir(folder) if f.name.endswith('.jpg'))
        cam['error'] = (status or {}).get('error') if live else None
        if want != (self.t['cam_want'] is not None):
            self.t['cam_want'] = now if want else None
        try:
            stills.write_json(self.request_path, dict(on=bool(want), dir=folder, fps=STILLS_FPS, t=now,
                                                      nav=self.nav() if want else None))
        except OSError as ex:
            cam['error'] = f'stills request not written: {ex}'
        waited = want and now - self.t['cam_want'] >= FALLBACK_AFTER_S
        self._fallback(bool(waited and not cam['saving'] and cam['service']), folder)

    def _fallback(self, on, folder):
        """Save the stream's joined picture while cam_pair is not saving stills itself."""
        fb = self.fallback
        running = fb['thread'] is not None and fb['thread'].is_alive()
        if on and not running:
            fb['stop'] = threading.Event()
            fb['thread'] = threading.Thread(target=self._fallback_run, args=(fb['stop'], folder), daemon=True)
            fb['thread'].start()
            self._say('fallback', 'record: cam_pair is not saving stills - saving the stream picture (composite)')
        elif not on and running:
            fb['stop'].set()
            self._say('fallback', '')

    def _fallback_run(self, stop, folder):
        try:
            last = -1e9
            for jpg in self.frames(f'http://127.0.0.1:{CAMERA}/stream', stop):
                now = self.clock()
                if now - last < 1.0 / STILLS_FPS:
                    continue
                last = now
                n = self.fallback['count'] + 1
                file = f'PAIR_{n:06d}.jpg'
                with open(os.path.join(folder, file), 'wb') as f:
                    f.write(jpg)
                with open(os.path.join(folder, 'frames.jsonl'), 'a') as f:
                    f.write(json.dumps(dict(file=file, camera='PAIR', t=round(now, 3), wall=round(self.wall(), 3),
                                            **(self.nav() or {}))) + '\n')
                self.fallback['count'], self.fallback['t'] = n, now
        except (OSError, ValueError, urllib.error.URLError) as ex:
            self.cam['error'] = f'stream not readable: {ex}'

    # ------------------------------------------------------------------ sonar
    def _sonar(self, now, want, poll):
        sonar = self.sonar
        if not poll:
            return
        st = self._ask(LAUNCHER, '/status')
        sonar['service'] = bool(st and st.get('webUp'))
        data = self._ask(SONAR, '/mapdata') if sonar['service'] else None
        data = data if isinstance(data, dict) else None
        sweep = data.get('sweep') if data else None
        if sweep is not None and sweep != sonar['sweep']:
            self.t['sweep'] = now
        sonar['sweep'] = sweep
        sonar['rec'] = data.get('recording') if data else None
        mine = sonar['mine'] = bool(sonar['rec'] and self.session
                                    and self._slug() in str(sonar['rec'].get('name')))
        if mine:
            sonar['folder'] = sonar['rec'].get('name')
        sonar['error'] = None
        if want and not sonar['service']:
            sonar['error'] = 'the sonar tool is not running'
            if now - self.t['sonar_start'] >= RETRY_S:
                self.t['sonar_start'] = now
                reply = self._ask(LAUNCHER, '/start?tool=map', b'')
                msg = reply.get('msg') if isinstance(reply, dict) else None
                self._say('sonar_start', 'record: starting the sonar map tool' if msg == 'ok' else
                          f'record: the sonar tool did not start ({msg or "launcher on 8096 not answering"})')
        elif want and data and not sonar['rec'] and now - self.t['sonar_rec'] >= 5.0:
            self.t['sonar_rec'] = now
            self._ask(SONAR, '/record', ('start ' + self._slug()).encode())
        elif not want and mine and now - self.t['sonar_rec'] >= 5.0:
            self.t['sonar_rec'] = now
            self._ask(SONAR, '/record', b'stop')

    def _slug(self):
        return 'radio-' + self.session[len('radio_'):].replace('_', '-')    # as the sonar Recorder will spell it

    # ------------------------------------------------------------------ one pass
    def step(self):
        now = self.clock()
        with self.lock:
            want = dict(self.want)
        if now - self.t['free'] >= 5.0:
            self.t['free'] = now
            try:
                os.makedirs(self.root, exist_ok=True)
                self.free_gib = self.free(self.root) / 2 ** 30
            except OSError:
                self.free_gib = None
        full = self.free_gib is not None and self.free_gib < MIN_FREE_GIB
        self._say('disk', f'record: only {self.free_gib:.1f} GiB free - nothing is recorded below '
                          f'{MIN_FREE_GIB:.0f} GiB' if full else '')
        if full:
            want = {'camera': False, 'sonar': False}
        try:
            if any(want.values()) and self.session is None:
                self._open_session(want)
            poll = now - self.t['poll'] >= POLL_S
            if poll:
                self.t['poll'] = now
            self._camera(now, want['camera'], poll)
            self._sonar(now, want['sonar'], poll)
            if self.session and now - self.t['nav'] >= NAV_S:
                self.t['nav'] = now
                self.nav_file.write(json.dumps(dict(self.nav() or {}, t=round(now, 3),
                                                    wall=round(self.wall(), 3))) + '\n')
                self.nav_file.flush()
            fb = self.fallback['thread']
            if (self.session and not any(want.values()) and not self.sonar['mine']
                    and not (fb and fb.is_alive())):
                self._close_session()
        except (OSError, ValueError) as ex:         # ValueError: nav.jsonl closed under us at shutdown
            self._say('io', f'record: {type(ex).__name__}: {ex}')

    def run(self):
        while not self.closed.wait(NAV_S):
            self.step()

    def close(self):
        self.closed.set()
        if self.fallback['stop']:
            self.fallback['stop'].set()
        if self.nav_file:
            self.nav_file.close()

    # ------------------------------------------------------------------ for the status frame
    def snapshot(self):
        now = self.clock()
        with self.lock:
            want = dict(self.want)
        cam, sonar, fb = self.cam, self.sonar, self.fallback
        composite = bool(fb['thread'] and fb['thread'].is_alive() and now - fb['t'] <= CAMERA_FRESH_S)
        ages = cam['ages']
        return dict(
            rec_cam_want=want['camera'], rec_sonar_want=want['sonar'],
            rec_cam_on=cam['saving'] or composite, rec_cam_composite=composite and not cam['saving'],
            rec_sonar_on=bool(sonar['rec']),
            cam_service=cam['service'],
            cam_down='DOWN' in ages and ages['DOWN'] <= CAMERA_FRESH_S,
            cam_front='FRONT' in ages and ages['FRONT'] <= CAMERA_FRESH_S,
            cameras={n: round(a, 1) for n, a in ages.items()},
            sonar_service=sonar['service'],
            sonar_pinging=sonar['service'] and now - self.t['sweep'] <= PING_FRESH_S,
            rec_cam_count=cam['count'] if self.session else 0,
            rec_sonar_count=(sonar['rec'] or {}).get('sweeps', 0),
            disk_free_gib=self.free_gib, disk_low=self.free_gib is not None and self.free_gib < MIN_FREE_GIB,
            session=self.session, sonar_folder=sonar['folder'],
            cam_error=cam['error'], sonar_error=sonar['error'])
