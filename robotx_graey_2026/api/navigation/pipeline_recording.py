"""Nonblocking mission adapter for existing host sonar service and DOWN camera."""
import json
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path


class PipelineRecording:
    def __init__(self, folder, run_id):
        self.folder = Path(folder)/('survey-'+run_id)
        self.run_id = run_id
        self.desired = False
        self.closed = threading.Event()
        self.state = dict(ready=False, reason='Recorders idle')
        self.thread = threading.Thread(target=self.worker, daemon=True)
        self.thread.start()

    def update(self, desired):
        self.desired = bool(desired)
        return dict(self.state)

    def close(self):
        self.desired = False
        self.closed.set()
        self.thread.join(timeout=8)

    @staticmethod
    def request(port, path, body=None):
        request = urllib.request.Request(f'http://127.0.0.1:{port}{path}',
            data=None if body is None else body.encode())
        with urllib.request.urlopen(request, timeout=1) as response:
            return response.read().decode()

    def worker(self):
        camera = None
        log = None
        active = False
        segment = 0
        sonar_started = False
        configured = False
        last_sweeps = None
        last_sonar = 0.
        previous_settings = None
        camera_attempt = 0
        camera_started = 0.
        try:
            while not self.closed.is_set():
                try:
                    if self.desired and not active:
                        segment += 1
                        folder = self.folder/f'segment-{segment:03d}'
                        folder.mkdir(parents=True, exist_ok=True)
                        log = (folder/'camera.log').open('w')
                        camera_attempt = 1
                        camera_folder = folder/'camera-part-001'
                        camera_folder.mkdir()
                        camera = subprocess.Popen([sys.executable, '-m',
                            'robotx_graey_2026.api.vision.bottom_recorder', '--directory', str(camera_folder)],
                            stdout=log, stderr=subprocess.STDOUT)
                        camera_started = time.monotonic()
                        active = True
                        configured = sonar_started = False
                        last_sweeps = None
                    if active and not self.desired:
                        if camera.poll() is None:
                            camera.terminate()
                            camera.wait(timeout=5)
                        log.close()
                        if sonar_started:
                            self.request(8095, '/record', 'stop')
                        if previous_settings:
                            self.restore_settings(previous_settings)
                            previous_settings = None
                        active = sonar_started = False
                        self.state = dict(ready=False, reason='Recording saved', directory=str(folder))
                    if active:
                        if camera.poll() is not None:
                            if time.monotonic()-camera_started >= 5:
                                camera_attempt += 1
                                camera_folder = folder/f'camera-part-{camera_attempt:03d}'
                                camera_folder.mkdir()
                                camera = subprocess.Popen([sys.executable, '-m',
                                    'robotx_graey_2026.api.vision.bottom_recorder', '--directory', str(camera_folder)],
                                    stdout=log, stderr=subprocess.STDOUT)
                                camera_started = time.monotonic()
                        if not configured:
                            service = json.loads(self.request(8096, '/status'))
                            if not service.get('webUp'):
                                self.request(8096, '/start?tool=map', '')
                            state = json.loads(self.request(8095, '/state'))
                            if not state.get('map'):
                                raise RuntimeError('Sonar is not running the map/recording tool')
                            existing = json.loads(self.request(8095, '/mapdata')).get('recording')
                            owned = existing and self.run_id[:16]+f'-segment-{segment}' in existing.get('name', '')
                            if existing and not owned:
                                raise RuntimeError('Existing sonar recording belongs to another run; finish it first')
                            if previous_settings is None:
                                previous_settings = dict(range=state['range'], sector=state['sector'])
                            self.request(8095, '/range', '6')
                            self.request(8095, '/sector', '225 315')
                            state = json.loads(self.request(8095, '/state'))
                            if state.get('range') != 6 or state.get('sector') != [225, 315]:
                                raise RuntimeError('Sonar settings readback differs from 6 m / 225–315°')
                            if not owned:
                                self.request(8095, '/record', 'start '+self.run_id[:16]+f'-segment-{segment}')
                            sonar_started = configured = True
                        sonar = json.loads(self.request(8095, '/mapdata')).get('recording') or {}
                        if self.run_id[:16]+f'-segment-{segment}' not in sonar.get('name', ''):
                            raise RuntimeError('Waiting for this mission sonar recording acknowledgement')
                        settings = json.loads(self.request(8095, '/state'))
                        if settings.get('range') != 6 or settings.get('sector') != [225, 315]:
                            raise RuntimeError('Pipeline sonar settings changed during survey')
                        sweeps = sonar.get('sweeps', 0)
                        if sweeps and sweeps != last_sweeps:
                            last_sweeps, last_sonar = sweeps, time.monotonic()
                        cam = json.loads((camera_folder/'camera-status.json').read_text()) if (camera_folder/'camera-status.json').exists() else {}
                        ready = (camera.poll() is None and 0 <= time.monotonic()-cam.get('monotonic', 0) < 2
                                 and sweeps > 0 and time.monotonic()-last_sonar < 30)
                        self.state = dict(ready=ready, reason='Camera and sonar recording' if ready else
                            'Waiting for fresh saved camera frames and sonar sweeps; failed camera processes retry every 5 s',
                            directory=str(folder), camera_directory=str(camera_folder), sonar=sonar)
                        (folder/'recording-status.json').write_text(json.dumps(self.state, indent=2))
                except Exception as exc:
                    configured = False  # Reconnect/relaunch sonar without blocking mission control.
                    self.state = dict(ready=False, reason='Recording: '+str(exc))
                self.closed.wait(.5)
        finally:
            if camera is not None and camera.poll() is None:
                camera.terminate()
                try:
                    camera.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    camera.kill()
                    camera.wait()
            if log is not None:
                log.close()
            if sonar_started:
                try:
                    self.request(8095, '/record', 'stop')
                except Exception:
                    pass
            if previous_settings:
                try:
                    self.restore_settings(previous_settings)
                except Exception:
                    pass

    def restore_settings(self, previous):
        self.request(8095, '/range', str(previous['range']))
        self.request(8095, '/sector', ' '.join(str(v) for v in previous['sector']))
