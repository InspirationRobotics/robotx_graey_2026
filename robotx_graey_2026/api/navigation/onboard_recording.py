"""Non-blocking onboard camera/sonar recording for the SE mission."""
import copy
import json
import threading
import time
import urllib.error
import urllib.request


RECORDING_URL = 'http://127.0.0.1:8090/api/recording'


def recording_request(command=None, timeout=12):
    body = None if command is None else json.dumps(command).encode()
    request = urllib.request.Request(
        RECORDING_URL, data=body,
        headers={'Content-Type': 'application/json'} if body is not None else {},
        method='POST' if body is not None else 'GET')
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read(1 << 20))
    except urllib.error.HTTPError as exc:
        try:
            detail = json.loads(exc.read(1 << 20)).get('msg')
        except (ValueError, AttributeError):
            detail = None
        raise RuntimeError(detail or f'Recording service HTTP {exc.code}') from exc


class OnboardMissionRecording:
    """Own one camera+sonar session without delaying mission control.

    Recording starts in a worker thread when SE is accepted.  Failure is
    observable but never gates motion.  The same worker stops the session on
    completion/abort, so start and stop cannot race each other.
    """
    def __init__(self, run_id, log, request=recording_request):
        self.run_id = run_id
        self.log = log
        self.request = request
        self.stop_event = threading.Event()
        self.started_event = threading.Event()
        self.lock = threading.Lock()
        self.thread = None
        self.state = dict(
            onboard=True, requested=False, starting=False, recording=False,
            camera=False, sonar=False, stopping=False, stopped=False,
            message='Waiting for SE', errors=[])

    def _update(self, **values):
        with self.lock:
            self.state.update(values)

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def start(self):
        if self.thread is not None:
            return
        self._update(requested=True, starting=True, message='Starting onboard camera and sonar')
        self.thread = threading.Thread(
            target=self._run, daemon=True,
            name='graey-headless-recording')
        self.thread.start()

    def _read_sources(self):
        status = self.request(None)
        session = status.get('session') or {}
        sources = session.get('sources') or {}
        camera = (sources.get('camera') or {}).get('recording') is True
        sonar_row = sources.get('sonar') or {}
        sonar = sonar_row.get('recording') is True
        errors = list(session.get('errors') or [])
        for kind in ('camera', 'sonar'):
            error = (sources.get(kind) or {}).get('error')
            if error and f'{kind}: {error}' not in errors:
                errors.append(f'{kind}: {error}')
        return camera, sonar, errors, session.get('name')

    def _run(self):
        attempted = False
        try:
            attempted = True
            result = self.request(dict(
                action='start', name='rc_dive_' + self.run_id[:16],
                camera=True, sonar=True, sensors=False, topics=[]))
            deadline = time.monotonic() + 8
            camera = sonar = False
            errors = []
            session = None
            while time.monotonic() < deadline and not self.stop_event.is_set():
                camera, sonar, errors, session = self._read_sources()
                if (camera and sonar) or errors:
                    break
                self.stop_event.wait(.25)
            self._update(
                starting=False, recording=bool(camera or sonar),
                camera=camera, sonar=sonar, errors=errors,
                session=session,
                message=result.get('msg', 'Onboard recording requested'))
            self.log('HEADLESS_RECORDING_START ' + json.dumps(self.snapshot(), allow_nan=False))
        except (OSError, ValueError, RuntimeError) as exc:
            self._update(starting=False, recording=False, errors=[str(exc)], message=str(exc))
            self.log('HEADLESS_RECORDING_ERROR ' + json.dumps(self.snapshot(), allow_nan=False))
        finally:
            self.started_event.set()

        self.stop_event.wait()
        if attempted:
            self._update(stopping=True, message='Stopping onboard camera and sonar')
            try:
                result = self.request(dict(action='stop'))
                self._update(
                    stopping=False, stopped=True, recording=False,
                    camera=False, sonar=False,
                    message=result.get('msg', 'Onboard recording stopped'))
                self.log('HEADLESS_RECORDING_STOP ' + json.dumps(self.snapshot(), allow_nan=False))
            except (OSError, ValueError, RuntimeError) as exc:
                errors = self.snapshot().get('errors', []) + [f'stop: {exc}']
                self._update(stopping=False, stopped=False, errors=errors, message=str(exc))
                self.log('HEADLESS_RECORDING_ERROR ' + json.dumps(self.snapshot(), allow_nan=False))

    def stop(self):
        self.stop_event.set()

    def close(self, timeout=15):
        self.stop()
        if self.thread is not None:
            self.thread.join(timeout)

