"""Ruth's survey run as a GUI mission (Nov 11): the Missions page's "Survey run (Ruth)".

gui_node starts tools/survey_run.py in GUI mode with the route's points: surface GPS legs (Beichen's waypoint
mission, one run per point) to the start buoy, recordings, pipeline_mission's transit 4 m above the floor to the
end buoy, surface. Stop mission sends it SIGUSR1 (survey_run aborts the leg, or the transit surfaces).
survey_run runs from the source tree, as tools/survey.sh runs it: tools/ is not installed.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

SCRIPT = 'tools/survey_run.py'


def source_dir():
    here = Path(__file__).resolve().parents[3]
    for item in (os.environ.get('GRAEY_SOURCE_DIR'), here, '/root/robotx_ws/src/robotx_graey_2026',
                 Path.home()/'robotx_ws/src/robotx_graey_2026'):
        if item and (Path(item)/SCRIPT).is_file():
            return Path(item)
    raise ValueError('tools/survey_run.py not found (set GRAEY_SOURCE_DIR to the source checkout)')


def validate_points(value):
    """[[lat, lon], ...] from [{lat, lon}, ...]: at least 2 (the last two are the start and end buoys)."""
    if not isinstance(value, list) or not 2 <= len(value) <= 256:
        raise ValueError('Plot at least 2 points: the last two are the start buoy and the end buoy')
    points = []
    for item in value:
        try:
            lat, lon = float(item['lat']), float(item['lon'])
        except (TypeError, KeyError, ValueError):
            raise ValueError('Invalid route point') from None
        if not (math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 85 and abs(lon) <= 180):
            raise ValueError('Invalid route point')
        points.append([lat, lon])
    (a, b), (c, d) = points[-2], points[-1]
    north = math.radians(c-a)*6371000.
    east = math.radians(d-b)*6371000.*math.cos(math.radians(a))
    if math.hypot(north, east) < 1.:
        raise ValueError('The start and end buoys must be at least 1 m apart')
    return points


class SurveyLauncher:
    def __init__(self, log_dir=None, popen=subprocess.Popen, clock=time.monotonic):
        self.log_dir = Path(log_dir or os.environ.get('GRAEY_WAYPOINT_LOG_DIR', str(Path.home()/'robotx_ws/logs')))
        self.popen, self.clock = popen, clock
        self.process = self.launch = None

    def running(self):
        return self.process is not None and self.process.poll() is None

    def owns(self, run_id):
        return bool(run_id) and self.running() and run_id == self.launch['run_id']

    def start(self, points, dry=False, competition=False):
        if type(dry) is not bool:
            raise ValueError('dry must be a boolean')
        if type(competition) is not bool:
            raise ValueError('competition must be a boolean')
        points = validate_points(points)
        if self.running():
            raise ValueError('A survey run is already running')
        folder = source_dir()
        run_id = uuid.uuid4().hex
        self.log_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.log_dir/('survey-'+run_id+'.log')
        argv = [sys.executable, '-u', str(folder/SCRIPT), '--points', json.dumps(points), '--run-id', run_id,
                '--logdir', str(self.log_dir)] + (['--dry'] if dry else []) + (['--competition'] if competition else [])
        env = dict(os.environ, PYTHONPATH=str(folder)+(':'+os.environ['PYTHONPATH'] if os.environ.get('PYTHONPATH') else ''))
        with log_path.open('w') as log:
            # The GUI's own process group, like the waypoint mission: an isolated SITL shutdown reaps it too.
            self.process = self.popen(argv, cwd=str(folder), env=env, stdout=log, stderr=subprocess.STDOUT)
        self.launch = dict(run_id=run_id, started=self.clock(), log=str(log_path), dry=dry,
                           competition=competition, points=points,
                           stop_requested=False)
        return dict(ok=True, run_id=run_id)

    def stop(self, run_id):
        if not self.owns(run_id):
            raise ValueError('Current survey run ID required')
        self.process.send_signal(getattr(signal,'SIGUSR1',signal.SIGTERM))
        self.launch['stop_requested'] = True
        return dict(ok=True, msg='Stop requested: an aborted leg stops at the surface, a transit surfaces. '
                                 'Verify on the vehicle and take control if needed.')

    def snapshot(self):
        if not self.launch:
            return dict(running=False)
        code = None if self.process.poll() is None else self.process.returncode
        try:
            with open(self.launch['log'], 'rb') as f:
                f.seek(0, 2)
                f.seek(max(0, f.tell()-6000))
                tail = f.read().decode('utf-8', 'replace').splitlines()[-25:]
        except OSError:
            tail = []
        result = {None: None, 0: 'DONE', 1: 'FAILED', 2: 'STOPPED'}.get(code, 'EXITED %s' % code)
        return dict(running=code is None, run_id=self.launch['run_id'], result=result, exit_code=code,
                    dry=self.launch['dry'], competition=self.launch['competition'], points=self.launch['points'], stop_requested=self.launch['stop_requested'],
                    age_s=round(self.clock()-self.launch['started'], 1), log=tail)
