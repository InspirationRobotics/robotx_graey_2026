"""SITL: does the GUI keep showing the Cube's pose while the computer is overloaded? (Nov 11, localhost only)

On Graey (load 13 on 6 cores: the sonar map and cameras on top of the stack) gui_node's Cube streams went stale for
minutes while QGC showed the pose. This runs sitl/run.sh plus telemetry_gateway (as validate_survey_run.py does),
waits for a live GUI pose, then starts BURNERS busy processes (default 2 per CPU) for LOAD_S seconds and samples the
GUI API every 0.5 s: the ages of qgc_global (GLOBAL_POSITION_INT), attitude and heartbeat, and how long the API took.
Passes when, under load, qgc_global is fresh (< 2 s old) in at least 90% of the samples.
    python3 sitl/validate_gui_under_load.py [burners] [load_s]
"""
import json
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_survey_run import CONTROL_NODE, GATEWAY_PORT, api  # noqa: E402

STREAMS = ('qgc_global', 'attitude', 'heartbeat')


def sample():
    t = time.monotonic()
    state = api()
    took = time.monotonic()-t
    streams = state['navigation']['streams']
    return dict(api_s=round(took, 3), **{k: round(streams.get(k, {}).get('age', float('inf')), 2) for k in STREAMS})


def summary(rows):
    if not rows:
        return {}
    fresh = sum(r['qgc_global'] < 2 for r in rows)/len(rows)
    return dict(samples=len(rows), qgc_global_fresh=round(fresh, 3),
                max_age={k: max(r[k] for r in rows) for k in STREAMS},
                max_api_s=max(r['api_s'] for r in rows))


def main():
    burners = int(sys.argv[1]) if len(sys.argv) > 1 else 2*(os.cpu_count() or 4)
    load_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60.
    root = Path(__file__).resolve().parents[1]
    folder = root/'sitl/logs'/('gui-load-'+uuid.uuid4().hex[:10])
    folder.mkdir(parents=True)
    env = dict(os.environ, SITL_LOG_DIR=str(folder), SITL_RUN_DIR=str(folder/'runtime'), CONTINUOUS_FUSION='true',
               GRAEY_WAYPOINT_LOG_DIR=str(folder), ROS_DOMAIN_ID='73', ROS_LOCALHOST_ONLY='1',
               GRAEY_RUNTIME_SCOPE='sitl', CONTROL_NODE=CONTROL_NODE)
    env.pop('QGC_HOST', None)
    result = dict(passed=False, burners=burners, load_s=load_s)
    procs, burn = [], []
    try:
        with (folder/'runner.log').open('w') as log:
            procs.append(subprocess.Popen(['bash', 'sitl/run.sh'], cwd=root, env=env, stdout=log,
                                          stderr=subprocess.STDOUT, start_new_session=True))
        ros = 'source /opt/ros/humble/setup.bash && source install/setup.bash && '
        extras, deadline = False, time.monotonic()+600
        while time.monotonic() < deadline:
            if procs[0].poll() is not None:
                raise RuntimeError('SITL exited')
            if not extras and (root/'install/setup.bash').exists() and (folder/'ros.log').exists():
                for name, cmd in (('gateway', f'ros2 run robotx_graey_2026 telemetry_gateway --ros-args '
                                              f'-p mavlink:=udpout:127.0.0.1:{GATEWAY_PORT}'),
                                  ('watchdog', 'python3 -c "$CONTROL_NODE"')):
                    with (folder/(name+'.log')).open('w') as log:
                        procs.append(subprocess.Popen(['bash', '-c', ros+cmd], cwd=root, env=env, stdout=log,
                                                      stderr=subprocess.STDOUT, start_new_session=True))
                extras = True
            try:
                if sample()['qgc_global'] < 1:
                    break
            except (OSError, KeyError, ValueError):
                pass
            time.sleep(1.)
        else:
            raise RuntimeError('no live GUI pose within 600 s')
        quiet = []
        for _ in range(40):
            quiet.append(sample())
            time.sleep(.5)
        print('quiet', json.dumps(summary(quiet)), flush=True)
        burn = [subprocess.Popen([sys.executable, '-c', 'while True: pass']) for _ in range(burners)]
        loaded, end = [], time.monotonic()+load_s
        while time.monotonic() < end:
            try:
                loaded.append(sample())
            except (OSError, KeyError, ValueError):
                loaded.append(dict(api_s=float('inf'), **{k: float('inf') for k in STREAMS}))
            time.sleep(.5)
        print('loaded', json.dumps(summary(loaded)), flush=True)
        result.update(quiet=summary(quiet), loaded=summary(loaded))
        result['passed'] = summary(loaded).get('qgc_global_fresh', 0) >= .9
        (folder/'samples.json').write_text(json.dumps(dict(quiet=quiet, loaded=loaded)))
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        for p in burn:
            p.kill()
        for proc in reversed(procs):
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for proc in reversed(procs):
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        (folder/'validation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(evidence=str(folder), **result)), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
