"""Native ArduSub route test, localhost only; camera/sonar are explicitly stubbed."""
import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
from validate_waypoint import api
from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording-failure', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    folder = root/'sitl/logs'/('competition-'+uuid.uuid4().hex[:10])
    folder.mkdir(parents=True)
    env = dict(os.environ, SITL_LOG_DIR=str(folder), SITL_RUN_DIR=str(folder/'runtime'),
               WAYPOINT_INTERACTIVE='true', SITL_CONTROL_FIXTURE='true', CONTINUOUS_FUSION='true',
               GRAEY_WAYPOINT_LOG_DIR=str(folder))
    env.pop('QGC_HOST', None)
    env['GRAEY_SITL_RECORDING_FAILURE'] = '1' if args.recording_failure else '0'
    result = dict(passed=False, stages=[], max_depth_m=0., recording='SIMULATION STUB',
                  recording_failure_injected=args.recording_failure)
    proc = None
    run_id = None
    try:
        with (folder/'runner.log').open('w') as log:
            proc = subprocess.Popen(['bash', 'sitl/run.sh'], cwd=root, env=env,
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+600
            with (folder/'observations.jsonl').open('w') as observations:
                while time.monotonic() < deadline:
                    if proc.poll() is not None:
                        raise RuntimeError('SITL exited before completion')
                    try:
                        state = api()
                        if state['capabilities'].get('simulation') is not True:
                            raise RuntimeError('Refusing non-SITL GUI')
                        nav = state['navigation']; streams = nav['streams']
                        bridge = streams.get('bridge', {}).get('data', {})
                        cube = nav['cube_estimate']
                        if not run_id and cube.get('valid') and bridge.get('configuration_verified'):
                            origin = (cube['lat'], cube['lon'])
                            points = []
                            for kind, n, e in [('pipeline_start', 2., 0.), ('pipeline_buoy', 2., 1.),
                                               ('pipeline_end', 2., 4.), ('waypoint', 0., 4.)]:
                                lat, lon = ne_to_geo(n, e, origin)
                                points.append(dict(kind=kind, lat=lat, lon=lon))
                            plan = api(dict(action='preview', plan=dict(points=points,
                                pipeline_end_action='surface', recording_failure_action='finish_segment')))
                            run_id = api(dict(action='start', plan_id=plan['plan_id']))['run_id']
                        mission = streams.get('waypoint', {}).get('data', {})
                        if run_id and mission.get('run_id') == run_id:
                            stage = mission['stage']
                            depth = bridge.get('depth_m')
                            if isinstance(depth, (int, float)):
                                result['max_depth_m'] = max(result['max_depth_m'], depth)
                            observations.write(json.dumps(dict(stage=stage, depth=depth, cube=cube))+'\n')
                            observations.flush()
                            if not result['stages'] or result['stages'][-1] != stage:
                                result['stages'].append(stage)
                                print(stage, flush=True)
                            if stage in ('COMPLETE', 'ABORTED'):
                                result['mission'] = mission
                                result['passed'] = (stage == 'COMPLETE' and 'PIPELINE' in result['stages']
                                    and 3.8 <= result['max_depth_m'] <= 4.5
                                    and depth is not None and depth < .2)
                                if args.recording_failure:
                                    result['passed'] &= bool(mission.get('recording_warning'))
                                break
                    except (OSError, KeyError, ValueError) as exc:
                        result['last_retry'] = str(exc)
                    time.sleep(.2)
                else:
                    result['error'] = 'Validation timed out'
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        (folder/'validation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(evidence=str(folder), **result)), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

