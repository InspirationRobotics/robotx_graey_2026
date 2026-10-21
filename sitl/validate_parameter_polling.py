#!/usr/bin/env python3
"""Validate production readback batching against isolated ArduSub, never arm.

Only uses loopback TCP6170 on simulator instance41. No QGC/ROS/MAVProxy and no
live-vehicle endpoint option. Loads only the production request() method to
exercise its actual parameter schedule without importing a ROS node.
"""
import ast
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import time
from types import SimpleNamespace as NS

from robotx_graey_2026.api.navigation.odometry_profile import PROFILE
from robotx_graey_2026.api.navigation.cube_north import COMPASS_PARAMS
from robotx_graey_2026.api.pixhawk.mavlink import Link


def main():
    root=Path(__file__).resolve().parents[1]
    evidence=root/'sitl/logs'/('parameter-polling-'+time.strftime('%Y%m%d-%H%M%S'))
    evidence.mkdir(parents=True)
    stage=Path(tempfile.mkdtemp(prefix='graey-polling-',dir='/tmp'))
    binary=Path(os.environ.get('SITL_BIN',str(root.parent/'output/gps-sitl-2026-10-02/ardusub')))
    shutil.copy2(binary,stage/'ardusub');(stage/'ardusub').chmod(0o700)
    probe=socket.socket();probe.bind(('127.0.0.1',6170));probe.close()
    defaults=dict(PROFILE,COMPASS_ENABLE=1,COMPASS_USE=0,COMPASS_LEARN=0,EK2_ENABLE=0)
    (stage/'defaults.parm').write_text('\n'.join(f'{k} {v}' for k,v in defaults.items()))
    path=root/'robotx_graey_2026/api/navigation/fused_nav_bridge.py'
    tree=ast.parse(path.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FusedNavBridge')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='request')
    namespace=dict(PROFILE=PROFILE,COMPASS_PARAMS=COMPASS_PARAMS)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(path),'exec'),namespace)
    result=dict(passed=False,qgc_forwarding=False,ros_started=False,endpoint='tcp:127.0.0.1:6170',phases=[])
    log=(evidence/'firmware.txt').open('w')
    proc=subprocess.Popen([str(stage/'ardusub'),'-M','vectored','-I','41','-s','1',
                           '--defaults',str(stage/'defaults.parm')],cwd=stage,stdout=log,stderr=subprocess.STDOUT)
    link=None
    try:
        # Let the TCP server start; all connection retries remain local.
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            try:link=Link('tcp:127.0.0.1:6170',190,system=255);break
            except OSError:time.sleep(.2)
        if link is None or link.mav.wait_heartbeat(timeout=10) is None:raise RuntimeError('No isolated firmware heartbeat')
        node=NS(link=link,settings={},sync_clock=lambda:None)
        for compass in (False,True):
            node.output_permitted=not compass
            required=set(PROFILE)|set(COMPASS_PARAMS if compass else ())
            received={};values={};started=time.monotonic();poll=0.;checks=0;maximum_age=0.
            while time.monotonic()-started<7.:
                now=time.monotonic()
                if now>=poll:
                    namespace['request'](node);poll=now+1.
                m=link.mav.recv_match(blocking=True,timeout=.02)
                if m is not None and (m.get_srcSystem(),m.get_srcComponent())==(1,1):
                    if m.get_type()=='HEARTBEAT' and m.base_mode&128:raise RuntimeError('Simulator unexpectedly armed')
                    if m.get_type()=='PARAM_VALUE':
                        key=m.param_id.rstrip('\x00')
                        if key in required:received[key]=time.monotonic();values[key]=m.param_value
                now=time.monotonic()
                if now-started>=2.5:
                    missing=required-set(received)
                    if missing:raise RuntimeError('Missing readbacks: '+repr(sorted(missing)))
                    age=max(now-received[k] for k in required);maximum_age=max(maximum_age,age)
                    if age>=3.:raise RuntimeError('Readback freshness exceeded three seconds')
                    checks+=1
            result['phases'].append(dict(mode='compass' if compass else 'navigation',
                required_fields=len(required),maximum_readback_age_s=maximum_age,checks=checks,values=values))
            print(json.dumps(result['phases'][-1]),flush=True)
        result['passed']=True
    except Exception as error:result['error']=str(error)
    finally:
        if link:link.mav.close()
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close();(evidence/'result.json').write_text(json.dumps(result,indent=2))
        print('Evidence: '+str(evidence),flush=True)
    if not result['passed']:raise SystemExit(result.get('error','Polling validation failed'))


if __name__=='__main__':main()
