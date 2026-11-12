"""Local split-view demo: real GUI + ArduSub SIM_STATE, no vehicle endpoints."""
import json, math, os, signal, subprocess, threading, time, urllib.request
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pymavlink import mavutil
from robotx_graey_2026.api.navigation.waypoint_logic import ne_to_geo, geo_to_ne
from robotx_graey_2026.api.pixhawk.identity import FCU_COMPONENT, FCU_SYSTEM

ROOT=Path(__file__).resolve().parents[1]
STATE={'truth':None,'plan':None,'error':None,'origin':None}
LOCK=threading.Lock()

def api(body=None):
    request=urllib.request.Request('http://127.0.0.1:8090/api/waypoint',data=json.dumps(body).encode() if body else None,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=3) as r:return json.load(r)

def demo():
    try:
        state=api()
        if not state['capabilities'].get('simulation'):raise ValueError('Not an isolated simulation')
        cube=state['navigation']['cube_estimate']
        origin=(cube['lat'],cube['lon'])
        points=[]
        for kind,n,e in [('pipeline_start',2,0),('pipeline_buoy',2,1),('pipeline_end',2,4),('waypoint',0,4)]:
            lat,lon=ne_to_geo(n,e,origin);points.append(dict(kind=kind,lat=lat,lon=lon,name=kind.replace('_',' ')))
        plan=dict(points=points,pipeline_end_action='surface',recording_failure_action='finish_segment')
        preview=api(dict(action='preview',plan=plan))
        run=api(dict(action='start',plan_id=preview['plan_id']))
        with LOCK:STATE.update(plan=plan,error=None,run_id=run['run_id'])
        print('DEMO STARTED '+run['run_id'],flush=True)
    except Exception as exc:
        with LOCK:STATE['error']=str(exc)
        raise

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ('/','/split'):
            self.reply((ROOT/'sitl/split_view.html').read_bytes(),'text/html');return
        if self.path=='/sim-state':
            with LOCK:value=dict(STATE)
            self.reply(json.dumps(value).encode(),'application/json');return
        self.proxy()
    def do_POST(self):
        if self.path=='/demo':
            try:
                with LOCK:
                    if STATE.get('starting'):raise ValueError('Start in progress')
                    STATE['starting']=True
                demo();self.reply(b'{"ok":true}','application/json')
            except Exception as exc:self.reply(json.dumps(dict(error=str(exc))).encode(),'application/json',409)
            finally:
                with LOCK:STATE['starting']=False
            return
        self.proxy()
    def reply(self,data,kind,code=200):
        self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def proxy(self):
        if not self.path.startswith('/') or self.path.startswith('//'):
            self.reply(b'Invalid path','text/plain',400);return
        try:
            if self.command=='POST' and not api()['capabilities'].get('simulation'):
                raise RuntimeError('Refusing commands outside isolated SITL')
            body=self.rfile.read(int(self.headers.get('Content-Length',0))) if self.command=='POST' else None
            request=urllib.request.Request('http://127.0.0.1:8090'+self.path,data=body,headers={'Content-Type':self.headers.get('Content-Type','application/json')})
            with urllib.request.urlopen(request,timeout=4) as r:self.reply(r.read(),r.headers.get('Content-Type','application/octet-stream'),r.status)
        except urllib.error.HTTPError as e:self.reply(e.read(),e.headers.get('Content-Type','text/plain'),e.code)
        except Exception as e:self.reply(str(e).encode(),'text/plain',502)
    def log_message(self,*args):pass

def truth_loop(stop):
    link=mavutil.mavlink_connection('udpout:127.0.0.1:14559',source_system=255,source_component=202)
    last=0;surface=None;origin=None
    try:
        while not stop.is_set():
            now=time.monotonic()
            if now-last>2:
                link.mav.heartbeat_send(6,8,0,0,0)
                link.mav.command_long_send(FCU_SYSTEM,FCU_COMPONENT,511,0,mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE,100000,0,0,0,0,0)
                last=now
            m=link.recv_match(type='SIM_STATE',blocking=True,timeout=.2)
            if m is None:continue
            if surface is None:surface=m.alt;origin=(m.lat,m.lon)
            n,e=geo_to_ne(m.lat,m.lon,origin)
            with LOCK:STATE.update(origin=origin,truth=dict(north=n,east=e,depth=max(0.,surface-m.alt),yaw=math.degrees(m.yaw)%360,speed=math.hypot(m.vn,m.ve),received=time.time()))
    finally:link.close()

def main():
    folder=ROOT/'sitl/logs'/('split-'+time.strftime('%Y%m%d-%H%M%S'));folder.mkdir(parents=True)
    env=dict(os.environ,SITL_LOG_DIR=str(folder),SITL_RUN_DIR=str(folder/'runtime'),WAYPOINT_INTERACTIVE='true',SITL_CONTROL_FIXTURE='true',GRAEY_WAYPOINT_LOG_DIR=str(folder),GRAEY_SITL_RECORDING_FAILURE='0')
    env.pop('QGC_HOST',None)
    stop=threading.Event();proc=None
    try:
        with (folder/'runner.log').open('w') as log:
            proc=subprocess.Popen(['bash','sitl/run.sh'],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            for _ in range(180):
                if proc.poll() is not None:raise RuntimeError('SITL exited; '+str(folder/'runner.log'))
                try:
                    state=api()
                    if state.get('capabilities',{}).get('simulation') and state['navigation']['cube_estimate'].get('valid'):break
                except Exception:pass
                time.sleep(1)
            else:raise RuntimeError('SITL startup timed out')
            threading.Thread(target=truth_loop,args=(stop,),daemon=True).start()
            server=ThreadingHTTPServer(('127.0.0.1',8108),Handler)
            # Access is local test UI only; the proxied GUI was confirmed isolated above.
            print('SPLIT READY http://localhost:8108 Logs '+str(folder),flush=True)
            def start_demo():
                time.sleep(5)
                try:demo()
                except Exception as e:print('Demo start failed: '+str(e),flush=True)
            threading.Thread(target=start_demo,daemon=True).start()
            server.serve_forever()
    finally:
        stop.set()
        if proc:
            try:os.killpg(proc.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL)

if __name__=='__main__':main()
