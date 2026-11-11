"""Vehicle-owned recording sessions; network loss does not stop acquisition.

Uses Crusader's rosbag subprocess and measured growth conventions. Camera and
sonar are requested through existing owners, never by opening their devices.
RFD data copies the radio submerge test's own log rows (boat packets heard, raw
depth, depth from start) into the session while it records.
"""
import os
import copy
import json
import queue
import re
import shutil
import threading
import time
import uuid
import urllib.request
from pathlib import Path

from .bag_recorder import BagRecorder, describe_topics
from .crusader_system_info import mount_for
from .recorder import GrowthMeter, Recorder, safe_session_dir


def service_request(port,path,body=None):
    request=urllib.request.Request(f'http://127.0.0.1:{port}{path}',data=body)
    with urllib.request.urlopen(request,timeout=2) as response:
        data=response.read(1<<20)
        return json.loads(data) if response.headers.get_content_type()=='application/json' else data.decode()


class RecordingService:
    def __init__(self,root,*,request=service_request,bag=None,free=None,start_worker=True,persistence=mount_for,radio_log_dir=None):
        self.persistence=persistence
        # Where the radio submerge test writes waypoint-<run_id>-radio.csv (gui_node's mission log folder).
        self.radio_log_dir=Path(radio_log_dir or os.environ.get('GRAEY_WAYPOINT_LOG_DIR',str(Path.home()/'robotx_ws/logs')))
        self.rfd_active=False;self.rfd_runs={}
        self.root=Path(root);self.request_service=request;self.bag=bag or BagRecorder()
        self.free=free or (lambda:shutil.disk_usage(self.root).free)
        self.lock=threading.RLock();self.commands=queue.Queue(maxsize=1);self.closed=threading.Event();self.operation_done=threading.Event();self.operation_done.set()
        self.topics=[];self.session=None;self.busy=False;self.error=None;self.published=None
        self.camera_owned=self.sonar_owned=False;self.meter=GrowthMeter();self.telemetry=None
        self.thread=threading.Thread(target=self.worker,daemon=True,name='graey-recording')
        if start_worker:self.thread.start()

    def update_sources(self,pairs,telemetry):
        with self.lock:self.topics=describe_topics(pairs);self.telemetry=telemetry

    def submit(self,body):
        if not isinstance(body,dict) or body.get('action') not in ('start','stop','bag_stop'):
            raise ValueError('recording action must be start, stop or bag_stop')
        with self.lock:
            if self.busy:raise ValueError('Recording operation already pending')
            command=copy.deepcopy(body)
            if body['action']=='start':
                if self.session and self.session['active']:raise ValueError('A recording session is already active')
                for key in ('camera','sonar','sensors','rfd'):
                    if type(body.get(key,False)) is not bool:raise ValueError('Recording selections must be boolean')
                topics=body.get('topics',[]) if body.get('sensors') else []
                command['topics']=topics
                available={r['name'] for r in self.topics}
                if (not isinstance(topics,list) or len(topics)>128 or any(
                        not isinstance(t,str) or not t.startswith('/') or t not in available for t in topics)):
                    raise ValueError('Select currently advertised ROS topics')
                if body.get('sensors') and not topics:raise ValueError('Select at least one sensor topic')
                if not any(body.get(k) for k in ('camera','sonar','sensors','rfd')):raise ValueError('Select a recording source')
                label=body.get('name','session')
                if not isinstance(label,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,48}',label):
                    raise ValueError('Session name must use 1–48 letters, digits, underscores or hyphens')
            self.commands.put_nowait(command);self.busy=True;self.operation_done.clear()
        return dict(ok=True,msg='Recording request accepted; check source status.')

    def wait_result(self,action,timeout=10):
        if not self.operation_done.wait(timeout):return dict(ok=False,msg='Recording outcome not confirmed yet; inspect status before retrying')
        state=self.snapshot();session=state.get('session') or {}
        if state.get('error'):return dict(ok=False,msg=state['error'])
        if action=='start' and not session.get('active'):return dict(ok=False,msg='; '.join(session.get('errors',[])) or 'No selected source started')
        details=[kind+': '+('starting' if source.get('pending') else 'recording' if source.get('recording') else source.get('error','stopped')) for kind,source in session.get('sources',{}).items()]
        return dict(ok=True,msg='; '.join(details) or 'No recording session active')

    def snapshot(self):
        with self.lock:
            return dict(busy=self.busy,error=self.error,session=copy.deepcopy(self.published),topics=copy.deepcopy(self.topics))

    def sessions(self):
        rows=[]
        for path in sorted(self.root.glob('*/session.json'),reverse=True)[:50]:
            if path.resolve().parent.parent!=self.root.resolve():continue
            try:
                if path.stat().st_size>256000:continue
                data=json.loads(path.read_text(encoding='utf-8'))
                rows.append(dict(name=data['name'],directory=data['directory'],started_wall=data['started_wall'],
                                 stopped_wall=data.get('stopped_wall'),errors=data.get('errors',[])))
            except (OSError,ValueError,KeyError):continue
        return rows

    def delete(self,name):
        if self.session and self.session['name']==name and self.session['active']:
            raise ValueError('Refusing to delete the session being recorded')
        ok,message=Recorder(str(self.root)).delete(name)
        if not ok:raise ValueError(message)
        return dict(ok=True,msg=message)

    def archive_into(self,name,output):
        if self.session and self.session['name']==name and self.session['active']:
            raise ValueError('Stop recording before downloading the session')
        path=safe_session_dir(str(self.root),name)
        if path is None:raise ValueError('Invalid session name')
        manifest=Path(path)/'session.json'
        if manifest.is_file():
            data=json.loads(manifest.read_text(encoding='utf-8'))
            for kind,source in data.get('sources',{}).items():
                if kind in ('camera','sonar','rfd') and source.get('acquired'):
                    from ..recording_paths import session_record_path
                    path=session_record_path(self.root,name,kind,data['owner_token'])
                    if not path.exists():raise ValueError('Raw '+kind+' data is not visible in the shared session volume; archive refused instead of returning partial data')
        return Recorder(str(self.root)).archive_into(name,output)

    def start(self,command):
        self.root.mkdir(parents=True,exist_ok=True)
        if not self.persistence(str(self.root))[2]:raise ValueError('Recording directory is not on a persistent bind mount; use the configured persistent recordings volume')
        if self.free()<2*1024**3:raise ValueError('Less than 2 GiB free; recording not started')
        name=time.strftime('%Y%m%d-%H%M%S')+'-'+command.get('name','session')+'-'+uuid.uuid4().hex[:6]
        folder=self.root/name;folder.mkdir()
        self.meter.reset();self.error=None
        self.session=dict(name=name,owner_token='graey-'+uuid.uuid4().hex[:16],directory=str(folder),active=True,started_wall=time.time(),sources={},errors=[],selected=command)
        for kind in ('sensors','camera','sonar','rfd'):
            if not command.get(kind):continue
            try:
                if kind=='rfd':
                    # No device to start: rows are copied from the radio test's log each sample.
                    # Recording usually starts before the test, so wait for it rather than refuse.
                    self.rfd_active=True;self.rfd_runs={}
                    self.session['sources'][kind]=dict(recording=True,acquired=False,rows=0,runs=[],
                        path='rfd/'+self.session['owner_token'],waiting=True)
                elif kind=='sensors':
                    ok,message=self.bag.start(str(folder/'bag'),command['topics'])
                    if not ok:raise RuntimeError(message)
                    self.session['sources'][kind]=dict(recording=True,path=str(folder/'bag'))
                elif kind=='camera':
                    status=self.request_service(8096,'/status')
                    if status.get('recording_session_v1') is not True:raise RuntimeError('Camera service needs the GRAEY session-recording update')
                    if status.get('recording'):raise RuntimeError('Camera recording already owned elsewhere; not changed')
                    if not status.get('camUp'):raise RuntimeError('Start cameras on Sensors before recording')
                    result=self.request_service(8096,'/record/start?name='+self.session['owner_token']+'&session='+name,b'')
                    if not result.get('recording'):raise RuntimeError(result.get('msg','Camera recorder did not start'))
                    self.camera_owned=True
                    self.session['sources'][kind]=dict(recording=True,acquired=True,path='camera/'+self.session['owner_token']+'.mjpg')
                else:
                    state=self.request_service(8095,'/state')
                    if state.get('recording_session_v1') is not True:raise RuntimeError('Sonar service needs the GRAEY session-recording update')
                    if not state.get('map'):raise RuntimeError('Start the sonar map tool on Sensors before recording')
                    state=self.request_service(8095,'/mapdata')
                    if state.get('recording'):raise RuntimeError('Sonar recording already owned elsewhere; not changed')
                    self.request_service(8095,'/record',('start GRAEY_SESSION:'+name+':'+self.session['owner_token']).encode())
                    self.sonar_owned=True
                    self.session['sources'][kind]=dict(recording=False,pending=True,acquired=True,path='sonar/'+self.session['owner_token'])
            except (OSError,ValueError,RuntimeError) as exc:
                self.session['sources'][kind]=dict(recording=False,error=str(exc))
                self.session['errors'].append(kind+': '+str(exc))
        self.session['active']=bool(self.bag.recording or self.camera_owned or self.sonar_owned or self.rfd_active)
        self.persist()

    def copy_rfd(self):
        """Copy radio submerge test rows logged since the last sample into rfd/<owner>/<run_id>.csv."""
        from ..recording_paths import session_record_path
        from ..navigation.radio_submerge import copy_radio_rows, radio_log_path
        source=self.session['sources']['rfd']
        mission=((self.telemetry or {}).get('streams',{}).get('waypoint') or {}).get('data') or {}
        run=mission.get('run_id')
        if (mission.get('plan') or {}).get('kind')=='radio_submerge' and isinstance(run,str) and run not in self.rfd_runs:
            radio_log_path(self.radio_log_dir,run)          # refuses anything but a mission run ID
            self.rfd_runs[run]=None                         # first copy skips rows from before this session
        folder=session_record_path(self.root,self.session['name'],'rfd',self.session['owner_token'])
        for run,cursor in list(self.rfd_runs.items()):
            log=radio_log_path(self.radio_log_dir,run)
            if not log.exists():continue
            folder.mkdir(parents=True,exist_ok=True)
            self.rfd_runs[run],copied=copy_radio_rows(log,folder/(run+'.csv'),cursor,since_wall=self.session['started_wall'])
            source['rows']+=copied
        source.update(runs=sorted(self.rfd_runs),acquired=source['rows']>0,waiting=source['rows']==0)
        source.pop('error',None)

    def persist(self):
        if self.session:
            folder=Path(self.session['directory'])
            tmp=folder/'session.json.tmp';tmp.write_text(json.dumps(self.session,allow_nan=False,indent=2),encoding='utf-8');tmp.replace(folder/'session.json')

    def stop(self):
        errors=[]
        if self.bag.recording:
            ok,message=self.bag.stop()
            if not ok or self.bag.status().get('error'):errors.append(message)
        if self.camera_owned:
            try:
                current=self.request_service(8096,'/status')
                if current.get('recording') and self.session['owner_token'] not in (current.get('recFile') or ''):
                    self.camera_owned=False
                    raise RuntimeError('Camera ownership changed; external recorder was not stopped')
                result=self.request_service(8096,'/record/stop',b'')
                if result.get('recording'):raise RuntimeError('Camera still recording')
                self.camera_owned=False
            except (OSError,ValueError,RuntimeError) as exc:errors.append('camera stop: '+str(exc))
        if self.sonar_owned:
            try:
                current=self.request_service(8095,'/mapdata').get('recording')
                if current and self.session['owner_token'] not in current.get('name',''):
                    self.sonar_owned=False
                    raise ValueError('Sonar ownership changed; external recorder was not stopped')
                self.request_service(8095,'/record',b'stop')
                for _ in range(6):
                    if not self.request_service(8095,'/mapdata').get('recording'):
                        self.sonar_owned=False;break
                    time.sleep(.2)
                if self.sonar_owned:errors.append('Sonar stop not yet confirmed')
            except (OSError,ValueError) as exc:errors.append('sonar stop: '+str(exc))
        if self.rfd_active:
            try:self.copy_rfd()                             # rows logged up to the Stop
            except (OSError,ValueError) as exc:errors.append('rfd stop: '+str(exc))
            self.rfd_active=False
        if self.session:
            self.session.update(active=bool(self.camera_owned or self.sonar_owned or self.bag.recording),stopped_wall=time.time())
            self.session['errors'].extend(errors)
            for kind,source in self.session['sources'].items():
                source['recording']=(self.camera_owned if kind=='camera' else self.sonar_owned if kind=='sonar'
                                     else self.rfd_active if kind=='rfd' else self.bag.recording)
            self.persist()
        if errors:self.error='; '.join(errors)

    def sample(self):
        if not self.session or not self.session['active']:return
        folder=Path(self.session['directory']);free=self.free()
        if free<2*1024**3:
            self.error='Recording stopped: disk below 2 GiB reserve';self.stop();return
        if self.telemetry is not None:
            with (folder/'telemetry.jsonl').open('a',encoding='utf-8') as stream:
                stream.write(json.dumps(dict(wall_time=time.time(),navigation=self.telemetry),allow_nan=False)+'\n')
        if self.camera_owned:
            try:
                state=self.request_service(8096,'/status')
                self.session['sources']['camera'].update(recording=state.get('recording') is True,bytes=state.get('recBytes'),service_file=state.get('recFile'))
                if state.get('recording') and self.session['owner_token'] not in (state.get('recFile') or ''):
                    self.camera_owned=False
                    self.session['sources']['camera'].update(recording=None,error='Camera recorder ownership changed; not stopping it')
            except (OSError,ValueError) as exc:self.session['sources']['camera'].update(recording=None,error=str(exc))
        if self.sonar_owned:
            try:
                state=self.request_service(8095,'/mapdata').get('recording')
                self.session['sources']['sonar'].update(recording=bool(state),pending=not state and time.time()-self.session['started_wall']<5,details=state)
                if state and self.session['owner_token'] not in state.get('name',''):
                    self.sonar_owned=False
                    self.session['sources']['sonar'].update(recording=None,error='Sonar recorder ownership changed; not stopping it')
            except (OSError,ValueError) as exc:self.session['sources']['sonar'].update(recording=None,error=str(exc))
        if self.rfd_active:
            try:self.copy_rfd()
            except (OSError,ValueError) as exc:self.session['sources']['rfd'].update(error=str(exc))
        self.bag.tick();self.session['bag']=self.bag.status(free)
        self.meter.sample(str(folder));self.session.update(self.meter.readout(free))
        self.session.update(free_bytes=free,elapsed_s=round(time.time()-self.session['started_wall'],1))
        self.persist()

    def worker(self):
        while not self.closed.is_set():
            try:
                command=self.commands.get(timeout=.5)
            except queue.Empty:command=None
            try:
                if command:
                    if command['action']=='start':self.start(command)
                    elif command['action']=='bag_stop':self.bag.stop()
                    else:self.stop()
                self.sample()
            except Exception as exc:
                with self.lock:self.error=str(exc)
            finally:
                with self.lock:
                    self.published=copy.deepcopy(self.session)
                    if command:self.busy=False;self.operation_done.set()
            self.closed.wait(.5)
        self.stop()

    def close(self):
        self.closed.set()
        if self.thread.is_alive():self.thread.join(15)
