"""ADAPTED: GRAEY ROS/mission/sensor interfaces -> unchanged Crusader GCS frames.

This owns no serial device and grants no control permission. All writes use the
existing GRAEY backend checks. Common metrics/archive logic is copied Crusader.
"""
import json
import socket
import time
from urllib.parse import urlencode
from . import crusader_system_info as system_info


class FleetAdapter:
    def __init__(self,request):
        self.request=request;self.cpu=system_info.CpuMeter();self.cursor=0;self.generation=None;self.receipts={};self.min_level=20;self.log_replay=False
    def http(self,path,body=None):
        status,kind,data=self.request(dict(method='POST' if body is not None else 'GET',path=path,content_type='application/json'),
                                      b'' if body is None else json.dumps(body).encode())
        obj=json.loads(data)
        if status>=400 or obj.get('ok') is False:raise ValueError(obj.get('msg','GRAEY request refused'))
        return obj
    def frames(self):
        try:
            nav=self.http('/api/navigation');control=self.http('/api/controller');comms=self.http('/api/communications')
            cube=nav.get('cube_estimate',{});streams=nav.get('streams',{});display=nav.get('qgc_pose',cube);p=display if display.get('valid') else {}
            def sample(key,limit=2):
                row=streams.get(key,{})
                return row.get('data',{}) if row.get('age',999)<limit else {}
            corrected=sample('bridge',1)
            if p and corrected.get('heading_reference_validated') and corrected.get('map_heading_deg') is not None:
                p=dict(p,heading_deg=corrected['map_heading_deg'])
            att=sample('attitude',.7).get('rpy',[None,None,None]);gps=sample('gps');m=sample('waypoint')
            live=control.get('online') and control.get('vehicle_fresh');run=comms.get('data',{}).get('ocs_run') if comms.get('fresh') else None
            mission=None if not m else dict(state='WAITING_RUN_START' if m.get('stage')=='WAIT_RUN_START' else m.get('stage'),name=m.get('plan',{}).get('kind','GRAEY mission'),elapsed_s=m.get('elapsed_s'),waiting_s=None,running_node=m.get('stage'),current_task=m.get('current_task'),warning=m.get('reason'),last=m.get('reason'))
            sys=system_info.snapshot(self.cpu)
            state=dict(t='state',stamp=time.time(),boat=dict(lat=p.get('lat'),lon=p.get('lon'),hdg=p.get('heading_deg'),spd=cube.get('speed_mps'),age=streams.get('global',{}).get('age')),att=dict(zip(('roll','pitch','yaw'),att)),
                fcu=dict(mode=control.get('mode') if live else None,armed=control.get('armed') if live else None,age=None if not live else streams.get('heartbeat',{}).get('age')),
                health=None if not live else dict(batt_v=None,batt_pct=None,fix=gps.get('fix'),sats=gps.get('satellites'),hdop=gps.get('hdop'),ekf=(2 if cube.get('local_valid',cube.get('valid')) else 1) if sample('ekf') else 0,ekf_flags=sample('ekf').get('flags'),prearm=0,estop=2 if control.get('sa_active') is True or control.get('esc_commanded_on') is False else 1 if control.get('sa_active') is False and control.get('esc_commanded_on') is True else 0,text='',sev=None,age=streams.get('heartbeat',{}).get('age')),
                mission=mission,ocs_run=run,camera=None,lidar=None,
                sys=dict(cpu=sys.get('cpu_percent'),temp=sys.get('temp_c'),mem_gb=sys.get('mem_used_gb'),mem_total_gb=sys.get('mem_total_gb'),disk_gb=sys.get('disk_free_gb'),uptime=system_info.format_uptime(sys.get('uptime_s'))),record=None,
                power=dict(available=True,reason='',locked=not live or control.get('armed') is not False,lock_reason='Fresh disarmed telemetry required',hostname=socket.gethostname()))
            groups=self.http('/api/groups');items=[]
            for group,protected in [(g,g.get('managed',False)) for g in groups.get('groups',[])]+[(g,True) for g in groups.get('readonly',[])]:
                up=group.get('up',0)>0;items.append(dict(name=group['key'],group='core' if protected else 'devices',protected=protected,note=group.get('label',''),running=up,state='running' if up else 'stopped',detail=str(group.get('instances',{})),stoppable=up and not protected,stop_reason='Core-managed' if protected else '',tail=[],conflicts=[]))
            try:graph=self.http('/api/params').get('graph',[])
            except (OSError,ValueError):graph=[]
            nodes=dict(t='nodes',groups=[dict(id='core',label='Core',hint='Existing GRAEY core ownership'),dict(id='devices',label='Sensors / mission support',hint='GRAEY device owners')],items=items,profiles=[],graph=graph)
            data=comms.get('data',{});wifi=data.get('wifi',{});link=dict(connected=comms.get('fresh') is True and wifi.get('connected') is True,peer=wifi.get('host'),target=str(wifi.get('host') or '')+':'+str(wifi.get('port') or 37564),queued=data.get('pending',0),unreceipted=data.get('pending',0),owed=data.get('pending',0))
            events=[dict(stamp=time.time(),dir='link',kind='link',status='connected' if link['connected'] else 'disconnected',body=link)]
            for report in data.get('reports',[]):
                key=report['sequence'];verdict=report['verdict']
                if self.receipts.get(key)==verdict:continue
                self.receipts[key]=verdict
                events.append(dict(stamp=time.time(),dir='rx' if verdict!='pending' else 'tx',kind=report['kind'],vseq=key,status='queued' if verdict=='pending' else verdict,receipt=verdict!='pending',body=dict(verdict=verdict,seq=report.get('robocommand_sequence'),detail=report.get('detail') or 'Broker receipt; no score/mission acceptance implied',observation_id=report['observation_id'],run_id=report['run_id'])))
            frames=[state,nodes,dict(t='ocs',items=events)]
            try:
                log=self.http('/api/fleet/logs?'+urlencode(dict(after=self.cursor,min_level=self.min_level)));self.cursor=log.get('cursor',0)
                if log.get('items') or self.log_replay:
                    frames.append(dict(t='log',items=log.get('items',[]),replay=self.log_replay));self.log_replay=False
            except (OSError,ValueError):pass
            return frames
        except (OSError,ValueError,TypeError,KeyError):return []

    def operation(self,req):
        op=req.get('op')
        if op=='log_level':
            self.min_level=max(0,min(50,int(req.get('level',20))));self.cursor=0;self.log_replay=True
            return dict(level=self.min_level)
        if op=='missions_list':return dict(names=[m['name'] for m in self.http('/api/missions').get('missions',[])])
        if op=='record_topics':return dict(topics=self.http('/api/recording').get('topics',[]))
        if op=='record':
            topics=req.get('topics',[])
            action='bag_stop' if req.get('kind')=='bag' and req.get('action')=='stop' else req.get('action')
            return self.http('/api/recording',dict(action=action,name='graey_session',camera=req.get('camera',False),sonar=req.get('sonar',False),sensors=bool(topics),topics=topics))
        if op=='record_list':
            s=self.http('/api/recording');session=s.get('session') or {};rows=self.http('/api/recording/sessions').get('sessions',[]);folder=session.get('directory')
            if folder:mount,source,persists=system_info.mount_for(folder)
            else:mount=source=None;persists=False
            return dict(live=dict(session,frames={},error='; '.join(filter(None,[s.get('error'),*session.get('errors',[])]))) if session.get('active') else None,bag=session.get('bag',{}),sessions=[dict(row,size_mb=row.get('size_mb'),frames={},recording=session.get('active') is True and row['name']==session.get('name')) for row in rows],dir=folder or '',persist=dict(persists=persists,mount=mount,source=source))
        if op=='record_delete':return self.http('/api/recording/session',dict(action='delete',name=req.get('name')))
        if op=='node':
            action=req.get('action');name=req.get('name')
            if action not in ('start','stop','restart') or not isinstance(name,str):raise ValueError('Unknown node action')
            if action=='restart':self.http('/api/stop?'+urlencode(dict(group=name)))
            return self.http('/api/'+('stop' if action=='stop' else 'start')+'?'+urlencode(dict(group=name)))
        if op=='power':
            if req.get('hostname')!=socket.gethostname():raise ValueError('Hostname confirmation does not match GRAEY')
            return self.http('/api/power?'+urlencode(dict(do=req.get('verb',''))))
        if op=='params_list':return self.http('/api/params?'+urlencode(dict(node=req.get('node',''))))
        if op=='params_set':return self.http('/api/params',dict(node=req.get('node'),values=req.get('values')))
        if op=='ocs_tiers':
            value=req.get('tiers')
            if isinstance(value,str):value=json.loads(value)
            return self.http('/api/communications/tiers',dict(tiers=value,mission=req.get('mission','GRAEY')))
        if op in ('obstacles','planner_profiles_list'):return dict(points=[],profiles=[],sources=[])
        if op=='run_release':raise ValueError('GRAEY competition missions require verified RunStart; no bench bypass')
        if op=='radio_test':raise ValueError('GRAEY RFD application integration is deferred')
        raise ValueError('Not a GRAEY mission/sensor operation: '+str(op))
