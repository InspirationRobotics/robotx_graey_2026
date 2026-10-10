import subprocess
code = r'''
import json,time,os,urllib.request,math
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
def guard():
    with urllib.request.urlopen('http://127.0.0.1:8090/api/controller',timeout=4) as r:c=json.load(r)
    assert c['vehicle_fresh'] and c['armed'] is False and not c['mission_claimed'],c
guard()
m=mavutil.mavlink_connection('udpout:127.0.0.1:14554',source_system=255,source_component=199)
m.mav.heartbeat_send(18,8,0,0,0)
names=['AHRS_EKF_TYPE','AHRS_OPTIONS','AHRS_ORIENTATION','AHRS_TRIM_X','AHRS_TRIM_Y',
 'EK3_SRC1_YAW','EK3_SRC2_YAW','EK3_SRC3_YAW','EK3_MAG_CAL','EK3_MAG_MASK','EK3_IMU_MASK',
 'COMPASS_USE','COMPASS_USE2','COMPASS_USE3','COMPASS_DEV_ID','COMPASS_DEV_ID2','COMPASS_DEV_ID3',
 'COMPASS_PRIO1_ID','COMPASS_PRIO2_ID','COMPASS_PRIO3_ID','COMPASS_EXTERNAL','COMPASS_EXTERN2',
 'COMPASS_EXTERN3','COMPASS_ORIENT','COMPASS_ORIENT2','COMPASS_ORIENT3','COMPASS_DEC',
 'COMPASS_AUTODEC','LOG_DISARMED']
params={};last={};counts={};logs={};text=[];latest=None
m.mav.log_request_list_send(1,1,0,0)
end=time.monotonic()+22;retry=0
while time.monotonic()<end:
    if time.monotonic()>retry:
        guard()
        for n in names:
            if n not in params:m.mav.param_request_read_send(1,1,n.encode(),-1)
        for mid in (148,178,182):m.mav.command_long_send(1,1,512,0,mid,0,0,0,0,0,0)
        retry=time.monotonic()+6
    x=m.recv_match(blocking=True,timeout=.2)
    if x is None or x.get_srcSystem()!=1 or x.get_srcComponent()!=1:continue
    k=x.get_type();counts[k]=counts.get(k,0)+1
    if k=='HEARTBEAT' and x.base_mode&128:raise RuntimeError('Armed; stopped')
    if k=='PARAM_VALUE' and x.param_id.rstrip('\x00') in names:params[x.param_id.rstrip('\x00')]=x.param_value
    if k in ('ATTITUDE','AHRS2','AHRS3','AUTOPILOT_VERSION','EKF_STATUS_REPORT','SYS_STATUS'):last[k]=x.to_dict()
    if k=='STATUSTEXT':text.append(x.text)
    if k=='LOG_ENTRY':
        logs[x.id]=x.to_dict()
        if latest is None:
            latest=x.last_log_num;m.mav.log_request_list_send(1,1,latest,latest)
m.close()
print(json.dumps(dict(parameters=params,missing=[n for n in names if n not in params],
    last=last,logs=logs,counts=counts,text=text)),flush=True)
'''
r=subprocess.run(['docker','exec','-i','graey','python3','-'],input=code,text=True,capture_output=True,timeout=40)
print(r.stdout,r.stderr,flush=True)
raise SystemExit(r.returncode)
