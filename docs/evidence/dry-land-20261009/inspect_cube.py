import subprocess
code = r'''
import json, time, urllib.request, os
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
with urllib.request.urlopen('http://127.0.0.1:8090/api/controller',timeout=4) as r:
    safety=json.load(r)
assert safety['armed'] is False and safety['vehicle_fresh'] and not safety['mission_claimed'], safety
m=mavutil.mavlink_connection('udpout:127.0.0.1:14554',source_system=255,source_component=199)
m.mav.heartbeat_send(18,8,0,0,0)
names=['AHRS_EKF_TYPE','EK3_ENABLE','EK3_SRC1_POSXY','EK3_SRC1_VELXY','EK3_SRC1_YAW',
       'EK3_SRC2_POSXY','EK3_SRC2_VELXY','EK3_SRC2_YAW','EK3_SRC1_POSZ','EK3_SRC2_POSZ',
       'EK3_SRC_OPTIONS','VISO_TYPE','VISO_ORIENT','AHRS_ORIENTATION','GPS_TYPE',
       'GPS1_TYPE','SERIAL4_PROTOCOL','SERIAL4_BAUD','SCR_ENABLE','LOG_DISARMED',
       'EK3_POSNE_M_NSE','EK3_VELNE_M_NSE','EK3_GPS_CHECK']
params={}; last={}; counts={}
end=time.monotonic()+20; retry=0
while time.monotonic()<end:
    if time.monotonic()>retry:
        for n in names:
            if n not in params: m.mav.param_request_read_send(1,1,n.encode(),-1)
        retry=time.monotonic()+5
    msg=m.recv_match(blocking=True,timeout=.2)
    if msg is None or msg.get_srcSystem()!=1 or msg.get_srcComponent()!=1: continue
    k=msg.get_type(); counts[k]=counts.get(k,0)+1
    if k=='HEARTBEAT' and msg.base_mode & 128: raise RuntimeError('Vehicle armed; stopping readback')
    if k=='PARAM_VALUE' and msg.param_id.rstrip('\x00') in names: params[msg.param_id.rstrip('\x00')]=msg.param_value
    if k in ('EKF_STATUS_REPORT','LOCAL_POSITION_NED','ATTITUDE','GPS_RAW_INT'): last[k]=msg.to_dict()
    if k=='NAMED_VALUE_FLOAT' and msg.name.rstrip('\x00')=='NAV_SRC': last['NAV_SRC']=msg.to_dict()
m.close()
print(json.dumps(dict(parameters=params,missing_parameters=[n for n in names if n not in params],counts=counts,last=last)))
'''
r=subprocess.run(['docker','exec','-i','graey','python3','-'],input=code,text=True,capture_output=True,timeout=35)
print(r.stdout,r.stderr,flush=True)
raise SystemExit(r.returncode)
