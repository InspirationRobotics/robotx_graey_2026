"""Authorized disarmed-only +180 degree startup test; restore source in finally.

Run on Jetson host via SSH. Only the VN process is signalled. Installed source
is backed up and restored; the new node retains its cached startup parameter.
"""
import subprocess

code = r'''
import hashlib,json,os,signal,time,tempfile,urllib.request
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters

def api(name):
    with urllib.request.urlopen('http://127.0.0.1:8090/api/'+name,timeout=4) as r:
        return json.load(r)
def guard():
    c=api('controller')
    assert c.get('vehicle_fresh') and c.get('armed') is False, c
    assert c.get('esc_commanded_on') is False and c.get('sa_active') is True, c
    assert not c.get('mission_claimed') and not c.get('autonomous'), c
    return c
def pids(name):
    result=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try: args=(p/'cmdline').read_bytes().split(b'\0')
        except OSError: continue
        if any(a.endswith(('/lib/robotx_graey_2026/'+name).encode()) for a in args):
            result.append(int(p.name))
    return sorted(result)

path=Path('/root/robotx_ws/install/robotx_graey_2026/lib/python3.10/site-packages/robotx_graey_2026/api/navigation/vn100_node.py')
original=path.read_bytes()
expected='b0728413aa7206394cc5f867e41484f5c5b8f02fec989cf3381b1ff847f8c413'
assert hashlib.sha256(original).hexdigest()==expected, 'Installed VN source changed; inspect first'
old=b"self.declare_parameter('yaw_offset_deg', 0.0)"
assert original.count(old)==1
patched=original.replace(old,b"self.declare_parameter('yaw_offset_deg', 180.0)")
compile(patched,str(path),'exec')
before=pids('vn100_node'); bridge=pids('nav_ekf_bridge')
assert len(before)==len(bridge)==1, (before,bridge)
guard()
nav=api('navigation')['streams']
assert nav['supervisor']['data']['mode']=='Observation'
assert nav['supervisor']['data']['navigation_ready'] is False
assert not nav.get('dvl_valid',{}).get('fresh') or nav['dvl_valid']['data']['valid'] is False
backup=Path(tempfile.mkdtemp(prefix='graey-vn-offset-'))/'vn100_node.py.original'
backup.write_bytes(original)
print('BACKUP',str(backup), 'VN_PID',before, 'BRIDGE_PID',bridge,flush=True)
rclpy.init(); node=rclpy.create_node('temporary_vn_offset_readback')
client=node.create_client(GetParameters,'/vn100_node/get_parameters')
applied=False
try:
    guard()
    path.write_bytes(patched)
    os.kill(before[0],signal.SIGINT)
    end=time.monotonic()+25
    while time.monotonic()<end:
        guard()
        current=pids('vn100_node')
        if len(current)==1 and current!=before and client.wait_for_service(timeout_sec=.5):
            req=GetParameters.Request();req.names=['yaw_offset_deg']
            f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=1)
            if f.done() and f.result() and f.result().values[0].double_value==180.0:
                applied=True;break
        time.sleep(.5)
    assert applied, 'VN did not restart with confirmed +180 offset before deadline'
finally:
    if path.read_bytes()==patched:
        path.write_bytes(original)
    else:
        raise RuntimeError('Concurrent source change: not overwriting; original backup at '+str(backup))
    node.destroy_node();rclpy.shutdown()
    print('SOURCE_RESTORED',hashlib.sha256(path.read_bytes()).hexdigest()==expected,flush=True)
assert pids('nav_ekf_bridge')==bridge, 'Bridge process changed independently during test'
time.sleep(2)
nav=api('navigation')['streams']
assert nav['vn_heading']['fresh'], 'Offset parameter confirmed but heading not fresh'
print('RESULT',json.dumps(dict(offset_deg=180, temporary_until_vn_restart=True,
    vn_pid=pids('vn100_node'),bridge_pid=pids('nav_ekf_bridge'),controller=guard(),
    streams={k:nav[k] for k in ('vn_heading','attitude','supervisor') if k in nav})),flush=True)
'''
r=subprocess.run(['docker','exec','-i','graey','bash','-lc',
    'source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && python3 -'],
    input=code,text=True,capture_output=True,timeout=60)
print(r.stdout,r.stderr,flush=True)
raise SystemExit(r.returncode)
