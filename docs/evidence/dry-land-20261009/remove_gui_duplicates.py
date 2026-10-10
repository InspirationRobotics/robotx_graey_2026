"""Stop only verified GUI-owned duplicate navigation children, keep core owners."""
import subprocess
code = r'''
import json,os,signal,time,urllib.request
from pathlib import Path
def guard():
    with urllib.request.urlopen('http://127.0.0.1:8090/api/controller',timeout=4) as r:c=json.load(r)
    assert c.get('vehicle_fresh') and c.get('armed') is False and c.get('sa_active') is True,c
    assert c.get('esc_commanded_on') is False and not c.get('mission_claimed') and not c.get('autonomous'),c
def table():
    rows={}
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():continue
        try:
            args=[a.decode() for a in (d/'cmdline').read_bytes().split(b'\0') if a]
            fields=(d/'stat').read_text().rsplit(')',1)[1].split()
            rows[int(d.name)]=dict(args=args,ppid=int(fields[1]))
        except (OSError,ValueError):continue
    return rows
def executable(row,name):
    return any(a.endswith('/lib/robotx_graey_2026/'+name) for a in row['args'])
guard();rows=table();core={};extra={}
for name in ('vn100_node','dvl_node','nav_ekf_bridge'):
    owners=[(pid,r) for pid,r in rows.items() if executable(r,name)]
    assert len(owners)==2,(name,owners)
    for pid,r in owners:
        parent=rows.get(r['ppid'],{})
        args=parent.get('args',[])
        if 'launch' in args and 'core.launch.py' in args and 'robotx_graey_2026' in args:
            assert name not in core;core[name]=pid
        elif 'run' in args and name in args and executable(rows.get(parent.get('ppid'),{'args':[]}), 'gui_node'):
            assert name not in extra;extra[name]=(pid,r['ppid'])
        else:raise RuntimeError('Unexpected process owner: '+str((pid,r,parent)))
assert len(core)==len(extra)==3
print('VERIFIED_OWNERS',json.dumps(dict(retain_core=core,stop_gui=extra)),flush=True)
for name,(pid,wrapper) in extra.items():
    guard()
    current=table()
    assert current.get(pid)==rows[pid] and current.get(wrapper)==rows[wrapper]
    os.kill(pid,signal.SIGINT)
deadline=time.monotonic()+8
while time.monotonic()<deadline:
    current=table()
    if all(pid not in current and wrapper not in current for pid,wrapper in extra.values()):break
    time.sleep(.2)
current=table()
for name,(pid,wrapper) in extra.items():
    assert pid not in current, 'Child did not exit: '+str(pid)
    if wrapper in current:
        state=Path('/proc',str(wrapper),'stat').read_text().rsplit(')',1)[1].split()[0]
        if state=='Z':continue  # Exited ros2 wrapper awaiting GUI reaping; no live node.
        assert current[wrapper]==rows[wrapper]
        os.kill(wrapper,signal.SIGTERM)
time.sleep(2);guard();current=table()
for name,pid in core.items():
    assert current.get(pid)==rows[pid], 'Core owner changed: '+name
    assert [p for p,r in current.items() if executable(r,name)]==[pid], 'Duplicate remains: '+name
print('CLEANUP_CONFIRMED',json.dumps(core),flush=True)
'''
r=subprocess.run(['docker','exec','-i','graey','python3','-'],input=code,text=True,capture_output=True,timeout=25)
print(r.stdout,r.stderr,flush=True)
if r.returncode:raise SystemExit(r.returncode)
