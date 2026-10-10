import json, time, urllib.request
from pathlib import Path
from datetime import datetime, timezone

def get(p):
    with urllib.request.urlopen('http://192.168.2.2:8090/api/'+p, timeout=3) as r:
        return json.load(r)

path=Path(__file__).with_name('manual-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'.jsonl')
start=time.monotonic()
with path.open('x') as f:
    print('CAPTURE',path,flush=True)
    while time.monotonic()-start < 65:
        c=get('controller')
        if not (c.get('vehicle_fresh') and c.get('armed') is False and c.get('sa_active') is True and c.get('esc_commanded_on') is False and not c.get('mission_claimed')):
            print('STOP_SAFETY',json.dumps(c),flush=True);break
        n=get('navigation')['streams']
        f.write(json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),t=time.monotonic()-start,streams=n))+'\n');f.flush()
        if time.monotonic()-start < 1:
            print('SAFE_DVL',json.dumps(n.get('dvl_valid')),flush=True)
        time.sleep(.2)
print('DONE',flush=True)
