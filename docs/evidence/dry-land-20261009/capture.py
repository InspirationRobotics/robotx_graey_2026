import json, math, time, urllib.request
from pathlib import Path
from datetime import datetime, timezone

root = Path(__file__).parent
rows = []
def get(path):
    with urllib.request.urlopen('http://192.168.2.2:8090/api/'+path, timeout=4) as r:
        return json.load(r)
with (root/'stationary.jsonl').open('w', encoding='utf-8') as f:
    for i in range(60):
        c, n = get('controller'), get('navigation')
        row = dict(utc=datetime.now(timezone.utc).isoformat(), controller=c, streams=n['streams'])
        f.write(json.dumps(row)+'\n'); f.flush()
        rows.append(row)
        if c.get('armed') is not False or not c.get('vehicle_fresh') or c.get('mission_claimed'):
            print('Capture stopped: controller no longer confirms fresh disarmed idle state', flush=True)
            break
        time.sleep(1)
gps = [r['streams']['gps']['data'] for r in rows if r['streams'].get('gps', {}).get('fresh')]
summary = dict(samples=len(rows), start=rows[0]['utc'], end=rows[-1]['utc'],
               controller=rows[-1]['controller'], streams=rows[-1]['streams'])
if gps:
    lat=sum(x['lat'] for x in gps)/len(gps); lon=sum(x['lon'] for x in gps)/len(gps)
    summary['gps_summary'] = dict(mean_lat=lat, mean_lon=lon,
        north_range_m=(max(x['lat'] for x in gps)-min(x['lat'] for x in gps))*111320,
        east_range_m=(max(x['lon'] for x in gps)-min(x['lon'] for x in gps))*111320*math.cos(math.radians(lat)),
        reported_accuracy_range_m=[min(x['accuracy'] for x in gps),max(x['accuracy'] for x in gps)])
(root/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary), flush=True)
