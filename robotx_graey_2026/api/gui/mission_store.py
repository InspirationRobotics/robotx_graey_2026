"""Named GRAEY JSON routes, retaining Crusader's vehicle-owned mission library."""
import json
import re
import threading
from pathlib import Path
from ..navigation.route_plan import validate_route


def validate_mission(value):
    if not isinstance(value,dict) or value.get('kind') not in ('waypoints','pipeline','simple_sequence','radio_submerge'):
        raise ValueError('Select Waypoints, Pipeline Survey, 5–2–5 or Radio submerge test')
    kind=value['kind']
    if type(value.get('competition',False)) is not bool:raise ValueError('competition must be boolean')
    result=dict(kind=kind,competition=value.get('competition',False))
    if kind in ('simple_sequence','radio_submerge'):return result
    route=validate_route(dict(points=value.get('points'),recording_mode='manual'))
    kinds=[p['kind'] for p in route['points']]
    if kind=='waypoints' and any(k!='waypoint' for k in kinds):raise ValueError('Waypoints mission accepts waypoint points only')
    if kind=='pipeline' and kinds!=['pipeline_start','pipeline_end']:
        raise ValueError('Pipeline Survey needs a start and an end')
    result['points']=route['points']
    return result


class MissionStore:
    def __init__(self,root):self.root=Path(root);self.lock=threading.Lock()
    def path(self,name):
        if not isinstance(name,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',name):raise ValueError('Use 1–64 letters, digits, underscores or hyphens for mission names')
        path=(self.root/(name+'.json')).resolve()
        if path.parent!=self.root.resolve():raise ValueError('Mission path escapes library')
        return path
    def list(self):
        with self.lock:
            return [dict(name=p.stem,modified=p.stat().st_mtime) for p in sorted(self.root.glob('*.json')) if p.is_file()]
    def get(self,name):
        with self.lock:return validate_mission(json.loads(self.path(name).read_text(encoding='utf-8')))
    def save(self,name,value):
        mission=validate_mission(value)
        with self.lock:
            self.root.mkdir(parents=True,exist_ok=True);path=self.path(name)
            tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(mission,indent=2),encoding='utf-8');tmp.replace(path)
        return mission
    def delete(self,name):
        with self.lock:self.path(name).unlink()
