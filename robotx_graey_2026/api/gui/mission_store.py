"""Named GRAEY JSON routes, retaining Crusader's vehicle-owned mission library."""
import json
import re
import threading
from pathlib import Path
from ..navigation.route_plan import validate_route
from ..comms.mission_radio import revision


def validate_mission(value):
    choices=('waypoints','pipeline','simple_sequence','survey','dive_here',
             'pool_dive','surface_route','dock_pipeline_dock')
    if not isinstance(value,dict) or value.get('kind') not in choices:
        raise ValueError('Select a supported GRAEY mission')
    kind=value['kind']
    if type(value.get('competition',False)) is not bool:raise ValueError('competition must be boolean')
    result=dict(kind=kind,competition=value.get('competition',False))
    if kind in ('simple_sequence','dive_here','pool_dive'):
        if value.get('points') not in (None,[]):raise ValueError(kind+' does not accept geographic points')
        if kind!='simple_sequence':result['dive']=_dive_settings(kind,value.get('dive'))
        return result
    route=validate_route(dict(points=value.get('points'),recording_mode='manual'))
    kinds=[p['kind'] for p in route['points']]
    if kind in ('waypoints','survey') and any(k!='waypoint' for k in kinds):raise ValueError('Waypoints mission accepts waypoint points only')
    # Ruth's survey run (Nov 11): the path, then the start buoy and the end buoy
    if kind=='survey' and len(kinds)<2:raise ValueError('Survey run needs at least 2 points: the start and end buoys last')
    if kind=='pipeline' and kinds!=['pipeline_start','pipeline_end']:
        raise ValueError('Pipeline Survey needs a start and an end')
    if kind in ('surface_route','dock_pipeline_dock') and len(kinds)>15:
        raise ValueError('Chris-derived radio routes accept at most 15 points')
    result['points']=route['points']
    if kind=='dock_pipeline_dock':result['dive']=_dive_settings(kind,value.get('dive'))
    return result


def _dive_settings(kind,value):
    import math
    if not isinstance(value,dict) or set(value)-{'depth_m','altitude_m','max_depth_m'}:
        raise ValueError('Invalid dive settings')
    out={}
    for key,item in value.items():
        if isinstance(item,bool) or not isinstance(item,(int,float)) or not math.isfinite(item) or item<=0:
            raise ValueError('Invalid '+key)
        out[key]=float(item)
    if kind=='pool_dive':
        if set(out)!={'depth_m'}:raise ValueError('Pool Dive requires pressure depth only')
    else:
        out.setdefault('altitude_m',3.5)
        if 'max_depth_m' not in out or 'depth_m' in out:
            raise ValueError('DVL dive requires altitude and maximum depth')
    return out


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
    def load(self,value):
        """Stage an operator mission without starting or preparing it."""
        mission=validate_mission(value);rev=revision(mission)
        with self.lock:
            folder=self.root/'.selected';folder.mkdir(parents=True,exist_ok=True)
            path=folder/'mission.json';tmp=folder/'mission.tmp'
            tmp.write_text(json.dumps(dict(revision=rev,mission=mission),indent=2),encoding='utf-8')
            tmp.replace(path)
        return dict(revision=rev,kind=mission['kind'],point_count=len(mission.get('points',[])),mission=mission)
    def selected(self):
        with self.lock:
            path=self.root/'.selected'/'mission.json'
            if not path.exists():return None
            value=json.loads(path.read_text(encoding='utf-8'))
        mission=validate_mission(value.get('mission'))
        return dict(revision=revision(mission),kind=mission['kind'],point_count=len(mission.get('points',[])),mission=mission)
    def delete(self,name):
        with self.lock:self.path(name).unlink()
