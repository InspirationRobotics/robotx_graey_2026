"""Pre-mission stability measurements, not a guarantee of geographic accuracy."""
import math
from collections import deque


class SurfaceMeasurement:
    def __init__(self, duration=20.):
        self.duration=duration
        self.rows=deque()
        self.last_stamp=None
        self.last_received=None
        self.distance=0.
        self.motion_stamp=None
        self.motion_speed=0.

    def update(self, now, bridge):
        fix=bridge.get('gps_observation') or {}
        dvl=bridge.get('heading_dvl') or {}
        heading=bridge.get('heading_vn') or {}
        admitted=bridge.get('gps_admission') or {}
        velocity=dvl.get('raw_velocity',dvl.get('velocity')) or []
        position=bridge.get('position_ned') or []
        values=[fix.get(k) for k in ('lat','lon','accuracy')]+velocity+[heading.get('rate')]+position
        good=(len(velocity)==3 and len(position)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in values)
              and all(0<=now-x.get('received',-math.inf)<.5 for x in (fix,dvl,heading))
              and bridge.get('healthy') is True and bridge.get('surfaced') is True
              and bridge.get('gps_eligible') is True and admitted.get('qualified') is True
              and not (bridge.get('dvl_continuity') or {}).get('recovering')
              and dvl.get('valid') is True and 0<fix.get('accuracy',math.inf)<=2.
              and math.sqrt(sum(v*v for v in velocity))<=.05
              and abs(heading['rate'])<=2.)  # bridge reports degrees/second
        reason='Hold stationary at the surface with qualified GPS (accuracy ≤2 m), DVL and VN'
        if not good:
            self.rows.clear();self.last_stamp=self.last_received=None
            self.motion_stamp=None
            return dict(ready=False,elapsed_s=0.,required_s=self.duration,reason=reason)
        speed=math.sqrt(sum(v*v for v in velocity))
        if self.motion_stamp is not None and 0<dvl['received']-self.motion_stamp<.5:
            self.distance+=max(speed,self.motion_speed)*(dvl['received']-self.motion_stamp)
        self.motion_stamp,self.motion_speed=dvl['received'],speed
        stamp=fix.get('cube_fix_time_us')
        if self.last_received is not None and now-self.last_received>.5:
            self.rows.clear()
        if stamp!=self.last_stamp:
            self.last_stamp,self.last_received=stamp,now
            self.rows.append((now,fix['lat'],fix['lon'],self.distance))
        while len(self.rows)>1 and now-self.rows[1][0]>=self.duration:
            self.rows.popleft()
        if not self.rows:
            return dict(ready=False,elapsed_s=0.,required_s=self.duration,reason=reason)
        first=self.rows[0];lat,lon=first[1:3]
        offsets=[((r[1]-lat)*111320.,(r[2]-lon)*111320.*math.cos(math.radians(lat))) for r in self.rows]
        mean=tuple(sum(p[i] for p in offsets)/len(offsets) for i in (0,1))
        scatter=max(math.hypot(p[0]-mean[0],p[1]-mean[1]) for p in offsets)
        # DVL-only path length is independent of GPS and frame heading. Never
        # use GPS-aided bridge displacement to validate the same receiver.
        disagreement=max(max(0.,math.hypot(*p)-(r[3]-first[3])) for p,r in zip(offsets,self.rows))
        elapsed=now-first[0]
        ready=elapsed>=self.duration and len(self.rows)>=40 and scatter<=1.5 and disagreement<=2.
        if scatter>1.5 or disagreement>2.:
            reason='GPS scatter or GPS/DVL displacement disagreement excessive; wait for a stable window'
        else:
            reason=f'Surface measurement {min(elapsed,self.duration):.1f}/{self.duration:.0f} s; scatter {scatter:.2f} m'
        return dict(ready=ready,elapsed_s=elapsed,required_s=self.duration,
                    scatter_radius_m=scatter,gps_dvl_disagreement_m=disagreement,reason=reason)
