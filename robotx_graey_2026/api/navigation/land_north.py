"""Session-only known-bearing capture, independent of pressure and DVL."""
import math
from collections import deque
from .bridge_estimator import yaw, wrap


class LandNorth:
    def __init__(self):
        self.rows = deque(maxlen=1024)
        self.reference = None

    def invalidate(self):
        self.reference = None
        self.rows.clear()

    def observe(self, now, q, rates):
        if self.rows and not 0 < now-self.rows[-1][0] < .5:
            self.invalidate()
        if self.rows:
            dt=now-self.rows[-1][0]
            angle=math.degrees(2*math.acos(min(1.,abs(sum(a*b for a,b in zip(q,self.rows[-1][1]))))))
            if angle>5+math.degrees(max(math.sqrt(sum(v*v for v in rates)),math.sqrt(sum(v*v for v in self.rows[-1][2]))))*dt:
                self.invalidate()
        self.rows.append((now, tuple(q), tuple(rates)))

    def current(self, now):
        if not self.rows or not 0 <= now-self.rows[-1][0] < .5:
            self.invalidate()
        return dict(self.reference,current_heading_deg=(yaw(self.rows[-1][1])+self.reference['offset_deg'])%360) if self.reference else None

    def capture(self, now, heading):
        if isinstance(heading,bool) or not isinstance(heading,(int,float)) or not math.isfinite(heading) or not 0 <= heading < 360:
            raise ValueError('Enter an independently known true bearing from 0 to below 360 degrees')
        rows=[r for r in self.rows if 0 <= now-r[0] <= 2.5]
        if len(rows)<15 or rows[-1][0]-rows[0][0]<2 or now-rows[-1][0]>.25:
            raise ValueError('Hold level and still for two seconds with fresh VN')
        for _,q,rates in rows:
            if (not all(math.isfinite(v) for v in (*q,*rates))
                    or abs(sum(v*v for v in q)-1)>.02
                    or 1-2*(q[1]*q[1]+q[2]*q[2])<math.cos(math.radians(15))
                    or max(abs(math.degrees(v)) for v in rates)>2
                    or abs(wrap(yaw(q)-yaw(rows[-1][1])))>3):
                raise ValueError('Keep the nose on the known bearing, level and stationary')
        self.reference=dict(offset_deg=wrap(heading-yaw(rows[-1][1])),true_heading_deg=heading,
                            captured_monotonic=now,method='Known true bearing captured on land')
        return self.reference
