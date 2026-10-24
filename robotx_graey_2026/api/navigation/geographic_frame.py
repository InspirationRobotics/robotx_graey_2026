"""One Jetson x=north/y=east map over a fixed Cube local XY frame.

GPS height is deliberately absent. Pressure/Cube local-down remain independent.
No transform update changes Cube odometry or its yaw reference.
"""
import math
from .waypoint_logic import geo_to_ne, ne_to_geo


def rotation(xy, degrees):
    a=math.radians(degrees);c,s=math.cos(a),math.sin(a)
    return (c*xy[0]-s*xy[1],s*xy[0]+c*xy[1])


class GeographicFrame:
    def __init__(self):
        self.origin=None
        self.angle=None
        self.translation=(0.,0.)
        self.revision=0

    def align(self, lat, lon, local, angle):
        if not all(math.isfinite(v) for v in (lat,lon,*local[:2],angle)):
            raise ValueError('Finite geographic registration required')
        if self.origin is None:self.origin=(lat,lon)
        point=geo_to_ne(lat,lon,self.origin)
        rotated=rotation(local,angle)
        self.angle=angle
        self.translation=tuple(point[i]-rotated[i] for i in (0,1))
        self.revision+=1

    def to_map(self, local):
        if self.angle is None:raise ValueError('Geographic north not yet established')
        p=rotation(local,self.angle)
        return tuple(p[i]+self.translation[i] for i in (0,1))

    def to_local(self, north_east):
        if self.angle is None:raise ValueError('Geographic north not yet established')
        return rotation(tuple(north_east[i]-self.translation[i] for i in (0,1)),-self.angle)

    def gps_to_local(self, lat, lon):
        return self.to_local(geo_to_ne(lat,lon,self.origin))

    def to_geo(self, local):
        return ne_to_geo(*self.to_map(local),self.origin)

    def correct_position(self, lat, lon, local, maximum_step=.1):
        residual=tuple(a-b for a,b in zip(geo_to_ne(lat,lon,self.origin),self.to_map(local)))
        error=math.hypot(*residual)
        # Only the map registration changes; DVL metres and Cube XY do not.
        gain=min(.2,maximum_step/max(error,1e-9))
        self.translation=tuple(self.translation[i]+gain*residual[i] for i in (0,1))
        return error

    def correct_angle(self, step, local):
        # Preserve the current map position when refining north orientation.
        anchor=self.to_map(local)
        self.angle=(self.angle+step+180)%360-180
        p=rotation(local,self.angle)
        self.translation=tuple(anchor[i]-p[i] for i in (0,1))

    def snapshot(self):
        return dict(ready=self.angle is not None,origin=self.origin,
            local_to_ne_deg=self.angle,translation_ne=self.translation,
            revision=self.revision,axes='x=north, y=east',units='metres')

    @classmethod
    def restore(cls, value):
        result=cls()
        if not value or not value.get('ready'):return result
        result.origin=tuple(value['origin']);result.angle=float(value['local_to_ne_deg'])
        result.translation=tuple(value['translation_ne']);result.revision=value['revision']
        if not all(math.isfinite(v) for v in (*result.origin,result.angle,*result.translation)):
            raise ValueError('Invalid geographic transform')
        return result
