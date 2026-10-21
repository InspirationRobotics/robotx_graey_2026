"""Bounded surface setpoints: horizontal velocity and upward-only NED Z."""
import math


def surface_velocity(target, local, velocity, depth):
    values=(*target[:2],*local[:2],*velocity[:2],depth)
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
        raise ValueError('Finite surface control measurements required')
    # Slow down near the point; velocity feedback damps momentum.
    horizontal=[.4*(target[i]-local[i])-.6*velocity[i] for i in (0,1)]
    speed=math.hypot(*horizontal)
    if speed>.2:horizontal=[v*.2/speed for v in horizontal]
    # NED positive Z is down. Inside the surface pressure deadband, request
    # zero vertical speed; an impossible continuous ascent at the air/water
    # boundary can keep driving the Cube Z controller upward. Correct only
    # confirmed depth beyond 3 cm, without ever requesting descent.
    # Zero velocity is still closed-loop control, not a neutral motor command.
    down=-min(.05,max(0.,depth-.03)*.5)
    return (*horizontal,down)


class SurfaceLeg:
    """Turn first, then follow the frozen start-to-waypoint line in N/E.

    Position/velocity feedback comes from Cube's bridge-aided estimate. This
    controller never interprets a desired bearing as a sensor measurement.
    """
    def __init__(self):
        self.last=None
        self.command_yaw=None
        self.aligned_since=None
        self.tracking=False
        self.state=dict(stage='waiting',reason='Waiting for surface transit')

    def step(self,now,start,target,local,velocity,depth,yaw):
        values=(now,*start[:2],*target[:2],*local[:2],*velocity[:2],depth,yaw)
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
            raise ValueError('Finite surface leg measurements required')
        delta=tuple(target[i]-start[i] for i in (0,1));length=math.hypot(*delta)
        if length<.3:raise ValueError('Surface leg must have a distinct target')
        unit=tuple(v/length for v in delta)
        projection=sum((local[i]-start[i])*unit[i] for i in (0,1))
        nearest=tuple(start[i]+projection*unit[i] for i in (0,1))
        cross=math.dist(local[:2],nearest)
        lookahead=min(length,max(0.,projection)+1.5)
        aim=tuple(start[i]+lookahead*unit[i] for i in (0,1))
        distance=math.dist(local[:2],target[:2])
        desired=math.atan2(aim[1]-local[1],aim[0]-local[0])
        if distance<.5:
            desired=self.command_yaw if self.command_yaw is not None else yaw
        error=(desired-yaw+math.pi)%(2*math.pi)-math.pi
        dt=min(.2,max(0.,now-self.last)) if self.last is not None else .1
        self.last=now
        if self.command_yaw is None:self.command_yaw=yaw
        change=(desired-self.command_yaw+math.pi)%(2*math.pi)-math.pi
        self.command_yaw+=(max(-math.radians(15)*dt,min(math.radians(15)*dt,change)))
        self.command_yaw=(self.command_yaw+math.pi)%(2*math.pi)-math.pi
        if abs(error)>math.radians(40):self.tracking=False;self.aligned_since=None
        if not self.tracking:
            if abs(error)<=math.radians(15):
                self.aligned_since=now if self.aligned_since is None else self.aligned_since
                if now-self.aligned_since>=.7:self.tracking=True
            else:self.aligned_since=None
        if not self.tracking:
            horizontal=surface_velocity(local,local,velocity,depth)
            self.state=dict(stage='aligning',reason='Turning toward waypoint before forward travel',
                            cross_track_m=cross,heading_error_deg=math.degrees(error))
            return horizontal,self.command_yaw
        # Along-track progress plus cross-track feedback keeps the commanded
        # earth-frame motion near the original line instead of a frozen heading.
        remaining=max(0.,length-projection)
        forward=min(.2,.4*remaining)*max(0.,math.cos(error))**2
        command=[forward*unit[i]-.18*(local[i]-nearest[i])-.35*velocity[i] for i in (0,1)]
        if distance<.5 or projection>=length:
            command=list(surface_velocity(target,local,velocity,depth)[:2])
        speed=math.hypot(*command)
        if speed>.2:command=[v*.2/speed for v in command]
        down=surface_velocity(local,local,velocity,depth)[2]
        self.state=dict(stage='tracking',reason='Tracking GPS-aided position with DVL velocity feedback',
                        cross_track_m=cross,heading_error_deg=math.degrees(error),
                        along_track_m=projection,target_distance_m=distance)
        return (*command,down),self.command_yaw
