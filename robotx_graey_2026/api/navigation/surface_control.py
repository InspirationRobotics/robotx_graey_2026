"""Bounded surface setpoints: horizontal velocity and upward-only NED Z."""
import math


def surface_velocity(target, local, velocity, depth):
    values=(*target[:2],*local[:2],*velocity[:2],depth)
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
        raise ValueError('Finite surface control measurements required')
    # Slow down near the point; velocity feedback damps momentum.
    horizontal=[.4*(target[i]-local[i])-.6*velocity[i] for i in (0,1)]
    speed=math.hypot(*horizontal)
    if speed>.1:horizontal=[v*.1/speed for v in horizontal]
    # NED positive Z is down. Inside the surface pressure deadband, request
    # zero vertical speed; an impossible continuous ascent at the air/water
    # boundary can keep driving the Cube Z controller upward. Correct only
    # confirmed depth beyond 3 cm, without ever requesting descent.
    # Zero velocity is still closed-loop control, not a neutral motor command.
    down=-min(.05,max(0.,depth-.03)*.5)
    return (*horizontal,down)


class SurfaceLeg:
    """Translate and steer together along the start-to-waypoint line in N/E.

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
        if length<.3:
            # Alignment can finish beside the selected point. Brake/hold there
            # while the mission verifies its arrival dwell; do not crash the node.
            self.state=dict(stage='tracking',reason='Holding nearby target',
                target_distance_m=math.dist(local[:2],target[:2]),cross_track_m=0.,
                lookahead_m=0.,yaw_rate_limit_deg_s=0.)
            return surface_velocity(target,local,velocity,depth),yaw
        unit=tuple(v/length for v in delta)
        projection=sum((local[i]-start[i])*unit[i] for i in (0,1))
        nearest=tuple(start[i]+projection*unit[i] for i in (0,1))
        cross=math.dist(local[:2],nearest)
        distance=math.dist(local[:2],target[:2])
        lookahead_distance=min(1.5,max(.25,.5*distance))
        lookahead=min(length,max(0.,projection)+lookahead_distance)
        aim=tuple(start[i]+lookahead*unit[i] for i in (0,1))
        desired=math.atan2(aim[1]-local[1],aim[0]-local[0])
        if distance<1.:
            desired=self.command_yaw if self.command_yaw is not None else yaw
        error=(desired-yaw+math.pi)%(2*math.pi)-math.pi
        dt=min(.2,max(0.,now-self.last)) if self.last is not None else .1
        self.last=now
        if self.command_yaw is None:self.command_yaw=yaw
        change=(desired-self.command_yaw+math.pi)%(2*math.pi)-math.pi
        yaw_rate=math.radians(min(10.,max(1.,3.*distance)))
        self.command_yaw+=(max(-yaw_rate*dt,min(yaw_rate*dt,change)))
        self.command_yaw=(self.command_yaw+math.pi)%(2*math.pi)-math.pi
        self.tracking=True
        # Along-track progress plus cross-track feedback keeps the commanded
        # earth-frame motion near the original line instead of a frozen heading.
        remaining=max(0.,length-projection)
        # N/E translation is independent of nose direction: a vectored sub can
        # move laterally while GUIDED slews yaw. There is no turn-only stage.
        forward=min(.1,.4*remaining)
        command=[forward*unit[i]-.18*(local[i]-nearest[i])-.35*velocity[i] for i in (0,1)]
        if distance<.5 or projection>=length:
            command=list(surface_velocity(target,local,velocity,depth)[:2])
        speed=math.hypot(*command)
        if speed>.1:command=[v*.1/speed for v in command]
        down=surface_velocity(local,local,velocity,depth)[2]
        self.state=dict(stage='tracking',reason='Tracking GPS-aided position with DVL velocity feedback',
                        cross_track_m=cross,heading_error_deg=math.degrees(error),
                        along_track_m=projection,target_distance_m=distance)
        self.state.update(lookahead_m=lookahead_distance,yaw_rate_limit_deg_s=math.degrees(yaw_rate))
        return (*command,down),self.command_yaw


def gentle_horizontal(command, previous, dt):
    """Ramp horizontal commands at 0.04 m/s^2, capped at 0.10 m/s.

    Explicit stops bypass this function so braking is never delayed.
    """
    x,y,z=command
    speed=math.hypot(x,y)
    if speed>.1:x,y=x*.1/speed,y*.1/speed
    dx,dy=x-previous[0],y-previous[1]
    delta=math.hypot(dx,dy);limit=.04*min(.2,max(0.,dt))
    if delta>limit and delta>0:x,y=previous[0]+dx*limit/delta,previous[1]+dy*limit/delta
    return x,y,z
