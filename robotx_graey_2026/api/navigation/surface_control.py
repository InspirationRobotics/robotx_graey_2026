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
