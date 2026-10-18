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
    # NED positive Z is down. Never request descent or a positive-depth target.
    # Gentle ascent at surface, strengthened by pressure if sinking. This is
    # a velocity request, not proof of physical thrust/motor polarity.
    down=-min(.10,.03+max(0.,depth)*.8)
    return (*horizontal,down)
