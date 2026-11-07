"""Fixed starting-heading line in Cube-local XY, never a rotating body line."""
import math


def straight_command(start, position, velocity, heading):
    forward = (math.cos(heading), math.sin(heading))
    right = (-forward[1], forward[0])
    displacement = (position[0]-start[0], position[1]-start[1])
    along = sum(a*b for a,b in zip(displacement, forward))
    cross = sum(a*b for a,b in zip(displacement, right))
    sideways = sum(a*b for a,b in zip(velocity[:2], right))
    correction = max(-.06, min(.06, -.15*cross-.4*sideways))
    speed = min(.1, max(0., .2*(10.-along)))
    return along, cross, (speed*forward[0]+correction*right[0],
                          speed*forward[1]+correction*right[1], heading)
