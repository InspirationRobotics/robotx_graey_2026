"""One-time orientation before position-only GUIDED travel."""
import math


class GuidedOrientation:
    def __init__(self,hold,target):
        self.hold=tuple(hold)
        self.heading=math.atan2(target[1]-hold[1],target[0]-hold[0])

    def ready(self,yaw):
        error=(self.heading-yaw+math.pi)%(2*math.pi)-math.pi
        return abs(error)<=math.radians(3.)

    def target(self):
        return (*self.hold,self.heading)
