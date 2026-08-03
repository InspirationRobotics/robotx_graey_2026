#!/usr/bin/env python3
"""Frame conversions shared by the missions and the position server."""
import math


def body_to_world(n, e, yaw, forward, right):
    """A (forward, right) offset from a pose at (n, e, yaw) -> absolute NED."""
    c, s = math.cos(yaw), math.sin(yaw)
    return n + forward * c - right * s, e + forward * s + right * c
