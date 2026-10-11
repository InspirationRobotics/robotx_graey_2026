"""Navigation scheduling follows elapsed time, independently of UTC adjustments.

The node's ROS clock and message timestamps are NOT replaced. Callers that
explicitly supply a timer clock retain it. Synthetic SITL sensor ordering uses
steady_clock explicitly because its consumers need elapsed time, not UTC.
"""
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class SteadyNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

    def create_timer(self, timer_period_sec, callback, callback_group=None, clock=None, **kwargs):
        return super().create_timer(timer_period_sec, callback,
            callback_group=callback_group,
            clock=self.steady_clock if clock is None else clock, **kwargs)
