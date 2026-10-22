"""Explicitly launched surface maneuver. Shared MAVProxy, no Cube serial access."""
import json
import math
import time
from std_msgs.msg import String, Bool
from robotx_graey_2026.api.steady_node import SteadyNode
from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link, MODE_MANUAL
from .north_maneuver import NorthManeuver


class NorthManeuverNode(SteadyNode):
    def __init__(self):
        super().__init__('north_maneuver')
        self.declare_parameter('max_distance_m',5.)
        self.logic = NorthManeuver(time.monotonic(),self.get_parameter('max_distance_m').value)
        # Exclusive with waypoint_mission through GUI mission inventory and lease.
        self.link = Link('udpout:127.0.0.1:14553',198,self.get_logger(),system=255)
        self.bridge = self.control = {}
        self.bridge_at = self.control_at = -math.inf
        self.cancelled = self.active = False
        self.stopped_at = None
        self.pub = self.create_publisher(String,'/graey/navigation/north_maneuver',10)
        self.claim = self.create_publisher(Bool,'/graey/autonomy_active',10)
        self.create_subscription(String,'/graey/navigation/bridge_status',self.on_bridge,10)
        self.create_subscription(String,'/graey/control_watchdog/status',self.on_control,10)
        self.create_subscription(String,'/graey/navigation/north_abort',self.on_abort,10)
        self.create_timer(.05,self.tick)
        self.create_timer(1.,self.link.heartbeat)

    def on_bridge(self,msg):
        try:self.bridge=json.loads(msg.data);self.bridge_at=time.monotonic()
        except (ValueError,TypeError):pass

    def on_control(self,msg):
        try:self.control=json.loads(msg.data);self.control_at=time.monotonic()
        except (ValueError,TypeError):pass

    def on_abort(self,msg):
        self.cancelled = True

    def tick(self):
        now=time.monotonic();b,c=self.bridge,self.control
        def age(stamp):
            return now-stamp if isinstance(stamp,(int,float)) and math.isfinite(stamp) else math.inf
        fresh=(0<=now-self.bridge_at<.5 and 0<=now-b.get('published_monotonic',-math.inf)<.5
               and 0<=age(b.get('heading_vn',{}).get('received'))<.5
               and 0<=now-self.control_at<.7 and b.get('inputs',{}).get('vn') and not b.get('fault'))
        control=(c.get('vehicle_fresh') and c.get('armed') is True and c.get('mode')=='MANUAL'
                 and c.get('sa_active') is False and c.get('esc_commanded_on') is True and not c.get('inhibit'))
        if not self.active:
            # DDS discovery only; never send motion before fresh interlocks.
            if not fresh or not control:
                if now-self.logic.started<3:return
            self.active=True
            self.logic.started=self.logic.last=now
        v=b.get('heading_dvl',{}).get('velocity') or [0.,0.,0.]
        h=b.get('heading_vn') or {}
        gps=b.get('gps_admission') or {}
        was_terminal=self.logic.stage in self.logic.terminal
        x,r=self.logic.step(now,dict(control_ok=bool(control),abort=self.cancelled or (b.get('north_alignment') or {}).get('active') is False,fresh=bool(fresh),
            depth=b.get('depth_m'),yaw=h.get('raw_yaw',math.nan),rate=h.get('rate',math.nan),
            speed=math.sqrt(sum(a*a for a in v)),forward=v[0],sideways=v[1],
            dvl_valid=b.get('inputs',{}).get('dvl') and not b.get('dvl_continuity',{}).get('recovering'),
            gps_qualified=gps.get('qualified'),gps_speed=(b.get('gps_observation') or {}).get('speed_mps',math.inf),
            candidate_ready=(b.get('north_alignment') or {}).get('candidate_ready')))
        # Yield immediately on pilot mode change/kill/disarm; never fight takeover.
        if control and not was_terminal:
            self.link.send('manual_control_send',1,x,self.logic.lateral_command,500,r,0)
        terminal=self.logic.stage in self.logic.terminal
        self.claim.publish(Bool(data=not terminal))
        self.pub.publish(String(data=json.dumps(dict(self.logic.snapshot(),published_monotonic=now,
            observation=dict(bridge_age_s=age(self.bridge_at) if math.isfinite(age(self.bridge_at)) else None,
                control_age_s=age(self.control_at) if math.isfinite(age(self.control_at)) else None,
                depth_m=b.get('depth_m'),vn_fresh=b.get('inputs',{}).get('vn'),
                bridge_fault=b.get('fault'),fresh=bool(fresh),control_ok=bool(control))),allow_nan=False)))
        if terminal:
            self.stopped_at=now if self.stopped_at is None else self.stopped_at
            if now-self.stopped_at>2:raise SystemExit(0)


def main():
    run(NorthManeuverNode,ownership='north_maneuver')
