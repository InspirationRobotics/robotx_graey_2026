"""Read-only navigation status projection. No MAVLink or mission authority."""
import json
import time
from std_msgs.msg import String
from robotx_graey_2026.api.steady_node import SteadyNode as Node
from robotx_graey_2026.api.node_util import run


class NavigationSupervisor(Node):
    def __init__(self):
        super().__init__('navigation_supervisor')
        self.bridge={};self.received=None;self.mission={}
        self.pub=self.create_publisher(String,'/graey/navigation/status',10)
        self.create_subscription(String,'/graey/navigation/bridge_status',self.observe,10)
        self.create_subscription(String,'/graey/waypoint/status',self.observe_mission,10)
        self.create_timer(.1,self.tick)

    def observe(self,msg):
        try:
            value=json.loads(msg.data)
            if not isinstance(value,dict):return
            self.bridge=value;self.received=time.monotonic()
        except (ValueError,TypeError):pass

    def observe_mission(self,msg):
        try:
            value=json.loads(msg.data)
            if isinstance(value,dict):self.mission=value
        except (ValueError,TypeError):pass

    def tick(self):
        now=time.monotonic()
        b=self.bridge if self.received is not None and now-self.received<1. else {}
        h=b.get('estimator_health') or {}
        gps=b.get('gps_admission') or {}
        state=h.get('state','INITIALIZING')
        status=dict(state=state,reason=b.get('fault') or ('Sensor fusion '+state.lower()),
            mode='Observation',read_only=True,navigation_ready=h.get('usable',False),
            estimator_health=h,configuration_verified=b.get('configuration_verified',False),
            depth_m=b.get('depth_m'),surfaced=b.get('surfaced'),gps_qualified_input=gps.get('qualified',False),
            gps_quality_reason=gps.get('reason'),gps_admission=gps,north_alignment=b.get('north_alignment'),
            coordinate_frame=b.get('coordinate_frame'),local_navigation_ready=b.get('local_navigation_ready',False),
            map_transform=b.get('map_transform'),dvl_continuity=b.get('dvl_continuity'),
            run_id=self.mission.get('run_id'),reference=None,reference_current=False,
            published_monotonic=now)
        self.pub.publish(String(data=json.dumps(status,allow_nan=False)))


def main():run(NavigationSupervisor,ownership='navigation_supervisor')
