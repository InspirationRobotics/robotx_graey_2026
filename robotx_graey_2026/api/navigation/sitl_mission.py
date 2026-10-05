"""One-waypoint surfaced -> dive -> underwater transit -> surfaced SITL run.

Only for the local ArduSub SITL instance. Uses the repository's MAVLink Link,
live navigation supervisor status and one mission GPS target. The real
prequalification mission runner is intentionally not modified by this harness.
"""
import json
import math
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

from rclpy.node import Node
from std_msgs.msg import String

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import Link, MODE_GUIDED, MODE_MANUAL, mavutil


def offset_gps(lat, lon, north_m, east_m):
    return lat+north_m/111_320.0, lon+east_m/(111_320.0*math.cos(math.radians(lat)))


class SitlMission(Node):
    def __init__(self):
        super().__init__('sitl_mission')
        defaults = dict(mavlink='udpout:127.0.0.1:14559', home_lat=32.9240586,
                        home_lon=-117.0385389, waypoint_north_m=1.5,
                        waypoint_east_m=1.0, dive_depth_m=.5,
                        horizontal_tolerance_m=.25, depth_tolerance_m=.08,
                        timeout_s=90., result_file='/tmp/graey-sitl-result.json')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        cfg = {key: self.get_parameter(key).value for key in defaults}
        self.link = Link(cfg['mavlink'], 198, self.get_logger())
        self.home = (cfg['home_lat'], cfg['home_lon'])
        lat, lon = offset_gps(*self.home, cfg['waypoint_north_m'], cfg['waypoint_east_m'])
        self.target_gps = {'lat': lat, 'lon': lon,
                           'label': 'Configured SITL target; pool position is approximate'}
        self.target_n = cfg['waypoint_north_m']
        self.target_e = cfg['waypoint_east_m']
        self.depth = cfg['dive_depth_m']
        self.xy_tol = cfg['horizontal_tolerance_m']
        self.z_tol = cfg['depth_tolerance_m']
        self.timeout = cfg['timeout_s']
        self.result_file = cfg['result_file']
        self.run_id = str(uuid.uuid4())
        self.sequence = 0
        self.phase = 'SURFACE'
        self.stage = 'WAIT_SURFACE_GPS'
        self.stage_started = time.monotonic()
        self.status = {}
        self.last_status_summary = None
        self.local = None
        self.attitude = None
        self.gps_enabled = True
        self.gps_fix_type = 0
        self.gps_lat_lon = None
        self.gps_received = -math.inf
        self.gps_disable_readback = None
        self.gps_loss_verified = False
        self.gps_recovery_verified = False
        self.dive_reached = False
        self.waypoint_reached = False
        self.source_history = []
        self.armed = False
        self.mission_origin = None
        self.create_subscription(String, '/graey/navigation/status', self.on_status, 10)
        self.create_timer(.02, self.pump)
        self.create_timer(.1, self.tick)
        self.get_logger().info(f'SITL target GPS {lat:.7f},{lon:.7f}; local N/E {self.target_n:.2f}/{self.target_e:.2f} m')

    def enter(self, stage, phase=None):
        self.get_logger().info(f'SCENARIO {self.stage} -> {stage}')
        self.stage, self.stage_started = stage, time.monotonic()
        if phase:
            self.phase = phase

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
            source = self.status.get('confirmed_source')
            if source and (not self.source_history or source != self.source_history[-1]):
                self.source_history.append(source)
            summary = tuple(self.status.get(key) for key in
                ('state', 'reason', 'confirmed_source', 'navigation_ready',
                 'gps_qualified_input', 'configuration_verified'))
            if summary != self.last_status_summary:
                self.get_logger().info('SUPERVISOR ' + json.dumps(dict(zip(
                    ('state', 'reason', 'confirmed_source', 'navigation_ready',
                     'gps_qualified_input', 'configuration_verified'), summary))))
                self.last_status_summary = summary
        except (ValueError, TypeError):
            pass

    def publish_intent(self):
        self.sequence += 1
        # Publish at 10 Hz with monotonic sequence numbers, as the live mission does.
        self.intent_pub.publish(String(data=json.dumps(dict(run_id=self.run_id,
            phase=self.phase, sequence=self.sequence, target_gps=self.target_gps))))

    def pump(self):
        self.link.heartbeat()
        def consume(kind, msg):
            if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
                return
            if kind == 'LOCAL_POSITION_NED':
                self.local = (msg.x, msg.y, msg.z)
            elif kind == 'ATTITUDE':
                self.attitude = msg.yaw
            elif kind == 'GPS_RAW_INT':
                self.gps_fix_type = msg.fix_type
                self.gps_received = time.monotonic()
                if msg.fix_type >= 3:
                    self.gps_lat_lon = (msg.lat/1e7, msg.lon/1e7)
            elif kind == 'PARAM_VALUE' and msg.param_id.rstrip('\x00') == 'SIM_GPS_DISABLE':
                self.gps_disable_readback = int(round(msg.param_value))
            elif kind == 'HEARTBEAT':
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.link.drain(consume)

    def set_gps(self, enabled):
        if self.gps_enabled == enabled:
            return
        self.gps_enabled = enabled
        value = 0 if enabled else 1
        self.link.send('param_set_send', 1, 1, b'SIM_GPS_DISABLE', value,
                       mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self.link.send('param_request_read_send', 1, 1, b'SIM_GPS_DISABLE', -1)
        self.get_logger().info(f'SITL simulated GPS {"enabled" if enabled else "disabled"}')

    def gps_fix_is_stale_or_lost(self):
        lost = (self.gps_disable_readback == 1 and
                (self.gps_fix_type < 3 or time.monotonic()-self.gps_received > 1.0))
        if lost and not self.gps_loss_verified:
            self.gps_loss_verified = True
            self.get_logger().info(f'GPS outage verified: SIM_GPS_DISABLE={self.gps_disable_readback}, fix_type={self.gps_fix_type}')
        return lost

    def write_result(self, passed):
        result = dict(schema=1, passed=bool(passed), run_id=self.run_id,
            completed_utc=datetime.now(timezone.utc).isoformat(),
            starting_gps={'lat': self.home[0], 'lon': self.home[1]},
            waypoint_gps=self.target_gps, waypoint_local_ne_m=[self.target_n, self.target_e],
            requested_dive_depth_m=self.depth, dive_depth_reached=self.dive_reached,
            gps_loss_verified=self.gps_loss_verified,
            underwater_waypoint_reached=self.waypoint_reached,
            gps_recovery_verified=self.gps_recovery_verified,
            gps_fix_type=self.gps_fix_type, confirmed_source_history=self.source_history,
            final_local_ned=self.local, final_gps={'lat': self.gps_lat_lon[0], 'lon': self.gps_lat_lon[1]}
                if self.gps_lat_lon else None,
            final_supervisor_state=self.status.get('state'))
        path = Path(self.result_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
        self.get_logger().info(f'SCENARIO RESULT {json.dumps(result, allow_nan=False)}')

    def command_target(self, n, e, down):
        yaw = self.attitude if self.attitude is not None else 0.0
        self.link.goto_ned(n, e, down, yaw)

    def tick(self):
        if not hasattr(self, 'intent_pub'):
            self.intent_pub = self.create_publisher(String, '/graey/navigation/intent', 10)
        self.publish_intent()
        if self.stage in ('COMPLETE', 'FAULT'):
            return
        if time.monotonic()-self.stage_started > self.timeout:
            self.get_logger().error(f'SCENARIO timeout in {self.stage}; requesting surface and pilot mode')
            self.link.disarm(tries=2, gap=.1)
            self.link.set_mode(MODE_MANUAL)
            self.enter('FAULT')
            return
        status = self.status
        if self.stage == 'WAIT_SURFACE_GPS':
            if status.get('state') == 'Surface GPS' and status.get('navigation_ready'):
                self.enter('WAIT_DIVE_REFERENCE', 'DIVE')
        elif self.stage == 'WAIT_DIVE_REFERENCE':
            ref = status.get('reference')
            if (status.get('reference_current') and status.get('navigation_ready')
                    and ref and status.get('confirmed_source') == 'Underwater'):
                self.mission_origin = tuple(ref['cube_ned'])
                self.link.set_mode(MODE_GUIDED)
                self.link.arm()
                self.enter('DIVE_TO_0_5M')
        elif self.stage == 'DIVE_TO_0_5M' and self.local and self.mission_origin:
            n, e, d0 = self.mission_origin
            self.command_target(n, e, d0+self.depth)
            depth = status.get('depth_m')
            if isinstance(depth, (int, float)) and depth >= .4:
                self.set_gps(False)
            lost = self.gps_fix_is_stale_or_lost()
            if (isinstance(depth, (int, float)) and abs(depth-self.depth) <= self.z_tol
                    and lost):
                self.dive_reached = True
                self.get_logger().info(f'DIVE DEPTH REACHED: measured={depth:.3f} m target={self.depth:.3f} m; GPS loss confirmed')
                self.enter('UNDERWATER_WAYPOINT', 'UNDERWATER')
        elif self.stage == 'UNDERWATER_WAYPOINT' and self.local and self.mission_origin:
            n0, e0, d0 = self.mission_origin
            self.command_target(n0+self.target_n, e0+self.target_e, d0+self.depth)
            d = status.get('depth_m')
            err = math.hypot(self.local[0]-(n0+self.target_n), self.local[1]-(e0+self.target_e))
            if err <= self.xy_tol and isinstance(d, (int, float)) and abs(d-self.depth) <= self.z_tol:
                self.waypoint_reached = True
                self.get_logger().info(f'UNDERWATER WAYPOINT REACHED: horizontal_error={err:.3f} m depth={d:.3f} m GPS loss={self.gps_loss_verified}')
                self.set_gps(True)
                self.enter('SURFACING', 'SURFACE')
        elif self.stage == 'SURFACING' and self.local and self.mission_origin:
            n0, e0, d0 = self.mission_origin
            self.command_target(n0+self.target_n, e0+self.target_e, d0)
            if (status.get('state') == 'Surface GPS' and status.get('navigation_ready')
                    and self.gps_fix_type >= 3 and time.monotonic()-self.gps_received < 1.0):
                self.gps_recovery_verified = True
                self.link.disarm(tries=2, gap=.1)
                self.link.set_mode(MODE_MANUAL)
                self.enter('COMPLETE')
                self.write_result(self.dive_reached and self.gps_loss_verified and
                                  self.waypoint_reached and self.gps_recovery_verified)


def main():
    run(SitlMission)
