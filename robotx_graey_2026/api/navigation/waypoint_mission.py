"""Operator-started waypoint mission; the same controller runs in isolated SITL."""
import json
import math
import os
import time
from pathlib import Path

from robotx_graey_2026.api.steady_node import SteadyNode as Node
from std_msgs.msg import Bool, String
from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.gui.navigation_view import NavigationView
from robotx_graey_2026.api.navigation.waypoint_logic import WaypointRun
from robotx_graey_2026.api.navigation.guided_surface_mission import GuidedSurfaceRun
from robotx_graey_2026.api.navigation.geographic_frame import GeographicFrame
from robotx_graey_2026.api.navigation.surface_measurement import SurfaceMeasurement
from robotx_graey_2026.api.navigation.surface_control import surface_velocity,SurfaceLeg,gentle_horizontal,StabilizeSurfaceControl
from robotx_graey_2026.api.pixhawk.mavlink import Link, MODE_GUIDED, MODE_MANUAL, MODE_STABILIZE


class WaypointMission(Node):
    # cm/s and cm/s² on the verified ArduSub 4.5 parameter interface.
    SHALLOW_LIMITS = dict(WPNAV_SPEED_DN=10., WPNAV_SPEED_UP=15., WPNAV_ACCEL_Z=50.)

    def __init__(self):
        super().__init__('waypoint_mission')
        from .odometry_profile import SHALLOW_LIMITS
        self.SHALLOW_LIMITS=SHALLOW_LIMITS.copy()
        for key, value in dict(plan_json='', run_id='', commissioned=False, simulation=False,
                              mavlink='udpout:127.0.0.1:14553', result_file='').items():
            self.declare_parameter(key, value)
        cfg = {key: self.get_parameter(key).value for key in
               ('plan_json', 'run_id', 'commissioned', 'simulation', 'mavlink', 'result_file')}
        self.sim = cfg['simulation']
        if self.sim and not (os.environ.get('GRAEY_RUNTIME_SCOPE') == 'sitl'
                            and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
                            and os.environ.get('ROS_DOMAIN_ID') == '73'
                            and cfg['mavlink'] == 'udpout:127.0.0.1:14553'):
            raise ValueError('Simulation requires isolated localhost SITL domain and endpoint')
        if not cfg['run_id'] or (not self.sim and not cfg['commissioned']):
            raise ValueError('Mission run ID and commissioned live configuration required')
        plan=json.loads(cfg['plan_json'])
        self.logic = (GuidedSurfaceRun if plan.get('mode','surface')=='surface' else WaypointRun)(plan,cfg['run_id'],time.monotonic())
        self.result_file = cfg['result_file']
        self.link = Link(cfg['mavlink'], 198, self.get_logger(),system=255 if isinstance(self.logic,GuidedSurfaceRun) else 1)
        self.view = NavigationView()
        self.supervisor, self.safety = {}, {}
        self.bridge = {}
        self.bridge_time = -math.inf
        self.supervisor_time = self.safety_time = -math.inf
        self.aborted, self.written = False, False
        self.sequence, self.stamps = 0, {}
        self.nav_limits = {}
        self.gps_enabled = True
        self.last_stage = None
        self.last_pause = False
        self.last_trace = -math.inf
        self.sent_target = None
        self.surface_leg=SurfaceLeg()
        self.stabilize_control=StabilizeSurfaceControl()
        self.pilot_override_active=False
        self.surface_measurement=(SurfaceMeasurement(duration=5.,tolerated_gps_gap=4.,tolerated_sensor_gap=1.5)
                                  if self.logic.plan['mode']=='surface' else SurfaceMeasurement())
        self.surface_measurement_status={}
        self.fc_status_text = []
        self.pub = self.create_publisher(String, '/graey/waypoint/status', 10)
        self.intent_pub = self.create_publisher(String, '/graey/navigation/intent', 10)
        self.auto_pub = self.create_publisher(Bool, '/graey/autonomy_active', 10)
        self.create_subscription(String, '/graey/navigation/status', self.on_status, 10)
        self.create_subscription(String, '/graey/navigation/bridge_status', self.on_bridge, 10)
        self.create_subscription(String, '/graey/control_watchdog/status', self.on_safety, 10)
        self.create_subscription(String, '/graey/waypoint/abort', self.on_abort, 10)
        self.create_timer(.02, self.pump)
        self.create_timer(.1, self.tick)
        self.create_timer(1., self.link.heartbeat)
        self.create_timer(5., self.request)
        self.request()

    def request(self):
        for mid in (24, 30, 32, 33, 49, 193):
            self.link.command(511, mid, 100000)
        for name in self.SHALLOW_LIMITS:
            self.link.send('param_request_read_send',1,1,name.encode(),-1)

    def limits_ready(self, now):
        for name,maximum in self.SHALLOW_LIMITS.items():
            stamp,value=getattr(self,'nav_limits',{}).get(name,(-math.inf,None))
            if not (0<=now-stamp<12 and isinstance(value,(int,float))
                    and math.isfinite(value) and 0<value<=maximum+max(1e-7,abs(maximum)*1e-6)):
                return False
        return True

    def destroy_node(self):
        self.release_surface_controls()
        return super().destroy_node()

    def release_surface_controls(self):
        if self.pilot_override_active:
            # Release only the first eight pilot channels, not RF kill/arm inputs.
            self.link.send('rc_channels_override_send',1,1,*([0]*8))
            self.pilot_override_active=False

    def send_surface(self, now, velocity, heading, o, stopping=False):
        if not isinstance(self.logic,GuidedSurfaceRun):
            self.link.surface_velocity(*velocity,heading);return
        if o.get('mode')!=MODE_STABILIZE or not o.get('armed') or o.get('failure'):
            self.release_surface_controls();return
        if not o.get('fresh') and not o.get('navigation_hold_safe'):
            self.release_surface_controls();return
        command=self.stabilize_control.step(now,velocity,o['velocity_ned'],o['yaw'],heading,stopping)
        self.link.send('manual_control_send',1,*command,0)
        self.pilot_override_active=True
        self.last_manual_control=command

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.supervisor, self.supervisor_time = data, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_safety(self, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.safety, self.safety_time = data, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_bridge(self, msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                self.bridge,self.bridge_time=data,time.monotonic()
        except (ValueError,TypeError):
            pass

    def on_abort(self, msg):
        if msg.data == self.logic.run_id:
            self.aborted = True

    def pump(self):
        def receive(kind, m):
            if m.get_srcSystem() != 1 or m.get_srcComponent() != 1:
                return
            if kind=='PARAM_VALUE' and m.param_id.rstrip('\x00') in self.SHALLOW_LIMITS:
                self.nav_limits[m.param_id.rstrip('\x00')]=(time.monotonic(),m.param_value)
            if kind=='STATUSTEXT':
                item=dict(at=time.monotonic(),severity=int(m.severity),text=str(m.text))
                self.fc_status_text=(self.fc_status_text+[item])[-10:]
                self.get_logger().warning('FC_STATUS_TEXT '+json.dumps(item))
            if kind in ('LOCAL_POSITION_NED', 'ATTITUDE', 'GLOBAL_POSITION_INT'):
                previous = self.stamps.get(kind, -1)
                if m.time_boot_ms+1000 < previous:
                    self.aborted = True
                if m.time_boot_ms <= previous:
                    return
                self.stamps[kind] = m.time_boot_ms
            self.view.consume(kind, m)
        self.link.drain(receive)

    def observation(self, now):
        streams = self.view.snapshot()['streams']
        def sample(name, limit):
            s = streams.get(name, {})
            return s.get('data', {}) if s.get('age', math.inf) < limit else {}
        hb, local, att = sample('heartbeat', 2), sample('local', .5), sample('attitude', .5)
        glob, origin = sample('global', .5), sample('cube_origin', 10)
        s = self.supervisor
        fresh = bool(hb and local and att and origin and self.view.position_healthy()
                     and 0 <= now-self.supervisor_time < .7
                     and 0 <= now-s.get('published_monotonic', -math.inf) < .7)
        ned, velocity = local.get('ned'), local.get('velocity')
        depth = s.get('depth_m')
        fresh = fresh and all(isinstance(v, (int, float)) and math.isfinite(v)
                             for v in (*(ned or [None]), *(velocity or [None]),
                                       *att.get('rpy', [None]), depth))
        failure = ''
        limits_ready=self.limits_ready(now)
        if not limits_ready:
            fresh=False
            if self.logic.stage!='WAIT_SURFACE':
                failure='Shallow waypoint vertical speed/acceleration limits unavailable or excessive'
        if not self.sim:
            c = self.safety
            if not (0 <= now-self.safety_time < 2 and c.get('vehicle_fresh')
                    and not c.get('inhibit') and not c.get('sa_active') and c.get('esc_commanded_on')):
                fresh = False
                # DDS subscriptions can take a moment to match at startup.
                # Wait disarmed for the first status, without granting readiness.
                if not (self.safety_time == -math.inf and self.logic.stage == 'WAIT_SURFACE'
                        and now-self.logic.since < 5):
                    failure = 'Control watchdog or ESC permission unavailable'
        b=getattr(self,'bridge',{})
        cube_fresh=bool(hb and local and att and origin and self.view.position_healthy()
            and ned and velocity and depth is not None
            and all(isinstance(v,(int,float)) and math.isfinite(v)
                    for v in (*ned,*velocity,*att.get('rpy',[None]),depth)))
        continuity=b.get('dvl_continuity') or {}
        # Brief DVL loss permits bounded hold, never continued travel.
        dvl_hold=bool(continuity.get('recovering') is True
            and continuity.get('state')=='COASTING'
            and 0<=continuity.get('gap_age_s',math.inf)<5.0
            and 0<=continuity.get('position_sigma_m',math.inf)<=5.
            and not continuity.get('requires_position_revalidation',True)
            and b.get('initialized') is True and b.get('enabled') is True
            and not b.get('heading_continuity_lost',True)
            and (b.get('inputs') or {}).get('vn') is True
            and 0<=now-(b.get('pressure_sample') or {}).get('received',-math.inf)<.5)
        hold_safe=bool(cube_fresh and not failure
            and 0<=now-getattr(self,'bridge_time',-math.inf)<.3
            and 0<=now-b.get('published_monotonic',-math.inf)<.3
            and ((b.get('healthy') is True and b.get('navigation_valid') is True
                  and b.get('transmitting') is True) or dvl_hold)
            and b.get('configuration_verified') is True
            and not b.get('fault'))
        gps=b.get('gps_admission') or {}
        fix=b.get('gps_observation') or {}
        gps_current=bool(0<=now-getattr(self,'bridge_time',-math.inf)<.3
            and b.get('gps_eligible') is True and gps.get('qualified') is True
            and (gps.get('applied_recently') is True or gps.get('stationary_validated_recently') is True))
        gps_at_target=False
        if gps_current:
            try:
                from .waypoint_logic import geo_to_ne
                gps_at_target=(0<fix['accuracy']<=2. and math.dist(geo_to_ne(fix['lat'],fix['lon'],
                    (self.logic.plan['lat'],self.logic.plan['lon'])),(0.,0.))<=1.5)
            except (KeyError,ValueError,TypeError):
                pass
        geographic=(glob['lat'],glob['lon']) if glob.get('valid') else None
        if b.get('coordinate_frame')=='fixed_local':
            frame=GeographicFrame.restore(b.get('map_transform'))
            geographic=frame.to_geo(ned) if frame.angle is not None and ned else ((fix['lat'],fix['lon']) if gps_current else None)
        return dict(fresh=fresh, failure=failure, abort=self.aborted,bridge=b,
            surface_gps_current=gps_current,surface_gps_at_target=gps_at_target,
            dvl_continuity=b.get('dvl_continuity') if 0<=now-getattr(self,'bridge_time',-math.inf)<.3 else {},
            vertical_limits_verified=limits_ready,
            navigation_hold_safe=hold_safe,
            velocity_ned=velocity,
            supervisor=s, armed=hb.get('armed'), mode=hb.get('mode_id'),
            local=ned, speed=math.hypot(*velocity[:2]) if velocity and all(v is not None for v in velocity) else math.inf,
            yaw=math.radians(att['rpy'][2]) if att and att['rpy'][2] is not None else None,
            depth=depth, origin=(origin['lat'], origin['lon'], origin['alt']) if origin else None,
            global_healthy=glob.get('valid'),
            **{'global': geographic})

    def tick(self):
        now = time.monotonic()
        self.sequence += 1
        # Surface missions keep SURFACE intent throughout; no GPS source switch.
        if self.logic.stage not in self.logic.terminal:
            self.intent_pub.publish(String(data=json.dumps(dict(run_id=self.logic.run_id,
                phase=self.logic.phase, sequence=self.sequence, target_gps=self.logic.plan))))
        o = self.observation(now)
        if not isinstance(self.logic,GuidedSurfaceRun) and self.logic.stage=='CHECK_SURFACE' and getattr(self,'measurement_stage',None)!='CHECK_SURFACE':
            self.surface_measurement=SurfaceMeasurement(duration=5.,tolerated_gps_gap=4.,tolerated_sensor_gap=1.5)
        self.measurement_stage=self.logic.stage
        if self.logic.stage in ('WAIT_SURFACE','CHECK_SURFACE') or (isinstance(self.logic,GuidedSurfaceRun) and self.logic.stage in ('GUIDED','ARM')):
            b=self.bridge if 0<=now-self.bridge_time<.3 else {}
            self.surface_measurement_status=self.surface_measurement.update(now,b)
        o['surface_measurement']=self.surface_measurement_status
        if self.logic.stage=='WAIT_SURFACE' and not o['vertical_limits_verified']:
            self.logic.reason='Waiting for shallow waypoint vertical speed/acceleration limit readback'
        for action, payload in self.logic.step(now, o):
            if action in ('align_velocity','stop'):
                # Alignment and braking need the same surface protection as transit.
                # An abort with invalid depth must still be able to send a stop.
                depth=o.get('depth')
                down=0. if isinstance(self.logic,GuidedSurfaceRun) else surface_velocity((0.,0.),(0.,0.),(0.,0.),depth)[2] if o.get('fresh') and isinstance(depth,(int,float)) and math.isfinite(depth) else 0.
                v=(payload[0],payload[1],down) if action=='align_velocity' else (0.,0.,down)
                heading=payload[2] if action=='align_velocity' else payload[0]
                if action!='stop':
                    if self.logic.navigation_wait_since is None:
                        v=gentle_horizontal(v,getattr(self,'sent_surface_velocity',(0.,0.,0.)),now-getattr(self,'surface_command_at',now-.1))
                self.surface_command_at=now
                self.send_surface(now,v,heading,o,stopping=action=='stop' or self.logic.navigation_wait_since is not None)
                self.sent_surface_velocity=v;self.sent_surface_heading=heading
            elif action == 'target':
                self.sent_target=payload
                if self.logic.plan['mode']=='surface':
                    if self.logic.stage=='TRANSIT' and self.logic.target is not None and self.logic.navigation_wait_since is None:
                        v,heading=self.surface_leg.step(now,self.logic.start,self.logic.target,
                            o['local'],o['velocity_ned'],o['depth'],o['yaw'])
                    else:
                        v=surface_velocity(payload,o['local'],o['velocity_ned'],o['depth'])
                        heading=payload[3]
                    if self.logic.navigation_wait_since is None:
                        v=gentle_horizontal(v,getattr(self,'sent_surface_velocity',(0.,0.,0.)),now-getattr(self,'surface_command_at',now-.1))
                    if isinstance(self.logic,GuidedSurfaceRun):v=(*v[:2],0.)
                    self.surface_command_at=now
                    self.send_surface(now,v,heading,o,stopping=action=='stop' or self.logic.navigation_wait_since is not None)
                    self.sent_surface_velocity=v
                    self.sent_surface_heading=heading
                else:
                    self.link.goto_ned(*payload)
            elif action in ('guided', 'manual'):
                self.get_logger().warning('MISSION_MODE_REQUEST '+json.dumps(dict(
                    run_id=self.logic.run_id,action=action,stage=self.logic.stage,
                    reason=self.logic.reason,published_monotonic=now)))
                if action=='guided' and isinstance(self.logic,GuidedSurfaceRun):
                    self.link.send('manual_control_send',1,0,0,500,0,0)
                    self.pilot_override_active=True
                if action=='manual':self.release_surface_controls()
                self.link.set_mode((MODE_STABILIZE if isinstance(self.logic,GuidedSurfaceRun) else MODE_GUIDED) if action=='guided' else MODE_MANUAL)
            elif action in ('arm', 'disarm'):
                self.link.command(400, 1 if action == 'arm' else 0)
        if self.logic.stage in self.logic.terminal:self.release_surface_controls()
        if self.logic.stage=='ABORTED' and self.logic.abort_context is None:
            self.logic.abort_context=dict(telemetry_fresh=bool(o['fresh']),
                navigation_hold_safe=o.get('navigation_hold_safe'),
                supervisor_age_s=now-self.supervisor_time if math.isfinite(self.supervisor_time) else None,
                supervisor_reason=self.supervisor.get('reason'),supervisor_state=self.supervisor.get('state'),
                navigation_ready=self.supervisor.get('navigation_ready'),reference_current=self.supervisor.get('reference_current'),
                bridge_fault=self.bridge.get('fault'),dvl_continuity=self.bridge.get('dvl_continuity'),
                pressure_depth_m=o.get('depth'),cube_local_ned=o.get('local'),
                vertical_offset_error_m=self.logic.vertical_offset_error,
                fc_status_text=self.fc_status_text,
                watchdog_failure=o.get('failure'),cube_mode=o.get('mode'),cube_armed=o.get('armed'))
        if self.sim:
            enabled = not (self.logic.stage in ('DIVE', 'TRANSIT', 'SURFACE')
                           and isinstance(o['depth'], (int, float)) and o['depth'] > .2)
            if enabled != self.gps_enabled:
                self.gps_enabled = enabled
                self.link.send('param_set_send', 1, 1, b'SIM_GPS_DISABLE', float(not enabled), 9)
        status = self.logic.snapshot()
        paused=status['navigation_pause']['active']
        if paused!=self.last_pause:
            self.last_pause=paused
            self.get_logger().warning('MISSION_NAVIGATION_PAUSE '+json.dumps(dict(
                run_id=self.logic.run_id,published_monotonic=now,**status['navigation_pause'])))
        status.update(control_mode='STABILIZE' if isinstance(self.logic,GuidedSurfaceRun) else 'GUIDED', vertical_control='neutral pilot input' if isinstance(self.logic,GuidedSurfaceRun) else 'depth control',last_manual_control=getattr(self,'last_manual_control',None),simulation=self.sim, published_monotonic=now,
                      surface_leg=self.surface_leg.state,
                      last_sent_surface_heading_rad=getattr(self,'sent_surface_heading',None),
                      bridge_course_heading=self.bridge.get('course_heading'),
                      surface_measurement=self.surface_measurement_status,
                      vertical_limits_verified=o['vertical_limits_verified'],
                      telemetry_fresh=bool(o['fresh']), confirmed_source=self.supervisor.get('confirmed_source'))
        status['control_evidence']={k:self.safety.get(k) for k in (
            'source','inhibit','armed','mode','sa_active','esc_commanded_on',
            'mission_claimed','pilot_arm_switch_pwm','pilot_mode_switch_pwm','pilot_retry_remaining_s')}
        status['cube_mode_id']=o.get('mode')
        status['cube_armed']=o.get('armed')
        if self.logic.stage in self.logic.moving and now-self.last_trace>=.5:
            self.last_trace=now
            # Log independent inputs and sent frame references together. Stage
            # summaries alone cannot diagnose wrong-axis motion after the run.
            self.get_logger().info('WAYPOINT_TRACE '+json.dumps(dict(
                run_id=self.logic.run_id,stage=self.logic.stage,monotonic=now,
                plan=self.logic.plan,target_ned=self.logic.target,
                last_sent_target_ned_yaw=self.sent_target,
                last_manual_control=getattr(self,'last_manual_control',None),
                last_sent_surface_velocity_ned=getattr(self,'sent_surface_velocity',None),
                surface_leg=self.surface_leg.state,
                last_sent_surface_heading_rad=getattr(self,'sent_surface_heading',None),
                bridge_course_heading=self.bridge.get('course_heading'),
                start_ned=self.logic.start,cube_ned=o.get('local'),
                cube_yaw_rad=o.get('yaw'),pressure_depth_m=o.get('depth'),
                bridge_ned=self.bridge.get('position_ned'),
                bridge_velocity=self.bridge.get('heading_dvl'),
                bridge_heading=self.bridge.get('heading_vn'),
                dvl_continuity=self.bridge.get('dvl_continuity'),
                gps_admission=self.bridge.get('gps_admission'),
                gps_observation=self.bridge.get('gps_observation')),
                allow_nan=False))
        if self.logic.stage != self.last_stage:
            self.last_stage = self.logic.stage
            self.get_logger().info('WAYPOINT_STAGE '+json.dumps(status, allow_nan=False))
        self.pub.publish(String(data=json.dumps(status, allow_nan=False)))
        self.auto_pub.publish(Bool(data=self.logic.stage not in self.logic.terminal))
        if self.logic.stage in self.logic.terminal:
            if self.result_file and not self.written:
                path = Path(self.result_file)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(status, indent=2), encoding='utf-8')
                self.written = True
            # Publish the terminal state long enough for the GUI. On a nav abort
            # continue requesting takeover until the Cube acknowledges leaving GUIDED.
            if now-self.logic.since > 3 and (self.logic.stage == 'COMPLETE'
                    or o.get('mode') not in (None, MODE_STABILIZE if isinstance(self.logic,GuidedSurfaceRun) else MODE_GUIDED) or self.logic.pilot_takeover
                    or not self.logic.takeover_requested):
                raise SystemExit(0)


def main():
    run(WaypointMission, ownership='waypoint_mission')
