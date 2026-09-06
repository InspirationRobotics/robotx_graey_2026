#!/usr/bin/env python3
"""Graey kill, pilot controls, and selectable RF/tether watchdog.

RF:
    Pilot connection health comes from alternating CH12 and fresh RC data.

TETHER:
    Pilot connection health comes from interface-bound ICMP replies.
    CH12 loss does not trip the tether watchdog.

Both controllers remain accepted:
    This node does not filter QGC commands or disable the Cube's RC receiver.
    SB/SC handling remains available whenever the RF heartbeat is healthy.

RadioMaster:
    CH7  / SC: low MANUAL, high STABILIZE
    CH10 / SA: high KILL, low release
    CH11 / SB: low DISARM, high ARM
    CH12 / L01: alternating transmitter heartbeat

ESC relay:
    SERVO9 = 1900: kill
    SERVO9 = 1100: permit ESC power

Physical magnetic kill is independent and is not sensed by this node.

Startup and source changes inhibit ESC power:
    RF: restore heartbeat, SA KILL, SB DISARM, then release SA.
    TETHER: deliberate GUI recovery after health/disarm checks.

Tether recovery restores power permission, not arming.
"""

import ipaddress
import json
import math
import re
import shutil
import subprocess
import threading
import time
import uuid

from collections import OrderedDict

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import (
    Link,
    MODE_MANUAL,
    MODE_STABILIZE,
    mavutil,
)


RC_MESSAGE_STALE_S = 2.0
HEARTBEAT_TIMEOUT_S = 1.8
HEARTBEAT_STARTUP_GRACE_S = 3.0
VEHICLE_STALE_S = 3.0
DISARM_CONFIRM_S = 0.5

MODE_RESEND_S = 1.0
CMD_HOLD_S = 3.0

PWM_MIN = 800
PWM_MAX = 2200

AUTO_MODES = {'AUTO', 'GUIDED', 'CIRCLE', 'POSHOLD', 'SURFACE', 'RTL'}

REQUEST_TOPIC = '/graey/control_watchdog/request'
REPLY_TOPIC = '/graey/control_watchdog/reply'
STATUS_TOPIC = '/graey/control_watchdog/status'

RF_RESETTABLE = {'STARTUP', 'SOURCE_CHANGED', 'RF_LOST', 'FC_LOST'}
TETHER_RESETTABLE = {
    'STARTUP', 'SOURCE_CHANGED', 'TETHER_LOST', 'FC_LOST', 'SA_KILL'
}


def valid(pwm):
    return PWM_MIN <= pwm <= PWM_MAX


class TetherProbe:
    """Background ICMP probe; never blocks a ROS safety callback.

    A successful reply is timestamped at probe START, not completion.
    Therefore a delayed result cannot make an old probe look newer.

    The target must be another IPv4 host on the configured interface's subnet.
    ping -I binds the socket to that interface; no Wi-Fi fallback is used.
    """

    def __init__(self, address, interface, interval, timeout):
        self.address = str(ipaddress.IPv4Address(address))
        if not re.fullmatch(r'[A-Za-z0-9_.:-]{1,15}', interface):
            raise ValueError('Invalid tether interface name')

        if not math.isfinite(interval) or interval < 0.5:
            raise ValueError('tether_ping_interval_s must be at least 0.5')
        if not math.isfinite(timeout) or timeout < interval + 1.0:
            raise ValueError(
                'tether_timeout_s must exceed ping interval by at least 1 s'
            )

        self.interface = interface
        self.interval = interval
        self.timeout = timeout
        self.ping_program = shutil.which('ping')
        self.ip_program = shutil.which('ip')

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.last_success = None
        self.detail = 'No successful probe'
        self.thread = threading.Thread(
            target=self.worker, daemon=True
        )
        self.thread.start()

    def snapshot(self):
        with self.lock:
            stamp = self.last_success
            detail = self.detail

        age = None if stamp is None else time.monotonic() - stamp
        return {
            'alive': age is not None and age <= self.timeout,
            'age': age,
            'detail': detail,
        }

    def interface_matches(self):
        if not self.ip_program or not self.ping_program:
            raise RuntimeError('Jetson requires the ip and ping commands')

        result = subprocess.run(
            [
                self.ip_program, '-j', '-4', 'address',
                'show', 'dev', self.interface,
            ],
            capture_output=True,
            text=True,
            timeout=0.5,
            check=False,
        )
        if result.returncode != 0:
            raise RuntimeError('Tether interface is unavailable')

        target = ipaddress.IPv4Address(self.address)
        for device in json.loads(result.stdout):
            for item in device.get('addr_info', []):
                if item.get('family') != 'inet':
                    continue
                local = ipaddress.IPv4Interface(
                    f"{item['local']}/{item['prefixlen']}"
                )
                if target == local.ip:
                    raise RuntimeError('Tether target is the Jetson itself')
                if (
                    target in local.network
                    and target != local.network.network_address
                    and target != local.network.broadcast_address
                ):
                    return

        raise RuntimeError('Laptop IP is not on the tether interface subnet')

    def worker(self):
        while not self.stop_event.is_set():
            started = time.monotonic()
            try:
                self.interface_matches()
                result = subprocess.run(
                    [
                        self.ping_program, '-4', '-n',
                        '-I', self.interface,
                        '-c', '1', '-W', '1',
                        self.address,
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=1.3,
                    check=False,
                )
                success = result.returncode == 0
                detail = 'Reply received' if success else 'No ICMP reply'
            except (
                OSError, ValueError, KeyError, RuntimeError,
                subprocess.TimeoutExpired,
            ) as exc:
                success = False
                detail = str(exc)

            with self.lock:
                if success:
                    self.last_success = started
                self.detail = detail

            elapsed = time.monotonic() - started
            self.stop_event.wait(max(0.05, self.interval - elapsed))

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)


class KillSwitch(Node):
    def __init__(self):
        super().__init__('kill_switch')

        defaults = {
            'mavlink': 'udpout:127.0.0.1:14556',
            'kill_channel': 10,
            'arm_channel': 11,
            'heartbeat_channel': 12,
            'mode_channel': 7,
            'high': 1700,
            'relay_servo': 9,
            'relay_kill_pwm': 1900,
            'relay_run_pwm': 1100,
            'tether_ip': '192.168.2.1',
            'tether_interface': 'enP8p1s0',
            'tether_ping_interval_s': 1.0,
            'tether_timeout_s': 3.0,
        }
        # Configuration is accepted at startup, not silently changed at runtime.
        for name, value in defaults.items():
            self.declare_parameter(
                name, value,
                ParameterDescriptor(read_only=True),
            )

        def value(name):
            return self.get_parameter(name).value

        self.kill_ch = value('kill_channel')
        self.arm_ch = value('arm_channel')
        self.heartbeat_ch = value('heartbeat_channel')
        self.mode_ch = value('mode_channel')
        self.high = value('high')
        self.relay_servo = value('relay_servo')
        self.relay_kill_pwm = value('relay_kill_pwm')
        self.relay_run_pwm = value('relay_run_pwm')

        self.link = Link(value('mavlink'), 194, self.get_logger())
        self.started = self.now()

        self.source = 'RF'
        self.inhibit = 'STARTUP'
        self.session = uuid.uuid4().hex
        self.revision = 0
        self.reply_cache = OrderedDict()

        self.rc = {}
        self.rc_t = None
        self.armed = None
        self.mode = None
        self.vehicle_t = None
        self.disarmed_since = None

        self.heartbeat_state = None
        self.heartbeat_last_edge = None
        self.heartbeat_ready = False
        self.heartbeat_reported = None

        self.auto_until = 0.0
        self.sa_active = False
        self.esc_commanded_on = False
        self.armed_req = None
        self.mode_req = None
        self.cmd_until = 0.0
        self.mode_t = 0.0

        self.probe = TetherProbe(
            value('tether_ip'),
            value('tether_interface'),
            value('tether_ping_interval_s'),
            value('tether_timeout_s'),
        )

        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.reply_pub = self.create_publisher(String, REPLY_TOPIC, 10)

        self.create_subscription(
            String, REQUEST_TOPIC, self.on_request, 10
        )
        self.create_subscription(
            Bool, '/graey/autonomy_active', self.on_auto, 10
        )

        self.create_timer(1.0, self.link.heartbeat)
        self.create_timer(5.0, self.request_rc)
        self.create_timer(0.05, self.pump)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.2, self.publish_status)

        self.stop_outputs()
        self.request_rc()
        self.get_logger().warn(
            'Starting in RF with ESC power inhibited; '
            'restore RF and set SA KILL / SB DISARM, '
            'or select tether and recover through the GUI'
        )

    @staticmethod
    def now():
        # Watchdogs use monotonic time, not adjustable wall/ROS time.
        return time.monotonic()

    def rc_fresh(self):
        return (
            self.rc_t is not None
            and self.now() - self.rc_t <= RC_MESSAGE_STALE_S
        )

    def vehicle_fresh(self):
        return (
            self.vehicle_t is not None
            and self.now() - self.vehicle_t <= VEHICLE_STALE_S
        )

    def confirmed_disarmed(self):
        return (
            self.vehicle_fresh()
            and self.armed is False
            and self.disarmed_since is not None
            and self.now() - self.disarmed_since >= DISARM_CONFIRM_S
        )

    def rf_alive(self):
        return (
            self.rc_fresh()
            and self.heartbeat_ready
            and self.heartbeat_last_edge is not None
            and self.now() - self.heartbeat_last_edge <= HEARTBEAT_TIMEOUT_S
        )

    def selected_alive(self):
        if self.source == 'RF':
            return self.rf_alive()
        return self.probe.snapshot()['alive']

    def autonomous(self):
        # Existing mission nodes fly in GUIDED.
        # A Bool claim alone is also published by dry runs / WAIT_NAV,
        # so it cannot safely exempt manual driving from its watchdog.
        return self.vehicle_fresh() and self.mode in AUTO_MODES

    def mission_claimed(self):
        # Conservative interlock for GUI selection/recovery only.
        return self.now() < self.auto_until

    def on_auto(self, msg):
        self.auto_until = self.now() + 2.0 if msg.data else 0.0

    def request_rc(self):
        self.link.command(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
            100000,
        )

    def update_heartbeat(self, pwm):
        if not valid(pwm):
            return

        state = pwm > self.high
        if self.heartbeat_state is None:
            self.heartbeat_state = state
            return

        if state != self.heartbeat_state:
            self.heartbeat_state = state
            self.heartbeat_last_edge = self.now()
            self.heartbeat_ready = True

    def pump(self):
        def consume(kind, msg):
            if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
                return

            if kind == 'RC_CHANNELS':
                self.rc_t = self.now()
                for channel in (
                    self.kill_ch, self.arm_ch,
                    self.heartbeat_ch, self.mode_ch,
                ):
                    self.rc[channel] = getattr(
                        msg, f'chan{channel}_raw', 0
                    )
                self.update_heartbeat(self.rc.get(self.heartbeat_ch, 0))

            elif kind == 'HEARTBEAT':
                now = self.now()
                previously_fresh = self.vehicle_fresh()
                armed = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )

                if armed:
                    self.disarmed_since = None
                elif (
                    self.armed is not False
                    or not previously_fresh
                    or self.disarmed_since is None
                ):
                    self.disarmed_since = now

                self.armed = armed
                self.mode = self.link.mav.flightmode
                self.vehicle_t = now

        self.link.drain(consume)

    def cancel_arm_request(self):
        self.cmd_until = 0.0
        self.armed_req = None

    def disarm(self):
        self.link.command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0
        )

    def set_esc_power(self, enabled):
        self.esc_commanded_on = bool(enabled)
        self.link.command(
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            self.relay_servo,
            self.relay_run_pwm if enabled else self.relay_kill_pwm,
        )

    def stop_outputs(self, force_manual=False):
        self.cancel_arm_request()
        self.set_esc_power(False)
        self.disarm()

        if force_manual and self.now() - self.mode_t >= MODE_RESEND_S:
            self.mode_t = self.now()
            self.link.set_mode(MODE_MANUAL)

    def latch(self, reason):
        # Preserve the original fault until deliberately recovered.
        if self.inhibit is None:
            self.inhibit = reason
            self.revision += 1
            self.get_logger().error(
                f'{reason}: ESC power inhibited; disarming'
            )
        self.cancel_arm_request()

    def refresh_sa(self):
        pwm = self.rc.get(self.kill_ch, 0)

        # In tether mode, only a live RF heartbeat makes a NEW SA sample
        # trustworthy. A previously observed kill remains remembered.
        usable = (
            self.rc_fresh()
            and valid(pwm)
            and (self.source == 'RF' or self.rf_alive())
        )
        if not usable:
            return

        active = pwm > self.high
        if active != self.sa_active:
            self.sa_active = active
            self.revision += 1
            self.get_logger().warn(
                'SA KILL engaged' if active else 'SA KILL released'
            )
            self.cancel_arm_request()

    def rf_switches_safe(self):
        return (
            self.rf_alive()
            and valid(self.rc.get(self.kill_ch, 0))
            and valid(self.rc.get(self.arm_ch, 0))
            and self.rc[self.kill_ch] > self.high
            and self.rc[self.arm_ch] <= self.high
        )

    def reset_rf_if_safe(self):
        if (
            self.source == 'RF'
            and self.inhibit in RF_RESETTABLE
            and self.confirmed_disarmed()
            and self.rf_switches_safe()
        ):
            self.inhibit = None
            self.revision += 1
            self.cancel_arm_request()
            self.get_logger().warn(
                'RF inhibit cleared: heartbeat restored, '
                'SA KILL, SB DISARM; release SA before arming'
            )

    def select_reason(self, requested):
        if requested not in ('RF', 'TETHER'):
            return 'Invalid controller selection.'
        if not self.confirmed_disarmed():
            return 'Fresh Cube telemetry must confirm disarmed.'
        if self.mission_claimed() or self.autonomous():
            return 'Stop the mission and leave autonomous mode first.'
        if self.inhibit not in (None, 'STARTUP', 'SOURCE_CHANGED'):
            return 'Recover the existing fault before changing selection.'

        healthy = (
            self.rf_alive()
            if requested == 'RF'
            else self.probe.snapshot()['alive']
        )
        if not healthy:
            return 'Requested connection is not healthy.'
        return None

    def recovery_reason(self):
        if self.source != 'TETHER':
            return 'Tether is not selected.'
        if self.inhibit not in TETHER_RESETTABLE:
            return 'There is no recoverable tether inhibit.'
        if not self.confirmed_disarmed():
            return 'Fresh Cube telemetry must confirm disarmed.'
        if self.mission_claimed() or self.autonomous():
            return 'Stop the mission and leave autonomous mode first.'
        if not self.probe.snapshot()['alive']:
            return 'Tether ping is not healthy.'
        if self.sa_active:
            return 'Release the known SA kill with a live RF link first.'
        return None

    def on_request(self, msg):
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                return
            request_id = request.get('id')
            if not isinstance(request_id, str) or len(request_id) > 80:
                return
        except (TypeError, ValueError):
            return

        if request_id in self.reply_cache:
            self.reply_pub.publish(
                String(data=self.reply_cache[request_id])
            )
            return

        self.refresh_sa()
        ok = False
        message = 'Invalid request.'

        deadline = request.get('deadline')
        valid_deadline = (
            isinstance(deadline, (int, float))
            and math.isfinite(deadline)
            and self.now() <= deadline <= self.now() + 5.0
        )

        if not valid_deadline:
            message = 'Request expired.'
        elif request.get('session') != self.session:
            message = 'Safety node restarted. Refresh status and retry.'
        elif request.get('revision') != self.revision:
            message = 'Safety state changed. Refresh status and retry.'
        elif request.get('action') == 'select':
            requested = request.get('source')
            reason = self.select_reason(requested)
            if reason:
                message = reason
            elif requested == self.source:
                ok = True
                message = f'{self.source} is already selected.'
            else:
                self.source = requested
                self.inhibit = 'SOURCE_CHANGED'
                self.revision += 1
                self.mode_req = None
                self.stop_outputs()
                ok = True
                message = (
                    'TETHER selected. Click Recover tether control '
                    'to enable ESC power; arming is separate.'
                    if requested == 'TETHER'
                    else 'RF selected. Set SA KILL and SB DISARM '
                         'with RF healthy, then release SA.'
                )

        elif request.get('action') == 'recover':
            reason = self.recovery_reason()
            if reason:
                message = reason
            else:
                # This clears this node's retries, not commands still
                # being sent by QGC or another MAVLink client.
                self.cancel_arm_request()
                self.mode_req = None
                self.disarm()
                self.inhibit = None
                self.revision += 1
                self.set_esc_power(True)
                ok = True
                message = (
                    'Tether control recovered; ESC power permitted. '
                    'Vehicle remains disarmed. Use a new arm action.'
                )
                self.get_logger().warn(message)

        response = json.dumps({
            'id': request_id,
            'ok': ok,
            'msg': message,
            'session': self.session,
            'revision': self.revision,
        })
        self.reply_cache[request_id] = response
        while len(self.reply_cache) > 64:
            self.reply_cache.popitem(last=False)

        self.reply_pub.publish(String(data=response))
        self.publish_status()

    def publish_status(self):
        ping = self.probe.snapshot()
        self.status_pub.publish(String(data=json.dumps({
            'session': self.session,
            'revision': self.revision,
            'source': self.source,
            'inhibit': self.inhibit,
            'armed': self.armed,
            'mode': self.mode,
            'vehicle_fresh': self.vehicle_fresh(),
            'rf_alive': self.rf_alive(),
            'tether_alive': ping['alive'],
            'ping_age': ping['age'],
            'ping_detail': ping['detail'],
            'tether_ip': self.probe.address,
            'tether_interface': self.probe.interface,
            'sa_active': self.sa_active,
            'esc_commanded_on': self.esc_commanded_on,
            'autonomous': self.autonomous(),
            'mission_claimed': self.mission_claimed(),
            'can_select_rf': self.select_reason('RF') is None,
            'can_select_tether': self.select_reason('TETHER') is None,
            'can_recover_tether': self.recovery_reason() is None,
        })))

    def report_rf(self):
        alive = self.rf_alive()
        if (
            not self.heartbeat_ready
            and self.now() - self.started < HEARTBEAT_STARTUP_GRACE_S
        ):
            return

        if alive != self.heartbeat_reported:
            self.heartbeat_reported = alive
            if alive:
                self.get_logger().warn('RC heartbeat -> OK')
            else:
                self.get_logger().warn(
                    'RC heartbeat -> LOST'
                    + (' (tether watchdog selected)' if self.source == 'TETHER'
                       else '')
                )

    def tick(self):
        self.report_rf()
        self.refresh_sa()

        if not self.vehicle_fresh():
            if self.inhibit is None:
                self.latch('FC_LOST')
            self.stop_outputs()
            return

        # RF reset still requires SA killed and SB disarmed.
        self.reset_rf_if_safe()

        if self.sa_active:
            if self.source == 'TETHER':
                self.latch('SA_KILL')
            self.stop_outputs(force_manual=True)
            return

        if self.inhibit is not None:
            self.stop_outputs()
            return

        autonomous = self.autonomous()
        healthy = self.selected_alive()

        if not autonomous and not healthy:
            # Inhibit even if currently disarmed: restoring a connection
            # must not automatically restore already-permitted ESC power.
            self.latch(
                'RF_LOST' if self.source == 'RF' else 'TETHER_LOST'
            )
            self.stop_outputs()
            return

        self.set_esc_power(True)

        # Cached RC values must not manufacture SB/SC actions.
        # This is freshness checking, not exclusive-controller filtering.
        if not self.rf_alive():
            self.cancel_arm_request()
            self.mode_req = None
            return

        arm_pwm = self.rc.get(self.arm_ch, 0)
        if not valid(arm_pwm):
            self.cancel_arm_request()
            return

        self.do_mode(self.rc.get(self.mode_ch, 0))
        self.do_arm(arm_pwm, healthy)

    def do_mode(self, pwm):
        if not valid(pwm):
            return

        want = MODE_STABILIZE if pwm > self.high else MODE_MANUAL

        if self.mode_req is None:
            self.mode_req = want
            return
        if want == self.mode_req:
            return

        self.mode_req = want
        if self.autonomous() or self.mission_claimed():
            self.get_logger().warn(
                'Mode switch ignored while mission is active'
            )
            return

        self.get_logger().warn(
            'Mode -> '
            + ('STABILIZE' if want == MODE_STABILIZE else 'MANUAL')
        )
        self.link.set_mode(want)

    def do_arm(self, pwm, selected_healthy):
        want = pwm > self.high

        # Adopt the current switch after kill/recovery/source change.
        # A later low-to-high transition is needed for a new ARM request.
        if self.armed_req is None:
            self.armed_req = want
            return

        if want != self.armed_req:
            self.armed_req = want

            if want and not selected_healthy:
                self.cmd_until = 0.0
                self.get_logger().error(
                    'ARM blocked: selected connection is unhealthy'
                )
                return

            self.cmd_until = self.now() + CMD_HOLD_S
            self.get_logger().warn(
                'ARM requested' if want else 'DISARM requested'
            )

        if want and not selected_healthy:
            self.cmd_until = 0.0
            return

        if self.now() < self.cmd_until and self.armed != want:
            if want:
                self.link.set_mode(
                    MODE_MANUAL
                    if self.mode_req is None
                    else self.mode_req
                )

            self.link.command(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                1 if want else 0,
            )

    def destroy_node(self):
        self.stop_outputs()
        self.probe.close()
        return super().destroy_node()


def main():
    run(KillSwitch)
