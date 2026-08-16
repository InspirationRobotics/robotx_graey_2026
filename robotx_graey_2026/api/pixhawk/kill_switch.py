#!/usr/bin/env python3
"""Graey pilot controls, emergency kill, and transmitter heartbeat failsafe.

RadioMaster channels:
  CH7  / SC: Flight mode. Low is MANUAL; high is STABILIZE.
  CH10 / SA: ESC relay permission. High kills; low enables power.
  CH11 / SB: Arming request. Low disarms; high arms in the CH7 mode.
  CH12 / L01: EdgeTX heartbeat. It continuously alternates low/high.

Why MANUAL is the default:
  STABILIZE holds roll and pitch level, and on a vectored 6DOF frame it does that
  with the VERTICAL thrusters. They come alive the instant you arm. Stabilisation
  is therefore opted into deliberately rather than being where arming lands you.

Why SA reads inverted, and why both PWM values look backwards:
  ESC power runs through a normally-OPEN solid-state relay, driven by a Pololu RC
  switch that activates on LOW pulse widths. It replaced a mechanical relay wired
  normally-CLOSED, so the sense of both the CH10 input and the SERVO9 output is
  the opposite of what it was. CH10 is reversed in the transmitter rather than the
  autopilot because SERVOn_REVERSED has no effect on an rc-passthrough output
  (verified on the bench, 2026-08-08).

  This node's set_esc_power() runs every tick, so DO_SET_SERVO overrides the
  passthrough whenever the node is alive. Getting either polarity wrong here means
  the node silently re-enables power against the switch - the reed switch is the
  only kill that stays honest.

Why CH12 is required:
  ArduPilot continues publishing cached RC_CHANNELS values after the RP3 stops
  receiving. MAVLink RSSI is unavailable, and SYS_STATUS does not expose valid
  receiver-health information on this configuration. The alternating heartbeat
  freezes when the transmitter link disappears, making link loss observable.

Failsafe behavior:
  - SA high kills and disarms in every mode.
  - Lost heartbeat while armed in a pilot-controlled mode kills and disarms.
  - Lost heartbeat in an autonomous mode does not interrupt the mission.
  - A pilot failsafe stays latched until the link returns and both SA and SB
    are explicitly placed in their safe/down positions.
"""
from rclpy.node import Node
from std_msgs.msg import Bool

from robotx_graey_2026.api.node_util import run
from robotx_graey_2026.api.pixhawk.mavlink import (
    Link, MODE_MANUAL, MODE_STABILIZE, mavutil)

RC_MESSAGE_STALE_S = 2.0

# EdgeTX L01 changes CH12 every 0.5 s. This allows more than three expected
# edges to be missed before declaring the transmitter lost.
HEARTBEAT_TIMEOUT_S = 1.8
HEARTBEAT_STARTUP_GRACE_S = 3.0

MODE_RESEND_S = 1.0
CMD_HOLD_S = 3.0
ARM_WARN_S = 10.0

PWM_MIN = 800
PWM_MAX = 2200

# Receiver loss is deliberately ignored in these modes.
AUTO_MODES = {'AUTO', 'GUIDED', 'CIRCLE', 'POSHOLD', 'SURFACE', 'RTL'}


def valid(pwm):
    return PWM_MIN <= pwm <= PWM_MAX        # 0 means the channel is not being sent


class KillSwitch(Node):
    def __init__(self):
        super().__init__('kill_switch')

        p = self.declare_parameter
        p('mavlink', 'udpout:127.0.0.1:14556')
        p('kill_channel', 10)
        p('arm_channel', 11)
        p('heartbeat_channel', 12)
        # SC. Below CH8, so a QGC joystick's MANUAL_CONTROL would pin it - harmless
        # for a mode switch, but move it above CH8 if a joystick ever comes back.
        p('mode_channel', 7)
        p('high', 1700)
        p('relay_servo', 9)                 # AUX OUT 1 is SERVO9
        p('relay_kill_pwm', 1900)           # high leaves the Pololu off, so the SSR opens
        p('relay_run_pwm', 1100)            # low switches the Pololu on, closing the SSR

        g = self.get_parameter
        self.kill_ch = g('kill_channel').value
        self.arm_ch = g('arm_channel').value
        self.heartbeat_ch = g('heartbeat_channel').value
        self.mode_ch = g('mode_channel').value
        self.high = g('high').value
        self.relay_servo = g('relay_servo').value
        self.relay_kill_pwm = g('relay_kill_pwm').value
        self.relay_run_pwm = g('relay_run_pwm').value

        self.link = Link(g('mavlink').value, 194, self.get_logger())
        self.started = self.now()

        self.rc = {}                        # channel -> last raw PWM
        self.rc_t = 0.0

        self.killed = None
        self.armed_req = None
        self.mode_req = None                # None until the switch is first read
        self.armed = None
        self.mode = None

        self.auto_until = 0.0
        self.tripped = False
        self.cmd_until = 0.0
        self.mode_t = 0.0
        self.arm_warn_t = 0.0

        self.heartbeat_state = None
        self.heartbeat_last_edge = 0.0
        self.heartbeat_ready = False
        self.heartbeat_reported = None

        self.fc_heartbeat_timeout_s = 2.0
        self.fc_heartbeat_t = 0.0
        self.fc_heartbeat_lost = False
        
        self.create_subscription(Bool, '/graey/autonomy_active', self.on_auto, 10)
        self.create_timer(1.0, self.link.heartbeat)
        self.create_timer(5.0, self.request_rc)
        self.create_timer(0.05, self.pump)
        self.create_timer(0.1, self.tick)
        self.request_rc()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def startup_complete(self):
        return self.now() - self.started >= HEARTBEAT_STARTUP_GRACE_S

    def on_auto(self, msg):
        self.auto_until = self.now() + 2.0 if msg.data else 0.0

    def request_rc(self):
        """Request RC_CHANNELS at 10 Hz."""
        self.link.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                          mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 100000)

    def update_heartbeat(self, pwm):
        """Record real transitions generated by EdgeTX on CH12."""
        if not valid(pwm):
            return

        state = pwm > self.high
        now = self.now()

        if self.heartbeat_state is None:
            self.heartbeat_state = state
            self.heartbeat_last_edge = now
            return

        if state == self.heartbeat_state:
            return

        self.heartbeat_state = state
        self.heartbeat_last_edge = now

        if not self.heartbeat_ready:
            self.heartbeat_ready = True
            self.get_logger().warn(f'RC heartbeat detected on ch{self.heartbeat_ch}')

    def pump(self):
        def handle(kind, message):
            if kind == 'RC_CHANNELS':
                self.rc_t = self.now()
                for ch in (self.kill_ch, self.arm_ch, self.heartbeat_ch, self.mode_ch):
                    self.rc[ch] = getattr(message, f'chan{ch}_raw', 0)
                self.update_heartbeat(self.rc.get(self.heartbeat_ch, 0))

            elif (kind == 'HEARTBEAT' and message.get_srcSystem() == 1
                    and message.get_srcComponent() == 1):
                self.fc_heartbeat_t = self.now()
                self.armed = bool(message.base_mode
                                  & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = self.link.mav.flightmode

        self.link.drain(handle)

    def fc_heartbeat_alive(self):
        if self.fc_heartbeat_t == 0.0 or (self.now() - self.fc_heartbeat_timeout_s) <= self.fc_heartbeat_t:
            return True
        return False

    def heartbeat_alive(self, rc_messages_fresh):
        if not rc_messages_fresh or not self.heartbeat_ready:
            return False
        return self.now() - self.heartbeat_last_edge <= HEARTBEAT_TIMEOUT_S

    def report_heartbeat(self, alive):
        """Log only when heartbeat status changes."""
        if not self.heartbeat_ready and not self.startup_complete:
            return
        if alive == self.heartbeat_reported:
            return

        self.heartbeat_reported = alive
        if alive:
            self.get_logger().warn('RC heartbeat -> OK')
        else:
            self.get_logger().error('RC heartbeat -> LOST')

    def disarm(self):
        """Send one nonblocking disarm packet."""
        self.link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def set_esc_power(self, enabled):
        """Command AUX OUT 1/SERVO9."""
        pwm = self.relay_run_pwm if enabled else self.relay_kill_pwm
        self.link.command(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, self.relay_servo, pwm)

    def tick(self):
        rc_messages_fresh = self.now() - self.rc_t <= RC_MESSAGE_STALE_S
        heartbeat_alive = self.heartbeat_alive(rc_messages_fresh)
        self.report_heartbeat(heartbeat_alive)
        self.fc_heartbeat_lost = not self.fc_heartbeat_alive()

        if self.pilot_failsafe(heartbeat_alive):
            return
        if not rc_messages_fresh:
            return

        kill_pwm = self.rc.get(self.kill_ch, 0)
        if valid(kill_pwm):
            self.do_kill(kill_pwm)
        if self.killed:
            return

     

        arm_pwm = self.rc.get(self.arm_ch, 0)
        if not valid(arm_pwm):
            if self.now() - self.arm_warn_t > ARM_WARN_S:
                self.arm_warn_t = self.now()
                self.get_logger().warn(f'arm channel ch{self.arm_ch} reads '
                                       f'{arm_pwm} - not being transmitted')
            return

        self.do_mode(self.rc.get(self.mode_ch, 0))
        self.do_arm(arm_pwm, heartbeat_alive)

    def do_mode(self, pwm):
        """CH7 picks MANUAL or STABILIZE. Edge triggered, never during a mission.

        STABILIZE holds attitude with the vertical thrusters, so it must be chosen,
        not landed in. Sent on the switch's edge rather than held, because a held
        mode would fight anything else that legitimately changes it.
        """
        if not valid(pwm):
            return                          # channel absent; leave the mode alone

        want = MODE_STABILIZE if pwm > self.high else MODE_MANUAL

        if self.mode_req is None:
            self.mode_req = want            # adopt the switch, command nothing
            return
        if want == self.mode_req:
            return

        self.mode_req = want

        # mission_base reads ANY departure from GUIDED as the pilot taking the
        # vehicle and exits, so a stray flick must not end a run.
        if self.mode in AUTO_MODES or self.now() <= self.auto_until:
            self.get_logger().warn('mode switch ignored - vehicle is autonomous')
            return

        self.get_logger().warn(
            f'mode -> {"STABILIZE" if want == MODE_STABILIZE else "MANUAL"} '
            f'- ch{self.mode_ch}={pwm}')
        self.link.set_mode(want)

    def do_kill(self, pwm):
        killed = pwm > self.high            # CH10 is reversed in the transmitter

        if killed != self.killed:
            self.mode_t = 0.0
            self.get_logger().warn(
                f'KILL {"ENGAGED" if killed else "released"} - ch{self.kill_ch}={pwm}')

        self.killed = killed
        self.set_esc_power(not killed)      # match AUX1 to the deliberate SA position

        if not killed:
            return

        # DISARM FIRST. pixhawk_led_node tests `not armed` before it looks at the mode,
        # so if the mode change lands first there is one heartbeat reading armed+MANUAL -
        # which is YELLOW. A kill must go green straight to red, never through yellow.
        self.disarm()

        if self.now() - self.mode_t > MODE_RESEND_S:
            self.mode_t = self.now()
            self.link.set_mode(MODE_MANUAL)

        self.armed_req = None
        self.tripped = False

    def pilot_failsafe(self, heartbeat_alive):
        """Latch heartbeat loss during pilot-controlled operation."""
        heartbeat_failed = (not heartbeat_alive
                            and (self.heartbeat_ready or self.startup_complete))
        pilot_controlled = (self.armed and self.mode is not None
                            and self.mode not in AUTO_MODES
                            and self.now() > self.auto_until)

        if pilot_controlled and heartbeat_failed and not self.tripped and self.fc_heartbeat_lost:
            self.tripped = True
            self.get_logger().error(
                'PILOT FAILSAFE - transmitter heartbeat lost '
                f'while armed in {self.mode} - cutting ESC power and disarming')

        if not self.tripped:
            return False

        self.set_esc_power(False)           # repeat both the power kill and the disarm
        self.disarm()

        # The latch clears only after heartbeat recovery and both switches
        # have been deliberately returned to their safe positions.
        if heartbeat_alive:
            kill_pwm = self.rc.get(self.kill_ch, 0)
            arm_pwm = self.rc.get(self.arm_ch, 0)
            switches_safe = (valid(kill_pwm) and valid(arm_pwm)
                             and kill_pwm > self.high    # SA up is the killed position
                             and arm_pwm <= self.high)
            if switches_safe:
                self.tripped = False
                self.armed_req = False
                self.get_logger().warn('pilot failsafe cleared - '
                                       'heartbeat restored, SA killed, SB disarmed')

        return True

    def do_arm(self, pwm, heartbeat_alive):
        want = pwm > self.high

        if self.armed_req is None:
            self.armed_req = want
            return

        if want != self.armed_req:
            self.armed_req = want

            if want and not heartbeat_alive:
                self.cmd_until = 0.0
                self.get_logger().error(
                    'ARM blocked - RC heartbeat is not active; '
                    'move SB down, fix CH12 heartbeat, then try again')
                return

            self.cmd_until = self.now() + CMD_HOLD_S
            self.get_logger().warn(
                f'{"ARM" if want else "DISARM"} requested - ch{self.arm_ch}={pwm}')

        # Never continue an outstanding arm request after heartbeat loss.
        if want and not heartbeat_alive:
            return

        if self.now() < self.cmd_until and self.armed != want:
            if want:
                # Whatever CH7 is asking for, defaulting to MANUAL until the switch
                # has been read. Explicit None test: MODE_STABILIZE is 0, so
                # `self.mode_req or MODE_MANUAL` would silently force MANUAL.
                self.link.set_mode(MODE_MANUAL if self.mode_req is None
                                   else self.mode_req)

            self.link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                              1 if want else 0)


def main():
    run(KillSwitch)
