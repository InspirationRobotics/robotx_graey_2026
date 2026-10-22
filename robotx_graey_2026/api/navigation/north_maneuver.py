"""Bounded surface alignment decisions, independent of Cube position/yaw.

No arming, mode changes, kill recovery or north guessing. The operator starts
already armed in MANUAL with a confirmed clear corridor. Neutral means stop
commanding thrust, not a guarantee of geographic position hold in current.
"""
import math


class NorthManeuver:
    terminal = {'COMPLETE', 'ABORTED'}

    def __init__(self, now, max_distance):
        if isinstance(max_distance, bool) or not isinstance(max_distance, (int,float)) or not math.isfinite(max_distance) or not max_distance == 10:
            raise ValueError('Confirm 10 m of clear travel plus additional stopping clearance')
        self.limit = max_distance
        self.stage, self.reason = 'MOVE', 'Collecting surface GPS/DVL north alignment'
        self.started = self.last = now
        self.distance = 0.
        self.heading = self.still_since = self.bad_since = None
        self.thrust = 0.
        self.speed = 0.
        self.integral = 0.
        self.lateral_command = 0

    def abort(self, reason):
        self.stage, self.reason = 'ABORTED', reason
        self.thrust=self.lateral_command=0
        return (0,0)  # Neutral forward/yaw; adapter yields after takeover.

    def step(self, now, o):
        if self.stage in self.terminal:
            return (0,0)
        dt, self.last = now-self.last, now
        if not 0 <= dt <= .3 or now-self.started > 60:
            return self.abort('Alignment timing limit exceeded')
        if not o.get('control_ok') or o.get('abort'):
            return self.abort('Kill, pilot takeover, disarm, cancel or watchdog loss')
        if not o.get('fresh') or not isinstance(o.get('depth'), (int,float)) or not -.1 <= o['depth'] <= .15:
            return self.abort('Fresh surface pressure/VN/bridge telemetry required')
        heading, speed, rate = o['yaw'], o['speed'], o['rate']
        if not all(math.isfinite(v) for v in (heading,speed,rate)):
            return self.abort('Invalid alignment sensor values')
        self.speed = speed
        self.heading = heading if self.heading is None else self.heading
        error = (self.heading-heading+180)%360-180
        if abs(error) > 20 or abs(rate) > 15:
            return self.abort('Relative heading changed excessively; alignment stopped')
        if not o.get('dvl_valid') or not o.get('gps_qualified'):
            self.distance += (abs(speed) if o.get('dvl_valid') else 1.)*dt
            if self.distance >= self.limit:
                return self.abort('Travel limit reached during sensor loss')
            self.bad_since = now if self.bad_since is None else self.bad_since
            self.still_since = None
            self.thrust = 0.
            self.lateral_command = 0
            self.reason = 'Paused: waiting for valid DVL and qualified GPS'
            if now-self.bad_since > 5:
                return self.abort('DVL/GPS did not recover within alignment pause')
            return (0,0)
        self.bad_since = None
        self.distance += abs(speed)*dt
        if self.distance >= self.limit+2 or speed > 1.:
            return self.abort('Travel or speed limit reached; stop and inspect clearance')
        yaw_command = round(max(-150,min(150,8*error-3*rate)))
        if self.stage == 'MOVE':
            if self.distance >= self.limit-.5:
                if not o.get('candidate_ready'):
                    return self.abort('10 m run did not produce a reliable GPS coordinate track; no north accepted')
                self.stage, self.reason = 'SETTLE', 'DVL velocity braking; verifying stationary GPS/DVL for 3 seconds'
                self.thrust = 0.
            elif now-self.started > 45:
                return self.abort('No accepted north alignment within 45 seconds')
            else:
                # Bounded speed feedback with rate-limited thrust. No geographic
                # position or Cube compass is used before alignment exists.
                self.integral=max(0.,min(300.,self.integral+120*(.4-o['forward'])*dt))
                target = max(0.,min(450.,400.*(.4-o['forward'])+self.integral))
                self.lateral_command=round(max(-150,min(150,-400*o.get('sideways',0.))))
                self.thrust = min(target,self.thrust+400.*dt)
                return (round(self.thrust),yaw_command)
        if self.stage == 'SETTLE':
            self.thrust=round(max(-300,min(300,-1000*o['forward'])))
            self.lateral_command=round(max(-300,min(300,-1000*o.get('sideways',0.))))
            gs=o.get('gps_speed')
            stationary = (speed <= .05 and abs(rate) <= 2 and isinstance(gs,(int,float)) and math.isfinite(gs) and gs <= .2)
            self.still_since = (now if self.still_since is None else self.still_since) if stationary else None
            if self.still_since is not None and now-self.still_since >= 3:
                if not o.get('candidate_ready'):
                    return self.abort('GPS coordinate track lost validity during stop; no north accepted')
                self.stage, self.reason = 'COMPLETE', 'North motion alignment and stationary verification complete; disarm and engage kill'
                self.thrust=self.lateral_command=0
                return (0,0)
            return (self.thrust,yaw_command)
        return (0,0)

    def snapshot(self):
        return dict(stage=self.stage,reason=self.reason,distance_m=self.distance,
                    measured_speed_mps=self.speed,forward_command=self.thrust,
                    sideways_command=self.lateral_command,
                    maximum_distance_m=self.limit,reference_relative_heading_deg=self.heading)
