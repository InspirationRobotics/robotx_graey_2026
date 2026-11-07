"""Motion arbitration at MAVProxy ingress; no serial ownership or ROS dependency."""
import time

class MissionInputGate:
    def __init__(self, clock=time.monotonic):
        self.clock=clock
        self.last=None
        self.revoked=False
        self.takeover_requested=False

    def allow(self, msg):
        now=self.clock()
        active=self.last is not None and 0 <= now-self.last < 0.75
        if not active:
            self.revoked=False
        owner=(msg.get_srcSystem(),msg.get_srcComponent()) == (255,198)
        kind=msg.get_type()
        command=getattr(msg,'command',None)
        disarm=kind in ('COMMAND_LONG','COMMAND_INT') and command==400 and getattr(msg,'param1',1)==0
        mode=kind=='SET_MODE' or (kind in ('COMMAND_LONG','COMMAND_INT') and command in (176,192))
        if not owner and (disarm or mode):
            if active:self.revoked=True
            return True
        if owner:
            if kind=='MANUAL_CONTROL':
                self.last=now
                return not self.revoked
            if kind=='RC_CHANNELS_OVERRIDE':
                # The mission sends this only to release its pilot overrides.
                release=all(getattr(msg,'chan%d_raw'%i,65535)==0 for i in range(1,9))
                if release:
                    self.last=None;self.revoked=False
                return True
            if active and self.revoked and not disarm:
                # A pilot takeover must not be undone by a late mission mode/arm command.
                if mode or command==400:return False
            return True
        if active and not self.revoked:
            if kind=='MANUAL_CONTROL':
                # Any explicit joystick button is pilot takeover, including disarm.
                # Neutral/stick-only packets cannot overwrite autonomous motion.
                if getattr(msg,'buttons',0) or getattr(msg,'buttons2',0):
                    self.revoked=True
                    self.takeover_requested=True
                    return True
                return False
            if kind=='RC_CHANNELS_OVERRIDE':return False
        return True
