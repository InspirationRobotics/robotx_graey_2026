"""Deterministic navigation supervisor. No transport, motor or parameter writes."""
from dataclasses import dataclass
import math

LABELS = {1: 'Underwater', 2: 'Surface GPS'}


@dataclass
class Observation:
    healthy: bool = False
    reason: str = 'Waiting for navigation telemetry'
    depth: float = float('nan')
    depth_valid: bool = False
    intent: str = ''
    intent_fresh: bool = False
    run_id: str = ''
    gps_good: bool = False
    gps_updated: float = -math.inf
    source: int = 0
    source_updated: float = -math.inf
    ack: str = ''
    ack_updated: float = -math.inf
    alignment_id: str = ''
    reference_ready: bool = False
    reference_saved: bool = False
    configuration_verified: bool = False
    continuity_ok: bool = True


class Supervisor:
    def __init__(self, active=False, surface_enter=.15, surface_exit=.4,
                 depth_dwell=2., gps_dwell=3., dropout_grace=3., timeout=3.):
        if not 0 <= surface_enter < surface_exit:
            raise ValueError('Surface thresholds must be ordered')
        self.active = active
        self.enter, self.exit = surface_enter, surface_exit
        self.dwell, self.gps_dwell = depth_dwell, gps_dwell
        self.grace, self.timeout = dropout_grace, timeout
        self.surface = False
        self.shallow_since = self.deep_since = self.good_since = None
        self.last_good = None
        self.pending = None
        self.alignment = None
        self.serial = 0
        self.fault = ''
        self.run_id = ''
        self.dive_since = None
        self.state = 'Initializing'
        self.reason = 'Waiting for navigation telemetry'
        self.desired = 1
        self.allowed = False
        self.healthy_since = None
        self.source_since = None
        self.last_source = 0

    def step(self, now, o):
        actions = []
        self.allowed = False
        self.healthy_since = (now if self.healthy_since is None else self.healthy_since) if o.healthy else None
        if o.source != self.last_source or not o.healthy:
            self.source_since = now
        self.last_source = o.source
        if o.run_id != self.run_id:
            self.run_id = o.run_id
            self.dive_since = None
        if o.intent == 'DIVE' and self.dive_since is None:
            self.dive_since = now
        if o.intent != 'DIVE':
            self.dive_since = None
        if o.gps_good:
            if self.good_since is None:
                self.good_since = now
        else:
            self.good_since = None
        qualified = self.good_since is not None and now-self.good_since >= self.gps_dwell
        if qualified:
            self.last_good = now

        if not o.depth_valid or not math.isfinite(o.depth):
            self.shallow_since = self.deep_since = None
        else:
            self.shallow_since = (now if self.shallow_since is None else self.shallow_since) if o.depth <= self.enter else None
            self.deep_since = (now if self.deep_since is None else self.deep_since) if o.depth >= self.exit else None
            if self.shallow_since is not None and now-self.shallow_since >= self.dwell:
                self.surface = True
            if self.deep_since is not None and now-self.deep_since >= self.dwell:
                self.surface = False

        if self.pending:
            target, sent = self.pending
            if o.ack_updated > sent and o.ack == 'rejected':
                self.fault = 'Pixhawk rejected the source change'
            elif (o.source == target and o.source_updated > sent
                  and o.ack_updated > sent and o.ack == 'accepted'):
                if not o.continuity_ok:
                    self.fault = 'Position discontinuity after source change'
                else:
                    self.pending = None
                    self.alignment = None
            elif now-sent > self.timeout:
                self.fault = 'Source change not confirmed before timeout'
        if self.alignment and now-self.alignment[1] > self.timeout and not self.pending:
            self.fault = 'External position alignment timed out'
        if self.fault:
            self.state, self.reason = 'Fault', self.fault
            return actions
        if not o.healthy or not o.depth_valid or not o.intent_fresh:
            self.state = 'Waiting for healthy navigation'
            self.reason = o.reason if not o.healthy else ('Depth reference unavailable' if not o.depth_valid else 'Mission intent unavailable or stale')
            return actions
        if self.pending:
            self.state = 'Switching to ' + LABELS[self.pending[0]].lower()
            self.reason = 'Waiting for command acknowledgment and fresh source evidence'
            return actions

        if (self.healthy_since is None or now-self.healthy_since < 1. or
                self.source_since is None or now-self.source_since < 1.):
            self.state, self.reason = 'Checking estimate', 'Waiting for stable healthy estimate and selected source'
            return actions

        diving = o.intent in ('DIVE', 'UNDERWATER') or not self.surface
        self.desired = 1 if diving else 2 if qualified or o.source == 2 else 1
        if diving:
            self.state = 'Preparing dive' if o.intent == 'DIVE' else 'Underwater'
            if o.intent == 'DIVE' and not o.reference_ready:
                self.reason = 'Waiting for a new qualified GPS fix and saved dive reference'
                if (self.surface and qualified and o.source == 2 and
                        o.gps_updated > self.dive_since):
                    actions.append(('capture_reference', None))
                elif self.surface and qualified and o.source != 2:
                    self.desired = 2
                elif self.surface:
                    return actions
                # If unexpectedly already submerged, still request underwater
                # sources but never grant a dive permission without a reference.
        else:
            self.state = 'Surface GPS' if qualified and o.source == 2 else 'Acquiring GPS'
            if o.source == 2 and not qualified:
                self.state = 'Surface GPS degraded'

        if self.desired != o.source:
            self.reason = 'Recommend ' + LABELS[self.desired]
            if not self.active:
                self.reason += ' — observation mode'
                return actions
            if not o.configuration_verified:
                self.reason = 'Active switching blocked: configuration/frame validation incomplete'
                return actions
            if o.source not in (1, 2):
                self.reason = 'Current source unknown; waiting for fresh source evidence'
                return actions
            if self.desired == 1 and o.source == 2:
                if self.alignment is None:
                    self.serial += 1
                    self.alignment = (f'align-{self.serial}', now)
                    actions.append(('align_external_position', self.alignment[0]))
                if o.alignment_id != self.alignment[0]:
                    self.state, self.reason = 'Aligning underwater position', 'Waiting for navigation bridge alignment'
                    return actions
            elif self.desired == 1 and o.source == 0:
                self.reason = 'Current source unknown; cannot safely align external position'
                return actions
            self.pending = (self.desired, now)
            actions.append(('select_source', self.desired))
            self.state = 'Switching to ' + LABELS[self.desired].lower()
            return actions

        self.reason = 'Navigation inputs healthy'
        if self.state == 'Surface GPS degraded':
            self.reason = 'GPS unavailable; bounded DVL continuation'
        permitted = (diving and o.reference_ready and o.reference_saved) or (
            not diving and o.source == 2 and (qualified or (
                self.last_good is not None and now-self.last_good <= self.grace)))
        if not permitted:
            self.reason = 'Waiting for dive reference' if diving else 'Waiting for reliable GPS'
        self.allowed = bool(self.active and o.configuration_verified and permitted)
        return actions

    def snapshot(self, o):
        return dict(state=self.state, reason=self.reason,
                    mode='Active' if self.active else 'Observation',
                    requested_source=LABELS.get(self.desired, 'Unknown'),
                    confirmed_source=LABELS.get(o.source, 'Unknown'),
                    navigation_ready=self.allowed, surfaced=self.surface,
                    run_id=self.run_id)
