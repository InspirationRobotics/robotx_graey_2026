"""Continuous fusion supervision: no EKF selection or bridge reset actions."""
import math
from dataclasses import dataclass

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
    reference_ready: bool = False
    reference_saved: bool = False
    configuration_verified: bool = False
    heading_allowed: bool = False
    heading_reason: str = 'Pre-dive heading check unavailable'
    surface_authoritative: bool = False
    surface_observed: object = None


class ContinuousSupervisor:
    def __init__(self, active=False, surface_enter=.15, surface_exit=.4, depth_dwell=2.):
        if not 0 <= surface_enter < surface_exit:
            raise ValueError('Surface thresholds must be ordered')
        self.active = active
        self.enter, self.exit, self.dwell = surface_enter, surface_exit, depth_dwell
        self.surface = self.shallow_since = self.deep_since = self.last_step = None
        self.fault = self.run_id = ''
        self.state, self.reason = 'Initializing', 'Waiting for navigation telemetry'
        self.allowed = self.dive_heading_approved = False

    def step(self, now, o):
        self.allowed = False
        if self.last_step is not None and not 0 <= now-self.last_step < 1.:
            self.fault = 'Supervisor clock discontinuity; revalidate navigation'
        self.last_step = now
        if o.run_id != self.run_id:
            self.run_id = o.run_id
            self.dive_heading_approved = False
        depth_known = o.depth_valid and math.isfinite(o.depth)
        if o.surface_authoritative:
            self.surface = o.surface_observed if depth_known else None
        elif depth_known:
            self.shallow_since = (now if self.shallow_since is None else self.shallow_since) if o.depth <= self.enter else None
            self.deep_since = (now if self.deep_since is None else self.deep_since) if o.depth >= self.exit else None
            if self.shallow_since is not None and now-self.shallow_since >= self.dwell:
                self.surface = True
            if self.deep_since is not None and now-self.deep_since >= self.dwell:
                self.surface = False
        else:
            self.shallow_since = self.deep_since = None
        self.state = 'GPS aiding admitted' if o.gps_good else 'DVL continuation'
        if self.fault:
            self.state, self.reason = 'Fault', self.fault
        elif not depth_known or self.surface is None:
            self.reason = 'Calibrated depth or surface state unavailable'
        elif not o.healthy:
            self.reason = o.reason
        elif not o.configuration_verified:
            self.reason = 'Continuous fusion configuration not commissioned'
        elif not o.intent_fresh:
            self.reason = 'Waiting for mission intent'
        else:
            diving = o.intent in ('DIVE', 'UNDERWATER')
            if o.intent == 'SURFACE_NAV':
                if not o.reference_ready:
                    self.reason = 'Waiting for admitted surface GPS and validated heading to capture reference'
                    if self.active and self.surface and o.gps_good and o.heading_allowed:
                        self.dive_heading_approved = True
                        return [('capture_reference', None)]
                else:
                    # Mission adapter separately bounds GPS outages. Never
                    # turn each rejected GPS packet into a permission reset.
                    self.allowed = bool(self.active and self.surface
                        and self.dive_heading_approved and o.reference_saved)
                    self.reason = ('Surface GPS aiding' if o.gps_good else 'Surface DVL/VN continuation') if self.allowed else 'Surface navigation reference unavailable'
            elif not diving:
                self.dive_heading_approved = False
                self.allowed = bool(self.active and o.gps_good and self.surface)
                self.reason = 'GPS admitted' if self.allowed else 'Waiting for admitted GPS at surface'
            elif not o.reference_ready:
                self.reason = 'Waiting for surface GPS and validated heading to capture reference'
                if self.active and self.surface and o.gps_good and o.heading_allowed:
                    self.dive_heading_approved = True
                    return [('capture_reference', None)]
            else:
                self.allowed = bool(self.active and self.dive_heading_approved and o.reference_saved)
                self.reason = 'Continuous estimate healthy' if self.allowed else 'Dive preparation not validated'
        return []

    def snapshot(self, o):
        surface = self.surface if o.depth_valid and math.isfinite(o.depth) else None
        return dict(state=self.state, reason=self.reason,
            mode='Active' if self.active else 'Observation', fusion_policy='continuous',
            requested_source='Continuous fusion',
            confirmed_source='Continuous fusion' if o.configuration_verified else 'Unverified',
            navigation_ready=self.allowed, surfaced=surface,
            surface_state='Surfaced' if surface is True else 'Submerged' if surface is False else 'Unknown',
            run_id=self.run_id)
