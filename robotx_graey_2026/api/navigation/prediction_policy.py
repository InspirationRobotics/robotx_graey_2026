"""Keep measurement availability separate from flight-controller estimate health."""
import math


def cube_prediction_allowed(bridge, now, cube_healthy):
    """No permission from stale status, a paused bridge, or a broken frame."""
    return bool(cube_healthy and bridge.get('coordinate_frame') == 'fixed_local'
        and bridge.get('initialized') is True and bridge.get('enabled') is True
        and bridge.get('configuration_verified') is True and not bridge.get('fault')
        and bridge.get('heading_continuity_lost') is False
        and (bridge.get('inputs') or {}).get('vn') is True
        and 0 <= now-bridge.get('published_monotonic', -math.inf) < .3
        and 0 <= now-(bridge.get('pressure_sample') or {}).get('received', -math.inf) < .5)


def new_dvl_measurement(est, now, last_stamp):
    return bool(est.motion_valid and est.fresh(est.motion_at, now)
        and est.dvl_gap_start is None and est.motion_at != last_stamp)
