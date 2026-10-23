"""Commissioning permission checks; never arms, selects modes or clears faults."""
import math


def commission_reason(controller, bridge, cube_healthy, pose, heading, surface, intent):
    if not (controller.get('vehicle_fresh') is True and controller.get('armed') is False
            and controller.get('sa_active') is True and controller.get('esc_commanded_on') is False
            and controller.get('mission_claimed') is False and controller.get('autonomous') is False):
        return 'Fresh disarmed vehicle, kill engaged, ESC off and no mission required'
    if intent:
        return 'Mission intent is active; cannot commission during a run'
    local_frame=bridge.get('coordinate_frame')=='fixed_local' and bridge.get('local_navigation_ready') is True
    if not all(bridge.get(k) is True for k in ('healthy', 'navigation_valid',
            'configuration_verified', 'transmitting')) or not (local_frame or bridge.get('heading_reference_validated') is True):
        return 'Healthy aligned odometry output and verified Cube profile required'
    if (bridge.get('gps_admission') or {}).get('qualified') is not True:
        return 'Qualified surface GPS required'
    if surface is not True or not isinstance(bridge.get('depth_m'), (int, float)) or not abs(bridge['depth_m']) <= .1:
        return 'Confirmed surface with calibrated depth near zero required'
    if not cube_healthy or not pose or not all(math.isfinite(v) for v in pose.get('ned', [])) or len(pose.get('ned', [])) != 3:
        return 'Healthy Cube estimate and fresh local position required'
    if not heading.get('allowed'):
        return heading.get('reason', 'VN/Cube heading consistency required')
    return ''
