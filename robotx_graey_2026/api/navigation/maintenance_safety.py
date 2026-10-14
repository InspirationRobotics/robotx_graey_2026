"""Fresh control-state requirements for disarmed navigation maintenance."""
import json
import urllib.request


def maintenance_reason(c):
    if not (c.get('online') is True and c.get('vehicle_fresh') is True
            and c.get('armed') is False and c.get('sa_active') is True
            and c.get('esc_commanded_on') is False
            and c.get('mission_claimed') is False and c.get('autonomous') is False):
        return 'Keep vehicle disarmed, kill engaged, ESC power off and no mission; fresh controller telemetry required'
    return ''


def require_maintenance():
    with urllib.request.urlopen('http://localhost:8090/api/controller', timeout=.2) as source:
        c = json.load(source)
    reason = maintenance_reason(c)
    if reason:
        raise ValueError(reason)
    return c
