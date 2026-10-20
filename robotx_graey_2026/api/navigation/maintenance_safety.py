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


def compass_setup_reason(c):
    """Selecting compass yaw needs disarmed MANUAL, not an ESC-off checkbox."""
    if not (c.get('online') is True and c.get('vehicle_fresh') is True):
        return 'Fresh controller and vehicle telemetry required'
    if c.get('armed') is not False:
        return 'Disarm before setting up Compass Learn; learning itself runs armed'
    if c.get('mode') != 'MANUAL':
        return 'Select MANUAL before setting up Compass Learn'
    if c.get('mission_claimed') is not False or c.get('autonomous') is not False:
        return 'Stop the mission before setting up Compass Learn'
    return ''


def require_compass_setup():
    return require_maintenance(compass_setup_reason)


def require_maintenance(check=maintenance_reason):
    with urllib.request.urlopen('http://localhost:8090/api/controller', timeout=.2) as source:
        c = json.load(source)
    reason = check(c)
    if reason:
        raise ValueError(reason)
    return c
