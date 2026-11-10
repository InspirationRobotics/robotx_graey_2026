"""ADAPTED: GRAEY live calibration/watchdog updates, without control recovery."""
import math


def values(parameters):return {p.name:p.value for p in parameters}


def positive(value,name):
    if type(value) not in (float,int) or not math.isfinite(value) or value<=0:
        raise ValueError(name+' must be finite and positive')
    return float(value)


def dvl_scale(node,parameters,now):
    changes=values(parameters)
    if 'scale' not in changes:return
    value=positive(changes['scale'],'scale')
    c=getattr(node,'runtime_control',{})
    if not (0<=now-getattr(node,'runtime_control_at',-math.inf)<1 and c.get('vehicle_fresh') is True
            and c.get('armed') is False and not c.get('mission_claimed')):
        raise ValueError('Fresh disarmed telemetry and stopped missions required for DVL calibration')
    node.scale=value


def water_density(node,parameters,now):
    changes=values(parameters)
    if 'water_density' not in changes:return
    value=positive(changes['water_density'],'water_density')
    c=node.control_state
    if (node.output_permitted or not (node.control_received is not None and 0<=now-node.control_received<1
            and c.get('vehicle_fresh') is True and c.get('armed') is False and not c.get('mission_claimed'))):
        raise ValueError('Pause odometry with fresh disarmed telemetry before changing water density; Prepare is required afterward')
    node.settings['water_density']=value
    node.est.pressure_depth(now,math.nan)


def tether_timing(node,parameters):
    changes=values(parameters)
    keys={'tether_ping_interval_s','tether_timeout_s'}
    if not keys.intersection(changes):return
    interval=positive(changes.get('tether_ping_interval_s',node.probe.interval),'tether_ping_interval_s')
    timeout=positive(changes.get('tether_timeout_s',node.probe.timeout),'tether_timeout_s')
    if interval<.5 or timeout<interval+1:raise ValueError('Ping interval must be >=0.5 s and timeout >= interval+1 s')
    if not (node.vehicle_fresh() and node.armed is False and not node.esc_commanded_on and not node.mission_claimed()):
        raise ValueError('Fresh disarmed telemetry, stopped missions and inhibited ESC power required')
    node.probe.interval,node.probe.timeout=interval,timeout


def result(callback,parameters):
    from rcl_interfaces.msg import SetParametersResult
    try:callback(parameters);return SetParametersResult(successful=True)
    except (ValueError,TypeError) as exc:return SetParametersResult(successful=False,reason=str(exc))
