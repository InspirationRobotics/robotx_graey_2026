"""Explicit Prepare warm restart of receive guards, never of the EKF frame."""
from .gps_admission import CubeClock


def recover_ingress(bridge, now):
    recovered = []
    for key, when in list(bridge.ingress_rejections.items()):
        if 0 <= now - when < 3.:
            bridge.stamps.pop(key, None)
            recovered.append(key)
        bridge.ingress_rejections.pop(key, None)
    gps = bridge.est.gps
    cube_restart = bridge.est.fault == 'Cube restarted; revalidate navigation frame'
    if cube_restart or gps.fault == 'Cube GPS clock regressed; restart and revalidate':
        # Keep position, covariance, north offset, quality history and pressure
        # baseline. Only the Cube clock/measurement epoch is obsolete.
        gps.clock = CubeClock()
        gps.last_fix = None
        gps.fault = ''
        gps.invalidate('Prepare restarted Cube timing; awaiting fresh synchronized measurements')
        bridge.stamps.pop('pressure', None)
        bridge.boot_ms = None
        bridge.params.clear()
        bridge.pressures.clear()
        bridge.pressure_sample = None
        bridge.est.depth_at = None
        if cube_restart:
            bridge.est.fault = ''
        recovered.append('cube_timing')
    return dict(at=now, recovered=recovered,
                message=('Sensor ingestion restarted for '+', '.join(recovered)+
                         '; awaiting fresh samples; position and heading retained'
                         if recovered else 'No latched ingress rejection; sensor health shown live'))
