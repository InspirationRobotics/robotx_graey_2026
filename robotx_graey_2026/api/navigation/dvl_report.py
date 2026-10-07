"""Validate a velocity report before publishing any part of the measurement."""
import math


def validate_report(report, scale):
    if not isinstance(report, dict):
        raise ValueError('DVL report must be an object')
    if 'vx' not in report:
        return None  # Non-velocity reports share the same stream.
    velocity = [float(report[key])*scale for key in ('vx', 'vy', 'vz')]
    altitude = float(report.get('altitude', -1))
    if not all(math.isfinite(v) for v in velocity+[altitude]):
        raise ValueError('Nonfinite DVL measurement')
    valid = report.get('velocity_valid', False)
    if not isinstance(valid, bool):
        raise ValueError('DVL validity must be boolean')
    covariance = report.get('covariance')
    if covariance is not None:
        if len(covariance) != 3 or any(len(row) != 3 for row in covariance):
            raise ValueError('DVL covariance must be 3 by 3')
        covariance = [[float(value)*scale*scale for value in row] for row in covariance]
        if not all(math.isfinite(v) for row in covariance for v in row):
            raise ValueError('Nonfinite DVL covariance')
        if any(covariance[i][i] < 0 for i in range(3)):
            raise ValueError('Negative DVL variance')
    return velocity, altitude, valid, covariance
