"""A dive's geographic marker and mission NED zero, owned by Jetson.

Never changes Cube home/origin or GUI display origin. The frame is translated,
not rotated: north/east/down retain the Cube NED axes. Restored files are history
until the caller explicitly validates their vehicle/frame epoch.
"""
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import tempfile
import uuid


def vector(value):
    if len(value) != 3 or not all(math.isfinite(float(v)) for v in value):
        raise ValueError('Expected three finite coordinates')
    return tuple(float(v) for v in value)


@dataclass(frozen=True)
class MissionReference:
    reference_id: str
    run_id: str
    vehicle_epoch: str
    gps_lat: float
    gps_lon: float
    gps_msl_alt: float
    gps_accuracy_m: float
    gps_time_us: int
    cube_ned: tuple
    captured_utc: str

    @classmethod
    def capture(cls, *, run_id, vehicle_epoch, gps, pose, now, captured_utc,
                surface_confirmed, gps_qualified, ekf_healthy,
                max_age_s=1.0, max_skew_s=.5, max_accuracy_m=3.0):
        """Capture paired fresh inputs; receipt-time skew is not time sync proof.

        Supervisor additionally requires a new GPS fix after the dive request,
        confirmed GPS source selection and stable fusion before calling this.
        gps: lat/lon/alt/accuracy/fix/receiver_time_us/received
        pose: ned/received. Both received times use the same monotonic clock.
        """
        if not (surface_confirmed and gps_qualified and ekf_healthy):
            raise ValueError('Surface, qualified GPS and healthy EKF required')
        if not run_id or not vehicle_epoch:
            raise ValueError('Run and vehicle-frame epoch required')
        for sample in (gps, pose):
            age = now - sample['received']
            if not math.isfinite(age) or not 0 <= age <= max_age_s:
                raise ValueError('Stale or future-dated measurement')
        if abs(gps['received']-pose['received']) > max_skew_s:
            raise ValueError('GPS and pose receipt times too far apart')
        lat, lon, alt = vector((gps['lat'], gps['lon'], gps['alt']))
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError('Invalid geographic coordinates')
        accuracy = float(gps['accuracy'])
        if (gps['fix'] < 3 or gps['receiver_time_us'] <= 0
                or not math.isfinite(accuracy) or not 0 < accuracy <= max_accuracy_m):
            raise ValueError('GPS fix/accuracy unavailable or unacceptable')
        return cls(str(uuid.uuid4()), run_id, vehicle_epoch, lat, lon, alt,
                   accuracy, int(gps['receiver_time_us']), vector(pose['ned']), captured_utc)

    def _check_epoch(self, vehicle_epoch):
        if not vehicle_epoch or vehicle_epoch != self.vehicle_epoch:
            raise ValueError('Reference belongs to a different or unverified Cube frame')

    def from_cube(self, ned, *, vehicle_epoch):
        self._check_epoch(vehicle_epoch)
        return tuple(a-b for a, b in zip(vector(ned), self.cube_ned))

    def to_cube(self, mission_ned, *, vehicle_epoch):
        self._check_epoch(vehicle_epoch)
        return tuple(a+b for a, b in zip(vector(mission_ned), self.cube_ned))

    def snapshot(self):
        return dict(schema=1, **asdict(self))

    def save(self, path):
        """Atomic persistence; failure must block dive permission at caller."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                             dir=path.parent, delete=False) as handle:
                temporary = handle.name
                json.dump(self.snapshot(), handle, allow_nan=False, indent=2)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    @classmethod
    def load_history(cls, path):
        """Load for display/review; does not grant live-navigation permission."""
        with open(path, encoding='utf-8') as handle:
            value = json.load(handle)
        if value.pop('schema', None) != 1:
            raise ValueError('Unsupported reference format')
        value['cube_ned'] = vector(value['cube_ned'])
        vector((value['gps_lat'], value['gps_lon'], value['gps_msl_alt']))
        return cls(**value)
