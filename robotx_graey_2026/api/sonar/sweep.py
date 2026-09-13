"""Piece 1: talk to the sonar, hand back one sweep.

The only file that knows the hardware exists. Everything downstream works on a
Sweep object, so it works equally on a recorded one - which is what lets you
develop on a laptop with no sonar attached.

The brping import is deliberately inside the constructor rather than at the top
of the file, so Sweep and everything that reads a Sweep stays importable on a
machine with no sonar library installed.
"""
import math

import numpy as np

from . import settings as S


class Sweep:
    """One sweep, plus everything needed to interpret it.

    image        2D uint8. Rows are scan angles, columns are range bins.
    angles_deg   scan-plane angle of each row, in the 0=right/90=up/180=left/
                 270=down convention.
    heading_deg  the sub's compass heading when this sweep was taken, or None.
                 Carried along so later sweeps can be compared to earlier ones.
    """

    def __init__(self, image, angles_deg, metres_per_bin, heading_deg=None):
        self.image = image
        self.angles_deg = list(angles_deg)
        self.metres_per_bin = metres_per_bin
        self.heading_deg = heading_deg

    @property
    def step_deg(self):
        if len(self.angles_deg) < 2:
            return S.BEAM_IN_PLANE_DEG
        return abs(self.angles_deg[1] - self.angles_deg[0])

    def row_to_angle_deg(self, row):
        r = int(round(row))
        r = max(0, min(len(self.angles_deg) - 1, r))
        return self.angles_deg[r]

    def col_to_range_m(self, col):
        return col * self.metres_per_bin

    def offset_and_height(self, angle_deg, range_m):
        """Angle and range -> (offset, height) relative to the sonar.

        offset is positive to starboard, height is positive up - so anything
        below the sub comes back with a negative height.
        """
        a = math.radians(angle_deg)
        return range_m * math.cos(a), range_m * math.sin(a)


def gradian_to_angle_deg(gradian, down_gradian=None):
    """Hardware gradian -> scan-plane angle in our convention."""
    if down_gradian is None:
        down_gradian = S.DOWN_GRADIAN
    return (270.0 + (gradian - down_gradian) * S.DEG_PER_GRADIAN) % 360.0


def angle_deg_to_gradian(angle_deg, down_gradian=None):
    """Our convention -> hardware gradian."""
    if down_gradian is None:
        down_gradian = S.DOWN_GRADIAN
    g = down_gradian + (angle_deg - 270.0) / S.DEG_PER_GRADIAN
    return int(round(g)) % S.GRADIANS


class Sonar:
    def __init__(self, device=None, udp=None, baudrate=115200,
                 down_gradian=None):
        """device= for serial, or udp=(host, port) to go through pingproxy.

        Going through pingproxy lets Ping Viewer stay connected at the same
        time, which is worth having while you are still learning what the
        returns look like. Test that both really can hold a connection at once
        before you depend on it.
        """
        from brping import Ping360

        self._ping = Ping360()
        if udp is not None:
            self._ping.connect_udp(*udp)
        elif device is not None:
            self._ping.connect_serial(device, baudrate)
        else:
            raise ValueError("give device= for serial or udp=(host, port)")

        if not self._ping.initialize():
            raise RuntimeError("Ping360 did not initialize. Powered? It needs "
                               "11-25 V of its own; USB does not run it.")

        self.down_gradian = S.DOWN_GRADIAN if down_gradian is None else down_gradian
        self._configured_range = None

    def _configure_range(self, max_range_m):
        """Set sample count and timing for the range we want.

        To hear something max_range away, sound must travel there and back, so
        the sonar listens for 2*max_range/speed seconds. That listening time is
        divided into range bins; sample count times sample period has to come
        out to exactly that.
        """
        if self._configured_range == max_range_m:
            return
        n = int(min(S.MAX_SAMPLES,
                    2 * max_range_m / (S.TICK * S.MIN_SAMPLE_PERIOD * S.SPEED_OF_SOUND)))
        period = int(2 * max_range_m / (n * S.TICK * S.SPEED_OF_SOUND))
        duration = int(max(period * S.TICK / 400,
                           (8000 * max_range_m) / S.SPEED_OF_SOUND))

        self._ping.set_number_of_samples(n)
        self._ping.set_sample_period(period)
        self._ping.set_transmit_duration(duration)

        self.n_samples = n
        self.metres_per_bin = max_range_m / n
        self._configured_range = max_range_m

    def sweep(self, start_deg, end_deg, step_deg, max_range_m, heading_deg=None,
              on_ping=None):
        """Sweep an arc of the scan plane and return a Sweep.

        Angles are in our convention; they get converted to gradians here so
        nothing downstream ever has to think about the hardware's units.

        on_ping(partial) is called as the head moves, with a Sweep holding only
        what has been measured so far. A full sweep takes many seconds - the
        motor step dominates, not the sound - so a display that waits for the
        whole thing looks frozen. The partial is a real Sweep, so a viewer needs
        no special case for it.
        """
        self._configure_range(max_range_m)

        angles = []
        a = float(start_deg)
        while a <= end_deg + 1e-9:
            angles.append(a % 360.0)
            a += step_deg

        rows = []
        for ang in angles:
            resp = self._ping.transmitAngle(angle_deg_to_gradian(ang, self.down_gradian))
            data = getattr(resp, "data", None)
            if data is None:
                rows.append(np.zeros(self.n_samples, dtype=np.uint8))
                continue
            row = np.frombuffer(data, dtype=np.uint8)
            if len(row) < self.n_samples:
                row = np.pad(row, (0, self.n_samples - len(row)))
            rows.append(row[: self.n_samples])

            if on_ping is not None:
                on_ping(Sweep(np.vstack(rows), angles[:len(rows)],
                              self.metres_per_bin, heading_deg))

        image = np.vstack(rows) if rows else np.zeros((0, 1), dtype=np.uint8)
        return Sweep(image, angles, self.metres_per_bin, heading_deg)

    def sweep_for_state(self, state, heading_deg=None):
        """Sweep using the settings for whichever state the driver is in."""
        cfg = S.SWEEP[state]
        return self.sweep(cfg["start_deg"], cfg["end_deg"], cfg["step_deg"],
                          cfg["max_range_m"], heading_deg)
