"""Recordings: everything one stretch of sonar work saw, in one folder.

Press Record on the Map tab (with a note saying what the run is) and until
Stop, sonar_map.py writes into sonar_data/rec_<date_time>_<note>/:

  meta.json     the note, start and end, every setting (range, sector, step,
                threshold, floor cut), how the sonar is mounted, the Ping360's
                own settings (gain, frequency, pulse length, sample period) and
                which code version ran
  sweep_*.npz   every sweep, as the map tool always saves them: the pings, the
                pose at each ping (position, heading, roll, pitch, DVL altitude)
  nav.jsonl     every VN-100 and DVL reading as it arrived, full rate, one JSON
                per line with its time ("att" quaternion, "vel" body m/s,
                "ok" DVL valid, "alt" DVL altitude)
  marks.jsonl   Mark presses with the note box's text, and (auto) every Start,
                Pause, Reset and settings change

Times are the Jetson's monotonic clock, the same one the pings carry, so the
sweeps, nav readings and marks line up. For Ruth's reference, and for
calibrating the simulator against what the real sonar heard (sim/POOL_CALIBRATION.md).
"""
import json
import os
import re
import subprocess
import threading
import time


def _code_version():
    try:
        return subprocess.run(["git", "describe", "--always", "--dirty"], capture_output=True,
                              text=True, timeout=2).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


class Recorder:
    def __init__(self, base_dir="sonar_data"):
        self.base_dir = base_dir
        self.dir = None
        self._lock = threading.Lock()
        self._nav = self._marks = None
        self._meta = {}
        self._t0 = 0.0
        self.sweeps = 0

    @property
    def active(self):
        return self.dir is not None

    def start(self, note, meta):
        """Begin a recording. meta: settings etc. for meta.json. Returns its folder."""
        if self.active:
            self.stop()
        slug = re.sub(r"[^A-Za-z0-9]+", "-", note).strip("-")[:40]
        name = time.strftime("rec_%Y%m%d_%H%M%S") + (f"_{slug}" if slug else "")
        self.dir = os.path.join(self.base_dir, name)
        os.makedirs(self.dir, exist_ok=True)
        self._t0 = time.monotonic()
        self.sweeps = 0
        self._meta = dict(meta, name=name, note=note, started=time.strftime("%Y-%m-%d %H:%M:%S"),
                          t0_monotonic=self._t0, code=_code_version())
        self._write_meta()
        with self._lock:
            self._nav = open(os.path.join(self.dir, "nav.jsonl"), "w")
            self._marks = open(os.path.join(self.dir, "marks.jsonl"), "w")
        self.mark(f"recording started: {note}", auto=True)
        return self.dir

    def stop(self, **more):
        if not self.active:
            return None
        self.mark("recording stopped", auto=True)
        with self._lock:
            for f in (self._nav, self._marks):
                f.close()
            self._nav = self._marks = None
        self._meta.update(more, ended=time.strftime("%Y-%m-%d %H:%M:%S"),
                          seconds=round(time.monotonic() - self._t0, 1), sweeps=self.sweeps)
        self._write_meta()
        done, self.dir = self.dir, None
        return done

    def nav(self, t, packet):
        """One VN-100/DVL reading (called from pose.UdpPose's thread)."""
        with self._lock:
            if self._nav is not None:
                self._nav.write(json.dumps(dict(packet, t=round(t, 4))) + "\n")

    def mark(self, note, auto=False):
        with self._lock:
            if self._marks is not None:
                self._marks.write(json.dumps({"t": round(time.monotonic(), 4),
                                              "wall": time.strftime("%H:%M:%S"),
                                              "note": note, "auto": auto}) + "\n")
                self._marks.flush()

    def sweep_path(self, number):
        """Where to save sweep `number` too, or None when not recording."""
        if not self.active:
            return None
        self.sweeps += 1
        return os.path.join(self.dir, f"sweep_{number:05d}.npz")

    def status(self):
        if not self.active:
            return None
        return {"name": os.path.basename(self.dir), "sweeps": self.sweeps,
                "seconds": int(time.monotonic() - self._t0)}

    def _write_meta(self):
        with open(os.path.join(self.dir, "meta.json"), "w") as f:
            json.dump(self._meta, f, indent=1, default=str)
