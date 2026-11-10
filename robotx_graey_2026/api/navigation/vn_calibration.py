"""Bounded in-process VN calibration; the existing reader keeps serial ownership."""
import json
import math
from pathlib import Path
import time
from .vn_protocol import transact, relative_heading


def gravity_face(ypr):
    _, pitch, roll = map(math.radians, ypr)
    down = (-math.sin(pitch), math.sin(roll)*math.cos(pitch), math.cos(roll)*math.cos(pitch))
    axis = max(range(3), key=lambda i: abs(down[i]))
    return axis*2+int(down[axis] < 0) if abs(down[axis]) > .85 else None


class VNCalibration:
    def __init__(self, seconds=120., folder=None, clock=time.monotonic):
        self.clock, self.seconds = clock, seconds
        self.folder = Path(folder or Path.home()/'robotx_ws/logs')
        self.phase, self.reason = 'idle', 'Keep VN mounted. Calibration requires rotation through all six faces.'
        self.faces, self.started, self.sample_at = [0]*6, None, None
        self.path = None
        self.saved = False

    @property
    def running(self):
        return self.phase == 'collecting'

    def record(self, event, data):
        with self.path.open('a', encoding='utf-8') as output:
            output.write(json.dumps(dict(monotonic=self.clock(),event=event,data=data),allow_nan=False)+'\n')

    def start(self, port):
        if self.running:
            raise ValueError('Calibration already running')
        self.folder.mkdir(parents=True, exist_ok=True)
        self.path = self.folder/f'vn-gui-calibration-{time.time_ns()}.jsonl'
        with self.path.open('x'):
            pass
        self.record('before', {str(r):transact(port,f'VNRRG,{r:02d}') for r in (1,4,23,26,35,44,47)})
        relative_heading(port)
        self.faces, self.sample_at = [0]*6, None
        self.saved = False
        self.started, self.phase = self.clock(), 'collecting'
        try:
            transact(port,'VNWRG,44,2,3,1')
            transact(port,'VNWRG,44,1,3,1')
            self.reason = 'Rotate slowly through all six faces for 120 seconds; keep kill engaged'
        except BaseException:
            self.stop(port, 'Calibration start failed', failed=True)
            raise

    def sample(self, ypr):
        if not self.running or (self.sample_at is not None and self.clock()-self.sample_at < 1.):
            return
        self.sample_at = self.clock()
        face = gravity_face(ypr)
        if face is not None:
            self.faces[face] += 1
        self.record('coverage', dict(ypr=ypr,faces=self.faces[:]))

    def stop(self, port, reason=None, failed=False):
        if not self.running:
            return
        # Mark failed before the transaction: failed ACK must never look complete.
        self.phase, self.reason = 'failed', 'Calibration freeze not confirmed; inspect VN before use'
        transact(port,'VNWRG,44,0,3,1')
        after = {str(r):transact(port,f'VNRRG,{r:02d}') for r in (35,44,47)}
        if after['44'] != ['0','3','1'] or len(after['35']) != 4 or after['35'][1] != '1':
            raise OSError('VN relative-mode or calibration-freeze readback failed')
        covered = min(self.faces) >= 5
        if covered and not failed:
            self.reason = 'Calibration save not confirmed; inspect the reported error before use'
            # Register 47 is the calculated result; persist it in user register 23.
            values = after['47']
            if len(values) != 12 or not all(math.isfinite(float(v)) for v in values):
                raise OSError('Invalid calculated VN calibration; not saved')
            transact(port,'VNWRG,23,'+','.join(values))
            readback = transact(port,'VNRRG,23')
            if len(readback)!=12 or any(not math.isclose(float(a),float(b),rel_tol=1e-5,abs_tol=1e-7) for a,b in zip(values,readback)):
                raise OSError('VN calibration compensation readback failed; not saved')
            transact(port,'VNWRG,44,0,1,1')
            if transact(port,'VNRRG,44') != ['0','1','1']:
                raise OSError('VN saved-compensation selection readback failed; not saved')
            transact(port,'VNWNV',timeout=3.)
            self.saved = True
        self.phase = 'failed' if failed or not covered else 'collected'
        self.reason = reason or ('Calibration automatically saved to VN nonvolatile memory; north reference remains separate'
            if self.saved else 'Insufficient six-face coverage; repeat calibration')
        self.record('after', dict(registers=after, faces=self.faces, phase=self.phase,
                                  accepted=self.saved,saved=self.saved,reason=self.reason))

    def status(self):
        return dict(phase=self.phase, reason=self.reason, faces=self.faces[:],
                    remaining_s=max(0.,self.seconds-(self.clock()-self.started)) if self.running else 0.,
                    log=str(self.path) if self.path else None, saved=self.saved,
                    heading_mode='relative', accepted=self.saved)
