"""Thread-safe operator snapshot; cached transport state never implies a live link."""
import copy
import math
import threading
import time


class CommunicationsView:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.sample = None
        self.received = None

    def update(self, data):
        if not isinstance(data, dict):
            raise ValueError('communications status must be an object')
        with self.lock:
            self.sample = copy.deepcopy(data)
            stamp=data.get('published_monotonic')
            self.received=stamp if type(stamp) in (int,float) and math.isfinite(stamp) and stamp<=self.clock() else None

    def snapshot(self):
        with self.lock:
            age = None if self.received is None else max(0,self.clock()-self.received)
            fresh = age is not None and age < 2
            data = copy.deepcopy(self.sample) if self.sample is not None else {}
            return dict(fresh=fresh,age_s=age,data=data,
                        connection=('connected' if data.get('wifi',{}).get('connected') else 'disconnected')
                        if fresh else 'unknown',
                        run_verified=bool(fresh and data.get('run_confirmed')))
