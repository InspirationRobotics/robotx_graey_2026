"""Read-only Linux host measurements, with unknown values on unsupported hosts."""
import os
import shutil
import threading
from pathlib import Path


class SystemInfo:
    def __init__(self,proc='/proc',thermal='/sys/class/thermal',disk='/'):
        self.proc,self.thermal,self.disk=Path(proc),Path(thermal),disk
        self.previous=None
        self.lock=threading.Lock()

    def snapshot(self):
        result=dict(cpu_percent=None,memory_used_bytes=None,memory_total_bytes=None,
                    temperature_c=None,uptime_s=None,disk_free_bytes=None,disk_total_bytes=None)
        with self.lock:
            try:
                values=[int(v) for v in (self.proc/'stat').read_text().splitlines()[0].split()[1:9]]
                current=(sum(values),values[3]+values[4])
                if self.previous and current[0]>self.previous[0]:
                    result['cpu_percent']=max(0.,min(100.,100*(1-(current[1]-self.previous[1])/(current[0]-self.previous[0]))))
                self.previous=current
            except (OSError,ValueError,IndexError):pass
        try:
            memory={line.split(':')[0]:int(line.split()[1])*1024 for line in (self.proc/'meminfo').read_text().splitlines()}
            result.update(memory_total_bytes=memory['MemTotal'],memory_used_bytes=memory['MemTotal']-memory['MemAvailable'])
        except (OSError,ValueError,KeyError,IndexError):pass
        try:result['uptime_s']=float((self.proc/'uptime').read_text().split()[0])
        except (OSError,ValueError,IndexError):pass
        temperatures=[]
        for path in self.thermal.glob('thermal_zone*/temp'):
            try:
                value=float(path.read_text())/1000
                if -20<=value<=150:temperatures.append(value)
            except (OSError,ValueError):pass
        if temperatures:result['temperature_c']=max(temperatures)
        try:
            usage=shutil.disk_usage(self.disk)
            result.update(disk_free_bytes=usage.free,disk_total_bytes=usage.total)
        except OSError:pass
        return result
