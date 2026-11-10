"""Bounded incremental file-log reads; handle rotation and truncation explicitly."""
from pathlib import Path


def read_log(path,after=0,generation='',errors=False,limit=65536):
    if type(after) is not int or after<0:raise ValueError('invalid log cursor')
    path=Path(path)
    try:
        with path.open('rb') as stream:
            import os
            stat=os.fstat(stream.fileno())
            token=str(stat.st_dev)+':'+str(stat.st_ino)
            reset=generation!=token or after>stat.st_size
            start=max(0,stat.st_size-limit) if reset else after
            stream.seek(start)
            raw=stream.read(limit)
            end=raw.rfind(b'\n')
            if end>=0:raw=raw[:end+1]
            cursor=start+len(raw)
            if reset and start>0 and b'\n' in raw:raw=raw.split(b'\n',1)[1]
        lines=raw.decode('utf-8',errors='replace').splitlines()
        if errors:lines=[line for line in lines if any(word in line.upper() for word in ('WARN','ERROR','FATAL','TRACEBACK'))]
        return dict(generation=token,cursor=cursor,reset=reset,lines=lines,available=True)
    except FileNotFoundError:
        return dict(generation='',cursor=0,reset=True,lines=[],available=False)
