#!/usr/bin/env python3
"""Run installed MAVProxy with one checked ingress hook, without editing it."""
import pathlib
import shutil
from mission_input_gate import MissionInputGate

def hook_source(source):
    needle='            output.write(m.get_msgbuf())'
    if source.count(needle)!=1:
        raise SystemExit('Unsupported MAVProxy ingress: refusing unfiltered startup')
    source=source.replace(needle,'            if not _graey_input_gate.allow(m):\n                continue\n            if _graey_input_gate.takeover_requested:\n                output.mav.set_mode_send(1, 1, 19)\n                _graey_input_gate.takeover_requested=False\n'+needle)
    return source


def main():
    path=shutil.which('mavproxy.py')
    if not path:raise SystemExit('mavproxy.py not found')
    source=hook_source(pathlib.Path(path).read_text())
    namespace=globals()
    namespace.update(__name__='__main__',__file__=path,_graey_input_gate=MissionInputGate())
    exec(compile(source,path,'exec'),namespace)


if __name__=='__main__':main()
