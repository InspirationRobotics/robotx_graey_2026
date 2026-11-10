#!/usr/bin/env python3
"""Run installed MAVProxy with one checked ingress hook, without editing it."""
import pathlib
import shutil
import socket
import types
from mission_input_gate import MissionInputGate

def install_continuous_broadcast():
    """Keep subnet outputs broadcasting even after a QGC client replies."""
    from pymavlink import mavutil
    original = mavutil.mavlink_connection

    def connection(device, *args, **kwargs):
        if not device.startswith('udpall:'):
            return original(device, *args, **kwargs)
        link = original('udpbcast:' + device[len('udpall:'):], *args, **kwargs)
        destination = link.destination_addr

        def write_broadcast(self, buf):
            try:
                self.port.sendto(buf, destination)
            except socket.error:
                pass  # Match pymavlink's handling of temporary network loss.

        link.write = types.MethodType(write_broadcast, link)
        return link

    mavutil.mavlink_connection = connection

def hook_source(source):
    needle='            output.write(m.get_msgbuf())'
    if source.count(needle)!=1:
        raise SystemExit('Unsupported MAVProxy ingress: refusing unfiltered startup')
    source=source.replace(needle,'            if not _graey_input_gate.allow(m):\n                continue\n            if _graey_input_gate.takeover_requested:\n                output.mav.set_mode_send(1, 1, 19)\n                _graey_input_gate.takeover_requested=False\n'+needle)
    return source


def main():
    install_continuous_broadcast()
    path=shutil.which('mavproxy.py')
    if not path:raise SystemExit('mavproxy.py not found')
    source=hook_source(pathlib.Path(path).read_text())
    namespace=globals()
    namespace.update(__name__='__main__',__file__=path,_graey_input_gate=MissionInputGate())
    exec(compile(source,path,'exec'),namespace)


if __name__=='__main__':main()
