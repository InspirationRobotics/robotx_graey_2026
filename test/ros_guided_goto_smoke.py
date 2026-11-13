"""Real ROS + real telemetry_gateway + a UDP stand-in Cube: tools/guided_goto.py's target reaches the Cube.

    source /opt/ros/humble/setup.bash && PYTHONPATH=.:$PYTHONPATH python3 test/ros_guided_goto_smoke.py

The stand-in Cube is armed in GUIDED at local (2, -1, 0.5) facing east; a control-watchdog stand-in
permits motion. guided_goto 1.5 must send SET_POSITION_TARGET_LOCAL_NED (2.0, 0.5, 0.5) as component 198.
No serial device, vehicle address or production ROS domain is used.
"""
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.update(ROS_DOMAIN_ID='122', ROS_LOCALHOST_ONLY='1', GRAEY_RUNTIME_SCOPE='guided-goto-smoke')
from robotx_graey_2026.api.pixhawk.mavlink import mavutil  # noqa: E402


def main():
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 0))
    udp.settimeout(.02)
    port = udp.getsockname()[1]
    done, targets = threading.Event(), []
    parser = mavutil.mavlink.MAVLink(None)
    cube = mavutil.mavlink.MAVLink(None, srcSystem=3, srcComponent=1)

    def fcu():
        peer, last = None, 0.
        while not done.is_set():
            try:
                raw, peer = udp.recvfrom(4096)
                for m in parser.parse_buffer(raw) or []:
                    if m.get_msgId() == 84:
                        targets.append((m.get_srcComponent(), m.x, m.y, m.z, m.yaw))
            except socket.timeout:
                pass
            if peer and time.monotonic()-last > .05:
                last = time.monotonic()
                for msg in (cube.heartbeat_encode(12, 3, 129, 4, 4, 3),      # armed (128) GUIDED (4)
                            cube.local_position_ned_encode(0, 2., -1., .5, 0, 0, 0),
                            cube.attitude_encode(0, 0, 0, math.radians(90), 0, 0, 0)):
                    udp.sendto(bytes(msg.pack(cube)), peer)
    worker = threading.Thread(target=fcu, daemon=True)
    worker.start()
    env = dict(os.environ, PYTHONPATH=f'{ROOT}:{os.environ.get("PYTHONPATH", "")}')
    gateway = subprocess.Popen([sys.executable, '-c', 'from robotx_graey_2026.api.pixhawk import telemetry_gateway as g; g.main()',
                                '--ros-args', '--params-file', str(ROOT/'config/graey_params.yaml'),
                                '-p', f'mavlink:=udpout:127.0.0.1:{port}'], cwd=ROOT, env=env)
    watchdog = subprocess.Popen([sys.executable, '-c', (
        'import json,time,rclpy\nfrom std_msgs.msg import String\nrclpy.init();n=rclpy.create_node("watchdog_standin")\n'
        'p=n.create_publisher(String,"/graey/control_watchdog/status",10)\n'
        'try:\n while True:\n  p.publish(String(data=json.dumps(dict(vehicle_fresh=True,sa_active=False,esc_commanded_on=True,'
        'inhibit=False,armed=True,published_monotonic=time.monotonic()))));time.sleep(.05)\nexcept KeyboardInterrupt:pass')], env=env)
    goto = None
    try:
        time.sleep(3.)
        goto = subprocess.Popen([sys.executable, 'tools/guided_goto.py', '1.5'], cwd=ROOT, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.monotonic()+15
        while len(targets) < 5 and time.monotonic() < deadline and goto.poll() is None:
            time.sleep(.1)
        goto.send_signal(signal.SIGINT)
        out = goto.communicate(timeout=10)[0]
        ok = (len(targets) >= 5 and all(t[0] == 198 for t in targets)
              and all(abs(a-b) < 1e-4 for a, b in zip(targets[-1][1:4], (2., .5, .5))))
        print(json.dumps(dict(result='PASS' if ok else 'FAIL', targets_received=len(targets),
                              last_target=targets[-1] if targets else None, tool_output=out.strip()), indent=2))
        return 0 if ok else 1
    finally:
        for proc in (goto, gateway, watchdog):
            if proc and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        done.set()
        worker.join(2)
        udp.close()


if __name__ == '__main__':
    raise SystemExit(main())
