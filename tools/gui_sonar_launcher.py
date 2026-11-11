#!/usr/bin/env python3
"""Start and stop the sonar tools from the GUI's Sonar tab -> http://<jetson>:8096

The GUI runs inside the ROS container, but the sonar tools must run here on the
Jetson itself (the Ping360 library is not in the container). This small server
is the go-between. It can start ONE sonar tool from the fixed list below, or
stop it - nothing else. It runs from boot as graey-sonar-launcher.service and
sits idle until a button is pressed.

    GET  /status              what is running, and the end of its output
    POST /start?tool=map      tools/sonar_map.py
    POST /start?tool=radar    tools/sonar_pole_test.py --web
    POST /stop                like Ctrl-C

Both tools serve their page on port 8095, which the Sonar tab shows. A sonar
tool started by hand in a terminal is left alone, and nothing new is started
while it runs - only one program can talk to the Ping360.

The cameras too (Ruth, Oct 24), for the Camera tab's buttons and tools/task2.sh:

    POST /camera/start        tools/cam_pair.py in the graey container -> port 8080
    POST /camera/stop         like Ctrl-C, so the OAKs close cleanly
    POST /record/start        the camera stream to ~/camera_rec/<name>_<date>_<time>.mjpg
                              (?name=..., default cam)
    POST /record/stop
"""
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 8096
WEB_PORT = 8095                 # where the tools serve their page
PYTHON = "/usr/bin/python3"
SONAR = "/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D2010VWF-if00-port0"
TOOLS = {
    "map":   ["tools/sonar_map.py", "--device", SONAR, "--port", str(WEB_PORT), "--no-save"],
    "radar": ["tools/sonar_pole_test.py", "--device", SONAR, "--web",
              "--port", str(WEB_PORT), "--range", "4"],
}
LOG = os.path.join(REPO, "sonar_data", "sonar_tool.log")
# the same pattern as the safety check run before starting a tool by hand
RUNNING = re.compile(r"^(/usr/bin/)?python3 (-u )?tools/sonar_")

CAM_PORT = 8080                 # tools/cam_pair.py's stream (DOWN | FRONT)
CAM_LOG = os.path.join(REPO, "sonar_data", "camera_tool.log")
CAM_RUNNING = re.compile(r"tools/cam_pair\.py")
CAM_CMD = ["docker", "exec", "graey", "bash", "-c",
           "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && "
           "cd /root/robotx_ws/src/robotx_graey_2026 && PYTHONPATH=.:$PYTHONPATH exec python3 -u tools/cam_pair.py"]
REC_DIR = os.path.expanduser("~/camera_rec")

_lock = threading.Lock()
_job = {"tool": None, "proc": None, "since": None, "asked": False}
_cam = {"proc": None, "since": None}
_rec = {"proc": None, "path": None, "since": None}


def _others():
    """Sonar tools running that this launcher did not start."""
    mine = _job["proc"].pid if _job["proc"] and _job["proc"].poll() is None else None
    out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True).stdout
    found = []
    for line in out.splitlines():
        pid, _, cmd = line.strip().partition(" ")
        if pid.isdigit() and int(pid) != mine and RUNNING.match(cmd.strip()):
            found.append(cmd.strip())
    return found


def _web_up(port=WEB_PORT):
    try:
        socket.create_connection(("127.0.0.1", port), timeout=0.3).close()
        return True
    except OSError:
        return False


def _tail(n=8, path=LOG):
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 4000))
            return f.read().decode(errors="replace").splitlines()[-n:]
    except OSError:
        return []


def status():
    proc = _job["proc"]
    running = proc is not None and proc.poll() is None
    return {"running": _job["tool"] if running else None,
            "last": _job["tool"],
            "exit": None if proc is None or running else proc.returncode,
            "since": _job["since"],
            "asked": _job["asked"],     # stopped with the Stop button, not by itself
            "webUp": _web_up(),         # also true for a tool started by hand
            "others": _others(),
            "log": _tail(),
            **cam_status()}


def start(tool):
    if tool not in TOOLS:
        return "unknown tool"
    if _job["proc"] is not None and _job["proc"].poll() is None:
        return f"{_job['tool']} is already running - stop it first"
    if _others():
        return "a sonar tool started in a terminal is running - stop it there first"
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    log = open(LOG, "ab")
    log.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} start {tool}\n".encode())
    log.flush()
    env = dict(os.environ, PYTHONPATH=REPO, PYTHONUNBUFFERED="1")
    _job["proc"] = subprocess.Popen([PYTHON] + TOOLS[tool], cwd=REPO, env=env,
                                    stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
    log.close()                 # the tool has its own copy
    _job["tool"], _job["since"], _job["asked"] = tool, time.time(), False
    return "ok"


def stop():
    proc = _job["proc"]
    if proc is None or proc.poll() is not None:
        return "nothing running"
    _job["asked"] = True
    # Ctrl-C first, so the map tool closes the relay and the sonar cleanly
    for sig, wait in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(wait)
            break
        except subprocess.TimeoutExpired:
            continue
    return "ok"


# ---------------- cameras (Ruth, Oct 24) ----------------
def _cam_running():
    """tools/cam_pair.py running in the container (started here or by hand): the host sees it in ps."""
    out = subprocess.run(["ps", "-eo", "args="], capture_output=True, text=True).stdout
    return any(CAM_RUNNING.search(line) and "pkill" not in line for line in out.splitlines())


def cam_status():
    rec = _rec["proc"]
    recording = rec is not None and rec.poll() is None
    size = None
    if _rec["path"]:
        try:
            size = os.path.getsize(_rec["path"])
        except OSError:
            pass
    return {"recording_session_v1": True, "camRunning": _cam_running(),
            "camUp": _web_up(CAM_PORT),
            "camSince": _cam["since"],
            "camLog": _tail(4, CAM_LOG),
            "recording": recording,
            "recFile": os.path.basename(_rec["path"]) if _rec["path"] else None,
            "recBytes": size,
            "recSince": _rec["since"]}


def cam_start():
    if _cam_running() or _web_up(CAM_PORT):
        return "the cameras are already running"
    os.makedirs(os.path.dirname(CAM_LOG), exist_ok=True)
    log = open(CAM_LOG, "ab")
    log.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} start cameras\n".encode())
    log.flush()
    _cam["proc"] = subprocess.Popen(CAM_CMD, stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
    log.close()
    _cam["since"] = time.time()
    return "ok"


def cam_stop():
    if not _cam_running():
        return "the cameras are not running"
    rec_stop()                  # nothing left to record
    # Ctrl-C inside the container (it runs as root there), so cam_pair closes the OAKs: a
    # hard stop has left a camera crash dump twice (cam_pair.py)
    for sig, wait in (("INT", 8), ("TERM", 3), ("KILL", 2)):
        subprocess.run(["docker", "exec", "graey", "pkill", f"-{sig}", "-f", "tools/cam_pair.py"],
                       capture_output=True)
        end = time.time() + wait
        while time.time() < end and _cam_running():
            time.sleep(0.2)
        if not _cam_running():
            break
    return "ok" if not _cam_running() else "the cameras did not stop"


def rec_start(name,session=None):
    if _rec["proc"] is not None and _rec["proc"].poll() is None:
        return "already recording " + os.path.basename(_rec["path"])
    if not _web_up(CAM_PORT):
        return "no camera stream on port 8080 - start the cameras first"
    name = re.sub(r"[^A-Za-z0-9_-]", "_", name or "cam")[:40] or "cam"
    if session is not None:
        sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from robotx_graey_2026.api.recording_paths import session_record_path
        path=str(session_record_path(os.environ.get('GRAEY_RECORD_DIR','~/robotx_ws/recordings'),session,'camera',name))
        os.makedirs(os.path.dirname(path),exist_ok=True)
    else:
        os.makedirs(REC_DIR,exist_ok=True)
        path=os.path.join(REC_DIR,f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.mjpg")
    _rec["proc"] = subprocess.Popen(["curl", "-s", "-N", f"http://127.0.0.1:{CAM_PORT}/stream", "-o", path],
                                    stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL, start_new_session=True)
    _rec["path"], _rec["since"] = path, time.time()
    return "ok"


def rec_stop():
    proc = _rec["proc"]
    if proc is None or proc.poll() is not None:
        return "not recording"
    proc.terminate()
    try:
        proc.wait(3)
    except subprocess.TimeoutExpired:
        proc.kill()
    return "ok"


class Handler(BaseHTTPRequestHandler):
    def _send(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        # the GUI page comes from port 8090, so the browser needs this to read replies
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if urlparse(self.path).path != "/status":
            self.send_error(404)
            return
        with _lock:
            self._send(status())

    def do_POST(self):
        u = urlparse(self.path)
        with _lock:
            if u.path == "/start":
                msg = start(parse_qs(u.query).get("tool", [""])[0])
            elif u.path == "/stop":
                msg = stop()
            elif u.path == "/camera/start":
                msg = cam_start()
            elif u.path == "/camera/stop":
                msg = cam_stop()
            elif u.path == "/record/start":
                args=parse_qs(u.query)
                try:msg=rec_start(args.get("name",["cam"])[0],args.get("session",[None])[0])
                except ValueError as error:msg=str(error)
            elif u.path == "/record/stop":
                msg = rec_stop()
            else:
                self.send_error(404)
                return
            self._send(dict(status(), msg=msg))

    def log_message(self, *a):
        pass


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else PORT
    # A program started in the background inherits "ignore Ctrl-C", and so would
    # every tool started from here - then Stop's Ctrl-C would do nothing.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    print(f"sonar launcher on :{port}, tools from {REPO}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", port), Handler).serve_forever()


if __name__ == "__main__":
    main()
