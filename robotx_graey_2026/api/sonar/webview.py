#!/usr/bin/env python3
"""Serve the radar view as MJPEG, so it can be watched from a laptop browser.

The Jetson has no screen, so cv2.imshow has nothing to draw on. This is the
same trick vision/camera.py already uses for the OAK-D view, and it is copied
rather than imported because that module pulls in depthai at import time - the
sonar package deliberately depends on nothing heavier than numpy and opencv,
which is what lets it run on a laptop with no sub attached.

Default port is 8081, NOT 8080, so this and the camera view can be open in two
browser tabs at once.

    from . import webview
    webview.serve(8081)
    ...
    webview.publish(render(perception, state="SEARCH"))

THE TARGET BOX. The page carries one setting: what to score blobs against.
Leave it blank and nothing is judged and no rings are drawn. Type a name from
the object library, or ideal values like "height_m=1.5,brightness=140", and the
rings come back shaded by how well each blob matched.

This server only carries the string. It does not know what a target is, cannot
resolve one, and never scores anything - it hands the text to whichever tool is
publishing frames, and that tool decides what to do with it and writes a line
back with set_note(). Trying different ideals against a real object is then
typing in a browser rather than killing an SSH session and restarting.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

_frame = None
_target = ""
_note = ""
_editable = True
_lock = threading.Lock()


def target():
    """Whatever was last typed into the target box."""
    with _lock:
        return _target


def set_target_editable(flag):
    """Whether the browser may change the target.

    A tool that never reads target() must call this with False, or the page
    shows a box you can type into that silently does nothing - which is worse
    than having no box at all.

    It is also the safe default for anything that MOVES THE SUB. Changing what
    the vehicle is hunting, from a browser, midway through an autonomous run, is
    not a knob worth having: the state machine would keep its old heading and
    its old memory while the detector started scoring something else.
    """
    global _editable
    with _lock:
        _editable = bool(flag)


def set_target(text):
    """Seed the box, so a --target given on the command line shows up in it."""
    global _target
    with _lock:
        _target = text or ""


def set_note(text):
    """One line echoed back beside the box: what the tool made of the string."""
    global _note
    with _lock:
        _note = text or ""


# Fills the window, keeping the aspect ratio. The frame is rendered large to
# begin with, so on a laptop this is at or near native size rather than a blurry
# stretch of a small image.
PAGE = b"""<html><head><meta charset="utf-8"><title>Graey sonar</title></head>
<body style="margin:0;background:#161412;height:100vh;display:flex;flex-direction:column">
<div style="display:flex;gap:10px;align-items:center;padding:7px 12px;background:#221f1c;
font:13px/1.4 ui-monospace,Menlo,monospace;color:#ddd">
<label for="t">target</label>
<input id="t" spellcheck="false" placeholder="blank = no rings. or a name, or height_m=1.5,brightness=140"
style="flex:1;background:#0e0c0b;color:#eee;border:1px solid #444;border-radius:3px;
padding:5px 7px;font:13px ui-monospace,Menlo,monospace">
<button id="go" style="background:#3a3632;color:#eee;border:1px solid #555;border-radius:3px;
padding:5px 12px;font:13px ui-monospace,Menlo,monospace;cursor:pointer">apply</button>
<span id="note" style="color:#8ea;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;
max-width:45vw"></span></div>
<img src="/stream" style="flex:1;min-height:0;width:100%;object-fit:contain">
<script>
var t=document.getElementById('t'),n=document.getElementById('note'),
g=document.getElementById('go');
function send(){if(!t.disabled)fetch('/target',{method:'POST',body:t.value});}
g.onclick=send;
t.addEventListener('keydown',function(e){if(e.key==='Enter')send();});
(function poll(){fetch('/state').then(function(r){return r.json();}).then(function(s){
n.textContent=s.note;if(document.activeElement!==t)t.value=s.target;
var lock=(s.editable===false);
if(t.disabled!==lock){t.disabled=g.disabled=lock;
t.style.opacity=g.style.opacity=lock?'0.45':'1';
t.placeholder=lock?'fixed for this run':
'blank = no rings. or a name, or height_m=1.5,brightness=140';}})
.catch(function(){}).then(function(){setTimeout(poll,1000);});})();
</script></body></html>"""

# OpenCV's default is already 95. 98 trims the last of the ringing around thin
# text and one-pixel rings, and on a tether the extra bytes cost nothing. Most of
# the sharpness gain came from rendering larger with heavier fonts, not this.
_QUALITY = [cv2.IMWRITE_JPEG_QUALITY, 98]


def publish(frame):
    """Encode a BGR image and hand it to the server."""
    global _frame
    ok, buf = cv2.imencode('.jpg', frame, _QUALITY)
    if ok:
        with _lock:
            _frame = buf.tobytes()


class _Handler(BaseHTTPRequestHandler):
    def _send(self, body, kind):
        self.send_response(200)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        """The target box. The string is stored and nothing else happens here.

        Capped at a kilobyte: this listens on 0.0.0.0 so anything on the tether
        network can reach it, and a setting is a short line of text.
        """
        global _target
        if self.path != '/target':
            self.send_error(404)
            return
        # Refused here, not just greyed out in the page. The lock is on the
        # server because anything on the tether network can reach this port, and
        # a disabled input is only a suggestion.
        with _lock:
            locked = not _editable
        if locked:
            self._send(b'target is fixed for this run', 'text/plain')
            return
        length = min(int(self.headers.get('Content-Length') or 0), 1024)
        text = self.rfile.read(length).decode('utf-8', 'replace').strip()
        with _lock:
            _target = text
        self._send(b'ok', 'text/plain')

    def do_GET(self):
        if self.path == '/':
            self._send(PAGE, 'text/html')
            return
        if self.path == '/state':
            with _lock:
                state = {'target': _target, 'note': _note, 'editable': _editable}
            self._send(json.dumps(state).encode(), 'application/json')
            return
        if self.path != '/stream':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()
        try:
            while True:
                with _lock:
                    data = _frame
                if data:
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
                # A sweep takes seconds, not milliseconds, so there is no point
                # spinning fast here - this only controls how quickly a newly
                # opened browser tab picks up the frame already rendered.
                time.sleep(0.2)
        except (ConnectionResetError, BrokenPipeError):
            pass

    def log_message(self, *a):
        pass                                        # silence per-request logging


def serve(port=8081):
    threading.Thread(target=lambda: ThreadingHTTPServer(
        ('0.0.0.0', port), _Handler).serve_forever(), daemon=True).start()
