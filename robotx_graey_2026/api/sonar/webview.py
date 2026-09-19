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

THE DETECTION BUTTONS. The picture is a JPEG, so nothing drawn in it can be
clicked. Underneath it the page builds one real HTML button per detection from
set_rows(), pages through them when there are more than fit, and posts back
which ones you picked. The tool passes those to render(), which boxes a long
one and circles a compact one - so "which blob is my pipe" is a click instead
of a screenshot with a circle drawn on it in red.

Selecting is never locked, even on a tool that moves the sub: looking at a
detection changes nothing about what the vehicle does.
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
_rows = []                          # one summary per detection, for the buttons
_per_page = 10
_view = {"page": 0, "selected": []}
_lock = threading.Lock()


def set_rows(rows, per_page=10):
    """Publish a summary of this sweep's detections, for the buttons.

    The picture is a JPEG, so nothing in it can be clicked. The page draws one
    real HTML button per detection from this list, and the tool reads back which
    ones you picked. Keep it small - it goes over the tether once a second.
    """
    global _rows, _per_page
    with _lock:
        _rows = list(rows)
        _per_page = max(1, int(per_page))
        # A sweep with fewer blobs than the last one must not leave you stranded
        # on a page that no longer exists.
        last = max(0, (len(_rows) - 1) // _per_page)
        if _view["page"] > last:
            _view["page"] = last


def view():
    """(page, selected indices) the browser is currently asking for."""
    with _lock:
        return _view["page"], set(_view["selected"])


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
<div id="bar" style="display:flex;gap:6px;align-items:center;flex-wrap:wrap;
padding:7px 12px;background:#221f1c;font:13px ui-monospace,Menlo,monospace;color:#ddd">
<button id="prev" style="background:#3a3632;color:#eee;border:1px solid #555;border-radius:3px;
padding:4px 10px;font:13px ui-monospace,Menlo,monospace;cursor:pointer">&#9664;</button>
<span id="pg" style="min-width:86px;text-align:center;color:#999"></span>
<button id="next" style="background:#3a3632;color:#eee;border:1px solid #555;border-radius:3px;
padding:4px 10px;font:13px ui-monospace,Menlo,monospace;cursor:pointer">&#9654;</button>
<span style="color:#666">|</span><span id="blobs" style="display:flex;gap:5px;flex-wrap:wrap"></span>
<button id="none" style="background:#2a2724;color:#999;border:1px solid #444;border-radius:3px;
padding:4px 10px;font:13px ui-monospace,Menlo,monospace;cursor:pointer">clear</button></div>
<script>
var t=document.getElementById('t'),n=document.getElementById('note'),
g=document.getElementById('go');
function send(){if(!t.disabled)fetch('/target',{method:'POST',body:t.value});}
g.onclick=send;
t.addEventListener('keydown',function(e){if(e.key==='Enter')send();});
var pg=document.getElementById('pg'),blobs=document.getElementById('blobs'),
page=0,sel=[],rows=[],per=10,drawn='';
function push(){fetch('/view',{method:'POST',
body:JSON.stringify({page:page,selected:sel})});}
function turn(d){var last=Math.max(0,Math.ceil(rows.length/per)-1);
page=Math.max(0,Math.min(page+d,last));push();paint();}
document.getElementById('prev').onclick=function(){turn(-1);};
document.getElementById('next').onclick=function(){turn(1);};
document.getElementById('none').onclick=function(){sel=[];push();paint();};
function toggle(i){var k=sel.indexOf(i);if(k<0)sel.push(i);else sel.splice(k,1);
push();paint();}
function paint(){
var pages=Math.max(1,Math.ceil(rows.length/per));
if(page>pages-1)page=pages-1;
pg.textContent=rows.length?('page '+(page+1)+'/'+pages):'no blobs';
var slice=rows.slice(page*per,(page+1)*per);
var key=page+'|'+rows.length+'|'+sel.join(',')+'|'+slice.map(function(r){
return r.i+':'+r.label;}).join(',');
if(key===drawn)return; drawn=key;
blobs.innerHTML='';
slice.forEach(function(r){var b=document.createElement('button');
var on=sel.indexOf(r.i)>=0;
b.textContent=r.i+' - '+r.label;
b.style.cssText='border-radius:3px;padding:4px 9px;cursor:pointer;'+
'font:13px ui-monospace,Menlo,monospace;border:1px solid '+
(on?'#eb3cf0':'#555')+';background:'+(on?'#eb3cf0':'#3a3632')+';color:'+
(on?'#120b13':'#eee');
b.onclick=function(){toggle(r.i);};blobs.appendChild(b);});}
(function poll(){fetch('/state').then(function(r){return r.json();}).then(function(s){
n.textContent=s.note;if(document.activeElement!==t)t.value=s.target;
var lock=(s.editable===false);
if(t.disabled!==lock){t.disabled=g.disabled=lock;
t.style.opacity=g.style.opacity=lock?'0.45':'1';
t.placeholder=lock?'fixed for this run':
'blank = no rings. or a name, or height_m=1.5,brightness=140';}
rows=s.rows||[];per=s.perPage||10;paint();})
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

    def _body(self, cap=1024):
        length = min(int(self.headers.get('Content-Length') or 0), cap)
        return self.rfile.read(length)

    def do_POST(self):
        """Two settings come back from the page: the target, and the view.

        Bodies are capped: this listens on 0.0.0.0 so anything on the tether
        network can reach it, and both of these are short.
        """
        global _target
        if self.path == '/view':
            # Which page to show and which detections to highlight. Never
            # locked, because looking at something changes nothing - you can
            # pick a blob apart mid-mission without touching what the sub does.
            try:
                want = json.loads(self._body(4096).decode('utf-8', 'replace'))
            except ValueError:
                self.send_error(400)
                return
            with _lock:
                last = max(0, (len(_rows) - 1) // _per_page)
                _view["page"] = max(0, min(int(want.get("page", 0)), last))
                _view["selected"] = sorted(
                    {int(i) for i in want.get("selected", [])
                     if 0 <= int(i) < len(_rows)})[:32]
            self._send(b'ok', 'text/plain')
            return
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
        text = self._body().decode('utf-8', 'replace').strip()
        with _lock:
            _target = text
        self._send(b'ok', 'text/plain')

    def do_GET(self):
        if self.path == '/':
            self._send(PAGE, 'text/html')
            return
        if self.path == '/state':
            with _lock:
                state = {'target': _target, 'note': _note, 'editable': _editable,
                         'rows': _rows, 'perPage': _per_page,
                         'page': _view["page"], 'selected': _view["selected"]}
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
