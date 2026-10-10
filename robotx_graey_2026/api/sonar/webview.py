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

THE RANGE BOX. Only shown when the tool calls set_range(), and only editable
when it also calls set_range_editable(True). The tool checks it after every
ping, so a new range restarts the scan straight away. What was drawn before
stays on the picture in grey until the new scan paints over it.
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
_range = None                       # metres; None = the tool doesn't show a range box
_range_editable = False
_map_on = False                     # the tool has a breadcrumb map: show the tabs
_map_json = b'{}'                   # latest map snapshot, already encoded
_map_cmds = []                      # Start/Pause/Resume/Reset presses, oldest first
_rec_cmds = []                      # ("start", note) / ("stop", "") / ("mark", note)
_sector = None                      # (start, end) degrees, swept counterclockwise
_sector_editable = True
_threshold = None                   # crumb threshold, 0-255
_floor_cut = 0.0                     # m above the floor with no crumbs; 0 = off
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


# Below 1 m almost everything is inside the 0.75 m blind zone; 50 m is the
# Ping360's own limit. A tool can narrow it with set_range_limits().
RANGE_MIN_M, RANGE_MAX_M = 1.0, 50.0
_range_lo, _range_hi = RANGE_MIN_M, RANGE_MAX_M


def set_range_limits(lo, hi):
    """What the range box accepts, e.g. 2-20 m for the map."""
    global _range_lo, _range_hi
    with _lock:
        _range_lo, _range_hi = float(lo), float(hi)


def set_range(metres):
    """Show the range box, seeded with the range the tool is using."""
    global _range
    with _lock:
        _range = float(metres)


def range_m():
    """The range last set from the browser (or by set_range)."""
    with _lock:
        return _range


def set_range_editable(flag):
    """Let the browser change the range. Off by default, like the target box."""
    global _range_editable
    with _lock:
        _range_editable = bool(flag)


# ---- the breadcrumb map (Map tab) ------------------------------------------
# Only there when the tool calls enable_map(). The tool draws nothing for it: it
# hands over a snapshot (set_map) and the page draws it, so zooming the map is
# instant and costs the Jetson nothing.

def enable_map():
    global _map_on
    with _lock:
        _map_on = True


def set_map(snapshot):
    """Latest map state for the page. A dict of plain numbers and lists."""
    global _map_json
    data = json.dumps(snapshot, separators=(',', ':')).encode()
    with _lock:
        _map_json = data


def record_commands():
    """Record / Stop / Mark presses since last asked, oldest first."""
    global _rec_cmds
    with _lock:
        cmds, _rec_cmds = _rec_cmds, []
    return cmds


def map_commands():
    """Button presses since last asked, oldest first, then forgotten."""
    global _map_cmds
    with _lock:
        cmds, _map_cmds = _map_cmds, []
    return cmds


def set_sector(start, end, editable=True):
    global _sector, _sector_editable
    with _lock:
        _sector = (float(start) % 360.0, float(end) % 360.0)
        _sector_editable = bool(editable)


def sector():
    with _lock:
        return _sector


def set_threshold(value):
    global _threshold
    with _lock:
        _threshold = int(value)


def threshold():
    with _lock:
        return _threshold


def set_floor_cut(value):
    global _floor_cut
    with _lock:
        _floor_cut = float(value)


def floor_cut():
    with _lock:
        return _floor_cut


# Fills the window, keeping the aspect ratio. The frame is rendered large to
# begin with, so on a laptop this is at or near native size rather than a blurry
# stretch of a small image.
PAGE = b"""<html><head><meta charset="utf-8"><title>Graey sonar</title></head>
<body style="margin:0;background:#161412;height:100vh;display:flex;flex-direction:column">
<div style="display:flex;gap:10px;align-items:center;padding:7px 12px;background:#221f1c;
font:13px/1.4 ui-monospace,Menlo,monospace;color:#ddd">
<span id="tabs" style="display:none;white-space:nowrap"><b style="color:#fff">Radar</b>
<a href="/map" style="color:#8bd">Map</a></span>
<label for="t">target</label>
<input id="t" spellcheck="false" placeholder="blank = no rings. or a name, or height_m=1.5,brightness=140"
style="flex:1;background:#0e0c0b;color:#eee;border:1px solid #444;border-radius:3px;
padding:5px 7px;font:13px ui-monospace,Menlo,monospace">
<button id="go" style="background:#3a3632;color:#eee;border:1px solid #555;border-radius:3px;
padding:5px 12px;font:13px ui-monospace,Menlo,monospace;cursor:pointer">apply</button>
<span id="rgw" style="display:none;white-space:nowrap">range
<input id="rg" type="number" step="0.5" min="1" max="50" style="width:62px;background:#0e0c0b;
color:#eee;border:1px solid #444;border-radius:3px;padding:5px;font:13px ui-monospace,Menlo,monospace"> m
<button id="rgo" style="background:#3a3632;color:#eee;border:1px solid #555;border-radius:3px;
padding:5px 10px;font:13px ui-monospace,Menlo,monospace;cursor:pointer">set</button>
<span id="rgn" style="color:#8ea"></span></span>
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
var rgw=document.getElementById('rgw'),rg=document.getElementById('rg'),
rgo=document.getElementById('rgo'),rgn=document.getElementById('rgn');
function sendRange(){if(rg.disabled)return;fetch('/range',{method:'POST',body:rg.value})
.then(function(r){return r.text();}).then(function(x){rgn.textContent=x;});}
rgo.onclick=sendRange;
rg.addEventListener('keydown',function(e){if(e.key==='Enter')sendRange();});
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
rgw.style.display=(s.range===null||s.range===undefined)?'none':'inline';
if(s.range!==null&&document.activeElement!==rg)rg.value=s.range;
rg.min=s.rangeMin;rg.max=s.rangeMax;
document.getElementById('tabs').style.display=s.map?'inline':'none';
rg.disabled=rgo.disabled=!s.rangeEditable;
rows=s.rows||[];per=s.perPage||10;paint();})
.catch(function(){}).then(function(){setTimeout(poll,1000);});})();
</script></body></html>"""

# The Map tab. Top-down, y = forward along the heading at Start, x = right.
# Range, sector and threshold go to the tool; map area and follow are only how
# this page draws, so they never touch the sonar.
MAP_PAGE = b"""<html><head><meta charset="utf-8"><title>Graey sonar map</title><style>
body{margin:0;background:#161412;height:100vh;display:flex;flex-direction:column;
font:13px/1.4 ui-monospace,Menlo,monospace;color:#ddd}
.bar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;padding:7px 12px;background:#221f1c}
input{background:#0e0c0b;color:#eee;border:1px solid #444;border-radius:3px;padding:5px;
font:inherit;width:58px}
button{background:#3a3632;color:#eee;border:1px solid #555;border-radius:3px;padding:5px 12px;
font:inherit;cursor:pointer}
a{color:#8bd}
#wrap{flex:1;min-height:0;position:relative}
canvas{position:absolute;left:0;top:0;width:100%;height:100%}
#banner{position:absolute;left:50%;top:12px;transform:translateX(-50%);padding:6px 14px;
border-radius:4px;color:#fff;display:none}
</style></head><body>
<div class="bar">
<span style="white-space:nowrap"><a href="/">Radar</a> <b style="color:#fff">Map</b></span>
<button id="go">Start</button><button id="rs">Reset</button>
<span title="what this recording is: where the pipe is, which way, what you're doing"><input id="rn" placeholder="note for the recording" style="width:15em"></span>
<button id="rec">Record</button><button id="mk" title="stamp this moment, with the note">Mark</button>
<span>range <input id="rg" type="number" step="0.5" min="2" max="20"> m</span>
<span>sector <input id="s0" type="number" step="1" min="0" max="359"> to
<input id="s1" type="number" step="1" min="0" max="359"></span>
<span>threshold <input id="th" type="number" step="5" min="1" max="255"></span>
<span title="no crumbs from echoes this close above the seafloor (DVL altitude); 0 = off">floor cut <input id="fc" type="number" step="0.1" min="0" max="3"> m</span>
<button id="set">set</button>
<span>map area <input id="ar" type="number" step="1" min="1" max="200" value="10"> m</span>
<label style="white-space:nowrap"><input id="fo" type="checkbox" checked style="width:auto"> follow sub</label>
<label style="white-space:nowrap"><input id="ol" type="checkbox" checked style="width:auto"> outlines</label>
<span id="msg" style="color:#8ea"></span></div>
<div class="bar" style="padding-top:0"><span id="st" style="color:#bbb"></span></div>
<div id="wrap"><canvas id="cv"></canvas><div id="banner"></div></div>
<div class="bar" style="color:#bbb">
<span>crumb: weak <span style="display:inline-block;width:90px;height:10px;vertical-align:middle;
background:linear-gradient(90deg,hsl(210,95%,58%),hsl(105,95%,58%),hsl(0,95%,58%))"></span> strong</span>
<span><span style="opacity:.3">&#9679;</span> half faded</span>
<span style="color:#3cb4e6">&#9473; sub's track</span><span>&#9650; sub</span>
<span style="color:#78dca0">&#9473; sonar slice</span><span>&#9675; start (0,0)</span>
<span>grid <span id="gl"></span></span>
<span style="color:#888">sector: counterclockwise, 0 right 90 up 180 left 270 down, same = full circle</span></div>
<script>
var $=function(i){return document.getElementById(i);};
var cv=$('cv'),ctx=cv.getContext('2d'),D=null,dirty={};
function post(path,body){return fetch(path,{method:'POST',body:body})
.then(function(r){return r.text();});}
['rg','s0','s1','th','fc'].forEach(function(i){
$(i).addEventListener('input',function(){dirty[i]=1;});
$(i).addEventListener('keydown',function(e){if(e.key==='Enter')apply();});});
function apply(){var jobs=[];
if(dirty.rg)jobs.push(post('/range',$('rg').value).then(function(x){return 'range: '+x;}));
if(dirty.s0||dirty.s1)jobs.push(post('/sector',$('s0').value+' '+$('s1').value)
.then(function(x){return 'sector: '+x;}));
if(dirty.th)jobs.push(post('/threshold',$('th').value).then(function(x){return 'threshold: '+x;}));
if(dirty.fc)jobs.push(post('/floorcut',$('fc').value).then(function(x){return 'floor cut: '+x;}));
dirty={};
if(!jobs.length){$('msg').textContent='nothing changed';return;}
Promise.all(jobs).then(function(m){$('msg').textContent=m.join('   ');});}
$('set').onclick=apply;
$('go').onclick=function(){var c={waiting:'start',running:'pause',paused:'resume'}[D&&D.state];
if(c)post('/mapcmd',c);};
$('rs').onclick=function(){post('/mapcmd','reset');};
$('rec').onclick=function(){post('/record',D&&D.recording?'stop':'start '+$('rn').value)
.then(function(x){$('msg').textContent='record: '+x;});};
$('mk').onclick=function(){post('/mark',$('rn').value).then(function(x){$('msg').textContent='mark: '+x;});};
$('ar').addEventListener('input',draw);$('fo').addEventListener('change',draw);$('ol').addEventListener('change',draw);
window.addEventListener('resize',draw);
function fill(i,v){var e=$(i);
if(!dirty[i]&&document.activeElement!==e&&v!==null&&v!==undefined)e.value=v;}
function nice(a){var s=[0.1,0.25,0.5,1,2,5,10,25,50];
for(var i=0;i<s.length;i++)if(a/s[i]<=14)return s[i];return 100;}
function hue(b,th){var f=Math.max(0,Math.min(1,(b-th)/Math.max(1,255-th)));
return 'hsl('+Math.round(210-210*f)+',95%,58%)';}
function draw(){
var r=cv.getBoundingClientRect(),k=window.devicePixelRatio||1;
cv.width=Math.round(r.width*k);cv.height=Math.round(r.height*k);ctx.setTransform(k,0,0,k,0,0);
var W=r.width,H=r.height;ctx.fillStyle='#161412';ctx.fillRect(0,0,W,H);
if(!D)return;
var area=Math.max(1,parseFloat($('ar').value)||10),px=Math.min(W,H)/area;
var c=($('fo').checked&&D.pose)?D.pose:[0,0];
var X=function(x){return W/2+(x-c[0])*px;},Y=function(y){return H/2-(y-c[1])*px;};
var g=nice(area),xa=c[0]-W/2/px,xb=c[0]+W/2/px,ya=c[1]-H/2/px,yb=c[1]+H/2/px,v;
ctx.lineWidth=1;
for(v=Math.ceil(xa/g)*g;v<=xb;v+=g){ctx.strokeStyle=Math.abs(v)<g/2?'#5d6b73':'#2e2a27';
ctx.beginPath();ctx.moveTo(X(v),0);ctx.lineTo(X(v),H);ctx.stroke();}
for(v=Math.ceil(ya/g)*g;v<=yb;v+=g){ctx.strokeStyle=Math.abs(v)<g/2?'#5d6b73':'#2e2a27';
ctx.beginPath();ctx.moveTo(0,Y(v));ctx.lineTo(W,Y(v));ctx.stroke();}
$('gl').textContent=g+' m';
ctx.strokeStyle='#ddd';ctx.beginPath();ctx.arc(X(0),Y(0),6,0,7);ctx.stroke();
var th=D.threshold||100,R=Math.max(2.5,Math.min(5,px*0.05));
[1,0].forEach(function(faded){ctx.globalAlpha=faded?0.3:1;
D.crumbs.forEach(function(q){if((q[3]>0?1:0)!==faded)return;
ctx.fillStyle=hue(q[2],th);ctx.beginPath();ctx.arc(X(q[0]),Y(q[1]),R,0,7);ctx.fill();});});
ctx.globalAlpha=1;
if(D.truth){ctx.save();ctx.setLineDash([7,5]);ctx.strokeStyle='rgba(255,255,255,0.6)';ctx.lineWidth=2;
ctx.beginPath();D.truth.pipe.forEach(function(p,i){if(i)ctx.lineTo(X(p[0]),Y(p[1]));else ctx.moveTo(X(p[0]),Y(p[1]));});
ctx.stroke();ctx.restore();ctx.strokeStyle='#ffa83c';ctx.lineWidth=2;
D.truth.boxes.forEach(function(b){ctx.strokeRect(X(b[0])-5,Y(b[1])-5,10,10);});}
if($('ol').checked&&D.outlines){ctx.font='12px sans-serif';
D.outlines.forEach(function(g,i){var best=i===0&&g.score>=0.5;
ctx.strokeStyle=best?'rgba(255,220,80,0.9)':'rgba(255,255,255,0.45)';ctx.lineWidth=1;ctx.beginPath();
g.lines.forEach(function(l){ctx.moveTo(X(l[0]),Y(l[1]));ctx.lineTo(X(l[2]),Y(l[3]));});ctx.stroke();
if(best){ctx.strokeStyle='rgba(255,220,80,0.5)';ctx.lineWidth=4;ctx.beginPath();
g.spine.forEach(function(p,k){if(k)ctx.lineTo(X(p[0]),Y(p[1]));else ctx.moveTo(X(p[0]),Y(p[1]));});ctx.stroke();}
if(g.score>=0.2){var m=g.spine[Math.floor(g.spine.length/2)];ctx.fillStyle=best?'#ffdc50':'#ddd';
ctx.fillText('pipe '+Math.round(g.score*100)+'%',X(m[0])+8,Y(m[1])-8);}});}
if(D.track.length>1){ctx.strokeStyle='#3cb4e6';ctx.lineWidth=2;ctx.beginPath();
D.track.forEach(function(p,i){if(i)ctx.lineTo(X(p[0]),Y(p[1]));else ctx.moveTo(X(p[0]),Y(p[1]));});
ctx.stroke();}
if(D.pose&&D.sonar){var h=D.pose[2]*Math.PI/180,rx=Math.cos(h),ry=-Math.sin(h),s=D.sonar;
ctx.strokeStyle='rgba(120,220,160,0.6)';ctx.lineWidth=2;ctx.beginPath();
ctx.moveTo(X(s[0]-rx*D.slice[0]),Y(s[1]-ry*D.slice[0]));
ctx.lineTo(X(s[0]+rx*D.slice[1]),Y(s[1]+ry*D.slice[1]));ctx.stroke();}
if(D.pose){var a=D.pose[2]*Math.PI/180,sx=X(D.pose[0]),sy=Y(D.pose[1]),fx=Math.sin(a),fy=-Math.cos(a);
ctx.fillStyle='#fff';ctx.beginPath();ctx.moveTo(sx+fx*14,sy+fy*14);
ctx.lineTo(sx-fx*8-fy*7,sy-fy*8+fx*7);ctx.lineTo(sx-fx*8+fy*7,sy-fy*8-fx*7);
ctx.closePath();ctx.fill();}}
(function poll(){fetch('/mapdata').then(function(r){return r.json();}).then(function(d){
D=d;fill('rg',d.range);
if(d.sector){fill('s0',Math.round(d.sector[0]));fill('s1',Math.round(d.sector[1]));}
fill('th',d.threshold);if(d.floorCut!=null)fill('fc',d.floorCut);
$('go').textContent={waiting:'Start',running:'Pause',paused:'Resume'}[d.state]||'Start';
var p=d.pose;
var rc=d.recording;$('rec').textContent=rc?'Stop rec':'Record';
$('rec').style.background=rc?'#8a1a1a':'';
$('st').textContent=(d.truth?'SIM: dashed = real pipeline, orange = light boxes   ':'')+
(rc?'\\u25cf REC '+rc.name+' '+rc.seconds+' s, '+rc.sweeps+' sweeps   ':'')+
d.state+'   sweep '+d.sweep+'   '+d.count+' crumbs   DVL '+d.dvl+
(p?'   x '+p[0].toFixed(2)+'  y '+p[1].toFixed(2)+'  heading '+p[2].toFixed(0)+'\\u00b0':'');
var msg='',bg='';
if(d.dvl==='no data'){msg='no VectorNav / DVL data yet - is pose_relay.py running?';bg='#7a2a1a';}
else if(d.dvl==='lost'){msg='DVL lost - not adding crumbs';bg='#8a1a1a';}
else if(d.state==='waiting'){msg='press Start: (0,0) will be where the sub is then';bg='#2a4a5a';}
else if(d.turning){msg='turning fast - not adding crumbs';bg='#5a3a6a';}
else if(d.state==='paused'){msg='paused - map frozen, sub still tracked';bg='#5a4a1a';}
var b=$('banner');b.style.display=msg?'block':'none';b.textContent=msg;b.style.background=bg;
draw();}).catch(function(){}).then(function(){setTimeout(poll,400);});})();
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
        """Settings that come back from the pages: the target, the range, the
        view, and on the Map tab the sector, the threshold and its buttons.

        Bodies are capped: this listens on 0.0.0.0 so anything on the tether
        network can reach it, and all of these are short.
        """
        global _target, _range, _sector, _threshold
        if self.path in ('/sector', '/threshold', '/floorcut', '/mapcmd', '/record', '/mark'):
            self._map_post()
            return
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
        if self.path == '/range':
            with _lock:
                locked = not _range_editable
            if locked:
                self._send(b'range is fixed for this run', 'text/plain')
                return
            try:
                want = float(self._body().decode('utf-8', 'replace').strip())
            except ValueError:
                want = float('nan')
            # NaN fails both comparisons, so it is refused here too.
            with _lock:
                lo, hi = _range_lo, _range_hi
            if not lo <= want <= hi:
                self._send(f'need {lo:g}-{hi:g} m'.encode(), 'text/plain')
                return
            with _lock:
                _range = want
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

    def _map_post(self):
        global _sector, _threshold, _floor_cut
        with _lock:
            on = _map_on
        if not on:
            self.send_error(404)
            return
        text = self._body().decode('utf-8', 'replace').strip()[:300]
        if self.path in ('/record', '/mark'):
            if self.path == '/mark':
                cmd = ("mark", text)
            elif text == 'stop':
                cmd = ("stop", "")
            elif text.startswith('start'):
                cmd = ("start", text[5:].strip())
            else:
                self._send(b'say start <note> or stop', 'text/plain')
                return
            with _lock:
                _rec_cmds.append(cmd)
            self._send(b'ok', 'text/plain')
            return
        if self.path == '/mapcmd':
            if text not in ('start', 'pause', 'resume', 'reset'):
                self._send(b'unknown button', 'text/plain')
                return
            with _lock:
                _map_cmds.append(text)
            self._send(b'ok', 'text/plain')
            return
        if self.path == '/sector':
            with _lock:
                locked = not _sector_editable
            if locked:
                self._send(b'sector is fixed for this run', 'text/plain')
                return
            try:
                a, b = (float(v) for v in text.replace(',', ' ').split())
            except ValueError:
                a = b = float('nan')
            if not (0 <= a < 360 and 0 <= b < 360):
                self._send(b'need two angles, 0-359', 'text/plain')
                return
            with _lock:
                _sector = (a, b)
            self._send(b'ok', 'text/plain')
            return
        if self.path == '/floorcut':
            try:
                cut = float(text)
            except ValueError:
                cut = -1.0
            if not 0.0 <= cut <= 3.0:
                self._send(b'need 0-3 m', 'text/plain')
                return
            with _lock:
                _floor_cut = cut
            self._send(b'ok', 'text/plain')
            return
        try:
            want = int(float(text))
        except ValueError:
            want = -1
        if not 1 <= want <= 255:
            self._send(b'need 1-255', 'text/plain')
            return
        with _lock:
            _threshold = want
        self._send(b'ok', 'text/plain')

    def do_GET(self):
        if self.path == '/':
            self._send(PAGE, 'text/html')
            return
        if self.path in ('/map', '/mapdata'):
            with _lock:
                on, data = _map_on, _map_json
            if not on:
                self.send_error(404)
            elif self.path == '/map':
                self._send(MAP_PAGE, 'text/html')
            else:
                self._send(data, 'application/json')
            return
        if self.path == '/state':
            with _lock:
                state = {'target': _target, 'note': _note, 'editable': _editable,
                         'range': _range, 'rangeEditable': _range_editable,
                         'rangeMin': _range_lo, 'rangeMax': _range_hi, 'map': _map_on,
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
