/* ADAPTED: only GRAEY identity, approved missions/sensors, and deferred RFD.
   page.html/map.js/editor.js/gcs_app.py remain byte-for-byte Crusader. */
'use strict';
document.title='GRAEY GCS';
// User-requested light mode, including browsers with an older dark preference.
// Apply once; subsequent explicit theme choices retain Crusader's persistence.
try {
  if(localStorage.getItem('graey-light-default-v1')!=='1') {
    themeSet('light',true);
    localStorage.setItem('graey-light-default-v1','1');
  }
} catch(e) { themeSet('light',false); }
themeChosen=true;
for(const node of document.querySelectorAll('body *'))for(const text of node.childNodes){if(text.nodeType===Node.TEXT_NODE&&text.textContent.trim()==='Crusader')text.textContent=text.textContent.replace('Crusader','GRAEY')}
const originalShow=show;
const embeds={};
function embed(id,url){const section=$('t-'+id),legacy=document.createElement('div');while(section.firstChild)legacy.appendChild(section.firstChild);legacy.hidden=true;section.appendChild(legacy);const frame=document.createElement('iframe');frame.title='GRAEY '+id;frame.style.cssText='width:100%;height:calc(100vh - 160px);border:0';frame.src=url;section.appendChild(frame);embeds[id]=frame}
embed('mission','/missions');embed('sensors','/sensors');
const sensorTools=document.createElement('iframe');sensorTools.title='GRAEY camera and sonar';sensorTools.src='/graey-tools';sensorTools.style.cssText='width:100%;height:620px;border:0';$('t-sensors').appendChild(sensorTools);
CrsdEditor.enter=()=>{};CrsdEditor.status=()=>{};loadMissions=async()=>{};
loadCamera=async()=>{};camLoop=()=>{};lidarLoop=()=>{};
show=function(id){if(id==='map')id='mission';originalShow(id);sensorTools.contentWindow.postMessage({graeySensorsVisible:id==='sensors'},location.origin)};
document.querySelector('[data-tab="map"]').hidden=true;
$('gate-release').hidden=true;$('radio-test').hidden=true;
const radioNote=document.createElement('p');radioNote.textContent='GRAEY RFD reporting is deferred; WiFi is the active reporting link.';$('t-radio').prepend(radioNote);
// Rover dock/search/obstacle planner profiles are mission-specific.
planners=async()=>{};for(const id of ['pp-apply','pp-save','pp-name','pp-info','pp-new']){const el=$(id);if(el)el.hidden=true}
const options=document.createElement('div');options.innerHTML='<label><input type="checkbox" id="graey-rec-camera"> Camera stream (Down + Front)</label> <label><input type="checkbox" id="graey-rec-sonar"> Sonar sweeps</label><p class="muted">Start camera/sonar in Sensors. Recording uses those existing owners and is independent of viewing and mission execution.</p>';$('rec-start').parentElement.prepend(options);
$('rec-hz').disabled=true;$('rec-hz').title='GRAEY camera/sonar services record their native streams';
const originalReq=req;req=function(op,fields={},quiet=false){if(op==='record'&&fields.action==='start')fields={...fields,camera:$('graey-rec-camera').checked,sonar:$('graey-rec-sonar').checked};return originalReq(op,fields,quiet)};
show(new URLSearchParams(location.search).get('tab')==='sensors'?'sensors':new URLSearchParams(location.search).get('tab')==='record'?'record':'mission');

// Sensor-specific battery uncertainty must not inherit Crusader's calibration claim.
battTile=function(h){return ['stale','—','not measured','A fresh GRAEY battery measurement has not been supplied to this adapter']};

function syncGraeyTheme(){const theme=document.documentElement.dataset.theme;for(const frame of [...Object.values(embeds),sensorTools])frame.contentWindow.postMessage({graeyTheme:theme},location.origin)}
for(const frame of [...Object.values(embeds),sensorTools])frame.addEventListener('load',syncGraeyTheme);
new MutationObserver(syncGraeyTheme).observe(document.documentElement,{attributes:true,attributeFilter:['data-theme']});syncGraeyTheme();
for(const node of document.querySelectorAll('span'))if(node.textContent.includes('crusader_params.yaml'))node.textContent=node.textContent.replace('crusader_params.yaml','graey_params.yaml');

posTile=function(b){return num(b.lat)&&num(b.lon)?['info',`${b.lat.toFixed(7)}, ${b.lon.toFixed(7)}`,`${fmt(b.spd,1,' m/s')} · ${fmt(b.hdg,0,'°')}`,'GRAEY Cube fused geographic position and attitude heading; raw GPS is a separate surface measurement']:null};
ekfTip=function(flags){return 'GRAEY existing local/geographic estimator-health readout; raw Cube EKF flags '+(flags==null?'unavailable':flags)};
const stopTile=TILES.find(t=>t[0]==='estop');stopTile[2]=(s,h)=>h&&triTile(h.estop,['ENGAGED','bad'],['RUN','ok'],'GRAEY software kill / ESC permission only. Physical magnetic kill is independent and is not sensed here.');
