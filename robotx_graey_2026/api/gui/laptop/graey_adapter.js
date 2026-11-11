/* ADAPTED: only GRAEY identity, approved missions/sensors, and deferred RFD.
   page.html/map.js/editor.js/gcs_app.py remain byte-for-byte Crusader. */
'use strict';
document.title='GRAEY GCS';
// GRAEY's competition_link_node is the equivalent of Crusader's ocs_client.
const originalNodes=nodes;
nodes=function(){
  const client=(G.nodes?.items||[]).find(item=>item.name==='communications');
  if(client)client.note='OCS client · automatic discovery / reconnect';
  return originalNodes();
};
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
const preparation=document.createElement('iframe');preparation.title='GRAEY VN calibration and north alignment';preparation.src='/calibration';preparation.style.cssText='width:100%;height:1300px;border:0';$('t-sensors').appendChild(preparation);

const sensorTools=document.createElement('iframe');sensorTools.title='GRAEY camera and sonar';sensorTools.src='/graey-tools';sensorTools.style.cssText='width:100%;height:620px;border:0';$('t-sensors').appendChild(sensorTools);
// Compact GRAEY adaptation; retain Crusader's existing pages and handlers.
const operatorStyle=document.createElement('style');operatorStyle.textContent=`
:root{--operator-height:calc(100dvh - 140px)}
#t-mission,#t-sensors{padding:0}
#t-mission>iframe{display:block;height:var(--operator-height)!important}
.graey-sensor-layout{display:grid;grid-template-columns:minmax(0,1fr) minmax(400px,1fr);gap:6px;height:var(--operator-height);grid-template-rows:minmax(0,1fr)}
.graey-sensor-layout>iframe{display:block;width:100%;height:100%!important;min-width:0;border:0}
.graey-sensor-layout>details{grid-column:1/-1;border:1px solid var(--line);padding:4px 8px}
.graey-sensor-layout>details>iframe{height:300px!important;display:block;width:100%}
#tabs select{font:inherit;padding:4px;background:var(--btn);color:var(--fg);border:1px solid var(--line)}
@media(max-width:900px){.graey-sensor-layout{height:auto;grid-template-columns:1fr;grid-template-rows:55dvh 65dvh auto}}
`;document.head.appendChild(operatorStyle);
const sensorLayout=document.createElement('div');sensorLayout.className='graey-sensor-layout';$('t-sensors').appendChild(sensorLayout);sensorLayout.append(embeds.sensors,preparation);
const viewers=document.createElement('details');viewers.className='box wide';viewers.innerHTML='<summary>Camera / sonar live views</summary>';sensorTools.style.height='420px';viewers.appendChild(sensorTools);
// Presentation only: place the existing viewer inside the same sensor-card grid.
embeds.sensors.addEventListener('load',()=>{const grid=embeds.sensors.contentDocument.querySelector('main.grid');if(grid)grid.insertBefore(viewers,grid.querySelector('details'))});
const toolsMenu=document.createElement('select');toolsMenu.setAttribute('aria-label','Tools');toolsMenu.innerHTML='<option value="">Tools…</option>';
for(const button of document.querySelectorAll('#tabs button')){const id=button.dataset.tab;if(!['mission','sensors','record'].includes(id)){button.hidden=true;if(id!=='map')toolsMenu.add(new Option(button.textContent,id))}}
$('tabs').appendChild(toolsMenu);toolsMenu.onchange=()=>{if(toolsMenu.value)show(toolsMenu.value)};
function fitOperator(){document.documentElement.style.setProperty('--operator-height',Math.max(360,innerHeight-document.querySelector('header').getBoundingClientRect().height-$('tabs').getBoundingClientRect().height-22)+'px')}
addEventListener('resize',fitOperator);new ResizeObserver(fitOperator).observe(document.querySelector('header'));fitOperator();

CrsdEditor.enter=()=>{};CrsdEditor.status=()=>{};loadMissions=async()=>{};
loadCamera=async()=>{};camLoop=()=>{};lidarLoop=()=>{};
let sensorsVisible=false;
function syncSensorVisibility(){sensorTools.contentWindow?.postMessage({graeySensorsVisible:sensorsVisible&&viewers.open},location.origin)}
sensorTools.addEventListener('load',syncSensorVisibility);viewers.addEventListener('toggle',syncSensorVisibility);
show=function(id){if(id==='map')id='mission';originalShow(id);toolsMenu.value=["mission","sensors","record"].includes(id)?"":id;sensorsVisible=id==='sensors';syncSensorVisibility()};
document.querySelector('[data-tab="map"]').hidden=true;
$('gate-release').hidden=true;$('radio-test').hidden=true;
const radioNote=document.createElement('p');radioNote.textContent='GRAEY RFD reporting is deferred; WiFi is the active reporting link.';$('t-radio').prepend(radioNote);
// Rover dock/search/obstacle planner profiles are mission-specific.
planners=async()=>{};for(const id of ['pp-apply','pp-save','pp-name','pp-info','pp-new']){const el=$(id);if(el)el.hidden=true}
const options=document.createElement('div');options.innerHTML='<label><input type="checkbox" id="graey-rec-camera"> Camera stream (Down + Front)</label> <label><input type="checkbox" id="graey-rec-sonar"> Sonar sweeps</label> <label title="Radio submerge test log: boat packets heard, raw depth, depth from start"><input type="checkbox" id="graey-rec-rfd"> RFD data</label><p class="muted">Recording starts selected sources if needed. Viewing and recording are independent; download saved sessions below. RFD data copies the radio submerge test log into the session while it records; start recording before the test.</p>';$('rec-start').parentElement.prepend(options);
$('rec-hz').disabled=true;$('rec-hz').title='GRAEY camera/sonar services record their native streams';
const originalReq=req;req=function(op,fields={},quiet=false){if(op==='record'&&fields.action==='start')fields={...fields,camera:$('graey-rec-camera').checked,sonar:$('graey-rec-sonar').checked,rfd:$('graey-rec-rfd').checked};return originalReq(op,fields,quiet)};
show(new URLSearchParams(location.search).get('tab')==='sensors'?'sensors':new URLSearchParams(location.search).get('tab')==='record'?'record':'mission');

// Sensor-specific battery uncertainty must not inherit Crusader's calibration claim.
battTile=function(h){return ['stale','—','not measured','A fresh GRAEY battery measurement has not been supplied to this adapter']};

function syncGraeyTheme(){const theme=document.documentElement.dataset.theme;for(const frame of [...Object.values(embeds),sensorTools,preparation])frame.contentWindow?.postMessage({graeyTheme:theme},location.origin)}
for(const frame of [...Object.values(embeds),sensorTools,preparation])frame.addEventListener('load',syncGraeyTheme);
new MutationObserver(syncGraeyTheme).observe(document.documentElement,{attributes:true,attributeFilter:['data-theme']});syncGraeyTheme();
for(const node of document.querySelectorAll('span'))if(node.textContent.includes('crusader_params.yaml'))node.textContent=node.textContent.replace('crusader_params.yaml','graey_params.yaml');

posTile=function(b){return num(b.lat)&&num(b.lon)?['info',`${b.lat.toFixed(7)}, ${b.lon.toFixed(7)}`,`${fmt(b.spd,1,' m/s')} · ${fmt(b.hdg,0,'°')}`,'GRAEY Cube fused geographic position and attitude heading; raw GPS is a separate surface measurement']:null};
ekfTip=function(flags){return 'GRAEY existing local/geographic estimator-health readout; raw Cube EKF flags '+(flags==null?'unavailable':flags)};
const stopTile=TILES.find(t=>t[0]==='estop');stopTile[2]=(s,h)=>h&&triTile(h.estop,['ENGAGED','bad'],['RUN','ok'],'GRAEY software kill / ESC permission only. Physical magnetic kill is independent and is not sensed here.');
