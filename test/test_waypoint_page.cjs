const fs=require('fs'),vm=require('vm'),assert=require('assert');
const code=fs.readFileSync('tools/waypoint.html','utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
async function render(data,options={}){
  const elements={},posts=[],imageSources=[],ctx2d=new Proxy({},{get:()=>()=>{}});let now=100,starts=0;
  const el=id=>elements[id]??=(id==='map'?{getContext:()=>ctx2d,getBoundingClientRect:()=>({width:900,height:600,...options.viewport,left:0,top:0})}:{checked:options.corridor!==false,value:id==='maptype'?'geographic':id==='imagery'?'current':id==='depth'?'0.5':'',textContent:''});
    const ctx={document:{getElementById:el},window:{},Image:class{complete=!!options.tilesLoaded;naturalWidth=options.tilesLoaded?256:0;set src(url){imageSources.push(url)}},devicePixelRatio:1,
    location:{pathname:'/waypoint'},performance:{now:()=>now},AbortSignal:{timeout:()=>null},
    setTimeout:()=>{},fetch:async(url,request)=>{if(!request?.body)return{ok:true,json:async()=>data};const body=JSON.parse(request.body);posts.push(body);if(body.action==='start'&&++starts===1&&options.timeoutStart)throw Error('Timeout');return{ok:!options.reject,json:async()=>options.reject?{ok:false,msg:'Target rejected'}:{ok:true,plan_id:'single-plan',run_id:'run'}}}};
  vm.runInNewContext(code,ctx);await new Promise(setImmediate);elements.posts=posts;elements.imageSources=imageSources;elements.refreshAt=async t=>{now=t;await ctx.poll()};elements.tileXY=ctx.tileXY;elements.tileGeo=ctx.tileGeo;elements.tileURL=ctx.tileURL;return elements;
}
(async()=>{
  let e=await render({capabilities:{enabled:false},navigation:{streams:{},cube_estimate:{valid:false}}});
  assert(e.start.disabled);assert.match(e.readiness.textContent,/Start navigation first/);
  assert.match(e.readiness.textContent,/Cube control telemetry unavailable/);
  const ready={capabilities:{enabled:true,simulation:true},navigation:{streams:{bridge:{fresh:true,data:{estimator_health:{usable:true,state:'AIDED',sensors:{gps:true,dvl:true,vn:true},position_sigma_m:1.}}},supervisor:{fresh:true,data:{mode:'Active',configuration_verified:true,surfaced:true,state:'GPS aiding admitted'}}},cube_estimate:{valid:true,lat:32.9,lon:-117,heading_deg:0,depth_m:0}}};
  e=await render(ready);
  const bow=vm.runInNewContext(code.slice(code.indexOf('function bowPoint'),code.indexOf('function mark'))+'; bowPoint');
  for(const [heading,x,y] of [[0,0,-25],[90,25,0],[180,0,25],[270,-25,0]]){const p=bow([0,0],heading,25);assert(Math.abs(p[0]-x)<1e-8&&Math.abs(p[1]-y)<1e-8)}
  const northwest=bow([0,0],305,25);assert(northwest[0]<0&&northwest[1]<0);
  assert.match(e.readiness.textContent,/Select a target/);assert(e.start.disabled);
  e.zoomIn.onclick();e.zoomOut.onclick();e.fit.onclick();
  e.map.onclick({clientX:500,clientY:300});assert.equal(e.posts.length,0);assert(!e.start.disabled);
  await e.refreshAt(60100);assert(!e.start.disabled);await e.start.onclick();assert.deepEqual(e.posts.map(p=>p.action),['preview','start']);assert.equal(e.posts[1].plan_id,'single-plan');assert(e.start.disabled);
  assert.equal(e.posts[0].plan.mode,'surface');assert.equal(e.posts[0].plan.depth_m,0);
  e=await render(ready,{reject:true});e.map.onclick({clientX:500,clientY:300});await e.start.onclick();assert.equal(e.posts.length,1);assert.match(e.message.textContent,/Target rejected/);
  e=await render(ready,{timeoutStart:true});e.map.onclick({clientX:500,clientY:300});await e.start.onclick();await e.start.onclick();assert.deepEqual(e.posts.map(p=>p.action),['preview','start','start']);assert.equal(e.posts[1].plan_id,e.posts[2].plan_id);
  e=await render({...ready,capabilities:{enabled:true,simulation:false}});e.maptype.value='pool';e.maptype.onchange();
  e.map.onclick({clientX:500,clientY:300});assert(e.start.disabled);assert(!e.useGrid.hidden);assert.match(e.readiness.textContent,/satellite or geographic/);
  e.useGrid.onclick();assert.equal(e.maptype.value,'geographic');assert(e.useGrid.hidden);assert(e.start.disabled);assert.equal(e.posts.length,0);
  e.map.onclick({clientX:500,clientY:300});assert(!e.start.disabled);
  e=await render({...ready,capabilities:{enabled:true,simulation:false}},{tilesLoaded:true});e.maptype.value='satellite';e.maptype.onchange();
  assert(e.useGrid.hidden);assert(!e.attribution.hidden);assert.match(e.tileStatus.textContent,/Satellite imagery loaded/);
  e.map.onclick({clientX:500,clientY:300});assert(!e.start.disabled);await e.start.onclick();assert.equal(e.posts[0].plan.map,'geographic');
  for(const p of [{lat:32.9236125,lon:-117.0385235},{lat:-33.5,lon:151.2}]){const xy=e.tileXY(p,20),q=e.tileGeo(xy.x,xy.y,20);assert(Math.abs(q.lat-p.lat)<1e-10);assert(Math.abs(q.lon-p.lon)<1e-10)}
  e=await render(ready,{tilesLoaded:true});e.maptype.value='satellite';e.maptype.onchange();
  e.map.onclick({clientX:450,clientY:400});
  e.map.onclick({clientX:550,clientY:300});
  await e.start.onclick();
  assert(e.posts[0].plan.lon>ready.navigation.cube_estimate.lon);
  assert(Math.abs(e.posts[0].plan.lat-ready.navigation.cube_estimate.lat)<1e-8);
  e=await render(ready,{tilesLoaded:true});e.maptype.value='satellite';e.maptype.onchange();
  assert.match(e.tileURL(183388,422631,20),/MapServer\/tile\/20\/422631\/183388$/);
  e.imagery.value='archive2023';e.imagery.onchange();
  assert.match(e.tileURL(183388,422631,20),/tile\/64776\/20\/422631\/183388$/);
  e.findPool.onclick();e.retryTiles.onclick();e.map.onclick({clientX:450,clientY:300});
  assert.match(e.target.textContent,/32\.9236125, -117\.0385235/);assert.equal(e.posts.length,0);
  e.imagery.value='current';e.imagery.onchange();assert(e.start.disabled);
  assert.equal(e.tileURL(183388,422631,20),'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/20/422631/183388');
  e.map.onclick({clientX:450,clientY:300});await e.start.onclick();assert.equal(e.posts[0].plan.map,'geographic');
  e=await render(ready);e.maptype.value='satellite';e.maptype.onchange();e.map.onclick({clientX:500,clientY:300});assert(e.start.disabled);assert.match(e.message.textContent,/Wait for satellite/);assert.equal(e.posts.length,0);
  const telemetry={...ready,navigation:{...ready.navigation,streams:{...ready.navigation.streams,
    local:{fresh:true,age:.1,data:{ned:[1,2,.5],velocity:[.3,.4,0]}},
    attitude:{fresh:true,age:.1,data:{rpy:[2,-3,359]}},
    bridge:{fresh:true,age:.1,data:{inputs:{vn:true},pressure_sample:{source:'SCALED_PRESSURE3'},depth_m:.4,heading_reference_validated:true,transmitting:true,heading_vn:{yaw:1},north_alignment:{offset_deg:90},heading_bias_deg:2}},
    ekf:{fresh:true,age:.2,data:{flags:3,velocity_variance:.1,horizontal_variance:.2,vertical_variance:.3}},
    gps:{fresh:true,age:.2,data:{valid:true,lat:32.9,lon:-117}}
  }}};
  e=await render(telemetry);assert.equal(e.headingDifference.textContent,'-2.0°');assert.match(e.alignedHeading.textContent,/1.0° · transmitting/);
  e=await render({...telemetry,navigation:{...telemetry.navigation,streams:{...telemetry.navigation.streams,
    bridge:{fresh:true,age:.1,data:{...telemetry.navigation.streams.bridge.data,course_heading:{reason:'Collecting trusted motion',method:null}}},
    waypoint:{fresh:true,data:{stage:'TRANSIT',surface_leg:{stage:'aligning',reason:'Turning toward waypoint',cross_track_m:.12}}}}}});
  assert.match(e.surfaceGuidance.textContent,/aligning.*Turning toward waypoint.*0.12 m/);
  assert.match(e.courseHeading.textContent,/Collecting trusted motion.*2.0°/);
  assert.match(e.depthSource.textContent,/SCALED_PRESSURE3 · 0.40 m/);
  assert.match(e.rawGPSStatus.textContent,/NOT AIDING/);
  e=await render({...telemetry,navigation:{...telemetry.navigation,streams:{...telemetry.navigation.streams,
    waypoint:{fresh:true,data:{stage:'WAIT_SURFACE',surface_measurement:{ready:false,reason:'Surface measurement 12/20 s'}}}}}});
  assert.match(e.surfaceMeasurement.textContent,/pending.*12\/20.*not proof/);
  assert.equal(e.headingOffsets.textContent,'90.0° / 2.0°');assert.equal(e.cubeSpeed.textContent,'0.50');assert.equal(e.gpsSeparation.textContent,'0.00');assert.match(e.estimateAge.textContent,/local: 0.10 s/);
  e=await render({...telemetry,navigation:{...telemetry.navigation,streams:{...telemetry.navigation.streams,bridge:{fresh:false,age:4,data:telemetry.navigation.streams.bridge.data},local:{fresh:false,age:4,data:telemetry.navigation.streams.local.data}}}});
  assert.match(e.alignedHeading.textContent,/Unavailable/);assert.equal(e.headingDifference.textContent,'—');assert.equal(e.cubeSpeed.textContent,'—');assert.match(e.estimateAge.textContent,/4.00 s · stale/);
  e=await render(ready,{tilesLoaded:true,viewport:{width:1440,height:900}});e.maptype.value='satellite';e.imagery.value='archive2023';e.maptype.onchange();
  assert(e.imageSources.some(s=>s.includes('/tile/64776/20/')));assert(!e.imageSources.some(s=>s.includes('/tile/64776/21/')));
  assert.match(e.tileStatus.textContent,/zoom 20/);e.zoomIn.onclick();assert(!e.imageSources.some(s=>s.includes('/tile/64776/21/')));
  e.imagery.value='current';e.imagery.onchange();assert(e.imageSources.some(s=>s.includes('/MapServer/tile/19/')));assert(!e.imageSources.some(s=>s.includes('/MapServer/tile/21/')));
  e.findLake.onclick();e.map.onclick({clientX:720,clientY:450});
  assert.match(e.target.textContent,/32\.9160000, -117\.0990000/);
  assert.equal(e.posts.length,0);
  e.panN.onclick();e.map.onclick({clientX:720,clientY:450});assert(!e.target.textContent.includes('32.9160000'));
  e=await render({...ready,navigation:{...ready.navigation,cube_estimate:{...ready.navigation.cube_estimate,lat:32.9137076,lon:-117.1001865}}},{tilesLoaded:true});
  e.imagery.value='localMiramar';e.maptype.value='satellite';e.imagery.onchange();
  assert(e.imageSources.some(s=>s==='/vendor/miramar/19-211332-91604.jpg'));
  e.map.onclick({clientX:450,clientY:300});assert(!e.start.disabled);
  e.alignmentCorridor.checked=false;e.alignmentCorridor.onchange();assert(e.start.disabled);
  assert.match(e.readiness.textContent,/clear 10 m corridor/);
  e.alignmentCorridor.checked=true;e.alignmentCorridor.onchange();assert(!e.start.disabled);
  e.findLake.onclick();assert(e.imageSources.some(s=>s==='/vendor/miramar/16-26416-11450.jpg'));
  e=await render(ready);
  e.targetLat.value='32.92357';e.targetLon.value='-117.03855';e.useCoordinates.onclick();
  assert.match(e.target.textContent,/32.9235700, -117.0385500/);assert.equal(e.posts.length,0);
  e.targetLat.value='';e.useCoordinates.onclick();assert.match(e.message.textContent,/Enter latitude/);
  e.targetLat.value='91';e.useCoordinates.onclick();assert.match(e.message.textContent,/Enter latitude/);
  assert(e.recordGPS.disabled);e.recordGPS.onclick();assert.match(e.gpsRecord.textContent,/fresh valid raw GPS/);
  const gpsData={...ready,navigation:{...ready.navigation,streams:{...ready.navigation.streams,gps:{fresh:true,data:{valid:true,lat:32.92357,lon:-117.03855,accuracy:1.2}}}}};
  e=await render(gpsData);assert(!e.recordGPS.disabled);e.recordGPS.onclick();
  assert.equal(e.targetLat.value,'32.9235700');assert.equal(e.targetLon.value,'-117.0385500');
  assert.match(e.gpsRecord.textContent,/1.20 m/);assert.equal(e.posts.length,0);assert(e.start.disabled);
  e.useCoordinates.onclick();assert(!e.start.disabled);
  e=await render({...gpsData,navigation:{...gpsData.navigation,streams:{...gpsData.navigation.streams,gps:{fresh:false,data:gpsData.navigation.streams.gps.data}}}});assert(e.recordGPS.disabled);
  console.log('Waypoint map, desktop archive resolution, readiness, Start retry and estimate stats checks pass');
})().catch(e=>{console.error(e);process.exit(1)});
