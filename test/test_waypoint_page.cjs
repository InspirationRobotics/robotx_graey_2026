const fs=require('fs'),vm=require('vm'),assert=require('assert');
const code=fs.readFileSync('tools/waypoint.html','utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
async function render(data,options={}){
  const elements={},posts=[],imageSources=[],ctx2d=new Proxy({},{get:()=>()=>{}});let now=100,starts=0;
  const el=id=>elements[id]??=(id==='map'?{getContext:()=>ctx2d,getBoundingClientRect:()=>({width:900,height:600,...options.viewport,left:0,top:0})}:{value:id==='maptype'?'geographic':id==='depth'?'0.5':'',textContent:''});
    const ctx={document:{getElementById:el},window:{},Image:class{complete=!!options.tilesLoaded;naturalWidth=options.tilesLoaded?256:0;set src(url){imageSources.push(url)}},devicePixelRatio:1,
    location:{pathname:'/waypoint'},performance:{now:()=>now},AbortSignal:{timeout:()=>null},
    setTimeout:()=>{},fetch:async(url,request)=>{if(!request?.body)return{ok:true,json:async()=>data};const body=JSON.parse(request.body);posts.push(body);if(body.action==='start'&&++starts===1&&options.timeoutStart)throw Error('Timeout');return{ok:!options.reject,json:async()=>options.reject?{ok:false,msg:'Target rejected'}:{ok:true,plan_id:'single-plan',run_id:'run'}}}};
  vm.runInNewContext(code,ctx);await new Promise(setImmediate);elements.posts=posts;elements.imageSources=imageSources;elements.refreshAt=async t=>{now=t;await ctx.poll()};elements.tileXY=ctx.tileXY;elements.tileGeo=ctx.tileGeo;elements.tileURL=ctx.tileURL;return elements;
}
(async()=>{
  let e=await render({capabilities:{enabled:false},navigation:{streams:{},cube_estimate:{valid:false}}});
  assert(e.start.disabled);assert.match(e.readiness.textContent,/commissioning disabled/);
  assert.match(e.readiness.textContent,/Cube estimate unavailable/);
  const ready={capabilities:{enabled:true,simulation:true},navigation:{streams:{supervisor:{fresh:true,data:{mode:'Active',configuration_verified:true,surfaced:true,state:'GPS aiding admitted'}}},cube_estimate:{valid:true,lat:32.9,lon:-117,heading_deg:0,depth_m:0}}};
  e=await render(ready);
  const bow=vm.runInNewContext(code.slice(code.indexOf('function bowPoint'),code.indexOf('function mark'))+'; bowPoint');
  for(const [heading,x,y] of [[0,0,-25],[90,25,0],[180,0,25],[270,-25,0]]){const p=bow([0,0],heading,25);assert(Math.abs(p[0]-x)<1e-8&&Math.abs(p[1]-y)<1e-8)}
  const northwest=bow([0,0],305,25);assert(northwest[0]<0&&northwest[1]<0);
  assert.match(e.readiness.textContent,/Select a target/);assert(e.start.disabled);
  e.zoomIn.onclick();e.zoomOut.onclick();e.fit.onclick();
  e.map.onclick({clientX:500,clientY:300});assert.equal(e.posts.length,0);assert(!e.start.disabled);
  await e.refreshAt(60100);assert(!e.start.disabled);await e.start.onclick();assert.deepEqual(e.posts.map(p=>p.action),['preview','start']);assert.equal(e.posts[1].plan_id,'single-plan');assert(e.start.disabled);
  e=await render(ready,{reject:true});e.map.onclick({clientX:500,clientY:300});await e.start.onclick();assert.equal(e.posts.length,1);assert.match(e.message.textContent,/Target rejected/);
  e=await render(ready,{timeoutStart:true});e.map.onclick({clientX:500,clientY:300});await e.start.onclick();await e.start.onclick();assert.deepEqual(e.posts.map(p=>p.action),['preview','start','start']);assert.equal(e.posts[1].plan_id,e.posts[2].plan_id);
  e=await render({...ready,capabilities:{enabled:true,simulation:false}});e.maptype.value='pool';e.maptype.onchange();
  e.map.onclick({clientX:500,clientY:300});assert(e.start.disabled);assert(!e.useGrid.hidden);assert.match(e.readiness.textContent,/view-only/);
  e.useGrid.onclick();assert.equal(e.maptype.value,'geographic');assert(e.useGrid.hidden);assert(e.start.disabled);assert.equal(e.posts.length,0);
  e.map.onclick({clientX:500,clientY:300});assert(!e.start.disabled);
  e=await render({...ready,capabilities:{enabled:true,simulation:false}},{tilesLoaded:true});e.maptype.value='satellite';e.maptype.onchange();
  assert(e.useGrid.hidden);assert(!e.attribution.hidden);assert.match(e.tileStatus.textContent,/Satellite imagery loaded/);
  e.map.onclick({clientX:500,clientY:300});assert(!e.start.disabled);await e.start.onclick();assert.equal(e.posts[0].plan.map,'geographic');
  for(const p of [{lat:32.9236125,lon:-117.0385235},{lat:-33.5,lon:151.2}]){const xy=e.tileXY(p,20),q=e.tileGeo(xy.x,xy.y,20);assert(Math.abs(q.lat-p.lat)<1e-10);assert(Math.abs(q.lon-p.lon)<1e-10)}
  e=await render(ready,{tilesLoaded:true});e.maptype.value='satellite';e.maptype.onchange();
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
  assert.match(e.depthSource.textContent,/SCALED_PRESSURE3 · 0.40 m/);
  assert.match(e.rawGPSStatus.textContent,/NOT AIDING/);
  assert.equal(e.headingOffsets.textContent,'90.0° / 2.0°');assert.equal(e.cubeSpeed.textContent,'0.50');assert.equal(e.gpsSeparation.textContent,'0.00');assert.match(e.estimateAge.textContent,/local: 0.10 s/);
  e=await render({...telemetry,navigation:{...telemetry.navigation,streams:{...telemetry.navigation.streams,bridge:{fresh:false,age:4,data:telemetry.navigation.streams.bridge.data},local:{fresh:false,age:4,data:telemetry.navigation.streams.local.data}}}});
  assert.match(e.alignedHeading.textContent,/Unavailable/);assert.equal(e.headingDifference.textContent,'—');assert.equal(e.cubeSpeed.textContent,'—');assert.match(e.estimateAge.textContent,/4.00 s · stale/);
  e=await render(ready,{tilesLoaded:true,viewport:{width:1440,height:900}});e.maptype.value='satellite';e.maptype.onchange();
  assert(e.imageSources.some(s=>s.includes('/tile/64776/20/')));assert(!e.imageSources.some(s=>s.includes('/tile/64776/21/')));
  assert.match(e.tileStatus.textContent,/zoom 20/);e.zoomIn.onclick();assert(!e.imageSources.some(s=>s.includes('/tile/64776/21/')));
  e.imagery.value='current';e.imagery.onchange();assert(e.imageSources.some(s=>s.includes('/MapServer/tile/21/')));
  console.log('Waypoint map, desktop archive resolution, readiness, Start retry and estimate stats checks pass');
})().catch(e=>{console.error(e);process.exit(1)});
