const fs=require('fs'),vm=require('vm'),assert=require('assert');
const code=fs.readFileSync('tools/waypoint.html','utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
async function render(data,options={}){
  const elements={},posts=[],ctx2d=new Proxy({},{get:()=>()=>{}});let now=100,starts=0;
  const el=id=>elements[id]??=(id==='map'?{getContext:()=>ctx2d,getBoundingClientRect:()=>({width:900,height:600,left:0,top:0})}:{value:id==='maptype'?'geographic':id==='depth'?'0.5':'',textContent:''});
    const ctx={document:{getElementById:el},window:{},Image:class{complete=!!options.tilesLoaded;naturalWidth=options.tilesLoaded?256:0},devicePixelRatio:1,
    location:{pathname:'/waypoint'},performance:{now:()=>now},AbortSignal:{timeout:()=>null},
    setTimeout:()=>{},fetch:async(url,request)=>{if(!request?.body)return{ok:true,json:async()=>data};const body=JSON.parse(request.body);posts.push(body);if(body.action==='start'&&++starts===1&&options.timeoutStart)throw Error('Timeout');return{ok:!options.reject,json:async()=>options.reject?{ok:false,msg:'Target rejected'}:{ok:true,plan_id:'single-plan',run_id:'run'}}}};
  vm.runInNewContext(code,ctx);await new Promise(setImmediate);elements.posts=posts;elements.refreshAt=async t=>{now=t;await ctx.poll()};elements.tileXY=ctx.tileXY;elements.tileGeo=ctx.tileGeo;return elements;
}
(async()=>{
  let e=await render({capabilities:{enabled:false},navigation:{streams:{},cube_estimate:{valid:false}}});
  assert(e.start.disabled);assert.match(e.readiness.textContent,/commissioning disabled/);
  assert.match(e.readiness.textContent,/Cube estimate unavailable/);
  const ready={capabilities:{enabled:true,simulation:true},navigation:{streams:{supervisor:{fresh:true,data:{mode:'Active',configuration_verified:true,surfaced:true,state:'GPS aiding admitted'}}},cube_estimate:{valid:true,lat:32.9,lon:-117,heading_deg:0,depth_m:0}}};
  e=await render(ready);
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
  e=await render(ready);e.maptype.value='satellite';e.maptype.onchange();e.map.onclick({clientX:500,clientY:300});assert(e.start.disabled);assert.match(e.message.textContent,/Wait for satellite/);assert.equal(e.posts.length,0);
  console.log('Waypoint rendering, readiness explanations and zoom controls pass');
})().catch(e=>{console.error(e);process.exit(1)});
