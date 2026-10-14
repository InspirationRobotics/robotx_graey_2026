const fs=require('fs'),vm=require('vm'),assert=require('assert');
const code=fs.readFileSync('tools/waypoint.html','utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
async function render(data){
  const elements={},ctx2d=new Proxy({},{get:()=>()=>{}});
  const el=id=>elements[id]??=(id==='map'?{getContext:()=>ctx2d,getBoundingClientRect:()=>({width:900,height:600,left:0,top:0})}:{value:id==='maptype'?'geographic':id==='depth'?'0.5':'',textContent:''});
  const ctx={document:{getElementById:el},window:{},Image:class{},devicePixelRatio:1,
    location:{pathname:'/waypoint'},performance:{now:()=>100},AbortSignal:{timeout:()=>null},
    setTimeout:()=>{},fetch:async()=>({ok:true,json:async()=>data})};
  vm.runInNewContext(code,ctx);await new Promise(setImmediate);return elements;
}
(async()=>{
  let e=await render({capabilities:{enabled:false},navigation:{streams:{},cube_estimate:{valid:false}}});
  assert(e.start.disabled);assert.match(e.readiness.textContent,/commissioning disabled/);
  assert.match(e.readiness.textContent,/Cube estimate unavailable/);
  e=await render({capabilities:{enabled:true,simulation:true},navigation:{streams:{supervisor:{fresh:true,data:{mode:'Active',configuration_verified:true,surfaced:true,state:'GPS aiding admitted'}}},cube_estimate:{valid:true,lat:32.9,lon:-117,heading_deg:0,depth_m:0}}});
  assert.match(e.readiness.textContent,/Preview target/);assert(e.start.disabled);
  e.zoomIn.onclick();e.zoomOut.onclick();e.fit.onclick();
  console.log('Waypoint rendering, readiness explanations and zoom controls pass');
})().catch(e=>{console.error(e);process.exit(1)});
