const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync('tools/calibration.html','utf8');
const code=html.match(/<script>([\s\S]*?)<\/script>/)[1];
async function page(state){
 const e={},posts=[];let now=100;
 const el=id=>e[id]??={checked:false,disabled:false,textContent:'',value:id==='alignmentMode'?'gps':id==='alignmentDistance'?10:''};
 const ctx={document:{getElementById:el},performance:{now:()=>now},setTimeout:()=>{},setInterval:()=>{},fetch:async(url,options)=>{
  if(options){posts.push(JSON.parse(options.body));return{ok:true,json:async()=>({ok:true})}}
  return{ok:true,json:async()=>state};}};
 vm.runInNewContext(code,ctx);await new Promise(setImmediate);
 return{e,posts,refresh:()=>e.surfaceConfirm.onchange(),expire:()=>{now=5000;e.surfaceConfirm.onchange()}};
}
(async()=>{
 assert(!html.includes('compassStart'));assert(!html.includes('COMPASS_LEARN'));
 assert(html.includes('10 m'));assert(html.includes('Vertical input stays neutral'));
 const safe={online:true,vehicle_fresh:true,armed:false,mode:'MANUAL',sa_active:true,esc_commanded_on:false,mission_claimed:false,autonomous:false};
 const data={controller:safe,job:{busy:false,phase:'idle'},navigation:{streams:{vn_calibration:{fresh:true,data:{phase:'idle'}}}}};
 let p=await page(data);assert(p.e.prepare.disabled);p.e.surfaceConfirm.checked=true;p.refresh();assert(!p.e.prepare.disabled);
 await p.e.prepare.onclick();assert.equal(p.posts[0].alignment_mode,'gps');assert(p.posts[0].surface_confirmed);
 p.expire();assert(p.e.prepare.disabled);assert(p.e.alignAuto.disabled);
 data.controller={...safe,armed:true,sa_active:false,esc_commanded_on:true};
 data.job={busy:true,action:'prepare',alignment_mode:'gps',phase:'preparing'};
 data.navigation.streams.bridge={fresh:true,data:{inputs:{vn:true,dvl:true},gps_eligible:true,gps_admission:{qualified:true}}};
 p=await page(data);assert(p.e.alignAuto.disabled);p.e.clearanceConfirm.checked=true;p.refresh();assert(!p.e.alignAuto.disabled);
 await p.e.alignAuto.onclick();assert.equal(p.posts[0].action,'align_auto');assert.equal(p.posts[0].max_distance_m,10);
 assert(!p.e.cancel.disabled);
 data.controller.mode='GUIDED';p=await page(data);p.e.clearanceConfirm.checked=true;p.refresh();assert(p.e.alignAuto.disabled);
 data.controller.mode='MANUAL';data.navigation.streams.bridge.data.profile_mismatches={COMPASS_LEARN:{actual:3,expected:0}};
 p=await page(data);p.e.clearanceConfirm.checked=true;p.refresh();assert(p.e.alignAuto.disabled);assert.match(p.e.alignmentStatus.textContent,/COMPASS_LEARN/);
 console.log('GPS track calibration GUI tests passed');
})().catch(e=>{console.error(e);process.exit(1)});
