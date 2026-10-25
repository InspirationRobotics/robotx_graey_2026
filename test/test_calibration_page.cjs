const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync('tools/calibration.html','utf8');
const code=html.match(/<script>([\s\S]*?)<\/script>/)[1];
async function page(state){
 const e={},posts=[];let now=100;
 const el=id=>e[id]??={checked:false,disabled:false,textContent:'',value:id==='alignmentMode'?'gps':id==='alignmentDistance'?5:''};
 const ctx={document:{getElementById:el},performance:{now:()=>now},setTimeout:()=>{},setInterval:()=>{},fetch:async(url,options)=>{
  if(options){posts.push(JSON.parse(options.body));return{ok:true,json:async()=>({ok:true})}}
  return{ok:true,json:async()=>state};}};
 vm.runInNewContext(code,ctx);await new Promise(setImmediate);
 return{e,posts,refresh:()=>e.surfaceConfirm.onchange(),expire:()=>{now=5000;e.surfaceConfirm.onchange()}};
}
(async()=>{
 assert(!html.includes('compassStart'));assert(!html.includes('COMPASS_LEARN'));
 assert(html.includes('Start Navigation'));
 const safe={online:true,vehicle_fresh:true,armed:false,mode:'MANUAL',sa_active:true,esc_commanded_on:false,mission_claimed:false,autonomous:false};
 const data={controller:safe,job:{busy:false,phase:'idle'},navigation:{streams:{vn_calibration:{fresh:true,data:{phase:'idle'}}}}};
 let p=await page(data);assert(p.e.prepare.disabled);p.e.surfaceConfirm.checked=true;p.refresh();assert(!p.e.prepare.disabled);
 await p.e.prepare.onclick();assert.equal(p.posts[0].alignment_mode,'gps');assert(p.posts[0].surface_confirmed);
 p.expire();assert(p.e.prepare.disabled);
 p=await page({...data,controller:{...safe,armed:true,sa_active:false,esc_commanded_on:true}});p.e.surfaceConfirm.checked=true;p.refresh();assert(!p.e.prepare.disabled);assert(p.e.calibrate.disabled);
 p=await page({...data,controller:{...safe,mission_claimed:true}});p.e.surfaceConfirm.checked=true;p.refresh();assert(p.e.prepare.disabled);
 assert(!html.includes('id="alignAuto"'));
 assert(html.includes('10 m track'));
 console.log('GPS track calibration GUI tests passed');
})().catch(e=>{console.error(e);process.exit(1)});
