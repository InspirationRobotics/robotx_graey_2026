const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync('tools/calibration.html','utf8');
const code=html.match(/<script>([\s\S]*?)<\/script>/)[1];
async function render(state){
 const elements={},posts=[];let now=100;
 const el=id=>elements[id]??={checked:false,disabled:false,textContent:'',value:0};
 const ctx={document:{getElementById:el},performance:{now:()=>now},setTimeout:()=>{},setInterval:()=>{},
 fetch:async(url,options)=>{if(options){posts.push(JSON.parse(options.body));return{ok:true,json:async()=>({ok:true})}}return{ok:true,json:async()=>state}}};
 vm.runInNewContext(code,ctx);await new Promise(setImmediate);
 return {elements,posts,expire:()=>{now=5000;el('calConfirm').onchange()}};
}
(async()=>{
 const safe={online:true,vehicle_fresh:true,armed:false,mode:'MANUAL',sa_active:true,esc_commanded_on:false,mission_claimed:false,autonomous:false};
 let page=await render({controller:safe,job:{phase:'idle',busy:false},navigation:{streams:{vn_calibration:{fresh:true,data:{phase:'idle',faces:[0,0,0,0,0,0]}}}}});
 let e=page.elements;assert(e.calibrate.disabled);assert(e.prepare.disabled);
 e.calConfirm.checked=true;e.calConfirm.onchange();assert(!e.calibrate.disabled);
 e.surfaceConfirm.checked=true;e.surfaceConfirm.onchange();assert(!e.prepare.disabled);
 await e.prepare.onclick();assert.equal(page.posts[0].action,'prepare');assert(page.posts[0].surface_confirmed);
 page.expire();assert(e.calibrate.disabled);assert(e.prepare.disabled);assert.match(e.status.textContent,/stale/);
 page=await render({controller:{...safe,armed:true},job:{phase:'verify',busy:true,message:'Stop, disarm and engage kill'},navigation:{streams:{}}});
 assert(page.elements.calibrate.disabled);assert(page.elements.prepare.disabled);assert(!page.elements.cancel.disabled);
 assert.match(page.elements.status.textContent,/disarm/);
 page=await render({controller:safe,job:{busy:true,phase:'collecting'},navigation:{streams:{vn_calibration:{fresh:true,data:{phase:'collecting',remaining_s:60,faces:[5,5,0,2,0,1],reason:'Rotate all faces'}}}}});
 assert.equal(page.elements.progress.value,60);assert.match(page.elements.coverage.textContent,/5 \/ 5 \/ 0/);
 page=await render({controller:safe,job:{phase:'preparing'},navigation:{streams:{bridge:{fresh:true,data:{gps_admission:{qualified:false,reason:'GPS qualifying',last_rejection:'GPS displacement disagrees with DVL motion'},gps_observation:{accuracy:3.5,speed_accuracy_mps:.3,speed_mps:.1}}}}}});
 assert.match(page.elements.gpsQuality.textContent,/3.50 m/);assert.match(page.elements.gpsQuality.textContent,/displacement/);
 page=await render({controller:safe,job:{phase:'idle'},navigation:{streams:{vn_calibration:{fresh:true,data:{phase:'idle'}}}}});
 e=page.elements;e.alignmentMode.value='known';e.surfaceConfirm.checked=true;e.alignmentMode.onchange();assert(e.prepare.disabled);
 e.trueHeading.value='90';e.headingConfirm.checked=true;e.headingConfirm.onchange();assert(!e.prepare.disabled);
 await e.prepare.onclick();assert.equal(page.posts[0].alignment_mode,'known');assert.equal(page.posts[0].true_heading_deg,90);assert(page.posts[0].heading_confirmed);
 assert(html.includes('not an independent compass reference'));assert(html.includes('not save flash'));
 assert(html.includes('armed MANUAL'));assert(html.includes('COMPASS_LEARN=3'));
 assert(html.includes('No button or disarm is required for this passive capture'));
 page=await render({controller:safe,job:{phase:'idle'},navigation:{streams:{bridge:{fresh:true,data:{compass_north:{phase:'idle'}}}}}});
 e=page.elements;assert(!e.compassStart.disabled);assert(!html.includes('id="compassConfirm"'));
 await e.compassStart.onclick();assert.equal(page.posts[0].action,'compass_start');assert(page.posts[0].confirmed);assert(!page.posts[0].surface_confirmed);
 const compassState={controller:{...safe,armed:true,sa_active:false,esc_commanded_on:true},job:{phase:'idle'},navigation:{streams:{bridge:{fresh:true,data:{compass_north:{phase:'idle'}}}}}};
 page=await render(compassState);assert(!page.elements.compassStart.disabled);assert(page.elements.calibrate.disabled);
 await page.elements.compassStart.onclick();assert.equal(page.posts[0].action,'compass_start');
 page=await render({...compassState,navigation:{streams:{bridge:{fresh:true,data:{compass_north:{phase:'learning'}}}}}});
 assert(!page.elements.compassCancel.disabled);assert(page.elements.compassStart.disabled);
 await page.elements.compassCancel.onclick();assert.equal(page.posts[0].action,'compass_cancel');
 const compassInstructions=html.split('<h2>2 · Compass Learn')[1].split('</section>')[0];
 assert(!compassInstructions.includes('switch ESC power off'));assert(!compassInstructions.includes('Select disarmed'));
 for(const change of [{armed:null},{mode:'GUIDED'},{vehicle_fresh:false},{mission_claimed:true}]){
  page=await render({...compassState,controller:{...compassState.controller,...change}});
  assert(page.elements.compassStart.disabled);assert(page.elements.compassStart.title.length>0);
 }
 page=await render({controller:safe,job:{phase:'idle'},navigation:{streams:{bridge:{fresh:true,data:{compass_north:{phase:'ready'},land_north_reference:{offset_deg:10}}}}}});
 e=page.elements;e.healthConfirm.checked=true;e.healthConfirm.onchange();assert(!e.healthPrepare.disabled);
 await e.healthPrepare.onclick();assert.equal(page.posts[0].action,'prepare');assert.equal(page.posts[0].alignment_mode,'retained');assert(page.posts[0].surface_confirmed);
 for(const [north,reference,reason] of [
  [{phase:'learning',finished_seen:false},null,'not confirmed'],
  [{phase:'learning',finished_seen:true},null,'hold level'],
  [{phase:'restoring'}, {offset_deg:10},'readbacks'],
  [{phase:'cancelled'},null,'No validated north']]){
  page=await render({controller:safe,job:{busy:false},navigation:{streams:{bridge:{fresh:true,data:{compass_north:north,land_north_reference:reference}}}}});
  page.elements.healthConfirm.checked=true;page.elements.healthConfirm.onchange();
  assert(page.elements.healthPrepare.disabled);assert(page.elements.healthPrepareReason.textContent.includes(reason));
 }
 page=await render({controller:safe,job:{phase:'idle'},navigation:{streams:{}}});
 e=page.elements;e.trueHeading.value='90';e.headingConfirm.checked=true;e.headingConfirm.onchange();
 assert(!e.alignLand.disabled);assert(!e.surfaceConfirm.checked);
 await e.alignLand.onclick();assert.equal(page.posts[0].action,'align_land');assert(page.posts[0].confirmed);assert(!page.posts[0].surface_confirmed);
 const recovery={controller:safe,job:{phase:'ready'},navigation:{streams:{bridge:{fresh:true,data:{fault:'DVL continuity lost; position uncertainty requires revalidation',heading_reference_validated:true,heading_continuity_lost:false,inputs:{vn:true,dvl:true}}}}}};
 page=await render(recovery);e=page.elements;assert(e.recoverDvl.disabled);e.surfaceConfirm.checked=true;e.surfaceConfirm.onchange();assert(!e.recoverDvl.disabled);
 await e.recoverDvl.onclick();assert.equal(page.posts[0].action,'recover_dvl');assert(page.posts[0].surface_confirmed);
 const auto={controller:{...safe,armed:true,sa_active:false,esc_commanded_on:true,mode:'MANUAL'},job:{action:'prepare',alignment_mode:'gps',busy:true},navigation:{streams:{bridge:{fresh:true,data:{gps_eligible:true,inputs:{vn:true,dvl:true},gps_admission:{qualified:true}}}}}};
 page=await render(auto);e=page.elements;assert(e.alignAuto.disabled);
 e.alignmentDistance.value='6';e.clearanceConfirm.checked=true;e.clearanceConfirm.onchange();assert(!e.alignAuto.disabled);
 await e.alignAuto.onclick();assert.equal(page.posts[0].action,'align_auto');assert.equal(page.posts[0].max_distance_m,6);assert(page.posts[0].clearance_confirmed);
 page=await render({...auto,controller:safe});e=page.elements;e.clearanceConfirm.checked=true;e.alignmentDistance.value='6';e.clearanceConfirm.onchange();assert(e.alignAuto.disabled);
 const blocked={controller:{...safe,sa_active:false,inhibit:'RF_LOST'},job:{action:'calibrate',phase:'collected',busy:false,alignment_mode:'gps'},navigation:{streams:{bridge:{fresh:true,data:{fault:'Navigation maintenance; Prepare required before reuse',inputs:{vn:true,dvl:false},profile_mismatches:{EK3_SRC1_POSZ:{actual:1,expected:6}}}}}}};
 page=await render(blocked);e=page.elements;assert(e.alignAuto.disabled);
 for(const reason of ['Prepare Navigation first','RF_LOST','DVL bottom lock','EK3_SRC1_POSZ: 1; required 6'])assert(e.alignmentStatus.textContent.includes(reason),reason);
 assert(e.alignAuto.title.includes('RF_LOST'));
 page=await render({...recovery,controller:{...safe,armed:true}});e=page.elements;e.surfaceConfirm.checked=true;e.surfaceConfirm.onchange();assert(e.recoverDvl.disabled);
 console.log('Calibration/recovery page confirmations, stale/armed controls, progress and instructions pass');
})().catch(error=>{console.error(error);process.exit(1)});
