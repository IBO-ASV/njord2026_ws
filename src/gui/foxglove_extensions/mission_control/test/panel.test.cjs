const test=require('node:test');const assert=require('node:assert/strict');
const {activate}=require('../src/extension.js');
test('tables distinguish preparation, AUTO intent, permission and output; unsafe/stale actions are blocked',async()=>{
 const elements=[];let timer;let time=10000;const originalNow=Date.now;Date.now=()=>time;
 class Element{constructor(tag){this.tag=tag;this.style={};this.children=[];this.value='';elements.push(this);}setAttribute(){}replaceChildren(){this.children=[];}append(...items){this.children.push(...items);if(this.tag==='select'&&!this.value)this.value=items[0]?.value||'';}}
 global.document={createElement:tag=>new Element(tag)};
 const originalInterval=global.setInterval;global.setInterval=f=>{timer=f;return 1;};const originalClear=global.clearInterval;global.clearInterval=()=>{};
 let panel;activate({registerPanel:p=>panel=p});const calls=[],subscriptions=[];
 const context={panelElement:new Element('root'),subscribe:topics=>subscriptions.push(topics.map(t=>t.topic)),watch:()=>{},callService:async(service,request)=>{calls.push([service,request]);return service==='/mission/list_tasks'?{tasks:[{task_id:'task1',display_name:'Task1',availability:1}]}:{accepted:true,message:'accepted'};}};
 const cell=(label,column=0)=>elements.find(e=>e.tag==='tr'&&e.children[0]?.textContent===label)?.children[column+1];
 const send=(topic,message)=>context.onRender({currentFrame:[{topic,message}]},()=>{});
 const control=(overrides={})=>send('/control/state',{requested_mode:0,effective_source:0,auto_permitted:false,emergency_stop:false,inhibit_reason_codes:[3],inhibit_reasons:['no active task control policy'],...overrides});
 const requirements=(overrides={})=>send('/control/requirements',{data:JSON.stringify({policy:'',requirements:{require_localization_heartbeat:true,require_rtk_fix:false,require_driver_heartbeat:false},preparation_permitted:true,preparation_inhibit_reasons:[],nav2_ready:true,task_ready:false,...overrides})});
 let cleanup;
 try {
  cleanup=panel.initPanel(context);await new Promise(resolve=>setImmediate(resolve));
  assert.equal(elements.filter(e=>e.tag==='table').length,3);
  assert(!subscriptions.at(-1).includes('/mission/task2/path_safe'));
  const start=elements.find(e=>e.textContent==='準備してAUTO開始');
  const stop=elements.find(e=>e.textContent==='停止・MANUALへ');
  assert.equal(start.disabled,true);
  send('/mission/status',{state:0});control();
  assert.equal(start.disabled,true); // Requirements must also be received.
  requirements();
  assert.equal(start.disabled,false);
  assert.match(cell('AUTO出力許可').textContent,/準備前/);
  assert.equal(cell('実際の出力源').textContent,'停止（ZERO）');

  control({emergency_stop:true});assert.equal(start.disabled,true);
  const before=calls.length;await start.onclick();assert.equal(calls.length,before);
  control();requirements({preparation_permitted:false,preparation_inhibit_reasons:['local_odometry is not OK']});
  assert.equal(start.disabled,true);await start.onclick();assert.equal(calls.length,before);
  assert(elements.some(e=>String(e.textContent).includes('local_odometry is not OK')));
  requirements();control({inhibit_reason_codes:[8],inhibit_reasons:['RTK not ready']});
  assert.equal(start.disabled,true); // Task policy faults also block, even with common prechecks OK.
  control();assert.equal(start.disabled,false);
  await start.onclick();assert.equal(calls.at(-1)[0],'/mission/start_task');
  assert.equal(calls.at(-1)[1].task_id,'task1');assert.equal(calls.at(-1)[1].request_auto_mode,true);
  const afterStart=calls.length;await start.onclick();assert.equal(calls.length,afterStart);

  send('/mission/status',{state:4,execution_id:'execution-123',task_id:'task1',progress:.5});
  requirements({policy:'task1'});control({requested_mode:1,inhibit_reason_codes:[7],inhibit_reasons:['GNSS stale']});
  assert.equal(cell('要求モード').textContent,'AUTO要求中');
  assert.equal(cell('AUTO出力許可').textContent,'禁止');
  assert.equal(cell('実際の出力源').textContent,'停止（ZERO）');
  assert.equal(stop.disabled,false);await stop.onclick();assert.equal(calls.at(-1)[1].execution_id,'execution-123');
  control({requested_mode:1,effective_source:2,auto_permitted:true,inhibit_reason_codes:[],inhibit_reasons:[]});
  assert.equal(cell('AUTO出力許可').textContent,'許可');assert.equal(cell('実際の出力源').textContent,'自律航行');

  send('/alert_lamp/command',{color:6,pattern:2,period:.5,reason:'sensor not ready'});
  assert.match(cell('状態灯（指令）').textContent,/黄＋赤 · 点滅 \(0.5秒周期\)\nsensor not ready/);
  send('/health/state',{summary_state:3,message:'sensor stale',signals:[{name:'local_odometry',state:4,age_sec:2.1,measured_frequency_hz:0,message:'no odometry'}]});
  assert.equal(cell('自己位置').textContent,'STALE');
  assert.match(cell('自己位置',1).textContent,/age=2.1s\n0.0Hz\nno odometry/);
  assert.equal(cell('自己位置',2).textContent,'適用');
  assert.equal(cell('GNSS受信').textContent,'監視信号が未登録');
  assert.equal(cell('GNSS受信',2).textContent,'監視のみ');
  send('/micon/thruster_command_status',{data:'thrust command stale: timeout 0.5 s'});
  assert.match(cell('serial writer').textContent,/thrust command stale/);

  send('/mission/status',{state:5,execution_id:'execution-2',task_id:'task2'});
  assert(subscriptions.at(-1).includes('/mission/task2/path_safe'));
  assert.match(cell('待機・停止理由').textContent,/避航経路の判定: STALE/);
  send('/mission/task2/path_safe',{data:false});
  assert.match(cell('待機・停止理由').textContent,/避航経路の安全条件未成立/);
  send('/mission/status',{state:5,task_id:'task1'});
  assert(!subscriptions.at(-1).includes('/mission/task2/path_safe'));
  send('/mission/status',{state:5,task_id:'task2'});
  assert(subscriptions.at(-1).includes('/mission/task2/path_safe'));
  send('/mission/status',{state:8,task_id:'task2'});
  assert(!subscriptions.at(-1).includes('/mission/task2/path_safe'));
  send('/mission/status',{state:0});control({requested_mode:1});
  assert.equal(stop.disabled,false);await stop.onclick();
  assert.deepEqual(calls.at(-1),['/control/set_mode',{requested_mode:0}]);
  time+=4000;timer();assert.equal(start.disabled,true);
  const staleCount=calls.length;await start.onclick();assert.equal(calls.length,staleCount);
  assert.equal(cell('AUTO出力許可').textContent,'STALE / 未受信');
  send('/mission/last_result',{task_id:'task1',state:9,message:'navigation failed'});
  assert(elements.some(e=>String(e.textContent).includes('FAILED')));
 } finally {cleanup?.();Date.now=originalNow;global.setInterval=originalInterval;global.clearInterval=originalClear;delete global.document;}
});
