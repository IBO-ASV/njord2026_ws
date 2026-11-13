const test=require('node:test');const assert=require('node:assert/strict');
const {activate}=require('../src/extension.js');
test('Mission controls use task discovery, execution IDs and freshness',async()=>{
 const elements=[];let timer;let time=10000;const originalNow=Date.now;Date.now=()=>time;
 class Element{constructor(tag){this.tag=tag;this.style={};this.children=[];this.value='';elements.push(this);}setAttribute(){}replaceChildren(){this.children=[];}append(...items){this.children.push(...items);if(this.tag==='select'&&!this.value)this.value=items[0]?.value||'';}}
 global.document={createElement:tag=>new Element(tag)};
 const originalInterval=global.setInterval;global.setInterval=f=>{timer=f;return 1;};const originalClear=global.clearInterval;global.clearInterval=()=>{};
 let panel;activate({registerPanel:p=>panel=p});const calls=[];
 const subscriptions=[];
 const context={panelElement:new Element('root'),subscribe:topics=>subscriptions.push(topics.map(t=>t.topic)),watch:()=>{},callService:async(service,request)=>{calls.push([service,request]);return service==='/mission/list_tasks'?{tasks:[{task_id:'task1',display_name:'Task1',availability:1}]}:{accepted:true,message:'accepted'};}};
 let cleanup;
 try{
  cleanup=panel.initPanel(context);await new Promise(resolve=>setImmediate(resolve));
  assert(!subscriptions.at(-1).includes('/mission/task2/path_safe'));
  const start=elements.find(e=>e.textContent==='AUTOで開始');const stop=elements.find(e=>e.textContent==='停止・MANUALへ');assert.equal(start.disabled,true);
  const render=(mission,control)=>context.onRender({currentFrame:[{topic:'/mission/status',message:mission},{topic:'/control/state',message:control}]},()=>{});
  render({state:0},{requested_mode:0,effective_source:0,auto_permitted:false});assert.equal(start.disabled,false);
  await start.onclick();assert.equal(calls.at(-1)[0],'/mission/start_task');assert.equal(calls.at(-1)[1].task_id,'task1');assert.equal(calls.at(-1)[1].request_auto_mode,true);
  render({state:5,execution_id:'execution-123',task_id:'task1',progress:.5},{requested_mode:1,effective_source:0,auto_permitted:false,inhibit_reasons:['GNSS stale']});
  assert.equal(start.disabled,true);assert.equal(stop.disabled,false);await stop.onclick();assert.equal(calls.at(-1)[1].execution_id,'execution-123');
  context.onRender({currentFrame:[{topic:'/alert_lamp/command',message:{color:6,pattern:2,period:.5,reason:'sensor not ready'}},{topic:'/health/state',message:{summary_state:3,message:'GNSS stale'}}]},()=>{});
  assert(elements.some(e=>String(e.textContent).includes('黄＋赤 · 点滅 (0.5秒周期)')));
  assert(elements.some(e=>String(e.textContent).includes('Health: ERROR · GNSS stale')));
  context.onRender({currentFrame:[
   {topic:'/control/requirements',message:{data:JSON.stringify({policy:'task1',requirements:{require_localization_heartbeat:true,require_rtk_fix:false,require_driver_heartbeat:false}})}},
   {topic:'/health/state',message:{summary_state:3,message:'sensor stale',signals:[{name:'local_odometry',state:4,age_sec:2.1,measured_frequency_hz:0,message:'no odometry'}]}},
   {topic:'/micon/thruster_command_status',message:{data:'thrust command stale: timeout 0.5 s'}}
  ]},()=>{});
  assert(elements.some(e=>String(e.textContent).includes('自己位置: STALE')&&String(e.textContent).includes('age=2.1s')&&String(e.textContent).includes('AUTO禁止条件: ON')));
  assert(elements.some(e=>String(e.textContent).includes('GNSS受信: 監視信号が未登録 ｜ AUTO禁止条件: OFF')));
  assert(elements.some(e=>String(e.textContent).includes('driver heartbeat: 監視信号が未登録')));
  assert(elements.some(e=>String(e.textContent).includes('thrust command stale: timeout 0.5 s')));
  context.onRender({currentFrame:[{topic:'/mission/status',message:{state:5,execution_id:'execution-2',task_id:'task2'}},{topic:'/mission/task2/path_safe',message:{data:false}}]},()=>{});
  assert(elements.some(e=>String(e.textContent).includes('避航経路の安全条件未成立')));
  assert(subscriptions.at(-1).includes('/mission/task2/path_safe'));
  render({state:5,task_id:'task1'},{requested_mode:1,effective_source:0,auto_permitted:false});
  assert(!subscriptions.at(-1).includes('/mission/task2/path_safe'));
  render({state:5,task_id:'task2'},{requested_mode:1,effective_source:0,auto_permitted:false});
  assert(subscriptions.at(-1).includes('/mission/task2/path_safe'));
  render({state:8,task_id:'task2'},{requested_mode:0,effective_source:0,auto_permitted:false});
  assert(!subscriptions.at(-1).includes('/mission/task2/path_safe'));
  time+=4000;timer();assert.equal(start.disabled,true);assert(elements.some(e=>String(e.textContent).includes('STALE')));
  context.onRender({currentFrame:[{topic:'/mission/last_result',message:{task_id:'task1',state:9,message:'navigation failed'}}]},()=>{});
  assert(elements.some(e=>String(e.textContent).includes('FAILED')));
 }finally{cleanup?.();Date.now=originalNow;global.setInterval=originalInterval;global.clearInterval=originalClear;delete global.document;}
});
