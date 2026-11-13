const STATES = ['IDLE','VALIDATING','CONFIGURING','WAITING FOR INPUT','WAITING FOR AUTO','RUNNING','PAUSED','CANCELING','SUCCEEDED','FAILED','REJECTED'];
const SOURCES = ['STOPPED / ZERO','MANUAL','AUTO','SBUS'];
const HEALTH = ['UNKNOWN','OK','DEGRADED','ERROR','STALE','DISABLED'];
const PATTERNS = ['消灯','点灯','点滅'];
function lampColors(value){return [[1,'緑'],[2,'黄'],[4,'赤']].filter(([bit])=>value&bit).map(([,label])=>label).join('＋')||'消灯';}
function initPanel(context) {
  const root = context.panelElement;
  root.replaceChildren();
  root.style.cssText='height:100%;overflow:auto;box-sizing:border-box;padding:16px;background:#101820;color:#edf3fa;font:14px system-ui';
  const heading=document.createElement('h3'); heading.textContent='NJORD Mission';
  const select=document.createElement('select'); select.setAttribute('aria-label','Task');
  const refresh=document.createElement('button'); refresh.textContent='タスク一覧を更新';
  const start=document.createElement('button'); start.textContent='AUTOで開始';
  const stop=document.createElement('button'); stop.textContent='停止・MANUALへ'; stop.style.background='#b91c1c';stop.style.color='white';
  const mode=document.createElement('p'),state=document.createElement('p'),detail=document.createElement('p'),health=document.createElement('p'),lamp=document.createElement('p'),feedback=document.createElement('p');
  const progress=document.createElement('progress'); progress.max=1; progress.style.width='100%';
  for(const b of [select,refresh,start,stop]) b.style.cssText+=';margin:4px;padding:8px';
  root.append(heading,select,refresh,start,stop,mode,state,progress,detail,health,lamp,feedback);
  let mission=null,control=null,lastMission=0,lastControl=0,lastHealth=0,lastLamp=0,busy=false,tasks=[],summary=null,lampMessage=null,lastExecution='',lastPathSafety=0,pathSafe=true,requirements=null,lastRequirements=0,lastSerial=0,serialStatus='';
  health.style.whiteSpace='pre-wrap';
  const fresh=t=>t>0&&Date.now()-t<3000;
  function render(){
    const ready=fresh(lastMission)&&fresh(lastControl);
    const task=tasks.find(t=>t.task_id===select.value);
    start.disabled=busy||!ready||!task||task.availability>1||mission.state!==0;
    stop.disabled=busy||!mission?.execution_id||![1,2,3,4,5,6,7].includes(mission.state);
    mode.textContent=fresh(lastControl)?`要求: ${control.requested_mode===1?'AUTO':'MANUAL'} ｜ AUTO許可: ${control.auto_permitted?'可':'不可'} ｜ 実効出力: ${SOURCES[control.effective_source]??'UNKNOWN'}${control.emergency_stop?' ｜ 緊急停止':''}`:'制御状態: STALE / 未受信';
    mode.style.color=ready?(control.emergency_stop?'#f87171':control.effective_source===2?'#4ade80':'#fbbf24'):'#f87171';
    state.textContent=fresh(lastMission)?`${mission.task_id||'--'} · ${STATES[mission.state]??'UNKNOWN'} · ${mission.stage||'--'}`:'Mission: STALE / 未受信';
    progress.value=fresh(lastMission)?Math.max(0,Math.min(1,mission.progress||0)):0;
    const reasons=[...(control?.inhibit_reasons||[]),...(mission?.inhibit_reasons||[])];
    const pathReason=mission?.task_id==='task2'&&mission.state===5&&lastPathSafety?(fresh(lastPathSafety)?(pathSafe?'':'避航経路の安全条件未成立（停止）'):'避航経路の判定: STALE'):'';
    detail.textContent=[mission?.message||'',pathReason,...new Set(reasons)].filter(Boolean).join(' / ');
    const checks=[['自己位置','local_odometry','require_localization_heartbeat'],['GNSS受信','gnss','require_rtk_fix'],['driver heartbeat','driver_heartbeat','require_driver_heartbeat']];
    const checkLines=checks.map(([label,name,key])=>{
      const signal=fresh(lastHealth)?summary.signals?.find(s=>s.name===name):null;
      const gate=fresh(lastRequirements)&&typeof requirements?.requirements?.[key]==='boolean'?(requirements.requirements[key]?'ON':'OFF'):'不明';
      const info=signal?`${HEALTH[signal.state]??'UNKNOWN'} · age=${Number(signal.age_sec).toFixed(1)}s · ${Number(signal.measured_frequency_hz).toFixed(1)}Hz · ${signal.message||''}`:(fresh(lastHealth)?'監視信号が未登録':'STALE / Health未受信');
      return `${label}: ${info} ｜ AUTO禁止条件: ${gate}`;
    });
    health.textContent=[fresh(lastHealth)?`Health: ${HEALTH[summary.summary_state]??'UNKNOWN'} · ${summary.message||''}`:'Health: STALE / 未受信',`AUTO条件の適用先: ${fresh(lastRequirements)?(requirements?.policy||'未選択（共通条件）'):'STALE / 未受信'}`,...checkLines,'※GNSS受信監視は更新頻度の判定です。RTK FIX品質の判定とは異なります。',`serial writer: ${fresh(lastSerial)?serialStatus:'STALE / 未受信（simでは実シリアル未起動）'}`].join('\n');
    lamp.style.color=fresh(lastLamp)?(lampMessage.color&4?'#f87171':lampMessage.color&2?'#fbbf24':lampMessage.color&1?'#4ade80':'#edf3fa'):'#f87171';
    lamp.textContent=fresh(lastLamp)?`状態灯（指令）: ${lampColors(lampMessage.color)} · ${PATTERNS[lampMessage.pattern]??'不明'}${lampMessage.pattern===2?` (${lampMessage.period}秒周期)`:''} / ${lampMessage.reason||''}`:'状態灯: STALE / 未受信';
  }
  async function call(service,request){busy=true;render();try{const response=await context.callService(service,request);feedback.textContent=response?.message||'応答を受信';return response;}catch(e){feedback.textContent=`操作失敗: ${e.message||e}`;return null;}finally{busy=false;render();}}
  async function list(){const response=await call('/mission/list_tasks',{});if(!response)return;tasks=response.tasks||[];const old=select.value;select.replaceChildren();for(const task of tasks){const o=document.createElement('option');o.value=task.task_id;o.textContent=`${task.display_name}${task.availability>1?' (利用不可)':''}`;o.disabled=task.availability>1;o.title=task.reason||'';select.append(o);}if(tasks.some(t=>t.task_id===old))select.value=old;render();}
  refresh.onclick=list; select.onchange=render;
  start.onclick=()=>call('/mission/start_task',{task_id:select.value,request_auto_mode:true,dry_run:false,request_id:`foxglove-${Date.now()}-${Math.random().toString(16).slice(2)}`});
  stop.onclick=()=>call('/mission/stop_task',{execution_id:mission.execution_id,return_to_manual:true});
  const baseTopics=['/mission/status','/mission/last_result','/control/state','/health/state','/alert_lamp/command','/control/requirements','/micon/thruster_command_status'];
  let task2Subscribed=false;
  function updateSubscriptions(){const active=mission?.task_id==='task2'&&mission.state>=1&&mission.state<=7;if(active===task2Subscribed)return;task2Subscribed=active;lastPathSafety=0;context.subscribe([...baseTopics,...(active?['/mission/task2/path_safe']:[])].map(topic=>({topic})));}
  context.subscribe(baseTopics.map(topic=>({topic})));
  context.watch('currentFrame');
  context.onRender=(rs,done)=>{try{for(const e of rs.currentFrame||[]){const m=e.message||{};if(e.topic==='/control/requirements'){try{requirements=JSON.parse(m.data);lastRequirements=Date.now();}catch{requirements=null;lastRequirements=0;}}else if(e.topic==='/micon/thruster_command_status'){serialStatus=m.data;lastSerial=Date.now();}else if(e.topic==='/mission/task2/path_safe'){pathSafe=m.data;lastPathSafety=Date.now();}else if(e.topic==='/mission/last_result'){feedback.textContent=`直近の結果 ${m.task_id}: ${STATES[m.state]} · ${m.message}`;}else if(e.topic==='/mission/status'){mission=m;updateSubscriptions();lastMission=Date.now();if(m.execution_id&&m.execution_id!==lastExecution){lastExecution=m.execution_id;feedback.textContent='';}if([8,9,10].includes(m.state))feedback.textContent=`${STATES[m.state]}: ${m.message}`;}else if(e.topic==='/control/state'){control=m;lastControl=Date.now();}else if(e.topic==='/health/state'){summary=m;lastHealth=Date.now();}else if(e.topic==='/alert_lamp/command'){lampMessage=m;lastLamp=Date.now();}}render();}finally{done();}};
  const timer=setInterval(render,500);render();list();
  return()=>{clearInterval(timer);root.replaceChildren();};
}
function activate(extensionContext){extensionContext.registerPanel({name:'mission-control',initPanel});}
module.exports={activate};
