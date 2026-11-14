const STATES = ['IDLE','VALIDATING','CONFIGURING','WAITING FOR INPUT','WAITING FOR AUTO','RUNNING','PAUSED','CANCELING','SUCCEEDED','FAILED','REJECTED'];
const SOURCES = ['停止（ZERO）','手動操縦','自律航行','SBUS'];
const HEALTH = ['UNKNOWN','OK','DEGRADED','ERROR','STALE','DISABLED'];
const PATTERNS = ['消灯','点灯','点滅'];
function lampColors(value) {
  return [[1,'緑'],[2,'黄'],[4,'赤']].filter(([bit])=>value&bit).map(([,label])=>label).join('＋')||'消灯';
}
function initPanel(context) {
  const root = context.panelElement;
  root.replaceChildren();
  root.style.cssText = 'height:100%;overflow:auto;box-sizing:border-box;padding:12px;background:#101820;color:#edf3fa;font:14px/1.5 system-ui';
  function element(tag, text, style='') {
    const node = document.createElement(tag);
    if (text !== undefined) node.textContent = text;
    node.style.cssText = style;
    return node;
  }
  function section(title) {
    const box = element('section', undefined, 'margin:0 0 16px');
    box.append(element('h3', title, 'font-size:14px;margin:0 0 8px;color:#a9c3d7'));
    root.append(box);
    return box;
  }
  function table(parent, headings) {
    const wrap = element('div', undefined, 'overflow-x:auto;border:1px solid #385064;border-radius:6px');
    const node = element('table', undefined, 'width:100%;border-collapse:collapse;table-layout:fixed');
    if (headings) {
      const head = element('thead');
      const row = element('tr');
      for (const title of headings) row.append(element('th', title, 'text-align:left;padding:8px;background:#223342;font-weight:600'));
      head.append(row); node.append(head);
      node.style.minWidth = '440px';
    }
    const body = element('tbody'); node.append(body); wrap.append(node); parent.append(wrap);
    return body;
  }
  function row(body, label, columns=1) {
    const tr = element('tr');
    tr.append(element('th', label, 'width:30%;text-align:left;vertical-align:top;padding:8px;border-bottom:1px solid #304354;font-weight:500;color:#bbcad7;overflow-wrap:anywhere'));
    const values = [];
    for (let i=0;i<columns;i++) {
      const td = element('td', '', 'padding:8px;vertical-align:top;border-bottom:1px solid #304354;white-space:pre-wrap;overflow-wrap:anywhere');
      tr.append(td); values.push(td);
    }
    body.append(tr);
    return columns === 1 ? values[0] : values;
  }
  function set(node, text, tone) {
    node.textContent = text;
    node.style.color = tone === 'bad' ? '#fca5a5' : tone === 'ok' ? '#86efac' : tone === 'wait' ? '#fde68a' : '#edf3fa';
  }
  const actions = section('NJORD Mission Control');
  const toolbar = element('div', undefined, 'display:flex;flex-wrap:wrap;gap:8px;align-items:center');
  const select = element('select'); select.setAttribute('aria-label','Task');
  const refresh = element('button', 'タスク一覧を更新');
  const start = element('button', '準備してAUTO開始');
  const stop = element('button', '停止・MANUALへ');
  for (const control of [select,refresh,start,stop]) {
    control.style.cssText = 'padding:8px 10px;border:1px solid #536c80;border-radius:5px;background:#223342;color:#edf3fa;font:inherit';
  }
  select.style.flex = '1 1 160px';
  start.style.background = '#14532d'; stop.style.background = '#881337';
  toolbar.append(select,refresh,start,stop); actions.append(toolbar);
  const explanation = element('p', '開始操作はタスク準備とAUTO要求を送ります。安全条件を満たすまでは自律出力されません。', 'font-size:12px;color:#bbcad7;margin:8px 0');
  const startHint = element('p', '', 'margin:8px 0;white-space:pre-wrap');
  actions.append(explanation,startHint);
  const tuningBox=section('速度ゲイン調整');
  tuningBox.append(element('p','開始すると船体へ速度指令が出ます。AUTO許可と計測値を確認してから開始してください。','font-size:12px;color:#fde68a;margin:6px 0'));
  const tuningInputs=element('div',undefined,'display:flex;flex-wrap:wrap;gap:10px;margin:8px 0');
  function numericInput(labelText,step) { const wrap=element('label',undefined,'display:flex;flex-direction:column;gap:3px;min-width:130px;flex:1'); wrap.append(element('span',labelText,'font-size:12px;color:#bbcad7')); const input=element('input'); input.type='number'; input.step=step; input.min='0'; input.value=''; input.style.cssText='padding:8px;border:1px solid #536c80;border-radius:5px;background:#17232d;color:#edf3fa;font:inherit'; wrap.append(input); tuningInputs.append(wrap); return input; }
  const targetInput=numericInput('目標速度 (m/s)','0.01');
  const maxSpeedInput=numericInput('速度上限 (m/s)','0.01');
  const thrustCapInput=numericInput('推力上限 (N / 基)','1');
  const tuneToolbar=element('div',undefined,'display:flex;flex-wrap:wrap;gap:8px');
  const tuneStart=element('button','自動調整を開始'), tunePause=element('button','一時停止して手動へ'), tuneResume=element('button','AUTOで再開'), tuneStop=element('button','終了して設定を復元');
  for(const b of [tuneStart,tunePause,tuneResume,tuneStop]) b.style.cssText='padding:8px 10px;border:1px solid #536c80;border-radius:5px;background:#223342;color:#edf3fa;font:inherit';
  tuneStart.style.background='#14532d'; tunePause.style.background='#854d0e'; tuneStop.style.background='#881337'; tuneToolbar.append(tuneStart,tunePause,tuneResume,tuneStop);
  const tuneStatus=element('p','調整器の状態: 未受信','margin:8px 0;white-space:pre-wrap');
  const tuneMetrics=element('p','実測速度: — · 推力 [FR, FL, RR, RL]: —','font-size:12px;color:#bbcad7;margin:6px 0;white-space:pre-wrap');
  tuningBox.append(tuningInputs,tuneToolbar,tuneStatus,tuneMetrics);

  const statusBody = table(section('運転状態'));
  const requested = row(statusBody, '要求モード');
  const permission = row(statusBody, 'AUTO出力許可');
  const output = row(statusBody, '実際の出力源');
  const emergency = row(statusBody, '非常停止');
  const missionState = row(statusBody, 'Mission');
  const stage = row(statusBody, '実行段階');
  const detail = row(statusBody, '待機・停止理由');
  const progressBox = section('進行状況');
  const progress = element('progress', undefined, 'width:100%;height:12px'); progress.max=1;
  const progressLabel = element('div', '0%'); progressBox.append(progress,progressLabel);

  const checkBox = section('安全条件と受信状態');
  const conditionBody = table(checkBox, ['項目','状態','受信情報','AUTO条件']);
  const checks = [
    ['自己位置','local_odometry','require_localization_heartbeat'],
    ['GNSS受信','gnss','require_rtk_fix'],
    ['driver heartbeat','driver_heartbeat','require_driver_heartbeat'],
    ['地上局heartbeat','ground_station_heartbeat','require_ground_station'],
  ].map(([label,name,key])=>({name,key,cells:row(conditionBody,label,3)}));
  const readyChecks = [
    {label:'Nav2',key:'nav2_ready',gate:'require_nav2_ready'},
    {label:'Task準備',key:'task_ready',gate:'require_task_ready'},
  ].map(check=>({...check,cells:row(conditionBody,check.label,3)}));
  const healthSummary = element('p', '', 'font-size:12px;margin:8px 0;white-space:pre-wrap');
  const policyLabel = element('p', '', 'font-size:12px;color:#bbcad7;margin:8px 0');
  checkBox.append(healthSummary,policyLabel,element('p','GNSS受信の状態は更新頻度の監視です。RTK FIX品質を保証する表示ではありません。','font-size:12px;color:#bbcad7'));
  const hardwareBody = table(section('機器への指令'));
  const lamp = row(hardwareBody,'状態灯（指令）');
  const serial = row(hardwareBody,'serial writer');
  const feedback = element('p', '', 'white-space:pre-wrap;overflow-wrap:anywhere;margin:8px 0');
  section('操作結果・直近の完了結果').append(feedback);

  let mission=null,control=null,summary=null,lampMessage=null,requirements=null;
  let lastMission=0,lastControl=0,lastHealth=0,lastLamp=0,lastRequirements=0,lastSerial=0;
  let busy=false,tasks=[],lastExecution='',lastPathSafety=0,pathSafe=false,serialStatus='',pendingStart=false;
  let tuningMessage='',lastTuning=0,tuningOdom=null,lastTuningOdom=0,tuningForces=null,lastTuningForces=0;
  const fresh = t=>t>0&&Date.now()-t<3000;
  const active = ()=>mission?.execution_id&&mission.state>=1&&mission.state<=7;
  function startBlockers() {
    if (!fresh(lastMission)||!fresh(lastControl)||!fresh(lastRequirements)) return ['Mission・Control・安全条件の受信待ち / STALE'];
    if (control.emergency_stop) return ['非常停止が有効です'];
    if (requirements.preparation_permitted !== true) {
      return requirements.preparation_inhibit_reasons?.length ? requirements.preparation_inhibit_reasons : ['開始条件が未確認です（SafetySupervisorの更新が必要）'];
    }
    // These conditions cannot be solved by preparing a task or Nav2 profile.
    const hardFaults = (control.inhibit_reason_codes||[]).some(code=>[1,5,6,7,8,9,11].includes(code));
    return hardFaults ? (control.inhibit_reasons?.length ? control.inhibit_reasons : ['安全条件により禁止されています']) : [];
  }
  function render() {
    const task = tasks.find(t=>t.task_id===select.value);
    const blockers = startBlockers();
    const tuningPhase=fresh(lastTuning)?tuningMessage.split(/\s+/,1)[0]:'STALE';
    const tunerActive=['PREPARING','RUNNING','PAUSED','RESTORING','COMPLETE'].includes(tuningPhase);
    const targetValue=Number(targetInput.value),maxSpeedValue=Number(maxSpeedInput.value),capValue=Number(thrustCapInput.value);
    const tuningConfigValid=Number.isFinite(targetValue)&&targetValue>0&&Number.isFinite(maxSpeedValue)&&maxSpeedValue>=targetValue&&Number.isFinite(capValue)&&capValue>0;
    tuneStart.disabled=busy||tunerActive||!tuningConfigValid||!fresh(lastControl)||control.requested_mode!==1||control.auto_permitted!==true||control.emergency_stop===true;
    tunePause.disabled=busy||tuningPhase!=='RUNNING';
    tuneResume.disabled=busy||tuningPhase!=='PAUSED'||!fresh(lastControl)||control.emergency_stop===true;
    tuneStop.disabled=busy||!tunerActive||!fresh(lastControl);
    for(const input of [targetInput,maxSpeedInput,thrustCapInput]) input.disabled=busy||tunerActive;
    for(const b of [tuneStart,tunePause,tuneResume,tuneStop]) { b.style.opacity=b.disabled?'0.45':'1'; b.style.cursor=b.disabled?'not-allowed':'pointer'; }
    set(tuneStatus,`${fresh(lastTuning)?tuningMessage:'調整器の状態: 未受信 / STALE'}\n${fresh(lastControl)?`モード=${control.requested_mode===1?'AUTO':'MANUAL'} · AUTO許可=${control.auto_permitted?'あり':'なし'}`:'制御状態: 未受信'}`,tuningPhase==='RUNNING'?'ok':tuningPhase==='PAUSED'?'wait':undefined);
    const actualSpeed=fresh(lastTuningOdom)&&tuningOdom?`${Number(tuningOdom.twist?.twist?.linear?.x||0).toFixed(3)} m/s`:'— / STALE';
    const actualForces=fresh(lastTuningForces)&&tuningForces?`[${tuningForces.map(v=>Number(v).toFixed(1)).join(', ')}] N`:'— / STALE';
    tuneMetrics.textContent=`実測速度: ${actualSpeed} · 推力 [FR, FL, RR, RL]: ${actualForces}`;
    start.disabled = busy||pendingStart||blockers.length>0||!task||task.availability>1||mission?.state!==0;
    stop.disabled = busy||(!active()&&!(fresh(lastControl)&&control.requested_mode===1));
    select.disabled = busy||pendingStart||Boolean(active());
    refresh.disabled = busy;
    for (const button of [start,stop,refresh]) {
      button.style.opacity = button.disabled ? '0.45' : '1';
      button.style.cursor = button.disabled ? 'not-allowed' : 'pointer';
    }
    const hint = pendingStart ? '開始受付済み：Missionの状態更新待ち' : active() ? 'タスク実行中：停止操作でMANUALへ戻せます' : blockers.length ? `開始できません：${blockers.join(' / ')}` : '準備開始可能：Task・Nav2・Task固有の条件は準備後に判定します';
    set(startHint,hint,blockers.length?'bad':'wait');
    set(requested,fresh(lastControl)?(control.requested_mode===1?'AUTO要求中':'MANUAL要求中'):'STALE / 未受信');
    const unprepared = fresh(lastControl)&&fresh(lastRequirements)&&!requirements.policy&&requirements.preparation_permitted===true&&!control.emergency_stop;
    set(permission,!fresh(lastControl)?'STALE / 未受信':control.auto_permitted?'許可':unprepared?'準備前（有効なTaskなし）':'禁止',control?.auto_permitted?'ok':'wait');
    set(output,fresh(lastControl)?(SOURCES[control.effective_source]||'UNKNOWN'):'STALE / 未受信',control?.effective_source===2?'ok':'wait');
    set(emergency,fresh(lastControl)?(control.emergency_stop?'有効 — 出力停止':'解除'):'STALE / 未受信',!fresh(lastControl)||control?.emergency_stop?'bad':'ok');
    set(missionState,fresh(lastMission)?`${mission.task_id||'未選択'} · ${STATES[mission.state]||'UNKNOWN'}`:'STALE / 未受信');
    set(stage,fresh(lastMission)?(mission.stage||'—'):'STALE / 未受信');
    const pathReason = mission?.task_id==='task2'&&mission.state===5 ? (!fresh(lastPathSafety)?'避航経路の判定: STALE / 未受信':!pathSafe?'避航経路の安全条件未成立（停止）':'') : '';
    const reasons = [...new Set([...(fresh(lastControl)?control.inhibit_reasons||[]:[]),...(fresh(lastMission)?mission.inhibit_reasons||[]:[])])];
    set(detail,[fresh(lastMission)?mission.message||'':'Mission未受信',pathReason,...reasons].filter(Boolean).join('\n')||'なし');
    progress.value = fresh(lastMission)?Math.max(0,Math.min(1,mission.progress||0)):0;
    progressLabel.textContent = `${Math.round(progress.value*100)}%`;
    for (const {name,key,cells} of checks) {
      const signal = fresh(lastHealth)?summary?.signals?.find(s=>s.name===name):null;
      const gate = fresh(lastRequirements)?requirements.requirements?.[key]:undefined;
      set(cells[0],signal?(HEALTH[signal.state]||'UNKNOWN'):(fresh(lastHealth)?'監視信号が未登録':'STALE / 未受信'),signal?.state===1?'ok':'wait');
      set(cells[1],signal?`age=${Number(signal.age_sec).toFixed(1)}s\n${Number(signal.measured_frequency_hz).toFixed(1)}Hz\n${signal.message||''}`:'—');
      set(cells[2],typeof gate==='boolean'?(gate?'適用':'監視のみ'):'未受信',gate?'wait':undefined);
    }
    for (const check of readyChecks) {
      const value = fresh(lastRequirements)?requirements[check.key]:undefined;
      const gate = fresh(lastRequirements)?requirements.requirements?.[check.gate]:undefined;
      set(check.cells[0],typeof value==='boolean'?(value?'READY':'未準備'):'未受信',value?'ok':'wait');
      set(check.cells[1],'タスク準備後にも再判定');
      set(check.cells[2],typeof gate==='boolean'?(gate?'適用':'監視のみ'):'未受信');
    }
    set(healthSummary,fresh(lastHealth)?`Health: ${HEALTH[summary.summary_state]||'UNKNOWN'} · ${summary.message||''}`:'Health: STALE / 未受信');
    policyLabel.textContent = `AUTO条件の適用先: ${fresh(lastRequirements)?requirements.policy||'共通条件（Task準備前）':'STALE / 未受信'}`;
    set(serial,fresh(lastSerial)?serialStatus:'未受信 / STALE（simでは実シリアル未起動）');
    set(lamp,fresh(lastLamp)?`${lampColors(lampMessage.color)} · ${PATTERNS[lampMessage.pattern]||'不明'}${lampMessage.pattern===2?` (${lampMessage.period}秒周期)`:''}\n${lampMessage.reason||''}`:'STALE / 未受信',lampMessage?.color&4?'bad':'wait');
  }
  async function call(service,request) {
    busy=true;render();
    try {
      const response=await context.callService(service,request);
      feedback.textContent=response?.message||'応答を受信';
      return response;
    } catch(e) {feedback.textContent=`操作失敗: ${e.message||e}`;return null;}
    finally {busy=false;render();}
  }
  async function list() {
    const response=await call('/mission/list_tasks',{});
    if(!response)return;
    tasks=response.tasks||[];
    const old=select.value; select.replaceChildren();
    for(const task of tasks) {
      const option=element('option',`${task.display_name}${task.availability>1?' (利用不可)':''}`);
      option.value=task.task_id; option.disabled=task.availability>1; option.title=task.reason||''; select.append(option);
    }
    if(tasks.some(t=>t.task_id===old))select.value=old;
    render();
  }
  refresh.onclick=list;select.onchange=render;
  start.onclick=async()=>{
    render();if(start.disabled)return;
    pendingStart=true;
    const response=await call('/mission/start_task',{task_id:select.value,request_auto_mode:true,dry_run:false,request_id:`foxglove-${Date.now()}-${Math.random().toString(16).slice(2)}`});
    if(!response?.accepted)pendingStart=false;
    render();
  };
  stop.onclick=()=>{
    render();if(stop.disabled)return;
    if(active())return call('/mission/stop_task',{execution_id:mission.execution_id,return_to_manual:true});
    return call('/control/set_mode',{requested_mode:0});
  };
  const waitForControl=async predicate=>{for(let i=0;i<30;i++){if(fresh(lastControl)&&predicate(control))return true;await new Promise(resolve=>setTimeout(resolve,100));}return false;};
  const modeRequest=requested_mode=>call('/control/set_mode',{requested_mode,request_id:`foxglove-tuning-${Date.now()}`});
  tuneStart.onclick=async()=>{
    render(); if(tuneStart.disabled)return;
    const parameters=[['target_speed_mps',Number(targetInput.value)],['max_test_speed_mps',Number(maxSpeedInput.value)],['test_max_thruster_thrust_n',Number(thrustCapInput.value)]].map(([name,value])=>({name,value:{type:3,double_value:value}}));
    const configured=await call('/velocity_autotuner/set_parameters_atomically',{parameters});
    if(!configured?.result?.successful){feedback.textContent=configured?.result?.reason||'調整条件を設定できませんでした';return;}
    const result=await call('/velocity_tuning/start',{}); if(!result?.success)feedback.textContent=result?.message||'調整を開始できませんでした';
  };
  tunePause.onclick=async()=>{ if(tunePause.disabled)return; const result=await call('/velocity_tuning/pause',{}); if(!result?.success)return; await modeRequest(0); };
  tuneResume.onclick=async()=>{ if(tuneResume.disabled)return; const mode=await modeRequest(1); if(!mode?.accepted)return; if(!await waitForControl(state=>state.requested_mode===1&&state.auto_permitted&&!state.emergency_stop)){feedback.textContent='AUTO許可を確認できません。調整は再開していません。';return;} const result=await call('/velocity_tuning/resume',{}); if(!result?.success)feedback.textContent=result?.message||'再開できませんでした'; };
  tuneStop.onclick=async()=>{ if(tuneStop.disabled)return; const mode=await modeRequest(0); if(!mode?.accepted)return; if(!await waitForControl(state=>state.requested_mode===0)){feedback.textContent='MANUALへの切替を確認できません。設定復元は実行していません。';return;} const result=await call('/velocity_tuning/stop',{}); if(!result?.success)feedback.textContent=result?.message||'停止・復元できませんでした'; };
  const baseTopics=['/mission/status','/mission/last_result','/control/state','/health/state','/alert_lamp/command','/control/requirements','/micon/thruster_command_status','/velocity_tuning/status','/odometry/filtered/local','/debug/current_force'];
  let task2Subscribed=false;
  function updateSubscriptions() {
    const enabled=mission?.task_id==='task2'&&mission.state>=1&&mission.state<=7;
    if(enabled===task2Subscribed)return;
    task2Subscribed=enabled;lastPathSafety=0;
    context.subscribe([...baseTopics,...(enabled?['/mission/task2/path_safe']:[])].map(topic=>({topic})));
  }
  context.subscribe(baseTopics.map(topic=>({topic})));context.watch('currentFrame');
  context.onRender=(rs,done)=>{
    try {
      for(const event of rs.currentFrame||[]) {
        const message=event.message||{};
        switch(event.topic) {
          case '/control/requirements':
            try {requirements=JSON.parse(message.data);lastRequirements=Date.now();}
            catch {requirements=null;lastRequirements=0;}
            break;
          case '/micon/thruster_command_status':serialStatus=message.data;lastSerial=Date.now();break;
          case '/mission/task2/path_safe':pathSafe=message.data;lastPathSafety=Date.now();break;
          case '/velocity_tuning/status':tuningMessage=message.data;lastTuning=Date.now();break;
          case '/odometry/filtered/local':tuningOdom=message;lastTuningOdom=Date.now();break;
          case '/debug/current_force':tuningForces=message.data;lastTuningForces=Date.now();break;
          case '/mission/last_result':pendingStart=false;feedback.textContent=`直近の結果 ${message.task_id}: ${STATES[message.state]} · ${message.message}`;break;
          case '/mission/status':
            mission=message;updateSubscriptions();lastMission=Date.now();
            if(message.state!==0)pendingStart=false;
            if(message.execution_id&&message.execution_id!==lastExecution){lastExecution=message.execution_id;feedback.textContent='';}
            if([8,9,10].includes(message.state))feedback.textContent=`${STATES[message.state]}: ${message.message}`;
            break;
          case '/control/state':control=message;lastControl=Date.now();break;
          case '/health/state':summary=message;lastHealth=Date.now();break;
          case '/alert_lamp/command':lampMessage=message;lastLamp=Date.now();break;
        }
      }
      render();
    } finally {done();}
  };
  const timer=setInterval(render,500);render();list();
  return()=>{clearInterval(timer);root.replaceChildren();};
}
function activate(extensionContext){extensionContext.registerPanel({name:'mission-control',initPanel});}
module.exports={activate};
