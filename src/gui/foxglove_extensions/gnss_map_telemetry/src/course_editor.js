// The geographic course is distinct from Nav2's current global plan.
const COURSE_LINE_STYLE = {color:'#1644a6',weight:4,opacity:1,dashArray:'7 5',interactive:false};
const BUOY_CHOICES = [['RED','赤・左舷標識'],['GREEN','緑・右舷標識'],['N','北方位標識'],['E','東方位標識'],['S','南方位標識'],['W','西方位標識']];
function escapeMapText(value) {return String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
function createCourseEditor(context,L,map,root,onCourse) {
  const box=document.createElement('div');box.className='course-editor';root.appendChild(box);
  const controls=document.createElement('div');controls.className='course-editor-controls';
  const select=document.createElement('select');select.setAttribute('aria-label','編集するTask');
  const toggle=document.createElement('button');toggle.textContent='編集を開始';
  const add=document.createElement('button');add.textContent='ブイを追加';
  const save=document.createElement('button');save.textContent='YAMLへ保存';
  const reload=document.createElement('button');reload.textContent='変更を破棄・再読込';
  const info=document.createElement('div');info.className='course-editor-info';
  const choice=document.createElement('div');choice.className='course-editor-choice';
  controls.append(select,toggle,add,save,reload);box.append(controls,info,choice);
  L.DomEvent.disableClickPropagation(box);L.DomEvent.disableScrollPropagation(box);
  let course=null,draft=null,enabled=false,dirty=false,busy=false,adding=false,selectedBuoy=null,lastState=0;
  let courseLayers=[],route=null,signature='',conflict=false,notice='編集サーバーの受信待ち';
  const markers=[];
  const fresh=()=>lastState>0&&Date.now()-lastState<3000;
  const canEdit=()=>fresh()&&course?.editable===true&&!busy&&!conflict;
  function render() {
    toggle.disabled=!canEdit();toggle.textContent=enabled?'編集を終了':'編集を開始';
    add.disabled=!enabled||!canEdit();save.disabled=!dirty||!canEdit();
    reload.disabled=busy||!course;select.disabled=busy||dirty||enabled||!course||course.tasks.length===1;
    for(const marker of markers) {const action=enabled&&canEdit()?'enable':'disable';marker.dragging?.[action]();}
    const mode=course?.mode==='sim'?'SIM':'実機用';
    info.textContent=[course?`${mode} · ${course.task_id} · ${dirty?'未保存の変更あり':'保存内容を表示'}`:'',
      !fresh()?'編集サーバー未受信 / STALE':!course.editable?'編集ロック：MANUAL・Mission停止中のみ編集可能':'',
      adding?'地図をクリックしてブイを配置':enabled?'WP・ブイをドラッグ。ブイをクリックして種類を選択':'',notice,
      course?.source_file?`YAML: ${course.source_file}`:''].filter(Boolean).join('\n');
    if(!enabled||!canEdit()){choice.replaceChildren();selectedBuoy=null;adding=false;}
  }
  function rebuildRoute() {
    if(route)map.removeLayer(route);
    route=draft.waypoints.length>1?L.polyline(draft.waypoints.map(w=>[w.latitude,w.longitude]),COURSE_LINE_STYLE).addTo(map):null;
  }
  function draw() {
    courseLayers.forEach(layer=>map.removeLayer(layer));courseLayers=[];markers.length=0;choice.replaceChildren();
    if(!draft)return;
    draft.waypoints.forEach((point,index)=>makeMarker(point,false,index));
    draft.buoys.forEach((point,index)=>makeMarker(point,true,index));
    rebuildRoute();render();
  }
  function markDirty() {dirty=true;notice='変更はまだYAMLに保存されていません';render();}
  function makeMarker(point,buoy,index) {
    const kind=BUOY_CHOICES.find(([key])=>key===point.type);
    const color=point.type==='RED'?'#b91c1c':point.type==='GREEN'?'#166534':'#111827';
    const text=buoy?`${point.type} ブイ`:`${String(index+1).padStart(2,'0')} · ${point.name||point.id}`;
    const html=buoy?`<div class="map-buoy" style="background:${color}" title="${escapeMapText(kind?.[1]||point.type)}">${escapeMapText(point.type)}</div>`:
      `<div class="waypoint-pin">${index+1}<span class="waypoint-pin-leader"></span><span class="waypoint-pin-label">${escapeMapText(text)}</span></div>`;
    const marker=L.marker([point.latitude,point.longitude],{draggable:enabled&&canEdit(),interactive:true,
      icon:L.divIcon({className:'course-map-marker',iconSize:buoy?[32,32]:[220,24],iconAnchor:buoy?[16,16]:[12,12],html})}).addTo(map);
    marker.on('dragend',()=>{
      if(!enabled||!canEdit()){marker.setLatLng([point.latitude,point.longitude]);return;}
      const position=marker.getLatLng();point.latitude=position.lat;point.longitude=position.lng;
      rebuildRoute();markDirty();
    });
    if(buoy)marker.on('click',()=>{
      choice.replaceChildren();
      if(!enabled||!canEdit()){notice=`${kind?.[1]||point.type}：編集開始後に種類を変更できます`;render();return;}
      selectedBuoy=point.id;
      const label=document.createElement('span');label.textContent=`ブイ ${index+1} `;
      const types=document.createElement('select');types.setAttribute('aria-label','ブイの種類');
      for(const [value,title] of BUOY_CHOICES){const option=document.createElement('option');option.value=value;option.textContent=title;types.appendChild(option);}types.value=point.type;
      types.onchange=()=>{if(!canEdit())return;point.type=types.value;markDirty();draw();};
      const remove=document.createElement('button');remove.textContent='このブイを削除';
      remove.onclick=()=>{if(!canEdit())return;draft.buoys=draft.buoys.filter(b=>b.id!==selectedBuoy);markDirty();draw();};
      choice.append(label,types,remove);
    });
    markers.push(marker);courseLayers.push(marker);
  }
  async function transact(edit) {
    busy=true;render();
    try {
      const response=await context.callService('/course_editor/apply',{parameters:[{name:'edit',value:{type:4,string_value:JSON.stringify(edit)}}]});
      if(!response?.result?.successful)throw new Error(response?.result?.reason||'保存サーバーから応答がありません');
      notice=response.result.reason;
      return true;
    }catch(error){notice=`操作失敗：${error.message||error}`;return false;}
    finally{busy=false;render();}
  }
  toggle.onclick=()=>{if(!canEdit())return;enabled=!enabled;adding=false;draw();};
  add.onclick=()=>{if(enabled&&canEdit()){adding=true;notice='';render();}};
  map.on('click',event=>{
    if(!adding||!enabled||!canEdit())return;
    draft.buoys.push({id:`buoy-${Date.now()}-${Math.random().toString(16).slice(2)}`,type:'RED',latitude:event.latlng.lat,longitude:event.latlng.lng});
    adding=false;markDirty();draw();
  });
  save.onclick=async()=>{
    if(!dirty||!canEdit())return;
    const expected=course.revision;
    const ok=await transact({action:'save',task_id:course.task_id,revision:expected,waypoints:draft.waypoints,buoys:draft.buoys});
    if(ok){dirty=false;conflict=false;notice=course.requires_restart?'YAML保存済み。simを再起動すると経路・ブイへ反映されます':'YAML保存済み。次のMissionでwaypointを使用します。ブイは地図上の配置情報です';enabled=false;draw();}
  };
  reload.onclick=async()=>{
    if(!course||busy)return;
    dirty=false;conflict=false;enabled=false;
    draft=JSON.parse(JSON.stringify(course));draw();
    await transact({action:'load',task_id:course.task_id});
  };
  select.onchange=()=>transact({action:'load',task_id:select.value});
  function receive(message) {
    let next;try{next=JSON.parse(message.data);}catch{return;}
    lastState=Date.now();
    if(next.error||!Array.isArray(next.waypoints)||!Array.isArray(next.buoys)){notice=next.error||'コース情報が不正です';lastState=0;render();return;}
    if(dirty&&course&&(next.revision!==course.revision||next.task_id!==course.task_id)) {
      // Own successful save can arrive before the service reply. Do not discard
      // the draft; a reload after saving reconciles the authoritative revision.
      if(!busy){conflict=true;notice='保存先が別の操作で変更されました。変更を破棄・再読込してください';}
      course.editable=next.editable;render();return;
    }
    const changed=!course||next.revision!==course.revision||next.task_id!==course.task_id;
    course=next;
    if(!dirty&&(changed||!draft))draft=JSON.parse(JSON.stringify(course));
    if(changed) {
      select.replaceChildren();for(const id of course.tasks){const option=document.createElement('option');option.value=id;option.textContent=id;select.appendChild(option);}select.value=course.task_id;
      if(!dirty){enabled=false;draw();}
      const positions=course.waypoints.map(w=>[w.latitude,w.longitude]);
      const nextSignature=`${course.mode}:${course.task_id}`;
      if(signature!==nextSignature&&positions.length){map.fitBounds(L.latLngBounds(positions),{padding:[50,50],maxZoom:19,animate:false});signature=nextSignature;}
      notice=course.requires_restart?'sim編集内容は保存後、sim再起動で反映':'実機waypointは次のMissionから反映。ブイは地図用';
      onCourse?.(course);
    }
    render();
  }
  const timer=setInterval(render,500);render();
  return {receive,hasCourse:()=>Boolean(course),stats:()=>({task:course?.task_id,count:draft?.waypoints.length||0}),destroy:()=>{clearInterval(timer);courseLayers.forEach(layer=>map.removeLayer(layer));if(route)map.removeLayer(route);box.remove();}};
}
if(typeof module!=='undefined')module.exports={createCourseEditor,COURSE_LINE_STYLE,escapeMapText};
