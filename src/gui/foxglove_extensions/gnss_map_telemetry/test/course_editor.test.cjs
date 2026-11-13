const test=require('node:test');const assert=require('node:assert/strict');
const {createCourseEditor,COURSE_LINE_STYLE,escapeMapText}=require('../src/course_editor.js');
test('map drag, buoy type/add/delete, YAML save, conflicts and stale locks',async()=>{
 const elements=[],layers=[],calls=[];let tick;let now=10000;
 const saved={document:global.document,setInterval:global.setInterval,clearInterval:global.clearInterval,now:Date.now};
 class Element{constructor(tag){this.tag=tag;this.children=[];this.style={};this.value='';elements.push(this);}append(...items){this.children.push(...items);}appendChild(item){this.children.push(item);}replaceChildren(){this.children=[];}setAttribute(){}remove(){}}
 global.document={createElement:tag=>new Element(tag)};global.setInterval=f=>{tick=f;return 1};global.clearInterval=()=>{};Date.now=()=>now;
 const handlers={};const map={on:(event,f)=>handlers[event]=f,removeLayer:l=>{l.removed=true},fitBounds:()=>{}};
 const L={DomEvent:{disableClickPropagation:()=>{},disableScrollPropagation:()=>{}},latLngBounds:p=>p,divIcon:p=>p,
  marker:(position,options)=>{const m={position:{lat:position[0],lng:position[1]},options,events:{},dragging:{enable:()=>m.draggable=true,disable:()=>m.draggable=false},addTo:()=>m,on:(event,f)=>m.events[event]=f,getLatLng:()=>m.position,setLatLng:p=>m.position={lat:p[0],lng:p[1]}};layers.push(m);return m},
  polyline:(positions,style)=>{assert.equal(style.color,'#1644a6');assert.equal(style.opacity,1);return {addTo(){return this}}}};
 const context={callService:async(service,request)=>{calls.push([service,JSON.parse(request.parameters[0].value.string_value)]);return {result:{successful:true,reason:'saved'}}}};
 const initial={mode:'sim',task_id:'task1',tasks:['task1'],source_file:'/test/course.yaml',revision:'r1',requires_restart:true,editable:true,
  waypoints:[{id:'1',name:'GPS1',latitude:63,longitude:10},{id:'2',name:'GPS2',latitude:63.01,longitude:10.01}],
  buoys:[{id:'b1',type:'RED',latitude:63.005,longitude:10.005}]};
 const receive=data=>editor.receive({data:JSON.stringify(data)});
 const button=text=>elements.find(e=>e.textContent===text);
 const liveMarkers=()=>layers.filter(l=>!l.removed);
 let editor;
 try {
  editor=createCourseEditor(context,L,map,new Element('root'));
  assert.equal(button('編集を開始').disabled,true);
  receive(initial);assert.equal(editor.stats().count,2);
  assert.equal(button('編集を開始').disabled,false);button('編集を開始').onclick();
  assert(liveMarkers()[0].draggable);
  receive(initial);receive(initial); // Periodic state must preserve marker-to-draft references.
  const waypoint=liveMarkers()[0];waypoint.position={lat:63.001,lng:10.002};waypoint.events.dragend();
  assert.equal(button('YAMLへ保存').disabled,false);
  liveMarkers()[2].events.click();
  const choices=elements.filter(e=>e.tag==='select').at(-1);
  assert.equal(choices.children.length,6);choices.value='N';choices.onchange();
  button('ブイを追加').onclick();handlers.click({latlng:{lat:63.02,lng:10.02}});
  assert.equal(liveMarkers().length,4);
  liveMarkers().at(-1).events.click();button('このブイを削除').onclick();
  assert.equal(liveMarkers().length,3);
  await button('YAMLへ保存').onclick();
  assert.equal(calls.at(-1)[0],'/course_editor/apply');
  const edit=calls.at(-1)[1];assert.equal(edit.action,'save');assert.equal(edit.revision,'r1');
  assert.equal(edit.waypoints[0].latitude,63.001);assert.equal(edit.buoys[0].type,'N');
  assert(!('path' in edit));
  assert(elements.some(e=>String(e.textContent).includes('simを再起動')));
  receive({...initial,revision:'r2'});button('編集を開始').onclick();
  const moved=liveMarkers()[0];moved.position={lat:63.03,lng:10.03};moved.events.dragend();
  receive({...initial,revision:'external'});
  assert.equal(button('YAMLへ保存').disabled,true);
  const count=calls.length;await button('YAMLへ保存').onclick();assert.equal(calls.length,count);
  await button('変更を破棄・再読込').onclick();
  receive({...initial,revision:'external',editable:false});
  assert.equal(button('編集を開始').disabled,true);
  receive({...initial,revision:'external',editable:true});now+=4000;tick();
  assert.equal(button('編集を開始').disabled,true);
  assert(liveMarkers().every(m=>!m.draggable));
  assert.equal(COURSE_LINE_STYLE.weight,4);assert.equal(escapeMapText('<img>'),'&lt;img&gt;');
 } finally {editor?.destroy();Object.assign(global,{document:saved.document,setInterval:saved.setInterval,clearInterval:saved.clearInterval});Date.now=saved.now;}
});
