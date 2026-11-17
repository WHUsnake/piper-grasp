import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {KTX2Loader} from 'three/addons/loaders/KTX2Loader.js';
import {HDRLoader} from 'three/addons/loaders/HDRLoader.js';
const $=id=>document.getElementById(id),txt=(id,v)=>{if($(id).textContent!==v)$(id).textContent=v;};
let renderer;
try { renderer=new THREE.WebGLRenderer({antialias:true}); }
catch (error) {
  console.warn('3D view unavailable:', error);
  document.body.dataset.webgl = 'unavailable';
  $('loading').remove();
  const notice=document.createElement('div');
  notice.id='webglFallback';
  notice.setAttribute('role','status');
  notice.textContent='浏览器无法创建 WebGL 三维画面。抓取 RGB-D 面板仍可使用；三维场景请在支持 WebGL 的浏览器打开。';
  $('viewport').appendChild(notice);
  txt('status','三维画面不可用 · 抓取面板可用');
}
if (renderer) {
renderer.setSize(innerWidth,innerHeight);renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.05;renderer.localClippingEnabled=true;renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFShadowMap;$('viewport').appendChild(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color('#e9eeec');const camera=new THREE.PerspectiveCamera(44,innerWidth/innerHeight,.05,100);camera.up.set(0,0,1);camera.position.set(-14,-18,22);const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,0,.4);controls.enableDamping=true;controls.maxPolarAngle=Math.PI*.49;controls.minDistance=.8;controls.maxDistance=45;
const hemi=new THREE.HemisphereLight(0xffffff,0x76836b,1.4);hemi.up.set(0,0,1);hemi.position.set(0,0,10);scene.add(hemi);const sun=new THREE.DirectionalLight(0xfff2db,2.4);sun.position.set(-5,-8,14);sun.castShadow=true;sun.shadow.mapSize.set(2048,2048);Object.assign(sun.shadow.camera,{left:-10,right:10,top:10,bottom:-10,near:.1,far:40});sun.shadow.normalBias=.015;scene.add(sun);scene.add(new THREE.DirectionalLight(0xdce9ff,.6));
const ktx=new KTX2Loader().setTranscoderPath('/vendor/examples/jsm/libs/basis/').setWorkerLimit(2).detectSupport(renderer),loader=new GLTFLoader().setKTX2Loader(ktx);
let labelHeight=.5,minSpeed=.1,maxSpeed=2,locomotionReady=true,idleStatus='实时模式 · 按住方向键或右下按钮行走';
let controlInstance=null;
let ready=false,speed=.8,view='whole',lastFollow=null,manifest,live=null,latestRequest=0;const keys=new Set(),bodyNodes=new Map(),staticObjects=[],pins=[],root=new THREE.Vector3(),q=new THREE.Quaternion(),clip=new THREE.Plane(new THREE.Vector3(0,0,-1),2.65);
const roomNames={'living room':'客厅',bedroom:'卧室',kitchen:'厨房',bathroom:'卫生间','other room':'其他房间',familyroom:'家庭起居区',closet:'储物间','entryway/foyer/lobby':'玄关',office:'书房',hallway:'走廊','dining room':'餐厅','laundryroom/mudroom':'洗衣房'};
function fail(message){let overlay=$('loading');if(!overlay){overlay=document.createElement('div');overlay.id='loading';overlay.innerHTML='<div class="loading-card"><h2>画面暂时中断</h2><p id="loadText"></p><button class="retry" onclick="location.reload()">重新加载画面</button></div>';document.body.appendChild(overlay);}overlay.style.opacity='1';txt('loadText',message);$('retryLoad')?.removeAttribute('hidden');}
window.addEventListener('error',e=>{if(e.error){console.error(e.error);fail('页面异常：'+e.message);}});window.addEventListener('unhandledrejection',e=>{console.error(e.reason);fail('资源或连接异常，请重新加载。');});renderer.domElement.addEventListener('webglcontextlost',e=>{e.preventDefault();keys.clear();fail('图形连接已中断，请重新加载画面。');});
async function json(url,options={},timeout=1800){const r=await fetch(url,{...options,cache:'no-store',signal:AbortSignal.timeout(timeout)});if(!r.ok)throw Error('HTTP '+r.status);return r.json();}
async function control(velocity=[0,0,0],paused=false,reset=false){try{await json('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({velocity,paused,reset,instance:controlInstance})});}catch(e){keys.clear();txt('liveState','连接重试中 · 已停止移动指令');}}
function setView(v){camera.clearViewOffset();view=v;lastFollow=null;for(const id of ['whole','follow','top'])$(id).classList.toggle('selected',id===v);if(v==='whole'){camera.position.set(-8,-29,34);controls.target.set(0,0,.4);fitWhole();}else if(v==='bottle'){const item=manifest?.placed_objects?.find(o=>o.body_name==='bottle');if(!item)return;const position=live?.bodies?.[item.body_id]?.slice(0,3)||item.position;const point=new THREE.Vector3(...position).add(new THREE.Vector3(0,0,.015));$('roof').checked=false;$('cutaway').checked=true;updateClip();controls.target.copy(point);camera.position.copy(point).add(new THREE.Vector3(1.5,-1.8,1.3));}else if(v==='top'){camera.position.set(0,-.01,Math.max(23,(manifest?.source_bounds[1][0]-manifest?.source_bounds[0][0]||23)*1.5));controls.target.set(0,0,0);}else{$('roof').checked=false;$('cutaway').checked=true;updateClip();controls.target.copy(root);camera.position.copy(root).add(new THREE.Vector3(2.2,-2.8,3.2));lastFollow=root.clone();}controls.update();}
function fitWhole(){
 if(!staticObjects.length)return;camera.clearViewOffset();const box=new THREE.Box3();for(const object of staticObjects){object.updateMatrixWorld(true);box.union(new THREE.Box3().setFromObject(object));}
 const size=box.getSize(new THREE.Vector3()),center=box.getCenter(new THREE.Vector3());center.z=.7;
 const direction=(size.x>=size.y?new THREE.Vector3(.12,-.85,1):new THREE.Vector3(-.85,.12,1)).normalize();
 const corners=[];for(const x of [box.min.x,box.max.x])for(const y of [box.min.y,box.max.y])for(const z of [box.min.z,box.max.z])corners.push(new THREE.Vector3(x,y,z));
 let distance=Math.max(size.x,size.y)*1.4;controls.target.copy(center);
 for(let i=0;i<5;i++){camera.position.copy(center).addScaledVector(direction,distance);camera.lookAt(center);camera.updateMatrixWorld();let factor=1;for(const v of corners){const p=v.clone().project(camera);factor=Math.max(factor,Math.abs(p.x)/.87,Math.abs(p.y)/.57);}if(factor<=1.005)break;distance*=factor*1.02;}
 controls.maxDistance=Math.max(110,distance*1.5);if(innerWidth<800)camera.setViewOffset(innerWidth,innerHeight,0,-innerHeight*.08,innerWidth,innerHeight);
}
let surfaceClipHeight=.75;
function updateClip(){clip.constant=$('cutaway').checked?surfaceClipHeight:2.65;for(const object of staticObjects)object.traverse(o=>{if(o.isMesh)for(const m of Array.isArray(o.material)?o.material:[o.material]){m.clippingPlanes=$('roof').checked?[]:[clip];m.needsUpdate=true;}});}
for(const id of ['whole','follow','top'])$(id).onclick=()=>setView(id);$('bottleView').onclick=()=>setView('bottle');$('roof').onchange=()=>{if($('roof').checked)$('cutaway').checked=false;updateClip();};$('cutaway').onchange=()=>{if($('cutaway').checked)$('roof').checked=false;updateClip();};
addEventListener('lab:surface-view',event=>{
 const surface=event.detail;
 if(!surface?.table_bounds_world)return;
 const [xmin,xmax,ymin,ymax]=surface.table_bounds_world;
 const point=new THREE.Vector3((xmin+xmax)/2,(ymin+ymax)/2,surface.tabletop_height_m);
 view='surface';lastFollow=null;camera.clearViewOffset();
 for(const id of ['whole','follow','top'])$(id).classList.remove('selected');
 surfaceClipHeight=Math.max(.75,surface.tabletop_height_m+.35);
 $('roof').checked=false;$('cutaway').checked=true;updateClip();
 controls.target.copy(point);camera.position.copy(point).add(new THREE.Vector3(1.5,-1.8,1.5));controls.update();
});
function pose(a,b=a,f=0){if(!a?.bodies)return;for(const [i,node] of bodyNodes){const x=a.bodies[i],y=b.bodies[i]||x;if(!x)continue;node.position.set(...x.slice(0,3)).lerp(new THREE.Vector3(...y.slice(0,3)),f);node.quaternion.set(x[4],x[5],x[6],x[3]);q.set(y[4],y[5],y[6],y[3]);node.quaternion.slerp(q,f);}root.set(...a.pos).lerp(new THREE.Vector3(...b.pos),f);}
// Buffer display poses only; commands and authoritative physics are unchanged.
const poseFrames=[];
function acceptPose(data){const at=performance.now();if(live&&data.t<live.t)poseFrames.length=0;live=data;latestRequest=at;poseFrames.push({at,state:data});if(poseFrames.length>8)poseFrames.shift();}
function smoothPose(now){const at=now-80;while(poseFrames.length>2&&poseFrames[1].at<=at)poseFrames.shift();if(poseFrames.length<2){pose(live);return;}const a=poseFrames[0],b=poseFrames[1],f=Math.max(0,Math.min(1,(at-a.at)/Math.max(1,b.at-a.at)));pose(a.state,b.state,f);}
$('reset').onclick=()=>{if(confirm('确认重置整个仿真场景？机器人和物体都会回到初始位置。')){keys.clear();control([0,0,0],false,true);}};
function setSpeed(v){speed=Math.max(minSpeed,Math.min(maxSpeed,Number(v)));$('speed').value=speed;txt('speedValue',speed.toFixed(1)+' m/s');for(const b of document.querySelectorAll('[data-speed]'))b.classList.toggle('selected',Math.abs(Number(b.dataset.speed)-speed)<.01);}$('speed').oninput=e=>setSpeed(e.target.value);for(const b of document.querySelectorAll('[data-speed]'))b.onclick=()=>setSpeed(b.dataset.speed);
addEventListener('keydown',e=>{if(!ready||!locomotionReady||e.target.closest('input,select,textarea,[contenteditable]'))return;if(e.code.startsWith('Arrow')){e.preventDefault();keys.add(e.code);}if(/^Digit[123]$/.test(e.code)){const preset=document.querySelectorAll('[data-speed]')[Number(e.code.slice(-1))-1];if(preset)setSpeed(preset.dataset.speed);}});addEventListener('keyup',e=>keys.delete(e.code));addEventListener('blur',()=>{keys.clear();if(ready)control([0,0,0],false);});document.addEventListener('visibilitychange',()=>{if(document.hidden){keys.clear();control([0,0,0],false);}});window.addEventListener('pagehide',()=>navigator.sendBeacon('/api/control',new Blob([JSON.stringify({velocity:[0,0,0],paused:true,instance:controlInstance})],{type:'application/json'})));
for(const b of document.querySelectorAll('[data-drive]')){b.onpointerdown=e=>{e.preventDefault();if(!locomotionReady)return;keys.add(b.dataset.drive);b.classList.add('held');b.setPointerCapture(e.pointerId);};for(const name of ['pointerup','pointercancel','lostpointercapture'])b.addEventListener(name,()=>{keys.delete(b.dataset.drive);b.classList.remove('held');});}
let sending=false;setInterval(async()=>{if(!ready||sending)return;sending=true;try{await control([speed*(Number(keys.has('ArrowUp'))-Number(keys.has('ArrowDown'))),0,Math.min(1.5,Math.max(.6,speed))*(Number(keys.has('ArrowLeft'))-Number(keys.has('ArrowRight')))],false);}finally{sending=false;}},100);
async function poll(){if(ready){try{const data=await json('/api/state',{},1400);if(data.ready){acceptPose(data);txt('liveState',`实际 ${data.speed.toFixed(2)} m/s · 物理 ${data.rate.toFixed(2)}×`);}else txt('liveState',data.failed?'物理进程异常':'物理初始化中…');}catch(e){keys.clear();txt('liveState','连接重试中 · 请松开按键');}}setTimeout(poll,50);}poll();
async function init(){try{const robotInfo=await json('/api/robot');controlInstance=robotInfo.control_instance;locomotionReady=robotInfo.motion_ready;if(robotInfo.name==='Go2 + Piper')$('reset').style.display='none';document.title='Robot Lab · 三卧场景与 '+robotInfo.name;document.querySelector('.brand h1 span').textContent='/ '+robotInfo.name;document.querySelector('.brand p').textContent='HSSD / PARTNR · '+robotInfo.description;$('label').firstChild.nodeValue=robotInfo.name+' ';txt('follow','跟随');if(robotInfo.motion_mode==='moe-cts'){labelHeight=1.1;minSpeed=.1;maxSpeed=1.5;$('speed').min=.1;$('speed').max=1.5;for(const [i,b] of [...document.querySelectorAll('[data-speed]')].entries())b.dataset.speed=[.3,.8,1.5][i];setSpeed(.8);idleStatus='Go2 + Piper · MoE-CTS 行走 · 机械臂保持姿态';txt('modeHint','方向键行走与转向 · 机械臂保持姿态');document.querySelector('.live-mode').textContent='MoE-CTS 行走';document.querySelector('.metrics .badge').textContent='MoE-CTS · 82,500 次迭代';document.querySelector('.metrics p').textContent='MoE-CTS驱动腿部行走，Piper机械臂独立保持姿态。';}
manifest=await json('/manifest.json',{},10000);$('bottleView').hidden=!manifest.placed_objects?.some(o=>o.body_name==='bottle');
const cache=new Map();function load(url){if(!cache.has(url))cache.set(url,loader.loadAsync(url));return cache.get(url);}let done=0;const queue=[...manifest.visuals];async function worker(){while(queue.length){const item=queue.shift(),gltf=await load(item.url),group=new THREE.Group();group.matrix.fromArray(item.matrix);group.matrixAutoUpdate=false;const object=gltf.scene.clone(true);group.add(object);object.traverse(o=>{if(o.isMesh){o.receiveShadow=true;o.castShadow=false;o.userData.category=item.category;for(const m of Array.isArray(o.material)?o.material:[o.material])m.side=THREE.DoubleSide;}});staticObjects.push(group);scene.add(group);txt('loadText',`模型与纹理已就绪 ${++done} / ${manifest.visuals.length}`);}}await Promise.all([worker(),worker(),worker(),worker()]);
const dog=await load('/robot.glb?v='+encodeURIComponent(robotInfo.visual_sha256));dog.scene.traverse(o=>{if(/^body_\d+$/.test(o.name))bodyNodes.set(Number(o.name.slice(5)),o);if(o.isMesh){o.castShadow=true;o.receiveShadow=true;}});scene.add(dog.scene);
for(const room of manifest.rooms){const poly=room.polygon,c=poly.reduce((a,p)=>[a[0]+p[0]/poly.length,a[1]+p[1]/poly.length],[0,0]),el=document.createElement('span');el.className='room-pin';el.textContent=(roomNames[room.label]||room.label)+(room.name.includes('.')?' '+(Number(room.name.split('.').at(-1))+1):'');$('roomPins').appendChild(el);pins.push({el,point:new THREE.Vector3(c[0],c[1],.12)});}
for(const surface of manifest.manipulation_surfaces||[]){const [xmin,xmax,ymin,ymax]=surface.table_bounds_world;const el=document.createElement('button');el.className='surface-pin';el.textContent=surface.name+' · 测试瓶';el.setAttribute('aria-label','查看'+surface.name);el.onclick=()=>dispatchEvent(new CustomEvent('lab:surface-view',{detail:surface}));$('roomPins').appendChild(el);pins.push({el,point:new THREE.Vector3((xmin+xmax)/2,(ymin+ymax)/2,surface.tabletop_height_m+.30),surface:true});}
await control([0,0,0],false,false);const initial=await json('/api/state');if(initial.ready){acceptPose(initial);pose(initial);}updateClip();setView('whole');ready=true;$('loading').remove();ktx.dispose();txt('status',idleStatus);new HDRLoader().load('/environment.hdr',t=>{t.mapping=THREE.EquirectangularReflectionMapping;const pm=new THREE.PMREMGenerator(renderer);scene.environment=pm.fromEquirectangular(t).texture;scene.environmentIntensity=.6;hemi.intensity=.8;sun.intensity=2;t.dispose();pm.dispose();},undefined,e=>console.warn('HDR unavailable',e));}catch(e){console.error(e);fail('场景未能载入：'+e.message);}}
const projected=new THREE.Vector3();function pin(el,p){projected.copy(p).project(camera);el.hidden=projected.z>1||Math.abs(projected.x)>1||Math.abs(projected.y)>1;el.style.left=Math.round((projected.x*.5+.5)*innerWidth)+'px';el.style.top=Math.round((-.5*projected.y+.5)*innerHeight)+'px';}
let last=performance.now(),lastFPS=last,count=0;function render(){requestAnimationFrame(render);const now=performance.now(),dt=Math.min((now-last)/1000,.2);last=now;if(ready){if(live){smoothPose(now);if(now-latestRequest>2000)keys.clear();}if(view==='follow'){if(lastFollow){const delta=root.clone().sub(lastFollow);camera.position.add(delta);controls.target.add(delta);}lastFollow=root.clone();}pin($('label'),root.clone().add(new THREE.Vector3(0,0,labelHeight)));$('label').style.opacity='1';for(const p of pins){pin(p.el,p.point);if(!p.surface&&!$('roomLabels').checked)p.el.hidden=true;}txt('status',keys.size?'实时控制中 · 放开按键减速':idleStatus);}controls.update();renderer.render(scene,camera);count++;if(now-lastFPS>1500){txt('fps',Math.round(count*1000/(now-lastFPS))+' FPS · 浏览器显示');lastFPS=now;count=0;}}
addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);if(view==='whole')fitWhole();});setTimeout(()=>{if(!ready){$('retryLoad')?.removeAttribute('hidden');}},30000);render();init();
}
if (!renderer) {
  const keys = new Set();
  let instance = null, sending = false;
  fetch('/api/robot', {cache:'no-store'}).then(response=>response.json()).then(robot=>{
    instance = robot.control_instance;
    $('webglFallback').textContent = '三维画面不可用；RGB-D 抓取和右下角方向按钮仍可使用。按住移动，松开停止。';
  }).catch(error=>console.warn('Drive control unavailable:', error));
  const stop = () => keys.clear();
  addEventListener('keydown',event=>{
    if (!event.code.startsWith('Arrow') || event.repeat ||
        event.target.closest('input,select,textarea,[contenteditable]') ||
        (event.target.closest('#rgbCanvas') && document.body.dataset.graspHolding !== 'true')) return;
    event.preventDefault();keys.add(event.code);
  });
  addEventListener('keyup',event=>keys.delete(event.code));
  addEventListener('blur',stop);
  document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
  for (const button of document.querySelectorAll('[data-drive]')) {
    button.onpointerdown = event=>{event.preventDefault();keys.add(button.dataset.drive);button.classList.add('held');button.setPointerCapture(event.pointerId);};
    for (const name of ['pointerup','pointercancel','lostpointercapture'])
      button.addEventListener(name,()=>{keys.delete(button.dataset.drive);button.classList.remove('held');});
  }
  setInterval(async()=>{
    if (!instance || sending || document.hidden) return;
    sending = true;
    try {
      const velocity=[.3*(Number(keys.has('ArrowUp'))-Number(keys.has('ArrowDown'))),0,
                      .6*(Number(keys.has('ArrowLeft'))-Number(keys.has('ArrowRight')))];
      const response=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({instance,velocity,paused:false,reset:false})});
      if (!response.ok) keys.clear();
    } catch(error) { keys.clear(); }
    finally { sending=false; }
  },100);
}
