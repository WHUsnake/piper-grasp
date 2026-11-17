const element = id => document.getElementById(id);
const rgb = element('rgbCanvas'), depthCanvas = element('depthCanvas');
const colorContext = rgb.getContext('2d'), depthContext = depthCanvas.getContext('2d');
let frame = null, pixels = null, selfPixels = null, selection = null, instance = null;
let capturing = false, submitting = false, running = false, owner = null;
let captureQueued = false;
let rgbImage = null, depthImage = null, currentRun = null, displayedRun = null;
let connected = false, configured = false, supportsRestore = false, supportsPlace = false, supportsPlaceRetry = false, holding = false, lastOperation = null;
let pendingPose = null;
let frameCapturedAt = 0;
let suspectedLost = false;
let allowedActions = {};
let recoverableFault = false;
let screenRecorder = null, screenStream = null, screenChunks = [], screenTimer = null, screenStartedAt = 0;

function recordingStatus(message) {
  element('screenRecordStatus').textContent = message;
}

function recordingButtons() {
  element('screenRecordStart').disabled = Boolean(screenRecorder);
  element('screenRecordStop').disabled = !screenRecorder;
}

function finishRecording() {
  clearInterval(screenTimer);
  screenStream?.getTracks().forEach(track => track.stop());
  screenStream = null;
  const mime = screenRecorder?.mimeType || 'video/webm';
  screenRecorder = null;
  recordingButtons();
  if (!screenChunks.length) {
    recordingStatus('没有收到录像数据，请重新开始录制。');
    return;
  }
  const blob = new Blob(screenChunks, {type: mime});
  screenChunks = [];
  const filename = 'robot-lab-grasp-place-'+new Date().toISOString().replace(/[:.]/g, '-')+(mime.includes('mp4') ? '.mp4' : '.webm');
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = filename;
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 60000);
  recordingStatus(`录像已保存到浏览器下载目录：${filename}（${(blob.size/1048576).toFixed(1)} MB）`);
}

async function startRecording() {
  if (screenRecorder) return;
  if (!navigator.mediaDevices?.getDisplayMedia || typeof MediaRecorder === 'undefined') {
    recordingStatus('当前浏览器不支持屏幕录制，请使用 Firefox 或 Chrome。');
    return;
  }
  try {
    const stream = await navigator.mediaDevices.getDisplayMedia({video: {frameRate: 15}, audio: false});
    const mimeType = ['video/webm;codecs=vp8', 'video/webm'].find(type => MediaRecorder.isTypeSupported(type));
    const recorder = new MediaRecorder(stream, mimeType ? {mimeType} : undefined);
    screenStream = stream;
    screenChunks = [];
    screenRecorder = recorder;
    recorder.addEventListener('dataavailable', event => {if (event.data.size) screenChunks.push(event.data);});
    recorder.addEventListener('error', event => recordingStatus('录屏失败：'+(event.error?.message || '浏览器录制错误')));
    recorder.addEventListener('stop', finishRecording, {once: true});
    stream.getVideoTracks()[0].addEventListener('ended', () => {if (recorder.state !== 'inactive') recorder.stop();}, {once: true});
    recorder.start(1000);
    screenStartedAt = Date.now();
    recordingButtons();
    screenTimer = setInterval(() => recordingStatus('● 正在录制 '+Math.floor((Date.now()-screenStartedAt)/60_000).toString().padStart(2,'0')+':'+Math.floor((Date.now()-screenStartedAt)/1000%60).toString().padStart(2,'0')+' · 停止后自动下载 WebM'), 1000);
    recordingStatus('● 正在录制 00:00 · 停止后自动下载 WebM');
  } catch (error) {
    screenStream?.getTracks().forEach(track => track.stop());
    screenStream = screenRecorder = null;
    recordingButtons();
    recordingStatus(error.name === 'NotAllowedError' ? '已取消屏幕选择；需要录制时再点“开始录屏”。' : '无法开始录屏：'+error.message);
  }
}

element('screenRecordStart').addEventListener('click', startRecording);
element('screenRecordStop').addEventListener('click', () => {
  if (screenRecorder?.state === 'recording') screenRecorder.stop();
});
window.addEventListener('beforeunload', event => {
  if (!screenRecorder) return;
  event.preventDefault();
  event.returnValue = '';
});

async function api(path, options = {}) {
  const response = await fetch(path, {cache: 'no-store', ...options});
  const value = await response.json();
  if (!response.ok) throw new Error(value.error || `HTTP ${response.status}`);
  return value;
}

function status(message, failed = false) {
  element('graspStatus').textContent = message;
  element('graspStatus').dataset.error = String(failed);
}

function buttons() {
  const can = action => connected && !submitting && (!capturing || action === 'abort') && allowedActions[action] === true;
  element('armZero').disabled = !can('zero');
  element('armObservation').disabled = !can('observation');
  element('graspPreview').disabled = !can('preview') || !selection;
  element('graspRun').disabled = !can('run') || !selection;
  element('graspPlaceSelect').disabled = !can('place');
  element('graspPlace').disabled = !can('place') || !selection;
  element('graspAbort').disabled = !can('abort');
  element('graspRelease').disabled = !can('release');
  element('graspRestore').disabled = !can('restore_initial');
  element('graspRecoverFault').disabled = !can('recover_fault');
  element('rgbdRefresh').disabled = capturing || !can('observe');
  element('graspCamera').disabled = !can('observe');
}

function decode(value) {
  return Uint8Array.from(atob(value), character => character.charCodeAt(0));
}

function paint() {
  if (!rgbImage) return;
  colorContext.putImageData(rgbImage, 0, 0);
  depthContext.putImageData(depthImage, 0, 0);
  if (!selection) return;
  for (const context of [colorContext, depthContext]) {
    const [horizontal, vertical] = selection;
    context.strokeStyle = '#fff';
    context.lineWidth = 5;
    context.beginPath();
    context.arc(horizontal, vertical, 12, 0, Math.PI * 2);
    context.stroke();
    context.strokeStyle = '#ff573c';
    context.lineWidth = 2;
    context.beginPath();
    context.arc(horizontal, vertical, 12, 0, Math.PI * 2);
    context.moveTo(horizontal-20, vertical);
    context.lineTo(horizontal+20, vertical);
    context.moveTo(horizontal, vertical-20);
    context.lineTo(horizontal, vertical+20);
    context.stroke();
  }
}

async function capture(manual = false) {
  if (capturing) { if (manual) captureQueued = true; return; }
  if (submitting || running || document.hidden) return;
  capturing = true;
  buttons();
  try {
    const camera = element('graspCamera').value;
    const next = await api('/api/grasp/rgbd?camera='+encodeURIComponent(camera));
    if (camera !== element('graspCamera').value || instance !== next.instance) return;
    if (!manual && selection) return;
    const color = decode(next.rgb), bytes = next.depth ? decode(next.depth) : null;
    selfPixels = next.self_mask ? decode(next.self_mask) : new Uint8Array(640*480);
    const depths = bytes ? new DataView(bytes.buffer) : null;
    const nextRgb = colorContext.createImageData(640, 480);
    const nextDepth = depthContext.createImageData(640, 480);
    const values = new Float32Array(640*480);
    for (let index = 0; index < values.length; index++) {
      const offset = index*4, value = depths ? depths.getFloat32(offset, true) : NaN;
      values[index] = value;
      nextRgb.data[offset] = color[index*3];
      nextRgb.data[offset+1] = color[index*3+1];
      nextRgb.data[offset+2] = color[index*3+2];
      nextRgb.data[offset+3] = 255;
      const shade = Number.isFinite(value) && value > 0 ? Math.max(0, Math.min(255, (3-value)/2.9*255)) : 0;
      nextDepth.data[offset] = shade;
      nextDepth.data[offset+1] = shade;
      nextDepth.data[offset+2] = shade;
      nextDepth.data[offset+3] = 255;
    }
    frame = next;
    depthCanvas.parentElement.querySelector('figcaption').textContent = next.depth
      ? 'Depth 深度 · 0.1–3 m' : 'Go2 无深度 · RGB 仅供观察';
    frameCapturedAt = Date.now();
    pixels = values;
    rgbImage = nextRgb;
    depthImage = nextDepth;
    selection = null;
    element('graspPixel').textContent = '未选择目标';
    element('graspDepth').textContent = '深度：—';
    element('graspStamp').textContent = new Date(frame.stamp*1000).toLocaleTimeString('zh-CN')+' · 640×480';
    paint();
    if (!next.depth) status('Go2 机身只有 RGB，仅供观察；请切换腕部 RGB-D 选取抓取或放置点');
    else if (!owner) status('腕部 RGB-D 已连接 · 点击 RGB 中的目标冻结选点');
  } catch (error) {
    frame = null;
    selection = null;
    status('RGB-D采集失败：'+error.message, true);
  } finally {
    capturing = false;
    buttons();
    if (captureQueued) { captureQueued = false; capture(true); }
  }
}

function select(horizontal, vertical) {
  if (!frame || running || submitting || (owner && !holding)) return;
  if (!frame.depth || frame.camera !== 'wrist_rgbd') {
    status('机身 RGB 没有深度，不能确定三维落点；请切换腕部 RGB-D', true);
    return;
  }
  if (Date.now()-frameCapturedAt > 25000) {
    selection = null;
    status('RGB-D 画面已过期，正在刷新；请在新画面重新选点', true);
    capture(true);
    return;
  }
  const value = pixels[vertical*640+horizontal];
  if (selfPixels?.[vertical*640+horizontal]) {
    selection = null;
    status('这里被机械臂或夹爪遮挡；切换观察姿态后选择可见目标', true);
  } else if (!Number.isFinite(value) || value < .1 || value > 2.5) {
    selection = null;
    status('该像素深度无效或超出原抓取范围（0.1–2.5 m），请选择可达目标', true);
  } else {
    selection = [horizontal, vertical];
    element('rgbdLive').checked = false;
    element('graspPixel').textContent = `像素 (${horizontal}, ${vertical}) · 已冻结`;
    element('graspDepth').textContent = `深度：${value.toFixed(3)} m`;
    status(holding ? '放置点已选 · 确认后将检测水平支撑面并执行放置' : '目标已选 · 执行时自动规划并抓取；选点本身不移动机械臂');
  }
  paint();
  buttons();
}

rgb.addEventListener('click', event => {
  const bounds = rgb.getBoundingClientRect();
  select(Math.max(0, Math.min(639, Math.floor((event.clientX-bounds.left)*640/bounds.width))),
         Math.max(0, Math.min(479, Math.floor((event.clientY-bounds.top)*480/bounds.height))));
});
rgb.addEventListener('keydown', event => {
  if (holding && event.key.startsWith('Arrow')) return;
  if (!event.key.startsWith('Arrow') && event.key !== 'Enter') return;
  event.preventDefault();
  event.stopPropagation();
  const current = selection || [320, 240];
  const horizontal = current[0]+(event.key === 'ArrowRight' ? 1 : event.key === 'ArrowLeft' ? -1 : 0);
  const vertical = current[1]+(event.key === 'ArrowDown' ? 1 : event.key === 'ArrowUp' ? -1 : 0);
  select(Math.max(0, Math.min(639, horizontal)), Math.max(0, Math.min(479, vertical)));
});

async function action(operation, confirmed = false) {
  if (submitting) return;
  if (['preview', 'run', 'place'].includes(operation) && (!frame || !selection)) return;
  if (operation === 'place' && Date.now()-frameCapturedAt > 25000) {
    selection = null;
    status('放置画面已过期，正在刷新；请在新画面重新点击空白桌面', true);
    buttons();
    capture(true);
    return;
  }
  if (['run', 'preview'].includes(operation) && Date.now()-frameCapturedAt > 25000) {
    selection = null;
    status('抓取画面已过期，正在刷新；请在新画面重新点选目标', true);
    buttons();
    capture(true);
    return;
  }
  if (operation === 'run' && !confirm('仅仿真：确认让机械臂执行所选目标的抓取？停止不会自动张爪。')) return;
  if (operation === 'place' && !confirm('仅仿真：确认按当前 RGB-D 选点放置夹持物？规划和下降完成后会张开夹爪。')) return;
  if (operation === 'place' && Date.now()-frameCapturedAt > 25000) {
    selection = null;
    status('确认期间画面已过期，正在刷新；请重新点击空白桌面', true);
    buttons();
    capture(true);
    return;
  }
  if (operation === 'restore_initial' && !confirm('确认恢复场景初始状态？当前机器人和物体位置、夹持状态会重置；系统会先保存现场快照。')) return;
  if (operation === 'recover_fault' && !confirm('确认处理已停止的任务？系统会再次核对夹爪没有夹持物体，保存现场并清除任务占用；场景和物体位置保持当前状态。')) return;
  const pose = operation === 'zero' || operation === 'observation';
  if (pose && !confirmed) {
    pendingPose = operation;
    element('armPoseTitle').textContent = '切换到'+(operation === 'zero' ? '机械臂零位' : '观察姿态')+'？';
    element('armPoseConfirm').showModal();
    return;
  }
  if (pose) {
    frame = null;
    selection = null;
    element('graspPixel').textContent = '姿态变化后请刷新并重新选点';
    element('graspDepth').textContent = '深度：—';
  }
  submitting = true;
  buttons();
  let restored = false;
  let refreshAfterError = false;
  try {
    const result = await api('/api/grasp/action', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({operation, instance, frame_id: frame?.frame_id,
        u: selection?.[0], v: selection?.[1], execute: operation === 'run' || operation === 'place' || pose,
        confirm_restore: operation === 'restore_initial'})
    });
    if (result.started) {
      running = true;
      currentRun = result.run_directory;
      displayedRun = null;
      status(pose ? '正在检查零位映射与碰撞路径，并缓慢切换姿态…' : operation === 'preview' ? '正在运行原抓取感知与 MoveIt 规划，不执行机械臂轨迹…' : operation === 'place' ? '正在根据 RGB-D 平面规划并放置…' : '正在执行仿真抓取…');
    } else if (result.recovered) {
      restored = true;
      frame = selection = pixels = null;
      currentRun = displayedRun = null;
      status('已清除失败任务占用 · 场景保留，请刷新 RGB-D 后重新选点');
    } else if (result.restored) {
      restored = true;
      frame = selection = pixels = null;
      currentRun = displayedRun = null;
      element('graspPixel').textContent = '未选择目标';
      element('graspDepth').textContent = '深度：—';
      status('已恢复场景初始状态 · 现场快照：'+result.archive);
    } else {
      status(operation === 'abort' ? '已停止轨迹，未张爪；请检查恢复状态' : '已交还行走控制');
    }
  } catch (error) {
    status('请求失败：'+error.message, true);
    if (/RGB-D image expired or camera moved|Image expired/.test(error.message)) {
      selection = null;
      refreshAfterError = true;
    }
  } finally {
    submitting = false;
    buttons();
    if (restored || refreshAfterError) capture(true);
  }
}

async function poll() {
  try {
    const result = await api('/api/grasp/status');
    connected = true;
    configured = result.configured;
    supportsRestore = result.supports_restore_initial === true;
    supportsPlace = result.supports_rgbd_place === true;
    supportsPlaceRetry = result.supports_place_retry === true;
    element('observationDescription').textContent = Array.isArray(result.observation_pose_deg)
      ? '零位：六轴 0°；观察：'+result.observation_pose_deg.map((value,index)=>`J${index+1} ${value}°`).filter(item=>!item.endsWith(' 0°')).join('、')+'。按原 Piper URDF 映射；夹爪保持不变。'
      : '零位：六轴 0°；当前服务仍使用原 observation.json（仅 J5 +5°）。新版观察姿态需重启服务后启用。';
    if (result.instance !== instance) {
      frame = null;
      selection = null;
      instance = result.instance;
    }
    running = result.running;
    allowedActions = result.task?.allowed_actions || {};
    document.body.dataset.taskState = result.task?.state || 'disconnected';
    owner = result.world.owner;
    recoverableFault = result.recovery_available === true;
    const becameHolding = result.world.holding && !holding;
    holding = result.world.holding;
    lastOperation = result.operation;
    suspectedLost = Boolean(result.world.fault && !holding);
    document.body.dataset.graspHolding = String(holding && result.world.drive_enabled);
    element('graspModeTitle').textContent = suspectedLost ? '请检查夹持状态' : holding ? '点选放置位置' : '点选抓取目标';
    element('graspModeHint').textContent = holding
      ? suspectedLost
        ? (result.task?.error || '后端报告任务故障，请恢复后继续。')
        : lastOperation === 'place' && !supportsPlaceRetry
        ? '当前服务尚未加载放置重试修复；物体仍夹持。结束夹持并重启服务后可再次放置。'
        : '正在保持夹持 · 点击“选择放置点”刷新画面，再点击 RGB 中的空白桌面。'
      : '仅仿真 · 点击 RGB 中的瓶子冻结选点；不会自动移动机械臂。';
    element('armPoseFeedback').textContent = '当前模型关节角（°）：'+result.world.joints.map(value => (value*180/Math.PI).toFixed(1)).join(' / ');
    currentRun = result.run_directory;
    if (becameHolding) {
      frame = null;
      selection = null;
      element('graspPixel').textContent = '持物后请刷新并点击空白桌面';
      element('graspDepth').textContent = '深度：—';
      element('rgbdLive').checked = false;
      if (!running) capture(true);
    }
    element('graspEvidence').textContent = owner
      ? suspectedLost ? (result.task?.error || '任务故障，请恢复现场')
        : running && !holding ? '任务执行中 · 持续检查运动与接触反馈'
        : !holding && result.world.contact_force >= .1 ? '夹爪仍与物体接触，但任务未确认夹持 · 可恢复初始状态'
        : !holding ? '失败任务仍占用机器人 · 点击“处理故障并继续”后重试'
        : `保持夹持 · 夹持力 ${result.world.contact_force.toFixed(2)} N · ${result.world.drive_enabled ? '可用方向键或行走按钮移动' : '抓取动作中，完成后可行走'}`
      : '当前未被抓取任务占用';
    const phaseLabels = {idle: '空闲', planning: '规划', approaching: '接近', closing: '闭爪',
      lifting: '抬升', holding: '持物', place_descending: '几何引导下降', releasing: '到位缓慢释放',
      retreating: '撤离', returning_zero: '回零', retryable_failure: '可重试失败',
      recovery_required: '需要恢复'};
    element('graspEvidence').textContent = '阶段：'+(phaseLabels[result.task?.state] || '等待状态')+' · '+element('graspEvidence').textContent;
    element('graspLog').textContent = (currentRun || '尚无任务')+'\n'+(result.log || '');
    if (!running && currentRun && displayedRun !== currentRun) {
      displayedRun = currentRun;
      const report = result.result;
      if (report?.passed && result.operation === 'preview') {
        status('规划通过 · 此次仅预览；执行任务会采集新图并重新核对');
      } else if (report?.passed && ['zero', 'observation'].includes(result.operation)) {
        status('已到达'+(result.operation === 'zero' ? '零位' : '观察姿态')+' · 夹爪未改变；请刷新 RGB-D 并重新选点');
        capture(true);
      } else if (report?.passed && result.operation === 'place') {
        status('放置完成 · 腕部深度复核物体位于落点附近且连续稳定，机械臂已回零');
        capture(true);
      } else if (report?.passed) {
        status('物理抓取验证通过 · 保持夹持，可用方向键或行走按钮移动');
        if (!frame && !capturing) capture(true);
      } else {
        const error = report?.error || `进程退出 ${result.exit_code}，请展开日志`;
        status(result.operation === 'place' && report?.failed_phase === 'open_gripper'
          ? '张爪阶段已暂停：'+error+'。物体可能已松脱，请检查画面；确认已放下后可点“处理故障并继续”。'
          : result.operation === 'place' && report?.released && report?.returned_to_zero
          ? '已释放并回零，但视觉结果尚未确认：'+error+'。请查看物体后继续操作。'
          : result.operation === 'place' && report?.placed
          ? '物体已稳定放置，但机械臂未回零：'+error+'。可在确认周围无障碍后点“机械臂零位”。'
          : result.operation === 'place' && report?.released === false
          ? '放置未执行：'+error+'。物体仍夹持；请靠近可达桌面、刷新画面并重新选点。'
          : !holding && owner && result.world.contact_force < .1
          ? '任务未通过：'+error+'。点击“处理故障并继续”可保留场景后重新操作。'
          : '任务未通过：'+error, true);
        if (result.operation === 'place') {
          frame = null;
          selection = null;
          element('graspPixel').textContent = '放置失败后请重新选点';
          element('graspDepth').textContent = '深度：—';
        }
      }
    }
    if (result.world.fault && owner) status('需要恢复：'+result.world.fault, true);
  } catch (error) {
    connected = false;
    status('状态连接失败：'+error.message, true);
  }
  buttons();
  setTimeout(poll, 1200);
}

element('rgbdRefresh').onclick = () => capture(true);
element('graspPlaceSelect').onclick = () => {
  element('graspCamera').value = 'wrist_rgbd';
  selection = null;
  status('正在刷新放置画面；请在 RGB 中点击空白桌面');
  capture(true);
};
element('armPoseCancel').onclick = () => { pendingPose = null; element('armPoseConfirm').close(); };
element('armPoseConfirm').oncancel = () => { pendingPose = null; };
element('armPoseExecute').onclick = () => {
  const operation = pendingPose;
  pendingPose = null;
  element('armPoseConfirm').close();
  if (operation) action(operation, true);
};
element('graspCamera').onchange = () => {
  frame = selection = pixels = null;
  element('graspPixel').textContent = '相机已切换，正在刷新';
  element('graspDepth').textContent = '深度：—';
  buttons();
  capture(true);
};
element('rgbdLive').onchange = () => { if (element('rgbdLive').checked) capture(true); };
for (const [id, operation] of [['armZero', 'zero'], ['armObservation', 'observation'], ['graspPreview', 'preview'], ['graspRun', 'run'], ['graspPlace', 'place'], ['graspAbort', 'abort'], ['graspRelease', 'release'], ['graspRecoverFault', 'recover_fault'], ['graspRestore', 'restore_initial']]) {
  element(id).onclick = () => action(operation);
}
try {
  const robot = await api('/api/robot');
  if (robot.name === 'Go2 + Piper') {
    instance = robot.control_instance;
    document.body.dataset.grasp = 'true';
    element('graspPanel').hidden = false;
    poll();
    capture();
    setInterval(() => {
      if (element('rgbdLive').checked && !selection && !document.hidden) capture();
    }, 1600);
  }
} catch (error) {
  status('抓取面板连接失败：'+error.message, true);
}


// Test-table navigation is read-only and works even when WebGL is unavailable.
async function loadTestSurfaces() {
  try {
    const response = await fetch('/manifest.json', {cache:'no-store'});
    if (!response.ok) throw new Error('桌面列表读取失败');
    const manifest = await response.json();
    const surfaces = manifest.manipulation_surfaces || [];
    if (!surfaces.length) return;
    const select = document.getElementById('testSurface');
    select.replaceChildren();
    for (const [index, surface] of surfaces.entries()) {
      const option = document.createElement('option');
      option.value = String(index);
      option.textContent = surface.name+' · '+(surface.tabletop_height_m*100).toFixed(1)+' cm';
      select.appendChild(option);
    }
    const showInfo = () => {
      const surface = surfaces[Number(select.value)];
      const count = (manifest.placed_objects || []).filter(item => item.support_table === surface.support_table).length;
      const [xmin,xmax,ymin,ymax] = surface.table_bounds_world;
      document.getElementById('testSurfaceInfo').textContent =
        count+' 个测试物体 · 中心 ('+((xmin+xmax)/2).toFixed(2)+', '+((ymin+ymax)/2).toFixed(2)+') m';
      return surface;
    };
    select.addEventListener('change', showInfo);
    document.getElementById('testSurfaceView').addEventListener('click', () => {
      dispatchEvent(new CustomEvent('lab:surface-view', {detail:showInfo()}));
    });
    showInfo();
    document.getElementById('testSurfaces').hidden = false;
  } catch (error) { console.warn('Test tables:', error); }
}
loadTestSurfaces();
