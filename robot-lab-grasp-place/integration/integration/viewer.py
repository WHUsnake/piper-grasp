"""Read-only native spectator. F follows the base; R resets; mouse orbits/pans/zooms.

Native/web never step physics or send controls; go2 is an explicit interactive simulation mode.
"""

import argparse
import time
from contextlib import nullcontext
from integration.rpc import call
import base64
import io
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


def native_view(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--socket", required=True)
    p.add_argument("--piper-xml", required=True)
    a = p.parse_args(argv)
    import mujoco
    import mujoco.viewer

    authoritative = call(a.socket, "model")
    model = mujoco.MjModel.from_xml_string(authoritative["xml"])
    # This model is a read-only display replica. Keep expensive render buffers
    # out of the desktop GPU budget; authoritative cameras/physics are separate.
    model.vis.quality.shadowsize = 1024
    model.vis.quality.offsamples = 0
    data = mujoco.MjData(model)
    controls = {"follow": False, "reset": True}

    def key(code):
        if code == ord("F"):
            controls["follow"] = not controls["follow"]
        elif code == ord("R"):
            controls.update(follow=False, reset=True)

    session = None
    with mujoco.viewer.launch_passive(
        model, data, key_callback=key, show_left_ui=False, show_right_ui=False
    ) as viewer:
        with viewer.lock():
            # The vendor arm and kinematic base use group 3, hidden by MuJoCo's defaults.
            viewer.opt.geomgroup[:] = 1
            viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
            viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        while viewer.is_running():
            started = time.monotonic()
            try:
                state = call(a.socket, "snapshot")
                if time.time() - state["stamp"] > 1:
                    raise RuntimeError("Physics snapshot stale")
                if state["session_id"] != session:
                    session = state["session_id"]
                    print(
                        "Simulation session=" + session + " map=" + state["map_id"],
                        flush=True,
                    )
                with viewer.lock():
                    for field in ("qpos", "qvel", "act", "mocap_pos", "mocap_quat"):
                        getattr(data, field)[:] = state[field]
                    data.time = state["time"]
                    data.ctrl[:] = 0
                    data.xfrc_applied[:] = 0
                    mujoco.mj_forward(model, data)
                    viewer.perturb.active = 0
                    viewer.perturb.select = 0
                    if controls["reset"]:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                        viewer.cam.lookat[:] = data.xpos[model.body("base").id] + [
                            0,
                            0,
                            0.2,
                        ]
                        viewer.cam.distance = 3.5
                        viewer.cam.azimuth = 135
                        viewer.cam.elevation = -25
                        controls["reset"] = False
                    if controls["follow"]:
                        viewer.cam.lookat[:] = data.xpos[model.body("base").id]
                viewer.set_texts(
                    (
                        mujoco.mjtFontScale.mjFONTSCALE_100,
                        mujoco.mjtGridPos.mjGRID_TOPLEFT,
                        "READ ONLY | CONNECTED\nSession\nMap\nCamera",
                        "Simulation truth localization\n"
                        + session
                        + "\n"
                        + state["map_id"][:20]
                        + "...\nMouse: orbit/pan/zoom | F: follow | R: reset",
                    )
                )
                viewer.sync(state_only=True)
            except (OSError, RuntimeError, ValueError):
                viewer.set_texts(
                    (
                        None,
                        None,
                        "DISCONNECTED - physics state is stale",
                        "Close window or restore the owned simulation services",
                    )
                )
                viewer.sync(state_only=True)
                time.sleep(0.2)
            time.sleep(max(0, 1 / 30 - (time.monotonic() - started)))


PAGE = """<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>A2 + Piper · MuJoCo 实时仿真</title><style>
body{margin:0;background:#101820;color:#e8eef5;font:16px/1.6 system-ui,sans-serif}main{max-width:1100px;margin:32px auto;padding:0 24px}h1{font-size:28px;margin:12px 0}p{color:#b8c8d8}.badge{color:#75e5bd}a{color:#9ec8ff}section{min-width:0;background:#1a2734;border:1px solid #34495e;border-radius:12px;padding:20px;margin:18px 0}.layout{display:grid;grid-template-columns:minmax(0,2fr) minmax(260px,1fr);gap:20px}img{width:100%;border-radius:8px;background:#000}select{max-width:100%;background:#22394b;color:white;padding:9px;border:1px solid #547089;border-radius:6px}pre{white-space:pre-wrap;overflow-wrap:anywhere;font:14px/1.7 monospace}.pending{color:#ffd28d}#error{color:#ffb0b0}@media(max-width:950px){.layout{grid-template-columns:1fr}}
</style><main><span class="badge">仅本机仿真 · 不连接实机 · 实时物理画面</span><h1>A2 机身包络 + Piper + 瓶子</h1>
<p>这里是 MuJoCo 物理世界；云平台中的 UE 是地图工作台，两种画面用途不同。<a id="platform" target="_blank" rel="noopener">打开云平台工作台 ↗</a></p>
<section><strong class="pending">{{NAVIGATION_STATUS}}</strong><br>机身使用运动学包络，Piper 使用物理关节，瓶子是自由物体。当前页面只读，不发送运动或抓取命令。</section>
<div class="layout"><section><label>固定传感器画面（自由浏览请使用原生 MuJoCo 窗口） <select id="camera"><option value="overview">场景总览</option><option value="body_rgbd">机身 RGB-D 彩色图</option><option value="wrist_rgbd">腕部 RGB-D 彩色图</option></select></label><p><img id="frame" alt="MuJoCo 实时渲染画面"></p><div id="updated">正在连接物理世界…</div><div id="error"></div></section>
<section><strong>物理反馈（实时）</strong><pre id="state">等待反馈…</pre><hr><strong>当前配置</strong><p>运动学机身、Piper 物理关节、自由瓶子、双 RGB-D；定位使用仿真真值。</p><strong>物理结果逐次验证</strong><p>抓取需检查抬升和持续持有；搬运需保持接触；放置需确认指定桌面支撑、松爪及稳定。执行状态见 Agent 或工作台记录。</p></section></div></main>
<script>
const frame=document.querySelector('#frame'), camera=document.querySelector('#camera');
async function tick(){try{const response=await fetch('/api/state',{cache:'no-store'});if(!response.ok)throw Error(await response.text());const s=await response.json();document.querySelector('#platform').href=s.platform_url;document.querySelector('#state').textContent=JSON.stringify({模拟身份:s.feedback.simulated,仿真会话:s.feedback.session_id,导航地图:s.feedback.map_id,反馈时间:new Date(s.feedback.stamp*1000).toLocaleTimeString("zh-CN",{timeZone:"Asia/Shanghai"}),机身位置_米:s.feedback.base_position,关节位置_弧度:s.feedback.joint_positions.map(x=>+x.toFixed(4)),夹爪开口_米:+s.feedback.gripper_width_m.toFixed(4),接触停车:s.feedback.contact_stop},null,2);await new Promise((resolve,reject)=>{frame.onload=resolve;frame.onerror=()=>reject(Error('物理画面读取失败'));frame.src='/frame.jpg?camera='+encodeURIComponent(camera.value)+'&t='+Date.now()});document.querySelector('#updated').textContent='画面更新时间：'+new Date().toLocaleTimeString("zh-CN",{timeZone:"Asia/Shanghai"});document.querySelector('#error').textContent='';}catch(e){document.querySelector('#error').textContent='连接失败：'+e.message;}finally{setTimeout(tick,900)}}tick();
</script></html>"""


def web_view(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument(
        "--platform-url", default="http://127.0.0.1:18181/spatial-workbench"
    )
    parser.add_argument("--port", type=int, default=18182)
    parser.add_argument("--navigation-enabled", action="store_true")
    args = parser.parse_args(argv)
    import numpy as np
    from PIL import Image

    if call(args.socket, "health").get("simulated") is not True:
        raise RuntimeError("Simulation handshake failed")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, status, content, kind):
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            try:
                path = urlparse(self.path)
                if path.path == "/":
                    self.respond(
                        200,
                        PAGE.replace(
                            "{{NAVIGATION_STATUS}}",
                            (
                                "导航、抓取与放置由 Agent 或工作台发起；本页只观察实时物理世界。"
                                if args.navigation_enabled
                                else "当前配置：仅物理查看器，导航任务链未由此入口启动。"
                            ),
                        ).encode(),
                        "text/html; charset=utf-8",
                    )
                elif path.path == "/api/state":
                    feedback = call(args.socket, "feedback")
                    if feedback.get("simulated") is not True:
                        raise RuntimeError("Invalid simulation identity")
                    self.respond(
                        200,
                        json.dumps(
                            {"feedback": feedback, "platform_url": args.platform_url},
                            allow_nan=False,
                        ).encode(),
                        "application/json",
                    )
                elif path.path == "/frame.jpg":
                    name = parse_qs(path.query).get("camera", ["overview"])[0]
                    if name not in ("overview", "body_rgbd", "wrist_rgbd"):
                        raise ValueError("Unknown camera")
                    frame = call(args.socket, "camera", name=name)
                    rgb = np.frombuffer(
                        base64.b64decode(frame["rgb"]), np.uint8
                    ).reshape(frame["height"], frame["width"], 3)
                    output = io.BytesIO()
                    Image.fromarray(rgb).save(output, format="JPEG", quality=85)
                    self.respond(200, output.getvalue(), "image/jpeg")
                else:
                    self.respond(404, b"Not found", "text/plain")
            except (OSError, ValueError, RuntimeError) as exc:
                self.respond(503, str(exc).encode(), "text/plain; charset=utf-8")

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Live simulation: http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()


def go2_physics(assets, command, snapshot, stop, workspace=None, restore_state=None):
    """Independent authoritative loop; HTTP and GPU rendering cannot stall physics."""
    import traceback
    from integration.go2 import Go2World
    grasp = None
    try:
        bundle = json.loads((Path(assets)/'bundle.json').read_text())
        if bundle.get('robot') == 'go2-piper':
            from integration.go2 import Go2PiperMoeWorld
            world = Go2PiperMoeWorld(assets)
        else:
            world = Go2World(assets)
        if bundle.get('robot') == 'go2-piper' and workspace:
            from integration.go2_grasp import GraspWorld
            grasp = GraspWorld(world, workspace)
        generation = -1
        if restore_state:
            world.restore_state(restore_state)
            generation = int(command[4])
        next_tick = time.monotonic()
        rate, last_wall, last_sim = 1.0, next_tick, 0.0
        while not stop.is_set():
            with command.get_lock():
                current = list(command[:])
            with grasp.lock if grasp else nullcontext():
                if int(current[4]) != generation:
                    if not grasp or not grasp.owner:
                        world.reset()
                    generation = int(current[4])
                    next_tick = last_wall = time.monotonic()
                    last_sim = 0.0
                if grasp:
                    grasp.tick()
                    with snapshot.get_lock():
                        snapshot[-2] = float(bool(grasp.owner))
                        snapshot[-3] = float(grasp.drive_enabled)
                if ((grasp and grasp.owner) or not current[3]) and not (grasp and grasp.physics_fault):
                    velocity = current[:3] if time.monotonic()-current[5] < .7 else [0, 0, 0]
                    intent = [0, 0, 0] if grasp and grasp.owner and not grasp.drive_enabled else velocity
                    if grasp:
                        world.step(intent, substeps=grasp.physics_substeps,
                                   before_step=grasp.before_physics_step,
                                   after_step=grasp.after_physics_step)
                    else:
                        world.step(intent)
            if current[3] and not (grasp and grasp.owner):
                next_tick = time.monotonic()
                stop.wait(.005)
            else:
                next_tick += world.model.opt.timestep
                if time.monotonic()-next_tick > .5:
                    next_tick = time.monotonic()
                stop.wait(max(0, next_tick-time.monotonic()))
            if world.counter % 4 == 0 or current[3]:
                now = time.monotonic()
                if now-last_wall > 3:
                    rate = (world.data.time-last_sim)/(now-last_wall)
                    last_wall, last_sim = now, world.data.time
                with grasp.lock if grasp else nullcontext():
                    state = world.snapshot()
                bodies = [v for row in state['bodies'] for v in row]
                if len(bodies)+8 > len(snapshot)-2:
                    raise RuntimeError('Go2 snapshot exceeds capacity')
                with snapshot.get_lock():
                    snapshot[:8] = [1, state['t'], state['speed'], rate,
                                     len(state['bodies']), *state['pos']]
                    snapshot[8:8+len(bodies)] = bodies
                    snapshot[-1] = time.monotonic()
    except BaseException:
        traceback.print_exc()
        with snapshot.get_lock():
            snapshot[0] = -1
        stop.set()
    finally:
        if grasp:
            grasp.close()


def go2_browser(argv=None):
    """Interactive simulation preview; unlike native/web this accepts velocity intent."""
    parser = argparse.ArgumentParser(description=go2_browser.__doc__)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--port', type=int, default=18769)
    parser.add_argument('--restore-state', type=Path)
    args = parser.parse_args(argv)
    import mimetypes
    import multiprocessing as mp
    import signal
    import threading
    import uuid
    from urllib.parse import unquote
    from integration.paths import ROOT
    assets = args.assets.resolve()
    bundle = json.loads((assets/'bundle.json').read_text())
    control_instance = uuid.uuid4().hex
    static = ROOT/'integration/static/go2'
    context = mp.get_context('spawn')
    command = context.Array('d', [0, 0, 0, 0, 0, time.monotonic()])
    snapshot = context.Array('d', 8192)
    stop = context.Event()
    from integration.go2_grasp import GraspPanel
    grasp_panel = GraspPanel(args.workspace, control_instance) if bundle.get('robot') == 'go2-piper' else None

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *values):
            if '/api/' not in str(values):
                print(fmt % values, flush=True)

        def send_json(self, value, status=200):
            data = json.dumps(value, allow_nan=False, separators=(',', ':')).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = unquote(urlparse(self.path).path)
            if path in ('/api/grasp/status', '/api/grasp/rgbd'):
                try:
                    if not grasp_panel:
                        raise RuntimeError('Grasp requires the Go2 + Piper profile')
                    if path.endswith('/status'):
                        return self.send_json(grasp_panel.status())
                    camera_name = parse_qs(urlparse(self.path).query).get('camera', ['body_rgbd'])[0]
                    return self.send_json(grasp_panel.frame(camera_name))
                except (OSError, ValueError, RuntimeError, KeyError) as error:
                    return self.send_json({'error': str(error)}, 409)
            if path == '/api/robot':
                is_piper = bundle.get('robot') == 'go2-piper'
                visual = 'robot/go2_piper.glb' if is_piper else 'robot/go2.glb'
                return self.send_json({'name': 'Go2 + Piper' if is_piper else 'Go2',
                    'control_instance': control_instance,
                    'locomotion_ready': True,
                    'motion_ready': True,
                    'motion_mode': bundle.get('motion_mode', 'gait'),
                    'visual_sha256': bundle['files'][visual],
                    'description': 'Go2 + Piper · MoE-CTS 行走' if is_piper else 'Go2 · MoE-CTS 实时控制'})
            if path in ('/api/health', '/api/state'):
                with snapshot.get_lock():
                    head = list(snapshot[:8])
                    values = list(snapshot[8:8+int(head[4])*7])
                    age = time.monotonic()-snapshot[-1]
                if path == '/api/health':
                    import mujoco
                    return self.send_json({'ok': head[0] == 1 and age < 2 and worker.is_alive(),
                        'ready': head[0] == 1 and age < 2, 'failed': head[0] < 0,
                        'snapshot_age': age,
                        'physics_alive': worker.is_alive(), 'mujoco': mujoco.__version__,
                        'service': 'robot-lab-go2-preview', 'scene': bundle['scene'],
                        'workspace': str(args.workspace.resolve()), 'simulated': True})
                if head[0] != 1 or age > 2:
                    return self.send_json({'ready': False, 'failed': head[0] < 0, 'stale': age > 2})
                return self.send_json({'ready': True, 't': head[1], 'speed': head[2],
                    'rate': head[3], 'pos': head[5:8], 'simulated': True,
                    'bodies': [values[i:i+7] for i in range(0, len(values), 7)]})
            fixed = {'/robot.glb': assets/({'go2-piper':'robot/go2_piper.glb'}.get(bundle.get('robot'),'robot/go2.glb')),
                     '/environment.hdr': assets/'environment.hdr'}
            public = {'manifest.json', 'start.json', 'tasks.json', 'tasks_original.json',
                      '场景任务清单.csv'}
            if path in fixed:
                file = fixed[path]
            else:
                if path.startswith('/source/'):
                    base, rel = assets/'source', path[8:]
                elif path.startswith('/vendor/'):
                    base, rel = assets/'vendor', path[8:]
                elif path.lstrip('/') in public:
                    base, rel = assets, path.lstrip('/')
                else:
                    base, rel = static, 'index.html' if path == '/' else path.lstrip('/')
                file = (base/rel).resolve()
                if base.resolve() not in file.parents:
                    return self.send_error(403)
            if not file.is_file():
                return self.send_error(404)
            self.send_response(200)
            self.send_header('Content-Type', mimetypes.guess_type(str(file))[0]
                             or 'application/octet-stream')
            self.send_header('Content-Length', str(file.stat().st_size))
            self.send_header('Cache-Control', 'no-cache' if file.suffix in
                             ('.html', '.js', '.css', '.json') else 'public,max-age=3600')
            self.end_headers()
            try:
                with file.open('rb') as stream:
                    while True:
                        chunk = stream.read(1024*1024)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if self.path == '/api/grasp/action':
                try:
                    if not grasp_panel:
                        raise RuntimeError('Grasp requires the Go2 + Piper profile')
                    origin = self.headers.get('Origin')
                    if origin and origin not in ('http://127.0.0.1:'+str(args.port), 'http://localhost:'+str(args.port)):
                        return self.send_json({'error': 'Cross-origin grasp request rejected'}, 403)
                    if self.headers.get_content_type() != 'application/json':
                        raise ValueError('Grasp requests require application/json')
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 4096:
                        raise ValueError('Invalid request size')
                    value = json.loads(self.rfile.read(size))
                    if not isinstance(value, dict):
                        raise ValueError('Expected a JSON object')
                    return self.send_json(grasp_panel.submit(value))
                except (ValueError, TypeError, RuntimeError, OSError, KeyError) as error:
                    return self.send_json({'error': str(error)}, 409)
            if self.path != '/api/control':
                return self.send_error(404)
            try:
                import math
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 4096:
                    raise ValueError('Invalid request size')
                value = json.loads(self.rfile.read(size))
                # Old tabs must not drive a newly started robot on the same URL.
                if value.get('instance') != control_instance:
                    return self.send_json({'error': 'Preview instance changed; reload this page'}, 409)
                if value.get('reset') and grasp_panel:
                    return self.send_json({'error': 'Use Restore initial state in the grasp panel to archive and reset this scene'}, 409)
                with snapshot.get_lock():
                    grasp_busy = bool(snapshot[-2])
                    grasp_drive_enabled = bool(snapshot[-3])
                if grasp_busy and (value.get('reset') or value.get('paused') or
                                   (any(value.get('velocity', [])) and not grasp_drive_enabled)):
                    return self.send_json({'error': 'Finish the grasp before driving; reset/pause requires releasing the held object'}, 409)
                velocity = value.get('velocity', [0, 0, 0])
                if (not isinstance(velocity, list) or len(velocity) != 3 or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in velocity)):
                    raise ValueError('Invalid velocity')
                with command.get_lock():
                    command[:3] = [max(-limit, min(limit, v))
                                   for limit, v in zip((2, 1, 1.5), velocity)]
                    command[3] = float(bool(value.get('paused', False)))
                    command[5] = time.monotonic()
                    if value.get('reset'):
                        command[4] += 1
                self.send_json({'ok': True})
            except (ValueError, TypeError) as error:
                self.send_json({'error': str(error)}, 400)

    # Bind first: an occupied port must not create an orphan physics process.
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    worker = context.Process(target=go2_physics,
                             args=(assets, command, snapshot, stop, args.workspace, args.restore_state), daemon=True)
    worker.start()
    def shutdown(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print('ROBOT_LAB_PREVIEW http://127.0.0.1:%d/' % args.port, flush=True)
    try:
        server.serve_forever()
    finally:
        if grasp_panel:
            grasp_panel.close()
        stop.set()
        server.server_close()
        worker.join(3)
        if worker.is_alive():
            worker.terminate()
            worker.join(2)


def main():
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "native":
        native_view(sys.argv[2:])
    elif len(sys.argv) > 1 and sys.argv[1] == "web":
        web_view(sys.argv[2:])
    elif len(sys.argv) > 1 and sys.argv[1] == "go2":
        go2_browser(sys.argv[2:])
    else:
        parser = argparse.ArgumentParser(description="Simulation viewers")
        parser.add_argument("mode", choices=["native", "web", "go2"])
        parser.parse_args()


if __name__ == "__main__":
    main()
