#!/usr/bin/env python3
"""Detached process ownership and durable task RPC. No boot/autostart changes."""
import argparse, fcntl, json, os, signal, subprocess, sys, threading, time
from pathlib import Path
from common import atomic, rpc, serve
from core import Coordinator
from backend import Hardware
HERE=Path(__file__).resolve().parent

def clean_env():
    return {'HOME':str(Path.home()),'USER':os.environ.get('USER','unitree'),
            'PATH':'/usr/local/bin:/usr/bin:/bin','PYTHONNOUSERSITE':'1','PYTHONUNBUFFERED':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','YOLO_AUTOINSTALL':'false'}

def shell(setup, argv):
    import shlex
    return ['bash','--noprofile','--norc','-c','set -e; '+setup+'; exec '+shlex.join(list(map(str,argv)))]

class Supervisor:
    def __init__(self,cfg,directory):
        self.cfg=cfg;self.config=json.loads(cfg.read_text());self.directory=directory
        self.children={};self.logs=[];self.stopping=False;self.boot_error='';self.camera={'ownership':'managed','reused':False}
        self.backend=Hardware(directory,self.config)
        self.coordinator=Coordinator(directory,self.backend,self.config)
        if self.coordinator.state.get('restart_required'):
            self.coordinator.state['restart_required']=False;self.coordinator.save()
    def spawn(self,name,argv):
        log=(self.directory/(name+'.log')).open('a');self.logs.append(log)
        proc=subprocess.Popen(argv,env=clean_env(),stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        self.children[name]=proc
        atomic(self.directory/'owned.json',{n:{'pid':p.pid,'start_ticks':Path('/proc/'+str(p.pid)+'/stat').read_text().split()[21]} for n,p in self.children.items() if p.poll() is None})
    def boot(self):
        import shlex
        root=Path(self.config['root']);quote=shlex.quote
        arm_setup='source '+quote(str(root/'activate-ros2.sh'))
        nav_setup='source '+quote(str(root/'activate-ros1.sh'))+'; export ROS_MASTER_URI=http://127.0.0.1:11349 ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost'
        # Refuse external owners instead of guessing configuration from a PID/name.
        probe=subprocess.run(shell(arm_setup,['ros2','node','list','--no-daemon']),env=clean_env(),capture_output=True,text=True,timeout=15)
        if probe.returncode:raise RuntimeError('ROS2 graph query failed: '+probe.stderr)
        nodes=probe.stdout.splitlines()
        camera_nodes={'/camera/camera','/camera/camera_container','/local_camera_view_stream'}
        if any(line in camera_nodes for line in nodes):
            camera_probe=subprocess.run(shell(arm_setup,['python',HERE/'camera_check.py']),env=clean_env(),capture_output=True,text=True,timeout=25)
            if camera_probe.returncode:raise RuntimeError('Existing camera failed compatibility check: '+camera_probe.stdout.strip()+' '+camera_probe.stderr.strip())
            self.camera=json.loads(camera_probe.stdout.strip().splitlines()[-1]);self.camera['reused']=True
        conflicts=[line for line in nodes if line not in camera_nodes and any(x in line for x in ('piper','perception','task_executor','robot_planner','camera','grasp_generator'))]
        if conflicts:raise RuntimeError('Unmanaged ROS2 components present; configuration/ownership cannot be proved: '+str(conflicts))
        ps=subprocess.check_output(['ps','-eo','pid,args'],text=True)
        conflicts=[s for s in ps.splitlines() if any(x in s for x in ('single_view.launch.py','single-pick-session.py','search-to-grasp.py','go_zero.py','move-saved-grasp.py','navigation-session.py','object_navigation_agent'))]
        if conflicts:raise RuntimeError('Existing unmanaged process conflict: '+str(conflicts))
        # Stale ownership means supervisor may have crashed while descendants survived.
        owned=self.directory/'owned.json'
        if owned.exists():
            for name,item in json.loads(owned.read_text()).items():
                path=Path('/proc/'+str(item['pid'])+'/stat')
                if path.exists() and path.read_text().split()[21]==item['start_ticks']:
                    raise RuntimeError('Prior owned process survives: '+name+'; inspect/reconcile; never kill by name')
        if self.coordinator.state['locked']:raise RuntimeError('Durable task recovery lock survives restart; do not start fresh executor')
        import socket, xmlrpc.client
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1',11349))==0:
                raise RuntimeError('ROS1 11349 is owned elsewhere; stop/inspect its owner before up')
        flags=Path('/sys/class/net/can0/flags')
        if not flags.exists() or not int(flags.read_text().strip(),16)&1:
            raise RuntimeError('CAN_NOT_READY: can0 is missing/DOWN; no motor enable or automatic CAN reconfiguration performed')
        self.spawn('master',shell(nav_setup,['roscore','-p','11349']))
        end=time.monotonic()+10
        while time.monotonic()<end:
            try:
                if xmlrpc.client.ServerProxy('http://127.0.0.1:11349').getPid('/unified')[0]==1:break
            except OSError:pass
            time.sleep(.1)
        else:raise RuntimeError('ROS1 master not ready')
        self.spawn('arm',shell(arm_setup,['python',HERE/'arm_adapter.py',self.cfg,self.directory/'arm.sock']))
        self.spawn('base-feedback',shell(nav_setup.replace('11349','11311'),['python',HERE/'base_feedback.py',self.directory/'base.sock']))
        self.spawn('supervised',shell(nav_setup.replace('11349','11311'),['python',HERE/'supervised_base.py',self.directory]))
        self.spawn('authority',shell(nav_setup.replace('11349','11311'),['python',HERE/'authority.py',self.directory]))
        self.spawn('nav',shell(nav_setup,['python',HERE/'nav_adapter.py',self.cfg,self.directory/'nav.sock']))
        self.spawn('single-view',shell(arm_setup,['ros2','launch','robot_bringup','single_view.launch.py','start_camera:='+str(not self.camera['reused']).lower(),'start_rviz:=false','start_image:=false','auto_enable:=false','execute:=false']))
        # Only this dedicated ros1_bridge process combines ROS environments.
        bridge_setup=nav_setup+'; rosparam load '+quote(str(root/'config/sensors/orbbec/perception_bridge.yaml'))+'; source /opt/ros/humble/setup.bash; source '+quote(str(root/'workspaces/bridge_ws/install/setup.bash'))
        self.spawn('image-target-bridge',shell(bridge_setup,[root/'workspaces/bridge_ws/install/ros1_bridge/lib/ros1_bridge/parameter_bridge']))
        self.spawn('target-exporter',shell(nav_setup,['rosrun','a2_target_handoff','target_exporter']))
        # One navigation tracker and one on-demand, identity-bound grasp segmenter.
        # No additional segmentation process; single_view already includes it, idle by default.
        self.spawn('object-navigation',shell(nav_setup,['roslaunch','a2_bringup','navigation.launch','target_class:=bottle']))
    def status(self):
        status={'supervisor':'running','boot_error':self.boot_error,'processes':{n:{'pid':p.pid,'running':p.poll() is None,'exit':p.poll()} for n,p in self.children.items()},'task':self.coordinator.status(),'components':{},'ready':False,'camera':self.camera}
        for side in ('arm','nav'):
            if side not in self.children:
                status['components'][side]={'started':False,'errors':['Not started: '+(self.boot_error or 'startup pending')]}
                continue
            try:status['components'][side]=self.backend.call(side,'health',timeout=12)
            except Exception as exc:status['components'][side]={'errors':[str(exc)]}
        status['blockers']=[]
        try:
            status['authority']=rpc(self.directory/'authority.sock','health',timeout=1)
        except Exception as exc:
            status['blockers'].append('Control authority unavailable: '+str(exc))
        if not self.config.get('external_control_reviewed',False):
            status['blockers'].append('External DDS/remote control sources have not been verified exclusive')
        if self.config['travel_joint_deg'] is None:status['blockers'].append('Verified travel arm posture not configured')
        try:status['supervised_control']=rpc(self.directory/'supervised.sock','health',timeout=1)
        except Exception as exc:status['supervised_control']={'available':False,'reason':str(exc)}
        status['environment_ready']=bool(self.children) and not self.boot_error and all(x.poll() is None for x in self.children.values()) and all(not s['errors'] for s in status['components'].values())
        try:status['observation_pose']=self.backend.call('arm','pose',target=self.config['observation_joint_deg'])
        except Exception as exc:status['observation_pose']={'valid':False,'reason':str(exc)}
        status['supervised_ready']=bool(status['environment_ready'] and status['supervised_control'].get('available') and not status['supervised_control'].get('fault') and status['observation_pose'].get('valid'))
        atomic(self.directory/'last-status.json',status)
        status['ready']=status['environment_ready'] and not status['blockers']
        return status
    def dispatch(self,q):
        m=q['method']
        if m=='status':return self.status()
        if m in ('run','pick-once'):
            if self.stopping or self.boot_error:raise RuntimeError('Environment startup failed/stopping: '+self.boot_error)
            existing=self.coordinator.state['tasks'].get(q['task_id'],{})
            supervised=q.get('supervised',False) or existing.get('supervised',False)
            if not supervised and not self.config.get('external_control_reviewed',False):raise RuntimeError('Automatic mode requires exclusive control. Operator-supervised mode is a distinct explicit --supervised option, not physical exclusivity.')
            return self.coordinator.submit(m,q['task_id'],q.get('goal'),q.get('duration'),supervised=supervised)
        if m=='abort':return self.coordinator.abort()
        if m=='recover':return self.coordinator.recover(q['task_id'],q['note'])
        if m=='down':
            with self.coordinator.mutex:
                if self.coordinator.active or self.coordinator.state['locked']:raise RuntimeError('Active/recovery/holding lock; reconcile onsite before down')
                self.stopping=True
            threading.Thread(target=self.shutdown,daemon=True).start()
            return {'accepted':True,'stopping_owned_processes_only':True}
        raise ValueError('Unknown command')
    def shutdown(self):
        for name,p in reversed(list(self.children.items())):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    # Only a still-owned child process group, never pgrep/pkill.
                    os.killpg(p.pid,signal.SIGTERM)
                    try:p.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.boot_error='Owned child failed to terminate: '+name
                        self.stopping=False;return
        self.server.shutdown()

def main():
    p=argparse.ArgumentParser()
    p.add_argument('command',choices=['up','status','run','pick-once','abort','down','recover','_serve'])
    p.add_argument('--config',type=Path,default=HERE/'config.json')
    p.add_argument('--state-dir',type=Path)
    p.add_argument('--task-id');p.add_argument('--goal',default='bottle');p.add_argument('--timeout',type=float,default=180)
    p.add_argument('--note')
    p.add_argument('--supervised',action='store_true',help='Operator confirms clear path, observation posture suitable for approach, no other controllers, and onsite emergency-stop supervision; does not provide physical exclusivity')
    a=p.parse_args();cfg=a.config.resolve();config=json.loads(cfg.read_text())
    directory=(a.state_dir or Path(config['root'])/'data/logs/unified').resolve();directory.mkdir(parents=True,exist_ok=True);os.chmod(directory,0o700)
    sock=directory/'control.sock'
    if a.command=='_serve':
        lock=(directory/'daemon.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        # Also coordinate with legacy a2ctl go-zero and search-to-grasp arm lock.
        arm_lock=(Path(config['root'])/'data/logs/arm-motion.lock').open('a')
        fcntl.flock(arm_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        s=Supervisor(cfg,directory);s.server=serve(sock,s.dispatch)
        try:s.boot()
        except Exception as exc:s.boot_error=str(exc)
        atomic(directory/'supervisor.json',{'pid':os.getpid(),'config':str(cfg)})
        s.server.serve_forever();sock.unlink(missing_ok=True);return 0
    if a.command=='up':
        lock=(directory/'up.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX)
        try:
            result=rpc(sock,'status',timeout=30)
            # Retry only a failed preflight that started no children or task.
            if result.get('boot_error') and not result.get('processes') and not result['task'].get('locked') and not result['task'].get('active'):
                rpc(sock,'down',timeout=10)
                end=time.monotonic()+10
                while sock.exists() and time.monotonic()<end:time.sleep(.1)
                if sock.exists():raise ValueError('Previous supervisor did not stop')
                raise ConnectionRefusedError('Retry clean preflight')
        except OSError:
            log=(directory/'supervisor.log').open('a')
            proc=subprocess.Popen([sys.executable,str(HERE/'supervisor.py'),'_serve','--config',str(cfg),'--state-dir',str(directory)],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=clean_env())
            end=time.monotonic()+config['startup_timeout']
            while time.monotonic()<end:
                if proc.poll() is not None:raise RuntimeError('Supervisor exited; inspect '+str(directory/'supervisor.log'))
                try:result=rpc(sock,'status',timeout=20)
                except OSError:time.sleep(.25);continue
                if result['boot_error'] or result['environment_ready']:break
                time.sleep(.5)
            else:result=rpc(sock,'status',timeout=30)
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['ready'] else 3
    if a.command in ('run','pick-once','recover') and not a.task_id:p.error('--task-id required (persistent idempotency key)')
    if a.command=='status' and not sock.exists():
        state=directory/'state.json'; last=directory/'last-status.json'
        print(json.dumps({'supervisor':'down','task':json.loads(state.read_text()) if state.exists() else None,'last_check':json.loads(last.read_text()) if last.exists() else None},ensure_ascii=False,indent=2));return 0
    result=rpc(sock,a.command,timeout=35,task_id=a.task_id,goal=a.goal,duration=a.timeout,note=a.note,supervised=a.supervised)
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0

if __name__=='__main__':
    try:sys.exit(main())
    except Exception as exc:print(json.dumps({'error':str(exc)},ensure_ascii=False),file=sys.stderr);sys.exit(2)
