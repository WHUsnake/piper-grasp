"""Explicit local-source integration; no ROS or model imports on the host."""
import json
import math
from pathlib import Path
import time

from lab.common import read, write
from lab.paths import ROOT


def command(cfg, args):
    if not math.isfinite(args.table_height) or not math.isfinite(args.tcp_m) or not 0 < args.tcp_m <= .3:
        raise ValueError('Table height must be finite; TCP must be finite and within (0, 0.3] metres')
    if args.pixel is not None and not (0 <= args.pixel[0] < 640 and 0 <= args.pixel[1] < 480):
        raise ValueError('Selected pixel must lie inside the 640x480 camera image')
    if args.profile != 'hssd-go2-piper':
        raise RuntimeError('This adapter requires --profile hssd-go2-piper')
    file = ROOT/'config.local.json'
    local = read(file) if file.exists() else {}
    if args.operation == 'configure':
        if not args.source or not args.worker_python:
            raise ValueError('Configure requires --source and --worker-python')
        source = Path(args.source).expanduser().resolve()
        required = ['src/manipulation/grasp_generation/config/graspnet.json',
                    'src/manipulation/robot_planner/robot_planner/node.py',
                    'src/manipulation/task_executor/task_executor/node.py',
                    'workspaces/pipeline/install/setup.bash']
        for relative in required:
            if not (source/relative).is_file():
                raise RuntimeError('Local grasp source/build missing: '+relative)
        python = Path(args.worker_python).expanduser().absolute()
        if not python.is_file():
            raise RuntimeError('GraspNet Python missing')
        baseline = Path(args.baseline or Path(cfg['workspace'])/'sources/graspnet-baseline').resolve()
        checkpoint = Path(args.checkpoint or Path(cfg['workspace'])/'models/checkpoint-rs.tar').resolve()
        local.update(workspace=cfg['workspace'], preview_profile='hssd-go2-piper')
        local['go2_grasp'] = {'source': str(source), 'worker_python': str(python),
                              'baseline': str(baseline), 'checkpoint': str(checkpoint)}
        write(file, local)
        print('Configured local ROS grasp source without modifying it: '+str(source))
        return 0
    settings = local.get('go2_grasp')
    if not settings:
        raise RuntimeError('Run lab grasp configure first')
    if args.operation == 'prepare':
        import hashlib
        import shutil
        import subprocess
        package = Path(cfg['workspace'])/'sources/A2_loco_manipulation/third_party/piper_ros/src/piper_msgs'
        if not (package/'msg/PosCmd.msg').is_file():
            raise RuntimeError('Locked Piper message source missing: '+str(package))
        hashes = {str(path.relative_to(package)): hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in sorted(package.rglob('*')) if path.is_file()}
        identity = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()[:16]
        workspace = Path(cfg['workspace']).resolve()
        directory = (Path.home()/'.cache/robot-lab/ros-interfaces'/
                     hashlib.sha256(str(workspace).encode()).hexdigest()[:16]/identity)
        if not str(directory).isascii():
            raise RuntimeError('ROS message build requires an ASCII home/cache path')
        directory.mkdir(parents=True, exist_ok=True)
        write(directory/'source-fingerprints.json', {'source': str(package), 'sha256': hashes})
        staged = directory/'source/piper_msgs'
        if not staged.exists():
            shutil.copytree(package, staged)
        staged_hashes = {str(path.relative_to(staged)): hashlib.sha256(path.read_bytes()).hexdigest()
                         for path in sorted(staged.rglob('*')) if path.is_file()}
        if staged_hashes != hashes:
            raise RuntimeError('Cached message source changed; refusing to overwrite it')
        link = workspace/'builds'/('go2-grasp-interfaces-'+identity+'-cache')
        link.parent.mkdir(parents=True, exist_ok=True)
        if not link.is_symlink() and not link.exists():
            link.symlink_to(directory, target_is_directory=True)
        if not link.is_symlink() or link.resolve() != directory:
            raise RuntimeError('Message build record points elsewhere: '+str(link))
        script = ('set -e; source /opt/ros/humble/setup.bash; '
                  '/usr/bin/python3 -m colcon --log-base "$2/log" build --base-paths "$1" '
                  '--packages-select piper_msgs --build-base "$2/build" --install-base "$2/install" '
                  '--cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3')
        subprocess.run(['bash', '-c', script, 'go2-grasp-interfaces', str(staged), str(directory)],
                       cwd=directory, check=True)
        setup = directory/'install/local_setup.bash'
        check = ('set -e; source /opt/ros/humble/setup.bash; source "$1"; '
                 '/usr/bin/python3 -c "from piper_msgs.msg import PosCmd; '
                 'from rclpy.serialization import serialize_message; serialize_message(PosCmd())"')
        subprocess.run(['bash', '-c', check, 'go2-grasp-check', str(setup)], check=True)
        settings['interfaces_setup'] = str(setup)
        write(file, local)
        print('Prepared real Piper ROS messages; no driver or motion started: '+str(setup))
        return 0
    return run_configured(cfg, args, settings)


def run_configured(cfg, args, settings):
    import sys
    sys.path.insert(0, str(ROOT/'integration'))
    from integration.rpc import call
    record = read(Path(cfg['workspace'])/'state/go2-grasp-world.json')
    feedback = call(record['socket'], 'feedback')
    if args.operation == 'status':
        print(json.dumps({'settings': settings, 'world': feedback}, indent=2))
        return 0
    if args.operation == 'abort':
        if feedback['owner']:
            print(json.dumps(call(record['socket'], 'abort', session=feedback['session'], owner=feedback['owner'])))
        return 0
    if args.operation == 'release':
        if feedback['owner']:
            print(json.dumps(call(record['socket'], 'release', session=feedback['session'], owner=feedback['owner'])))
        return 0
    if args.operation == 'run' and not args.execute:
        raise RuntimeError('Motion requires grasp run --execute; preview never moves the arm')
    if feedback['owner']:
        raise RuntimeError('A grasp/recovery already owns the robot; inspect grasp status')
    directory = Path(cfg['workspace'])/'runs'/('go2-grasp-'+str(time.time_ns()))
    options = dict(operation=args.operation, socket=record['socket'],
                   world_session=feedback['session'],
                   camera=args.camera, table_height=args.table_height, pixel=args.pixel,
                   tcp_m=args.tcp_m)
    from integration.go2_grasp import launch_pipeline
    print('Grasp artifacts: '+str(directory), flush=True)
    return launch_pipeline(settings, options, directory).wait()
