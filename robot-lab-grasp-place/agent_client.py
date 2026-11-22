"""Small, dependency-free client for the Robot Lab grasp/place HTTP contract.

The server remains the authority for task state, sensing, safety and motion.
This client never chooses pixels or retries a failed motion automatically.
"""
import argparse
import base64
import json
from pathlib import Path
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def call(base, path, data=None):
    body = None if data is None else json.dumps(data).encode()
    request = Request(base.rstrip('/') + path, data=body,
                      headers={'Content-Type': 'application/json'} if body else {})
    try:
        with urlopen(request, timeout=15) as response:
            return json.load(response)
    except HTTPError as error:
        try:
            message = json.load(error).get('error', str(error))
        except (ValueError, OSError):
            message = str(error)
        raise RuntimeError(f'HTTP {error.code}: {message}') from error
    except URLError as error:
        raise RuntimeError(f'Lab service unavailable: {error.reason}') from error


def status(base):
    return call(base, '/api/grasp/status')


def capture(base, output):
    frame = call(base, '/api/grasp/rgbd?camera=wrist_rgbd')
    if frame.get('camera') != 'wrist_rgbd' or not frame.get('depth'):
        raise RuntimeError('Wrist RGB-D unavailable')
    output.mkdir(parents=True, exist_ok=True)
    (output/'rgb.ppm').write_bytes(b'P6\n640 480\n255\n' + base64.b64decode(frame['rgb']))
    (output/'depth-f32le.bin').write_bytes(base64.b64decode(frame['depth']))
    (output/'self-mask-u8.bin').write_bytes(base64.b64decode(frame['self_mask']))
    metadata = {key: value for key, value in frame.items()
                if key not in ('rgb', 'depth', 'self_mask')}
    (output/'frame.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + '\n')
    return {'frame': str(output/'frame.json'), 'rgb': str(output/'rgb.ppm'),
            'depth': str(output/'depth-f32le.bin'), 'metadata': metadata}


def action(base, operation, frame_path=None, u=None, v=None):
    current = status(base)
    if not current.get('task', {}).get('allowed_actions', {}).get(operation, False):
        raise RuntimeError(f'{operation} is not allowed in state '
                           f"{current.get('task', {}).get('state')}; inspect status")
    request = {'operation': operation, 'instance': current['instance']}
    if operation in ('run', 'place'):
        if frame_path is None or u is None or v is None:
            raise ValueError('run/place require --frame, --u and --v')
        frame = json.loads(frame_path.read_text())
        if frame['instance'] != current['instance']:
            raise RuntimeError('Captured frame belongs to a different Lab instance')
        if not (0 <= u < 640 and 0 <= v < 480):
            raise ValueError('Pixel must be inside 640x480 wrist image')
        request.update(frame_id=frame['frame_id'], u=u, v=v, execute=True)
    elif operation in ('zero', 'observation'):
        request['execute'] = True
    elif operation == 'restore_initial':
        request['confirm_restore'] = True
    return call(base, '/api/grasp/action', request)


def wait(base, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = status(base)
        if not result.get('running'):
            return result
        time.sleep(0.5)
    raise TimeoutError(f'Task still running after {timeout:g}s; query status before retry')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18769')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('status')
    image = sub.add_parser('capture')
    image.add_argument('--out', type=Path, required=True)
    motion = sub.add_parser('action')
    motion.add_argument('operation', choices=('run', 'place', 'abort', 'release',
                                               'recover_fault', 'zero', 'observation',
                                               'restore_initial'))
    motion.add_argument('--frame', type=Path)
    motion.add_argument('--u', type=int)
    motion.add_argument('--v', type=int)
    waiting = sub.add_parser('wait')
    waiting.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args(argv)
    try:
        result = (status(args.url) if args.command == 'status' else
                  capture(args.url, args.out) if args.command == 'capture' else
                  action(args.url, args.operation, args.frame, args.u, args.v)
                  if args.command == 'action' else wait(args.url, args.timeout))
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (RuntimeError, TimeoutError, ValueError, KeyError, OSError) as error:
        print(json.dumps({'error': str(error)}, ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
