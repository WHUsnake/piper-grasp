"""Local JSON RPC. No ROS environment is imported by the supervisor."""
import json, os, socket, socketserver, time
from pathlib import Path


def atomic(path, value):
    path = Path(path)
    tmp = path.with_suffix('.tmp')
    with tmp.open('w') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)
    fd = os.open(str(path.parent), os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)


def rpc(path, method, timeout=10, **args):
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(timeout)
        s.connect(str(path))
        s.sendall((json.dumps(dict(method=method, **args), allow_nan=False)+'\n').encode())
        with s.makefile('rb') as f:
            raw = f.readline(1024*1024)
        if not raw: raise RuntimeError('Adapter disconnected: '+method)
        result = json.loads(raw)
        if 'error' in result: raise RuntimeError(result['error'])
        return result


def serve(path, dispatch):
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.request.settimeout(10)
            try:
                request = json.loads(self.rfile.readline(65536))
                result = dispatch(request)
            except Exception as exc: result = {'error': str(exc)}
            try: self.wfile.write((json.dumps(result, ensure_ascii=False, allow_nan=False)+'\n').encode())
            except (BrokenPipeError, ConnectionResetError): pass
    class Server(socketserver.ThreadingUnixStreamServer):
        daemon_threads = True
    path = Path(path)
    if path.exists(): path.unlink()
    server = Server(str(path), Handler)
    os.chmod(path, 0o600)
    return server
