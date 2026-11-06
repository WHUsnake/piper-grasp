"""Bounded transport to a persistent, non-ROS Python environment."""
import json
import os
import select
import subprocess

class Worker:
    def __init__(self, python, script, config):
        env = os.environ.copy()
        for key in ('PYTHONPATH', 'PYTHONHOME', 'VIRTUAL_ENV'):
            env.pop(key, None)
        env.update(PYTHONNOUSERSITE='1', OMP_NUM_THREADS='1', CUBLAS_WORKSPACE_CONFIG=':4096:8')
        # Script imports sibling package; this path contains only our ROS-free modules.
        env['PYTHONPATH'] = str(script.parent.parent)
        self.proc = subprocess.Popen([python, '-m', 'grasp_generation.worker', config],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
        self.counter = 0
        try:
            if not self.read(120).get('ready'): raise RuntimeError('Worker not ready')
        except Exception:
            self.close(); raise

    def read(self, timeout):
        if not select.select([self.proc.stdout], [], [], timeout)[0]:
            self.close()
            raise TimeoutError('GraspNet worker timeout')
        line = self.proc.stdout.readline()
        if not line: raise RuntimeError('GraspNet worker exited')
        return json.loads(line)

    def predict(self, points):
        self.counter += 1
        self.proc.stdin.write((json.dumps(dict(id=self.counter, points=points.tolist()), allow_nan=False)+'\n').encode())
        self.proc.stdin.flush()
        reply = self.read(10)
        if reply.get('error'): raise RuntimeError(reply['error'])
        if reply.get('id') != self.counter: raise RuntimeError('Worker request mismatch')
        return reply

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try: self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill(); self.proc.wait()
        for stream in (self.proc.stdin, self.proc.stdout):
            if stream: stream.close()
