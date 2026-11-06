"""Durable, at-most-once navigation/pick coordinator; hardware supplied by adapters."""
import json, threading, time, math, re
from pathlib import Path
from common import atomic

TERMINAL = {'HOLDING', 'RECOVERY_REQUIRED', 'FAILED', 'CANCELLED', 'TIMEOUT'}

class Coordinator:
    def __init__(self, directory, backend, config):
        self.directory = Path(directory); self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory/'state.json'
        self.backend, self.config = backend, config
        self.mutex = threading.RLock(); self.cancelled = threading.Event(); self.pick_requested = threading.Event()
        self.state = json.loads(self.path.read_text()) if self.path.exists() else {'tasks': {}, 'current': None, 'locked': False}
        self.active = False
        if self.state['current']:
            task = self.state['tasks'][self.state['current']]
            if task['phase'] not in TERMINAL:
                task.update(phase='RECOVERY_REQUIRED', reason='Supervisor interrupted; physical outcome unknown')
                self.state['locked'] = True
        self.save()

    def save(self): atomic(self.path, self.state)

    def phase(self, task_id, phase, **values):
        with self.mutex:
            self.state['tasks'][task_id].update(phase=phase, updated=time.time(), **values)
            self.save()
            with (self.directory/'events.jsonl').open('a') as f:
                f.write(json.dumps(self.state['tasks'][task_id], ensure_ascii=False)+'\n')

    def status(self):
        with self.mutex: return json.loads(json.dumps(dict(self.state, active=self.active)))

    def submit(self, command, task_id, goal=None, timeout=None, supervised=False):
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,80}', task_id): raise ValueError('Invalid task_id')
        timeout = self.config['task_timeout'] if timeout is None else timeout
        if not isinstance(timeout, (int,float)) or not math.isfinite(timeout) or not 1 <= timeout <= 3600:
            raise ValueError('timeout must be 1..3600 seconds')
        if command not in ('run', 'pick-once'): raise ValueError('Invalid task command')
        if command == 'run':
            if goal not in self.config['goals']: raise ValueError('Unknown preset observation goal: '+str(goal))
            g = self.config['goals'][goal]
            if g != {'target_class':'bottle', 'stop_distance':0.52}:
                raise ValueError('Project navigation currently supports bottle at optical depth <=0.54m (0.52m + 0.02m tolerance)')
        with self.mutex:
            if task_id in self.state['tasks']:
                task=self.state['tasks'][task_id]
                if command=='pick-once' and self.active and task_id==self.state['current'] and task['phase']=='AWAITING_PICK' and not self.cancelled.is_set():
                    task.update(phase='PICK_REQUESTED',reason='Operator explicitly requested second stage')
                    self.save();self.pick_requested.set()
                    return {'accepted':True,'task_id':task_id,'result':'PICK_REQUESTED'}
                return {'duplicate': True, 'task': self.state['tasks'][task_id]}
            if self.active or self.state['locked'] or self.state.get('restart_required'): raise RuntimeError('Task/recovery lock is held')
            self.backend.supervised=bool(supervised)
            self.state['current'] = task_id
            self.state['tasks'][task_id] = dict(task_id=task_id, command=command, goal=goal, supervised=bool(supervised), phase='CHECK_ENV', reason='', created=time.time())
            # Write before spawning or submitting any side effect. Crash is never a retry.
            self.state['locked'] = True; self.save()
            self.active = True; self.cancelled.clear(); self.pick_requested.clear()
            threading.Thread(target=self._run, args=(task_id, command, goal, timeout), daemon=True).start()
            return {'accepted': True, 'task_id': task_id, 'result': 'PENDING'}

    def _run(self, tid, command, goal, timeout):
        deadline = time.monotonic()+timeout
        def check():
            if self.cancelled.is_set(): raise InterruptedError('Operator cancellation')
            if time.monotonic() >= deadline: raise TimeoutError('Task deadline exceeded')
        def step(phase, operation, *args):
            check(); self.phase(tid, phase)
            value = operation(*args, deadline=deadline, check=check)
            check(); return value
        try:
            step('CHECK_ENV', self.backend.ready)
            step('CLAIM_CONTROL', self.backend.claim, tid)
            if command == 'run':
                step('CHECK_TRAVEL_POSE', self.backend.pose, 'travel')
                result = step('NAVIGATING', self.backend.navigate, self.config['goals'][goal], tid)
                if result['result'] != 'ARRIVED':
                    phase = {'CANCELLED':'CANCELLED','TIMEOUT':'TIMEOUT'}.get(result['result'],'FAILED')
                    self.phase(tid, phase, result=result['result'], reason=result.get('reason','Navigation failed'))
                    return
            step('VERIFY_STOPPED', self.backend.stopped)
            if command=='run':
                self.phase(tid,'AWAITING_PICK',result='ARRIVED',reason='Approach finished and stop verified; waiting for pick-once with the same task_id')
                while not self.pick_requested.wait(.1):
                    check();self.backend.hold(deadline=deadline,check=check)
                check()
                # Operator delay invalidates old data; recheck before the second stage.
                step('VERIFY_STOPPED',self.backend.stopped)
            # No unverified automatic joint transition. Pose mismatch is a hard stop.
            step('CHECK_OBSERVATION_POSE', self.backend.pose, 'observation')
            result = step('PICKING', self.backend.pick_once, tid)
            if result.get('result') not in ('HOLDING','RECOVERY_REQUIRED','FAILED','TIMEOUT','CANCELLED'):
                raise RuntimeError('Invalid grasp final result')
            self.phase(tid, result['result'], result=result['result'], evidence=result,
                       reason=result.get('reason',''))
        except InterruptedError as exc: self.phase(tid, 'CANCELLED', result='CANCELLED', reason=str(exc))
        except TimeoutError as exc: self.phase(tid, 'TIMEOUT', result='TIMEOUT', reason=str(exc))
        except Exception as exc: self.phase(tid, 'FAILED', result='FAILED', reason=str(exc))
        finally:
            with self.mutex: phase = self.state['tasks'][tid]['phase']
            if phase != 'HOLDING':
                try: result = self.backend.abort(tid)
                except Exception as exc: result = {'confirmed':False,'reason':str(exc)}
                with self.mutex:
                    self.state['tasks'][tid]['abort'] = result; self.save()
            with self.mutex:
                self.active = False
                # All attempts require explicit reconciliation, including uncertain enable/start.
                self.state['locked'] = True; self.save()

    def abort(self):
        with self.mutex:
            self.cancelled.set()
            tid = self.state['current']
        return self.backend.abort(tid)

    def recover(self, task_id, note):
        if not note or len(note.strip())<10: raise ValueError('Provide an onsite recovery record (at least 10 characters)')
        with self.mutex:
            if self.active: raise RuntimeError('Task still active; abort and inspect first')
            if task_id != self.state['current']: raise RuntimeError('Recovery task_id mismatch')
            self.backend.recovery_check()
            self.state['tasks'][task_id]['recovery'] = {'note':note, 'time':time.time()}
            # Preserve dedup history forever. down must restart latched executor before another pick.
            self.state['locked'] = False; self.state['restart_required'] = True; self.save()
            return {'reconciled': True, 'next': 'down then up; old task IDs remain non-reusable'}
