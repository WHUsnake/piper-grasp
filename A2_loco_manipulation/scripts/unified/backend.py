import time
from concurrent.futures import ThreadPoolExecutor
from common import rpc

class Hardware:
    def __init__(self, directory, config):
        self.directory, self.config = directory, config
        self.supervised=False; self.authority_stage=None; self.owner = None; self.grasp_started = False; self.nav_started = False

    def call(self, side, method, timeout=8, **args):
        return rpc(self.directory/(side+'.sock'), method, timeout=timeout, **args)

    def ready(self, deadline=None, check=None):
        results = {s:self.call(s,'health') for s in ('arm','nav')}
        errors = [s+': '+', '.join(r['errors']) for s,r in results.items() if r['errors']]
        if errors: raise RuntimeError('; '.join(errors))
        return results

    def authority(self, method, **args):
        path = self.directory/'supervised.sock' if self.supervised else self.config.get('authority_socket')
        if not path:
            raise RuntimeError('CONTROL_AUTHORITY_UNAVAILABLE: running A2 driver has multiple ungated motion inputs; no verified exclusive base/arm interlock')
        result = rpc(path, method, timeout=1, task_id=self.owner, **args)
        if result.get('task_id') != self.owner or not (result.get('supervised') is True if self.supervised else result.get('exclusive') is True):
            raise RuntimeError('Control authority not confirmed for this task')
        self.authority_stage=result.get('stage',self.authority_stage)
        return result

    def claim(self, task_id, **kw):
        self.owner = task_id
        self.authority('claim', stage='stationary')

    def watch(self, check, stationary=False):
        check()
        self.authority('renew', stage='grasp' if stationary else 'navigation')
        if stationary:
            s = self.call('nav','get_status', task_id=self.owner)
            if not s.get('stopped'): raise RuntimeError('Base stop feedback/command inhibition lost: '+s.get('reason',''))
        arm = self.call('arm','feedback')
        if arm['errors']: raise RuntimeError('; '.join(arm['errors']))

    def maintained_call(self, side, method, check, stationary, **args):
        # Keep supervision alive while a ROS service waits for a response.
        pool=ThreadPoolExecutor(max_workers=1)
        future=pool.submit(self.call,side,method,**args)
        try:
            while not future.done():
                self.watch(check,stationary);time.sleep(.05)
            return future.result()
        finally:
            pool.shutdown(wait=False)

    def hold(self, deadline, check):
        self.watch(check, stationary=True)

    def pose(self, kind, **kw):
        key='observation' if self.supervised and kind=='travel' else kind
        target = self.config[key+'_joint_deg']
        if target is None: raise RuntimeError('No verified '+kind+' joint posture configured')
        r = self.call('arm','pose', target=target)
        if not r['valid']: raise RuntimeError(r['reason'])

    def navigate(self, goal, task_id, deadline, check):
        self.authority('renew', stage='navigation')
        self.nav_started = True
        self.maintained_call('nav','navigate',check,False, goal=goal, task_id=task_id, duration=max(.1,deadline-time.monotonic()))
        while True:
            self.watch(check)
            r = self.call('nav','get_status', task_id=task_id)
            if r['result'] != 'RUNNING': return r
            time.sleep(.05)

    def stopped(self, deadline, check):
        stage='grasp' if self.authority_stage=='grasp' else 'stationary'
        self.authority('renew', stage=stage)
        while True:
            check()
            self.authority('renew', stage=stage)
            s = self.call('nav','get_status', task_id=self.owner)
            if s['stopped']:
                self.authority('renew',stage='grasp')
                return s
            if time.monotonic()>=deadline: raise TimeoutError('Measured base stop not confirmed')
            time.sleep(.05)

    def pick_once(self, task_id, deadline, check):
        self.watch(check, True)
        identity=self.call('nav','select_target')
        self.call('arm','fresh_reset', identity=identity)
        while not self.call('arm','fresh')['valid']:
            self.watch(check, True); time.sleep(.05)
        # preview is asynchronous, so stop/authority monitoring continues during MoveIt planning.
        self.call('arm','preview')
        while True:
            self.watch(check, True)
            p = self.call('arm','preview_result')
            if p['done']: break
            time.sleep(.05)
        if not p['success']: raise RuntimeError('Preview failed: '+p['reason'])
        self.pose('observation')
        self.watch(check, True)
        self.maintained_call('arm','enable',check,True)
        self.watch(check, True)
        # Persisted coordinator lock predates both enable and start (ambiguous responses never retry).
        self.grasp_started = True
        self.call('arm','start')
        while True:
            self.watch(check, True)
            r = self.call('arm','get_status', task_id=task_id)
            if r['result'] != 'RUNNING': return r
            time.sleep(.05)

    def abort(self, task_id):
        result = {'software_only': True, 'physical_stop_confirmed': False}
        if self.owner:
            try:result['authority']=rpc(self.directory/'supervised.sock' if self.supervised else self.config['authority_socket'],'abort',timeout=1,task_id=self.owner)
            except Exception as exc:result['authority']={'confirmed':False,'reason':str(exc)}
        # Never abort an unrelated external task. Adapters also match their task ownership.
        for side, used in (('nav',self.nav_started),('arm',self.grasp_started)):
            try:
                result[side] = self.call(side,'abort',task_id=task_id) if used else {'requested':False,'reason':'No task submitted by this supervisor'}
            except Exception as exc: result[side] = {'confirmed':False,'reason':str(exc)}
        return result

    def recovery_check(self):
        s = self.call('nav','get_status',task_id=self.owner)
        if not s['stopped']: raise RuntimeError('Cannot reconcile without measured stopped feedback')
        arm = self.call('arm','feedback')
        if arm['errors']: raise RuntimeError('Arm feedback unavailable')
