import json, sys, tempfile, time, threading, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/unified'))
from core import Coordinator
from common import rpc,serve
from backend import Hardware

class Simulated:
    def __init__(self, nav='ARRIVED', pick='HOLDING', fail=None):
        self.nav=nav;self.pick=pick;self.fail=fail;self.calls=[];self.hold_at=None
    def invoke(self,name,kw):
        self.calls.append(name)
        if self.fail==name:raise RuntimeError(name+' unavailable/unsafe')
        if self.hold_at==name:
            while True:kw['check']();time.sleep(.005)
    def ready(self,**kw):self.invoke('ready',kw)
    def claim(self,*a,**kw):self.invoke('claim',kw)
    def pose(self,kind,**kw):self.invoke(kind,kw)
    def navigate(self,*a,**kw):
        self.invoke('navigate',kw);return {'result':self.nav,'reason':'simulated navigation '+self.nav}
    def hold(self,**kw):kw['check']()
    def stopped(self,**kw):self.invoke('stopped',kw)
    def pick_once(self,*a,**kw):
        self.invoke('pick_once',kw);return {'result':self.pick,'contact_evidence':True,'object_grasp_confirmed':False}
    def abort(self,*a):self.calls.append('abort');return {'confirmed':True,'software_only':True}
    def recovery_check(self):pass

class CoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.cfg={'task_timeout':10,'goals':{'bottle':{'target_class':'bottle','stop_distance':.52}}}
    def make(self,**kw):
        b=Simulated(**kw);return Coordinator(self.tmp.name,b,self.cfg),b
    def done(self,c):
        end=time.monotonic()+3
        while c.active and time.monotonic()<end:time.sleep(.005)
        self.assertFalse(c.active)
        return c.status()['tasks']['one']
    def await_pick(self,c):
        end=time.monotonic()+2
        while time.monotonic()<end:
            if c.status()['tasks']['one']['phase']=='AWAITING_PICK':return
            time.sleep(.005)
        self.fail('Did not enter explicit second-stage wait')
    def test_approach_does_not_auto_pick(self):
        c,b=self.make();c.submit('run','one','bottle');self.await_pick(c)
        time.sleep(.03);self.assertNotIn('pick_once',b.calls)
        self.assertTrue(c.submit('run','one','bottle')['duplicate'])
        with self.assertRaises(RuntimeError):c.submit('pick-once','wrong')
        c.abort();self.assertEqual(self.done(c)['phase'],'CANCELLED')
        self.assertTrue(c.submit('pick-once','one')['duplicate']);self.assertNotIn('pick_once',b.calls)
    def test_second_stage_rechecks_stop(self):
        c,b=self.make();c.submit('run','one','bottle');self.await_pick(c)
        b.fail='stopped';c.submit('pick-once','one');self.done(c);self.assertNotIn('pick_once',b.calls)
    def test_wait_for_operator_expires_without_pick(self):
        c,b=self.make();c.submit('run','one','bottle',timeout=1);self.await_pick(c)
        self.assertEqual(self.done(c)['phase'],'TIMEOUT');self.assertNotIn('pick_once',b.calls)
    def test_arrival_is_not_grasp_success(self):
        c,b=self.make();c.submit('run','one','bottle')
        self.await_pick(c)
        self.assertNotIn('pick_once',b.calls)
        self.assertTrue(c.submit('pick-once','one')['accepted'])
        t=self.done(c)
        self.assertEqual(t['phase'],'HOLDING');self.assertFalse(t['evidence']['object_grasp_confirmed'])
        self.assertEqual(b.calls,['ready','claim','travel','navigate','stopped','stopped','observation','pick_once'])
    def test_navigation_fail_timeout_cancel_never_pick(self):
        for state in ('FAILED','TIMEOUT','CANCELLED'):
            with self.subTest(state=state),tempfile.TemporaryDirectory() as d:
                b=Simulated(nav=state);c=Coordinator(d,b,self.cfg);c.submit('run','one','bottle')
                self.assertEqual(self.done(c)['phase'],state);self.assertNotIn('pick_once',b.calls)
    def test_missing_stop_never_pick(self):
        c,b=self.make(fail='stopped');c.submit('run','one','bottle');self.done(c);self.assertNotIn('pick_once',b.calls)
    def test_missing_control_authority_never_navigate(self):
        c,b=self.make(fail='claim');c.submit('run','one','bottle');self.done(c);self.assertNotIn('navigate',b.calls)
    def test_observation_pose_mismatch(self):
        c,b=self.make(fail='observation');c.submit('pick-once','one');self.done(c);self.assertNotIn('pick_once',b.calls)
    def test_grasp_failure_service_lost(self):
        for result in ('FAILED','RECOVERY_REQUIRED','TIMEOUT'):
            with self.subTest(result=result),tempfile.TemporaryDirectory() as d:
                b=Simulated(pick=result);c=Coordinator(d,b,self.cfg);c.submit('pick-once','one')
                self.assertEqual(self.done(c)['phase'],result);self.assertTrue(c.state['locked'])
        with tempfile.TemporaryDirectory() as d:
            b=Simulated(fail='pick_once');c=Coordinator(d,b,self.cfg);c.submit('pick-once','one');self.assertEqual(self.done(c)['phase'],'FAILED')
    def test_duplicate_submission_and_recovery_lock(self):
        c,b=self.make();c.submit('pick-once','one');self.done(c)
        self.assertTrue(c.submit('pick-once','one')['duplicate'])
        with self.assertRaises(RuntimeError):c.submit('pick-once','two')
        self.assertEqual(b.calls.count('pick_once'),1)
    def test_cancel_in_flight(self):
        c,b=self.make();b.hold_at='navigate';c.submit('run','one','bottle')
        while 'navigate' not in b.calls:time.sleep(.005)
        c.abort();self.assertEqual(self.done(c)['phase'],'CANCELLED');self.assertNotIn('pick_once',b.calls)
    def test_global_deadline(self):
        c,b=self.make();b.hold_at='navigate';c.submit('run','one','bottle',timeout=1)
        self.assertEqual(self.done(c)['phase'],'TIMEOUT');self.assertNotIn('pick_once',b.calls)
    def test_restart_preserves_holding_and_idempotency(self):
        c,b=self.make();c.submit('pick-once','one');self.done(c)
        restarted=Coordinator(self.tmp.name,b,self.cfg)
        self.assertTrue(restarted.state['locked']);self.assertTrue(restarted.submit('pick-once','one')['duplicate'])
    def test_crash_mid_task_not_replayed(self):
        c,b=self.make();c.state={'tasks':{'one':{'phase':'PICKING'}},'current':'one','locked':True};c.save()
        restarted=Coordinator(self.tmp.name,b,self.cfg)
        self.assertEqual(restarted.state['tasks']['one']['phase'],'RECOVERY_REQUIRED');self.assertEqual(b.calls,[])
    def test_recovery_requires_restart_keeps_old_id(self):
        c,b=self.make();c.submit('pick-once','one');self.done(c);c.recover('one','Onsite inspected empty gripper and stopped base')
        with self.assertRaises(RuntimeError):c.submit('pick-once','two')
        self.assertTrue(c.submit('pick-once','one')['duplicate'])
    def test_real_authority_absent_refuses(self):
        b=Hardware(Path(self.tmp.name),{'authority_socket':None})
        with self.assertRaisesRegex(RuntimeError,'CONTROL_AUTHORITY'):b.claim('one')
    def test_invalid_goal_and_nan_timeout(self):
        c,b=self.make()
        with self.assertRaises(ValueError):c.submit('run','one','unknown')
        with self.assertRaises(ValueError):c.submit('pick-once','one',timeout=float('nan'))
        self.assertEqual(b.calls,[])

if __name__=='__main__':unittest.main()
