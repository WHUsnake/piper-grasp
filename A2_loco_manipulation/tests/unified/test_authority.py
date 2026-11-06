import sys,time,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/unified'))
from authority import Authority
class Fake:
    def __init__(self):
        self.now=10.;self.messages=[];self.aborted=False
        self.data={'_received':10.,'stamp':10.,'boot_id':'boot','enabled':True,'legacy_inputs_disabled':True,'base_feedback_fresh':True,'stage':'idle','lease_valid':True}
    def status(self):return self.data
    def ros_time(self):return self.now
    def claim(self):return {'token':'token','boot_id':'boot'}
    def send(self,d):self.messages.append(d)
    def abort(self):self.aborted=True
    def await_ack(self,boot,task,seq,stage,timeout):return {'stage':stage}
class Tests(unittest.TestCase):
    def make(self):
        t=Fake();return Authority(t,Path('/unused'),lambda:t.now),t
    def claim(self,a):return a.dispatch({'method':'claim','task_id':'one'})
    def test_claim_is_stationary(self):
        a,t=self.make();self.claim(a);self.assertTrue(all(m['vx']==0 for m in t.messages))
    def test_disabled_driver_refused(self):
        a,t=self.make();t.data['enabled']=False
        with self.assertRaises(RuntimeError):self.claim(a)
        self.assertEqual(t.messages,[])
    def test_old_driver_inputs_refused(self):
        a,t=self.make();t.data['legacy_inputs_disabled']=False
        with self.assertRaises(RuntimeError):self.claim(a)
    def test_stale_driver_refused(self):
        a,t=self.make();t.now=11
        with self.assertRaises(RuntimeError):self.claim(a)
    def test_wrong_task_cannot_renew(self):
        a,t=self.make();self.claim(a)
        with self.assertRaises(RuntimeError):a.dispatch({'method':'renew','task_id':'two','stage':'navigation'})
    def test_duplicate_claim_refused(self):
        a,t=self.make();self.claim(a)
        with self.assertRaises(RuntimeError):self.claim(a)
    def test_watchdog_expires(self):
        a,t=self.make();self.claim(a);t.now=11;a.tick();self.assertTrue(t.aborted);self.assertTrue(a.fault)
    def test_grasp_cannot_return_to_navigation(self):
        a,t=self.make();self.claim(a);a.dispatch({'method':'renew','task_id':'one','stage':'grasp'})
        with self.assertRaises(RuntimeError):a.dispatch({'method':'renew','task_id':'one','stage':'navigation'})
    def test_communication_loss_revokes(self):
        a,t=self.make();self.claim(a)
        with patch('authority.rpc',side_effect=OSError('offline')):a.tick()
        self.assertTrue(t.aborted)
    def test_ack_required(self):
        a,t=self.make();t.await_ack=lambda *args:None
        with self.assertRaises(RuntimeError):self.claim(a)
    def test_navigation_capped(self):
        a,t=self.make();self.claim(a);a.stage='navigation'
        nav={'result':'RUNNING','task_id':'one','command_age':.01,'velocity':[.5,-.5,.5]}
        with patch('authority.rpc',side_effect=[{'errors':[]},nav]):a.tick()
        self.assertEqual([t.messages[-1][k] for k in ('vx','vy','wz')],[.1,-.05,.15])
    def test_grasp_with_navigation_revokes(self):
        a,t=self.make();self.claim(a);a.stage='grasp'
        with patch('authority.rpc',side_effect=[{'errors':[]},{'result':'RUNNING'}]):a.tick()
        self.assertTrue(t.aborted)
if __name__=='__main__':unittest.main()
