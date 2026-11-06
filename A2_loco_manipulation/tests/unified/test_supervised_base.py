import sys,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/unified'))
from supervised_base import Bridge
class T:
 def __init__(self):self.messages=[];self.other=[]
 def conflicts(self):return self.other
 def driver_ready(self):return True
 def mode(self,value):self.mode_value=value
 def publish(self,v):self.messages.append(v)
class Tests(unittest.TestCase):
 def make(self):
  t=T();a=Bridge(t,'/unused',lambda:10.);return a,t
 def claim(self,a):a.dispatch({'method':'claim','task_id':'x'})
 def nav(self,a):a.dispatch({'method':'renew','task_id':'x','stage':'navigation'})
 def replies(self,nav):return [{'errors':[]},{'received':10.,'age':0.},nav]
 def test_startup_no_output(self):
  a,t=self.make();a.tick();self.assertEqual(t.messages,[])
 def test_claim_no_output_no_false_exclusive(self):
  a,t=self.make();r=a.dispatch({'method':'claim','task_id':'x'});self.assertFalse(r['exclusive']);self.assertEqual(t.messages,[])
 def test_other_controller_refused(self):
  a,t=self.make();t.other=['remote']
  with self.assertRaises(RuntimeError):self.claim(a)
 def test_approach_forwards_existing_velocity_capped(self):
  a,t=self.make();self.claim(a);self.nav(a)
  n={'result':'RUNNING','task_id':'x','command_age':0.,'mode_request':'lingdong','mode_age':0.,'velocity':[.5,0,.5]}
  with patch('supervised_base.rpc',side_effect=self.replies(n)):a.tick()
  self.assertEqual(t.messages,[(.1,0.,.15)])
 def test_transition_to_grasp_sends_zero(self):
  a,t=self.make();self.claim(a);self.nav(a);a.sent=True
  a.dispatch({'method':'renew','task_id':'x','stage':'grasp'});self.assertEqual(t.messages[-1],(0.,0.,0.))
  with self.assertRaises(RuntimeError):self.nav(a)
 def test_disconnect_stops_no_retry(self):
  a,t=self.make();self.claim(a);self.nav(a);a.sent=True
  with patch('supervised_base.rpc',side_effect=OSError('lost')):a.tick()
  self.assertTrue(a.fault);self.assertEqual(t.messages[-1],(0.,0.,0.))
 def test_moving_during_grasp_aborts(self):
  a,t=self.make();self.claim(a);a.stage='grasp'
  replies=self.replies({'result':'ARRIVED','stopped':False})+[{},{}]
  with patch('supervised_base.rpc',side_effect=replies):a.tick()
  self.assertTrue(a.fault)
 def test_stale_nav_refused(self):
  a,t=self.make();self.claim(a);self.nav(a)
  n={'result':'RUNNING','task_id':'x','command_age':1,'velocity':[.1,0,0]}
  with patch('supervised_base.rpc',side_effect=self.replies(n)+[{},{}]):a.tick()
  self.assertTrue(a.fault);self.assertEqual(t.messages,[])
 def test_heartbeat_expiry_requests_zero(self):
  a,t=self.make();self.claim(a);a.sent=True;a.renewed=0
  with patch('supervised_base.rpc',return_value={}):a.tick()
  self.assertTrue(a.fault);self.assertEqual(t.messages[-1],(0.,0.,0.))
if __name__=='__main__':unittest.main()
