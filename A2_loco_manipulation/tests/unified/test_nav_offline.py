"""No ROS node or publisher is constructed by these tests."""
import sys,time,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/unified'))
from nav_adapter import Navigation

class StopTests(unittest.TestCase):
    def make(self):
        a=Navigation.__new__(Navigation);now=time.monotonic()
        a.config={'settle_seconds':2,'feedback_max_age':.5};a.still_since=now-3
        a.odom_at=now;a.cmd_at=now;a.cmd_zero=True
        a.objects_at=now;a.target_at=now;a.status='idle'
        return a
    def test_measured_stop_and_zero_command_required(self):self.assertTrue(self.make().stopped())
    def test_arrival_with_moving_command_fails(self):
        a=self.make();a.status='reached';a.cmd_zero=False;self.assertFalse(a.stopped())
    def test_arrival_with_stale_feedback_fails(self):
        a=self.make();a.status='reached';a.odom_at-=1;self.assertFalse(a.stopped())
    def test_arrival_with_no_command_feedback_fails(self):
        a=self.make();a.status='reached';a.cmd_at-=1;self.assertFalse(a.stopped())
    def test_short_stop_window_fails(self):
        a=self.make();a.still_since=time.monotonic()-.1;self.assertFalse(a.stopped())
    def test_ros_master_response_shape(self):
        a=self.make();services=[('/object_navigation_agent/start',['/agent']),('/object_navigation_agent/cancel',['/agent'])]
        with patch('nav_adapter.rospy.get_master',return_value=S(getSystemState=lambda:[1,'ok',[[],[],services]])):
            self.assertEqual(a.dispatch({'method':'health'})['errors'],[])

if __name__=='__main__':unittest.main()
