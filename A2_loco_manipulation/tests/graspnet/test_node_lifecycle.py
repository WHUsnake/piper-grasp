"""Uses real ROS messages, but no ROS nodes, network, worker or hardware."""
import sys
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from grasp_msgs.msg import BoundTargetCloud
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/manipulation/grasp_generation'))
from grasp_generation.node import GraspGenerator

class LifecycleTests(unittest.TestCase):
    def make(self):
        n=GraspGenerator.__new__(GraspGenerator)
        n.pending=None; n.future=None; n.identity=None; n.epoch=0; n.published_header=None
        n.backend='graspnet'; n.cfg={}; self.messages=[]; self.diagnostics=[]
        n.pub=S(publish=self.messages.append); n.status=S(publish=self.diagnostics.append)
        n.get_parameter=lambda k:S(value=.5)
        n.age=lambda h:.1
        return n
    def test_switch_invalidates_inflight(self):
        n=self.make(); n.future=Future(); n.active=(Header(),np.eye(3),0)
        b=BoundTargetCloud(); b.valid=True; b.exporter_session='a'; b.tracker_session='b'; b.track_id=4
        n.bound(b); n.future.set_result({'rows':[]}); n.poll()
        self.assertFalse(self.messages[-1].candidates)
        self.assertIn('superseded_target',self.diagnostics[-1].data)
    def test_stale_input_cancels_pending_and_epoch(self):
        n=self.make(); n.age=lambda h:2
        n.callback(create_cloud_xyz32(Header(frame_id='camera'),np.ones((40,3))))
        self.assertEqual(n.epoch,1); self.assertIsNone(n.pending)
        self.assertIn('stale_or_invalid_input',self.diagnostics[-1].data)
    def test_expired_inference_not_published(self):
        n=self.make(); n.age=lambda h:2
        n.future=Future(); n.active=(Header(),np.eye(3),0); n.future.set_result({'rows':[]})
        n.poll(); self.assertIn('inference_output_expired',self.diagnostics[-1].data)
    def test_cached_candidates_expire_without_new_cloud(self):
        n=self.make(); n.age=lambda h:2; n.published_header=Header()
        n.poll(); self.assertIsNone(n.published_header)
        self.assertIn('published_candidates_expired',self.diagnostics[-1].data)
    def test_failure_clears_candidates(self):
        n=self.make(); n.future=Future(); n.active=(Header(),np.eye(3),0)
        n.future.set_exception(RuntimeError('model failed')); n.poll()
        self.assertFalse(self.messages[-1].candidates)
        self.assertIn('model failed',self.diagnostics[-1].data)

if __name__=='__main__':unittest.main()
