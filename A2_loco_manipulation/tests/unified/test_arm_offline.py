"""Run in the project's ROS2 environment. No node, SDK, CAN or service is created."""
import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/unified'))
from arm_adapter import Arm

def msg(stamp):
    return S(header=S(stamp=S(sec=int(stamp),nanosec=int((stamp-int(stamp))*1e9)),frame_id='camera'),candidates=[1])

class ArmOfflineTests(unittest.TestCase):
    def make(self):
        a=Arm.__new__(Arm);a.cutoff=99.;a.bound_at=time.monotonic();a.bound_stamp=0.
        a.n=S(get_clock=lambda:S(now=lambda:S(nanoseconds=100_000_000_000)))
        a.seen={t:(time.monotonic(),msg(99.8)) for t in ['/camera/color/image_raw','/camera/depth/image_raw','/grasp/candidates']}
        return a
    def test_old_acquisition_cannot_execute(self):
        a=self.make();a.seen['/grasp/candidates']=(time.monotonic(),msg(98))
        self.assertFalse(a.dispatch({'method':'fresh'})['valid'])
    def test_pre_stop_frame_cannot_execute(self):
        a=self.make();a.cutoff=99.9
        self.assertFalse(a.dispatch({'method':'fresh'})['valid'])
    def test_future_frame_cannot_execute(self):
        a=self.make();a.seen['/camera/depth/image_raw']=(time.monotonic(),msg(101))
        self.assertFalse(a.dispatch({'method':'fresh'})['valid'])
    def test_missing_identity_cloud_cannot_execute(self):
        a=self.make();a.bound_at=0
        self.assertFalse(a.dispatch({'method':'fresh'})['valid'])
    def test_only_new_fresh_bound_frames_pass(self):
        self.assertTrue(self.make().dispatch({'method':'fresh'})['valid'])
    def test_graspnet_preview_cannot_enable(self):
        a=self.make();a.seen['/grasp/candidates'][1].candidates=[S(generator='graspnet_baseline_preview')]
        with self.assertRaisesRegex(RuntimeError,'preview only'):a.dispatch({'method':'enable'})
    def test_graspnet_preview_remains_blocked_after_candidate_replaced(self):
        a=self.make();a.preview_graspnet=True
        a.seen['/grasp/candidates'][1].candidates=[S(generator='visible_cloud_pca_obb')]
        with self.assertRaisesRegex(RuntimeError,'preview only'):a.dispatch({'method':'start'})
    def test_wrong_bottle_not_forwarded(self):
        a=self.make();a.identity={'tracker_session':'one','track_id':5,'class_id':39};calls=[];a.cloud=S(publish=calls.append)
        m=msg(99.8);m.valid=True;m.tracker_session='two';m.track_id=5;m.class_id=39
        a.bound_cb(m);self.assertEqual(calls,[])

if __name__=='__main__':unittest.main()
