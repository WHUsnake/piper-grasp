"""API bounds and geometry consistency. No hardware or ROS."""
import sys
import unittest
from pathlib import Path
import tempfile
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from server import validate_parameters,DEFAULT
from simulate import build,IK
import mujoco

class Parameters(unittest.TestCase):
    def test_default_tcp_is_95_mm(self):self.assertEqual(validate_parameters(DEFAULT)['tcp_m'],.095)
    def test_rejects_nan_and_arbitrary_keys(self):
        for p in [dict(DEFAULT,bottle_x=float('nan')),dict(DEFAULT,table_z=True),dict(DEFAULT,bottle_x=10),dict(DEFAULT,command='oops')]:
            with self.assertRaises(ValueError):validate_parameters(p)
    def test_lateral_bounds_and_old_requests(self):
        old={k:v for k,v in DEFAULT.items() if k!='bottle_y'}
        self.assertEqual(validate_parameters(old)['bottle_y'],0.)
        for side in [-.25,.25]:self.assertEqual(validate_parameters(dict(DEFAULT,bottle_y=side))['bottle_y'],side)
        for side in [-.26,.26]:
            with self.assertRaises(ValueError):validate_parameters(dict(DEFAULT,bottle_y=side))
    def test_lateral_moves_only_bottle(self):
        with tempfile.TemporaryDirectory() as folder:
            for side in [-.12,.12]:
                m=build(Path(folder),bottle_y=side);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
                self.assertAlmostEqual(d.body('bottle').xpos[1],side)
                self.assertEqual(m.geom('table_top').pos[1],0.)
                self.assertEqual(d.body('base_link').xpos[1],0.)
    def test_table_tracks_bottle_and_height(self):
        with tempfile.TemporaryDirectory() as folder:
            m=build(Path(folder),.83,.88,.095);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            np.testing.assert_allclose(m.geom('table_top').pos,[1.03,0,.855],atol=1e-8)
            self.assertAlmostEqual(d.body('bottle').xpos[0],.83)
            self.assertAlmostEqual(d.body('bottle').xpos[2],.985)
            self.assertAlmostEqual(m.geom('table_leg_0_0').pos[2],.415)
            self.assertAlmostEqual(m.geom('table_leg_0_0').size[2],.415)
            np.testing.assert_allclose(m.site('tcp').pos,[0,0,.095])
    def test_tcp_change_does_not_move_robot_mesh_or_fingers(self):
        with tempfile.TemporaryDirectory() as folder:
            a=build(Path(folder),tcp_m=.095);b=build(Path(folder),tcp_m=.105)
            da=mujoco.MjData(a);db=mujoco.MjData(b);mujoco.mj_forward(a,da);mujoco.mj_forward(b,db)
            np.testing.assert_allclose(da.body('p_link7').xpos,db.body('p_link7').xpos)
            self.assertAlmostEqual(np.linalg.norm(da.site('tcp').xpos-db.site('tcp').xpos),.01)

if __name__=='__main__':unittest.main()

class FailureMessages(unittest.TestCase):
    def test_endpoint_search_failure_is_not_declared_proven_unreachable(self):
        from server import failure_message
        import json
        text=failure_message(dict(failure='No candidate passed',planner=json.dumps({'candidates':[{'stage':'grasp_endpoint'}]}),execution_started=False))
        self.assertIn('未找到',text);self.assertIn('不等于证明',text)
    def test_stale_observation_stays_refused(self):
        from server import failure_message
        text=failure_message(dict(failure='No fresh matching observation after planning',execution_started=False))
        self.assertIn('本次未执行',text)
