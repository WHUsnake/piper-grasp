import json
import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/manipulation/grasp_generation'))
from grasp_generation.graspnet_backend import adapt, AXES

class AdaptTests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((ROOT/'src/manipulation/grasp_generation/config/graspnet.json').read_text())['adapter']
        self.row=np.r_[.8,.04,.02,.02,np.eye(3).ravel(),[.3,.1,.2],-1.]
    def test_axes_and_depth_reference(self):
        cs,_=adapt([self.row],np.eye(3),self.cfg)
        r=Rotation.from_quat(cs[0]['quaternion']).as_matrix()
        np.testing.assert_allclose(r[:,2],[1,0,0],atol=1e-7)
        np.testing.assert_allclose(r[:,1],[0,1,0],atol=1e-7)
        np.testing.assert_allclose(cs[0]['position'],[.29,.1,.2])
        row=self.row.copy(); row[3]=.04
        cs,_=adapt([row],np.eye(3),self.cfg)
        np.testing.assert_allclose(cs[0]['position'],[.31,.1,.2])
        # No 0.105 m link6 shift here; only the planner applies it.
    def test_horizontal_is_in_base_frame(self):
        rb=Rotation.from_euler('y',90,degrees=True).as_matrix()
        cs,counts=adapt([self.row],rb,self.cfg)
        self.assertFalse(cs); self.assertEqual(counts['nonhorizontal_approach'],1)
    def test_clearance_must_fit_physical_limit(self):
        cfg=dict(self.cfg,opening_clearance_m=.01)
        row=self.row.copy(); row[1]=.07
        self.assertFalse(adapt([row],np.eye(3),cfg)[0])
    def test_invalid_rotation_and_nan_rejected(self):
        row=self.row.copy(); row[4]=-1
        self.assertFalse(adapt([row],np.eye(3),self.cfg)[0])
        row[0]=float('nan')
        self.assertFalse(adapt([row],np.eye(3),self.cfg)[0])
    def test_nms_keeps_highest_score(self):
        row=self.row.copy(); row[0]=.9
        cs,_=adapt([self.row,row],np.eye(3),self.cfg)
        self.assertEqual(len(cs),1); self.assertAlmostEqual(cs[0]['score'],.9)
    def test_pregrasp_is_behind_grasp(self):
        c=adapt([self.row],np.eye(3),self.cfg)[0][0]
        z=Rotation.from_quat(c['quaternion']).as_matrix()[:,2]
        pre=np.array(c['position'])-c['pregrasp_distance']*z
        np.testing.assert_allclose(pre,[.19,.1,.2])
    def test_bad_base_tf_rejected(self):
        with self.assertRaises(ValueError): adapt([self.row],np.zeros((3,3)),self.cfg)

if __name__=='__main__':unittest.main()
