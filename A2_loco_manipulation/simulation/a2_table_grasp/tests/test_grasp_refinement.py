import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from grasp_refinement import fit_bottle,refine_candidates,PAD_CENTER_FROM_LINK6_M

class RefinementTests(unittest.TestCase):
    def setUp(self):
        angle,z=np.meshgrid(np.linspace(.5,2.5,60),np.linspace(.6,.82,50))
        self.points=np.c_[.61+.026*np.cos(angle.ravel()),.026*np.sin(angle.ravel()),z.ravel()]
        self.r=np.array([[0,0,1],[0,1,0],[-1,0,0.]])
        self.c=dict(position=[.57,0,.65],rotation_world=self.r.tolist(),depth=.02,score=.8,raw_index=1,width=.052)
    def test_partial_surface_fit(self):
        f=fit_bottle(self.points)
        np.testing.assert_allclose(f['axis_xy'],[.61,0],atol=1e-7)
        self.assertAlmostEqual(f['radius_m'],.026,places=7)
    def test_contact_is_middle_and_deeper_than_tip(self):
        cs,f=refine_candidates([self.c],self.points,.095);c=cs[0]
        self.assertAlmostEqual(c['height_fraction'],.5)
        contact=np.array(c['contact_center_world']);link=np.array(c['position'])-self.r[:,2]*.095
        local=self.r.T@(contact-link)
        np.testing.assert_allclose(local,[0,0,PAD_CENTER_FROM_LINK6_M],atol=1e-7)
        self.assertGreater(c['tip_margin_target_m'],.025)
        self.assertEqual(self.c['position'],[.57,0,.65])
    def test_tcp_change_preserves_physical_pad_target(self):
        a=refine_candidates([self.c],self.points,.085)[0][0]
        shifted=dict(self.c,position=(np.array(self.c['position'])+self.r[:,2]*.01).tolist())
        b=refine_candidates([shifted],self.points,.095)[0][0]
        np.testing.assert_allclose(np.array(a['position'])-self.r[:,2]*.085,np.array(b['position'])-self.r[:,2]*.095,atol=1e-8)
    def test_invalid_observation_rejected(self):
        with self.assertRaises(ValueError):fit_bottle(np.zeros((10,3)))


    def test_opening_matches_fitted_body_after_geometry_adaptation(self):
        c=refine_candidates([self.c],self.points,.095)[0][0]
        self.assertAlmostEqual(c['geometry_opening_m'],.064,places=6)
        self.assertGreaterEqual(c['width'],c['geometry_opening_m'])
        self.assertEqual(c['network_opening_m'],.052)

if __name__=='__main__':unittest.main()
