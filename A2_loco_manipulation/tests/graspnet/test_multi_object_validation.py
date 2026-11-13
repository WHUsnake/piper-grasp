import json
import importlib.util
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "validate_multi_object", ROOT / "scripts" / "graspnet" / "validate_multi_object.py"
)
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)
load_case = VALIDATOR.load_case


class MultiObjectEvidenceTests(unittest.TestCase):
    def write_metrics(self, root, kind="bottle", **overrides):
        assets = root / f"complex_table_{kind}_20261112" / "web" / "assets"
        assets.mkdir(parents=True)
        metrics = {
            "success": True,
            "grasp_success": True,
            "target_kind": kind,
            "pipeline": "ROS2_DC1_DepthComponents_GraspNet_MoveIt_TaskExecutor",
            "camera_source": "dc1_rgbd",
            "oracle_segmentation": False,
            "execute_robot": False,
            "multi_objects": True,
            "min_hold_lift_m": 0.083,
            "bilateral_hold_fraction": 1.0,
            "selected_dc1_pixel": [100, 120],
            "depth_objects": {"components": 5, "objects": [{"id": i} for i in range(5)]},
        }
        metrics.update(overrides)
        (assets / "metrics.json").write_text(json.dumps(metrics))

    def test_accepts_graspnet_multi_object_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_metrics(root)
            result = load_case(root, "bottle")
            self.assertTrue(result["success"])
            self.assertEqual(result["components"], 5)

    def test_rejects_legacy_oracle_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_metrics(root, oracle_segmentation=True)
            with self.assertRaisesRegex(ValueError, "oracle_segmentation"):
                load_case(root, "bottle")


if __name__ == "__main__":
    unittest.main()
