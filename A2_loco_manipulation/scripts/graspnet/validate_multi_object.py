#!/usr/bin/env python3
"""Validate the fixed five-object DC1 -> GraspNet evidence set.

This checker only reads run artifacts. It never starts ROS, MuJoCo, a camera,
CAN, or a robot controller.
"""
import argparse
import json
import sys
from pathlib import Path

EXPECTED = ("bottle", "apple", "small_carton", "can", "cup")
PIPELINE = "ROS2_DC1_DepthComponents_GraspNet_MoveIt_TaskExecutor"


def load_case(root: Path, kind: str) -> dict:
    case = root / f"complex_table_{kind}_20261112" / "web" / "assets"
    metrics_path = case / "metrics.json"
    if not metrics_path.is_file():
        raise ValueError(f"missing metrics.json: {metrics_path}")
    metrics = json.loads(metrics_path.read_text())
    errors = []
    if metrics.get("success") is not True or metrics.get("grasp_success") is not True:
        errors.append("success/grasp_success is not true")
    if metrics.get("target_kind") != kind:
        errors.append(f"target_kind={metrics.get('target_kind')!r}")
    if metrics.get("pipeline") != PIPELINE:
        errors.append(f"pipeline={metrics.get('pipeline')!r}")
    if metrics.get("camera_source") != "dc1_rgbd":
        errors.append(f"camera_source={metrics.get('camera_source')!r}")
    if metrics.get("oracle_segmentation") is not False:
        errors.append("oracle_segmentation must be false")
    if metrics.get("execute_robot") is not False:
        errors.append("execute_robot must be false")
    if metrics.get("multi_objects") is not True:
        errors.append("multi_objects must be true")
    if float(metrics.get("min_hold_lift_m", 0.0)) <= 0.08:
        errors.append("minimum hold lift is <= 8 cm")
    if float(metrics.get("bilateral_hold_fraction", 0.0)) <= 0.90:
        errors.append("bilateral hold fraction is <= 90%")
    objects = metrics.get("depth_objects", {})
    if objects.get("components") != 5 or len(objects.get("objects", [])) != 5:
        errors.append("depth evidence does not contain five components")
    if not isinstance(metrics.get("selected_dc1_pixel"), list) or len(metrics["selected_dc1_pixel"]) != 2:
        errors.append("selected_dc1_pixel is missing")
    if errors:
        raise ValueError(f"{kind}: " + "; ".join(errors))
    return {
        "target_kind": kind,
        "success": True,
        "min_hold_lift_m": metrics["min_hold_lift_m"],
        "bilateral_hold_fraction": metrics["bilateral_hold_fraction"],
        "point_count": metrics.get("point_count"),
        "graspnet_inference_sec": metrics.get("graspnet_inference_sec"),
        "selected_dc1_pixel": metrics["selected_dc1_pixel"],
        "components": objects["components"],
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--runs-root",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "simulation" / "a2_table_grasp" / "runs",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = {"pipeline": PIPELINE, "cases": [], "expected": list(EXPECTED)}
    try:
        for kind in EXPECTED:
            report["cases"].append(load_case(args.runs_root, kind))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        report["success"] = False
        report["error"] = str(exc)
        if args.output:
            args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 1
    report["success"] = True
    if args.output:
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
