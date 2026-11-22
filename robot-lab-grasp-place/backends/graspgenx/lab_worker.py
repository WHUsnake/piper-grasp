"""GraspGenX adapter for the Lab's isolated JSON-lines inference protocol."""
import contextlib
import json
from pathlib import Path
import sys
import time

import numpy as np
import torch

# PiPER asset closes along X, while Lab closes along Y; both approach along +Z.
ASSET_TO_TOOL_AXES = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
GRASPNET_TO_TOOL_AXES = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])


def adapt(poses, scores, points, tcp_m, tip_offset):
    rows = []
    rejected = {'nonfinite': 0, 'invalid_rotation': 0, 'insufficient_pad_points': 0}
    for pose, score in zip(poses, scores):
        if not np.isfinite(pose).all() or not np.isfinite(score):
            rejected['nonfinite'] += 1
            continue
        r = pose[:3, :3]
        origin = pose[:3, 3]
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(r), 1):
            rejected['invalid_rotation'] += 1
            continue
        local = (points - origin) @ r
        # Estimate the opening from the observed points near the finger pads.
        band = local[(abs(local[:, 1]) < .025) & (abs(local[:, 2] - tcp_m) < .027), 0]
        if len(band) < 15:
            rejected['insufficient_pad_points'] += 1
            continue
        width = 2 * np.quantile(abs(band), .95) + .004
        depth = .03
        native = r @ ASSET_TO_TOOL_AXES @ GRASPNET_TO_TOOL_AXES.T
        tcp = origin + r[:, 2] * tcp_m
        translation = tcp - native[:, 0] * (depth - tip_offset)
        rows.append([float(score), width, .02, depth, *native.ravel(), *translation, -1])
    return rows, {key: value for key, value in rejected.items() if value}


def main():
    config = json.loads(Path(sys.argv[1]).read_text())
    root = Path(config['model']['graspgenx_root'])
    ckpt = root / 'ext/graspgenx_checkpoints/release'
    tcp = config['refinement']['tcp_m']
    with contextlib.redirect_stdout(sys.stderr):
        from graspgenx.grasp_server import GraspGenXSampler
        from graspgenx.utils.checkpoint_io import load_model_cfg
        torch.manual_seed(0)
        sampler = GraspGenXSampler(load_model_cfg(ckpt/'gen', ckpt/'dis'), 'piper_hand',
                                   assets_dir=str(root/'assets'))
        with torch.inference_mode():
            sample = np.random.default_rng(0).normal(0, .02, (3500, 3)).astype(np.float32)
            sampler.run_inference(sample, sampler, max_tries=1, remove_outliers=False)
            torch.cuda.synchronize()
    print(json.dumps({'ready': True}), flush=True)
    for line in sys.stdin:
        try:
            item = json.loads(line)
            points = np.asarray(item['points'], dtype=np.float32)
            if points.ndim != 2 or points.shape[1] != 3 or len(points) < 30 or not np.isfinite(points).all():
                raise ValueError('Invalid target point cloud')
            start = time.monotonic()
            with contextlib.redirect_stdout(sys.stderr), torch.inference_mode():
                poses, scores = sampler.run_inference(points, sampler, max_tries=1, remove_outliers=False)
                rows, rejected = adapt(poses.cpu().numpy(), scores.cpu().numpy(), points, tcp,
                                       config['adapter']['tcp_from_tip_m'])
            reply = dict(id=item['id'], rows=rows, raw_model_poses=len(poses),
                         worker_adapter_rejected=rejected, inference_sec=time.monotonic()-start)
        except Exception as exc:
            reply = dict(error=str(exc))
        print(json.dumps(reply, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
