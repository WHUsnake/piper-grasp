"""ROS-independent GraspNet inference and PiPER proposal adaptation (metres)."""
import hashlib
import os
import subprocess
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

AXES = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])


def _configured_path(value, variable):
    expanded = os.path.expandvars(str(value)).strip()
    if not expanded or expanded.startswith('${'):
        raise ValueError(f'Set {variable} to the GraspNet path before starting inference')
    return Path(expanded).expanduser()


class Predictor:
    def __init__(self, config):
        import torch
        g = config
        repo = _configured_path(g['repository'], 'GRASPNET_BASELINE')
        revision = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
        if revision != g['commit']:
            raise ValueError('GraspNet commit mismatch')
        if subprocess.check_output(['git', '-C', str(repo), 'status', '--porcelain'], text=True).strip():
            raise ValueError('GraspNet source is modified')
        checkpoint = _configured_path(g['checkpoint'], 'GRASPNET_CHECKPOINT')
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != g['sha256']:
            raise ValueError('Checkpoint SHA256 mismatch')
        if not torch.cuda.is_available():
            raise RuntimeError('Official compiled operators require CUDA')
        for sub in ['models', 'utils', 'pointnet2', 'knn']:
            sys.path.insert(0, str(repo / sub))
        from graspnet import GraspNet, pred_decode
        torch.manual_seed(0)
        self.torch, self.decode = torch, pred_decode
        self.net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
                            cylinder_radius=.05, hmin=-.02, hmax_list=[.01,.02,.03,.04],
                            is_training=False).cuda()
        ckpt = torch.load(checkpoint, map_location='cuda', weights_only=True)
        self.net.load_state_dict(ckpt['model_state_dict'], strict=True)
        self.net.eval()
        self.num_points = int(g.get('num_points', 20000))
        # Compile/initialize kernels before accepting a live observation.
        self.predict(np.random.default_rng(0).normal(size=(self.num_points, 3)).astype('float32') * .03)

    def predict(self, points):
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('Expected Nx3 cloud')
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) < 30:
            raise ValueError('Insufficient finite target points')
        rng = np.random.default_rng(0)
        if len(points) >= self.num_points:
            idx = rng.choice(len(points), self.num_points, replace=False)
        else:
            idx = np.r_[np.arange(len(points)), rng.choice(len(points), self.num_points-len(points))]
        torch = self.torch
        with torch.inference_mode():
            rows = self.decode(self.net({'point_clouds': torch.as_tensor(points[idx][None], device='cuda')}))[0]
        return rows.cpu().numpy()


def adapt(rows, cloud_to_base_rotation, config):
    """Native finger tip is x=depth. TCP is tip minus tcp_from_tip_m.

    tcp_from_tip_m is a preview assumption until measured on the actual PiPER.
    link6->TCP is deliberately NOT applied here; the existing planner owns it.
    """
    rb = np.asarray(cloud_to_base_rotation, float)
    if rb.shape != (3, 3) or not np.isfinite(rb).all() or not np.allclose(rb.T @ rb, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rb), 1):
        raise ValueError('Invalid acquisition-time base rotation')
    result, counts = [], {}
    def reject(reason): counts[reason] = counts.get(reason, 0) + 1
    for i, row in enumerate(np.asarray(rows)):
        if row.shape != (17,) or not np.isfinite(row).all():
            reject('invalid_row'); continue
        score, width, height, depth = row[:4]
        native = row[4:13].reshape(3, 3)
        if not np.allclose(native.T @ native, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(native), 1, atol=1e-4):
            reject('invalid_rotation'); continue
        opening = float(width + config['opening_clearance_m'])
        if not config['minimum_width_m'] <= opening <= config['maximum_width_m']:
            reject('width'); continue
        if not 0 < depth <= .05 or height <= 0:
            reject('depth_height'); continue
        r = native @ AXES
        base_r = rb @ r
        if abs(base_r[2, 2]) > np.sin(np.deg2rad(config['horizontal_tolerance_deg'])):
            reject('nonhorizontal_approach'); continue
        if abs(base_r[2, 1]) > np.sin(np.deg2rad(config['closing_tolerance_deg'])):
            reject('nonhorizontal_closing'); continue
        position = row[13:16] + native[:, 0] * (depth - config['tcp_from_tip_m'])
        result.append(dict(position=position.tolist(), quaternion=Rotation.from_matrix(r).as_quat().tolist(),
                           width=opening, predicted_width=float(width), score=float(score), depth=float(depth),
                           raw_index=i, rotation=r, pregrasp_distance=config['pregrasp_distance_m']))
    result.sort(key=lambda c: c['score'], reverse=True)
    kept = []
    for c in result:
        if any(np.linalg.norm(np.array(c['position'])-k['position']) < config['nms_translation_m'] and
               (Rotation.from_matrix(k['rotation']).inv()*Rotation.from_matrix(c['rotation'])).magnitude() <
               np.deg2rad(config['nms_rotation_deg']) for k in kept):
            reject('duplicate'); continue
        kept.append(c)
        if len(kept) >= config['max_candidates']: break
    for c in kept: c.pop('rotation')
    return kept, counts
