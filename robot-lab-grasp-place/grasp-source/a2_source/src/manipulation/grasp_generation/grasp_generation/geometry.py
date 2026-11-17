"""Observable-cloud geometry. Scores are heuristics, not success probabilities."""
import numpy as np
from scipy.spatial.transform import Rotation


def candidates(points, minimum=0.005, maximum=0.075, clearance=0.006):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 30:
        raise ValueError('At least 30 XYZ points required')
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite cloud')
    center = np.median(points, axis=0)
    values, axes = np.linalg.eigh(np.cov((points-center).T))
    if values[-2] < 1e-8:
        raise ValueError('Degenerate cloud')
    axes = axes[:, ::-1]
    axes[:, 2] = np.cross(axes[:, 0], axes[:, 1])
    local = (points-center) @ axes
    low, high = np.percentile(local, [2, 98], axis=0)
    midpoint = center + axes @ ((low+high)/2)
    results = []
    # Tool +Z approaches the object; tool +Y is the closing axis.
    # Sample the visible cloud OBB axes, reject openings outside gripper limits.
    for closing in range(3):
        width = float(high[closing]-low[closing]+clearance)
        if not minimum <= width <= maximum:
            continue
        for approach in range(3):
            if closing == approach:
                continue
            for sign in (-1, 1):
                z = axes[:, approach]*sign
                y = axes[:, closing]
                x = np.cross(y, z)
                rotation = np.column_stack([x, y, z])
                results.append(dict(position=midpoint.copy(), quaternion=Rotation.from_matrix(rotation).as_quat(),
                                    width=width, score=float(1-width/maximum), approach=z.copy()))
    return sorted(results, key=lambda c: c['score'], reverse=True)
