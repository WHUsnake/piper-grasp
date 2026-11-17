import cv2
import numpy as np


def masked_cloud(depth, encoding, mask, k, minimum=0.1, maximum=2.5,
                 distortion=(), distortion_model='plumb_bob', min_valid_ratio=0.35):
    if depth.ndim != 2 or mask.shape != depth.shape:
        raise ValueError('Registered depth and mask must have matching dimensions')
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError('Unsupported depth encoding')
    matrix = np.asarray(k).reshape(3, 3)
    if not np.isfinite(matrix).all() or min(matrix[0,0], matrix[1,1]) <= 0:
        raise ValueError('Invalid camera intrinsics')
    coefficients = np.asarray(distortion, dtype=np.float64)
    if coefficients.ndim != 1 or not np.isfinite(coefficients).all():
        raise ValueError('Invalid distortion coefficients')
    if distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
        raise ValueError('Unsupported distortion model: ' + distortion_model)
    allowed = (0, 4, 5) if distortion_model == 'plumb_bob' else (0, 4, 5, 8)
    if len(coefficients) not in allowed or (not distortion_model and np.any(coefficients)):
        raise ValueError('Distortion coefficients do not match model')
    z = depth.astype(float)*(0.001 if encoding == '16UC1' else 1.0)
    mask = cv2.erode(mask.astype(np.uint8), np.ones((3,3), np.uint8)).astype(bool)
    valid = mask & np.isfinite(z) & (z >= minimum) & (z <= maximum)
    if not 0.01 <= min_valid_ratio <= 1.0:
        raise ValueError('min_valid_ratio must be in [0.01, 1.0]')
    if valid.sum() < 30 or valid.sum() < min_valid_ratio*max(1, mask.sum()):
        raise ValueError('Insufficient valid object depth')
    v, u = np.nonzero(valid)
    d = z[v,u]
    # Depth is registered to the raw color grid. Undistort rays, not the depth
    # image: interpolation across object boundaries would mix surfaces.
    pixels = np.column_stack((u, v)).astype(np.float64).reshape(-1, 1, 2)
    rays = cv2.undistortPoints(pixels, matrix.astype(np.float64),
                             coefficients if coefficients.size else None).reshape(-1, 2)
    points = np.column_stack((rays[:, 0]*d, rays[:, 1]*d, d))
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite undistorted points')
    return points[::max(1, int(np.ceil(len(points)/5000)))].astype(np.float32)
