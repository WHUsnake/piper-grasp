"""Select a tabletop object from registered depth, without semantic labels."""
import numpy as np
from scipy import ndimage


def tabletop_component(depth, intrinsics, rotation_world, translation_world,
                       table_height_world, min_pixels=100, selected_pixel=None):
    z = np.asarray(depth, dtype=np.float32)
    h, w = z.shape
    v, u = np.indices((h, w))
    fx, fy, cx, cy = intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    rotation = np.asarray(rotation_world, dtype=float).reshape(3, 3)
    translation = np.asarray(translation_world, dtype=float)
    world_x = rotation[0, 0] * x + rotation[0, 1] * y + rotation[0, 2] * z + translation[0]
    world_y = rotation[1, 0] * x + rotation[1, 1] * y + rotation[1, 2] * z + translation[1]
    world_z = rotation[2, 0] * x + rotation[2, 1] * y + rotation[2, 2] * z + translation[2]
    foreground = (np.isfinite(z) & (z > .1) & (z < 2.5) &
                  (world_z > table_height_world + .008) &
                  (world_z < table_height_world + .35) &
                  (world_x > .3) & (world_x < 1.2) &
                  (np.abs(world_y) < .5))
    labels, count = ndimage.label(foreground, structure=np.ones((3, 3), dtype=bool))
    sizes = np.bincount(labels.ravel(), minlength=count + 1)
    sizes[0] = 0
    valid = np.flatnonzero(sizes >= min_pixels)
    objects = []
    for i in valid:
        yy, xx = np.where(labels == i)
        objects.append({'id': int(i), 'pixels': int(sizes[i]),
                        'bbox_xyxy': [int(xx.min()), int(yy.min()), int(xx.max()) + 1, int(yy.max()) + 1],
                        'centroid_uv': [float(np.median(xx)), float(np.median(yy))],
                        'centroid_world_m': [float(np.median(axis[labels == i])) for axis in (world_x, world_y, world_z)]})
    if len(valid) == 0:
        return np.zeros_like(foreground), {'components': 0, 'selected_pixels': 0, 'objects': objects}
    if selected_pixel is not None:
        u, v = map(int, selected_pixel)
        if not (0 <= u < w and 0 <= v < h) or labels[v, u] not in valid:
            return np.zeros_like(foreground), {'components': len(objects), 'selected_pixels': 0,
                                                'objects': objects, 'selection_error': 'clicked_pixel_has_no_object'}
        selected = int(labels[v, u])
    else:
        # Single-object default. With several objects require an explicit selection.
        if len(valid) > 1:
            return np.zeros_like(foreground), {'components': len(objects), 'selected_pixels': 0,
                                                'objects': objects, 'selection_error': 'multiple_objects_select_one'}
        selected = int(valid[0])
    mask = labels == selected
    return mask, {'components': len(objects), 'selected_id': selected,
                  'selected_pixels': int(sizes[selected]), 'objects': objects,
                  'centroid_world_m': [float(np.median(axis[mask])) for axis in (world_x, world_y, world_z)]}
