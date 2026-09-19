#!/usr/bin/env python3
"""Build paired option 20 maps in a common, rigidly levelled PCD frame.

Unobserved navigation cells stay unknown. A saved PCD contains no sensor rays:
this tool cannot certify traversability, glass, drop-offs, or moving objects.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

FREE, OCCUPIED, UNKNOWN = 254, 0, 205
BANDS = ((1.10, 1.50), (1.50, 1.90), (1.90, 2.30))
RESOLUTION = 0.10
MAX_CELLS = 16000000
GROUND_TOLERANCE = 0.10
FILL_DISTANCE = 0.20


def ground_transform(coefficients):
    """Return rigid original-to-map transform for z=a*x+b*y+c."""
    a, b, c = np.asarray(coefficients, dtype=float)
    normal = np.array([-a, -b, 1.0])
    norm = np.linalg.norm(normal)
    normal /= norm
    # Projection of the original X axis fixes yaw without introducing shear.
    x_axis = np.array([1.0, 0.0, 0.0]) - normal[0] * normal
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    transform = np.eye(4)
    transform[:3, :3] = np.stack((x_axis, y_axis, normal))
    transform[2, 3] = -c / norm
    return transform


def apply_transform(points, transform):
    return np.asarray(points) @ transform[:3, :3].T + transform[:3, 3]


def bounded_ground_fill(ground, occupied, steps):
    """Expand observed ground at most N cells, never through occupied cells."""
    free = ground & ~occupied
    for _ in range(steps):
        expanded = free.copy()
        expanded[1:, :] |= free[:-1, :]
        expanded[:-1, :] |= free[1:, :]
        expanded[:, 1:] |= free[:, :-1]
        expanded[:, :-1] |= free[:, 1:]
        free = expanded & ~occupied
    return free


def build_grids(points, resolution=RESOLUTION, max_cells=MAX_CELLS):
    """Accept levelled finite Nx3 points; arrays use bottom-left row order."""
    points = np.asarray(points)
    if (points.ndim != 2 or points.shape[1] != 3 or not len(points)
            or not np.isfinite(points).all()):
        raise ValueError('map requires finite nonempty Nx3 points')
    if not np.isfinite(resolution) or resolution <= 0 or max_cells < 1:
        raise ValueError('invalid resolution or cell limit')
    ground = np.abs(points[:, 2]) <= GROUND_TOLERANCE
    obstacle = (points[:, 2] >= .12) & (points[:, 2] <= 2.30)
    localization = np.zeros(len(points), dtype=bool)
    for low, high in BANDS:
        localization |= (points[:, 2] >= low) & (points[:, 2] <= high)
    if not ground.any() or not obstacle.any() or not localization.any():
        raise ValueError('ground, obstacle and localization observations are all required')
    selected = ground | obstacle
    xy = points[selected, :2]
    origin = np.floor((xy.min(axis=0) - .5) / resolution) * resolution
    upper = np.ceil((xy.max(axis=0) + .5) / resolution) * resolution
    dimensions = np.ceil((upper - origin) / resolution)
    if (not np.isfinite(dimensions).all() or np.any(dimensions < 1)
            or np.any(dimensions > max_cells)):
        raise ValueError(f'map dimensions exceed {max_cells} cell limit')
    width, height = (int(value) for value in dimensions)
    if width * height > max_cells:
        raise ValueError(f'map {width}x{height} exceeds {max_cells} cell limit')

    def rasterize(mask):
        indices = np.floor((points[mask, :2] - origin) / resolution).astype(np.int64)
        grid = np.zeros((height, width), dtype=bool)
        grid[indices[:, 1], indices[:, 0]] = True
        return grid

    ground_cells = rasterize(ground)
    occupied = rasterize(obstacle)
    loc_occupied = rasterize(localization)
    free = bounded_ground_fill(ground_cells, occupied,
                               int(math.floor(FILL_DISTANCE / resolution + 1e-9)))
    nav = np.full((height, width), UNKNOWN, dtype=np.uint8)
    nav[free] = FREE
    nav[occupied] = OCCUPIED
    loc = np.full_like(nav, FREE)
    loc[loc_occupied] = OCCUPIED
    statistics = {
        'input_points': len(points), 'ground_points': int(ground.sum()),
        'obstacle_points': int(obstacle.sum()),
        'localization_points': int(localization.sum()),
        'observed_ground_cells': int(ground_cells.sum()),
        'navigation_free_cells': int(free.sum()),
        'navigation_occupied_cells': int(occupied.sum()),
        'navigation_unknown_cells': int((nav == UNKNOWN).sum()),
        'localization_occupied_cells': int(loc_occupied.sum()),
        'below_ground_fraction': float(np.mean(points[:, 2] < -.30)),
        'localization_band_points': [int(((points[:, 2] >= low)
                                         & (points[:, 2] <= high)).sum())
                                     for low, high in BANDS],
    }
    return loc, nav, origin, statistics


def prepare(pcd_path, output_dir, floor_z=None):
    import open3d as o3d

    pcd_path = Path(pcd_path).resolve(strict=True)
    output_dir = Path(output_dir).resolve()
    if not pcd_path.is_file() or pcd_path.suffix.lower() != '.pcd':
        raise ValueError('--pcd must name an existing PCD file')
    if floor_z is not None and not np.isfinite(floor_z):
        raise ValueError('--floor-z must be finite')
    source_stat = pcd_path.stat()
    cloud = o3d.io.read_point_cloud(str(pcd_path))
    points = np.asarray(cloud.points)
    if not len(points) or not np.isfinite(points).all():
        raise ValueError('PCD is empty or contains non-finite points')
    # Reuse the repository estimator rather than inventing another floor model.
    helper = Path(__file__).resolve().parents[1] / (
        'src/traymover_robot_nav/scripts/pcd2pgm.py')
    spec = importlib.util.spec_from_file_location('option20_pcd2pgm', helper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    legacy_floor, legacy_method = module.estimate_floor_z(points[:, 2], .02)
    if floor_z is None:
        plane, diagnostics = module.fit_ground_plane(points)
        diagnostics['method'] = 'validated-low-surface-plane'
    else:
        plane = np.array([0., 0., floor_z])
        diagnostics = {'method': 'explicit-horizontal-floor-override'}
    diagnostics = {key: value.item() if isinstance(value, np.generic) else value
                   for key, value in diagnostics.items()}
    diagnostics['histogram_floor_z'] = legacy_floor
    diagnostics['histogram_method'] = legacy_method
    diagnostics['histogram_below_030_fraction'] = float(
        np.mean(points[:, 2] < legacy_floor - .30))
    transform = ground_transform(plane)
    levelled = apply_transform(points, transform)
    loc, nav, origin, statistics = build_grids(levelled)
    if statistics['below_ground_fraction'] > .15:
        raise ValueError('more than 15% of PCD lies below fitted ground; '
                         'inspect multiple levels or a deformed map')
    after_stat = pcd_path.stat()
    if (source_stat.st_size, source_stat.st_mtime_ns) != (
            after_stat.st_size, after_stat.st_mtime_ns):
        raise ValueError('PCD changed during conversion; retry with a stable map')
    manifest = {
        'schema_version': 1, 'pcd_path': str(pcd_path),
        'pcd_size': source_stat.st_size, 'pcd_mtime_ns': source_stat.st_mtime_ns,
        'floor_z': float(plane[2]), 'ground_plane': plane.tolist(),
        'original_to_map': transform.tolist(), 'map_frame': 'map',
        'ground_diagnostics': diagnostics, 'bands': BANDS,
        'resolution': RESOLUTION, 'origin': [float(origin[0]), float(origin[1]), 0.],
        'width': int(nav.shape[1]), 'height': int(nav.shape[0]),
        'statistics': statistics,
        'safety_limits': {
            'max_cells': MAX_CELLS, 'ground_tolerance_m': GROUND_TOLERANCE,
            'ground_fill_m': FILL_DISTANCE, 'obstacle_height_m': [.12, 2.30],
            'localization_inflation_cells': 0,
            'unobserved_navigation_cells': 'unknown',
            'limitations': ['single approximately planar floor only',
                            'PCD endpoints do not prove traversability',
                            'no inferred clearing rays or glass/drop-off detection',
                            'inspect floor coverage and free-space connectivity before driving'],
        },
    }
    manifest_json = json.dumps(manifest, indent=2) + '\n'
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, grid in (('localization', loc), ('navigation', nav)):
        image_path = output_dir / f'{name}.pgm'
        Image.fromarray(np.flipud(grid), mode='L').save(image_path)
        metadata = {'image': image_path.name, 'resolution': RESOLUTION,
                    'origin': manifest['origin'], 'negate': 0,
                    'occupied_thresh': .65, 'free_thresh': .25, 'mode': 'trinary'}
        (output_dir / f'{name}.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    (output_dir / 'manifest.json').write_text(manifest_json)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--floor-z', type=float,
                        help='explicit horizontal floor override in original PCD frame')
    args = parser.parse_args()
    try:
        manifest = prepare(args.pcd, args.output_dir, args.floor_z)
    except (ValueError, OSError) as error:
        parser.exit(2, f'option20 map preparation failed: {error}\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
