#!/usr/bin/env python3
"""Offline tool: project a FAST_LIO 3D PCD into a 2D occupancy grid PGM+YAML
consumable by nav2_map_server.

Usage:
    pcd2pgm.py --pcd <in.pcd> --out <out_prefix> \
               [--resolution 0.05] [--z-min 0.10] [--z-max 2.20] \
               [--z-reference floor|plane|absolute] \
               [--floor-quantile 0.02] [--min-points 1] [--dilate 2] \
               [--min-region-cells 2]

Outputs <out_prefix>.pgm and <out_prefix>.yaml.
"""
import argparse
import os
import numpy as np
import open3d as o3d
import yaml
from PIL import Image

FREE_VAL = 254
OCC_VAL = 0
# In trinary mode this is between occupied_thresh=0.65 and free_thresh=0.25.
UNKNOWN_VAL = 128


def dilate3x3(grid):
    out = grid.copy()
    out[1:, :] |= grid[:-1, :]
    out[:-1, :] |= grid[1:, :]
    out[:, 1:] |= grid[:, :-1]
    out[:, :-1] |= grid[:, 1:]
    return out


def remove_small_components(grid, min_region_cells):
    """Drop 4-connected occupied islands smaller than min_region_cells."""
    if min_region_cells <= 1:
        return grid

    grid = grid.copy()
    height, width = grid.shape
    visited = np.zeros_like(grid, dtype=bool)

    for row in range(height):
        for col in range(width):
            if not grid[row, col] or visited[row, col]:
                continue

            stack = [(row, col)]
            component = []
            visited[row, col] = True

            while stack:
                r, c = stack.pop()
                component.append((r, c))
                for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                    if nr < 0 or nr >= height or nc < 0 or nc >= width:
                        continue
                    if visited[nr, nc] or not grid[nr, nc]:
                        continue
                    visited[nr, nc] = True
                    stack.append((nr, nc))

            if len(component) < min_region_cells:
                for r, c in component:
                    grid[r, c] = False

    return grid


def estimate_floor_z(z_values, fallback_quantile, low_quantile=0.005,
                     high_quantile=0.30, bin_size=0.05):
    """Estimate the floor height from a noisy FAST-LIO map.

    Low outliers several meters below the real floor are common in saved PCDs.
    A raw low-quantile estimate then slices the wrong height band and produces
    a 2D map that no longer matches the 3D map used by NDT.

    Strategy:
    1. Trim z into a low-but-not-extreme band.
    2. Build a histogram in that band.
    3. Pick the densest z slice and use its median as floor.
    4. Fall back to the legacy quantile only if the trimmed band is too sparse.
    """
    z_values = np.asarray(z_values, dtype=float)
    if z_values.size == 0:
        raise ValueError('cannot estimate floor from an empty z array')

    legacy_floor = float(np.quantile(z_values, fallback_quantile))
    z_low = float(np.quantile(z_values, low_quantile))
    z_high = float(np.quantile(z_values, high_quantile))
    if not np.isfinite(z_low) or not np.isfinite(z_high) or z_high <= z_low:
        return legacy_floor, 'quantile-fallback'

    trimmed = z_values[(z_values >= z_low) & (z_values <= z_high)]
    if trimmed.size < 50:
        return legacy_floor, 'quantile-fallback'

    bins = np.arange(trimmed.min(), trimmed.max() + bin_size, bin_size)
    if bins.size < 2:
        return float(np.median(trimmed)), 'median-fallback'

    hist, edges = np.histogram(trimmed, bins=bins)
    best = int(hist.argmax())
    band_min = edges[best]
    band_max = edges[best + 1]
    band = trimmed[(trimmed >= band_min) & (trimmed < band_max)]
    if band.size == 0:
        return legacy_floor, 'quantile-fallback'

    return float(np.median(band)), 'histogram-mode'


def height_above_plane(points, coefficients):
    """Signed normal height above z = a*x + b*y + c; preserve map XY."""
    points = np.asarray(points, dtype=float)
    a, b, c = coefficients
    return (points[:, 2] - a * points[:, 0] - b * points[:, 1] - c) / np.sqrt(
        1.0 + a * a + b * b)


def fit_ground_plane(points, cell_size=5.0, distance_threshold=0.15,
                     max_tilt_deg=15.0, max_samples=500000, iterations=1000):
    """Fit a deterministic low-surface plane, rejecting weak/roof support.

    A fitted reference is a geometric estimate, not proof of traversable ground.
    Fail closed when low observations contradict the dominant surface.
    """
    if (not np.isfinite(cell_size) or cell_size <= 0
            or not np.isfinite(distance_threshold) or distance_threshold <= 0
            or not np.isfinite(max_tilt_deg) or not 0 < max_tilt_deg < 90
            or not isinstance(max_samples, (int, np.integer)) or max_samples < 20
            or not isinstance(iterations, (int, np.integer)) or iterations < 1):
        raise ValueError('invalid ground-plane fitting parameters')
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('ground-plane fitting requires Nx3 points')
    if len(points) == 0 or not np.isfinite(points).all():
        raise ValueError('ground-plane fitting requires finite nonempty points')
    rng = np.random.default_rng(20260917)
    sample = points
    if len(points) > max_samples:
        sample = points[rng.choice(len(points), max_samples, replace=False)]
    cells = np.floor((sample[:, :2] - sample[:, :2].min(axis=0)) / cell_size)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    order = np.argsort(inverse, kind='stable')
    groups = np.split(order, np.flatnonzero(np.diff(inverse[order])) + 1)
    anchors = []
    for group in groups:
        if len(group) < 20:
            continue
        local = sample[group]
        low, high = np.quantile(local[:, 2], [0.10, 0.20])
        band = local[(local[:, 2] >= low) & (local[:, 2] <= high)]
        if len(band):
            anchors.append(np.mean(band, axis=0))
    anchors = np.asarray(anchors)
    if len(anchors) < 12:
        raise ValueError('ground-plane fit needs at least 12 populated ground cells')
    xy_center = anchors[:, :2].mean(axis=0)
    xy_scale = anchors[:, :2].std(axis=0)
    if np.any(xy_scale < 1e-8):
        raise ValueError('ground-plane fit lacks two-dimensional spatial support')
    design = np.column_stack(((anchors[:, :2] - xy_center) / xy_scale,
                              np.ones(len(anchors))))

    def map_coefficients(normalized):
        slopes = normalized[:2] / xy_scale
        return np.array([slopes[0], slopes[1], normalized[2] - slopes @ xy_center])

    slope_limit = np.tan(np.deg2rad(max_tilt_deg))
    required = max(12, int(np.ceil(0.35 * len(anchors))))
    best = None
    best_score = (-1, -np.inf)
    for _ in range(iterations):
        chosen = rng.choice(len(anchors), 3, replace=False)
        if np.linalg.matrix_rank(design[chosen]) < 3:
            continue
        coefficients = map_coefficients(
            np.linalg.solve(design[chosen], anchors[chosen, 2]))
        if np.linalg.norm(coefficients[:2]) > slope_limit:
            continue
        signed_residual = height_above_plane(anchors, coefficients)
        if np.mean(signed_residual < -2 * distance_threshold) > 0.15:
            continue
        residual = np.abs(signed_residual)
        inliers = residual <= distance_threshold
        score = (int(inliers.sum()), -float(np.median(residual[inliers])))
        if score > best_score:
            best, best_score = inliers, score
    if best is None or best.sum() < required:
        raise ValueError('ground-plane fit lacks sufficient low-surface support')
    for _ in range(5):
        coefficients = map_coefficients(
            np.linalg.lstsq(design[best], anchors[best, 2], rcond=None)[0])
        inliers = np.abs(height_above_plane(anchors, coefficients)) <= distance_threshold
        if inliers.sum() < required:
            raise ValueError('ground-plane refinement lost low-surface support')
        if np.array_equal(inliers, best):
            break
        best = inliers
    coefficients = map_coefficients(
        np.linalg.lstsq(design[best], anchors[best, 2], rcond=None)[0])
    final = np.abs(height_above_plane(anchors, coefficients)) <= distance_threshold
    if final.sum() < required or np.linalg.norm(coefficients[:2]) > slope_limit:
        raise ValueError('ground-plane final fit is unsupported or too steep')
    support = anchors[final, :2]
    spread = np.linalg.svd(support - support.mean(axis=0), compute_uv=False)
    if len(spread) < 2 or spread[1] / np.sqrt(len(support)) < cell_size * 0.5:
        raise ValueError('ground-plane fit lacks two-dimensional spatial support')
    below = height_above_plane(sample, coefficients) < -2 * distance_threshold
    populated = [group for group in groups if len(group) >= 20]
    conflicts = sum(below[group].sum() >= max(5, int(np.ceil(0.03 * len(group))))
                    for group in populated)
    if conflicts / len(populated) > 0.15:
        raise ValueError('ground-plane fit conflicts with lower observations (possible roof)')
    return coefficients, {
        'anchor_count': len(anchors),
        'inlier_count': int(final.sum()),
        'tilt_deg': float(np.rad2deg(np.arctan(np.linalg.norm(coefficients[:2])))),
        'conflicting_cells': conflicts,
    }


def resolve_z_bounds(z_reference, z_min, z_max, floor_z=None):
    if z_min >= z_max:
        raise ValueError('z-min must be lower than z-max')
    if z_reference == 'absolute':
        return float(z_min), float(z_max)
    if z_reference == 'floor':
        if floor_z is None:
            raise ValueError('floor-relative Z bounds require an estimated floor')
        return float(floor_z + z_min), float(floor_z + z_max)
    raise ValueError(f'unsupported Z reference: {z_reference}')


def occupancy_from_points(slab_xy, resolution, padding, min_points, extent_xy=None):
    extent = slab_xy if extent_xy is None else extent_xy
    x_min, y_min = extent.min(axis=0) - padding
    x_max, y_max = extent.max(axis=0) + padding
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))

    col = np.floor((slab_xy[:, 0] - x_min) / resolution).astype(np.int32)
    row = np.floor((slab_xy[:, 1] - y_min) / resolution).astype(np.int32)
    np.clip(col, 0, width - 1, out=col)
    np.clip(row, 0, height - 1, out=row)
    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (row, col), 1)

    occ = counts >= min_points
    return occ, x_min, y_min, width, height


def observed_ground(xy, shape, origin, resolution):
    """Allow small interpolation gaps between returns near the estimated floor."""
    from scipy.ndimage import distance_transform_edt, uniform_filter

    cells = np.floor((xy - origin) / resolution).astype(np.intp)
    x, y = cells.T
    inside = (x >= 0) & (y >= 0) & (x < shape[1]) & (y < shape[0])
    anchors = np.zeros(shape, dtype=bool)
    anchors[y[inside], x[inside]] = True
    width = 2 * int(np.ceil(0.18 / resolution)) + 1
    neighbors = uniform_filter(anchors.astype(float), size=width, mode='constant') * width**2
    anchors &= neighbors >= 2.5
    if not anchors.any():
        raise ValueError('PCD has no clustered ground returns for the option 22 map')
    return distance_transform_edt(~anchors, sampling=resolution) <= 0.32


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pcd', required=True, help='input .pcd file')
    p.add_argument('--out', required=True, help='output prefix (no extension)')
    p.add_argument('--resolution', type=float, default=0.05, help='meters per pixel')
    p.add_argument('--z-reference', choices=('floor', 'plane', 'absolute'), default='floor',
                   help='interpret --z-min/--z-max relative to the estimated floor '
                        'or fitted plane, or as absolute PCD map-frame heights')
    p.add_argument('--z-min', type=float, default=0.10,
                   help='lower Z slice bound [m], interpreted by --z-reference')
    p.add_argument('--z-max', type=float, default=2.20,
                   help='upper Z slice bound [m], interpreted by --z-reference')
    p.add_argument('--floor-quantile', type=float, default=0.02,
                   help='fallback quantile used if robust floor estimation fails')
    p.add_argument('--floor-low-quantile', type=float, default=0.005,
                   help='lower quantile bound for robust floor estimation band')
    p.add_argument('--floor-high-quantile', type=float, default=0.30,
                   help='upper quantile bound for robust floor estimation band')
    p.add_argument('--floor-bin-size', type=float, default=0.05,
                   help='histogram bin size [m] used by robust floor estimation')
    p.add_argument('--min-points', type=int, default=1,
                   help='min point count per cell to mark occupied')
    p.add_argument('--dilate', type=int, default=2,
                   help='occupancy dilation passes (1-cell structuring element)')
    p.add_argument('--min-region-cells', type=int, default=2,
                   help='drop occupied 4-connected islands smaller than this many cells')
    p.add_argument('--padding', type=float, default=1.0,
                   help='extra free-space border around mapped extent [m]')
    p.add_argument('--observed-ground-only', action='store_true',
                   help='mark areas without nearby ground returns as unknown')
    args = p.parse_args()

    cloud = o3d.io.read_point_cloud(args.pcd)
    pts = np.asarray(cloud.points)
    if pts.size == 0:
        raise SystemExit(f'no points in {args.pcd}')
    print(f'loaded {len(pts)} points; z range [{pts[:, 2].min():.2f}, {pts[:, 2].max():.2f}]')

    floor_z = None
    if args.z_reference == 'floor':
        floor_z, floor_method = estimate_floor_z(
            pts[:, 2],
            fallback_quantile=args.floor_quantile,
            low_quantile=args.floor_low_quantile,
            high_quantile=args.floor_high_quantile,
            bin_size=args.floor_bin_size,
        )
        print(
            f'estimated floor z={floor_z:.2f} using {floor_method} '
            f'(fallback_quantile={args.floor_quantile:.3f}, '
            f'band=[{args.floor_low_quantile:.3f}, {args.floor_high_quantile:.3f}], '
            f'bin={args.floor_bin_size:.2f} m)'
        )

    try:
        if args.z_reference == 'plane':
            slab_z_min, slab_z_max = resolve_z_bounds('absolute', args.z_min, args.z_max)
            if not np.isfinite([slab_z_min, slab_z_max]).all():
                raise ValueError('plane height bounds must be finite')
            plane, diagnostics = fit_ground_plane(pts)
            heights = height_above_plane(pts, plane)
            print(f'estimated ground plane z={plane[0]:.6f}*x '
                  f'+ {plane[1]:.6f}*y + {plane[2]:.6f}; {diagnostics}')
        else:
            slab_z_min, slab_z_max = resolve_z_bounds(
                args.z_reference, args.z_min, args.z_max, floor_z)
            heights = pts[:, 2]
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    mask = (heights >= slab_z_min) & (heights <= slab_z_max)
    slab = pts[mask][:, :2]
    print(
        f'kept {len(slab)} points in {args.z_reference} Z slab '
        f'[{slab_z_min:.2f}, {slab_z_max:.2f}]'
    )
    extent = None
    if args.observed_ground_only:
        if args.z_reference == 'absolute':
            raise SystemExit('--observed-ground-only requires a floor or plane reference')
        ground_heights = heights - floor_z if args.z_reference == 'floor' else heights
        ground = pts[np.abs(ground_heights) <= 0.18, :2]
        if len(ground) < 3:
            raise SystemExit('PCD has too few ground returns for the option 22 map')
        extent = np.vstack((slab, ground))
    elif slab.size == 0:
        raise SystemExit('no points survived z filter; adjust --z-min/--z-max')

    res = args.resolution
    occ, x_min, y_min, width, height = occupancy_from_points(
        slab_xy=slab,
        resolution=res,
        padding=args.padding, extent_xy=extent,
        min_points=args.min_points,
    )
    print(f'grid: {width}x{height} cells, resolution {res} m, '
          f'origin ({x_min:.2f}, {y_min:.2f})')
    occ_before = int(occ.sum())
    occ = remove_small_components(occ, args.min_region_cells)
    occ_after = int(occ.sum())
    if occ_before != occ_after:
        print(f'removed {occ_before - occ_after} occupied cells in small islands '
              f'(< {args.min_region_cells} cells)')
    for _ in range(args.dilate):
        occ = dilate3x3(occ)
    occ_final = int(occ.sum())
    print(f'occupied cells after cleanup+dilate: {occ_final}')

    img = np.full((height, width), FREE_VAL, dtype=np.uint8)
    if args.observed_ground_only:
        try:
            supported = observed_ground(ground, occ.shape, [x_min, y_min], res)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
        img[~supported] = UNKNOWN_VAL
        print(f'ground support: {np.count_nonzero(supported & ~occ)} free cells, '
              f'{np.count_nonzero(~supported & ~occ)} unknown cells')
    img[occ] = OCC_VAL
    img = np.flipud(img)  # PGM: row 0 is top; map YAML origin is bottom-left

    pgm_path = args.out + '.pgm'
    yaml_path = args.out + '.yaml'
    Image.fromarray(img, mode='L').save(pgm_path)

    meta = {
        'image': os.path.basename(pgm_path),
        'resolution': float(res),
        'origin': [float(x_min), float(y_min), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    print(f'wrote {pgm_path} and {yaml_path}')


if __name__ == '__main__':
    main()
