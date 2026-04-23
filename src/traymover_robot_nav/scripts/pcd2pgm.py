#!/usr/bin/env python3
"""Offline tool: project a FAST_LIO 3D PCD into a 2D occupancy grid PGM+YAML
consumable by nav2_map_server.

Usage:
    pcd2pgm.py --pcd <in.pcd> --out <out_prefix> \
               [--resolution 0.05] [--z-min 0.10] [--z-max 2.20] \
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


def occupancy_from_points(slab_xy, resolution, padding, min_points):
    x_min, y_min = slab_xy.min(axis=0) - padding
    x_max, y_max = slab_xy.max(axis=0) + padding
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


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pcd', required=True, help='input .pcd file')
    p.add_argument('--out', required=True, help='output prefix (no extension)')
    p.add_argument('--resolution', type=float, default=0.05, help='meters per pixel')
    p.add_argument('--z-min', type=float, default=0.10,
                   help='lower slice height above estimated floor [m]')
    p.add_argument('--z-max', type=float, default=2.20,
                   help='upper slice height above estimated floor [m]')
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
    args = p.parse_args()

    cloud = o3d.io.read_point_cloud(args.pcd)
    pts = np.asarray(cloud.points)
    if pts.size == 0:
        raise SystemExit(f'no points in {args.pcd}')
    print(f'loaded {len(pts)} points; z range [{pts[:, 2].min():.2f}, {pts[:, 2].max():.2f}]')

    floor_z, floor_method = estimate_floor_z(
        pts[:, 2],
        fallback_quantile=args.floor_quantile,
        low_quantile=args.floor_low_quantile,
        high_quantile=args.floor_high_quantile,
        bin_size=args.floor_bin_size,
    )
    slab_z_min = floor_z + args.z_min
    slab_z_max = floor_z + args.z_max
    print(
        f'estimated floor z={floor_z:.2f} using {floor_method} '
        f'(fallback_quantile={args.floor_quantile:.3f}, '
        f'band=[{args.floor_low_quantile:.3f}, {args.floor_high_quantile:.3f}], '
        f'bin={args.floor_bin_size:.2f} m)'
    )

    mask = (pts[:, 2] >= slab_z_min) & (pts[:, 2] <= slab_z_max)
    slab = pts[mask][:, :2]
    print(f'kept {len(slab)} points in floor-relative slab [{slab_z_min:.2f}, {slab_z_max:.2f}]')
    if slab.size == 0:
        raise SystemExit('no points survived z filter; adjust --z-min/--z-max')

    res = args.resolution
    occ, x_min, y_min, width, height = occupancy_from_points(
        slab_xy=slab,
        resolution=res,
        padding=args.padding,
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
