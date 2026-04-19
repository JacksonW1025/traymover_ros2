#!/usr/bin/env python3
"""Offline tool: project a FAST_LIO 3D PCD into a 2D occupancy grid PGM+YAML
consumable by nav2_map_server.

Usage:
    pcd2pgm.py --pcd <in.pcd> --out <out_prefix> \
               [--resolution 0.05] [--z-min 0.15] [--z-max 1.5] \
               [--min-points 1] [--dilate 1]

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


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pcd', required=True, help='input .pcd file')
    p.add_argument('--out', required=True, help='output prefix (no extension)')
    p.add_argument('--resolution', type=float, default=0.05, help='meters per pixel')
    p.add_argument('--z-min', type=float, default=0.15, help='lower z slice [m]')
    p.add_argument('--z-max', type=float, default=1.5, help='upper z slice [m]')
    p.add_argument('--min-points', type=int, default=1,
                   help='min point count per cell to mark occupied')
    p.add_argument('--dilate', type=int, default=1,
                   help='occupancy dilation passes (1-cell structuring element)')
    p.add_argument('--padding', type=float, default=1.0,
                   help='extra free-space border around mapped extent [m]')
    args = p.parse_args()

    cloud = o3d.io.read_point_cloud(args.pcd)
    pts = np.asarray(cloud.points)
    if pts.size == 0:
        raise SystemExit(f'no points in {args.pcd}')
    print(f'loaded {len(pts)} points; z range [{pts[:, 2].min():.2f}, {pts[:, 2].max():.2f}]')

    mask = (pts[:, 2] >= args.z_min) & (pts[:, 2] <= args.z_max)
    slab = pts[mask][:, :2]
    print(f'kept {len(slab)} points in z slab [{args.z_min}, {args.z_max}]')
    if slab.size == 0:
        raise SystemExit('no points survived z filter; adjust --z-min/--z-max')

    x_min, y_min = slab.min(axis=0) - args.padding
    x_max, y_max = slab.max(axis=0) + args.padding
    res = args.resolution
    width = int(np.ceil((x_max - x_min) / res))
    height = int(np.ceil((y_max - y_min) / res))
    print(f'grid: {width}x{height} cells, resolution {res} m, '
          f'origin ({x_min:.2f}, {y_min:.2f})')

    col = np.floor((slab[:, 0] - x_min) / res).astype(np.int32)
    row = np.floor((slab[:, 1] - y_min) / res).astype(np.int32)
    np.clip(col, 0, width - 1, out=col)
    np.clip(row, 0, height - 1, out=row)
    counts = np.zeros((height, width), dtype=np.int32)
    np.add.at(counts, (row, col), 1)

    occ = counts >= args.min_points
    for _ in range(args.dilate):
        occ = dilate3x3(occ)

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


def dilate3x3(grid):
    out = grid.copy()
    out[1:, :] |= grid[:-1, :]
    out[:-1, :] |= grid[1:, :]
    out[:, 1:] |= grid[:, :-1]
    out[:, :-1] |= grid[:, 1:]
    return out


if __name__ == '__main__':
    main()
