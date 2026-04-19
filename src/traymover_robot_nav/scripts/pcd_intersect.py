#!/usr/bin/env python3
"""Offline tool: intersect multiple FAST-LIO PCDs (from separate mapping
sessions of the same space) to drop anything that only appears in a
minority of runs — i.e. moving people / trailing equipment whose position
differs between passes.

Pipeline:
  1. Read all input PCDs.
  2. Voxelize each to a common grid (default 0.1 m cells).
  3. For each voxel, count how many input PCDs contain it.
  4. Keep voxels whose count >= --min-votes (default: ceil(N/2)+1, i.e. strict
     majority).
  5. Reconstruct an output PCD from kept voxel centers.

Usage:
  ros2 run traymover_robot_nav pcd_intersect.py \
      --pcds map1.pcd map2.pcd map3.pcd \
      --out merged.pcd

**IMPORTANT COORDINATE-FRAME REQUIREMENT**
All input PCDs must be expressed in the *same* coordinate frame.  FAST-LIO
sets its world origin (`camera_init`) at the LiDAR pose at mapping start,
so naively concatenated mapping sessions will have incompatible origins.
To use this tool, either:
  - start every mapping run from the **exact same physical spot + heading**
    (recommended: mark a fixed starting pad, e.g. a charging dock), OR
  - pre-align the PCDs with an ICP / CloudCompare step before intersecting.
A safety check below warns if the vote overlap is very small, which is a
strong hint that the frames are misaligned.

Tip: you may want to run pcd_clean.py (SOR only) on each input first to
remove sparse noise before intersecting.
"""
import argparse
import math
import sys

import numpy as np
import open3d as o3d


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--pcds', nargs='+', required=True,
                   help='2 or more input .pcd files')
    p.add_argument('--out', required=True, help='output .pcd file')
    p.add_argument('--voxel', type=float, default=0.1,
                   help='voxel cell size [m] used for the intersection grid')
    p.add_argument('--min-votes', type=int, default=0,
                   help='minimum vote count to keep a voxel; '
                        '0 = strict majority (ceil(N/2)+1)')
    args = p.parse_args()

    if len(args.pcds) < 2:
        raise SystemExit('need at least 2 input PCDs')

    n = len(args.pcds)
    min_votes = args.min_votes if args.min_votes > 0 else math.ceil(n / 2) + 1
    if min_votes > n:
        raise SystemExit(f'--min-votes {min_votes} > number of inputs {n}')
    print(f'intersecting {n} PCDs with voxel={args.voxel} m, '
          f'min_votes={min_votes}/{n}')

    voxel_counts = {}  # (ix, iy, iz) -> vote count
    for path in args.pcds:
        cloud = o3d.io.read_point_cloud(path)
        pts = np.asarray(cloud.points)
        if pts.size == 0:
            print(f'  WARNING: {path} is empty, skipping', file=sys.stderr)
            continue
        ix = np.floor(pts[:, 0] / args.voxel).astype(np.int64)
        iy = np.floor(pts[:, 1] / args.voxel).astype(np.int64)
        iz = np.floor(pts[:, 2] / args.voxel).astype(np.int64)
        # unique voxel ids present in this PCD
        uniq = set(zip(ix.tolist(), iy.tolist(), iz.tolist()))
        for key in uniq:
            voxel_counts[key] = voxel_counts.get(key, 0) + 1
        print(f'  loaded {path}: {len(pts)} pts -> {len(uniq)} voxels')

    kept = [k for k, v in voxel_counts.items() if v >= min_votes]
    if not kept:
        raise SystemExit('no voxel reached the vote threshold — '
                         'do the inputs cover the same space?')
    retain_ratio = len(kept) / len(voxel_counts)
    print(f'kept {len(kept)} voxels (out of {len(voxel_counts)} total, '
          f'retain ratio {retain_ratio:.1%})')
    if retain_ratio < 0.1:
        print('WARNING: retain ratio < 10%. This usually means the input '
              'PCDs are in different coordinate frames (each FAST-LIO run '
              'sets its own camera_init origin). Either start every mapping '
              'run from the identical physical pose, or pre-align the PCDs '
              'with ICP / CloudCompare.', file=sys.stderr)

    # Reconstruct: use voxel centers.
    arr = np.asarray(kept, dtype=np.float64)
    centers = (arr + 0.5) * args.voxel
    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(centers)
    if not o3d.io.write_point_cloud(args.out, out):
        raise SystemExit(f'failed to write {args.out}')
    print(f'wrote {len(centers)} voxel-center points to {args.out}')


if __name__ == '__main__':
    main()
