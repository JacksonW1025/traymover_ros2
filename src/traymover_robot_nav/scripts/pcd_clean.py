#!/usr/bin/env python3
"""Offline tool: clean a FAST-LIO PCD of dynamic-object ghosts (people,
trailing personnel, briefly-present items) so the static map used for
localization and 2D-grid generation stays clean.

Pipeline (each stage independently toggleable):
  1. Statistical Outlier Removal (SOR)  — removes isolated sparse points.
     Set --sor-nb 0 to skip.
  2. Trajectory-corridor sparse-cell filter — optional. If you provide the
     vehicle path (for example FAST-LIO's Log/pos_log.txt), this stage removes
     sparse obstacle cells inside a buffered corridor around the driven path
     while keeping dense walls/fixtures intact.
  3. Floating-column filter  — aggressively removes xy-columns that have
     no ground anchor AND too few points to be a wall. Primary tool for
     deleting upper-body person ghosts (shoulder + head blobs floating
     in mid-air). Enable with --floating. Disabled by default because
     it will also drop thin hanging objects (signs, ceiling fixtures).
  4. DBSCAN clustering + person-sized cluster removal  — optional.
     Enable with --dbscan. Deletes clusters that look like a standing
     human (height z-band + bbox + point-count bounds).
  5. Voxel downsample  — optional. Enable with --voxel > 0.

Presets:
  --light       = gentle SOR only (nb=12, std=2.8). Best default now that
                  the mapping pipeline already applies basic filtering.
  --aggressive  = SOR(nb=30, std=1.5) + --floating + --dbscan (loose
                  person thresholds). One flag for "clean this map hard".

Usage:
  ros2 run traymover_robot_nav pcd_clean.py \\
      --pcd input.pcd --out output.pcd --aggressive

Recommended workflow: start with --light and inspect in RViz (PointCloud2
display). If upper-body person ghosts remain, add --floating first (fast,
no DBSCAN cost). If still unsatisfied, add --dbscan or use --aggressive to
enable everything at once.
"""
import argparse
import sys

import numpy as np
import open3d as o3d


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--pcd', required=True, help='input .pcd file')
    p.add_argument('--out', required=True, help='output .pcd file')
    # SOR
    p.add_argument('--sor-nb', type=int, default=20,
                   help='SOR neighbour count (0 disables SOR)')
    p.add_argument('--sor-std', type=float, default=2.0,
                   help='SOR std_ratio (lower = more aggressive)')
    # Trajectory-corridor sparse-cell filter
    p.add_argument('--trajectory-corridor', action='store_true',
                   help='drop sparse occupied cells inside a buffered vehicle '
                        'trajectory corridor; useful for cleaning drive-path '
                        'speckle without touching the rest of the map')
    p.add_argument('--trajectory-log', type=str, default='',
                   help='trajectory text file; FAST-LIO Log/pos_log.txt is '
                        'supported, as are simple whitespace-separated x y lines')
    p.add_argument('--trajectory-format', choices=['auto', 'fastlio-pos-log', 'xy'],
                   default='auto',
                   help='trajectory file format. auto: >=7 columns -> FAST-LIO '
                        'pos_log (x,y at cols 4,5), otherwise first two columns')
    p.add_argument('--corridor-width', type=float, default=0.80,
                   help='half-width of the kept vehicle corridor [m]')
    p.add_argument('--corridor-xy-res', type=float, default=0.20,
                   help='xy cell size for trajectory corridor cleanup [m]')
    p.add_argument('--corridor-sample-step', type=float, default=0.10,
                   help='trajectory interpolation step before corridor rasterization [m]')
    p.add_argument('--corridor-min-pts', type=int, default=120,
                   help='cells inside the corridor with fewer than this many '
                        'points are treated as speckle and dropped')
    # Floating-column filter
    p.add_argument('--floating', action='store_true',
                   help='enable floating-column filter (remove xy-columns '
                        'with no ground anchor and sparse point count)')
    p.add_argument('--floating-xy-res', type=float, default=0.25,
                   help='xy cell size for the floating-column filter [m]')
    p.add_argument('--floating-floor-z', type=float, default=0.3,
                   help='if a cell\'s min z <= this, it is "ground-anchored" '
                        'and kept regardless of point count [m]')
    p.add_argument('--floating-wall-min-pts', type=int, default=300,
                   help='if a cell has >= this many points, it is dense '
                        'enough to be a wall/fixture and kept regardless '
                        'of ground anchor')
    # DBSCAN
    p.add_argument('--dbscan', action='store_true',
                   help='enable DBSCAN cluster-based person-ghost removal')
    p.add_argument('--dbscan-eps', type=float, default=0.25,
                   help='DBSCAN neighbour radius [m]')
    p.add_argument('--dbscan-min', type=int, default=15,
                   help='DBSCAN min points per cluster')
    p.add_argument('--dbscan-voxel', type=float, default=0.05,
                   help='pre-cluster voxel downsample [m] (memory safety; '
                        '0 = no downsample, eats RAM on large clouds)')
    p.add_argument('--person-z-min', type=float, default=0.3,
                   help='cluster centroid z lower bound for "person"')
    p.add_argument('--person-z-max', type=float, default=1.8,
                   help='cluster centroid z upper bound for "person"')
    p.add_argument('--person-bbox-max', type=float, default=0.8,
                   help='cluster bbox width/depth upper bound [m]')
    p.add_argument('--person-height-max', type=float, default=2.2,
                   help='cluster bbox height upper bound [m]')
    p.add_argument('--person-points-max', type=int, default=800,
                   help='cluster point-count upper bound (avoid killing walls)')
    p.add_argument('--static-points-min', type=int, default=5000,
                   help='min points to qualify as a "large static cluster"')
    p.add_argument('--isolation-min', type=float, default=0.5,
                   help='min distance to nearest large static cluster [m]')
    # Voxel downsample
    p.add_argument('--voxel', type=float, default=0.0,
                   help='voxel downsample size [m]; 0 disables')
    # Presets
    p.add_argument('--light', action='store_true',
                   help='preset: gentle SOR only (nb=12, std=2.8); '
                        'recommended default for current mapping flow')
    p.add_argument('--aggressive', action='store_true',
                   help='preset: SOR(nb=30,std=1.5) + --floating + --dbscan '
                        'with looser person thresholds. Overrides the '
                        'corresponding defaults if those are still at '
                        'their default values.')
    args = p.parse_args()

    apply_light_preset(args, p)
    apply_aggressive_preset(args, p)

    cloud = o3d.io.read_point_cloud(args.pcd)
    pts_before = len(cloud.points)
    if pts_before == 0:
        raise SystemExit(f'no points in {args.pcd}')
    print(f'loaded {pts_before} points from {args.pcd}')

    # --- stage 1: SOR -------------------------------------------------------
    if args.sor_nb > 0:
        before = len(cloud.points)
        cloud, _ = cloud.remove_statistical_outlier(
            nb_neighbors=args.sor_nb, std_ratio=args.sor_std)
        print(f'after SOR (nb={args.sor_nb}, std={args.sor_std}): '
              f'{len(cloud.points)} points '
              f'(dropped {before - len(cloud.points)})')

    # --- stage 2: trajectory corridor sparse-cell filter --------------------
    if args.trajectory_corridor:
        cloud = trajectory_corridor_filter(cloud, args)

    # --- stage 3: floating-column filter ------------------------------------
    if args.floating:
        cloud = floating_column_filter(cloud, args)

    # --- stage 4: DBSCAN person filter --------------------------------------
    if args.dbscan:
        cloud = dbscan_person_filter(cloud, args)

    # --- stage 5: voxel downsample ------------------------------------------
    if args.voxel > 0:
        before = len(cloud.points)
        cloud = cloud.voxel_down_sample(voxel_size=args.voxel)
        print(f'after voxel downsample ({args.voxel} m): {len(cloud.points)} '
              f'(dropped {before - len(cloud.points)})')

    pts_after = len(cloud.points)
    if pts_after == 0:
        raise SystemExit('all points filtered out — your thresholds are too '
                         'aggressive.')
    write_pcd_xyzi_binary(args.out, np.asarray(cloud.points))
    print(f'wrote {pts_after} points to {args.out} '
          f'({100.0 * pts_after / pts_before:.1f}% of input)')


def write_pcd_xyzi_binary(path, pts):
    # lidar_localization_ros2 loads with pcl::PointXYZI and matches FIELDS by
    # name. Open3D's PointCloud drops intensity on read, so we emit intensity=0
    # here — NDT_OMP doesn't use intensity, and a missing "intensity" field in
    # the header causes PCL to read into uninitialized memory and SIGSEGV.
    n = len(pts)
    arr = np.zeros((n, 4), dtype=np.float32)
    arr[:, 0:3] = np.asarray(pts, dtype=np.float32)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        'DATA binary\n'
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(arr.tobytes())


def apply_light_preset(args, parser):
    """If --light is set, keep cleaning SOR-only and make it gentler."""
    if not args.light:
        return

    defaults = {a.dest: a.default for a in parser._actions}

    def bump(name, value):
        if getattr(args, name) == defaults[name]:
            setattr(args, name, value)

    bump('sor_nb', 12)
    bump('sor_std', 2.8)
    print(f'light preset applied: sor_nb={args.sor_nb}, sor_std={args.sor_std}')


def apply_aggressive_preset(args, parser):
    """If --aggressive is set, bump any default-valued knobs to the harsher
    preset. Explicitly user-set flags are left alone (simple check: compare
    current value to parser's default)."""
    if not args.aggressive:
        return
    defaults = {a.dest: a.default for a in parser._actions}

    def bump(name, value):
        if getattr(args, name) == defaults[name]:
            setattr(args, name, value)

    bump('sor_nb', 30)
    bump('sor_std', 1.5)
    args.floating = True
    args.dbscan = True
    # Loosen person-cluster criteria so DBSCAN catches more person-shaped
    # blobs in airport / open-space maps where walls are far away.
    bump('person_z_max', 2.2)
    bump('person_bbox_max', 1.0)
    bump('person_points_max', 2000)
    bump('isolation_min', 0.3)
    print('aggressive preset applied: '
          f'sor_nb={args.sor_nb}, sor_std={args.sor_std}, '
          f'floating=on, dbscan=on, '
          f'person_z_max={args.person_z_max}, '
          f'person_bbox_max={args.person_bbox_max}, '
          f'person_points_max={args.person_points_max}, '
          f'isolation_min={args.isolation_min}')


def load_trajectory_xy(path, fmt='auto'):
    data = np.loadtxt(path, dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 2:
        raise ValueError(f'trajectory file needs at least 2 columns: {path}')

    if fmt == 'fastlio-pos-log' or (fmt == 'auto' and data.shape[1] >= 7):
        xy = data[:, 4:6]
    else:
        xy = data[:, 0:2]

    keep = np.isfinite(xy).all(axis=1)
    xy = xy[keep]
    if len(xy) == 0:
        raise ValueError(f'no finite trajectory samples in {path}')
    return xy.astype(np.float64, copy=False)


def densify_polyline(xy, step):
    xy = np.asarray(xy, dtype=np.float64)
    if len(xy) <= 1 or step <= 0.0:
        return xy

    samples = [xy[0]]
    for i in range(len(xy) - 1):
        start = xy[i]
        end = xy[i + 1]
        delta = end - start
        length = float(np.linalg.norm(delta))
        if length <= 1e-9:
            continue
        count = max(int(np.ceil(length / step)), 1)
        for j in range(1, count + 1):
            samples.append(start + delta * (j / count))
    return np.asarray(samples, dtype=np.float64)


def build_corridor_cells(samples_xy, cell_res, corridor_width):
    if len(samples_xy) == 0:
        return set()
    radius_cells = max(int(np.ceil(corridor_width / cell_res)), 0)
    offsets = []
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if np.hypot(dx * cell_res, dy * cell_res) <= corridor_width + 1e-9:
                offsets.append((dx, dy))

    base_cells = np.floor(samples_xy / cell_res).astype(np.int64)
    corridor_cells = set()
    for cx, cy in base_cells:
        for dx, dy in offsets:
            corridor_cells.add((int(cx + dx), int(cy + dy)))
    return corridor_cells


def trajectory_corridor_filter(cloud, args):
    pts = np.asarray(cloud.points)
    n = len(pts)
    if n == 0:
        return cloud
    if not args.trajectory_log:
        raise SystemExit('--trajectory-corridor requires --trajectory-log')

    traj_xy = load_trajectory_xy(args.trajectory_log, args.trajectory_format)
    dense_xy = densify_polyline(traj_xy, args.corridor_sample_step)
    corridor_cells = build_corridor_cells(
        dense_xy, args.corridor_xy_res, args.corridor_width)
    if not corridor_cells:
        print('  trajectory corridor filter: no corridor cells generated; skipping')
        return cloud

    res = args.corridor_xy_res
    cx = np.floor(pts[:, 0] / res).astype(np.int64)
    cy = np.floor(pts[:, 1] / res).astype(np.int64)

    order = np.lexsort((cy, cx))
    cx_s = cx[order]
    cy_s = cy[order]
    if n == 1:
        cell_starts = np.array([0, 1], dtype=np.int64)
    else:
        change = (cx_s[1:] != cx_s[:-1]) | (cy_s[1:] != cy_s[:-1])
        cell_starts = np.concatenate(([0], np.where(change)[0] + 1, [n])).astype(np.int64)

    keep_sorted = np.ones(n, dtype=bool)
    drop_cells = 0
    drop_pts = 0
    corridor_total = 0
    for i in range(len(cell_starts) - 1):
        s, e = int(cell_starts[i]), int(cell_starts[i + 1])
        key = (int(cx_s[s]), int(cy_s[s]))
        if key not in corridor_cells:
            continue
        corridor_total += 1
        count = e - s
        if count >= args.corridor_min_pts:
            continue
        keep_sorted[s:e] = False
        drop_cells += 1
        drop_pts += count

    keep = np.empty(n, dtype=bool)
    keep[order] = keep_sorted
    print(f'  trajectory corridor filter ({args.trajectory_log}, width={args.corridor_width}, '
          f'xy_res={args.corridor_xy_res}, min_pts={args.corridor_min_pts}): '
          f'dropped {drop_pts} points across {drop_cells}/{corridor_total} sparse corridor cells')
    return subset_cloud(cloud, keep)


def floating_column_filter(cloud, args):
    """Drop points in xy-columns that are both (a) unanchored to the ground
    AND (b) too sparse to be a wall/fixture.

    Rationale: in a FAST-LIO map of an airport hall, the main "legitimate"
    vertical structures are walls, columns, signage boards, kiosks — all
    of which either reach the floor or are dense (columns span floor-to-
    ceiling, signage has many points). A person's shoulder+head residue
    (remaining after SOR removed their legs/body because those points were
    transient) shows up as a 100-300 point blob at z ≈ 1.1-1.8 m with no
    points anywhere below. A single xy-column test captures that exactly:
      - point count in cell < wall_min_pts  (too sparse for a wall)
      - AND z_min in cell > floor_z         (no ground anchor)
      -> drop entire cell.

    Runs in O(N log N) via lexsort; no per-point Python loop.
    """
    pts = np.asarray(cloud.points)
    n = len(pts)
    if n == 0:
        return cloud

    res = args.floating_xy_res
    cx = np.floor(pts[:, 0] / res).astype(np.int64)
    cy = np.floor(pts[:, 1] / res).astype(np.int64)

    # Sort points by (cx, cy) so each xy cell is a contiguous run.
    order = np.lexsort((cy, cx))
    cx_s = cx[order]
    cy_s = cy[order]
    z_s = pts[order, 2]

    # Cell boundary indices.
    if n == 1:
        cell_starts = np.array([0, 1], dtype=np.int64)
    else:
        change = (cx_s[1:] != cx_s[:-1]) | (cy_s[1:] != cy_s[:-1])
        cell_starts = np.concatenate(
            ([0], np.where(change)[0] + 1, [n])).astype(np.int64)

    keep_sorted = np.ones(n, dtype=bool)
    n_drop_cells = 0
    n_drop_pts = 0
    total_cells = len(cell_starts) - 1
    for i in range(total_cells):
        s, e = int(cell_starts[i]), int(cell_starts[i + 1])
        count = e - s
        if count >= args.floating_wall_min_pts:
            continue  # dense enough to be a wall/fixture
        if z_s[s:e].min() <= args.floating_floor_z:
            continue  # has ground contact
        keep_sorted[s:e] = False
        n_drop_cells += 1
        n_drop_pts += count

    # Un-sort the keep mask back to original index order.
    keep = np.empty(n, dtype=bool)
    keep[order] = keep_sorted

    print(f'  floating-column filter (xy_res={args.floating_xy_res}, '
          f'floor_z={args.floating_floor_z}, '
          f'wall_min_pts={args.floating_wall_min_pts}): '
          f'dropped {n_drop_pts} points across {n_drop_cells}/{total_cells} '
          f'unanchored sparse cells')

    return subset_cloud(cloud, keep)


def subset_cloud(cloud, keep_mask):
    """Return a new Open3D cloud keeping only points where keep_mask is True.
    Preserves colors and normals if present."""
    pts = np.asarray(cloud.points)
    new = o3d.geometry.PointCloud()
    new.points = o3d.utility.Vector3dVector(pts[keep_mask])
    if cloud.has_colors():
        new.colors = o3d.utility.Vector3dVector(
            np.asarray(cloud.colors)[keep_mask])
    if cloud.has_normals():
        new.normals = o3d.utility.Vector3dVector(
            np.asarray(cloud.normals)[keep_mask])
    return new


def dbscan_person_filter(cloud, args):
    """Run DBSCAN and drop clusters that look like a transient human.

    Policy: a cluster is considered "person-like ghost" and dropped iff
    ALL of these hold:
      - centroid z within [person_z_min, person_z_max]
      - bounding box width < person_bbox_max AND depth < person_bbox_max
      - bounding box height < person_height_max
      - point count < person_points_max
      - min distance to any large static cluster > isolation_min

    Large clouds (e.g. 2 M+ points) would OOM on full-resolution DBSCAN.
    We cluster on a voxel-downsampled copy (--dbscan-voxel) and then apply
    the drop decision back to the original points by nearest-neighbour
    lookup. This keeps the filter accurate (person clusters remain visible
    at 5 cm resolution) while DBSCAN only sees O(100 k) points.

    Any cluster failing any check is kept. Noise (DBSCAN label -1) is also
    kept verbatim so we don't accidentally delete sparse legitimate
    structure — SOR already handled that stage.
    """
    original_pts = np.asarray(cloud.points)
    n_orig = len(original_pts)

    # --- 1. Downsample for DBSCAN (memory safety) --------------------------
    if args.dbscan_voxel > 0:
        sub_cloud = cloud.voxel_down_sample(voxel_size=args.dbscan_voxel)
        print(f'  pre-cluster voxel downsample ({args.dbscan_voxel} m): '
              f'{n_orig} -> {len(sub_cloud.points)} points')
    else:
        sub_cloud = cloud
        if n_orig > 1_000_000:
            print(f'  WARNING: DBSCAN on {n_orig} points without voxel '
                  f'downsample may OOM. Consider --dbscan-voxel 0.05.')

    sub_pts = np.asarray(sub_cloud.points)

    # --- 2. DBSCAN on the (sub)sample --------------------------------------
    labels = np.asarray(
        sub_cloud.cluster_dbscan(eps=args.dbscan_eps,
                                 min_points=args.dbscan_min,
                                 print_progress=False))
    n_clusters = int(labels.max()) + 1 if labels.size else 0
    print(f'DBSCAN: {n_clusters} clusters, '
          f'{int((labels == -1).sum())} noise points '
          f'(on {len(sub_pts)}-point downsample)')

    if n_clusters == 0:
        return cloud

    # --- 3. Per-cluster stats ----------------------------------------------
    cluster_stats = []  # (cluster_id, centroid, bbox_min, bbox_max, sub_size)
    for cid in range(n_clusters):
        mask = labels == cid
        pts = sub_pts[mask]
        if pts.size == 0:
            continue
        centroid = pts.mean(axis=0)
        bbox_min = pts.min(axis=0)
        bbox_max = pts.max(axis=0)
        cluster_stats.append((cid, centroid, bbox_min, bbox_max, int(pts.shape[0])))

    # Scale size-based thresholds by the downsample ratio so tuned defaults
    # (meant for original density) still work on the sub-cloud.
    if args.dbscan_voxel > 0 and n_orig > 0:
        ratio = len(sub_pts) / n_orig
    else:
        ratio = 1.0
    static_min = max(1, int(args.static_points_min * ratio))
    person_max = max(1, int(args.person_points_max * ratio))

    static_centroids = np.array(
        [c for _, c, _, _, size in cluster_stats if size >= static_min],
        dtype=np.float64)
    print(f'  large static clusters (>= {static_min} sub-pts): '
          f'{len(static_centroids)}')

    drop_ids = set()
    for cid, centroid, bbox_min, bbox_max, size in cluster_stats:
        if size >= static_min:
            continue
        if not (args.person_z_min < centroid[2] < args.person_z_max):
            continue
        w = bbox_max[0] - bbox_min[0]
        d = bbox_max[1] - bbox_min[1]
        h = bbox_max[2] - bbox_min[2]
        if w >= args.person_bbox_max or d >= args.person_bbox_max:
            continue
        if h >= args.person_height_max:
            continue
        if size >= person_max:
            continue
        if len(static_centroids) > 0:
            dists = np.linalg.norm(static_centroids - centroid, axis=1)
            if dists.min() <= args.isolation_min:
                continue
        drop_ids.add(cid)

    if not drop_ids:
        print('  no clusters matched person-ghost criteria')
        return cloud

    dropped_sub_points = sum(size for cid, _, _, _, size in cluster_stats
                             if cid in drop_ids)
    print(f'  dropping {len(drop_ids)} person-ghost clusters '
          f'(~{dropped_sub_points} sub-points)')

    # --- 4. Map "dropped" decision from sub-cloud back to original points --
    if args.dbscan_voxel <= 0:
        # Clustered directly on the full cloud; labels align with originals.
        keep = ~np.isin(labels, list(drop_ids))
    else:
        # Nearest-neighbour lookup: for each original point, find its
        # closest sub-cloud point and inherit its cluster label.
        kdtree = o3d.geometry.KDTreeFlann(sub_cloud)
        drop_labels_set = drop_ids  # for O(1) membership test
        keep = np.ones(n_orig, dtype=bool)
        # Batch by chunks to keep the Python loop latency tolerable.
        for i in range(n_orig):
            _, idx, _ = kdtree.search_knn_vector_3d(original_pts[i], 1)
            if labels[idx[0]] in drop_labels_set:
                keep[i] = False
        print(f'  mapped drop mask back to {n_orig} original points: '
              f'keep {int(keep.sum())}, drop {int((~keep).sum())}')

    return subset_cloud(cloud, keep)


if __name__ == '__main__':
    main()
