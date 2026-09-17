#!/usr/bin/env python3
"""Generate no-region-filter PGM maps at several resolutions for comparison.

The Z limits are absolute values in the PCD map frame. Unlike pcd2pgm.py's
navigation defaults, this diagnostic tool keeps singleton occupied cells.
"""
import argparse
import math
from pathlib import Path

import numpy as np
import open3d as o3d
import yaml
from PIL import Image, ImageDraw, ImageFont

from pcd2pgm import FREE_VAL, OCC_VAL, dilate3x3, occupancy_from_points


def resolution_tag(resolution):
    return f'{resolution:g}'.replace('.', 'p')


def load_font(size):
    try:
        return ImageFont.truetype(
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)
    except OSError:
        return ImageFont.load_default()


def write_map(image, output_prefix, resolution, x_min, y_min):
    pgm_path = output_prefix.with_suffix('.pgm')
    png_path = output_prefix.with_suffix('.png')
    yaml_path = output_prefix.with_suffix('.yaml')

    image.save(pgm_path)
    image.save(png_path)
    metadata = {
        'image': pgm_path.name,
        'resolution': float(resolution),
        'origin': [float(x_min), float(y_min), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    with yaml_path.open('w') as stream:
        yaml.safe_dump(metadata, stream, default_flow_style=None, sort_keys=False)
    return png_path


def write_comparison(entries, output_path):
    panel_width = 900
    panel_height = 900
    label_height = 80
    canvas = Image.new('L', (panel_width * len(entries), panel_height), FREE_VAL)
    draw = ImageDraw.Draw(canvas)
    font = load_font(26)
    resampling = getattr(Image, 'Resampling', Image).LANCZOS

    for index, entry in enumerate(entries):
        source = Image.open(entry['png']).convert('L')
        source.thumbnail(
            (panel_width - 30, panel_height - label_height - 20),
            resampling,
        )
        panel_x = index * panel_width
        image_x = panel_x + (panel_width - source.width) // 2
        image_y = label_height + (panel_height - label_height - source.height) // 2
        canvas.paste(source, (image_x, image_y))
        label = (
            f"resolution {entry['resolution']:.2f} m | "
            f"occupied {entry['occupied_after_dilate']}"
        )
        draw.text((panel_x + 16, 20), label, fill=OCC_VAL, font=font)

    canvas.save(output_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', required=True, help='input PCD file')
    parser.add_argument('--out-dir', required=True, help='output directory')
    parser.add_argument('--z-min', type=float, default=0.12,
                        help='absolute lower Z limit in the PCD map frame [m]')
    parser.add_argument('--z-max', type=float, default=None,
                        help='absolute upper Z limit [m]; default: PCD maximum')
    parser.add_argument('--resolutions', type=float, nargs='+',
                        default=[0.05, 0.10, 0.20],
                        help='map resolutions to compare [m/pixel]')
    parser.add_argument('--min-points', type=int, default=1,
                        help='minimum point count per occupied cell')
    parser.add_argument('--dilate', type=int, default=2,
                        help='number of occupancy dilation passes')
    parser.add_argument('--padding', type=float, default=1.0,
                        help='free-space border around the selected points [m]')
    args = parser.parse_args()

    if not args.resolutions or any(value <= 0 for value in args.resolutions):
        parser.error('all resolutions must be positive')
    if args.min_points < 1:
        parser.error('--min-points must be at least 1')
    if args.dilate < 0:
        parser.error('--dilate cannot be negative')

    cloud = o3d.io.read_point_cloud(args.pcd)
    points = np.asarray(cloud.points)
    if points.size == 0:
        raise SystemExit(f'no points in {args.pcd}')

    z_max = float(points[:, 2].max()) if args.z_max is None else args.z_max
    if not math.isfinite(args.z_min) or not math.isfinite(z_max):
        parser.error('Z limits must be finite')
    if args.z_min >= z_max:
        parser.error('--z-min must be lower than --z-max')

    mask = (points[:, 2] >= args.z_min) & (points[:, 2] <= z_max)
    slab_xy = points[mask][:, :2]
    if slab_xy.size == 0:
        raise SystemExit(
            f'no points survived absolute Z filter [{args.z_min}, {z_max}]')

    output_dir = Path(args.out_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    pcd_stem = Path(args.pcd).stem
    print(f'loaded {len(points)} points from {args.pcd}')
    print(f'kept {len(slab_xy)} points in absolute Z [{args.z_min:.2f}, {z_max:.2f}] m')
    print('small-component removal: disabled')

    comparison_entries = []
    for resolution in args.resolutions:
        occupancy, x_min, y_min, width, height = occupancy_from_points(
            slab_xy=slab_xy,
            resolution=resolution,
            padding=args.padding,
            min_points=args.min_points,
        )
        occupied_before_dilate = int(occupancy.sum())
        for _ in range(args.dilate):
            occupancy = dilate3x3(occupancy)
        occupied_after_dilate = int(occupancy.sum())

        pixels = np.full((height, width), FREE_VAL, dtype=np.uint8)
        pixels[occupancy] = OCC_VAL
        image = Image.fromarray(np.flipud(pixels), mode='L')
        prefix = output_dir / (
            f'{pcd_stem}_no_region_filter_res_{resolution_tag(resolution)}')
        png_path = write_map(image, prefix, resolution, x_min, y_min)
        comparison_entries.append({
            'png': png_path,
            'resolution': resolution,
            'occupied_after_dilate': occupied_after_dilate,
        })
        print(
            f'resolution={resolution:.2f} m grid={width}x{height} '
            f'occupied={occupied_before_dilate} '
            f'occupied_after_dilate={occupied_after_dilate}'
        )

    comparison_path = output_dir / f'{pcd_stem}_no_region_filter_comparison.png'
    write_comparison(comparison_entries, comparison_path)
    print(f'wrote comparison image {comparison_path}')


if __name__ == '__main__':
    main()
