#!/usr/bin/env python3
"""Offline experiments for interference-sensitive localization.

The runner deliberately avoids ROS runtime dependencies. It reads existing PCD
files, computes map/interference statistics, and runs deterministic synthetic
gate sweeps that mirror the NDT accept/reject order in lidar_localization_ros2.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import struct
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT_DIR = WORKSPACE_ROOT / "tmp" / "offline_localization_experiments" / "data"
DEFAULT_PCD_DIRS = (
    WORKSPACE_ROOT / "src" / "traymover_robot_slam" / "FAST_LIO" / "PCD",
    WORKSPACE_ROOT / "src" / "traymover_robot_slam" / "ndt_omp_ros2" / "data",
)

SCAN_MIN_RANGE_M = 0.5
SCAN_MAX_RANGE_M = 120.0
FAR_POINT_BOOST_MIN_RANGE_M = 15.0
FAR_POINT_BOOST_FACTOR = 2
VOXEL_LEAF_SIZE_M = 0.15
REAR_BOX = {
    "x_min": -1.25,
    "x_max": -0.35,
    "abs_y_max": 0.90,
    "z_min": -0.50,
    "z_max": 2.00,
}


@dataclass(frozen=True)
class GateProfile:
    name: str
    score_threshold: float
    max_pose_jump_translation: float
    max_pose_jump_rotation: float
    max_map_odom_update_translation: float
    max_map_odom_update_rotation: float
    map_odom_smoothing: float = 0.30


@dataclass(frozen=True)
class Attempt:
    scenario: str
    index: int
    converged: bool
    fitness: float
    pose_jump_translation: float
    pose_jump_yaw: float
    map_odom_jump_translation: float
    map_odom_jump_yaw: float


BASELINE_PROFILE = GateProfile(
    name="baseline_current",
    score_threshold=1.0,
    max_pose_jump_translation=0.35,
    max_pose_jump_rotation=0.45,
    max_map_odom_update_translation=0.50,
    max_map_odom_update_rotation=0.25,
)

GATE_PROFILES = (
    BASELINE_PROFILE,
    replace(BASELINE_PROFILE, name="strict_score_0_5", score_threshold=0.5),
    replace(BASELINE_PROFILE, name="old_yaw_gate_0_25", max_pose_jump_rotation=0.25),
    replace(
        BASELINE_PROFILE,
        name="strict_3d_map_odom_gate",
        max_map_odom_update_translation=0.30,
        max_map_odom_update_rotation=0.12,
    ),
    replace(
        BASELINE_PROFILE,
        name="map_odom_gate_disabled",
        max_map_odom_update_translation=-1.0,
        max_map_odom_update_rotation=-1.0,
    ),
)


def input_path_label(path: Path) -> str:
    try:
        return str(path.relative_to(WORKSPACE_ROOT))
    except ValueError:
        return str(path)


def discover_default_pcds() -> List[Path]:
    pcds: List[Path] = []
    for directory in DEFAULT_PCD_DIRS:
        if directory.exists():
            pcds.extend(sorted(directory.glob("*.pcd")))
    return pcds


def parse_pcd_header(path: Path) -> Tuple[Dict[str, str], int]:
    header: Dict[str, str] = {}
    with path.open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: missing DATA line")
            text = line.decode("ascii", errors="replace").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split(None, 1)
            key = parts[0].upper()
            value = parts[1] if len(parts) > 1 else ""
            header[key] = value
            if key == "DATA":
                return header, f.tell()


def pcd_binary_layout(header: Dict[str, str]) -> Tuple[int, Dict[str, Tuple[int, int, str]]]:
    fields = header.get("FIELDS", "").split()
    sizes = [int(x) for x in header.get("SIZE", "").split()]
    types = header.get("TYPE", "").split()
    counts = [int(x) for x in header.get("COUNT", " ".join(["1"] * len(fields))).split()]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT lengths do not match")

    offset = 0
    layout: Dict[str, Tuple[int, int, str]] = {}
    for field, size, typ, count in zip(fields, sizes, types, counts):
        layout[field] = (offset, size, typ)
        offset += size * count
    return offset, layout


def read_pcd_points(path: Path, max_points: Optional[int] = None) -> Tuple[Dict[str, str], List[Tuple[float, float, float]]]:
    header, data_offset = parse_pcd_header(path)
    if header.get("DATA", "").lower() != "binary":
        raise ValueError(f"{path}: only binary PCD is supported by this runner")

    point_step, layout = pcd_binary_layout(header)
    for field in ("x", "y", "z"):
        if field not in layout:
            raise ValueError(f"{path}: missing {field} field")
        _, size, typ = layout[field]
        if size != 4 or typ.upper() != "F":
            raise ValueError(f"{path}: field {field} must be float32")

    point_count = int(header.get("POINTS", "0"))
    with path.open("rb") as f:
        f.seek(data_offset)
        data = f.read(point_step * point_count)

    available_points = min(point_count, len(data) // point_step)
    if max_points and max_points > 0 and available_points > max_points:
        stride = max(1, available_points // max_points)
        indices = range(0, available_points, stride)
    else:
        indices = range(available_points)

    x_offset = layout["x"][0]
    y_offset = layout["y"][0]
    z_offset = layout["z"][0]
    points: List[Tuple[float, float, float]] = []
    for i in indices:
        base = i * point_step
        x = struct.unpack_from("<f", data, base + x_offset)[0]
        y = struct.unpack_from("<f", data, base + y_offset)[0]
        z = struct.unpack_from("<f", data, base + z_offset)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
        if max_points and max_points > 0 and len(points) >= max_points:
            break
    return header, points


def percentile(sorted_values: Sequence[float], pct: float) -> float:
    if not sorted_values:
        return float("nan")
    idx = (len(sorted_values) - 1) * pct
    low = int(math.floor(idx))
    high = int(math.ceil(idx))
    if low == high:
        return sorted_values[low]
    return sorted_values[low] * (high - idx) + sorted_values[high] * (idx - low)


def in_rear_box(x: float, y: float, z: float) -> bool:
    return (
        REAR_BOX["x_min"] <= x <= REAR_BOX["x_max"]
        and abs(y) <= REAR_BOX["abs_y_max"]
        and REAR_BOX["z_min"] <= z <= REAR_BOX["z_max"]
    )


def pcd_stats(path: Path, max_points: Optional[int]) -> Dict[str, object]:
    header, points = read_pcd_points(path, max_points=max_points)
    original_points = int(header.get("POINTS", len(points)))
    sample_points = len(points)
    ranges: List[float] = []
    voxel_keys = set()
    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")
    range_clip_count = 0
    rear_box_count = 0
    synthetic_half_fov_count = 0
    far_return_count = 0

    for x, y, z in points:
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        min_z = min(min_z, z)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
        max_z = max(max_z, z)
        rng = math.sqrt(x * x + y * y + z * z)
        ranges.append(rng)
        if SCAN_MIN_RANGE_M <= rng <= SCAN_MAX_RANGE_M:
            range_clip_count += 1
        if in_rear_box(x, y, z):
            rear_box_count += 1
        if x >= 0.0:
            synthetic_half_fov_count += 1
        if rng >= FAR_POINT_BOOST_MIN_RANGE_M:
            far_return_count += 1
        voxel_keys.add(
            (
                math.floor(x / VOXEL_LEAF_SIZE_M),
                math.floor(y / VOXEL_LEAF_SIZE_M),
                math.floor(z / VOXEL_LEAF_SIZE_M),
            )
        )

    ranges.sort()
    scale = original_points / sample_points if sample_points else 0.0
    ghost_injection_points = int(round(min(sample_points * 0.10, 20000) * scale))
    far_boost_extra_points = int(round(far_return_count * max(0, FAR_POINT_BOOST_FACTOR - 1) * scale))

    return {
        "pcd": input_path_label(path),
        "original_points": original_points,
        "sample_points": sample_points,
        "sample_scale": round(scale, 6),
        "finite_points": int(round(sample_points * scale)),
        "bbox_x_min": round(min_x, 4),
        "bbox_x_max": round(max_x, 4),
        "bbox_y_min": round(min_y, 4),
        "bbox_y_max": round(max_y, 4),
        "bbox_z_min": round(min_z, 4),
        "bbox_z_max": round(max_z, 4),
        "range_median": round(percentile(ranges, 0.50), 4),
        "range_p90": round(percentile(ranges, 0.90), 4),
        "range_p95": round(percentile(ranges, 0.95), 4),
        "range_max": round(ranges[-1], 4) if ranges else float("nan"),
        "range_clip_kept_points": int(round(range_clip_count * scale)),
        "range_clip_removed_points": int(round((sample_points - range_clip_count) * scale)),
        "rear_box_removed_points": int(round(rear_box_count * scale)),
        "rear_box_removed_ratio": round(rear_box_count / sample_points, 6) if sample_points else 0.0,
        "synthetic_half_fov_kept_points": int(round(synthetic_half_fov_count * scale)),
        "synthetic_half_fov_removed_ratio": round(
            (sample_points - synthetic_half_fov_count) / sample_points, 6
        ) if sample_points else 0.0,
        "far_return_points": int(round(far_return_count * scale)),
        "far_return_ratio": round(far_return_count / sample_points, 6) if sample_points else 0.0,
        "far_boost_extra_points": far_boost_extra_points,
        "voxel_leaf_size_m": VOXEL_LEAF_SIZE_M,
        "voxel_unique_count": int(round(len(voxel_keys) * scale)),
        "synthetic_ghost_injection_points": ghost_injection_points,
    }


def generate_attempts(scenario: str, count: int = 80) -> List[Attempt]:
    rng = random.Random(f"traymover-{scenario}-2026-06-11")
    attempts: List[Attempt] = []
    for i in range(count):
        converged = True
        if scenario == "A_clean_map_default":
            fitness = rng.gauss(0.72, 0.07)
            pose_t = abs(rng.gauss(0.08, 0.03))
            pose_yaw = abs(rng.gauss(0.04, 0.02))
            map_t = abs(rng.gauss(0.045, 0.02))
            map_yaw = abs(rng.gauss(0.035, 0.015))
        elif scenario == "B_ghost_map":
            fitness = rng.gauss(0.92, 0.20)
            if i % 9 == 0:
                fitness += rng.uniform(0.35, 0.85)
            pose_t = abs(rng.gauss(0.16, 0.09))
            pose_yaw = abs(rng.gauss(0.10, 0.06))
            map_t = abs(rng.gauss(0.10, 0.08))
            map_yaw = abs(rng.gauss(0.08, 0.06))
            converged = i % 23 != 0
        elif scenario == "C_no_odom_prior":
            drift = min(0.75, 0.025 * i)
            fitness = rng.gauss(0.78 + 0.003 * i, 0.10)
            pose_t = abs(rng.gauss(0.10 + drift, 0.08))
            pose_yaw = abs(rng.gauss(0.06 + 0.004 * i, 0.04))
            map_t = abs(rng.gauss(0.08 + 0.30 * drift, 0.05))
            map_yaw = abs(rng.gauss(0.05 + 0.12 * drift, 0.03))
        elif scenario == "D_strict_score_sensitive":
            fitness = rng.gauss(0.76, 0.09)
            pose_t = abs(rng.gauss(0.07, 0.03))
            pose_yaw = abs(rng.gauss(0.04, 0.02))
            map_t = abs(rng.gauss(0.04, 0.02))
            map_yaw = abs(rng.gauss(0.03, 0.015))
        elif scenario == "E_u_turn":
            phase = abs(i - count // 2) / (count // 2)
            turn_boost = 1.0 - min(1.0, phase)
            fitness = rng.gauss(0.78 + 0.10 * turn_boost, 0.08)
            pose_t = abs(rng.gauss(0.12 + 0.05 * turn_boost, 0.04))
            pose_yaw = abs(rng.gauss(0.07 + 0.42 * turn_boost, 0.05))
            map_t = abs(rng.gauss(0.07 + 0.04 * turn_boost, 0.03))
            map_yaw = abs(rng.gauss(0.05 + 0.12 * turn_boost, 0.025))
        else:
            raise ValueError(f"unknown scenario: {scenario}")

        attempts.append(
            Attempt(
                scenario=scenario,
                index=i,
                converged=converged,
                fitness=max(0.0, fitness),
                pose_jump_translation=pose_t,
                pose_jump_yaw=pose_yaw,
                map_odom_jump_translation=map_t,
                map_odom_jump_yaw=map_yaw,
            )
        )
    return attempts


def evaluate_attempt(attempt: Attempt, profile: GateProfile, have_good_pose: bool) -> str:
    if not attempt.converged:
        return "reject_not_converged"
    if attempt.fitness > profile.score_threshold:
        return "reject_fitness"
    if have_good_pose:
        if (
            attempt.pose_jump_translation > profile.max_pose_jump_translation
            or abs(attempt.pose_jump_yaw) > profile.max_pose_jump_rotation
        ):
            return "reject_pose_jump"
        map_t_gate = profile.max_map_odom_update_translation
        map_yaw_gate = profile.max_map_odom_update_rotation
        if (
            (map_t_gate > 0.0 and attempt.map_odom_jump_translation > map_t_gate)
            or (map_yaw_gate > 0.0 and abs(attempt.map_odom_jump_yaw) > map_yaw_gate)
        ):
            return "reject_map_odom_jump"
    return "accepted"


def run_gate_sweeps() -> Tuple[List[Dict[str, object]], List[Dict[str, object]]]:
    scenarios = (
        "A_clean_map_default",
        "B_ghost_map",
        "C_no_odom_prior",
        "D_strict_score_sensitive",
        "E_u_turn",
    )
    attempts = [attempt for scenario in scenarios for attempt in generate_attempts(scenario)]
    attempt_rows: List[Dict[str, object]] = []
    summary_rows: List[Dict[str, object]] = []

    for profile in GATE_PROFILES:
        for scenario in scenarios:
            have_good_pose = False
            counts: Dict[str, int] = {
                "accepted": 0,
                "reject_not_converged": 0,
                "reject_fitness": 0,
                "reject_pose_jump": 0,
                "reject_map_odom_jump": 0,
            }
            scenario_attempts = [attempt for attempt in attempts if attempt.scenario == scenario]
            for attempt in scenario_attempts:
                decision = evaluate_attempt(attempt, profile, have_good_pose)
                counts[decision] += 1
                if decision == "accepted":
                    have_good_pose = True
                attempt_rows.append(
                    {
                        "profile": profile.name,
                        "scenario": scenario,
                        "attempt_index": attempt.index,
                        "decision": decision,
                        "converged": attempt.converged,
                        "fitness": round(attempt.fitness, 4),
                        "pose_jump_translation_m": round(attempt.pose_jump_translation, 4),
                        "pose_jump_yaw_rad": round(attempt.pose_jump_yaw, 4),
                        "map_odom_jump_translation_m": round(attempt.map_odom_jump_translation, 4),
                        "map_odom_jump_yaw_rad": round(attempt.map_odom_jump_yaw, 4),
                        "score_threshold": profile.score_threshold,
                        "max_pose_jump_translation_m": profile.max_pose_jump_translation,
                        "max_pose_jump_rotation_rad": profile.max_pose_jump_rotation,
                        "max_map_odom_update_translation_m": profile.max_map_odom_update_translation,
                        "max_map_odom_update_rotation_rad": profile.max_map_odom_update_rotation,
                    }
                )
            total = len(scenario_attempts)
            summary_rows.append(
                {
                    "profile": profile.name,
                    "scenario": scenario,
                    "total_attempts": total,
                    **counts,
                    "accepted_ratio": round(counts["accepted"] / total, 4),
                    "rejected_ratio": round((total - counts["accepted"]) / total, 4),
                }
            )
    return attempt_rows, summary_rows


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"no rows for {path}")
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, pcd_rows: Sequence[Dict[str, object]], gate_summary: Sequence[Dict[str, object]]) -> None:
    baseline_rows = [
        row for row in gate_summary if row["profile"] == "baseline_current"
    ]
    strict_rows = [
        row for row in gate_summary if row["profile"] == "strict_score_0_5"
    ]
    old_yaw_rows = [
        row for row in gate_summary if row["profile"] == "old_yaw_gate_0_25"
    ]
    lines = [
        "# Offline Experiment Summary",
        "",
        f"Generated at: {datetime.now(timezone.utc).isoformat()}",
        "",
        "## PCD Inputs",
        "",
        f"- Files analyzed: {len(pcd_rows)}",
        f"- Total original points: {sum(int(row['original_points']) for row in pcd_rows)}",
        f"- Total synthetic rear-box removals: {sum(int(row['rear_box_removed_points']) for row in pcd_rows)}",
        f"- Total far-return boost extra points: {sum(int(row['far_boost_extra_points']) for row in pcd_rows)}",
        "",
        "## Synthetic Gate Sweep",
        "",
        "Baseline profile accepted ratios:",
    ]
    for row in baseline_rows:
        lines.append(
            f"- {row['scenario']}: {row['accepted']}/{row['total_attempts']} "
            f"accepted ({row['accepted_ratio']})"
        )
    lines.extend(["", "Strict score 0.5 accepted ratios:"])
    for row in strict_rows:
        lines.append(
            f"- {row['scenario']}: {row['accepted']}/{row['total_attempts']} "
            f"accepted ({row['accepted_ratio']})"
        )
    lines.extend(["", "Old 0.25 rad yaw gate accepted ratios:"])
    for row in old_yaw_rows:
        lines.append(
            f"- {row['scenario']}: {row['accepted']}/{row['total_attempts']} "
            f"accepted ({row['accepted_ratio']})"
        )
    lines.extend(
        [
            "",
            "## Interpretation Boundary",
            "",
            "These are offline map statistics and deterministic synthetic gate decisions. "
            "They are useful for parameter sensitivity and report drafting, but they are "
            "not real vehicle NDT fitness, drift, or Nav2 abort measurements.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pcd",
        action="append",
        type=Path,
        help="PCD file to analyze. Can be passed more than once. Defaults to known repo PCDs.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for generated CSV/JSON/Markdown data.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Deterministically sample at most this many finite points per PCD. 0 means exact.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    pcd_paths = args.pcd if args.pcd else discover_default_pcds()
    pcd_paths = [path.resolve() for path in pcd_paths]
    if not pcd_paths:
        raise SystemExit("No PCD inputs found")

    max_points = args.max_points if args.max_points and args.max_points > 0 else None
    pcd_rows = [pcd_stats(path, max_points=max_points) for path in pcd_paths]
    gate_attempt_rows, gate_summary_rows = run_gate_sweeps()

    write_csv(output_dir / "pcd_interference_stats.csv", pcd_rows)
    write_csv(output_dir / "gate_sweep_attempts.csv", gate_attempt_rows)
    write_csv(output_dir / "gate_sweep_summary.csv", gate_summary_rows)
    write_summary(output_dir / "experiment_summary.md", pcd_rows, gate_summary_rows)

    metadata = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "workspace_root": str(WORKSPACE_ROOT),
        "pcd_inputs": [input_path_label(path) for path in pcd_paths],
        "max_points_per_pcd": max_points,
        "constants": {
            "scan_min_range_m": SCAN_MIN_RANGE_M,
            "scan_max_range_m": SCAN_MAX_RANGE_M,
            "far_point_boost_min_range_m": FAR_POINT_BOOST_MIN_RANGE_M,
            "far_point_boost_factor": FAR_POINT_BOOST_FACTOR,
            "voxel_leaf_size_m": VOXEL_LEAF_SIZE_M,
            "rear_box": REAR_BOX,
        },
        "profiles": [profile.__dict__ for profile in GATE_PROFILES],
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(f"Wrote offline experiment data to {output_dir}")
    print(f"Analyzed {len(pcd_rows)} PCD files")
    print(f"Wrote {len(gate_summary_rows)} gate summary rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
