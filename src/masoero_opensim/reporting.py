from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

from . import config
from .opensim_utils import (
    apply_pose_values,
    axis_index,
    body_transform,
    coordinate_defaults,
    load_model,
    marker_positions,
    read_storage_file,
)
from .runtime import UserFacingError, ensure_parent_dir
from .specs import read_yaml


def _guess_constraints_path(pose_path: Path) -> Path:
    name = pose_path.stem.lower()
    if "good" in name:
        return config.GOOD_CONSTRAINTS_PATH
    if "bad" in name:
        return config.BAD_CONSTRAINTS_PATH
    raise UserFacingError("Pass `--constraints` when the pose filename does not include `good` or `bad`.")


def _first_coordinate(pose_values: dict[str, float], *patterns: str) -> float | None:
    for pattern in patterns:
        for name, value in pose_values.items():
            if re.fullmatch(pattern, name):
                return float(value)
    return None


def compute_pose_metrics(model_path: Path, pose_path: Path, constraints_path: Path | None = None) -> dict[str, Any]:
    _, model, state = load_model(model_path)
    pose_values = coordinate_defaults(model)
    pose_values.update(read_storage_file(pose_path))
    apply_pose_values(model, state, pose_values)

    constraint_spec = read_yaml(constraints_path or _guess_constraints_path(pose_path))
    positions = marker_positions(model, state)
    axis_map = constraint_spec.get("axis_map", {})
    anterior_posterior_axis = axis_index(axis_map.get("anterior_posterior", "x"))

    plane = constraint_spec.get("plane", {})
    reference_markers = plane.get("reference_markers", [])
    constrained_markers = plane.get("constrained_markers", [])
    plane_reference = sum(positions[name][anterior_posterior_axis] for name in reference_markers) / len(reference_markers)
    plane_distances_mm = {
        name: abs(positions[name][anterior_posterior_axis] - plane_reference) * 1000.0 for name in constrained_markers
    }

    sternum_markers = constraint_spec.get("sternum_alignment", {}).get("markers", [])
    sternum_delta_mm = None
    if len(sternum_markers) == 2:
        sternum_delta_mm = (
            positions[sternum_markers[0]][anterior_posterior_axis]
            - positions[sternum_markers[1]][anterior_posterior_axis]
        ) * 1000.0

    metrics = {
        "pose_file": str(pose_path),
        "model_file": str(model_path),
        "plane_reference_markers": reference_markers,
        "plane_distances_mm": plane_distances_mm,
        "plane_mean_distance_mm": sum(plane_distances_mm.values()) / max(len(plane_distances_mm), 1),
        "plane_max_distance_mm": max(plane_distances_mm.values(), default=0.0),
        "sternum_delta_mm": sternum_delta_mm,
        "pelvis_tilt_deg": _first_coordinate(pose_values, r"^pelvis_tilt$"),
        "torso_tilt_deg": _first_coordinate(
            pose_values,
            r"^lumbar_extension$",
            r"^back_extension$",
            r"^torso_pitch$",
            r"^torso_tilt$",
        ),
        "knee_flexion_deg": {
            name: value for name, value in pose_values.items() if re.fullmatch(r"^knee_angle_[lr]$", name)
        },
    }

    if "good" in pose_path.stem.lower():
        metrics["acceptance"] = {
            "plane_mean_lt_15mm": metrics["plane_mean_distance_mm"] < 15.0,
            "plane_max_lt_30mm": metrics["plane_max_distance_mm"] < 30.0,
            "sternum_delta_lt_10mm": sternum_delta_mm is not None and abs(sternum_delta_mm) < 10.0,
            "knees_lt_5deg": all(abs(value) < 5.0 for value in metrics["knee_flexion_deg"].values()),
        }

    return metrics


def write_metrics_json(metrics: dict[str, Any], output_path: Path) -> None:
    ensure_parent_dir(output_path)
    output_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def export_body_transforms(model_path: Path, pose_path: Path) -> dict[str, Any]:
    _, model, state = load_model(model_path)
    pose_values = coordinate_defaults(model)
    pose_values.update(read_storage_file(pose_path))
    apply_pose_values(model, state, pose_values)

    transforms: dict[str, Any] = {"pose_file": str(pose_path), "model_file": str(model_path), "bodies": {}}
    body_set = model.getBodySet()
    for index in range(body_set.getSize()):
        body = body_set.get(index)
        transforms["bodies"][body.getName()] = body_transform(model, state, body.getName())
    return transforms


def write_json(data: dict[str, Any], output_path: Path) -> None:
    ensure_parent_dir(output_path)
    output_path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
