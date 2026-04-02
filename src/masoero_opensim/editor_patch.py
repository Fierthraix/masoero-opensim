from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from . import config
from .opensim_utils import add_markers, load_model, normalize_parent_frame_name, read_storage_table, save_model, write_storage_file
from .runtime import UserFacingError, ensure_parent_dir
from .specs import read_yaml, write_yaml


@dataclass(frozen=True)
class AppliedPatchResult:
    patch_path: Path
    output_pose_path: Path
    output_landmarks_path: Path
    output_model_path: Path


def _marker_edits_by_name(marker_edits: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    edits: dict[str, dict[str, Any]] = {}
    for marker_edit in marker_edits:
        name = str(marker_edit["name"])
        edits[name] = {
            "name": name,
            "parent_frame": normalize_parent_frame_name(str(marker_edit["parent_frame"])),
            "location_m": [float(value) for value in marker_edit["location_m"]],
        }
    return edits


def default_patch_path(pose_name: str) -> Path:
    return config.EDITS_DIR / f"{pose_name}.patch.json"


def build_editor_patch(
    *,
    pose_name: str,
    base_pose_file: Path,
    base_landmarks_file: Path,
    pose_edits: dict[str, float],
    marker_edits: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "base_pose_file": str(base_pose_file.resolve()),
        "base_landmarks_file": str(base_landmarks_file.resolve()),
        "marker_edits": [
            {
                "name": str(marker_edit["name"]),
                "parent_frame": normalize_parent_frame_name(str(marker_edit["parent_frame"])),
                "location_m": [float(value) for value in marker_edit["location_m"]],
            }
            for marker_edit in marker_edits
        ],
        "pose_edits": {str(name): float(value) for name, value in sorted(pose_edits.items())},
        "meta": {
            "pose_name": pose_name,
            "written_at": datetime.now(timezone.utc).isoformat(),
        },
    }


def write_editor_patch(path: Path, patch: dict[str, Any]) -> None:
    ensure_parent_dir(path)
    path.write_text(json.dumps(patch, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def read_editor_patch(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise UserFacingError(f"Patch file does not exist: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise UserFacingError(f"Expected a JSON object in `{path}`.")
    return data


def _default_output_paths(patch: dict[str, Any]) -> tuple[Path, Path, Path]:
    pose_name = str(patch.get("meta", {}).get("pose_name") or Path(str(patch["base_pose_file"])).stem)
    return (
        config.POSES_DIR / f"{pose_name}_edited.sto",
        config.SPECS_DIR / f"{pose_name}_edited_landmarks.yaml",
        config.MODELS_DIR / f"{pose_name}_edited_markers.osim",
    )


def apply_editor_patch(
    patch_path: Path,
    *,
    output_pose_path: Path | None = None,
    output_landmarks_path: Path | None = None,
    output_model_path: Path | None = None,
    input_model_path: Path = config.BASE_MODEL_PATH,
) -> AppliedPatchResult:
    patch = read_editor_patch(patch_path)
    default_pose_path, default_landmarks_path, default_model_path = _default_output_paths(patch)
    output_pose_path = output_pose_path or default_pose_path
    output_landmarks_path = output_landmarks_path or default_landmarks_path
    output_model_path = output_model_path or default_model_path

    base_pose_path = Path(str(patch["base_pose_file"]))
    base_landmarks_path = Path(str(patch["base_landmarks_file"]))
    coordinate_names, pose_values = read_storage_table(base_pose_path)
    pose_values.update({str(name): float(value) for name, value in patch.get("pose_edits", {}).items()})
    write_storage_file(output_pose_path, coordinate_names, pose_values)

    landmark_spec = read_yaml(base_landmarks_path)
    marker_specs = landmark_spec.get("markers", [])
    marker_edits = _marker_edits_by_name(list(patch.get("marker_edits", [])))
    updated_markers: list[dict[str, Any]] = []
    seen_names: set[str] = set()
    for marker_spec in marker_specs:
        name = str(marker_spec["name"])
        seen_names.add(name)
        if name in marker_edits:
            updated_markers.append(marker_edits[name])
        else:
            updated_markers.append(
                {
                    "name": name,
                    "parent_frame": normalize_parent_frame_name(str(marker_spec["parent_frame"])),
                    "location_m": [float(value) for value in marker_spec["location_m"]],
                }
            )
    for name in sorted(marker_edits):
        if name not in seen_names:
            updated_markers.append(marker_edits[name])
    landmark_spec["markers"] = updated_markers
    ensure_parent_dir(output_landmarks_path)
    write_yaml(output_landmarks_path, landmark_spec)

    _, model, _ = load_model(input_model_path)
    add_markers(model, updated_markers)
    save_model(model, output_model_path)
    return AppliedPatchResult(
        patch_path=patch_path,
        output_pose_path=output_pose_path,
        output_landmarks_path=output_landmarks_path,
        output_model_path=output_model_path,
    )
