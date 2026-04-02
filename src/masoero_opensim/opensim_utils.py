from __future__ import annotations

import math
import os
import re
from pathlib import Path
from typing import Any

from . import config
from .runtime import UserFacingError, ensure_parent_dir, import_dependency

TRANSLATION_COORDINATE_PATTERNS = (
    r"^pelvis_t[xyz]$",
    r".*_(tx|ty|tz)$",
    r".*translation.*",
)


def load_opensim():
    return import_dependency(
        "opensim",
        "OpenSim Python bindings are unavailable to `uv run`. Install OpenSim and ensure its bindings are visible to the interpreter used by uv.",
    )


def require_existing_file(path: Path, description: str) -> None:
    if not path.exists():
        raise UserFacingError(f"Missing {description}: {path}")


def is_translation_coordinate(name: str) -> bool:
    return any(re.fullmatch(pattern, name) for pattern in TRANSLATION_COORDINATE_PATTERNS)


def coordinate_units(name: str) -> str:
    return "m" if is_translation_coordinate(name) else "deg"


def storage_to_internal(name: str, value: float) -> float:
    if coordinate_units(name) == "m":
        return value
    return math.radians(value)


def internal_to_storage(name: str, value: float) -> float:
    if coordinate_units(name) == "m":
        return value
    return math.degrees(value)


def vector3_to_list(vec: Any) -> list[float]:
    if hasattr(vec, "get"):
        return [float(vec.get(index)) for index in range(3)]
    return [float(vec[index]) for index in range(3)]


def normalize_parent_frame_name(name: str) -> str:
    normalized = str(name).strip()
    if normalized.startswith("/bodyset/"):
        normalized = normalized[len("/bodyset/") :]
    if "/" in normalized:
        normalized = normalized.rsplit("/", 1)[-1]
    return normalized


def load_model(model_path: Path):
    require_existing_file(model_path, "OpenSim model")
    osim = load_opensim()
    model = osim.Model(str(model_path))
    state = model.initSystem()
    return osim, model, state


def coordinate_set(model: Any) -> list[Any]:
    coords = model.getCoordinateSet()
    return [coords.get(index) for index in range(coords.getSize())]


def coordinate_names(model: Any) -> list[str]:
    return [coord.getName() for coord in coordinate_set(model)]


def coordinate_defaults(model: Any) -> dict[str, float]:
    defaults: dict[str, float] = {}
    for coord in coordinate_set(model):
        defaults[coord.getName()] = internal_to_storage(coord.getName(), float(coord.getDefaultValue()))
    return defaults


def coordinate_bounds(coord: Any) -> tuple[float, float]:
    name = coord.getName()
    return (
        internal_to_storage(name, float(coord.getRangeMin())),
        internal_to_storage(name, float(coord.getRangeMax())),
    )


def coordinate_map(model: Any) -> dict[str, Any]:
    return {coord.getName(): coord for coord in coordinate_set(model)}


def unlock_coordinate(coord: Any, state: Any) -> None:
    if hasattr(coord, "getLocked") and coord.getLocked(state):
        coord.setLocked(state, False)


def apply_pose_values(model: Any, state: Any, pose_values: dict[str, float]) -> None:
    coords = coordinate_map(model)
    for name, value in pose_values.items():
        if name not in coords:
            continue
        coord = coords[name]
        unlock_coordinate(coord, state)
        coord.setValue(state, storage_to_internal(name, float(value)))
    model.realizePosition(state)


def marker_positions(model: Any, state: Any, names: list[str] | None = None) -> dict[str, list[float]]:
    selected = set(names or [])
    marker_set = model.getMarkerSet()
    positions: dict[str, list[float]] = {}
    for index in range(marker_set.getSize()):
        marker = marker_set.get(index)
        name = marker.getName()
        if selected and name not in selected:
            continue
        positions[name] = vector3_to_list(marker.getLocationInGround(state))
    return positions


def marker_names(model: Any) -> list[str]:
    marker_set = model.getMarkerSet()
    return [marker_set.get(index).getName() for index in range(marker_set.getSize())]


def add_markers(model: Any, marker_specs: list[dict[str, Any]]) -> list[tuple[str, str]]:
    osim = load_opensim()
    body_set = model.getBodySet()
    attached: list[tuple[str, str]] = []
    existing_markers = {model.getMarkerSet().get(index).getName(): model.getMarkerSet().get(index) for index in range(model.getMarkerSet().getSize())}
    for marker_spec in marker_specs:
        name = marker_spec["name"]
        parent_frame = normalize_parent_frame_name(marker_spec["parent_frame"])
        location = marker_spec["location_m"]
        frame = body_set.get(parent_frame)
        if name in existing_markers:
            marker = existing_markers[name]
            if normalize_parent_frame_name(marker.getParentFrameName()) != parent_frame:
                marker.setParentFrame(frame)
            marker.set_location(osim.Vec3(*location))
        else:
            marker = osim.Marker(name, frame, osim.Vec3(*location))
            model.addMarker(marker)
        attached.append((name, parent_frame))
    model.finalizeConnections()
    return attached


def save_model(model: Any, output_path: Path) -> None:
    ensure_parent_dir(output_path)
    model.printToXML(str(output_path))
    source_geometry = config.EXTERNAL_DIR / "Geometry"
    target_geometry = output_path.parent / "Geometry"
    if source_geometry.exists() and not target_geometry.exists():
        target_geometry.symlink_to(os.path.relpath(source_geometry, output_path.parent), target_is_directory=True)


def set_default_pose(model: Any, pose_values: dict[str, float]) -> None:
    for coord in coordinate_set(model):
        name = coord.getName()
        if name in pose_values:
            coord.setDefaultValue(storage_to_internal(name, pose_values[name]))


def write_storage_file(path: Path, coord_names: list[str], pose_values: dict[str, float]) -> None:
    ensure_parent_dir(path)
    lines = [
        "Coordinates",
        "version=1",
        "nRows=1",
        f"nColumns={len(coord_names) + 1}",
        "inDegrees=yes",
        "endheader",
        "\t".join(["time", *coord_names]),
        "\t".join(["0.0", *[f"{pose_values[name]:.8f}" for name in coord_names]]),
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def read_storage_table(path: Path) -> tuple[list[str], dict[str, float]]:
    require_existing_file(path, "pose storage file")
    lines = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    try:
        header_index = lines.index("endheader")
    except ValueError as exc:
        raise UserFacingError(f"`{path}` is not a valid OpenSim storage file") from exc
    if len(lines) < header_index + 3:
        raise UserFacingError(f"`{path}` does not contain a data row")
    columns = lines[header_index + 1].split()
    values = lines[header_index + 2].split()
    if len(columns) != len(values):
        raise UserFacingError(f"Column/value mismatch in `{path}`")
    pose_values: dict[str, float] = {}
    for name, value in zip(columns[1:], values[1:]):
        pose_values[name] = float(value)
    return columns[1:], pose_values


def read_storage_file(path: Path) -> dict[str, float]:
    _, pose_values = read_storage_table(path)
    return pose_values


def axis_index(axis_name: str) -> int:
    lookup = {"x": 0, "y": 1, "z": 2}
    try:
        return lookup[axis_name]
    except KeyError as exc:
        raise UserFacingError(f"Unsupported axis `{axis_name}`") from exc


def body_transform(model: Any, state: Any, body_name: str) -> dict[str, list[list[float]] | list[float]]:
    body = model.getBodySet().get(body_name)
    transform = body.getTransformInGround(state)
    rotation = transform.R()
    matrix = []
    for row in range(3):
        matrix.append([float(rotation.get(row, col)) for col in range(3)])
    return {
        "translation_m": vector3_to_list(transform.p()),
        "rotation_matrix": matrix,
    }
