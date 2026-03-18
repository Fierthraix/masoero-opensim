from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from . import config
from .opensim_utils import (
    apply_pose_values,
    axis_index,
    coordinate_bounds,
    coordinate_defaults,
    coordinate_map,
    coordinate_names,
    coordinate_set,
    coordinate_units,
    load_model,
    marker_positions,
    read_storage_file,
    save_model,
    set_default_pose,
    write_storage_file,
)
from .runtime import UserFacingError, import_dependency
from .specs import read_yaml


@dataclass
class CoordinateVariable:
    name: str
    lower: float
    upper: float
    initial: float


def _first_matching_spec(name: str, specs: list[dict[str, Any]]) -> dict[str, Any] | None:
    for spec in specs:
        if re.fullmatch(spec["pattern"], name):
            return spec
    return None


def _matching_specs(name: str, specs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [spec for spec in specs if re.fullmatch(spec["pattern"], name)]


def build_variables(model: Any, constraints: dict[str, Any]) -> list[CoordinateVariable]:
    variables: list[CoordinateVariable] = []
    movable_specs = constraints.get("movable_coordinates", [])
    for coord in coordinate_set(model):
        spec = _first_matching_spec(coord.getName(), movable_specs)
        if spec is None:
            continue
        name = coord.getName()
        unit = coordinate_units(name)
        if unit == "deg" and "bounds_deg" in spec:
            lower, upper = [float(value) for value in spec["bounds_deg"]]
            initial = float(spec.get("initial_deg", 0.0))
        elif unit == "m" and "bounds_m" in spec:
            lower, upper = [float(value) for value in spec["bounds_m"]]
            initial = float(spec.get("initial_m", 0.0))
        else:
            lower, upper = coordinate_bounds(coord)
            initial = coordinate_defaults(model)[name]
        initial = min(max(initial, lower), upper)
        variables.append(CoordinateVariable(name=name, lower=lower, upper=upper, initial=initial))
    if not variables:
        raise UserFacingError("No movable coordinates matched the constraint spec.")
    return variables


def residual_vector(
    model: Any,
    state: Any,
    base_pose: dict[str, float],
    variables: list[CoordinateVariable],
    candidate: list[float],
    constraints: dict[str, Any],
) -> list[float]:
    pose_values = dict(base_pose)
    for variable, value in zip(variables, candidate):
        pose_values[variable.name] = float(value)
    apply_pose_values(model, state, pose_values)

    axis_map = constraints.get("axis_map", {})
    anterior_posterior_axis = axis_index(axis_map.get("anterior_posterior", "x"))
    positions = marker_positions(model, state)
    residuals: list[float] = []

    plane = constraints.get("plane", {})
    reference_markers = plane.get("reference_markers", [])
    constrained_markers = plane.get("constrained_markers", [])
    if reference_markers and constrained_markers:
        reference = sum(positions[name][anterior_posterior_axis] for name in reference_markers) / len(reference_markers)
        plane_weight = math.sqrt(float(plane.get("weight", 1.0)))
        for name in constrained_markers:
            residuals.append(plane_weight * (positions[name][anterior_posterior_axis] - reference))

    sternum_alignment = constraints.get("sternum_alignment", {})
    sternum_markers = sternum_alignment.get("markers", [])
    if len(sternum_markers) == 2:
        sternum_axis = axis_index(sternum_alignment.get("axis", axis_map.get("anterior_posterior", "x")))
        sternum_weight = math.sqrt(float(sternum_alignment.get("weight", 1.0)))
        residuals.append(
            sternum_weight
            * (positions[sternum_markers[0]][sternum_axis] - positions[sternum_markers[1]][sternum_axis])
        )

    marker_verticality = constraints.get("marker_verticality", {})
    verticality_markers = marker_verticality.get("markers", [])
    if len(verticality_markers) >= 2:
        verticality_axis = axis_index(marker_verticality.get("axis", axis_map.get("anterior_posterior", "x")))
        verticality_weight = math.sqrt(float(marker_verticality.get("weight", 1.0)))
        for left, right in zip(verticality_markers[:-1], verticality_markers[1:]):
            residuals.append(verticality_weight * (positions[left][verticality_axis] - positions[right][verticality_axis]))

    coordinate_targets = constraints.get("coordinate_targets", [])
    current_pose = {variable.name: value for variable, value in zip(variables, candidate)}
    for name, value in current_pose.items():
        for target_spec in _matching_specs(name, coordinate_targets):
            target = float(target_spec.get("target_deg", target_spec.get("target_m", 0.0)))
            weight = math.sqrt(float(target_spec.get("weight", 1.0)))
            residuals.append(weight * (value - target))

    return residuals


def solve_pose(
    pose_name: str,
    model_path: Path,
    constraints_path: Path,
    output_pose_path: Path,
    output_model_path: Path | None,
    initial_pose_path: Path | None = None,
) -> dict[str, float]:
    scipy_optimize = import_dependency(
        "scipy.optimize",
        "SciPy is required for pose solving. Run `UV_CACHE_DIR=/tmp/uv-cache uv sync` first.",
    )
    _, model, state = load_model(model_path)
    constraints = read_yaml(constraints_path)
    variables = build_variables(model, constraints)

    base_pose = coordinate_defaults(model)
    if initial_pose_path is not None and initial_pose_path.exists():
        base_pose.update(read_storage_file(initial_pose_path))

    lower_bounds = [variable.lower for variable in variables]
    upper_bounds = [variable.upper for variable in variables]
    initial_guess = [base_pose.get(variable.name, variable.initial) for variable in variables]

    result = scipy_optimize.least_squares(
        lambda values: residual_vector(model, state, base_pose, variables, list(values), constraints),
        x0=initial_guess,
        bounds=(lower_bounds, upper_bounds),
        max_nfev=400,
    )
    solved_pose = dict(base_pose)
    for variable, value in zip(variables, result.x):
        solved_pose[variable.name] = float(value)

    coord_order = coordinate_names(model)
    write_storage_file(output_pose_path, coord_order, solved_pose)

    if output_model_path is not None:
        set_default_pose(model, solved_pose)
        save_model(model, output_model_path)

    return solved_pose


def default_output_paths(pose_name: str) -> tuple[Path, Path]:
    return (
        config.POSES_DIR / f"pose_{pose_name}.sto",
        config.POSES_DIR / f"pose_{pose_name}.osim",
    )
