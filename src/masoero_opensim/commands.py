from __future__ import annotations

import argparse
from pathlib import Path

from . import config
from .opensim_utils import (
    add_markers,
    coordinate_bounds,
    coordinate_set,
    load_model,
    marker_names,
    read_storage_file,
    save_model,
)
from .pose_solver import default_output_paths, solve_pose
from .reporting import compute_pose_metrics, export_body_transforms, write_json, write_metrics_json
from .runtime import UserFacingError
from .specs import read_yaml


def verify_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify OpenSim availability and inspect the base model.")
    parser.add_argument("--model", type=Path, default=config.BASE_MODEL_PATH)
    args = parser.parse_args(argv)

    osim, model, _ = load_model(args.model)
    print(f"OpenSim version: {osim.GetVersion()}")
    print(f"Model: {args.model}")
    print("Coordinates:")
    for coord in coordinate_set(model):
        lower, upper = coordinate_bounds(coord)
        print(f"  {coord.getName():<24} default={coord.getDefaultValue():>8.4f} range=[{lower:.3f}, {upper:.3f}]")
    print("Markers:")
    for name in marker_names(model):
        print(f"  {name}")
    return 0


def add_markers_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Add Masoero markers to the Rajagopal model.")
    parser.add_argument("--input-model", type=Path, default=config.BASE_MODEL_PATH)
    parser.add_argument("--landmarks", type=Path, default=config.LANDMARKS_PATH)
    parser.add_argument("--output-model", type=Path, default=config.MARKER_MODEL_PATH)
    args = parser.parse_args(argv)

    _, model, _ = load_model(args.input_model)
    landmark_spec = read_yaml(args.landmarks)
    markers = landmark_spec.get("markers", [])
    attached = add_markers(model, markers)
    save_model(model, args.output_model)
    print(f"Saved marker model to {args.output_model}")
    for name, parent_frame in attached:
        print(f"  {name} -> {parent_frame}")
    return 0


def solve_pose_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Solve a static pose from YAML constraints.")
    parser.add_argument("pose_name", choices=["good", "bad"])
    parser.add_argument("--model", type=Path, default=config.MARKER_MODEL_PATH)
    parser.add_argument("--constraints", type=Path)
    parser.add_argument("--output-pose", type=Path)
    parser.add_argument("--output-model", type=Path)
    parser.add_argument("--initial-pose", type=Path)
    parser.add_argument("--skip-posed-model", action="store_true")
    args = parser.parse_args(argv)

    default_pose_path, default_model_path = default_output_paths(args.pose_name)
    constraints_path = args.constraints or config.constraint_path_for_pose(args.pose_name)
    output_pose_path = args.output_pose or default_pose_path
    output_model_path = None if args.skip_posed_model else (args.output_model or default_model_path)
    initial_pose_path = args.initial_pose

    if args.pose_name == "bad" and initial_pose_path is None:
        candidate = config.POSES_DIR / "pose_good.sto"
        if candidate.exists():
            initial_pose_path = candidate

    solved_pose = solve_pose(
        pose_name=args.pose_name,
        model_path=args.model,
        constraints_path=constraints_path,
        output_pose_path=output_pose_path,
        output_model_path=output_model_path,
        initial_pose_path=initial_pose_path,
    )
    print(f"Solved pose `{args.pose_name}` to {output_pose_path}")
    if output_model_path is not None:
        print(f"Wrote posed model to {output_model_path}")
    print("Solved coordinates:")
    for name in sorted(solved_pose):
        print(f"  {name}: {solved_pose[name]:.4f}")
    return 0


def export_metrics_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Export JSON posture metrics for a solved pose.")
    parser.add_argument("pose_file", type=Path)
    parser.add_argument("--model", type=Path, default=config.MARKER_MODEL_PATH)
    parser.add_argument("--constraints", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    output_path = args.output or (config.REPORTS_DIR / f"{args.pose_file.stem}_metrics.json")
    metrics = compute_pose_metrics(args.model, args.pose_file, args.constraints)
    write_metrics_json(metrics, output_path)
    print(f"Wrote metrics to {output_path}")
    return 0


def export_transforms_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Export body transforms for a solved pose.")
    parser.add_argument("pose_file", type=Path)
    parser.add_argument("--model", type=Path, default=config.MARKER_MODEL_PATH)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    output_path = args.output or (config.REPORTS_DIR / f"{args.pose_file.stem}_transforms.json")
    data = export_body_transforms(args.model, args.pose_file)
    write_json(data, output_path)
    print(f"Wrote transforms to {output_path}")
    return 0


def run_entrypoint(entrypoint, argv: list[str] | None = None) -> int:
    try:
        return entrypoint(argv)
    except UserFacingError as exc:
        print(f"error: {exc}")
        return 1
