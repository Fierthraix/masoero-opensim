from __future__ import annotations

import math
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

from . import config
from .opensim_utils import apply_pose_values, body_transform, coordinate_defaults, load_model, marker_positions, read_storage_file
from .runtime import UserFacingError, ensure_parent_dir
from .specs import read_yaml

BODY_CONNECTIONS = (
    ("pelvis", "torso"),
    ("pelvis", "femur_r"),
    ("femur_r", "tibia_r"),
    ("tibia_r", "talus_r"),
    ("talus_r", "calcn_r"),
    ("calcn_r", "toes_r"),
    ("pelvis", "femur_l"),
    ("femur_l", "tibia_l"),
    ("tibia_l", "talus_l"),
    ("talus_l", "calcn_l"),
    ("calcn_l", "toes_l"),
    ("torso", "humerus_r"),
    ("humerus_r", "ulna_r"),
    ("ulna_r", "radius_r"),
    ("radius_r", "hand_r"),
    ("torso", "humerus_l"),
    ("humerus_l", "ulna_l"),
    ("ulna_l", "radius_l"),
    ("radius_l", "hand_l"),
)

AXIS_BODIES = (
    "pelvis",
    "torso",
    "femur_r",
    "tibia_r",
    "femur_l",
    "tibia_l",
    "humerus_r",
    "ulna_r",
    "humerus_l",
    "ulna_l",
)

LABEL_MARKERS = {
    "c7",
    "ear_left",
    "ear_right",
    "upper_sternum",
    "lower_sternum",
    "iliac_left",
    "iliac_right",
    "instep_left",
    "instep_right",
}


@dataclass(frozen=True)
class PanelSpec:
    name: str
    yaw_deg: float
    pitch_deg: float


@dataclass(frozen=True)
class SceneData:
    body_origins: dict[str, tuple[float, float, float]]
    body_axes: dict[str, dict[str, tuple[float, float, float]]]
    markers: dict[str, tuple[float, float, float]]


@dataclass(frozen=True)
class ProjectedPoint:
    x: float
    y: float
    depth: float


PANELS = (
    PanelSpec("Sagittal", 0.0, 0.0),
    PanelSpec("Three-Quarter", -38.0, 16.0),
)


def _as_tuple(values: list[float]) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


def _axis_endpoint(origin: tuple[float, float, float], rotation: list[list[float]], axis: int, scale: float) -> tuple[float, float, float]:
    return (
        origin[0] + scale * float(rotation[0][axis]),
        origin[1] + scale * float(rotation[1][axis]),
        origin[2] + scale * float(rotation[2][axis]),
    )


def _load_scene(model_path: Path, pose_path: Path) -> SceneData:
    _, model, state = load_model(model_path)
    pose_values = coordinate_defaults(model)
    pose_values.update(read_storage_file(pose_path))
    apply_pose_values(model, state, pose_values)

    body_origins: dict[str, tuple[float, float, float]] = {}
    body_axes: dict[str, dict[str, tuple[float, float, float]]] = {}
    body_set = model.getBodySet()
    for index in range(body_set.getSize()):
        body = body_set.get(index)
        name = body.getName()
        transform = body_transform(model, state, name)
        origin = _as_tuple(transform["translation_m"])
        rotation = transform["rotation_matrix"]
        body_origins[name] = origin
        if name in AXIS_BODIES:
            body_axes[name] = {
                "x": _axis_endpoint(origin, rotation, 0, 0.08),
                "y": _axis_endpoint(origin, rotation, 1, 0.08),
                "z": _axis_endpoint(origin, rotation, 2, 0.08),
            }

    landmark_spec = read_yaml(config.LANDMARKS_PATH)
    marker_names = [marker["name"] for marker in landmark_spec.get("markers", [])]
    markers = {name: _as_tuple(position) for name, position in marker_positions(model, state, marker_names).items()}
    return SceneData(body_origins=body_origins, body_axes=body_axes, markers=markers)


def _rotate(point: tuple[float, float, float], yaw_deg: float, pitch_deg: float) -> ProjectedPoint:
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    x, y, z = point

    yaw_x = math.cos(yaw) * x - math.sin(yaw) * z
    yaw_z = math.sin(yaw) * x + math.cos(yaw) * z

    pitch_y = math.cos(pitch) * y - math.sin(pitch) * yaw_z
    pitch_z = math.sin(pitch) * y + math.cos(pitch) * yaw_z
    return ProjectedPoint(yaw_x, -pitch_y, pitch_z)


def _project_panel(scene: SceneData, panel: PanelSpec, x0: float, y0: float, width: float, height: float) -> tuple[dict[str, ProjectedPoint], float, float, float]:
    samples = []
    for point in scene.body_origins.values():
        samples.append(_rotate(point, panel.yaw_deg, panel.pitch_deg))
    for point in scene.markers.values():
        samples.append(_rotate(point, panel.yaw_deg, panel.pitch_deg))
    for axes in scene.body_axes.values():
        for point in axes.values():
            samples.append(_rotate(point, panel.yaw_deg, panel.pitch_deg))
    if not samples:
        raise UserFacingError("No scene points available to render.")

    min_x = min(sample.x for sample in samples)
    max_x = max(sample.x for sample in samples)
    min_y = min(sample.y for sample in samples)
    max_y = max(sample.y for sample in samples)
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    scale = min((width - 80.0) / span_x, (height - 120.0) / span_y)

    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0
    offset_x = x0 + (width / 2.0) - (scale * center_x)
    offset_y = y0 + (height / 2.0) - (scale * center_y)

    return ({}, scale, offset_x, offset_y)


def _screen_point(point: tuple[float, float, float], panel: PanelSpec, scale: float, offset_x: float, offset_y: float) -> ProjectedPoint:
    projected = _rotate(point, panel.yaw_deg, panel.pitch_deg)
    return ProjectedPoint(
        x=(projected.x * scale) + offset_x,
        y=(projected.y * scale) + offset_y,
        depth=projected.depth,
    )


def _format_number(value: float) -> str:
    return f"{value:.2f}"


def _svg_text(x: float, y: float, text: str, size: int, fill: str, weight: str = "400") -> str:
    return (
        f'<text x="{_format_number(x)}" y="{_format_number(y)}" fill="{fill}" '
        f'font-family="DejaVu Sans, sans-serif" font-size="{size}" font-weight="{weight}">{text}</text>'
    )


def _render_svg(scene: SceneData, pose_name: str) -> str:
    width = 1500.0
    height = 900.0
    panel_gap = 30.0
    panel_width = (width - 90.0 - panel_gap) / 2.0
    panel_height = height - 160.0
    panel_y = 90.0
    panel_xs = (30.0, 30.0 + panel_width + panel_gap)

    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(width)}" height="{int(height)}" viewBox="0 0 {int(width)} {int(height)}">',
        '<rect width="100%" height="100%" fill="#f4efe4"/>',
        '<rect x="0" y="0" width="100%" height="70" fill="#e4d6bf"/>',
        _svg_text(40, 46, f"Masoero posture render: {pose_name}", 28, "#2f2618", "700"),
        _svg_text(40, 72, "Body origins and Masoero landmarks rendered from solved OpenSim pose", 16, "#5c4e38"),
    ]

    legend_x = width - 320.0
    parts.extend(
        [
            _svg_text(legend_x, 34, "Legend", 16, "#2f2618", "700"),
            '<line x1="1180" y1="52" x2="1214" y2="52" stroke="#5f5a4f" stroke-width="4"/>',
            _svg_text(1225, 57, "body chain", 14, "#5c4e38"),
            '<circle cx="1197" cy="76" r="5" fill="#d9732d"/>',
            _svg_text(1225, 81, "Masoero marker", 14, "#5c4e38"),
            '<line x1="1180" y1="100" x2="1214" y2="100" stroke="#d04444" stroke-width="3"/>',
            _svg_text(1225, 105, "x-axis", 14, "#5c4e38"),
            '<line x1="1180" y1="124" x2="1214" y2="124" stroke="#3a9953" stroke-width="3"/>',
            _svg_text(1225, 129, "y-axis", 14, "#5c4e38"),
            '<line x1="1180" y1="148" x2="1214" y2="148" stroke="#3862b8" stroke-width="3"/>',
            _svg_text(1225, 153, "z-axis", 14, "#5c4e38"),
        ]
    )

    for panel, panel_x in zip(PANELS, panel_xs):
        scale_info = _project_panel(scene, panel, panel_x, panel_y, panel_width, panel_height)
        _, scale, offset_x, offset_y = scale_info
        parts.append(
            f'<rect x="{_format_number(panel_x)}" y="{_format_number(panel_y)}" '
            f'width="{_format_number(panel_width)}" height="{_format_number(panel_height)}" '
            'rx="24" fill="#fbf8f3" stroke="#d7cab3" stroke-width="2"/>'
        )
        parts.append(_svg_text(panel_x + 24.0, panel_y + 36.0, panel.name, 22, "#2f2618", "700"))

        draw_items: list[tuple[float, str]] = []
        for body_a, body_b in BODY_CONNECTIONS:
            if body_a not in scene.body_origins or body_b not in scene.body_origins:
                continue
            start = _screen_point(scene.body_origins[body_a], panel, scale, offset_x, offset_y)
            end = _screen_point(scene.body_origins[body_b], panel, scale, offset_x, offset_y)
            depth = (start.depth + end.depth) / 2.0
            draw_items.append(
                (
                    depth,
                    f'<line x1="{_format_number(start.x)}" y1="{_format_number(start.y)}" '
                    f'x2="{_format_number(end.x)}" y2="{_format_number(end.y)}" '
                    'stroke="#5f5a4f" stroke-width="5" stroke-linecap="round" opacity="0.9"/>',
                )
            )

        for body_name, axes in scene.body_axes.items():
            origin = _screen_point(scene.body_origins[body_name], panel, scale, offset_x, offset_y)
            for axis_name, color in (("x", "#d04444"), ("y", "#3a9953"), ("z", "#3862b8")):
                endpoint = _screen_point(axes[axis_name], panel, scale, offset_x, offset_y)
                depth = (origin.depth + endpoint.depth) / 2.0
                draw_items.append(
                    (
                        depth,
                        f'<line x1="{_format_number(origin.x)}" y1="{_format_number(origin.y)}" '
                        f'x2="{_format_number(endpoint.x)}" y2="{_format_number(endpoint.y)}" '
                        f'stroke="{color}" stroke-width="2.5" stroke-linecap="round" opacity="0.8"/>',
                    )
                )

        for marker_name, point in scene.markers.items():
            marker = _screen_point(point, panel, scale, offset_x, offset_y)
            draw_items.append(
                (
                    marker.depth,
                    f'<circle cx="{_format_number(marker.x)}" cy="{_format_number(marker.y)}" '
                    'r="5" fill="#d9732d" stroke="#fff7eb" stroke-width="2"/>',
                )
            )
            if marker_name in LABEL_MARKERS:
                draw_items.append(
                    (
                        marker.depth,
                        _svg_text(marker.x + 8.0, marker.y - 7.0, marker_name, 11, "#6a5538", "500"),
                    )
                )

        for _, svg in sorted(draw_items, key=lambda item: item[0]):
            parts.append(svg)

    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def _write_svg(svg_text: str, output_path: Path) -> None:
    ensure_parent_dir(output_path)
    output_path.write_text(svg_text, encoding="utf-8")


def _convert_svg_to_png(svg_path: Path, png_path: Path) -> None:
    ensure_parent_dir(png_path)
    if shutil.which("inkscape"):
        subprocess.run(
            ["inkscape", str(svg_path), "--export-filename", str(png_path)],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return
    if shutil.which("magick"):
        subprocess.run(
            ["magick", str(svg_path), str(png_path)],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return
    raise UserFacingError("PNG export requires `inkscape` or `magick` to be available on PATH.")


def render_pose(model_path: Path, pose_path: Path, output_path: Path) -> None:
    scene = _load_scene(model_path, pose_path)
    svg_text = _render_svg(scene, pose_path.stem)
    suffix = output_path.suffix.lower()
    if suffix == ".svg":
        _write_svg(svg_text, output_path)
        return
    if suffix == ".png":
        svg_path = output_path.with_suffix(".svg")
        _write_svg(svg_text, svg_path)
        _convert_svg_to_png(svg_path, output_path)
        return
    raise UserFacingError("Render output must end in `.svg` or `.png`.")
