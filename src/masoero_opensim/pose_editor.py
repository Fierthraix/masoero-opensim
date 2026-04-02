from __future__ import annotations

import json
import threading
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import numpy as np

from . import config
from .editor_patch import build_editor_patch, default_patch_path, write_editor_patch
from .mesh_viewer import _geometry_dir_for_model, _parse_vtp_mesh, _read_mesh_specs
from .opensim_utils import (
    apply_pose_values,
    body_transform,
    coordinate_defaults,
    coordinate_units,
    load_model,
    load_opensim,
    read_storage_file,
    vector3_to_list,
)
from .pose_solver import build_variables
from .runtime import UserFacingError
from .specs import read_yaml

_MESH_CACHE: dict[str, tuple[np.ndarray, np.ndarray]] = {}


def _guess_constraints_path(pose_path: Path) -> Path:
    name = pose_path.stem.lower()
    if "good" in name:
        return config.GOOD_CONSTRAINTS_PATH
    if "bad" in name:
        return config.BAD_CONSTRAINTS_PATH
    raise UserFacingError("Pass `--constraints` when the pose filename does not include `good` or `bad`.")


def _round_list(values: list[float]) -> list[float]:
    return [round(float(value), 6) for value in values]


def _coordinate_group(name: str) -> str:
    if name.startswith("pelvis_"):
        return "pelvis"
    if name.startswith(("lumbar", "back", "torso", "neck", "head")):
        return "spine"
    if name.startswith(("hip_", "knee_", "ankle_", "subtalar_", "mtp_")):
        return "legs"
    if name.startswith(("arm_", "elbow_", "pro_sup_", "wrist_")):
        return "arms"
    return "other"


def _mesh_geometry(mesh_path: Path) -> tuple[np.ndarray, np.ndarray]:
    cache_key = str(mesh_path.resolve())
    cached = _MESH_CACHE.get(cache_key)
    if cached is None:
        cached = _parse_vtp_mesh(mesh_path)
        _MESH_CACHE[cache_key] = cached
    return cached


def _coerce_pose_edits(raw_pose_edits: Any) -> dict[str, float]:
    if raw_pose_edits is None:
        return {}
    if not isinstance(raw_pose_edits, dict):
        raise UserFacingError("Expected `pose_edits` to be a JSON object.")
    return {str(name): float(value) for name, value in raw_pose_edits.items()}


def _coerce_marker_edits(raw_marker_edits: Any) -> list[dict[str, Any]]:
    if raw_marker_edits is None:
        return []
    if isinstance(raw_marker_edits, dict):
        values = raw_marker_edits.values()
    elif isinstance(raw_marker_edits, list):
        values = raw_marker_edits
    else:
        raise UserFacingError("Expected `marker_edits` to be a JSON object or array.")
    edits: list[dict[str, Any]] = []
    for item in values:
        if not isinstance(item, dict):
            raise UserFacingError("Each marker edit must be a JSON object.")
        edits.append(
            {
                "name": str(item["name"]),
                "parent_frame": str(item["parent_frame"]),
                "location_m": [float(value) for value in item["location_m"]],
            }
        )
    return edits


def _apply_marker_edits(model: Any, pose_values: dict[str, float], marker_edits: list[dict[str, Any]]):
    osim = load_opensim()
    marker_set = model.getMarkerSet()
    body_set = model.getBodySet()
    frame_changed = False
    for marker_edit in marker_edits:
        marker = marker_set.get(marker_edit["name"])
        parent_frame = marker_edit["parent_frame"]
        if marker.getParentFrameName() != parent_frame:
            marker.setParentFrame(body_set.get(parent_frame))
            frame_changed = True
    if frame_changed:
        model.finalizeConnections()
    state = model.initSystem()
    apply_pose_values(model, state, pose_values)
    for marker_edit in marker_edits:
        marker = marker_set.get(marker_edit["name"])
        marker.set_location(osim.Vec3(*marker_edit["location_m"]))
    model.realizePosition(state)
    return state


def build_editor_scene(
    *,
    model_path: Path,
    pose_path: Path,
    landmarks_path: Path,
    constraints_path: Path | None = None,
    pose_edits: dict[str, float] | None = None,
    marker_edits: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    _, model, state = load_model(model_path)
    pose_values = coordinate_defaults(model)
    pose_values.update(read_storage_file(pose_path))
    pose_values.update(pose_edits or {})
    marker_edits = marker_edits or []
    if marker_edits:
        state = _apply_marker_edits(model, pose_values, marker_edits)
    else:
        apply_pose_values(model, state, pose_values)

    body_transforms: dict[str, dict[str, Any]] = {}
    body_set = model.getBodySet()
    for index in range(body_set.getSize()):
        body = body_set.get(index)
        transform = body_transform(model, state, body.getName())
        body_transforms[body.getName()] = {
            "translation_m": _round_list(transform["translation_m"]),
            "rotation_matrix": [[round(float(value), 6) for value in row] for row in transform["rotation_matrix"]],
        }

    mesh_specs = _read_mesh_specs(model_path)
    geometry_dir = _geometry_dir_for_model(model_path)
    meshes: list[dict[str, Any]] = []
    for spec in mesh_specs:
        mesh_path = geometry_dir / spec.mesh_file
        vertices, faces = _mesh_geometry(mesh_path)
        scaled = vertices * np.asarray(spec.scale_factors, dtype=np.float32)
        rotation = np.asarray(body_transforms[spec.body_name]["rotation_matrix"], dtype=np.float32)
        translation = np.asarray(body_transforms[spec.body_name]["translation_m"], dtype=np.float32)
        posed = (scaled @ rotation.T) + translation
        meshes.append(
            {
                "name": f"{spec.body_name}:{spec.mesh_file}",
                "body": spec.body_name,
                "color": [round(float(value), 6) for value in spec.color],
                "positions": posed.reshape(-1).round(6).tolist(),
                "indices": faces.reshape(-1).tolist(),
            }
        )

    marker_set = model.getMarkerSet()
    landmark_spec = read_yaml(landmarks_path)
    markers: list[dict[str, Any]] = []
    for marker_spec in landmark_spec.get("markers", []):
        marker = marker_set.get(marker_spec["name"])
        markers.append(
            {
                "name": marker.getName(),
                "parent_frame": marker.getParentFrameName(),
                "location_m": _round_list(vector3_to_list(marker.get_location())),
                "world_position": _round_list(vector3_to_list(marker.getLocationInGround(state))),
            }
        )

    constraints = read_yaml(constraints_path or _guess_constraints_path(pose_path))
    coordinates: list[dict[str, Any]] = []
    for variable in build_variables(model, constraints):
        coordinates.append(
            {
                "name": variable.name,
                "group": _coordinate_group(variable.name),
                "unit": coordinate_units(variable.name),
                "lower": round(float(variable.lower), 6),
                "upper": round(float(variable.upper), 6),
                "value": round(float(pose_values.get(variable.name, variable.initial)), 6),
            }
        )
    coordinates.sort(key=lambda item: (item["group"], item["name"]))

    return {
        "pose": pose_path.stem,
        "model": model_path.name,
        "landmarks_file": str(landmarks_path),
        "constraints_file": str(constraints_path or _guess_constraints_path(pose_path)),
        "meshes": meshes,
        "markers": markers,
        "body_transforms": body_transforms,
        "coordinates": coordinates,
    }


def _editor_html(payload: dict[str, Any]) -> str:
    payload_json = json.dumps(payload, separators=(",", ":"))
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{payload["scene"]["pose"]} editor</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #e8decc;
      --panel: rgba(251, 248, 243, 0.9);
      --border: #c9ba9c;
      --text: #2d2417;
      --muted: #685a43;
      --accent: #c36a2d;
      --accent-strong: #9d4e1f;
      --selected: #214f8f;
      --surface: rgba(255,255,255,0.58);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "DejaVu Sans", sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at top left, rgba(255,255,255,0.45), transparent 30%),
        linear-gradient(180deg, #eadfcf 0%, var(--bg) 100%);
    }}
    .shell {{
      display: grid;
      grid-template-columns: 360px minmax(0, 1fr);
      gap: 18px;
      min-height: 100vh;
      padding: 18px;
    }}
    .panel {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 22px;
      padding: 18px;
      backdrop-filter: blur(6px);
      box-shadow: 0 10px 30px rgba(70, 56, 34, 0.08);
    }}
    .sidebar {{
      display: grid;
      gap: 14px;
      align-content: start;
      max-height: calc(100vh - 36px);
      overflow: auto;
    }}
    .viewer {{
      position: relative;
      overflow: hidden;
      min-height: calc(100vh - 36px);
      padding: 0;
    }}
    canvas {{
      display: block;
      width: 100%;
      height: 100%;
    }}
    h1 {{
      margin: 0 0 6px;
      font-size: 30px;
      line-height: 1.1;
    }}
    h2 {{
      margin: 0 0 10px;
      font-size: 16px;
    }}
    p {{
      margin: 0;
      color: var(--muted);
      line-height: 1.45;
    }}
    .row {{
      display: flex;
      gap: 8px;
      align-items: center;
      justify-content: space-between;
    }}
    .mode-switch,
    .actions {{
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
    }}
    button {{
      appearance: none;
      border: 1px solid var(--border);
      background: rgba(255,255,255,0.65);
      color: var(--text);
      border-radius: 12px;
      padding: 10px 12px;
      font: inherit;
      cursor: pointer;
    }}
    button.primary {{
      background: var(--accent);
      color: white;
      border-color: var(--accent-strong);
    }}
    button.active {{
      background: var(--selected);
      color: white;
      border-color: #183d6f;
    }}
    .status {{
      padding: 10px 12px;
      border-radius: 14px;
      background: rgba(255,255,255,0.5);
      border: 1px solid rgba(201, 186, 156, 0.7);
      color: var(--muted);
      font-size: 13px;
    }}
    .section {{
      padding: 14px;
      border-radius: 18px;
      background: var(--surface);
      border: 1px solid rgba(201, 186, 156, 0.7);
      display: grid;
      gap: 10px;
    }}
    .meta-grid {{
      display: grid;
      gap: 6px;
      font-size: 13px;
      color: var(--muted);
    }}
    .marker-list {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 6px;
      max-height: 200px;
      overflow: auto;
    }}
    .marker-item {{
      text-align: left;
      padding: 8px 10px;
      font-size: 13px;
    }}
    .marker-item.selected {{
      background: rgba(33, 79, 143, 0.12);
      border-color: rgba(33, 79, 143, 0.5);
      color: var(--selected);
    }}
    .xyz-grid {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }}
    .control-block {{
      display: grid;
      gap: 6px;
    }}
    label {{
      font-size: 12px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    input[type="number"] {{
      width: 100%;
      border: 1px solid rgba(201, 186, 156, 0.9);
      border-radius: 10px;
      padding: 8px 9px;
      background: rgba(255,255,255,0.82);
      color: var(--text);
      font: inherit;
    }}
    .coordinate-groups {{
      display: grid;
      gap: 10px;
      max-height: 330px;
      overflow: auto;
    }}
    .coordinate-group {{
      display: grid;
      gap: 6px;
    }}
    .coordinate-group h3 {{
      margin: 0;
      font-size: 13px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }}
    .coordinate-row {{
      display: grid;
      gap: 5px;
      padding: 8px 10px;
      background: rgba(255,255,255,0.52);
      border: 1px solid rgba(201, 186, 156, 0.65);
      border-radius: 12px;
    }}
    .coordinate-row .topline {{
      display: flex;
      justify-content: space-between;
      gap: 10px;
      font-size: 13px;
    }}
    input[type="range"] {{
      width: 100%;
    }}
    .hud {{
      position: absolute;
      right: 18px;
      top: 18px;
      padding: 12px 14px;
      border-radius: 14px;
      background: rgba(251,248,243,0.86);
      border: 1px solid rgba(201, 186, 156, 0.85);
      color: var(--muted);
      font-size: 13px;
      max-width: 280px;
    }}
    .hidden {{
      display: none;
    }}
    @media (max-width: 1080px) {{
      .shell {{ grid-template-columns: 1fr; }}
      .viewer {{ min-height: 72vh; }}
      .sidebar {{ max-height: none; }}
    }}
  </style>
  <script type="importmap">
  {{
    "imports": {{
      "three": "/vendor/three/build/three.module.js",
      "three/addons/": "/vendor/three/examples/jsm/"
    }}
  }}
  </script>
</head>
<body>
  <div class="shell">
    <aside class="panel sidebar">
      <section>
        <h1>{payload["scene"]["pose"]}</h1>
        <p>Visual editor for landmarks and pose coordinates. Final PNG/SVG rendering stays on the existing render command.</p>
      </section>
      <div class="status" id="status">Ready.</div>
      <section class="section">
        <div class="meta-grid">
          <div><strong>Model:</strong> {payload["scene"]["model"]}</div>
          <div><strong>Patch:</strong> {payload["default_patch_path"]}</div>
        </div>
        <div class="mode-switch">
          <button id="landmark-mode" class="active">Landmarks</button>
          <button id="pose-mode">Pose</button>
        </div>
        <div class="actions">
          <button id="center-view">Center View</button>
          <button id="reset-edits">Reset Edits</button>
          <button id="export-patch" class="primary">Export Patch</button>
        </div>
      </section>
      <section class="section" id="landmark-panel">
        <div class="row">
          <h2>Landmarks</h2>
          <span id="marker-count"></span>
        </div>
        <p>Click a marker in the mesh or list, then drag the local-axis gizmo or type exact body-frame XYZ values.</p>
        <div class="marker-list" id="marker-list"></div>
        <div id="selected-marker-empty">No marker selected.</div>
        <div id="selected-marker-panel" class="hidden">
          <div class="meta-grid">
            <div><strong>Name:</strong> <span id="marker-name"></span></div>
            <div><strong>Parent Frame:</strong> <span id="marker-parent"></span></div>
          </div>
          <div class="xyz-grid">
            <div class="control-block">
              <label for="marker-x">X</label>
              <input id="marker-x" type="number" step="0.001">
            </div>
            <div class="control-block">
              <label for="marker-y">Y</label>
              <input id="marker-y" type="number" step="0.001">
            </div>
            <div class="control-block">
              <label for="marker-z">Z</label>
              <input id="marker-z" type="number" step="0.001">
            </div>
          </div>
        </div>
      </section>
      <section class="section hidden" id="pose-panel">
        <div class="row">
          <h2>Pose Controls</h2>
          <span id="coord-count"></span>
        </div>
        <p>Adjust coordinate sliders to preview body repositioning. Values are stored in the same units as the `.sto` file.</p>
        <div class="coordinate-groups" id="coordinate-groups"></div>
      </section>
    </aside>
    <section class="panel viewer">
      <div class="hud">
        Landmark mode: select a point and drag its local axes.<br>
        Pose mode: use sliders for pelvis, spine, legs, and arms.<br>
        Export writes a non-destructive patch JSON.
      </div>
      <canvas id="viewer"></canvas>
    </section>
  </div>
  <script type="module">
    import * as THREE from "three";
    import {{ OrbitControls }} from "three/addons/controls/OrbitControls.js";
    import {{ TransformControls }} from "three/addons/controls/TransformControls.js";

    const payload = {payload_json};
    const canvas = document.getElementById("viewer");
    const renderer = new THREE.WebGLRenderer({{ canvas, antialias: true, alpha: true }});
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf7f3ea);

    const camera = new THREE.PerspectiveCamera(35, 1, 0.01, 100);
    const controls = new OrbitControls(camera, canvas);
    controls.enableDamping = true;

    scene.add(new THREE.HemisphereLight(0xfff7ee, 0x9b8e76, 2.2));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.8);
    keyLight.position.set(2.8, 3.6, 2.2);
    scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0xcfd8ff, 0.9);
    fillLight.position.set(-2.0, 1.4, -1.8);
    scene.add(fillLight);

    const rootGroup = new THREE.Group();
    const meshGroup = new THREE.Group();
    const markerGroup = new THREE.Group();
    rootGroup.add(meshGroup);
    rootGroup.add(markerGroup);
    scene.add(rootGroup);

    const markerGeometry = new THREE.SphereGeometry(0.011, 18, 18);
    const markerMaterial = new THREE.MeshStandardMaterial({{
      color: 0xc36a2d,
      emissive: 0x67351a,
      emissiveIntensity: 0.15,
      roughness: 0.48
    }});
    const markerSelectedMaterial = new THREE.MeshStandardMaterial({{
      color: 0x214f8f,
      emissive: 0x132848,
      emissiveIntensity: 0.2,
      roughness: 0.4
    }});

    const transformTarget = new THREE.Object3D();
    scene.add(transformTarget);
    const transformControls = new TransformControls(camera, renderer.domElement);
    transformControls.setMode("translate");
    transformControls.space = "local";
    transformControls.size = 0.85;
    transformControls.visible = false;
    scene.add(transformControls);

    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(2.2, 80),
      new THREE.MeshStandardMaterial({{ color: 0xe7dcc8, roughness: 0.95, metalness: 0 }})
    );
    floor.rotation.x = -Math.PI / 2;
    scene.add(floor);

    const raycaster = new THREE.Raycaster();
    const pointer = new THREE.Vector2();
    const markerObjects = new Map();
    const coordinateInputs = new Map();

    let currentScene = payload.scene;
    let mode = "landmarks";
    let selectedMarkerName = null;
    let poseEdits = {{}};
    let markerEdits = {{}};
    let previewTimer = null;
    let previewToken = 0;

    const statusEl = document.getElementById("status");
    const markerListEl = document.getElementById("marker-list");
    const coordinateGroupsEl = document.getElementById("coordinate-groups");
    const landmarkPanelEl = document.getElementById("landmark-panel");
    const posePanelEl = document.getElementById("pose-panel");
    const markerCountEl = document.getElementById("marker-count");
    const coordCountEl = document.getElementById("coord-count");
    const selectedMarkerEmptyEl = document.getElementById("selected-marker-empty");
    const selectedMarkerPanelEl = document.getElementById("selected-marker-panel");
    const markerNameEl = document.getElementById("marker-name");
    const markerParentEl = document.getElementById("marker-parent");
    const markerXEl = document.getElementById("marker-x");
    const markerYEl = document.getElementById("marker-y");
    const markerZEl = document.getElementById("marker-z");

    function setStatus(text) {{
      statusEl.textContent = text;
    }}

    function buildMatrixFromTransform(transform) {{
      const r = transform.rotation_matrix;
      return new THREE.Matrix4().set(
        r[0][0], r[0][1], r[0][2], transform.translation_m[0],
        r[1][0], r[1][1], r[1][2], transform.translation_m[1],
        r[2][0], r[2][1], r[2][2], transform.translation_m[2],
        0, 0, 0, 1
      );
    }}

    function quaternionForBody(bodyName) {{
      const transform = currentScene.body_transforms[bodyName];
      const matrix = buildMatrixFromTransform(transform);
      const position = new THREE.Vector3();
      const quaternion = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      matrix.decompose(position, quaternion, scale);
      return quaternion;
    }}

    function worldToLocal(bodyName, worldPosition) {{
      const transform = currentScene.body_transforms[bodyName];
      const inverse = buildMatrixFromTransform(transform).invert();
      return worldPosition.clone().applyMatrix4(inverse);
    }}

    function findMarker(name) {{
      return currentScene.markers.find((marker) => marker.name === name) || null;
    }}

    function effectiveMarkerState(name) {{
      const marker = findMarker(name);
      return markerEdits[name] || marker;
    }}

    function effectivePoseValue(name) {{
      if (Object.prototype.hasOwnProperty.call(poseEdits, name)) {{
        return poseEdits[name];
      }}
      const coordinate = currentScene.coordinates.find((item) => item.name === name);
      return coordinate ? coordinate.value : 0;
    }}

    function updateSelectedMarkerPanel() {{
      if (!selectedMarkerName) {{
        selectedMarkerEmptyEl.classList.remove("hidden");
        selectedMarkerPanelEl.classList.add("hidden");
        markerNameEl.textContent = "";
        markerParentEl.textContent = "";
        return;
      }}
      const marker = effectiveMarkerState(selectedMarkerName);
      selectedMarkerEmptyEl.classList.add("hidden");
      selectedMarkerPanelEl.classList.remove("hidden");
      markerNameEl.textContent = marker.name;
      markerParentEl.textContent = marker.parent_frame;
      markerXEl.value = Number(marker.location_m[0]).toFixed(6);
      markerYEl.value = Number(marker.location_m[1]).toFixed(6);
      markerZEl.value = Number(marker.location_m[2]).toFixed(6);
    }}

    function attachTransformToSelection() {{
      if (!selectedMarkerName || mode !== "landmarks") {{
        transformControls.detach();
        transformControls.visible = false;
        return;
      }}
      const marker = findMarker(selectedMarkerName);
      if (!marker) {{
        transformControls.detach();
        transformControls.visible = false;
        return;
      }}
      transformTarget.position.set(...marker.world_position);
      transformTarget.quaternion.copy(quaternionForBody(marker.parent_frame));
      transformControls.visible = true;
      transformControls.attach(transformTarget);
    }}

    function setSelectedMarker(name) {{
      selectedMarkerName = name;
      markerObjects.forEach((mesh, markerName) => {{
        mesh.material = markerName === selectedMarkerName ? markerSelectedMaterial : markerMaterial;
      }});
      [...markerListEl.querySelectorAll(".marker-item")].forEach((button) => {{
        button.classList.toggle("selected", button.dataset.marker === selectedMarkerName);
      }});
      updateSelectedMarkerPanel();
      attachTransformToSelection();
    }}

    function buildMarkerList() {{
      markerListEl.innerHTML = "";
      markerCountEl.textContent = `${{currentScene.markers.length}} points`;
      for (const marker of currentScene.markers) {{
        const button = document.createElement("button");
        button.type = "button";
        button.className = "marker-item";
        button.dataset.marker = marker.name;
        button.textContent = marker.name;
        button.addEventListener("click", () => setSelectedMarker(marker.name));
        markerListEl.appendChild(button);
      }}
    }}

    function groupCoordinates(coordinates) {{
      const grouped = new Map();
      for (const coordinate of coordinates) {{
        if (!grouped.has(coordinate.group)) {{
          grouped.set(coordinate.group, []);
        }}
        grouped.get(coordinate.group).push(coordinate);
      }}
      return grouped;
    }}

    function updateCoordinateInputs() {{
      for (const [name, refs] of coordinateInputs.entries()) {{
        const value = effectivePoseValue(name);
        refs.range.value = String(value);
        refs.number.value = Number(value).toFixed(6);
        refs.value.textContent = `${{Number(value).toFixed(3)}} ${{refs.unit}}`;
      }}
    }}

    function buildCoordinateControls() {{
      coordinateGroupsEl.innerHTML = "";
      coordinateInputs.clear();
      coordCountEl.textContent = `${{currentScene.coordinates.length}} coords`;
      for (const [groupName, items] of groupCoordinates(currentScene.coordinates)) {{
        const group = document.createElement("section");
        group.className = "coordinate-group";
        const heading = document.createElement("h3");
        heading.textContent = groupName;
        group.appendChild(heading);
        for (const coordinate of items) {{
          const row = document.createElement("div");
          row.className = "coordinate-row";
          const topline = document.createElement("div");
          topline.className = "topline";
          const name = document.createElement("span");
          name.textContent = coordinate.name;
          const value = document.createElement("span");
          value.textContent = `${{Number(coordinate.value).toFixed(3)}} ${{coordinate.unit}}`;
          topline.appendChild(name);
          topline.appendChild(value);
          row.appendChild(topline);

          const range = document.createElement("input");
          range.type = "range";
          range.min = String(coordinate.lower);
          range.max = String(coordinate.upper);
          range.step = coordinate.unit === "m" ? "0.001" : "0.1";
          range.value = String(coordinate.value);
          row.appendChild(range);

          const number = document.createElement("input");
          number.type = "number";
          number.min = String(coordinate.lower);
          number.max = String(coordinate.upper);
          number.step = coordinate.unit === "m" ? "0.001" : "0.1";
          number.value = Number(coordinate.value).toFixed(6);
          row.appendChild(number);

          const applyValue = (rawValue) => {{
            const nextValue = Number(rawValue);
            if (Number.isNaN(nextValue)) return;
            poseEdits[coordinate.name] = nextValue;
            updateCoordinateInputs();
            schedulePreview();
          }};

          range.addEventListener("input", (event) => applyValue(event.target.value));
          number.addEventListener("change", (event) => applyValue(event.target.value));
          coordinateInputs.set(coordinate.name, {{ range, number, value, unit: coordinate.unit }});
          group.appendChild(row);
        }}
        coordinateGroupsEl.appendChild(group);
      }}
    }}

    function frameScene() {{
      const bounds = new THREE.Box3().setFromObject(rootGroup);
      const center = bounds.getCenter(new THREE.Vector3());
      const size = bounds.getSize(new THREE.Vector3());
      const maxDim = Math.max(size.x, size.y, size.z, 0.3);
      const fitHeightDistance = maxDim / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov * 0.5)));
      const fitWidthDistance = fitHeightDistance / Math.max(camera.aspect, 1e-3);
      const distance = 1.35 * Math.max(fitHeightDistance, fitWidthDistance);
      camera.near = Math.max(distance / 200, 0.01);
      camera.far = distance * 20;
      camera.position.set(center.x + distance * 0.42, center.y + size.y * 0.08, center.z + distance * 0.98);
      controls.target.copy(center);
      camera.lookAt(center);
      camera.updateProjectionMatrix();

      floor.geometry.dispose();
      floor.geometry = new THREE.CircleGeometry(Math.max(size.x, size.z, size.y) * 1.15 + 0.3, 80);
      floor.position.set(center.x, bounds.min.y - 0.015, center.z);
    }}

    function buildScene() {{
      meshGroup.clear();
      markerGroup.clear();
      markerObjects.clear();
      for (const meshData of currentScene.meshes) {{
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute("position", new THREE.Float32BufferAttribute(meshData.positions, 3));
        geometry.setIndex(meshData.indices);
        geometry.computeVertexNormals();
        const material = new THREE.MeshStandardMaterial({{
          color: new THREE.Color(...meshData.color),
          roughness: 0.72,
          metalness: 0.04
        }});
        const mesh = new THREE.Mesh(geometry, material);
        meshGroup.add(mesh);
      }}
      for (const marker of currentScene.markers) {{
        const dot = new THREE.Mesh(markerGeometry, markerMaterial);
        dot.position.set(...marker.world_position);
        dot.userData.markerName = marker.name;
        markerGroup.add(dot);
        markerObjects.set(marker.name, dot);
      }}
      frameScene();
      setSelectedMarker(selectedMarkerName);
    }}

    function resize() {{
      const width = canvas.clientWidth;
      const height = canvas.clientHeight;
      renderer.setSize(width, height, false);
      camera.aspect = width / Math.max(height, 1);
      frameScene();
    }}

    function gatherPayload() {{
      return {{
        pose_edits: poseEdits,
        marker_edits: Object.values(markerEdits)
      }};
    }}

    async function fetchPreview() {{
      const token = ++previewToken;
      setStatus("Refreshing preview...");
      const response = await fetch("/api/preview", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify(gatherPayload())
      }});
      const data = await response.json();
      if (!response.ok) {{
        throw new Error(data.error || "Preview failed");
      }}
      if (token !== previewToken) {{
        return;
      }}
      currentScene = data.scene;
      buildScene();
      updateSelectedMarkerPanel();
      updateCoordinateInputs();
      setStatus("Preview updated.");
    }}

    function schedulePreview() {{
      window.clearTimeout(previewTimer);
      previewTimer = window.setTimeout(() => {{
        fetchPreview().catch((error) => setStatus(error.message));
      }}, 160);
    }}

    async function exportPatch() {{
      setStatus("Writing patch...");
      const response = await fetch("/api/export-patch", {{
        method: "POST",
        headers: {{ "Content-Type": "application/json" }},
        body: JSON.stringify(gatherPayload())
      }});
      const data = await response.json();
      if (!response.ok) {{
        throw new Error(data.error || "Export failed");
      }}
      setStatus(`Patch written to ${{data.output_path}}`);
    }}

    function updateMarkerEditFromInputs() {{
      if (!selectedMarkerName) return;
      const baseMarker = findMarker(selectedMarkerName);
      if (!baseMarker) return;
      markerEdits[selectedMarkerName] = {{
        name: selectedMarkerName,
        parent_frame: baseMarker.parent_frame,
        location_m: [
          Number(markerXEl.value),
          Number(markerYEl.value),
          Number(markerZEl.value),
        ],
      }};
      schedulePreview();
    }}

    function setMode(nextMode) {{
      mode = nextMode;
      document.getElementById("landmark-mode").classList.toggle("active", mode === "landmarks");
      document.getElementById("pose-mode").classList.toggle("active", mode === "pose");
      landmarkPanelEl.classList.toggle("hidden", mode !== "landmarks");
      posePanelEl.classList.toggle("hidden", mode !== "pose");
      attachTransformToSelection();
    }}

    transformControls.addEventListener("dragging-changed", (event) => {{
      controls.enabled = !event.value;
      if (!event.value) {{
        schedulePreview();
      }}
    }});

    transformControls.addEventListener("objectChange", () => {{
      if (!selectedMarkerName) return;
      const marker = findMarker(selectedMarkerName);
      if (!marker) return;
      const local = worldToLocal(marker.parent_frame, transformTarget.position);
      markerEdits[selectedMarkerName] = {{
        name: selectedMarkerName,
        parent_frame: marker.parent_frame,
        location_m: [
          Number(local.x.toFixed(6)),
          Number(local.y.toFixed(6)),
          Number(local.z.toFixed(6)),
        ],
      }};
      updateSelectedMarkerPanel();
      schedulePreview();
    }});

    canvas.addEventListener("pointerdown", (event) => {{
      if (mode !== "landmarks") return;
      const rect = canvas.getBoundingClientRect();
      pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      const intersects = raycaster.intersectObjects([...markerObjects.values()], false);
      if (intersects.length > 0) {{
        const markerName = intersects[0].object.userData.markerName;
        if (markerName) {{
          setSelectedMarker(markerName);
        }}
      }}
    }});

    document.getElementById("landmark-mode").addEventListener("click", () => setMode("landmarks"));
    document.getElementById("pose-mode").addEventListener("click", () => setMode("pose"));
    document.getElementById("center-view").addEventListener("click", () => {{
      frameScene();
      setStatus("View centered.");
    }});
    document.getElementById("reset-edits").addEventListener("click", () => {{
      poseEdits = {{}};
      markerEdits = {{}};
      setStatus("Local edits reset.");
      fetchPreview().catch((error) => setStatus(error.message));
    }});
    document.getElementById("export-patch").addEventListener("click", () => {{
      exportPatch().catch((error) => setStatus(error.message));
    }});
    markerXEl.addEventListener("change", updateMarkerEditFromInputs);
    markerYEl.addEventListener("change", updateMarkerEditFromInputs);
    markerZEl.addEventListener("change", updateMarkerEditFromInputs);

    window.addEventListener("resize", resize);
    buildMarkerList();
    buildCoordinateControls();
    buildScene();
    updateSelectedMarkerPanel();
    updateCoordinateInputs();
    resize();

    function animate() {{
      controls.update();
      renderer.render(scene, camera);
      requestAnimationFrame(animate);
    }}
    animate();
  </script>
</body>
</html>
"""


class _EditorApp:
    def __init__(self, *, model_path: Path, pose_path: Path, landmarks_path: Path, constraints_path: Path | None = None):
        self.model_path = model_path
        self.pose_path = pose_path
        self.landmarks_path = landmarks_path
        self.constraints_path = constraints_path or _guess_constraints_path(pose_path)

    def payload(self) -> dict[str, Any]:
        scene = build_editor_scene(
            model_path=self.model_path,
            pose_path=self.pose_path,
            landmarks_path=self.landmarks_path,
            constraints_path=self.constraints_path,
        )
        return {
            "scene": scene,
            "base_pose_file": str(self.pose_path),
            "base_landmarks_file": str(self.landmarks_path),
            "default_patch_path": str(default_patch_path(scene["pose"])),
        }

    def preview(self, request_data: dict[str, Any]) -> dict[str, Any]:
        scene = build_editor_scene(
            model_path=self.model_path,
            pose_path=self.pose_path,
            landmarks_path=self.landmarks_path,
            constraints_path=self.constraints_path,
            pose_edits=_coerce_pose_edits(request_data.get("pose_edits")),
            marker_edits=_coerce_marker_edits(request_data.get("marker_edits")),
        )
        return {"scene": scene}

    def export_patch(self, request_data: dict[str, Any]) -> dict[str, Any]:
        payload = self.payload()
        scene = payload["scene"]
        patch = build_editor_patch(
            pose_name=scene["pose"],
            base_pose_file=self.pose_path,
            base_landmarks_file=self.landmarks_path,
            pose_edits=_coerce_pose_edits(request_data.get("pose_edits")),
            marker_edits=_coerce_marker_edits(request_data.get("marker_edits")),
        )
        output_path = Path(str(request_data.get("output_path") or payload["default_patch_path"]))
        if not output_path.is_absolute():
            output_path = config.REPO_ROOT / output_path
        write_editor_patch(output_path, patch)
        return {"output_path": str(output_path)}


def _json_response(handler: BaseHTTPRequestHandler, status: int, payload: dict[str, Any]) -> None:
    body = json.dumps(payload).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _text_response(handler: BaseHTTPRequestHandler, status: int, body: str, content_type: str) -> None:
    data = body.encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Content-Length", str(len(data)))
    handler.end_headers()
    handler.wfile.write(data)


def _serve_vendor_file(handler: BaseHTTPRequestHandler, raw_path: str) -> None:
    relative_path = Path(raw_path.lstrip("/"))
    vendor_root = (config.REPO_ROOT / "vendor").resolve()
    full_path = (config.REPO_ROOT / relative_path).resolve()
    try:
        full_path.relative_to(vendor_root)
    except ValueError as exc:
        raise UserFacingError("Only vendored editor assets can be served.") from exc
    if not full_path.exists() or not full_path.is_file():
        raise UserFacingError(f"Missing static asset `{relative_path}`.")
    content_type = "application/javascript; charset=utf-8"
    if full_path.suffix == ".css":
        content_type = "text/css; charset=utf-8"
    elif full_path.suffix == ".html":
        content_type = "text/html; charset=utf-8"
    handler.send_response(HTTPStatus.OK)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Content-Length", str(full_path.stat().st_size))
    handler.end_headers()
    handler.wfile.write(full_path.read_bytes())


def serve_pose_editor(
    *,
    model_path: Path,
    pose_path: Path,
    landmarks_path: Path,
    constraints_path: Path | None = None,
    host: str = "127.0.0.1",
    port: int = 8765,
    open_browser: bool = True,
) -> str:
    app = _EditorApp(
        model_path=model_path,
        pose_path=pose_path,
        landmarks_path=landmarks_path,
        constraints_path=constraints_path,
    )

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            try:
                if parsed.path == "/":
                    _text_response(self, HTTPStatus.OK, _editor_html(app.payload()), "text/html; charset=utf-8")
                    return
                if parsed.path == "/api/scene":
                    _json_response(self, HTTPStatus.OK, app.payload())
                    return
                if parsed.path.startswith("/vendor/"):
                    _serve_vendor_file(self, parsed.path)
                    return
                if parsed.path == "/favicon.ico":
                    self.send_response(HTTPStatus.NO_CONTENT)
                    self.end_headers()
                    return
                _json_response(self, HTTPStatus.NOT_FOUND, {"error": "Not found"})
            except UserFacingError as exc:
                _json_response(self, HTTPStatus.BAD_REQUEST, {"error": str(exc)})

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            content_length = int(self.headers.get("Content-Length", "0"))
            raw_body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                request_data = json.loads(raw_body.decode("utf-8") or "{}")
                if parsed.path == "/api/preview":
                    _json_response(self, HTTPStatus.OK, app.preview(request_data))
                    return
                if parsed.path == "/api/export-patch":
                    _json_response(self, HTTPStatus.OK, app.export_patch(request_data))
                    return
                _json_response(self, HTTPStatus.NOT_FOUND, {"error": "Not found"})
            except UserFacingError as exc:
                _json_response(self, HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            except json.JSONDecodeError:
                _json_response(self, HTTPStatus.BAD_REQUEST, {"error": "Invalid JSON body."})

        def log_message(self, format: str, *args: Any) -> None:
            del format, args

    httpd = ThreadingHTTPServer((host, port), Handler)
    bound_host, bound_port = httpd.server_address[:2]
    url = f"http://{bound_host}:{bound_port}/"
    if open_browser:
        threading.Thread(target=lambda: webbrowser.open(url), daemon=True).start()
    try:
        print(f"Pose editor running at {url}")
        print("Press Ctrl+C to stop.")
        httpd.serve_forever()
    finally:
        httpd.server_close()
    return url
