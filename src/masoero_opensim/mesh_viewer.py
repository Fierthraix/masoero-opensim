from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .opensim_utils import apply_pose_values, coordinate_defaults, load_model, marker_positions, read_storage_file
from .runtime import UserFacingError, ensure_parent_dir


@dataclass(frozen=True)
class MeshSpec:
    body_name: str
    mesh_file: str
    scale_factors: tuple[float, float, float]
    color: tuple[float, float, float]


def _as_color(color: str | None, body_name: str) -> tuple[float, float, float]:
    if color:
        parts = [float(value) for value in color.split()]
        if len(parts) == 3:
            return (parts[0], parts[1], parts[2])
    palette = {
        "pelvis": (0.84, 0.80, 0.72),
        "torso": (0.88, 0.84, 0.78),
        "femur_r": (0.86, 0.83, 0.77),
        "femur_l": (0.86, 0.83, 0.77),
        "tibia_r": (0.82, 0.80, 0.76),
        "tibia_l": (0.82, 0.80, 0.76),
        "humerus_r": (0.88, 0.85, 0.79),
        "humerus_l": (0.88, 0.85, 0.79),
    }
    return palette.get(body_name, (0.9, 0.88, 0.84))


def _read_mesh_specs(model_path: Path) -> list[MeshSpec]:
    root = ET.parse(model_path).getroot()
    specs: list[MeshSpec] = []
    for body in root.findall(".//BodySet/objects/Body"):
        body_name = body.attrib["name"]
        for mesh in body.findall("./attached_geometry/Mesh"):
            mesh_file = (mesh.findtext("mesh_file") or "").strip()
            if not mesh_file:
                continue
            scale_text = (mesh.findtext("scale_factors") or "1 1 1").strip()
            scale_values = tuple(float(value) for value in scale_text.split())
            color = _as_color(mesh.findtext("./Appearance/color"), body_name)
            specs.append(MeshSpec(body_name=body_name, mesh_file=mesh_file, scale_factors=scale_values, color=color))
    if not specs:
        raise UserFacingError(f"No attached mesh geometry found in `{model_path}`.")
    return specs


def _geometry_dir_for_model(model_path: Path) -> Path:
    candidate = model_path.parent / "Geometry"
    if candidate.exists():
        return candidate
    raise UserFacingError(f"Missing Geometry directory next to `{model_path}`.")


def _parse_vtp_mesh(path: Path) -> tuple[np.ndarray, np.ndarray]:
    root = ET.parse(path).getroot()
    piece = root.find("./PolyData/Piece")
    if piece is None:
        raise UserFacingError(f"`{path}` is not a supported VTP PolyData file.")

    points_node = piece.find("./Points/DataArray")
    connectivity_node = piece.find("./Polys/DataArray[@Name='connectivity']")
    offsets_node = piece.find("./Polys/DataArray[@Name='offsets']")
    if points_node is None or connectivity_node is None or offsets_node is None:
        raise UserFacingError(f"`{path}` is missing points or polygon connectivity.")

    points = np.fromstring(points_node.text or "", sep=" ", dtype=np.float32).reshape(-1, 3)
    connectivity = np.fromstring(connectivity_node.text or "", sep=" ", dtype=np.int32)
    offsets = np.fromstring(offsets_node.text or "", sep=" ", dtype=np.int32)
    if len(points) == 0 or len(offsets) == 0:
        raise UserFacingError(f"`{path}` does not contain renderable polygon data.")

    faces: list[list[int]] = []
    start = 0
    for offset in offsets:
        polygon = connectivity[start:offset].tolist()
        start = int(offset)
        if len(polygon) < 3:
            continue
        base = polygon[0]
        for idx in range(1, len(polygon) - 1):
            faces.append([base, polygon[idx], polygon[idx + 1]])
    return points, np.asarray(faces, dtype=np.int32)


def _pose_body_transforms(model_path: Path, pose_path: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    _, model, state = load_model(model_path)
    pose_values = coordinate_defaults(model)
    pose_values.update(read_storage_file(pose_path))
    apply_pose_values(model, state, pose_values)

    transforms: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    body_set = model.getBodySet()
    for index in range(body_set.getSize()):
        body = body_set.get(index)
        transform = body.getTransformInGround(state)
        rotation = np.array(
            [[float(transform.R().get(row, col)) for col in range(3)] for row in range(3)],
            dtype=np.float32,
        )
        translation = np.array([float(transform.p().get(i)) for i in range(3)], dtype=np.float32)
        transforms[body.getName()] = (rotation, translation)
    return transforms


def _posed_scene(model_path: Path, pose_path: Path) -> dict[str, object]:
    mesh_specs = _read_mesh_specs(model_path)
    geometry_dir = _geometry_dir_for_model(model_path)
    transforms = _pose_body_transforms(model_path, pose_path)
    mesh_cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    meshes: list[dict[str, object]] = []
    for spec in mesh_specs:
        mesh_path = geometry_dir / spec.mesh_file
        if spec.mesh_file not in mesh_cache:
            if not mesh_path.exists():
                raise UserFacingError(f"Missing mesh file `{mesh_path}`.")
            mesh_cache[spec.mesh_file] = _parse_vtp_mesh(mesh_path)
        vertices, faces = mesh_cache[spec.mesh_file]
        scaled = vertices * np.asarray(spec.scale_factors, dtype=np.float32)
        rotation, translation = transforms[spec.body_name]
        posed = (scaled @ rotation.T) + translation
        meshes.append(
            {
                "name": f"{spec.body_name}:{spec.mesh_file}",
                "body": spec.body_name,
                "color": list(spec.color),
                "positions": posed.reshape(-1).round(6).tolist(),
                "indices": faces.reshape(-1).tolist(),
            }
        )

    _, model, state = load_model(model_path)
    pose_values = coordinate_defaults(model)
    pose_values.update(read_storage_file(pose_path))
    apply_pose_values(model, state, pose_values)
    markers = [
        {"name": name, "position": [round(value, 6) for value in position]}
        for name, position in sorted(marker_positions(model, state).items())
    ]
    return {"pose": pose_path.stem, "model": model_path.name, "meshes": meshes, "markers": markers}


def _viewer_html(scene: dict[str, object]) -> str:
    scene_json = json.dumps(scene, separators=(",", ":"))
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{scene['pose']} viewer</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #e8decc;
      --panel: rgba(251, 248, 243, 0.9);
      --border: #c9ba9c;
      --text: #2d2417;
      --muted: #685a43;
      --accent: #c36a2d;
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
      grid-template-columns: 320px 1fr;
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
    h1 {{
      margin: 0 0 6px;
      font-size: 30px;
      line-height: 1.1;
    }}
    p {{
      margin: 0;
      color: var(--muted);
      line-height: 1.45;
    }}
    .stats {{
      margin-top: 18px;
      display: grid;
      gap: 10px;
    }}
    .stat {{
      padding: 12px 14px;
      border-radius: 16px;
      background: rgba(255,255,255,0.5);
      border: 1px solid rgba(201, 186, 156, 0.7);
    }}
    .stat strong {{
      display: block;
      font-size: 13px;
      color: var(--muted);
      margin-bottom: 4px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
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
    }}
    .swatch {{
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 999px;
      margin-right: 8px;
      background: var(--accent);
    }}
    @media (max-width: 920px) {{
      .shell {{ grid-template-columns: 1fr; }}
      .viewer {{ min-height: 72vh; }}
    }}
  </style>
  <script type="importmap">
  {{
    "imports": {{
      "three": "https://unpkg.com/three@0.164.1/build/three.module.js",
      "three/addons/": "https://unpkg.com/three@0.164.1/examples/jsm/"
    }}
  }}
  </script>
</head>
<body>
  <div class="shell">
    <aside class="panel">
      <h1>{scene['pose']}</h1>
      <p>Orbitable Rajagopal mesh viewer exported from the solved OpenSim pose.</p>
      <div class="stats">
        <div class="stat"><strong>Model</strong>{scene['model']}</div>
        <div class="stat"><strong>Meshes</strong>{len(scene['meshes'])}</div>
        <div class="stat"><strong>Markers</strong>{len(scene['markers'])}</div>
        <div class="stat"><strong>Controls</strong>Left drag orbit, right drag pan, wheel zoom.</div>
      </div>
    </aside>
    <section class="panel viewer">
      <div class="hud"><span class="swatch"></span>Masoero markers</div>
      <canvas id="viewer"></canvas>
    </section>
  </div>
  <script type="module">
    import * as THREE from "three";
    import {{ OrbitControls }} from "three/addons/controls/OrbitControls.js";

    const sceneData = {scene_json};
    const canvas = document.getElementById("viewer");
    const renderer = new THREE.WebGLRenderer({{ canvas, antialias: true, alpha: true }});
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf7f3ea);

    const camera = new THREE.PerspectiveCamera(35, 1, 0.01, 100);
    const controls = new OrbitControls(camera, canvas);
    controls.enableDamping = true;
    controls.target.set(0, 0.95, 0);

    scene.add(new THREE.HemisphereLight(0xfff7ee, 0x9b8e76, 2.2));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.8);
    keyLight.position.set(2.8, 3.6, 2.2);
    scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0xcfd8ff, 0.9);
    fillLight.position.set(-2.0, 1.4, -1.8);
    scene.add(fillLight);

    const group = new THREE.Group();
    scene.add(group);

    for (const meshData of sceneData.meshes) {{
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
      group.add(mesh);
    }}

    const markerGeometry = new THREE.SphereGeometry(0.011, 14, 14);
    const markerMaterial = new THREE.MeshStandardMaterial({{
      color: 0xc36a2d,
      emissive: 0x67351a,
      emissiveIntensity: 0.15,
      roughness: 0.48
    }});
    for (const marker of sceneData.markers) {{
      const dot = new THREE.Mesh(markerGeometry, markerMaterial);
      dot.position.set(...marker.position);
      group.add(dot);
    }}

    const bounds = new THREE.Box3().setFromObject(group);
    const center = bounds.getCenter(new THREE.Vector3());
    const size = bounds.getSize(new THREE.Vector3());
    group.position.sub(center);
    controls.target.set(0, size.y * 0.04, 0);

    function frameScene() {{
      const maxDim = Math.max(size.x, size.y, size.z);
      const fitHeightDistance = maxDim / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov * 0.5)));
      const fitWidthDistance = fitHeightDistance / Math.max(camera.aspect, 1e-3);
      const distance = 1.35 * Math.max(fitHeightDistance, fitWidthDistance);
      camera.near = Math.max(distance / 200, 0.01);
      camera.far = distance * 20;
      camera.position.set(distance * 0.42, size.y * 0.08, distance * 0.98);
      camera.lookAt(controls.target);
      camera.updateProjectionMatrix();
    }}

    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(Math.max(size.x, size.z, size.y) * 1.15 + 0.3, 80),
      new THREE.MeshStandardMaterial({{ color: 0xe7dcc8, roughness: 0.95, metalness: 0 }})
    );
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -size.y / 2 - 0.015;
    scene.add(floor);

    function resize() {{
      const width = canvas.clientWidth;
      const height = canvas.clientHeight;
      renderer.setSize(width, height, false);
      camera.aspect = width / Math.max(height, 1);
      frameScene();
    }}

    window.addEventListener("resize", resize);
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


def export_pose_viewer(model_path: Path, pose_path: Path, output_path: Path) -> None:
    scene = _posed_scene(model_path, pose_path)
    ensure_parent_dir(output_path)
    output_path.write_text(_viewer_html(scene), encoding="utf-8")
