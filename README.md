# Masoero OpenSim

Reproducible tooling for building two static posture assets on top of the Rajagopal2016 OpenSim model:

- `pose_good`: upright ribcage and pelvis aligned to the Masoero unity plane heuristic
- `pose_bad`: shortened-torso posture with worse unity-plane alignment

The repository is set up to be driven entirely from `uv` and YAML specs. The OpenSim model and geometry are external inputs and are not committed here.

## Layout

- `external/rajagopal2016/`: place `Rajagopal2016.osim` and its `Geometry/` directory here
- `specs/`: editable landmark definitions and posture constraints
- `scripts/`: thin CLI entrypoints
- `src/masoero_opensim/`: shared implementation
- `models/`, `poses/`, `reports/`, `renders/`: generated outputs

## OpenSim Runtime

The official Linux OpenSim Python package is distributed through the `opensim-org` Conda channel. In this repo, the working pattern is:

1. Create a dedicated Conda-compatible environment containing `opensim` and the matching scientific stack.
2. Run every Python entrypoint through `uv`, pointing at that interpreter:
   - `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/00_verify_opensim.py`

This keeps the user-facing workflow on `uv` while still using the officially packaged OpenSim bindings.

## Quick Start

1. Put the Rajagopal model at `external/rajagopal2016/Rajagopal2016.osim`.
2. Provision an OpenSim-capable interpreter at `.opensim-env/bin/python`.
3. Run `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/00_verify_opensim.py`.
4. Run `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/01_add_markers.py`.
4. Solve poses:
   - `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/02_solve_pose.py good`
   - `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/02_solve_pose.py bad`
5. Export metrics:
   - `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/03_export_metrics.py poses/pose_good.sto`
   - `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/03_export_metrics.py poses/pose_bad.sto`
6. Export transforms if needed:
   - `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/04_export_transforms_json.py poses/pose_good.sto`
7. Render screenshots:
   - `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/05_render_pose.py poses/pose_good.sto`
   - `UV_CACHE_DIR=/tmp/uv-cache uv run --no-project --python .opensim-env/bin/python python scripts/05_render_pose.py poses/pose_bad.sto`

## Notes

- The scripts fail fast with explicit messages when OpenSim is unavailable or the Rajagopal assets are missing.
- Marker offsets in `specs/landmarks.yaml` are conservative starting values and will need tuning in OpenSim.
- Constraint YAML is data-driven so posture targets can be iterated without changing code.
- `scripts/05_render_pose.py` produces deterministic PNG/SVG renders from solved OpenSim body transforms and Masoero markers. This avoids depending on the Simbody GUI path, which is unstable under the current Wayland/Xwayland setup.
