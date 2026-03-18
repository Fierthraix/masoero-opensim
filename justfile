uv_cache_dir := env_var_or_default("UV_CACHE_DIR", "/tmp/uv-cache")
opensim_python := ".opensim-env/bin/python"
uv_py := "UV_CACHE_DIR=" + uv_cache_dir + " uv run --no-project --python " + opensim_python + " python"
uv_plain := "UV_CACHE_DIR=" + uv_cache_dir + " uv run --no-project python"

default:
  @just --list

list:
  @just --list

compile:
  {{uv_py}} -m compileall src scripts

verify:
  {{uv_py}} scripts/00_verify_opensim.py

markers:
  {{uv_py}} scripts/01_add_markers.py

solve pose:
  {{uv_py}} scripts/02_solve_pose.py {{pose}}

metrics pose:
  {{uv_py}} scripts/03_export_metrics.py poses/pose_{{pose}}.sto

transforms pose:
  {{uv_py}} scripts/04_export_transforms_json.py poses/pose_{{pose}}.sto

render pose:
  {{uv_py}} scripts/05_render_pose.py poses/pose_{{pose}}.sto

viewer pose:
  {{uv_py}} scripts/06_export_pose_viewer.py poses/pose_{{pose}}.sto

serve port="8000":
  {{uv_plain}} -m http.server {{port}}

good:
  {{uv_py}} scripts/02_solve_pose.py good
  {{uv_py}} scripts/03_export_metrics.py poses/pose_good.sto
  {{uv_py}} scripts/05_render_pose.py poses/pose_good.sto
  {{uv_py}} scripts/06_export_pose_viewer.py poses/pose_good.sto

bad:
  {{uv_py}} scripts/02_solve_pose.py bad
  {{uv_py}} scripts/03_export_metrics.py poses/pose_bad.sto
  {{uv_py}} scripts/05_render_pose.py poses/pose_bad.sto
  {{uv_py}} scripts/06_export_pose_viewer.py poses/pose_bad.sto

all:
  {{uv_py}} scripts/00_verify_opensim.py
  {{uv_py}} scripts/01_add_markers.py
  {{uv_py}} scripts/02_solve_pose.py good
  {{uv_py}} scripts/02_solve_pose.py bad
  {{uv_py}} scripts/03_export_metrics.py poses/pose_good.sto
  {{uv_py}} scripts/03_export_metrics.py poses/pose_bad.sto
  {{uv_py}} scripts/05_render_pose.py poses/pose_good.sto
  {{uv_py}} scripts/05_render_pose.py poses/pose_bad.sto
  {{uv_py}} scripts/06_export_pose_viewer.py poses/pose_good.sto
  {{uv_py}} scripts/06_export_pose_viewer.py poses/pose_bad.sto
