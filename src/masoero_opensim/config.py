from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
EXTERNAL_DIR = REPO_ROOT / "external" / "rajagopal2016"
MODELS_DIR = REPO_ROOT / "models" / "masoero"
SPECS_DIR = REPO_ROOT / "specs"
EDITS_DIR = REPO_ROOT / "edits"
BASE_MODEL_PATH = EXTERNAL_DIR / "Rajagopal2016.osim"
MARKER_MODEL_PATH = MODELS_DIR / "rajagopal_masoero_markers.osim"
LANDMARKS_PATH = SPECS_DIR / "landmarks.yaml"
GOOD_CONSTRAINTS_PATH = SPECS_DIR / "constraints_good.yaml"
BAD_CONSTRAINTS_PATH = SPECS_DIR / "constraints_bad.yaml"
POSES_DIR = REPO_ROOT / "poses"
REPORTS_DIR = REPO_ROOT / "reports"
RENDERS_DIR = REPO_ROOT / "renders"


def constraint_path_for_pose(pose_name: str) -> Path:
    if pose_name == "good":
        return GOOD_CONSTRAINTS_PATH
    if pose_name == "bad":
        return BAD_CONSTRAINTS_PATH
    raise ValueError(f"Unsupported pose name: {pose_name}")
