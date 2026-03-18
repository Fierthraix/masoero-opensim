from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
EXTERNAL_DIR = REPO_ROOT / "external" / "rajagopal2016"
BASE_MODEL_PATH = EXTERNAL_DIR / "Rajagopal2016.osim"
MARKER_MODEL_PATH = REPO_ROOT / "models" / "masoero" / "rajagopal_masoero_markers.osim"
LANDMARKS_PATH = REPO_ROOT / "specs" / "landmarks.yaml"
GOOD_CONSTRAINTS_PATH = REPO_ROOT / "specs" / "constraints_good.yaml"
BAD_CONSTRAINTS_PATH = REPO_ROOT / "specs" / "constraints_bad.yaml"
POSES_DIR = REPO_ROOT / "poses"
REPORTS_DIR = REPO_ROOT / "reports"


def constraint_path_for_pose(pose_name: str) -> Path:
    if pose_name == "good":
        return GOOD_CONSTRAINTS_PATH
    if pose_name == "bad":
        return BAD_CONSTRAINTS_PATH
    raise ValueError(f"Unsupported pose name: {pose_name}")
