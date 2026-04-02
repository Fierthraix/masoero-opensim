from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from masoero_opensim.commands import apply_editor_patch_main, run_entrypoint


if __name__ == "__main__":
    raise SystemExit(run_entrypoint(apply_editor_patch_main))
