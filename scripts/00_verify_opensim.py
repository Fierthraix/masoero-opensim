from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from masoero_opensim.commands import run_entrypoint, verify_main


if __name__ == "__main__":
    raise SystemExit(run_entrypoint(verify_main))
