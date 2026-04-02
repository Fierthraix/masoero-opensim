from __future__ import annotations

from pathlib import Path
from typing import Any

from .runtime import UserFacingError, import_dependency


def read_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise UserFacingError(f"Spec file does not exist: {path}")
    yaml = import_dependency(
        "yaml",
        "PyYAML is required. Run `UV_CACHE_DIR=/tmp/uv-cache uv sync` first.",
    )
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise UserFacingError(f"Expected a mapping in {path}")
    return data


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    yaml = import_dependency(
        "yaml",
        "PyYAML is required. Run `UV_CACHE_DIR=/tmp/uv-cache uv sync` first.",
    )
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
