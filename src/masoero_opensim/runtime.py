from __future__ import annotations

import importlib
from pathlib import Path


class UserFacingError(RuntimeError):
    pass


def ensure_parent_dir(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def import_dependency(module_name: str, install_hint: str):
    try:
        return importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        raise UserFacingError(install_hint) from exc
