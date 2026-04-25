from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def repo_path(*parts: str) -> Path:
    return REPO_ROOT.joinpath(*parts)


def read_repo_file(*parts: str) -> str:
    return repo_path(*parts).read_text()
