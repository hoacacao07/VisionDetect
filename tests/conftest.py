"""Make `vision_detection` (src/) and `sim` importable for `python -m pytest tests`."""

import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
for path in (PROJECT / "src", PROJECT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
