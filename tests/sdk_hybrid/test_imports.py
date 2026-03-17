from __future__ import annotations

import sys
from pathlib import Path


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import sdk_hybrid
from sdk_hybrid.v0.cadquery import mesh_from_cadquery


def main() -> None:
    assert hasattr(sdk_hybrid, "ArticulatedObject")
    assert hasattr(sdk_hybrid, "ValidationError")
    assert hasattr(sdk_hybrid, "TestContext")
    assert hasattr(sdk_hybrid, "UnsupportedPartFinding")
    assert callable(mesh_from_cadquery)


if __name__ == "__main__":
    main()
