from __future__ import annotations

import sys
from pathlib import Path


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from cli.workbench import main as workbench_main
from storage.repo import StorageRepo


def main() -> None:
    assert callable(workbench_main)
    repo = StorageRepo(Path.cwd())
    assert repo.layout.runs_root.name == "runs"


if __name__ == "__main__":
    main()
