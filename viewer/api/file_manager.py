from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def open_in_file_manager(target: Path) -> None:
    if sys.platform == "darwin":
        subprocess.Popen(["open", str(target)])
        return
    if sys.platform.startswith("win"):
        os.startfile(str(target))
        return
    if sys.platform.startswith("linux"):
        subprocess.Popen(["xdg-open", str(target)])
        return
    raise RuntimeError(f"Unsupported platform: {sys.platform}")
