from __future__ import annotations

import logging
import os
from pathlib import Path

from dotenv import load_dotenv

logger = logging.getLogger(__name__)

DEFAULT_MODEL_ENV_VAR = "ARTICRAFT_MODEL"
DEFAULT_THINKING_LEVEL_ENV_VAR = "ARTICRAFT_THINKING_LEVEL"
DEFAULT_MODEL = "gpt-5.5-2026-04-23"
DEFAULT_THINKING_LEVEL = "high"


def load_repo_env(repo_root: Path | None = None) -> None:
    """Load repo-local defaults before CLI argument defaults are resolved."""
    root = repo_root or Path.cwd()
    dotenv_path = root / ".env"
    if dotenv_path.exists():
        load_dotenv(dotenv_path=dotenv_path, override=True)
        logger.debug("Loaded Articraft environment defaults from %s", dotenv_path)


def default_model_from_env() -> str:
    return _env_default(DEFAULT_MODEL_ENV_VAR, DEFAULT_MODEL)


def default_thinking_level_from_env() -> str:
    return _env_default(DEFAULT_THINKING_LEVEL_ENV_VAR, DEFAULT_THINKING_LEVEL)


def _env_default(name: str, fallback: str) -> str:
    value = os.environ.get(name)
    if value and value.strip():
        return value.strip()
    return fallback
