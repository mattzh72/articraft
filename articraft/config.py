from __future__ import annotations

import logging
import os
from collections.abc import Mapping, MutableMapping
from pathlib import Path

from dotenv import dotenv_values

logger = logging.getLogger(__name__)

ARTICRAFT_MODEL_ENV_VAR = "ARTICRAFT_MODEL"
ARTICRAFT_THINKING_LEVEL_ENV_VAR = "ARTICRAFT_THINKING_LEVEL"
ARTICRAFT_MAX_COST_USD_ENV_VAR = "ARTICRAFT_MAX_COST_USD"

DEFAULT_GENERATION_MODEL = "gpt-5.5-2026-04-23"
DEFAULT_THINKING_LEVEL = "high"

SUPPORTED_ENV_KEYS = (
    "OPENAI_API_KEYS",
    "OPENAI_API_KEY",
    "OPENROUTER_API_KEYS",
    "OPENROUTER_API_KEY",
    "ANTHROPIC_API_KEYS",
    "ANTHROPIC_API_KEY",
    "GEMINI_API_KEYS",
    ARTICRAFT_MODEL_ENV_VAR,
    ARTICRAFT_THINKING_LEVEL_ENV_VAR,
    ARTICRAFT_MAX_COST_USD_ENV_VAR,
    "GOOGLE_CLOUD_PROJECT",
    "GOOGLE_APPLICATION_CREDENTIALS",
    "GOOGLE_CLOUD_LOCATION",
    "DEEPSEEK_API_KEY",
)


def load_repo_env(
    repo_root: Path | None = None,
    *,
    environ: MutableMapping[str, str] | None = None,
) -> int:
    """Load non-empty repo-local .env values without overriding non-empty shell values."""
    root = repo_root or Path.cwd()
    dotenv_path = root / ".env"
    if not dotenv_path.exists():
        return 0

    values = os.environ if environ is None else environ
    loaded_count = 0
    for key, value in dotenv_values(dotenv_path=dotenv_path).items():
        if value is None or not value.strip():
            continue
        current_value = values.get(key)
        if current_value is not None and current_value.strip():
            continue
        values[key] = value
        loaded_count += 1
    logger.debug(
        "Loaded %s non-empty Articraft environment value(s) from %s",
        loaded_count,
        dotenv_path,
    )
    return loaded_count


def default_model_from_env(environ: Mapping[str, str] | None = None) -> str:
    return _env_default(environ, ARTICRAFT_MODEL_ENV_VAR, DEFAULT_GENERATION_MODEL)


def default_thinking_level_from_env(environ: Mapping[str, str] | None = None) -> str:
    return _env_default(environ, ARTICRAFT_THINKING_LEVEL_ENV_VAR, DEFAULT_THINKING_LEVEL)


def bootstrap_env(
    repo_root: Path,
    *,
    environ: Mapping[str, str] | None = None,
) -> tuple[bool, list[str]]:
    values = os.environ if environ is None else environ
    env_path = repo_root / ".env"
    if env_path.exists():
        return False, []

    template_path = repo_root / ".env.example"
    template_lines = template_path.read_text(encoding="utf-8").splitlines()

    rendered_lines: list[str] = []
    seen_keys: set[str] = set()
    imported_keys: list[str] = []

    for line in template_lines:
        key, separator, default_value = line.partition("=")
        normalized_key = key.strip()
        if separator and normalized_key in SUPPORTED_ENV_KEYS:
            seen_keys.add(normalized_key)
            env_value = values.get(normalized_key)
            if env_value:
                rendered_lines.append(f"{normalized_key}={_quote_env_value(env_value)}")
                imported_keys.append(normalized_key)
            else:
                rendered_lines.append(f"{normalized_key}={default_value}")
            continue
        rendered_lines.append(line)

    for key in SUPPORTED_ENV_KEYS:
        env_value = values.get(key)
        if key in seen_keys or not env_value:
            continue
        rendered_lines.append(f"{key}={_quote_env_value(env_value)}")
        imported_keys.append(key)

    env_path.write_text("\n".join(rendered_lines) + "\n", encoding="utf-8")
    return True, imported_keys


def _env_default(environ: Mapping[str, str] | None, name: str, fallback: str) -> str:
    values = os.environ if environ is None else environ
    value = values.get(name)
    if value and value.strip():
        return value.strip()
    return fallback


def _quote_env_value(value: str) -> str:
    if value == "":
        return value
    safe_chars = set("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789@%_+=:,./-")
    if all(char in safe_chars for char in value):
        return value
    return "'" + value.replace("'", "'\"'\"'") + "'"
