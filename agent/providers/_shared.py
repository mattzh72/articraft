from __future__ import annotations

import asyncio
import json
import logging
import os
import random
from collections.abc import Awaitable, Callable
from typing import Any

TRANSIENT_HTTP_STATUS_CODES = {408, 409, 425, 429, 500, 502, 503, 504}
TRANSIENT_ERROR_MESSAGE_FRAGMENTS = (
    "timeout",
    "timed out",
    "connection error",
    "connection reset",
    "connection aborted",
    "server disconnected",
    "protocol error",
    "temporarily unavailable",
    "rate limit",
    "bad gateway",
    "service unavailable",
)


def env_keys_from_values(
    *,
    primary_name: str,
    pool_name: str,
    env: dict[str, str] | None = None,
) -> list[str]:
    values = os.environ if env is None else env

    def split(raw: str | None) -> list[str]:
        if not raw:
            return []
        return [token.strip() for token in raw.replace("\n", ",").split(",") if token.strip()]

    keys: list[str] = []
    primary_key = values.get(primary_name)
    if primary_key and primary_key.strip():
        keys.append(primary_key.strip())
    keys.extend(split(values.get(pool_name)))

    unique_keys: list[str] = []
    seen: set[str] = set()
    for key in keys:
        if key in seen:
            continue
        unique_keys.append(key)
        seen.add(key)
    return unique_keys


def random_env_key(
    *,
    primary_name: str,
    pool_name: str,
    env: dict[str, str] | None = None,
) -> str | None:
    keys = env_keys_from_values(primary_name=primary_name, pool_name=pool_name, env=env)
    return random.choice(keys) if keys else None


def env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw.strip())
    except Exception:
        return default


def env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw.strip().replace("_", ""))
    except Exception:
        return default


def get_value(obj: Any, field: str) -> Any:
    if obj is None:
        return None
    if isinstance(obj, dict):
        return obj.get(field)
    return getattr(obj, field, None)


def serialize_json_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, list):
        return [serialize_json_value(item) for item in value]
    if isinstance(value, tuple):
        return [serialize_json_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): serialize_json_value(child) for key, child in value.items()}
    dump = getattr(value, "model_dump", None)
    if callable(dump):
        return dump(mode="json", exclude_none=True, warnings="none")
    dump_json = getattr(value, "model_dump_json", None)
    if callable(dump_json):
        return json.loads(dump_json(exclude_none=True))
    attrs = getattr(value, "__dict__", None)
    if isinstance(attrs, dict):
        return {
            str(key): serialize_json_value(child)
            for key, child in attrs.items()
            if not key.startswith("_")
        }
    return str(value)


def json_text_size(value: Any) -> int:
    if value is None:
        return 0
    try:
        return len(json.dumps(value, ensure_ascii=False, separators=(",", ":")))
    except Exception:
        return len(str(value))


def estimate_prompt_tokens(payload: dict[str, Any]) -> int:
    messages = payload.get("messages")
    tools = payload.get("tools")
    text_size = json_text_size(messages) + json_text_size(tools)
    structural_overhead = 128
    if isinstance(messages, list):
        structural_overhead += len(messages) * 16
    if isinstance(tools, list):
        structural_overhead += len(tools) * 128
    return max(1, (text_size + 2) // 3 + structural_overhead)


def extract_http_status(exc: BaseException) -> int | None:
    status = getattr(exc, "status_code", None) or getattr(exc, "status", None)
    if isinstance(status, int) and 100 <= status <= 599:
        return status
    response = getattr(exc, "response", None)
    if response is not None:
        status = getattr(response, "status_code", None) or getattr(response, "status", None)
        if isinstance(status, int) and 100 <= status <= 599:
            return status
    return None


def should_retry_transient_http_exception(exc: BaseException) -> bool:
    if isinstance(exc, (asyncio.TimeoutError, TimeoutError)):
        return True
    if isinstance(exc, json.JSONDecodeError):
        return True
    status = extract_http_status(exc)
    if status is not None:
        if status in TRANSIENT_HTTP_STATUS_CODES:
            return True
        if 400 <= status < 500:
            return False
        if status >= 500:
            return True
    message = str(exc).lower()
    return any(needle in message for needle in TRANSIENT_ERROR_MESSAGE_FRAGMENTS)


def format_retry_exception(exc: BaseException) -> str:
    status = extract_http_status(exc)
    message = str(exc).strip()
    summary = type(exc).__name__
    if status is not None:
        summary += f" (HTTP {status})"
    if message:
        return f"{summary}: {message}"
    return f"{summary}: {repr(exc)}"


async def async_retry(
    operation: Callable[[], Awaitable[Any]],
    *,
    max_attempts: int,
    should_retry: Callable[[BaseException], bool],
    base_delay: float,
    max_delay: float,
    logger: logging.Logger,
    context: str,
    sleep_fn: Callable[[float], Awaitable[Any]] = asyncio.sleep,
    rng: Callable[[], float] = random.random,
) -> Any:
    if max_attempts <= 0:
        raise ValueError("max_attempts must be >= 1")

    attempt = 0
    while True:
        try:
            return await operation()
        except Exception as exc:
            attempt += 1
            if attempt >= max_attempts or not should_retry(exc):
                raise

            cap = min(max_delay, base_delay * (2 ** (attempt - 1)))
            delay = max(0.0, float(rng()) * cap)
            logger.warning(
                "%s failed (attempt %s/%s), retrying in %.2fs: %s",
                context,
                attempt,
                max_attempts,
                delay,
                format_retry_exception(exc),
            )
            await sleep_fn(delay)


def create_openai_compatible_client(
    *,
    provider_label: str,
    api_key: str,
    base_url: str,
    timeout_seconds: float,
    extra_client_kwargs: dict[str, Any] | None = None,
) -> tuple[Any, bool]:
    client_kwargs: dict[str, Any] = {
        "api_key": api_key,
        "base_url": base_url,
        "max_retries": 0,
    }
    if extra_client_kwargs:
        client_kwargs.update(extra_client_kwargs)
    if timeout_seconds and timeout_seconds > 0:
        client_kwargs["timeout"] = float(timeout_seconds)

    try:
        from openai import AsyncOpenAI  # type: ignore

        return AsyncOpenAI(**client_kwargs), True
    except Exception:
        try:
            from openai import OpenAI  # type: ignore
        except Exception as exc:  # pragma: no cover
            raise RuntimeError(
                f"{provider_label} provider selected but the `openai` package is not installed. "
                "Install it (e.g. `uv add openai`), then try again."
            ) from exc

        return OpenAI(**client_kwargs), False
