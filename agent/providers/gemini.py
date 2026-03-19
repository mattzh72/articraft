"""
Gemini LLM wrapper with tool calling support.
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import mimetypes
import os
import random
import threading
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Union

try:
    from dotenv import load_dotenv  # type: ignore
except Exception:  # pragma: no cover

    def load_dotenv(*args: Any, **kwargs: Any) -> None:  # type: ignore
        return None


class GeminiLLM:
    """Minimal Gemini client for tool-calling workflows."""

    def __init__(
        self,
        model_id: str = "gemini-3.1-pro-preview",
        thinking_level: str = "high",
        *,
        dry_run: bool = False,
    ):
        self.model_id = model_id
        self.thinking_level = thinking_level
        self.include_thoughts = _env_bool("GEMINI_INCLUDE_THOUGHTS", True)
        # Hard timeout for a single Gemini request. This protects the harness from
        # hanging indefinitely when the underlying client stalls.
        # Set to 0/negative to disable.
        self.request_timeout_seconds = _env_float("GEMINI_REQUEST_TIMEOUT_SECONDS", 300.0)
        self.max_retries = _env_int("GEMINI_MAX_RETRIES", 6)
        self.retry_base_seconds = _env_float("GEMINI_RETRY_BASE_SECONDS", 0.5)
        self.retry_max_seconds = _env_float("GEMINI_RETRY_MAX_SECONDS", 20.0)
        if dry_run:
            self.client = None
            self._clients: list[Any] = []
            self._client_lock = threading.Lock()
            self._next_client_index = 0
        else:
            load_dotenv()
            config = gemini_client_config_from_env()
            from google import genai  # type: ignore

            if config.backend == "api_key":
                self._clients = [genai.Client(api_key=key) for key in config.api_keys]
            else:
                self._clients = [
                    genai.Client(vertexai=True, project=config.project, location=config.location)
                ]
            self._client_lock = threading.Lock()
            self._next_client_index = random.randrange(len(self._clients))
            # Keep the legacy attribute for compatibility with any external callers.
            self.client = self._clients[0]

    def build_request_preview(
        self,
        *,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        """
        Build the exact request arguments that would be sent to the Gemini client,
        without making any network calls.
        """
        from google.genai import types  # type: ignore

        tool_declarations = self._convert_tools(tools)
        contents = self._convert_messages(messages)

        config = types.GenerateContentConfig(
            system_instruction=system_prompt,
            tools=[types.Tool(function_declarations=tool_declarations)]
            if tool_declarations
            else None,
            temperature=0.7,
            thinking_config=self._build_thinking_config(),
        )

        return {
            "model": self.model_id,
            "contents": [c.model_dump(mode="json", exclude_none=True) for c in contents],
            "config": config.model_dump(mode="json", exclude_none=True),
        }

    async def generate_with_tools(
        self,
        system_prompt: str,
        messages: list[dict],
        tools: list[dict],
    ) -> dict:
        """Generate a response with optional tool calls."""
        from google.genai import types  # type: ignore

        tool_declarations = self._convert_tools(tools)
        contents = self._convert_messages(messages)

        config = types.GenerateContentConfig(
            system_instruction=system_prompt,
            tools=[types.Tool(function_declarations=tool_declarations)]
            if tool_declarations
            else None,
            temperature=0.7,
            thinking_config=self._build_thinking_config(),
        )

        async def _call() -> Any:
            client = self._next_client()
            coro = client.aio.models.generate_content(
                model=self.model_id,
                contents=contents,
                config=config,
            )
            if self.request_timeout_seconds and self.request_timeout_seconds > 0:
                return await asyncio.wait_for(coro, timeout=float(self.request_timeout_seconds))
            return await coro

        response = await _async_retry(
            _call,
            max_attempts=1 + max(0, self.max_retries),
            should_retry=_should_retry_gemini_exception,
            base_delay=self.retry_base_seconds,
            max_delay=self.retry_max_seconds,
            logger=_logger,
            context=f"gemini.generate_content(model={self.model_id})",
        )

        return self._convert_response(response)

    async def close(self) -> None:
        """Close any underlying resources (no-op for now)."""
        return None

    def _next_client(self) -> Any:
        if not self._clients:
            raise RuntimeError("Gemini client is not initialized")
        with self._client_lock:
            client = self._clients[self._next_client_index]
            self._next_client_index = (self._next_client_index + 1) % len(self._clients)
        return client

    def _build_thinking_config(self) -> Optional[Any]:
        try:
            from google.genai import types  # type: ignore
        except Exception:
            return None

        level = (self.thinking_level or "").strip().lower()
        if not level:
            return None

        model_id = (self.model_id or "").lower()
        if "2.5" in model_id:
            budget = self._thinking_budget_from_level(level, model_id)
            try:
                return types.ThinkingConfig(
                    thinking_budget=budget,
                    include_thoughts=self.include_thoughts,
                )
            except TypeError:
                return types.ThinkingConfig(thinking_budget=budget)

        normalized_level = self._normalize_thinking_level(level, model_id)
        try:
            return types.ThinkingConfig(
                thinking_level=normalized_level,
                include_thoughts=self.include_thoughts,
            )
        except TypeError:
            return types.ThinkingConfig(thinking_level=normalized_level)

    def _normalize_thinking_level(self, level: str, model_id: str) -> str:
        if level == "med":
            level = "medium"

        if level not in {"minimal", "low", "medium", "high"}:
            level = "high"

        if "flash" not in model_id:
            if level == "medium":
                level = "high"
            elif level == "minimal":
                level = "low"

        return level

    def _thinking_budget_from_level(self, level: str, model_id: str) -> int:
        min_budget, max_budget = self._thinking_budget_range(model_id)

        if level in {"low", "minimal"}:
            budget = min_budget
        elif level in {"med", "medium"}:
            budget = (min_budget + max_budget) // 2
        elif level in {"dynamic", "auto"}:
            budget = -1
        elif level in {"off", "none", "disable", "disabled", "0"}:
            budget = 0
        else:
            budget = max_budget

        if budget == 0 and min_budget > 0:
            budget = min_budget

        return budget

    def _thinking_budget_range(self, model_id: str) -> tuple[int, int]:
        if "flash-lite" in model_id or "flash_lite" in model_id:
            return 512, 24576
        if "flash" in model_id:
            return 0, 24576
        return 128, 32768

    def _convert_tools(self, tools: list[dict]) -> list[Any]:
        from google.genai.types import FunctionDeclaration  # type: ignore

        declarations: list[FunctionDeclaration] = []
        for tool in tools or []:
            func_def = tool.get("function") if isinstance(tool, dict) else None
            if not func_def:
                continue

            parameters = func_def.get("parameters")
            if parameters and isinstance(parameters, dict):
                parameters = {
                    key: value for key, value in parameters.items() if key != "additionalProperties"
                }

            declarations.append(
                FunctionDeclaration(
                    name=func_def.get("name", ""),
                    description=func_def.get("description", ""),
                    parameters=parameters,
                )
            )
        return declarations

    def _convert_messages(self, messages: list[dict]) -> list[Any]:
        return [self._convert_message(message) for message in messages]

    def _convert_message(self, message: dict) -> Any:
        from google.genai import types  # type: ignore
        from google.genai.types import Content, FunctionCall, Part  # type: ignore

        role = message.get("role", "user")
        if role == "assistant":
            role = "model"

        parts: list[Part] = []

        google_parts = (
            message.get("extra_content", {}).get("google", {}).get("parts")
            if isinstance(message, dict)
            else None
        )
        if role == "model" and isinstance(google_parts, list) and google_parts:
            for item in google_parts:
                if not isinstance(item, dict):
                    continue
                item_type = item.get("type")
                signature = item.get("thought_signature")
                if item_type == "text":
                    text = item.get("text")
                    if isinstance(text, str) and text:
                        parts.append(
                            Part(
                                text=text,
                                thought_signature=_decode_signature(signature),
                            )
                        )
                elif item_type == "function_call":
                    name = item.get("name", "")
                    args = item.get("arguments", {})
                    if isinstance(args, str):
                        try:
                            args = json.loads(args)
                        except json.JSONDecodeError:
                            args = {"_raw": args}
                    parts.append(
                        Part(
                            function_call=FunctionCall(name=name, args=args),
                            thought_signature=_decode_signature(signature),
                        )
                    )

        elif role == "model" and message.get("tool_calls"):
            if message.get("content"):
                parts.append(Part(text=message["content"]))
            for tool_call in message.get("tool_calls", []):
                function = tool_call.get("function", {})
                args = function.get("arguments", "{}")
                if isinstance(args, str):
                    try:
                        args = json.loads(args)
                    except json.JSONDecodeError:
                        args = {"_raw": args}
                signature = tool_call.get("thought_signature")
                if signature is None:
                    signature = (
                        tool_call.get("extra_content", {})
                        .get("google", {})
                        .get("thought_signature")
                    )
                parts.append(
                    Part(
                        function_call=FunctionCall(
                            name=function.get("name", ""),
                            args=args,
                        ),
                        thought_signature=_decode_signature(signature),
                    )
                )

        elif message.get("role") == "tool":
            response_body = message.get("content")
            if isinstance(response_body, str):
                try:
                    response_body = json.loads(response_body)
                except json.JSONDecodeError:
                    response_body = {"_raw": response_body}
            signature = message.get("thought_signature")
            if signature is None:
                signature = (
                    message.get("extra_content", {}).get("google", {}).get("thought_signature")
                )
            parts.append(
                Part(
                    function_response=types.FunctionResponse(
                        name=message.get("name") or message.get("tool_call_id", ""),
                        response=response_body,
                    ),
                    thought_signature=_decode_signature(signature),
                )
            )
            role = "user"

        else:
            content = message.get("content")
            if isinstance(content, list):
                parts.extend(self._convert_user_content_parts(content))
            elif content:
                parts.append(Part(text=content))

        if not parts:
            raise ValueError("Message must contain content or tool calls")

        return Content(role=role, parts=parts)

    def _convert_user_content_parts(self, content: list[dict[str, Any]]) -> list[Any]:
        from google.genai.types import Part  # type: ignore

        image_parts: list[Part] = []
        text_parts: list[Part] = []
        for item in content:
            if not isinstance(item, dict):
                continue
            item_type = item.get("type")
            if item_type == "input_image":
                image_parts.append(self._convert_input_image_part(item))
                continue
            if item_type in {"input_text", "text"}:
                text = item.get("text")
                if isinstance(text, str) and text:
                    text_parts.append(Part(text=text))

        return image_parts + text_parts

    def _convert_input_image_part(self, item: dict[str, Any]) -> Any:
        from google.genai.types import Part  # type: ignore

        if isinstance(item.get("image_path"), str) and item["image_path"]:
            image_path = Path(item["image_path"]).expanduser().resolve()
            mime_type, _ = mimetypes.guess_type(image_path.name)
            if not mime_type:
                raise ValueError(f"Could not determine MIME type for image: {image_path}")
            return Part.from_bytes(data=image_path.read_bytes(), mime_type=mime_type)

        raise ValueError("Gemini input_image parts currently require image_path")

    def _extract_usage(self, response: Any) -> Optional[dict[str, int]]:
        meta = getattr(response, "usage_metadata", None) or getattr(response, "usage", None)
        if meta is None:
            return None

        def get(name: str) -> Any:
            if isinstance(meta, dict):
                return meta.get(name)
            return getattr(meta, name, None)

        usage: dict[str, int] = {}
        mapping = {
            "prompt_tokens": "prompt_token_count",
            "candidates_tokens": "candidates_token_count",
            "total_tokens": "total_token_count",
            "cached_tokens": "cached_content_token_count",
        }
        for out_key, in_key in mapping.items():
            value = get(in_key)
            if isinstance(value, int):
                usage[out_key] = value

        return usage or None

    def _convert_response(self, response: Any) -> dict:
        text = ""
        thought = ""
        tool_calls: list[dict] = []
        google_parts: list[dict[str, Any]] = []
        usage = self._extract_usage(response)

        candidates = getattr(response, "candidates", None) or []
        if not candidates:
            result = {"content": "", "tool_calls": []}
            if usage:
                result["usage"] = usage
            return result

        candidate = candidates[0]
        content = getattr(candidate, "content", None)

        raw_parts = getattr(content, "parts", None)
        if raw_parts is None:
            parts: list[Any] = []
        elif isinstance(raw_parts, list):
            parts = raw_parts
        else:
            try:
                parts = list(raw_parts)
            except TypeError:
                parts = [raw_parts]

        for part in parts:
            if getattr(part, "text", None):
                encoded_signature = _encode_signature(getattr(part, "thought_signature", None))
                google_parts.append(
                    {
                        "type": "text",
                        "text": part.text,
                        "thought": bool(getattr(part, "thought", False)),
                        "thought_signature": encoded_signature,
                    }
                )
                if getattr(part, "thought", False):
                    thought += part.text
                else:
                    text += part.text

            function_call = getattr(part, "function_call", None)
            if function_call:
                args = getattr(function_call, "args", None)
                if args is None:
                    args = getattr(function_call, "arguments", None)
                raw_signature = getattr(part, "thought_signature", None) or getattr(
                    function_call, "thought_signature", None
                )
                encoded_signature = _encode_signature(raw_signature)
                google_parts.append(
                    {
                        "type": "function_call",
                        "id": getattr(function_call, "id", None),
                        "name": getattr(function_call, "name", ""),
                        "arguments": args,
                        "thought_signature": encoded_signature,
                    }
                )
                tool_calls.append(
                    {
                        "id": getattr(function_call, "id", None) or f"call_{uuid.uuid4().hex}",
                        "type": "function",
                        "thought_signature": encoded_signature,
                        "extra_content": {"google": {"thought_signature": encoded_signature}}
                        if encoded_signature
                        else {},
                        "function": {
                            "name": getattr(function_call, "name", ""),
                            "arguments": json.dumps(args)
                            if isinstance(args, (dict, list))
                            else str(args or ""),
                        },
                    }
                )

        # Some SDK responses expose function calls outside of content.parts.
        # Prefer parts-based extraction above, but fall back when needed.
        if not tool_calls:
            raw_calls = (
                getattr(candidate, "function_calls", None)
                or getattr(content, "function_calls", None)
                or getattr(response, "function_calls", None)
                or []
            )
            if raw_calls is None:
                raw_calls = []
            for call in raw_calls:
                name = getattr(call, "name", None)
                args = getattr(call, "args", None)
                if args is None:
                    args = getattr(call, "arguments", None)
                if name is None and isinstance(call, dict):
                    name = call.get("name")
                    args = call.get("args") if args is None else args
                    if args is None:
                        args = call.get("arguments")

                tool_calls.append(
                    {
                        "id": getattr(call, "id", None) or f"call_{uuid.uuid4().hex}",
                        "type": "function",
                        "function": {
                            "name": name or "",
                            "arguments": json.dumps(args)
                            if isinstance(args, (dict, list))
                            else str(args or ""),
                        },
                    }
                )

        # Avoid using the SDK's `.text` convenience accessor, which emits warnings
        # when non-text parts (e.g. function_call) are present. We intentionally
        # only extract from `content.parts` above.

        result = {"content": text, "tool_calls": tool_calls}
        if thought:
            result["thought_summary"] = thought
        if google_parts:
            result["extra_content"] = {"google": {"parts": google_parts}}
        if usage:
            result["usage"] = usage
        return result


def _encode_signature(signature: Optional[Union[bytes, str]]) -> Optional[str]:
    if signature is None:
        return None
    if isinstance(signature, bytes):
        return base64.b64encode(signature).decode("utf-8")
    return signature


def _decode_signature(signature: Optional[Union[str, bytes]]) -> Optional[bytes]:
    if signature is None:
        return None
    if isinstance(signature, bytes):
        return signature
    try:
        return base64.b64decode(signature)
    except Exception:
        return signature.encode("utf-8")


_logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class GeminiClientConfig:
    backend: str
    api_keys: tuple[str, ...] = ()
    project: Optional[str] = None
    location: str = "global"


def gemini_api_keys_from_env(env: Optional[dict[str, str]] = None) -> list[str]:
    values = os.environ if env is None else env

    def _split(raw: Optional[str]) -> list[str]:
        if not raw:
            return []
        return [token.strip() for token in raw.replace("\n", ",").split(",") if token.strip()]

    keys: list[str] = []
    keys.extend(_split(values.get("GEMINI_API_KEYS")))

    unique_keys: list[str] = []
    seen: set[str] = set()
    for key in keys:
        if key in seen:
            continue
        unique_keys.append(key)
        seen.add(key)

    return unique_keys


def gemini_client_config_from_env(env: Optional[dict[str, str]] = None) -> GeminiClientConfig:
    values = os.environ if env is None else env

    api_keys = gemini_api_keys_from_env(values)
    project = values.get("GOOGLE_CLOUD_PROJECT", "").strip() or _project_from_credentials_file(
        values.get("GOOGLE_APPLICATION_CREDENTIALS")
    )
    location = values.get("GOOGLE_CLOUD_LOCATION", "").strip() or "global"

    if project:
        return GeminiClientConfig(backend="vertex", project=project, location=location)

    if api_keys:
        return GeminiClientConfig(backend="api_key", api_keys=tuple(api_keys))

    raise ValueError(
        "Gemini configuration not found. Set GEMINI_API_KEYS for the Gemini API, "
        "or configure Vertex AI with GOOGLE_CLOUD_PROJECT. "
        "Authentication should come from GOOGLE_APPLICATION_CREDENTIALS. "
        "GOOGLE_CLOUD_LOCATION defaults to global."
    )


def _project_from_credentials_file(raw_path: Optional[str]) -> Optional[str]:
    if not raw_path:
        return None
    try:
        payload = json.loads(Path(raw_path).expanduser().read_text(encoding="utf-8"))
    except Exception:
        return None
    project = payload.get("project_id")
    return project.strip() if isinstance(project, str) and project.strip() else None


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw.strip())
    except Exception:
        return default


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw.strip())
    except Exception:
        return default


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    value = raw.strip().lower()
    if value in {"1", "true", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "no", "n", "off"}:
        return False
    return default


def _extract_http_status(exc: BaseException) -> Optional[int]:
    """
    Best-effort extraction of an HTTP status code from Gemini / google-genai errors.
    """

    for attr in ("status_code", "http_status", "status", "code"):
        value = getattr(exc, attr, None)
        try:
            if callable(value):
                value = value()
        except Exception:
            value = None

        if isinstance(value, int) and 100 <= value <= 599:
            return value

    response = getattr(exc, "response", None)
    status = getattr(response, "status_code", None) or getattr(response, "status", None)
    if isinstance(status, int) and 100 <= status <= 599:
        return status

    return None


def _should_retry_gemini_exception(exc: BaseException) -> bool:
    """
    Default retry policy for Gemini calls.

    - Retries transient 4xx/5xx responses (e.g. 429/503 overload).
    - Avoids retrying obvious auth/config errors (e.g. 401/403).
    - For unknown failures, prefers retry (caller still has max_attempts cap).
    """

    if isinstance(exc, (asyncio.TimeoutError, TimeoutError)):
        return True

    status = _extract_http_status(exc)
    if status is not None:
        if status in {408, 409, 425, 429, 500, 502, 503, 504}:
            return True
        if 400 <= status < 500:
            return False
        if status >= 500:
            return True

    message = str(exc).lower()
    if any(
        needle in message
        for needle in (
            "overloaded",
            "temporarily unavailable",
            "service unavailable",
            "timeout",
            "timed out",
            "deadline exceeded",
            "rate limit",
            "too many requests",
            "resource exhausted",
            "connection reset",
            "connection aborted",
            "connection refused",
            "internal error",
            "backend error",
        )
    ):
        return True

    if any(
        needle in message
        for needle in (
            "api key",
            "unauthorized",
            "permission denied",
            "forbidden",
            "invalid argument",
            "not found",
        )
    ):
        return False

    return True


def _format_retry_exception(exc: BaseException) -> str:
    """Build a readable retry error summary, even for blank exception messages."""
    status = _extract_http_status(exc)
    message = str(exc).strip()
    summary = type(exc).__name__
    if status is not None:
        summary += f" (HTTP {status})"
    if message:
        return f"{summary}: {message}"
    return f"{summary}: {repr(exc)}"


async def _async_retry(
    fn: Any,
    *,
    max_attempts: int,
    should_retry: Any,
    base_delay: float,
    max_delay: float,
    sleep_fn: Any = asyncio.sleep,
    rng: Any = random.random,
    logger: Optional[logging.Logger] = None,
    context: str = "gemini",
) -> Any:
    """
    Retry an async callable with exponential backoff + full jitter.
    """

    if max_attempts <= 0:
        raise ValueError("max_attempts must be >= 1")

    attempt = 0
    while True:
        try:
            return await fn()
        except Exception as exc:
            attempt += 1
            if attempt >= max_attempts or not bool(should_retry(exc)):
                raise

            cap = min(max_delay, base_delay * (2 ** (attempt - 1)))
            delay = max(0.0, float(rng()) * cap)

            if logger:
                logger.warning(
                    "%s failed (attempt %s/%s), retrying in %.2fs: %s",
                    context,
                    attempt,
                    max_attempts,
                    delay,
                    _format_retry_exception(exc),
                )

            await sleep_fn(delay)
