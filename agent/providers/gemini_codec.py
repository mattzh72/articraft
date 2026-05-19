from __future__ import annotations

import base64
import json
import mimetypes
import uuid
from pathlib import Path
from typing import Any, Optional, Union


def convert_message(message: dict) -> Any | None:
    from google.genai import types  # type: ignore
    from google.genai.types import Content, FunctionCall, Part  # type: ignore

    role = message.get("role", "user")
    if role == "assistant":
        role = "model"

    google_content = (
        message.get("extra_content", {}).get("google", {}).get("content")
        if isinstance(message, dict)
        else None
    )
    if role == "model" and isinstance(google_content, dict):
        return deserialize_content(google_content)

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
                if bool(item.get("thought", False)):
                    continue
                text = item.get("text")
                if isinstance(text, str) and text:
                    part_kwargs: dict[str, Any] = {
                        "text": text,
                        "thought_signature": decode_signature(signature),
                    }
                    if "thought" in item:
                        part_kwargs["thought"] = bool(item.get("thought", False))
                    try:
                        parts.append(Part(**part_kwargs))
                    except TypeError:
                        part_kwargs.pop("thought", None)
                        parts.append(Part(**part_kwargs))
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
                        thought_signature=decode_signature(signature),
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
                    tool_call.get("extra_content", {}).get("google", {}).get("thought_signature")
                )
            parts.append(
                Part(
                    function_call=FunctionCall(
                        name=function.get("name", ""),
                        args=args,
                    ),
                    thought_signature=decode_signature(signature),
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
            signature = message.get("extra_content", {}).get("google", {}).get("thought_signature")
        parts.append(
            Part(
                function_response=types.FunctionResponse(
                    name=message.get("name") or message.get("tool_call_id", ""),
                    response=response_body,
                ),
                thought_signature=decode_signature(signature),
            )
        )
        role = "user"

    else:
        content = message.get("content")
        if isinstance(content, list):
            parts.extend(convert_user_content_parts(content))
        elif content:
            parts.append(Part(text=content))

    if not parts and role == "model":
        return None

    if not parts:
        raise ValueError("Message must contain content or tool calls")

    return Content(role=role, parts=parts)


def convert_user_content_parts(content: list[dict[str, Any]]) -> list[Any]:
    from google.genai.types import Part  # type: ignore

    parts: list[Part] = []
    for item in content:
        if not isinstance(item, dict):
            continue
        item_type = item.get("type")
        if item_type == "input_image":
            parts.append(convert_input_image_part(item))
            continue
        if item_type in {"input_text", "text"}:
            text = item.get("text")
            if isinstance(text, str) and text:
                parts.append(Part(text=text))

    return parts


def convert_input_image_part(item: dict[str, Any]) -> Any:
    from google.genai.types import Part  # type: ignore

    if isinstance(item.get("image_path"), str) and item["image_path"]:
        image_path = Path(item["image_path"]).expanduser().resolve()
        mime_type, _ = mimetypes.guess_type(image_path.name)
        if not mime_type:
            raise ValueError(f"Could not determine MIME type for image: {image_path}")
        return Part.from_bytes(data=image_path.read_bytes(), mime_type=mime_type)

    raise ValueError("Gemini input_image parts currently require image_path")


def extract_usage(response: Any) -> Optional[dict[str, int]]:
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


def convert_response(response: Any) -> dict:
    text = ""
    thought = ""
    tool_calls: list[dict] = []
    usage = extract_usage(response)

    candidates = getattr(response, "candidates", None) or []
    if not candidates:
        result = {"content": "", "tool_calls": []}
        if usage:
            result["usage"] = usage
        return result

    candidate = candidates[0]
    content = getattr(candidate, "content", None)
    parts = list_content_parts(content)

    for part in parts:
        if getattr(part, "text", None):
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
            encoded_signature = encode_signature(raw_signature)
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

    result = {"content": text, "tool_calls": tool_calls}
    if thought:
        result["thought_summary"] = thought
    if content is not None:
        result["extra_content"] = {"google": {"content": serialize_content(content)}}
    if usage:
        result["usage"] = usage
    return result


def history_content_from_response(response: Any) -> Any | None:
    candidates = getattr(response, "candidates", None) or []
    if not candidates:
        return None

    candidate = candidates[0]
    content = getattr(candidate, "content", None)
    if content is not None and list_content_parts(content):
        return content

    if content is not None or getattr(candidate, "function_calls", None):
        synthesized = convert_response(response)
        assistant_message = {"role": "assistant"}
        extra_content = synthesized.get("extra_content")
        if isinstance(extra_content, dict):
            assistant_message["extra_content"] = extra_content
        if synthesized.get("content"):
            assistant_message["content"] = synthesized["content"]
        if synthesized.get("tool_calls"):
            assistant_message["tool_calls"] = synthesized["tool_calls"]
        return convert_message(assistant_message)
    return None


def list_content_parts(content: Any) -> list[Any]:
    raw_parts = getattr(content, "parts", None)
    if raw_parts is None:
        return []
    if isinstance(raw_parts, list):
        return raw_parts
    try:
        return list(raw_parts)
    except TypeError:
        return [raw_parts]


def encode_signature(signature: Optional[Union[bytes, str]]) -> Optional[str]:
    if signature is None:
        return None
    if isinstance(signature, bytes):
        return base64.b64encode(signature).decode("utf-8")
    return signature


def decode_signature(signature: Optional[Union[str, bytes]]) -> Optional[bytes]:
    if signature is None:
        return None
    if isinstance(signature, bytes):
        return signature
    try:
        return base64.b64decode(signature)
    except Exception:
        return signature.encode("utf-8")


def content_for_compaction_prompt(content: Any) -> dict[str, Any]:
    role = getattr(content, "role", None) or "user"
    if role == "model":
        role = "assistant"
    payload: dict[str, Any] = {"role": role}
    rendered_parts: list[str] = []
    tool_calls: list[dict[str, Any]] = []

    for part in getattr(content, "parts", None) or []:
        part_text = getattr(part, "text", None)
        if isinstance(part_text, str) and part_text.strip():
            if not bool(getattr(part, "thought", False)):
                rendered_parts.append(part_text)

        function_call = getattr(part, "function_call", None)
        if function_call is not None:
            tool_calls.append(
                {
                    "id": getattr(function_call, "id", None),
                    "name": getattr(function_call, "name", None),
                    "arguments": getattr(function_call, "args", None)
                    or getattr(function_call, "arguments", None),
                }
            )

        function_response = getattr(part, "function_response", None)
        if function_response is not None:
            payload["role"] = "tool"
            payload["name"] = getattr(function_response, "name", None)
            response_body = getattr(function_response, "response", None)
            payload["content"] = json.dumps(response_body, ensure_ascii=False, default=str)

        inline_data = getattr(part, "inline_data", None)
        if inline_data is not None:
            mime_type = getattr(inline_data, "mime_type", None) or "<image>"
            rendered_parts.append(f"[image] {mime_type}")

    if rendered_parts and "content" not in payload:
        payload["content"] = "\n".join(rendered_parts)
    if tool_calls:
        payload["tool_calls"] = tool_calls
    return payload


def serialize_content(content: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {"role": getattr(content, "role", "model")}
    parts_payload: list[dict[str, Any]] = []
    for part in getattr(content, "parts", None) or []:
        serialized = serialize_part(part)
        if serialized is not None:
            parts_payload.append(serialized)
    payload["parts"] = parts_payload
    return payload


def serialize_part(part: Any) -> dict[str, Any] | None:
    text = getattr(part, "text", None)
    if isinstance(text, str):
        return {
            "type": "text",
            "text": text,
            "thought": bool(getattr(part, "thought", False)),
            "thought_signature": encode_signature(getattr(part, "thought_signature", None)),
        }

    function_call = getattr(part, "function_call", None)
    if function_call is not None:
        args = getattr(function_call, "args", None)
        if args is None:
            args = getattr(function_call, "arguments", None)
        raw_signature = getattr(part, "thought_signature", None) or getattr(
            function_call, "thought_signature", None
        )
        return {
            "type": "function_call",
            "id": getattr(function_call, "id", None),
            "name": getattr(function_call, "name", ""),
            "arguments": args,
            "thought_signature": encode_signature(raw_signature),
        }

    return None


def deserialize_content(payload: dict[str, Any]) -> Any:
    from google.genai.types import Content  # type: ignore

    role = payload.get("role", "model")
    parts = [deserialize_part(part) for part in payload.get("parts", []) if isinstance(part, dict)]
    return Content(role=role, parts=parts)


def deserialize_part(payload: dict[str, Any]) -> Any:
    from google.genai import types  # type: ignore
    from google.genai.types import FunctionCall, Part  # type: ignore

    part_type = payload.get("type")
    signature = decode_signature(payload.get("thought_signature"))
    if part_type == "text":
        part_kwargs: dict[str, Any] = {
            "text": payload.get("text", ""),
            "thought_signature": signature,
        }
        if "thought" in payload:
            part_kwargs["thought"] = bool(payload.get("thought", False))
        try:
            return Part(**part_kwargs)
        except TypeError:
            part_kwargs.pop("thought", None)
            return Part(**part_kwargs)
    if part_type == "function_call":
        args = payload.get("arguments", {})
        if isinstance(args, str):
            try:
                args = json.loads(args)
            except json.JSONDecodeError:
                args = {"_raw": args}
        call_kwargs = {"name": payload.get("name", ""), "args": args}
        if payload.get("id") is not None:
            call_kwargs["id"] = payload.get("id")
        try:
            function_call = FunctionCall(**call_kwargs)
        except TypeError:
            call_kwargs.pop("id", None)
            function_call = FunctionCall(**call_kwargs)
        return Part(function_call=function_call, thought_signature=signature)
    if part_type == "function_response":
        return Part(
            function_response=types.FunctionResponse(
                name=payload.get("name", ""),
                response=payload.get("response"),
            ),
            thought_signature=signature,
        )
    raise ValueError(f"Unsupported Gemini content part type: {part_type}")


def content_from_text_message(message: dict[str, Any]) -> Any:
    from google.genai.types import Content, Part  # type: ignore

    return Content(role=message.get("role", "user"), parts=[Part(text=message.get("content", ""))])


def is_standard_user_content(content: Any) -> bool:
    if getattr(content, "role", None) != "user":
        return False
    parts = getattr(content, "parts", None) or []
    if not parts:
        return False
    for part in parts:
        if getattr(part, "function_response", None) is not None:
            continue
        return True
    return False


def parse_json_response_text(response: Any) -> dict[str, Any]:
    if hasattr(response, "parsed") and isinstance(response.parsed, dict):
        return response.parsed
    text = ""
    for candidate in getattr(response, "candidates", None) or []:
        content = getattr(candidate, "content", None)
        for part in getattr(content, "parts", None) or []:
            part_text = getattr(part, "text", None)
            if isinstance(part_text, str) and part_text.strip():
                text += part_text
    if not text.strip():
        raise RuntimeError("Gemini compaction response did not include JSON text")
    parsed = json.loads(text)
    if not isinstance(parsed, dict):
        raise RuntimeError("Gemini compaction response JSON was not an object")
    return parsed


def render_compaction_summary_message(payload: dict[str, Any]) -> dict[str, Any]:
    sections = [
        ("Task Requirements", payload.get("task_requirements")),
        ("Constraints", payload.get("constraints")),
        ("Tool Findings", payload.get("tool_findings")),
        ("Compile State", payload.get("compile_state")),
        ("Next Steps", payload.get("next_steps")),
    ]
    lines = [
        "[System-generated compaction summary]",
        "This is preserved prior run context. It is not a new user request.",
    ]
    for title, raw_items in sections:
        items = [str(item).strip() for item in raw_items or [] if str(item).strip()]
        if not items:
            continue
        lines.append("")
        lines.append(f"{title}:")
        lines.extend(f"- {item}" for item in items)
    return {"role": "user", "content": "\n".join(lines).strip()}
