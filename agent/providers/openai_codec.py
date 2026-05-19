from __future__ import annotations

import base64
import copy
import json
import mimetypes
import uuid
from pathlib import Path
from typing import Any, Optional


def make_json_schema_nullable(schema: dict[str, Any]) -> dict[str, Any]:
    normalized = copy.deepcopy(schema)
    schema_type = normalized.get("type")
    if isinstance(schema_type, str):
        if schema_type != "null":
            normalized["type"] = [schema_type, "null"]
        return normalized
    if isinstance(schema_type, list):
        if "null" not in schema_type:
            normalized["type"] = [*schema_type, "null"]
        return normalized

    any_of = normalized.get("anyOf")
    if isinstance(any_of, list) and not any(
        isinstance(option, dict) and option.get("type") == "null" for option in any_of
    ):
        normalized["anyOf"] = [*any_of, {"type": "null"}]
        return normalized

    one_of = normalized.get("oneOf")
    if isinstance(one_of, list) and not any(
        isinstance(option, dict) and option.get("type") == "null" for option in one_of
    ):
        normalized["oneOf"] = [*one_of, {"type": "null"}]
    return normalized


def normalize_tool_json_schema_for_responses(
    schema: dict[str, Any] | None,
    *,
    nullable: bool = False,
) -> dict[str, Any]:
    """Normalize a JSON Schema node to OpenAI Responses strict-mode requirements."""

    normalized: dict[str, Any] = copy.deepcopy(schema) if isinstance(schema, dict) else {}
    properties = normalized.get("properties")
    if isinstance(properties, dict):
        original_required = normalized.get("required")
        required_names = set(original_required) if isinstance(original_required, list) else set()
        normalized_properties: dict[str, Any] = {}
        for name, child_schema in properties.items():
            normalized_properties[name] = normalize_tool_json_schema_for_responses(
                child_schema,
                nullable=name not in required_names,
            )
        normalized["properties"] = normalized_properties
        normalized["required"] = list(normalized_properties.keys())
        normalized["additionalProperties"] = False

    items = normalized.get("items")
    if isinstance(items, dict):
        normalized["items"] = normalize_tool_json_schema_for_responses(items)
    elif isinstance(items, list):
        normalized["items"] = [
            normalize_tool_json_schema_for_responses(item) if isinstance(item, dict) else item
            for item in items
        ]

    if nullable:
        normalized = make_json_schema_nullable(normalized)
    return normalized


def normalize_function_parameters_for_responses(
    parameters: dict[str, Any] | None,
    *,
    required_override: list[str] | None = None,
) -> dict[str, Any]:
    normalized = normalize_tool_json_schema_for_responses(parameters or {"type": "object"})
    properties = normalized.get("properties")
    if not isinstance(properties, dict):
        properties = {}
        normalized["properties"] = properties
    normalized["type"] = "object"
    normalized["required"] = (
        list(required_override) if required_override is not None else list(properties.keys())
    )
    normalized["additionalProperties"] = False
    return normalized


def convert_message_content(content: Any) -> list[dict[str, Any]]:
    if isinstance(content, str):
        if not content.strip():
            return []
        return [{"type": "input_text", "text": content}]

    if not isinstance(content, list):
        return []

    parts: list[dict[str, Any]] = []
    for part in content:
        if not isinstance(part, dict):
            continue

        part_type = part.get("type")
        if part_type in {"input_text", "text"}:
            text = part.get("text")
            if isinstance(text, str) and text.strip():
                parts.append({"type": "input_text", "text": text})
            continue

        if part_type == "input_image":
            image_part = convert_image_part(part)
            if image_part:
                parts.append(image_part)

    return parts


def convert_image_part(part: dict[str, Any]) -> Optional[dict[str, Any]]:
    detail = part.get("detail")

    if isinstance(part.get("image_url"), str) and part["image_url"]:
        item: dict[str, Any] = {
            "type": "input_image",
            "image_url": part["image_url"],
        }
        if isinstance(detail, str) and detail:
            item["detail"] = detail
        return item

    if isinstance(part.get("file_id"), str) and part["file_id"]:
        item = {
            "type": "input_image",
            "file_id": part["file_id"],
        }
        if isinstance(detail, str) and detail:
            item["detail"] = detail
        return item

    image_path = part.get("image_path")
    if isinstance(image_path, str) and image_path:
        item = {
            "type": "input_image",
            "image_url": image_path_to_data_url(image_path),
        }
        if isinstance(detail, str) and detail:
            item["detail"] = detail
        return item

    return None


def convert_tools(tools: list[dict]) -> list[dict[str, Any]]:
    converted: list[dict[str, Any]] = []
    for tool in tools or []:
        if not isinstance(tool, dict):
            continue
        tool_type = tool.get("type")
        if tool_type == "custom":
            fmt = tool.get("format") if isinstance(tool.get("format"), dict) else None
            if not fmt:
                continue
            converted.append(
                {
                    "type": "custom",
                    "name": tool.get("name", ""),
                    "description": tool.get("description", ""),
                    "format": {
                        "type": fmt.get("type", ""),
                        "syntax": fmt.get("syntax", ""),
                        "definition": fmt.get("definition", ""),
                    },
                }
            )
            continue

        if tool_type == "function":
            func = tool.get("function") if isinstance(tool.get("function"), dict) else None
            if not func:
                continue
            declared_parameters = (
                func.get("parameters") if isinstance(func.get("parameters"), dict) else None
            )
            is_read_file = func.get("name") == "read_file"
            required_override = (
                declared_parameters.get("required")
                if isinstance(declared_parameters, dict)
                and isinstance(declared_parameters.get("required"), list)
                and is_read_file
                else None
            )

            converted.append(
                {
                    "type": "function",
                    "name": func.get("name", ""),
                    "description": func.get("description", ""),
                    "parameters": normalize_function_parameters_for_responses(
                        declared_parameters
                        if isinstance(declared_parameters, dict)
                        else {"type": "object", "properties": {}},
                        required_override=required_override,
                    ),
                    "strict": False if is_read_file else True,
                }
            )
    return converted


def serialize_response_output(response: Any) -> list[dict[str, Any]]:
    output = (
        response.get("output") if isinstance(response, dict) else getattr(response, "output", None)
    )
    if output is None:
        return []

    serialized: list[dict[str, Any]] = []
    for item in output:
        if item is None:
            continue
        if isinstance(item, dict):
            serialized.append(item)
            continue

        dump_json = getattr(item, "model_dump_json", None)
        if callable(dump_json):
            serialized.append(json.loads(dump_json(exclude_none=True)))
            continue

        dump = getattr(item, "model_dump", None)
        if callable(dump):
            serialized.append(
                dump(
                    mode="json",
                    exclude_none=True,
                    warnings="none",
                    serialize_as_any=True,
                )
            )
            continue

        record: dict[str, Any] = {}
        item_type = getattr(item, "type", None)
        if item_type:
            record["type"] = item_type
        for attr in (
            "role",
            "content",
            "name",
            "arguments",
            "input",
            "output",
            "call_id",
            "id",
            "summary",
        ):
            value = getattr(item, attr, None)
            if value is not None:
                record[attr] = value
        if record:
            serialized.append(record)

    return serialized


def extract_usage(response: Any) -> Optional[dict[str, int]]:
    usage = (
        response.get("usage") if isinstance(response, dict) else getattr(response, "usage", None)
    )
    if usage is None:
        return None

    def get(field: str) -> Any:
        if isinstance(usage, dict):
            return usage.get(field)
        return getattr(usage, field, None)

    def get_details() -> Any:
        for name in ("input_tokens_details", "prompt_tokens_details", "input_token_details"):
            details = get(name)
            if details is not None:
                return details
        return None

    def get_from(obj: Any, field: str) -> Any:
        if obj is None:
            return None
        if isinstance(obj, dict):
            return obj.get(field)
        return getattr(obj, field, None)

    input_tokens = get("input_tokens")
    output_tokens = get("output_tokens")
    total_tokens = get("total_tokens")
    cached_tokens = get("cached_tokens")
    if not isinstance(cached_tokens, int):
        details = get_details()
        for field in (
            "cached_tokens",
            "cached_input_tokens",
            "cache_read_tokens",
            "cache_read_input_tokens",
        ):
            value = get_from(details, field)
            if isinstance(value, int):
                cached_tokens = value
                break

    cleaned: dict[str, int] = {}
    if isinstance(input_tokens, int):
        cleaned["prompt_tokens"] = input_tokens
    if isinstance(output_tokens, int):
        cleaned["candidates_tokens"] = output_tokens
    if isinstance(total_tokens, int):
        cleaned["total_tokens"] = total_tokens
    if isinstance(cached_tokens, int):
        cleaned["cached_tokens"] = cached_tokens

    return cleaned or None


def convert_response(response: Any) -> dict[str, Any]:
    text_fragments: list[str] = []
    reasoning_fragments: list[str] = []
    tool_calls: list[dict[str, Any]] = []
    usage = extract_usage(response)

    output = (
        response.get("output") if isinstance(response, dict) else getattr(response, "output", None)
    )
    output = output or []
    for item in output:
        if item is None:
            continue

        item_type = item.get("type") if isinstance(item, dict) else getattr(item, "type", None)
        if item_type == "reasoning":
            summary = (
                item.get("summary") if isinstance(item, dict) else getattr(item, "summary", None)
            )
            reasoning_text = extract_reasoning_summary(summary)
            if reasoning_text:
                reasoning_fragments.append(reasoning_text)
            continue

        if item_type == "message":
            content = (
                item.get("content") if isinstance(item, dict) else getattr(item, "content", None)
            )
            text = extract_message_text(content)
            if text:
                text_fragments.append(text)
            continue

        if item_type == "function_call":
            name = item.get("name") if isinstance(item, dict) else getattr(item, "name", None)
            arguments = (
                item.get("arguments")
                if isinstance(item, dict)
                else getattr(item, "arguments", None)
            )
            call_id = (
                item.get("call_id") if isinstance(item, dict) else getattr(item, "call_id", None)
            )
            tool_calls.append(
                {
                    "id": call_id or f"call_{uuid.uuid4().hex}",
                    "type": "function",
                    "function": {
                        "name": str(name or ""),
                        "arguments": str(arguments or ""),
                    },
                }
            )
            continue

        if item_type == "custom_tool_call":
            name = item.get("name") if isinstance(item, dict) else getattr(item, "name", None)
            input_text = (
                item.get("input") if isinstance(item, dict) else getattr(item, "input", None)
            )
            call_id = (
                item.get("call_id") if isinstance(item, dict) else getattr(item, "call_id", None)
            )
            tool_calls.append(
                {
                    "id": call_id or f"call_{uuid.uuid4().hex}",
                    "type": "custom",
                    "custom": {
                        "name": str(name or ""),
                        "input": str(input_text or ""),
                    },
                }
            )
            continue

    result: dict[str, Any] = {
        "content": "\n".join(text_fragments).strip(),
        "tool_calls": tool_calls,
    }
    if reasoning_fragments:
        result["thought_summary"] = "\n".join(reasoning_fragments).strip()
    if usage:
        result["usage"] = usage
    return result


def extract_response_id(response: Any) -> Optional[str]:
    response_id = (
        response.get("id") if isinstance(response, dict) else getattr(response, "id", None)
    )
    if isinstance(response_id, str) and response_id:
        return response_id
    return None


def is_user_message_item(item: Any) -> bool:
    return isinstance(item, dict) and item.get("role") == "user" and item.get("content") is not None


def extract_reasoning_summary(summary: Any) -> str:
    if not summary:
        return ""
    if isinstance(summary, str):
        return summary.strip()
    parts: list[str] = []
    if isinstance(summary, list):
        for item in summary:
            if isinstance(item, str):
                if item.strip():
                    parts.append(item.strip())
                continue
            text = item.get("text") if isinstance(item, dict) else getattr(item, "text", None)
            if isinstance(text, str) and text.strip():
                parts.append(text.strip())
    return " ".join(parts).strip()


def extract_message_text(content: Any) -> str:
    if not content:
        return ""
    if isinstance(content, str):
        return content
    fragments: list[str] = []
    if isinstance(content, list):
        for part in content:
            if part is None:
                continue
            part_type = part.get("type") if isinstance(part, dict) else getattr(part, "type", None)
            if part_type in {"output_text", "input_text", "text"}:
                text = part.get("text") if isinstance(part, dict) else getattr(part, "text", None)
                if isinstance(text, str) and text:
                    fragments.append(text)
                continue
            text = part.get("text") if isinstance(part, dict) else getattr(part, "text", None)
            if isinstance(text, str) and text:
                fragments.append(text)
    return "\n".join(fragments).strip()


def image_path_to_data_url(image_path: str) -> str:
    path = Path(image_path).expanduser().resolve()
    mime_type, _ = mimetypes.guess_type(path.name)
    if not mime_type:
        mime_type = "application/octet-stream"
    data = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{mime_type};base64,{data}"
