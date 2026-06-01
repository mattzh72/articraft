from __future__ import annotations

import base64
import copy
import json
import mimetypes
import uuid
from pathlib import Path
from typing import Any, Optional


def _get_value(obj: Any, field: str) -> Any:
    if isinstance(obj, dict):
        return obj.get(field)
    return getattr(obj, field, None)


def _scalar(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)):
        return value
    return None


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
    usage = _get_value(response, "usage")
    if usage is None:
        return None

    def get(field: str) -> Any:
        return _get_value(usage, field)

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
    reasoning_tokens = None
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
    for details_name in (
        "output_tokens_details",
        "completion_tokens_details",
        "output_token_details",
    ):
        output_details = get(details_name)
        value = get_from(output_details, "reasoning_tokens")
        if isinstance(value, int):
            reasoning_tokens = value
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
    if isinstance(reasoning_tokens, int):
        cleaned["reasoning_tokens"] = reasoning_tokens

    return cleaned or None


def convert_response(response: Any, *, transport: str | None = None) -> dict[str, Any]:
    text_fragments: list[str] = []
    reasoning_fragments: list[str] = []
    tool_calls: list[dict[str, Any]] = []
    usage = extract_usage(response)

    output = _get_value(response, "output")
    output = output or []
    for item in output:
        if item is None:
            continue

        item_type = _get_value(item, "type")
        if item_type == "reasoning":
            summary = _get_value(item, "summary")
            reasoning_text = extract_reasoning_summary(summary)
            if reasoning_text:
                reasoning_fragments.append(reasoning_text)
            continue

        if item_type == "message":
            content = _get_value(item, "content")
            text = extract_message_text(content)
            if text:
                text_fragments.append(text)
            continue

        if item_type == "function_call":
            name = _get_value(item, "name")
            arguments = _get_value(item, "arguments")
            call_id = _get_value(item, "call_id")
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
            name = _get_value(item, "name")
            input_text = _get_value(item, "input")
            call_id = _get_value(item, "call_id")
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
    diagnostics = extract_provider_diagnostics(response, transport=transport)
    if diagnostics:
        result["provider_diagnostics"] = diagnostics
    return result


def extract_provider_diagnostics(response: Any, *, transport: str | None = None) -> dict[str, Any]:
    diagnostics: dict[str, Any] = {}

    response_id = _get_value(response, "id")
    if isinstance(response_id, str) and response_id:
        diagnostics["response_id"] = response_id

    status = _get_value(response, "status")
    if isinstance(status, str) and status:
        diagnostics["status"] = status

    incomplete_details = _sanitize_incomplete_details(_get_value(response, "incomplete_details"))
    if incomplete_details:
        diagnostics["incomplete_details"] = incomplete_details

    output_items = _summarize_output_items(_get_value(response, "output"))
    if output_items:
        diagnostics["output_items"] = output_items

    if diagnostics and isinstance(transport, str) and transport:
        diagnostics["transport"] = transport

    return diagnostics


def _sanitize_incomplete_details(details: Any) -> dict[str, Any]:
    if details is None:
        return {}

    raw: dict[str, Any] = {}
    dump = getattr(details, "model_dump", None)
    if callable(dump):
        try:
            dumped = dump(mode="json", exclude_none=True, warnings="none")
        except TypeError:
            dumped = dump()
        if isinstance(dumped, dict):
            raw = dumped
    elif isinstance(details, dict):
        raw = details
    else:
        for field in ("reason", "message", "type", "code"):
            value = _get_value(details, field)
            if value is not None:
                raw[field] = value

    sanitized: dict[str, Any] = {}
    for key in ("reason", "message", "type", "code"):
        value = raw.get(key)
        scalar = _scalar(value)
        if scalar is not None:
            sanitized[key] = scalar
    return sanitized


def _summarize_output_items(output: Any) -> list[dict[str, Any]]:
    if not isinstance(output, list):
        return []

    summaries: list[dict[str, Any]] = []
    for index, item in enumerate(output):
        if item is None:
            continue
        summary: dict[str, Any] = {"index": index}
        item_type = _get_value(item, "type")
        if isinstance(item_type, str) and item_type:
            summary["type"] = item_type
        status = _get_value(item, "status")
        if isinstance(status, str) and status:
            summary["status"] = status
        if len(summary) > 1:
            summaries.append(summary)
    return summaries


def extract_response_id(response: Any) -> Optional[str]:
    response_id = _get_value(response, "id")
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
