from __future__ import annotations

import asyncio

import pytest
from pydantic import ValidationError

from agent.tools import build_tool_registry


def test_provider_tool_registry_schemas() -> None:
    openai_registry = build_tool_registry("openai", sdk_package="sdk")
    gemini_registry = build_tool_registry("gemini", sdk_package="sdk")

    assert set(openai_registry.get_all_tool_names()) == {
        "read_file",
        "apply_patch",
        "compile_model",
        "probe_model",
        "find_examples",
    }
    assert set(gemini_registry.get_all_tool_names()) == {
        "read_code",
        "read_file",
        "edit_code",
        "compile_model",
        "probe_model",
        "find_examples",
    }
    openai_schemas = openai_registry.get_tool_schemas()
    apply_patch_schema = next(s for s in openai_schemas if s.get("name") == "apply_patch")
    read_file_schema = next(
        s for s in openai_schemas if s.get("function", {}).get("name") == "read_file"
    )
    compile_model_schema = next(
        s for s in openai_schemas if s.get("function", {}).get("name") == "compile_model"
    )
    probe_model_schema = next(
        s for s in openai_schemas if s.get("function", {}).get("name") == "probe_model"
    )
    find_examples_schema = next(
        s for s in openai_schemas if s.get("function", {}).get("name") == "find_examples"
    )
    gemini_schemas = gemini_registry.get_tool_schemas()
    edit_code_schema = next(
        s for s in gemini_schemas if s.get("function", {}).get("name") == "edit_code"
    )
    assert apply_patch_schema.get("type") == "custom"
    apply_patch_description = apply_patch_schema["description"]
    assert "current bound file" in apply_patch_description
    assert "Single-file mode only" in apply_patch_description
    assert "`*** Add File`, `*** Delete File`, or `*** Move to`" in apply_patch_description
    read_file_props = read_file_schema["function"]["parameters"]["properties"]
    assert set(read_file_props.keys()) == {"path", "offset", "limit"}
    edit_code_props = edit_code_schema["function"]["parameters"]["properties"]
    assert set(edit_code_props.keys()) == {"old_string", "new_string", "replace_all"}
    assert edit_code_schema["function"]["parameters"]["required"] == [
        "old_string",
        "new_string",
    ]
    compile_model_props = compile_model_schema["function"]["parameters"]["properties"]
    assert set(compile_model_props.keys()) == set()
    assert (
        "structured `<compile_signals>` block" in (compile_model_schema["function"]["description"])
    )
    assert set(probe_model_schema["function"]["parameters"]["properties"].keys()) == {
        "code",
        "timeout_ms",
        "include_stdout",
    }
    probe_description = probe_model_schema["function"]["description"]
    assert "`emit(value)`" in probe_description
    assert "exactly once" in probe_description
    assert "inspection-only" in probe_description
    assert "non-mutating inspection" in probe_description
    assert "exact probe helper catalog and signatures" in probe_description
    assert "current bound `model.py`" in probe_description.lower()
    assert set(find_examples_schema["function"]["parameters"]["properties"].keys()) == {
        "query",
        "limit",
    }
    assert "lexical search" in find_examples_schema["function"]["description"].lower()
    assert "does not search sdk docs" in find_examples_schema["function"]["description"].lower()
    assert (
        "not a general api search tool" in find_examples_schema["function"]["description"].lower()
    )
    assert "weakly relevant" in find_examples_schema["function"]["description"].lower()
    assert "available example titles" not in find_examples_schema["function"]["description"].lower()
    assert "short lexical query" in (
        find_examples_schema["function"]["parameters"]["properties"]["query"]["description"].lower()
    )


def test_tool_registry_rejects_hidden_file_path_parameter() -> None:
    openai_registry = build_tool_registry("openai", sdk_package="sdk")
    gemini_registry = build_tool_registry("gemini", sdk_package="sdk")

    with pytest.raises(ValidationError):
        asyncio.run(
            openai_registry.build_invocation(
                "read_file",
                {"path": "model.py", "offset": 1, "limit": 20, "file_path": "/tmp/model.py"},
            )
        )

    with pytest.raises(ValidationError):
        asyncio.run(
            gemini_registry.build_invocation(
                "read_file",
                {"file_path": "/tmp/model.py"},
            )
        )


def test_openai_tool_registry_treats_null_optional_args_as_defaults() -> None:
    openai_registry = build_tool_registry("openai", sdk_package="sdk")

    read_file = asyncio.run(
        openai_registry.build_invocation(
            "read_file",
            {"path": "model.py", "offset": None, "limit": None},
        )
    )
    assert read_file is not None
    assert read_file.params.path == "model.py"
    assert read_file.params.offset == 1
    assert read_file.params.limit == 200

    probe_model = asyncio.run(
        openai_registry.build_invocation(
            "probe_model",
            {"code": "emit(1)", "timeout_ms": None, "include_stdout": None},
        )
    )
    assert probe_model is not None
    assert probe_model.params.code == "emit(1)"
    assert probe_model.params.timeout_ms == 10_000
    assert probe_model.params.include_stdout is False
