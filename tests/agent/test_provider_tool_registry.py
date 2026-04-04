from __future__ import annotations

import asyncio

import pytest
from pydantic import ValidationError

from agent.tools import build_tool_registry


def test_provider_tool_registry_schemas() -> None:
    openai_registry = build_tool_registry("openai", sdk_package="sdk")
    gemini_registry = build_tool_registry("gemini", sdk_package="sdk")
    hybrid_openai_registry = build_tool_registry("openai", sdk_package="sdk_hybrid")
    hybrid_gemini_registry = build_tool_registry("gemini", sdk_package="sdk_hybrid")

    assert set(openai_registry.get_all_tool_names()) == {
        "read_file",
        "apply_patch",
        "compile_model",
        "probe_model",
        "find_examples",
    }
    assert set(gemini_registry.get_all_tool_names()) == {
        "read_code",
        "edit_code",
        "compile_model",
        "probe_model",
        "find_examples",
    }
    assert set(hybrid_openai_registry.get_all_tool_names()) == {
        "read_file",
        "apply_patch",
        "compile_model",
        "probe_model",
        "find_examples",
    }
    assert set(hybrid_gemini_registry.get_all_tool_names()) == {
        "read_code",
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
    assert apply_patch_schema.get("type") == "custom"
    read_file_props = read_file_schema["function"]["parameters"]["properties"]
    assert set(read_file_props.keys()) == {"offset", "limit"}
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
                {"offset": 1, "limit": 20, "file_path": "/tmp/model.py"},
            )
        )

    with pytest.raises(ValidationError):
        asyncio.run(
            gemini_registry.build_invocation(
                "read_code",
                {"file_path": "/tmp/model.py"},
            )
        )
