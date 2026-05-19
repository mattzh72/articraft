from __future__ import annotations

import asyncio
from pathlib import Path

import pytest
from pydantic import ValidationError

from agent.tools import build_tool_registry
from agent.workspace_docs import build_virtual_workspace


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
        "read_file",
        "replace",
        "write_file",
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
    replace_schema = next(
        s for s in gemini_schemas if s.get("function", {}).get("name") == "replace"
    )
    write_file_schema = next(
        s for s in gemini_schemas if s.get("function", {}).get("name") == "write_file"
    )
    assert apply_patch_schema.get("type") == "custom"
    apply_patch_description = apply_patch_schema["description"]
    assert "current bound file" in apply_patch_description
    assert "Single-file mode only" in apply_patch_description
    assert "`*** Add File`, `*** Delete File`, or `*** Move to`" in apply_patch_description
    read_file_props = read_file_schema["function"]["parameters"]["properties"]
    assert set(read_file_props.keys()) == {"path", "offset", "limit"}
    replace_props = replace_schema["function"]["parameters"]["properties"]
    assert set(replace_props.keys()) == {
        "old_string",
        "new_string",
        "instruction",
        "allow_multiple",
    }
    assert replace_schema["function"]["parameters"]["required"] == [
        "old_string",
        "new_string",
    ]
    write_file_props = write_file_schema["function"]["parameters"]["properties"]
    assert set(write_file_props.keys()) == {"content", "path"}
    assert write_file_schema["function"]["parameters"]["required"] == ["content"]
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
    assert read_file.params.offset is None
    assert read_file.params.limit is None

    probe_model = asyncio.run(
        openai_registry.build_invocation(
            "probe_model",
            {"code": "emit(1)", "timeout_ms": None, "include_stdout": None},
        )
    )
    assert probe_model is not None
    assert probe_model.params.code == "emit(1)"
    assert probe_model.params.timeout_ms == 600_000
    assert probe_model.params.include_stdout is False


def test_openai_tool_registry_executes_full_read_without_paging_args(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    model_path = tmp_path / "model.py"
    model_path.write_text("alpha\nbeta\ngamma\n", encoding="utf-8")

    workspace = build_virtual_workspace(
        repo_root,
        model_file_path=model_path,
        sdk_package="sdk",
    )
    openai_registry = build_tool_registry("openai", sdk_package="sdk")

    invocation = asyncio.run(
        openai_registry.build_invocation(
            "read_file",
            {"path": "model.py"},
        )
    )
    assert invocation is not None
    invocation.bind_virtual_workspace(workspace)
    result = asyncio.run(invocation.execute())

    assert result.error is None
    assert result.output == "L1: alpha\nL2: beta\nL3: gamma"


def test_gemini_tool_registry_reads_only_editable_model_region(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    model_path = tmp_path / "model.py"
    model_path.write_text(
        "\n".join(
            [
                "from sdk import *",
                "# >>> USER_CODE_START",
                "def build_object_model():",
                "    return 'alpha'",
                "",
                "def run_tests():",
                "    return None",
                "# >>> USER_CODE_END",
                "FOOTER = True",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    workspace = build_virtual_workspace(
        repo_root,
        model_file_path=model_path,
        sdk_package="sdk",
    )
    gemini_registry = build_tool_registry("gemini", sdk_package="sdk")

    invocation = asyncio.run(
        gemini_registry.build_invocation(
            "read_file",
            {"path": "model.py"},
        )
    )
    assert invocation is not None
    invocation.bind_virtual_workspace(workspace)
    result = asyncio.run(invocation.execute())

    assert result.error is None
    assert result.output == (
        "L1: def build_object_model():\n"
        "L2:     return 'alpha'\n"
        "L3: \n"
        "L4: def run_tests():\n"
        "L5:     return None"
    )
