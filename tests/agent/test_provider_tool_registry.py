from __future__ import annotations

from agent.tools import build_tool_registry


def test_provider_tool_registry_schemas() -> None:
    openai_registry = build_tool_registry("openai", sdk_package="sdk")
    gemini_registry = build_tool_registry("gemini", sdk_package="sdk")
    hybrid_openai_registry = build_tool_registry("openai", sdk_package="sdk_hybrid")
    hybrid_gemini_registry = build_tool_registry("gemini", sdk_package="sdk_hybrid")

    assert set(openai_registry.get_all_tool_names()) == {"read_file", "apply_patch"}
    assert set(gemini_registry.get_all_tool_names()) == {"read_code", "edit_code"}
    assert set(hybrid_openai_registry.get_all_tool_names()) == {
        "read_file",
        "apply_patch",
        "find_examples",
    }
    assert set(hybrid_gemini_registry.get_all_tool_names()) == {
        "read_code",
        "edit_code",
        "find_examples",
    }

    openai_schemas = openai_registry.get_tool_schemas()
    hybrid_openai_schemas = hybrid_openai_registry.get_tool_schemas()
    apply_patch_schema = next(s for s in openai_schemas if s.get("name") == "apply_patch")
    read_file_schema = next(
        s for s in openai_schemas if s.get("function", {}).get("name") == "read_file"
    )
    find_examples_schema = next(
        s for s in hybrid_openai_schemas if s.get("function", {}).get("name") == "find_examples"
    )
    assert apply_patch_schema.get("type") == "custom"
    read_file_props = read_file_schema["function"]["parameters"]["properties"]
    assert set(read_file_props.keys()) == {"offset", "limit"}
    assert set(find_examples_schema["function"]["parameters"]["properties"].keys()) == {
        "query",
        "limit",
    }
    assert "lexical search" in find_examples_schema["function"]["description"].lower()
    assert "does not search sdk docs" in find_examples_schema["function"]["description"].lower()
    assert (
        "not a general api search tool" in find_examples_schema["function"]["description"].lower()
    )
    assert "available example titles" not in find_examples_schema["function"]["description"].lower()
    assert "short lexical query" in (
        find_examples_schema["function"]["parameters"]["properties"]["query"]["description"].lower()
    )
