from __future__ import annotations

import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from agent.tools import build_tool_registry


def main() -> None:
    openai_registry = build_tool_registry("openai")
    gemini_registry = build_tool_registry("gemini")

    assert set(openai_registry.get_all_tool_names()) == {"read_file", "apply_patch"}
    assert set(gemini_registry.get_all_tool_names()) == {"read_code", "edit_code"}

    openai_schemas = openai_registry.get_tool_schemas()
    apply_patch_schema = next(s for s in openai_schemas if s.get("name") == "apply_patch")
    read_file_schema = next(
        s for s in openai_schemas if s.get("function", {}).get("name") == "read_file"
    )
    assert apply_patch_schema.get("type") == "custom"
    read_file_props = read_file_schema["function"]["parameters"]["properties"]
    assert set(read_file_props.keys()) == {"offset", "limit"}


if __name__ == "__main__":
    main()
