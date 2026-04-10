from __future__ import annotations

from pathlib import Path

import pytest

from agent import runner
from agent.prompts import (
    DESIGNER_PROMPT_NAME,
    GEMINI_DESIGNER_PROMPT_NAME,
    OPENAI_DESIGNER_PROMPT_NAME,
    load_system_prompt_text,
    resolve_system_prompt_path,
)
from agent.runner import build_provider_payload_preview


def _build_openai_preview(
    user_content: object = "a pair of scissors",
    *,
    model_id: str = "gpt-5.4",
    system_prompt_path: str = DESIGNER_PROMPT_NAME,
    sdk_package: str = "sdk",
    sdk_docs_mode: str = "full",
) -> dict:
    return build_provider_payload_preview(
        user_content,
        provider="openai",
        model_id=model_id,
        thinking_level="high",
        system_prompt_path=system_prompt_path,
        sdk_package=sdk_package,
        sdk_docs_mode=sdk_docs_mode,
    )


def test_openai_prompt_resolution_and_payload_preview() -> None:
    repo_root = Path(__file__).resolve().parents[2]

    resolved = resolve_system_prompt_path(
        DESIGNER_PROMPT_NAME,
        provider="openai",
        repo_root=repo_root,
    )
    assert resolved.name == OPENAI_DESIGNER_PROMPT_NAME

    loaded_path, loaded_text = load_system_prompt_text(
        DESIGNER_PROMPT_NAME,
        provider="openai",
        repo_root=repo_root,
    )
    assert loaded_path == resolved
    assert loaded_text == resolved.read_text(encoding="utf-8")

    payload = build_provider_payload_preview(
        "a pair of scissors",
        provider="openai",
        model_id="gpt-5.4",
        thinking_level="high",
        system_prompt_path=DESIGNER_PROMPT_NAME,
    )
    instructions = payload["instructions"]
    docs_message = payload["input"][0]["content"][0]["text"]
    task_message = payload["input"][1]["content"][0]["text"]

    # Section tags
    assert "<tools>" in instructions
    assert "<process>" in instructions
    assert "<modeling>" in instructions

    # Tool contract
    assert (
        "Available tools: `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`."
        in instructions
    )
    assert "FREEFORM tool" in instructions
    assert "write_code" not in instructions
    assert "Prefer several small `apply_patch` edits over one giant patch" in instructions
    assert "Treat `compile_model` as the full validation pass." in instructions
    assert "Prefer the smallest action that gives decisive evidence." in instructions
    assert (
        "If the cause is obvious from `model.py` and `compile_model` output, fix it directly."
        in instructions
    )

    # Three hard requirements
    assert "NO FLOATING PARTS" in instructions
    assert "NO UNINTENTIONAL OVERLAPS" in instructions
    assert "REALISTIC GEOMETRY" in instructions

    # Compact workflow + moved SDK guidance
    assert "Start with a short context pass:" in instructions
    assert "preloaded SDK quickstart/router" in instructions
    assert "only for the specific `docs/` references needed for that next change" in instructions
    assert "read the full file once, not a small slice" in instructions
    assert "Do not re-read it if it is already in context." in instructions
    assert "Start with the smallest coherent backbone or subassembly" in instructions
    assert "Always run `compile_model` on the latest revision before concluding." in instructions
    assert "PHASE 1" not in instructions

    # Provider/system guidance
    assert "inspection-only" in instructions
    assert "lexical search over curated examples for the active SDK" in instructions
    assert "<compile_signals>" in instructions
    assert (
        "Read mounted SDK docs as needed for placement, probe patterns, exact signatures, and testing guidance."
        in instructions
    )
    assert (
        "Prefer Articraft-native primitives and placement helpers when they represent the form credibly."
        in instructions
    )
    assert "Use CadQuery only for the parts that need lower-level shape control" in instructions
    assert "Do not provide `file_path`" not in instructions
    assert "missing exact geometry" not in instructions
    assert "means a gap, not an overlap" not in instructions

    # Runtime first-turn task guidance
    assert task_message.startswith("<runtime_task_guidance>")
    assert (
        'Read the exact current code with `read_file(path="model.py")` before editing.'
        in task_message
    )
    assert (
        "Start with a short context pass: decide the next coherent edit, then read only the docs/examples needed for it."
        in task_message
    )
    assert 'read the full file with `read_file(path="docs/...")`' in task_message
    assert "do not re-read a reference file that is already in context" in task_message
    assert task_message.endswith("a pair of scissors")

    # Cache key
    assert payload["prompt_cache_key"].startswith("ac1:")
    assert len(payload["prompt_cache_key"]) <= 64
    assert "prompt_cache_retention" not in payload

    # SDK docs router bundle is injected
    assert "## docs/sdk/references/quickstart.md" in docs_message
    assert "## docs/sdk/references/probe-tooling.md" in docs_message
    assert "## docs/sdk/references/testing.md" in docs_message
    assert "Virtual Workspace" in docs_message
    assert "Import from `sdk` in `model.py`." in docs_message
    assert "Use `place_on_surface(...)` by default" in docs_message
    assert "Once `run_tests()` references a visual by exact `elem_*` name" in docs_message


def test_openai_core_payload_preview_keeps_router_bundle() -> None:
    docs_message = _build_openai_preview(sdk_package="sdk", sdk_docs_mode="core")["input"][0][
        "content"
    ][0]["text"]
    assert "## docs/sdk/references/quickstart.md" in docs_message
    assert "## docs/sdk/references/probe-tooling.md" in docs_message
    assert "## docs/sdk/references/testing.md" in docs_message


def test_openai_payload_preview_includes_find_examples_tool() -> None:
    payload = _build_openai_preview(
        sdk_package="sdk",
        sdk_docs_mode="full",
    )

    tool_names = {tool["name"] for tool in payload["tools"]}
    assert "compile_model" in tool_names
    assert "probe_model" in tool_names
    assert "find_examples" in tool_names
    assert all(
        tool.get("type") != "function" or tool.get("strict") is True for tool in payload["tools"]
    )
    assert (
        "Available tools: `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`."
        in payload["instructions"]
    )
    assert "lexical search over curated examples for the active SDK" in payload["instructions"]


def test_openai_multimodal_payload_preview_keeps_image_and_appends_guidance(
    tmp_path: Path,
) -> None:
    image_path = tmp_path / "reference.png"
    image_path.write_bytes(b"\x89PNG\r\n\x1a\n")

    payload = _build_openai_preview(
        user_content=[
            {"type": "input_text", "text": "a table lamp"},
            {"type": "input_image", "image_path": str(image_path), "detail": "high"},
        ]
    )

    task_parts = payload["input"][1]["content"]

    assert task_parts[0]["type"] == "input_text"
    assert task_parts[0]["text"].startswith("<runtime_task_guidance>")
    assert "apply_patch" in task_parts[0]["text"]
    assert task_parts[1] == {"type": "input_text", "text": "a table lamp"}
    assert task_parts[2]["type"] == "input_image"
    assert task_parts[2]["detail"] == "high"
    assert task_parts[2]["image_url"].startswith("data:image/png;base64,")


def test_openai_prompt_cache_key_is_stable_across_user_prompt_changes() -> None:
    first_payload = _build_openai_preview("a pair of scissors")
    second_payload = _build_openai_preview("a tower crane")

    assert first_payload["prompt_cache_key"] == second_payload["prompt_cache_key"]


def test_openai_prompt_cache_key_is_stable_when_sdk_inputs_normalize_to_same_prefix() -> None:
    full_payload = _build_openai_preview(sdk_package="sdk", sdk_docs_mode="full")
    core_payload = _build_openai_preview(sdk_package="sdk", sdk_docs_mode="core")
    hybrid_payload = _build_openai_preview(sdk_package="sdk_hybrid", sdk_docs_mode="full")

    assert full_payload["prompt_cache_key"] == core_payload["prompt_cache_key"]
    assert full_payload["prompt_cache_key"] == hybrid_payload["prompt_cache_key"]


def test_openai_prompt_cache_key_changes_for_system_prompt_contents(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    _, base_text = load_system_prompt_text(
        DESIGNER_PROMPT_NAME,
        provider="openai",
        repo_root=repo_root,
    )
    prompt_a = tmp_path / "prompt_a.txt"
    prompt_b = tmp_path / "prompt_b.txt"
    prompt_a.write_text(base_text, encoding="utf-8")
    prompt_b.write_text(base_text + "\n# cache-key-variant\n", encoding="utf-8")

    payload_a = _build_openai_preview(system_prompt_path=str(prompt_a))
    payload_b = _build_openai_preview(system_prompt_path=str(prompt_b))

    assert payload_a["prompt_cache_key"] != payload_b["prompt_cache_key"]


def test_openai_prompt_cache_key_changes_for_tool_schema_set(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    base_payload = _build_openai_preview()
    original_builder = runner.build_tool_registry

    class _Registry:
        def __init__(self, tool_schemas: list[dict]) -> None:
            self._tool_schemas = tool_schemas

        def get_tool_schemas(self) -> list[dict]:
            return list(self._tool_schemas)

    def _build_modified_registry(provider: str, *, sdk_package: str = "sdk") -> _Registry:
        original = original_builder(provider, sdk_package=sdk_package)
        extra_tool = {
            "type": "function",
            "function": {
                "name": "cache_probe",
                "description": "Test-only tool variant",
                "parameters": {"type": "object", "properties": {}},
            },
        }
        return _Registry(original.get_tool_schemas() + [extra_tool])

    monkeypatch.setattr(runner, "build_tool_registry", _build_modified_registry)
    modified_payload = _build_openai_preview()

    assert base_payload["prompt_cache_key"] != modified_payload["prompt_cache_key"]


def test_openai_prompt_cache_key_can_be_disabled_by_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("OPENAI_PROMPT_CACHE_KEY_STRATEGY", "off")

    payload = _build_openai_preview()

    assert "prompt_cache_key" not in payload
    assert "prompt_cache_retention" not in payload


def test_openai_prompt_cache_retention_can_be_disabled_by_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("OPENAI_PROMPT_CACHE_RETENTION", "off")

    payload = _build_openai_preview()

    assert "prompt_cache_retention" not in payload
    assert payload["prompt_cache_key"].startswith("ac1:")
    assert len(payload["prompt_cache_key"]) <= 64


def test_openai_prompt_cache_retention_defaults_for_supported_models() -> None:
    payload = _build_openai_preview(model_id="gpt-5")

    assert payload["prompt_cache_retention"] == "24h"


def test_openai_prompt_cache_key_stays_within_length_limit_with_prefix(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("OPENAI_PROMPT_CACHE_KEY_PREFIX", "workspace/very-long-prefix-value")

    payload = _build_openai_preview()

    assert payload["prompt_cache_key"].startswith("ac1:")
    assert len(payload["prompt_cache_key"]) <= 64


def test_gemini_prompt_resolution_and_payload_preview() -> None:
    repo_root = Path(__file__).resolve().parents[2]

    gemini_resolved = resolve_system_prompt_path(
        DESIGNER_PROMPT_NAME,
        provider="gemini",
        repo_root=repo_root,
    )
    assert gemini_resolved.name == GEMINI_DESIGNER_PROMPT_NAME

    gemini_payload = build_provider_payload_preview(
        "a pair of scissors",
        provider="gemini",
        model_id="gemini-2.5-pro",
        thinking_level="high",
        system_prompt_path=DESIGNER_PROMPT_NAME,
    )
    gemini_instructions = gemini_payload["config"]["system_instruction"]
    gemini_docs_message = gemini_payload["contents"][0]["parts"][0]["text"]
    gemini_task_message = gemini_payload["contents"][1]["parts"][0]["text"]

    # Section tags
    assert "<process>" in gemini_instructions
    assert "<tools>" in gemini_instructions
    assert "<modeling>" in gemini_instructions
    assert "<compile_signals>" in gemini_instructions

    # Tool contract
    assert (
        "Available tools: `read_file`, `edit_code`, `compile_model`, `probe_model`, and `find_examples`."
        in gemini_instructions
    )
    assert 'old_string=""' in gemini_instructions
    assert "write_code" not in gemini_instructions
    assert "Prefer small exact `edit_code` replacements over broad rewrites" in gemini_instructions
    assert "Treat `compile_model` as the full validation pass." in gemini_instructions
    assert "Prefer the smallest action that gives decisive evidence." in gemini_instructions
    assert (
        "If the cause is obvious from `model.py` and `compile_model` output, fix it directly."
        in gemini_instructions
    )

    # Three hard requirements
    assert "NO FLOATING PARTS" in gemini_instructions
    assert "NO UNINTENTIONAL OVERLAPS" in gemini_instructions
    assert "REALISTIC GEOMETRY" in gemini_instructions

    # Compact workflow
    assert "Start with a short context pass:" in gemini_instructions
    assert "preloaded SDK quickstart/router" in gemini_instructions
    assert (
        "only for the specific `docs/` references needed for that next change"
        in gemini_instructions
    )
    assert "read the full file once, not a small slice" in gemini_instructions
    assert "Do not re-read it if it is already in context." in gemini_instructions
    assert "Start with the smallest coherent backbone or subassembly" in gemini_instructions
    assert (
        "Always run `compile_model` on the latest revision before concluding."
        in gemini_instructions
    )
    assert "PHASE 1" not in gemini_instructions

    # Provider/system guidance
    assert "inspection-only" in gemini_instructions
    assert "lexical search over curated examples for the active SDK" in gemini_instructions
    assert (
        "Read mounted SDK docs as needed for placement, probe patterns, exact signatures, and testing guidance."
        in gemini_instructions
    )
    assert (
        "Prefer Articraft-native primitives and placement helpers when they represent the form credibly."
        in gemini_instructions
    )
    assert (
        "Use CadQuery only for the parts that need lower-level shape control" in gemini_instructions
    )

    # Runtime first-turn task guidance
    assert gemini_task_message.startswith("<runtime_task_guidance>")
    assert (
        'Read the exact current code with `read_file(path="model.py")` before editing.'
        in gemini_task_message
    )
    assert (
        "Start with a short context pass: decide the next coherent edit, then read only the docs/examples needed for it."
        in gemini_task_message
    )
    assert 'read_file(path="docs/...")' in gemini_task_message
    assert "do not re-read a reference file that is already in context" in gemini_task_message
    assert gemini_task_message.endswith("a pair of scissors")

    assert "## docs/sdk/references/quickstart.md" in gemini_docs_message
    assert "## docs/sdk/references/probe-tooling.md" in gemini_docs_message
    assert "Mounted Reference Layout" in gemini_docs_message
    assert "Once `run_tests()` references a visual by exact `elem_*` name" in gemini_docs_message


def test_gemini_payload_preview_includes_find_examples_tool() -> None:
    payload = build_provider_payload_preview(
        "a pair of scissors",
        provider="gemini",
        model_id="gemini-2.5-pro",
        thinking_level="high",
        system_prompt_path=DESIGNER_PROMPT_NAME,
        sdk_package="sdk",
    )

    declarations = payload["config"]["tools"][0]["function_declarations"]
    tool_names = {tool["name"] for tool in declarations}
    assert "compile_model" in tool_names
    assert "probe_model" in tool_names
    assert "find_examples" in tool_names
    assert (
        "Available tools: `read_file`, `edit_code`, `compile_model`, `probe_model`, and `find_examples`."
        in payload["config"]["system_instruction"]
    )
    assert (
        "lexical search over curated examples for the active SDK"
        in payload["config"]["system_instruction"]
    )


def test_gemini_multimodal_payload_preview_keeps_image_and_appends_guidance(
    tmp_path: Path,
) -> None:
    image_path = tmp_path / "reference.png"
    image_path.write_bytes(b"\x89PNG\r\n\x1a\n")

    payload = build_provider_payload_preview(
        [
            {"type": "input_text", "text": "a table lamp"},
            {"type": "input_image", "image_path": str(image_path), "detail": "high"},
        ],
        provider="gemini",
        model_id="gemini-2.5-pro",
        thinking_level="high",
        system_prompt_path=DESIGNER_PROMPT_NAME,
    )

    task_parts = payload["contents"][1]["parts"]

    assert task_parts[0]["text"].startswith("<runtime_task_guidance>")
    assert any(
        isinstance(part, dict) and ("inline_data" in part or "inlineData" in part)
        for part in task_parts
    )
    assert any(part == {"text": "a table lamp"} for part in task_parts if isinstance(part, dict))
    assert "edit_code" in task_parts[0]["text"]
