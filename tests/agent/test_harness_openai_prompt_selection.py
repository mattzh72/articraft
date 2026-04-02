from __future__ import annotations

from pathlib import Path

import pytest

from agent import runner
from agent.prompts import (
    DESIGNER_PROMPT_NAME,
    GEMINI_DESIGNER_PROMPT_NAME,
    HYBRID_GEMINI_DESIGNER_PROMPT_NAME,
    HYBRID_OPENAI_DESIGNER_PROMPT_NAME,
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

    hybrid_resolved = resolve_system_prompt_path(
        DESIGNER_PROMPT_NAME,
        provider="openai",
        sdk_package="sdk_hybrid",
        repo_root=repo_root,
    )
    assert hybrid_resolved.name == HYBRID_OPENAI_DESIGNER_PROMPT_NAME

    payload = build_provider_payload_preview(
        "a pair of scissors",
        provider="openai",
        model_id="gpt-5.4",
        thinking_level="high",
        system_prompt_path=DESIGNER_PROMPT_NAME,
    )
    instructions = payload["instructions"]
    docs_message = payload["input"][0]["content"][0]["text"]

    # Section tags
    assert "<tools>" in instructions
    assert "<process>" in instructions
    assert "<modeling>" in instructions

    # Tool contract
    assert (
        "Use ONLY `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`"
        in instructions
    )
    assert "FREEFORM tool" in instructions
    assert "write_code" not in instructions
    assert "Do not provide `file_path`" in instructions
    assert "Use `compile_model` explicitly to run full compile + QC" in instructions

    # Three hard requirements
    assert "NO FLOATING PARTS" in instructions
    assert "NO UNINTENTIONAL OVERLAPS" in instructions
    assert "REALISTIC GEOMETRY" in instructions

    # Phased workflow
    assert "PHASE 1" in instructions
    assert "Do NOT write all geometry in one giant edit" in instructions
    assert "simple envelope geometry or minimal sketches/extrusions" in instructions
    assert "Upgrade each region from envelope geometry to realistic geometry" in instructions
    assert "keep its learned dimensions/joints/attachments" in instructions

    # Probe and testing guidance
    assert "inspection-only" in instructions
    assert "lexical search over curated base SDK examples" in instructions
    assert "<compile_signals>" in instructions
    assert "SDK docs" in instructions
    assert "object-first" in instructions
    assert "find_floating_parts(...)" in instructions
    assert "exactly one root part" in instructions
    assert "Use `run_tests()` for prompt-specific exact checks" in instructions
    assert "treat that name as a contract" in instructions
    assert "means a gap, not an overlap" in instructions

    # Cache key
    assert payload["prompt_cache_key"].startswith("ac1:")
    assert len(payload["prompt_cache_key"]) <= 64
    assert payload["prompt_cache_retention"] == "24h"

    # SDK docs injected
    assert "## sdk/_docs/common/00_quickstart.md" in docs_message
    assert "## sdk/_docs/common/70_probe_tooling.md" in docs_message
    assert "## sdk/_docs/common/80_testing.md" in docs_message


def test_openai_hybrid_payload_preview_includes_hybrid_docs() -> None:
    hybrid_docs_message = _build_openai_preview(
        sdk_package="sdk_hybrid",
        sdk_docs_mode="full",
    )["input"][0]["content"][0]["text"]
    assert "## sdk/_docs/common/00_quickstart.md" in hybrid_docs_message
    assert "## sdk/_docs/cadquery/35_cadquery.md" in hybrid_docs_message
    assert "## sdk/_docs/common/70_probe_tooling.md" in hybrid_docs_message
    assert "## sdk/_docs/common/80_testing.md" in hybrid_docs_message
    assert "## sdk/_docs/cadquery/39a_cadquery_examples.md" not in hybrid_docs_message


def test_openai_core_payload_preview_includes_probe_doc() -> None:
    docs_message = _build_openai_preview(sdk_package="sdk", sdk_docs_mode="core")["input"][0][
        "content"
    ][0]["text"]
    assert "## sdk/_docs/common/00_quickstart.md" in docs_message
    assert "## sdk/_docs/common/70_probe_tooling.md" in docs_message
    assert "## sdk/_docs/common/80_testing.md" in docs_message
    assert "## sdk/_docs/base/40_mesh_geometry.md" not in docs_message


def test_openai_hybrid_core_payload_preview_includes_probe_doc() -> None:
    docs_message = _build_openai_preview(sdk_package="sdk_hybrid", sdk_docs_mode="core")["input"][
        0
    ]["content"][0]["text"]
    assert "## sdk/_docs/common/00_quickstart.md" in docs_message
    assert "## sdk/_docs/cadquery/35_cadquery.md" in docs_message
    assert "## sdk/_docs/common/70_probe_tooling.md" in docs_message
    assert "## sdk/_docs/common/80_testing.md" in docs_message
    assert "## sdk/_docs/cadquery/36_cadquery_primer.md" not in docs_message


def test_openai_hybrid_payload_preview_includes_find_examples_tool() -> None:
    payload = _build_openai_preview(
        sdk_package="sdk_hybrid",
        sdk_docs_mode="full",
    )

    tool_names = {tool["name"] for tool in payload["tools"]}
    assert "compile_model" in tool_names
    assert "probe_model" in tool_names
    assert "find_examples" in tool_names
    assert (
        "Use ONLY `read_file`, `apply_patch`, `compile_model`, `probe_model`, and `find_examples`"
        in payload["instructions"]
    )
    assert "lexical search over curated hybrid/CadQuery examples" in payload["instructions"]


def test_openai_prompt_cache_key_is_stable_across_user_prompt_changes() -> None:
    first_payload = _build_openai_preview("a pair of scissors")
    second_payload = _build_openai_preview("a tower crane")

    assert first_payload["prompt_cache_key"] == second_payload["prompt_cache_key"]


def test_openai_prompt_cache_key_changes_for_sdk_package_and_docs_mode() -> None:
    base_payload = _build_openai_preview(sdk_package="sdk", sdk_docs_mode="full")
    hybrid_payload = _build_openai_preview(sdk_package="sdk_hybrid", sdk_docs_mode="full")
    core_payload = _build_openai_preview(sdk_package="sdk", sdk_docs_mode="core")

    assert base_payload["prompt_cache_key"] != hybrid_payload["prompt_cache_key"]
    assert base_payload["prompt_cache_key"] != core_payload["prompt_cache_key"]


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
    assert payload["prompt_cache_retention"] == "24h"


def test_openai_prompt_cache_retention_can_be_disabled_by_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("OPENAI_PROMPT_CACHE_RETENTION", "off")

    payload = _build_openai_preview()

    assert "prompt_cache_retention" not in payload
    assert payload["prompt_cache_key"].startswith("ac1:")
    assert len(payload["prompt_cache_key"]) <= 64


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

    gemini_hybrid_resolved = resolve_system_prompt_path(
        DESIGNER_PROMPT_NAME,
        provider="gemini",
        sdk_package="sdk_hybrid",
        repo_root=repo_root,
    )
    assert gemini_hybrid_resolved.name == HYBRID_GEMINI_DESIGNER_PROMPT_NAME

    gemini_payload = build_provider_payload_preview(
        "a pair of scissors",
        provider="gemini",
        model_id="gemini-2.5-pro",
        thinking_level="high",
        system_prompt_path=DESIGNER_PROMPT_NAME,
    )
    gemini_instructions = gemini_payload["config"]["system_instruction"]
    gemini_docs_message = gemini_payload["contents"][0]["parts"][0]["text"]

    # Section tags
    assert "<process>" in gemini_instructions
    assert "<tools>" in gemini_instructions
    assert "<modeling>" in gemini_instructions
    assert "<compile_signals>" in gemini_instructions

    # Tool contract
    assert (
        "Use ONLY `read_code`, `edit_code`, `compile_model`, `probe_model`, and `find_examples`"
        in gemini_instructions
    )
    assert 'old_string=""' in gemini_instructions
    assert "write_code" not in gemini_instructions
    assert "Use `compile_model` explicitly to run full compile + QC" in gemini_instructions

    # Three hard requirements
    assert "NO FLOATING PARTS" in gemini_instructions
    assert "NO UNINTENTIONAL OVERLAPS" in gemini_instructions
    assert "REALISTIC GEOMETRY" in gemini_instructions

    # Phased workflow
    assert "PHASE 1" in gemini_instructions
    assert "Do NOT write all geometry in one giant edit" in gemini_instructions

    # Probe and testing
    assert "inspection-only" in gemini_instructions
    assert "lexical search over curated base SDK examples" in gemini_instructions
    assert "object-first" in gemini_instructions
    assert "find_floating_parts(...)" in gemini_instructions
    assert "exactly one root part" in gemini_instructions
    assert "Use `run_tests()` for prompt-specific exact checks" in gemini_instructions

    assert "## sdk/_docs/common/70_probe_tooling.md" in gemini_docs_message


def test_gemini_hybrid_payload_preview_includes_find_examples_tool() -> None:
    payload = build_provider_payload_preview(
        "a pair of scissors",
        provider="gemini",
        model_id="gemini-2.5-pro",
        thinking_level="high",
        system_prompt_path=DESIGNER_PROMPT_NAME,
        sdk_package="sdk_hybrid",
    )

    declarations = payload["config"]["tools"][0]["function_declarations"]
    tool_names = {tool["name"] for tool in declarations}
    assert "compile_model" in tool_names
    assert "probe_model" in tool_names
    assert "find_examples" in tool_names
    assert (
        "Use ONLY `read_code`, `edit_code`, `compile_model`, `probe_model`, and `find_examples`"
        in payload["config"]["system_instruction"]
    )
    assert (
        "lexical search over curated hybrid/CadQuery examples"
        in payload["config"]["system_instruction"]
    )
