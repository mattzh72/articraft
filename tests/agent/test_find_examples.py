from __future__ import annotations

import asyncio
import json
from pathlib import Path

from agent.examples import load_example_documents, parse_example_document, search_example_documents
from agent.harness import ArticraftAgent
from agent.models import TerminateReason
from agent.tools.find_examples import FindExamplesTool


def test_parse_example_document_reads_frontmatter() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    path = repo_root / "sdk" / "_examples" / "cadquery" / "simple_rectangular_plate.md"

    doc = parse_example_document(path)

    assert doc.title == "Simple Rectangular Plate"
    assert doc.description.startswith("Just about the simplest possible example")
    assert "cadquery" in doc.tags
    assert doc.content.startswith("---\n")


def test_search_example_documents_prefers_structured_matches() -> None:
    matches = search_example_documents("fillet", sdk_package="sdk", limit=3)

    assert matches
    assert matches[0].title == "Rounding Corners with Fillet"
    assert matches[0].match_quality == "strong"


def test_search_example_documents_returns_full_content() -> None:
    matches = search_example_documents("counterbore", sdk_package="sdk", limit=1)

    assert len(matches) == 1
    assert "cboreHole" in matches[0].content
    assert matches[0].content.startswith("---\n")


def test_search_example_documents_default_limit_is_used() -> None:
    matches = search_example_documents("workplane", sdk_package="sdk")

    assert len(matches) == 3


def test_search_example_documents_honors_explicit_large_limit() -> None:
    matches = search_example_documents("workplane", sdk_package="sdk", limit=100)

    assert len(matches) > 3


def test_search_example_documents_returns_empty_list_for_no_match() -> None:
    assert search_example_documents("nonexistent-mechanism-token", sdk_package="sdk") == []


def test_search_example_documents_ignores_generic_cadquery_tag_matches() -> None:
    matches = search_example_documents(
        "bottle cadquery classic occ bottle",
        sdk_package="sdk",
        limit=10,
    )

    assert [doc.title for doc in matches] == ["The Classic OCC Bottle"]


def test_search_example_documents_keeps_specific_body_api_queries() -> None:
    matches = search_example_documents("cboreHole", sdk_package="sdk", limit=10)

    assert matches
    assert matches[0].title == "Making Counter-bored and Counter-sunk Holes"
    assert "A Parametric Bearing Pillow Block" in [doc.title for doc in matches]
    assert all(doc.match_quality == "strong" for doc in matches)


def test_sdk_example_corpus_titles_are_unique() -> None:
    docs = load_example_documents("sdk")

    assert docs
    titles = [doc.title for doc in docs]
    assert len(titles) == len(set(titles))


def test_find_examples_tool_returns_expected_shape() -> None:
    async def _run() -> list[dict[str, object]]:
        tool = FindExamplesTool(sdk_package="sdk")
        invocation = await tool.build({"query": "loft", "limit": 1})
        result = await invocation.execute()
        assert result.error is None
        assert isinstance(result.output, list)
        return result.output

    output = asyncio.run(_run())

    assert len(output) == 1
    assert output[0]["title"] == "Making Lofts"
    assert output[0]["example_id"] == "sdk/_examples/cadquery/making_lofts.md"
    assert output[0]["path"] == "sdk/_examples/cadquery/making_lofts.md"
    assert "content" in output[0]
    assert output[0]["match_quality"] == "strong"
    assert output[0]["matched_tokens"]
    assert output[0]["matched_fields"]


def test_search_example_documents_returns_base_sdk_jet_engine_example() -> None:
    matches = search_example_documents("jet engine nacelle turbofan", sdk_package="sdk", limit=3)

    assert matches
    assert matches[0].title == "Jet Engine with Smooth Nacelle and Dense Front Fan"
    assert matches[0].match_quality == "strong"


def test_search_example_documents_returns_base_sdk_atv_example() -> None:
    matches = search_example_documents(
        "atv quad bike steering suspension", sdk_package="sdk", limit=3
    )

    assert matches
    assert matches[0].title == "ATV Quad Bike with Front Steering and Suspension"


def test_search_example_documents_returns_base_sdk_tower_crane_example() -> None:
    matches = search_example_documents(
        "tower crane trolley lattice mast", sdk_package="sdk", limit=3
    )

    assert matches
    assert matches[0].title == "Tower Crane with Lattice Mast and Traveling Trolley"


def test_search_example_documents_returns_base_sdk_radio_telescope_example() -> None:
    matches = search_example_documents(
        "radio telescope dish azimuth elevation mount",
        sdk_package="sdk",
        limit=3,
    )

    assert matches
    assert matches[0].title == "Radio Telescope on Azimuth-Elevation Mount"


def test_search_example_documents_returns_base_sdk_midi_keyboard_example() -> None:
    matches = search_example_documents(
        "midi keyboard white keys knobs pads pitch wheel",
        sdk_package="sdk",
        limit=3,
    )

    assert matches
    assert matches[0].title == "MIDI Keyboard with Articulated Keys, Knobs, and Pads"


def test_search_example_documents_returns_base_sdk_stand_mixer_example() -> None:
    matches = search_example_documents(
        "artisan stand mixer tilt head balloon whisk speed slider",
        sdk_package="sdk",
        limit=3,
    )

    assert matches
    assert (
        matches[0].title
        == "Artisan Stand Mixer with Tilt Head, Balloon Whisk, and Articulated Controls"
    )


def test_find_examples_tool_supports_base_sdk_examples() -> None:
    async def _run() -> list[dict[str, object]]:
        tool = FindExamplesTool(sdk_package="sdk")
        invocation = await tool.build({"query": "jet engine nacelle", "limit": 3})
        result = await invocation.execute()
        assert result.error is None
        assert isinstance(result.output, list)
        return result.output

    output = asyncio.run(_run())

    assert output
    assert output[0]["title"] == "Jet Engine with Smooth Nacelle and Dense Front Fan"
    assert (
        output[0]["example_id"]
        == "sdk/_examples/base/jet_engine_with_smooth_nacelle_dense_front_fan.md"
    )
    assert (
        output[0]["path"] == "sdk/_examples/base/jet_engine_with_smooth_nacelle_dense_front_fan.md"
    )
    assert output[0]["match_quality"] == "strong"
    assert output[0]["matched_tokens"]
    assert output[0]["matched_fields"]


def test_find_examples_tool_can_omit_repo_paths_for_gemini() -> None:
    async def _run() -> list[dict[str, object]]:
        tool = FindExamplesTool(sdk_package="sdk", include_paths=False)
        invocation = await tool.build({"query": "loft", "limit": 1})
        result = await invocation.execute()
        assert result.error is None
        assert isinstance(result.output, list)
        return result.output

    output = asyncio.run(_run())

    assert len(output) == 1
    assert output[0]["title"] == "Making Lofts"
    assert output[0]["example_id"] == "sdk/_examples/cadquery/making_lofts.md"
    assert "path" not in output[0]


def test_search_example_documents_sdk_can_retrieve_cadquery_examples() -> None:
    matches = search_example_documents("making lofts", sdk_package="sdk", limit=3)

    assert matches
    assert matches[0].title == "Making Lofts"
    assert matches[0].path.as_posix().endswith("sdk/_examples/cadquery/making_lofts.md")


def test_search_example_documents_can_return_weakly_relevant_base_matches() -> None:
    matches = search_example_documents("support paddle", sdk_package="sdk", limit=5)

    assert matches
    assert [doc.title for doc in matches] == ["Radio Telescope on Azimuth-Elevation Mount"]
    assert all(doc.match_quality == "weakly_relevant" for doc in matches)
    assert len(matches) <= 2


def test_find_examples_repeated_results_replace_full_content_with_blurb() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._seen_find_example_paths = set()

    first = agent._compress_find_examples_output(
        [
            {
                "example_id": "sdk/_examples/cadquery/making_lofts.md",
                "title": "Making Lofts",
                "description": "Loft example",
                "tags": ["cadquery"],
                "path": "sdk/_examples/cadquery/making_lofts.md",
                "content": "# full example",
            }
        ]
    )
    second = agent._compress_find_examples_output(
        [
            {
                "example_id": "sdk/_examples/cadquery/making_lofts.md",
                "title": "Making Lofts",
                "description": "Loft example",
                "tags": ["cadquery"],
                "path": "sdk/_examples/cadquery/making_lofts.md",
                "content": "# full example",
            }
        ]
    )

    assert first[0]["content"] == "# full example"
    assert second[0]["content_skipped"] is True
    assert "already returned earlier in this run" in second[0]["content"]


def test_find_examples_cache_can_seed_from_prior_conversation() -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent._seen_find_example_paths = set()

    agent._seed_find_examples_cache_from_conversation(
        [
            {
                "role": "tool",
                "name": "find_examples",
                "content": json.dumps(
                    {
                        "result": [
                            {
                                "example_id": "sdk/_examples/cadquery/making_lofts.md",
                                "title": "Making Lofts",
                                "description": "Loft example",
                                "tags": ["cadquery"],
                                "path": "sdk/_examples/cadquery/making_lofts.md",
                                "content": "# full example",
                            }
                        ]
                    }
                ),
            }
        ]
    )

    compressed = agent._compress_find_examples_output(
        [
            {
                "example_id": "sdk/_examples/cadquery/making_lofts.md",
                "title": "Making Lofts",
                "description": "Loft example",
                "tags": ["cadquery"],
                "path": "sdk/_examples/cadquery/making_lofts.md",
                "content": "# full example",
            }
        ]
    )

    assert compressed[0]["content_skipped"] is True


def test_first_turn_no_tool_response_no_longer_injects_nudge(tmp_path: Path) -> None:
    class _FakeLLM:
        async def generate_with_tools(
            self, *, system_prompt: str, messages: list[dict], tools: list[dict]
        ) -> dict:
            return {"content": "Need more time.", "tool_calls": []}

    class _Display:
        current_turn = 0

        def start(self) -> None:
            return None

        def start_turn(self, turn: int) -> None:
            self.current_turn = turn

        def start_llm_wait(self) -> None:
            return None

        def stop_llm_wait(self) -> None:
            return None

        def end_turn(self, success: bool, error: str | None = None) -> None:
            return None

        def add_thinking_summary(self, thinking: str) -> None:
            return None

    class _ToolRegistry:
        def get_tool_schemas(self) -> list[dict]:
            return []

    agent = ArticraftAgent.__new__(ArticraftAgent)
    code_path = tmp_path / "model.py"
    code_path.write_text("from __future__ import annotations\n", encoding="utf-8")
    agent.file_path = str(code_path)
    agent.max_turns = 1
    agent.sdk_docs_context = ""
    agent.display = _Display()
    agent.llm = _FakeLLM()
    agent.tool_registry = _ToolRegistry()
    agent.trace_writer = None
    agent.on_turn_start = None
    agent.system_prompt = ""
    agent.cost_tracker = None
    agent.provider = "openai"
    agent._last_compile_failure_sig = None
    agent._post_success_design_audit_sent = False
    agent._seen_compile_signal_sigs = set()
    agent._seen_tool_error_sigs = set()
    agent._last_checkpoint_urdf_sig = None
    agent.checkpoint_urdf_path = None
    agent._ensure_code_file = lambda: None
    agent._should_terminate = lambda text, tool_calls, turn: (False, TerminateReason.MAX_TURNS, "")

    result = asyncio.run(agent.run("make a bracket"))

    assert result.reason == TerminateReason.MAX_TURNS
    assert len(result.conversation) == 3
    assert result.conversation[0]["content"].startswith("<runtime_task_guidance>")
    assert (
        "Prefer multiple small `apply_patch` edits over one giant patch."
        in result.conversation[0]["content"]
    )
    assert result.conversation[0]["content"].endswith("make a bracket")
    assert result.conversation[2]["content"].startswith("<compile_required>")
    assert all(
        "<first_turn_tool_nudge>" not in str(message.get("content", ""))
        for message in result.conversation
    )
