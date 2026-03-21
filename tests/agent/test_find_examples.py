from __future__ import annotations

import asyncio
from pathlib import Path

from agent.examples import load_example_documents, parse_example_document, search_example_documents
from agent.harness import ArticraftAgent
from agent.models import TerminateReason
from agent.tools.find_examples import FindExamplesTool


def test_parse_example_document_reads_frontmatter() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    path = repo_root / "sdk" / "_examples" / "hybrid" / "simple_rectangular_plate.md"

    doc = parse_example_document(path)

    assert doc.title == "Simple Rectangular Plate"
    assert doc.description.startswith("Just about the simplest possible example")
    assert "cadquery" in doc.tags
    assert doc.content.startswith("---\n")


def test_search_example_documents_prefers_structured_matches() -> None:
    matches = search_example_documents("fillet", sdk_package="sdk_hybrid", limit=3)

    assert matches
    assert matches[0].title == "Rounding Corners with Fillet"


def test_search_example_documents_returns_full_content() -> None:
    matches = search_example_documents("counterbore", sdk_package="sdk_hybrid", limit=1)

    assert len(matches) == 1
    assert "cboreHole" in matches[0].content
    assert matches[0].content.startswith("---\n")


def test_search_example_documents_default_limit_is_used() -> None:
    matches = search_example_documents("workplane", sdk_package="sdk_hybrid")

    assert len(matches) == 3


def test_search_example_documents_honors_explicit_large_limit() -> None:
    matches = search_example_documents("workplane", sdk_package="sdk_hybrid", limit=100)

    assert len(matches) > 3


def test_search_example_documents_returns_empty_list_for_no_match() -> None:
    assert search_example_documents("nonexistent-mechanism-token", sdk_package="sdk_hybrid") == []


def test_search_example_documents_excludes_incidental_single_body_hits() -> None:
    assert search_example_documents("articulated cap", sdk_package="sdk_hybrid", limit=10) == []


def test_search_example_documents_ignores_generic_cadquery_tag_matches() -> None:
    matches = search_example_documents(
        "bottle cadquery classic occ bottle",
        sdk_package="sdk_hybrid",
        limit=10,
    )

    assert [doc.title for doc in matches] == ["The Classic OCC Bottle"]


def test_search_example_documents_keeps_specific_body_api_queries() -> None:
    matches = search_example_documents("cboreHole", sdk_package="sdk_hybrid", limit=10)

    assert matches
    assert matches[0].title == "Making Counter-bored and Counter-sunk Holes"
    assert "A Parametric Bearing Pillow Block" in [doc.title for doc in matches]


def test_hybrid_example_corpus_titles_are_unique() -> None:
    docs = load_example_documents("sdk_hybrid")

    assert docs
    titles = [doc.title for doc in docs]
    assert len(titles) == len(set(titles))


def test_find_examples_tool_returns_expected_shape() -> None:
    async def _run() -> list[dict[str, object]]:
        tool = FindExamplesTool(sdk_package="sdk_hybrid")
        invocation = await tool.build({"query": "loft", "limit": 1})
        result = await invocation.execute()
        assert result.error is None
        assert isinstance(result.output, list)
        return result.output

    output = asyncio.run(_run())

    assert len(output) == 1
    assert output[0]["title"] == "Making Lofts"
    assert output[0]["path"] == "sdk/_examples/hybrid/making_lofts.md"
    assert "content" in output[0]


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
    assert len(result.conversation) == 2
    assert all(
        "<first_turn_tool_nudge>" not in str(message.get("content", ""))
        for message in result.conversation
    )
