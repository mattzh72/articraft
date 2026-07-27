from __future__ import annotations

from pathlib import Path

import pytest

from agent.compiler import _validate_experimental_sdk_policy
from experiments.agent_component_ablations.run import (
    CONDITIONS,
    _load_prompts,
    _single_pass_input,
    _strip_code_fence,
    evaluate_cell,
    system_prompt_for_condition,
)


def test_experiment_matrix_has_ten_prompts_and_five_conditions() -> None:
    _, prompts = _load_prompts()

    assert len(prompts) == 10
    assert [condition.id for condition in CONDITIONS] == [
        "full_baseline",
        "primitives_only",
        "no_compile_feedback",
        "no_example_retrieval",
        "single_pass",
    ]


def test_condition_prompts_match_the_requested_ablation_contracts() -> None:
    prompts = {condition.id: system_prompt_for_condition(condition) for condition in CONDITIONS}

    assert "compile_model" not in prompts["no_compile_feedback"]
    assert "TestContext" not in prompts["no_compile_feedback"]
    assert "find_examples" not in prompts["no_example_retrieval"]
    assert "sdk_primitives" in prompts["primitives_only"]
    assert "CadQuery, meshes" in prompts["primitives_only"]
    assert "<tools>" not in prompts["single_pass"]
    assert "find_examples" not in prompts["single_pass"]
    assert "raw Python only" in prompts["single_pass"]


def test_single_pass_input_contains_docs_and_scaffold_but_no_example_corpus() -> None:
    _, prompts = _load_prompts()

    content = _single_pass_input(prompts[0])

    assert "<object_request>" in content
    assert "<initial_scaffold>" in content
    assert "<sdk_documentation>" in content
    assert "sdk/_docs/common/00_quickstart.md" in content
    assert "sdk/_examples" not in content


def test_single_pass_code_fence_cleanup() -> None:
    assert _strip_code_fence("```python\nprint('ok')\n```") == "print('ok')\n"
    assert _strip_code_fence("print('ok')") == "print('ok')\n"


def test_primitives_policy_blocks_full_sdk_and_cadquery(tmp_path: Path) -> None:
    script = tmp_path / "main.py"
    script.write_text("from sdk import Box\nimport cadquery as cq\n", encoding="utf-8")

    with pytest.raises(ValueError, match="sdk_primitives policy violation"):
        _validate_experimental_sdk_policy(script, sdk_package="sdk_primitives")


def test_hidden_evaluation_exports_and_runs_standardized_qc(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("URDF_COMPILE_TIMEOUT_SECONDS", "0")
    script = tmp_path / "main.py"
    script.write_text(
        """from sdk import ArticulatedObject, Box, TestContext, TestReport


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box")
    model.part("body").visual(Box((0.2, 0.2, 0.2)))
    return model


def run_tests() -> TestReport:
    return TestContext(object_model).report()


object_model = build_object_model()
""",
        encoding="utf-8",
    )

    result = evaluate_cell(script, sdk_package="sdk")

    assert result["export_only"]["status"] == "success"
    assert result["native_full_compile"]["status"] == "success"
    assert result["standardized_baseline_qc"]["status"] == "success"
    assert (tmp_path / "model.urdf").exists()
