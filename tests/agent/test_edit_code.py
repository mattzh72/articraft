from __future__ import annotations

import asyncio
from pathlib import Path

from agent.tools.edit_code import ReplaceTool


def _write_scaffold(script_path: Path, *, editable_code: str) -> None:
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                'DEFAULT_NAME = "draft_model"',
                "",
                "# >>> USER_CODE_START",
                editable_code.rstrip(),
                "# >>> USER_CODE_END",
                "",
                "UNCHANGED_SENTINEL = True",
                "",
            ]
        ),
        encoding="utf-8",
    )


async def _run_edit(script_path: Path, params: dict[str, object]):
    tool = ReplaceTool()
    invocation = await tool.build(params)
    invocation.bind_file_path(str(script_path))
    return await invocation.execute()


def test_replace_defaults_allow_multiple_to_false_when_omitted(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_scaffold(
        script_path,
        editable_code="""
def build_object_model():
    return "draft_model"


def run_tests():
    return None
""",
    )

    result = asyncio.run(
        _run_edit(
            script_path,
            {
                "old_string": '"draft_model"',
                "new_string": '"draft_model_v2"',
            },
        )
    )

    assert result.error is None
    assert result.compilation == {"status": "success", "error": None}
    updated = script_path.read_text(encoding="utf-8")
    assert 'DEFAULT_NAME = "draft_model"' in updated
    assert 'return "draft_model_v2"' in updated
    assert updated.count('"draft_model_v2"') == 1


def test_replace_treats_null_allow_multiple_as_default_false(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_scaffold(
        script_path,
        editable_code="""
def build_object_model():
    return "draft_model"


def run_tests():
    return None
""",
    )

    result = asyncio.run(
        _run_edit(
            script_path,
            {
                "old_string": '"draft_model"',
                "new_string": '"draft_model_v2"',
                "allow_multiple": None,
            },
        )
    )

    assert result.error is None
    assert result.compilation == {"status": "success", "error": None}
    updated = script_path.read_text(encoding="utf-8")
    assert 'DEFAULT_NAME = "draft_model"' in updated
    assert 'return "draft_model_v2"' in updated
    assert updated.count('"draft_model_v2"') == 1
