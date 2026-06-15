from __future__ import annotations

import asyncio
from pathlib import Path

from agent.tools.edit_code import ReplaceTool
from agent.tools.write_code import WriteFileTool


def _write_model(script_path: Path, *, body: str) -> None:
    script_path.write_text(
        "\n".join(
            [
                "from __future__ import annotations",
                "",
                'DEFAULT_NAME = "draft_model"',
                "",
                body.rstrip(),
            ]
        ),
        encoding="utf-8",
    )


async def _run_edit(script_path: Path, params: dict[str, object]):
    tool = ReplaceTool()
    invocation = await tool.build(params)
    invocation.bind_file_path(str(script_path))
    return await invocation.execute()


async def _run_write(script_path: Path, params: dict[str, object]):
    tool = WriteFileTool()
    invocation = await tool.build(params)
    invocation.bind_file_path(str(script_path))
    return await invocation.execute()


def test_replace_defaults_allow_multiple_to_false_when_omitted(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_model(
        script_path,
        body="""
def build_object_model():
    return "draft_model"


def run_tests():
    return None


object_model = build_object_model()
""",
    )

    result = asyncio.run(
        _run_edit(
            script_path,
            {
                "old_string": 'return "draft_model"',
                "new_string": 'return "draft_model_v2"',
            },
        )
    )

    assert result.error is None
    assert result.compilation == {"status": "success", "error": None}
    updated = script_path.read_text(encoding="utf-8")
    assert 'DEFAULT_NAME = "draft_model"' in updated
    assert 'return "draft_model_v2"' in updated
    assert "object_model = build_object_model()" in updated


def test_replace_treats_null_allow_multiple_as_default_false(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_model(
        script_path,
        body="""
def build_object_model():
    return "draft_model"


def run_tests():
    return None


object_model = build_object_model()
""",
    )

    result = asyncio.run(
        _run_edit(
            script_path,
            {
                "old_string": 'return "draft_model"',
                "new_string": 'return "draft_model_v2"',
                "allow_multiple": None,
            },
        )
    )

    assert result.error is None
    assert result.compilation == {"status": "success", "error": None}
    updated = script_path.read_text(encoding="utf-8")
    assert 'DEFAULT_NAME = "draft_model"' in updated
    assert 'return "draft_model_v2"' in updated
    assert "object_model = build_object_model()" in updated


def test_write_file_requires_complete_model_contract(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text("from __future__ import annotations\n", encoding="utf-8")

    result = asyncio.run(
        _run_write(
            script_path,
            {
                "content": """
from __future__ import annotations


def build_object_model():
    return "draft_model"


def run_tests():
    return None
""".strip()
            },
        )
    )

    assert result.error is not None
    assert "object_model = build_object_model()" in result.error
    assert script_path.read_text(encoding="utf-8") == "from __future__ import annotations\n"


def test_replace_empty_file_requires_complete_model_contract(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text("", encoding="utf-8")

    result = asyncio.run(
        _run_edit(
            script_path,
            {
                "old_string": "",
                "new_string": """
def build_object_model():
    return "draft_model"


def run_tests():
    return None
""".strip(),
            },
        )
    )

    assert result.error is not None
    assert "object_model = build_object_model()" in result.error
    assert script_path.read_text(encoding="utf-8") == ""


def test_replace_preserves_complete_model_contract(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    _write_model(
        script_path,
        body="""
def build_object_model():
    return "draft_model"


def run_tests():
    return None


object_model = build_object_model()
""",
    )
    original = script_path.read_text(encoding="utf-8")

    result = asyncio.run(
        _run_edit(
            script_path,
            {
                "old_string": "\n\nobject_model = build_object_model()",
                "new_string": "",
            },
        )
    )

    assert result.error is not None
    assert "object_model = build_object_model()" in result.error
    assert script_path.read_text(encoding="utf-8") == original


def test_write_file_rewrites_full_model_file(tmp_path: Path) -> None:
    script_path = tmp_path / "model.py"
    script_path.write_text("old = True\n", encoding="utf-8")
    content = """
from __future__ import annotations

from sdk import ArticulatedObject, TestContext, TestReport


def build_object_model() -> ArticulatedObject:
    return ArticulatedObject(name="draft_model_v2")


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    return ctx.report()


object_model = build_object_model()
""".strip()

    result = asyncio.run(_run_write(script_path, {"content": content}))

    assert result.error is None
    assert result.compilation == {"status": "success", "error": None}
    assert script_path.read_text(encoding="utf-8") == content
