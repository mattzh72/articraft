from __future__ import annotations

import asyncio
from pathlib import Path

from agent.tools.read_file import ReadFileTool
from agent.workspace_docs import build_virtual_workspace, load_sdk_docs_bundle


def test_load_sdk_docs_bundle_mounts_router_and_default_refs() -> None:
    repo_root = Path(__file__).resolve().parents[2]

    bundle = load_sdk_docs_bundle(repo_root, sdk_package="sdk")

    assert bundle.router.virtual_path == "docs/sdk/references/quickstart.md"
    assert bundle.default_read_virtual_paths() == (
        "docs/sdk/references/quickstart.md",
        "docs/sdk/references/probe-tooling.md",
        "docs/sdk/references/testing.md",
    )
    assert "docs/sdk/references/assets.md" in bundle.files_by_path
    assert "docs/sdk/references/geometry/mesh-geometry.md" in bundle.files_by_path
    assert "docs/sdk/references/geometry/panels-and-grilles.md" in bundle.files_by_path
    assert "docs/sdk/references/geometry/knobs-and-controls.md" in bundle.files_by_path
    assert "docs/sdk/references/geometry/wheels-and-tires.md" in bundle.files_by_path
    assert "docs/sdk/references/cadquery/overview.md" in bundle.files_by_path
    assert "docs/sdk/references/cadquery/gears.md" in bundle.files_by_path


def test_virtual_workspace_resolves_model_and_docs_paths(tmp_path: Path) -> None:
    repo_root = Path(__file__).resolve().parents[2]
    model_path = tmp_path / "model.py"
    model_path.write_text("line1\nline2\n", encoding="utf-8")

    workspace = build_virtual_workspace(
        repo_root,
        model_file_path=model_path,
        sdk_package="sdk",
    )

    model_file = workspace.resolve("model.py")
    docs_file = workspace.resolve("docs/sdk/references/quickstart.md")

    assert model_file.disk_path == model_path
    assert docs_file.disk_path is not None
    assert docs_file.disk_path.name == "00_quickstart.md"


def test_read_file_tool_reads_virtual_model_and_docs_paths(tmp_path: Path) -> None:
    async def _run() -> tuple[str, str]:
        repo_root = Path(__file__).resolve().parents[2]
        model_path = tmp_path / "model.py"
        model_path.write_text("alpha\nbeta\ngamma\n", encoding="utf-8")
        workspace = build_virtual_workspace(
            repo_root,
            model_file_path=model_path,
            sdk_package="sdk",
        )

        tool = ReadFileTool()

        model_invocation = await tool.build({"path": "model.py", "offset": 2, "limit": 2})
        model_invocation.bind_virtual_workspace(workspace)
        model_result = await model_invocation.execute()
        assert model_result.error is None

        docs_invocation = await tool.build(
            {"path": "docs/sdk/references/quickstart.md", "offset": 1, "limit": 120}
        )
        docs_invocation.bind_virtual_workspace(workspace)
        docs_result = await docs_invocation.execute()
        assert docs_result.error is None

        return str(model_result.output), str(docs_result.output)

    model_output, docs_output = asyncio.run(_run())

    assert model_output == "L2: beta\nL3: gamma"
    assert "Virtual Workspace" in docs_output
    assert "Import from `sdk` in `model.py`." in docs_output
    assert "geometry/knobs-and-controls.md" in docs_output


def test_read_file_tool_rejects_unknown_virtual_path(tmp_path: Path) -> None:
    async def _run() -> str | None:
        repo_root = Path(__file__).resolve().parents[2]
        model_path = tmp_path / "model.py"
        model_path.write_text("alpha\n", encoding="utf-8")
        workspace = build_virtual_workspace(
            repo_root,
            model_file_path=model_path,
            sdk_package="sdk",
        )

        tool = ReadFileTool()
        invocation = await tool.build({"path": "docs/nope.md"})
        invocation.bind_virtual_workspace(workspace)
        result = await invocation.execute()
        return result.error

    error = asyncio.run(_run())

    assert error == "File docs/nope.md not found"


def test_read_file_tool_reads_full_file_when_offset_and_limit_missing(tmp_path: Path) -> None:
    async def _run() -> str:
        repo_root = Path(__file__).resolve().parents[2]
        model_path = tmp_path / "model.py"
        model_path.write_text("alpha\nbeta\ngamma\n", encoding="utf-8")
        workspace = build_virtual_workspace(
            repo_root,
            model_file_path=model_path,
            sdk_package="sdk",
        )

        tool = ReadFileTool()
        invocation = await tool.build({"path": "model.py"})
        invocation.bind_virtual_workspace(workspace)
        result = await invocation.execute()
        assert result.error is None
        return str(result.output)

    model_output = asyncio.run(_run())
    assert model_output == "L1: alpha\nL2: beta\nL3: gamma"


def test_read_file_tool_reads_from_offset_to_eof_when_limit_missing(tmp_path: Path) -> None:
    async def _run() -> str:
        repo_root = Path(__file__).resolve().parents[2]
        model_path = tmp_path / "model.py"
        model_path.write_text("alpha\nbeta\ngamma\n", encoding="utf-8")
        workspace = build_virtual_workspace(
            repo_root,
            model_file_path=model_path,
            sdk_package="sdk",
        )

        tool = ReadFileTool()
        invocation = await tool.build({"path": "model.py", "offset": 2})
        invocation.bind_virtual_workspace(workspace)
        result = await invocation.execute()
        assert result.error is None
        return str(result.output)

    model_output = asyncio.run(_run())
    assert model_output == "L2: beta\nL3: gamma"
