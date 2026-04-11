"""
ReadFile tool - Read an exact file from the virtual workspace with line numbers.
"""

from __future__ import annotations

import aiofiles

from agent.tools.base import (
    BaseDeclarativeTool,
    BoundFileToolInvocation,
    ToolParamsModel,
    ToolResult,
    make_tool_schema,
    validate_tool_params,
)
from agent.workspace_docs import VirtualWorkspace


class ReadFileParams(ToolParamsModel):
    """Parameters for read_file tool."""

    path: str
    offset: int = 1
    limit: int = 200


class ReadFileInvocation(BoundFileToolInvocation[ReadFileParams, str]):
    """Invocation for reading the target file."""

    def __init__(self, params: ReadFileParams):
        super().__init__(params)
        self.virtual_workspace: VirtualWorkspace | None = None

    def bind_virtual_workspace(self, workspace: VirtualWorkspace) -> None:
        self.virtual_workspace = workspace

    def get_description(self) -> str:
        return (
            f"Read virtual file {self.params.path!r} "
            f"(offset={self.params.offset}, limit={self.params.limit})"
        )

    async def execute(self) -> ToolResult:
        try:
            if self.params.offset < 1:
                return ToolResult(error="offset must be >= 1")
            if self.params.limit < 1:
                return ToolResult(error="limit must be >= 1")
            if self.virtual_workspace is None:
                return ToolResult(error="virtual workspace is not available")

            resolved = self.virtual_workspace.resolve(self.params.path)
            if resolved.content is not None:
                full_code = resolved.content
            elif resolved.disk_path is not None:
                async with aiofiles.open(resolved.disk_path, mode="r") as f:
                    full_code = await f.read()
            else:
                return ToolResult(error=f"Unable to resolve {self.params.path}")

            lines = full_code.splitlines()
            if not lines:
                if self.params.offset > 1:
                    return ToolResult(error="offset exceeds file length")
                return ToolResult(output="")

            if self.params.offset > len(lines):
                return ToolResult(error="offset exceeds file length")

            start = self.params.offset - 1
            end = min(len(lines), start + self.params.limit)
            formatted = [f"L{idx}: {lines[idx - 1]}" for idx in range(start + 1, end + 1)]
            return ToolResult(output="\n".join(formatted))
        except FileNotFoundError:
            return ToolResult(error=f"File {self.params.path} not found")
        except ValueError as exc:
            return ToolResult(error=str(exc))
        except Exception as exc:
            return ToolResult(error=f"Error reading file: {str(exc)}")


class ReadFileTool(BaseDeclarativeTool):
    """Tool for reading the current file with line numbers."""

    def __init__(self) -> None:
        schema = make_tool_schema(
            name="read_file",
            description=(
                "Read an exact file from the virtual workspace with 1-indexed line numbers.\n\n"
                'Use `path="model.py"` for the editable artifact script. '
                "Use `path` under `docs/` for read-only SDK guidance and references.\n\n"
                "Returned lines are formatted as `L{line_number}: ...`.\n\n"
                "Use offset/limit to read a slice before creating an apply_patch update."
            ),
            parameters={
                "path": {
                    "type": "string",
                    "description": (
                        "Virtual workspace path to read. Use `model.py` for the editable model "
                        "file and `docs/...` for mounted read-only SDK docs."
                    ),
                },
                "offset": {
                    "type": "integer",
                    "description": "1-indexed line to start from (default: 1).",
                },
                "limit": {
                    "type": "integer",
                    "description": "Maximum number of lines to return (default: 200).",
                },
            },
            required=["path"],
        )
        super().__init__("read_file", schema)

    async def build(self, params: dict) -> ReadFileInvocation:
        validated = validate_tool_params(ReadFileParams, params)
        return ReadFileInvocation(validated)
