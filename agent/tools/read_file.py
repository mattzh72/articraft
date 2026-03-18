"""
ReadFile tool - Read the current target file with line numbers.
"""

from __future__ import annotations

import aiofiles
from pydantic import BaseModel

from agent.tools.base import BaseDeclarativeTool, BaseToolInvocation, ToolResult, make_tool_schema


class ReadFileParams(BaseModel):
    """Parameters for read_file tool."""

    file_path: str | None = None
    offset: int = 1
    limit: int = 200


class ReadFileInvocation(BaseToolInvocation[ReadFileParams, str]):
    """Invocation for reading the target file."""

    def get_description(self) -> str:
        return (
            f"Read file {self.params.file_path} "
            f"(offset={self.params.offset}, limit={self.params.limit})"
        )

    async def execute(self) -> ToolResult:
        try:
            if not self.params.file_path:
                return ToolResult(error="file_path is required")
            if self.params.offset < 1:
                return ToolResult(error="offset must be >= 1")
            if self.params.limit < 1:
                return ToolResult(error="limit must be >= 1")

            async with aiofiles.open(self.params.file_path, mode="r") as f:
                full_code = await f.read()

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
            return ToolResult(error=f"File {self.params.file_path} not found")
        except Exception as exc:
            return ToolResult(error=f"Error reading file: {str(exc)}")


class ReadFileTool(BaseDeclarativeTool):
    """Tool for reading the current file with line numbers."""

    def __init__(self) -> None:
        schema = make_tool_schema(
            name="read_file",
            description=(
                "Read the current target file with 1-indexed line numbers.\n\n"
                "Returned lines are formatted as `L{line_number}: ...`.\n\n"
                "This run is bound to a single file by the harness, so do not pass file paths.\n"
                "Use offset/limit to read a slice before creating an apply_patch update."
            ),
            parameters={
                "offset": {
                    "type": "integer",
                    "description": "1-indexed line to start from (default: 1).",
                },
                "limit": {
                    "type": "integer",
                    "description": "Maximum number of lines to return (default: 200).",
                },
            },
            required=[],
        )
        super().__init__("read_file", schema)

    async def build(self, params: dict) -> ReadFileInvocation:
        validated = ReadFileParams(**params)
        return ReadFileInvocation(validated)
