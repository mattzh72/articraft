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
from agent.tools.code_region import extract_editable_code
from agent.workspace_docs import VirtualWorkspace


class ReadFileParams(ToolParamsModel):
    """Parameters for read_file tool."""

    path: str
    offset: int | None = None
    limit: int | None = None


class ReadFileInvocation(BoundFileToolInvocation[ReadFileParams, str]):
    """Invocation for reading the target file."""

    def __init__(
        self,
        params: ReadFileParams,
        *,
        offset_provided: bool = False,
        limit_provided: bool = False,
        editable_model_only: bool = False,
    ):
        super().__init__(params)
        self.virtual_workspace: VirtualWorkspace | None = None
        self.offset_provided = offset_provided
        self.limit_provided = limit_provided
        self.editable_model_only = editable_model_only

    def bind_virtual_workspace(self, workspace: VirtualWorkspace) -> None:
        self.virtual_workspace = workspace

    def get_description(self) -> str:
        return (
            f"Read virtual file {self.params.path!r} "
            f"(offset={self.params.offset}, limit={self.params.limit})"
        )

    async def execute(self) -> ToolResult:
        try:
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

            if self.editable_model_only and resolved.virtual_path == "model.py":
                full_code = extract_editable_code(full_code)

            lines = full_code.splitlines()
            offset = self.params.offset or 1
            if self.offset_provided and self.params.offset is None:
                return ToolResult(error="offset must be >= 1")
            if self.offset_provided and self.params.offset < 1:
                return ToolResult(error="offset must be >= 1")
            if self.limit_provided and self.params.limit is None:
                return ToolResult(error="limit must be >= 1")
            if self.limit_provided and self.params.limit < 1:
                return ToolResult(error="limit must be >= 1")

            if not lines:
                if self.offset_provided and offset > 1:
                    return ToolResult(error="offset exceeds file length")
                return ToolResult(output="")

            if self.offset_provided and self.params.offset > len(lines):
                return ToolResult(error="offset exceeds file length")

            start = offset - 1
            if self.limit_provided:
                end = min(len(lines), start + self.params.limit)
            else:
                end = len(lines)
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

    def __init__(self, *, editable_model_only: bool = False) -> None:
        self.editable_model_only = editable_model_only
        if editable_model_only:
            description = (
                "Read an exact file from the virtual workspace with 1-indexed line numbers.\n\n"
                'Use `path="model.py"` for the editable artifact script; for Gemini this returns the '
                "current editable code section only. "
                "Use `path` under `docs/` for read-only SDK guidance and references.\n\n"
                "Returned lines are formatted as `L{line_number}: ...`.\n\n"
                "Use `offset` to choose the first line (1-indexed) and `limit` to cap the total number "
                "of returned lines. Omit both for a full read of the selected view. Omit `limit` with "
                "an explicit `offset` to read from that offset to EOF."
            )
        else:
            description = (
                "Read an exact file from the virtual workspace with 1-indexed line numbers.\n\n"
                'Use `path="model.py"` for the editable artifact script. '
                "Use `path` under `docs/` for read-only SDK guidance and references.\n\n"
                "Returned lines are formatted as `L{line_number}: ...`.\n\n"
                "Use `offset` to choose the first line (1-indexed) and `limit` to cap the total number "
                "of returned lines. Omit both for a full-file read. Omit `limit` with an explicit `offset` "
                "to read from that offset to EOF."
            )
        schema = make_tool_schema(
            name="read_file",
            description=description,
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
                    "description": ("Optional. 1-indexed line to start from. Omit for `1`."),
                },
                "limit": {
                    "type": "integer",
                    "description": ("Optional. Maximum number of lines to return. Omit for EOF."),
                },
            },
            required=["path"],
        )
        super().__init__("read_file", schema)

    async def build(self, params: dict) -> ReadFileInvocation:
        validated = validate_tool_params(ReadFileParams, params)
        invocation = ReadFileInvocation(
            validated,
            offset_provided="offset" in params and params["offset"] is not None,
            limit_provided="limit" in params and params["limit"] is not None,
            editable_model_only=self.editable_model_only,
        )
        return invocation
