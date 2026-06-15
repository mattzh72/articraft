"""
WriteCode tool - Replace model.py in one operation.
"""

from __future__ import annotations

import aiofiles

from agent.tools.base import (
    BaseDeclarativeTool,
    BoundFileToolInvocation,
    ToolParamsModel,
    ToolResult,
    make_tool_schema,
)
from agent.tools.model_contract import missing_required_model_contract


class WriteCodeParams(ToolParamsModel):
    """Parameters for write_code tool"""

    code: str


class WriteCodeInvocation(BoundFileToolInvocation[WriteCodeParams, str]):
    """Invocation for replacing the full model file."""

    def get_description(self) -> str:
        preview = self.params.code[:50].replace("\n", "\\n")
        if len(self.params.code) > 50:
            preview += "..."
        return f"Rewrite current model.py file: '{preview}'"

    async def execute(self) -> ToolResult:
        try:
            if not self.file_path:
                return ToolResult(error="file_path is required")

            missing = missing_required_model_contract(self.params.code)
            if missing:
                return ToolResult(
                    error=(
                        "write_file must include the complete top-level model.py contract: "
                        + ", ".join(missing)
                    )
                )

            validation = self._validate_python_syntax(
                self.params.code, self.file_path or "<string>"
            )

            async with aiofiles.open(self.file_path, mode="w", encoding="utf-8") as f:
                await f.write(self.params.code)

            return ToolResult(output="model.py rewritten successfully", compilation=validation)
        except FileNotFoundError:
            return ToolResult(error=f"File {self.file_path} not found")
        except Exception as exc:
            return ToolResult(error=f"Error writing code: {str(exc)}")

    def _validate_python_syntax(self, full_code: str, filename: str) -> dict:
        try:
            compile(full_code, filename, "exec")
            return {
                "status": "success",
                "error": None,
            }
        except SyntaxError as exc:
            error_msg = f"Syntax error: {exc.msg} (line {exc.lineno})"
            return {
                "status": "error",
                "error": error_msg,
                "error_line": exc.lineno,
            }
        except Exception as exc:
            return {
                "status": "error",
                "error": f"Validation error: {str(exc)}",
            }


class WriteFileParams(ToolParamsModel):
    """Parameters for Gemini-style write_file alias."""

    content: str
    path: str | None = None


class WriteFileInvocation(BoundFileToolInvocation[WriteFileParams, str]):
    """Invocation for full model.py rewrites under the official-style name."""

    def get_description(self) -> str:
        preview = self.params.content[:50].replace("\n", "\\n")
        if len(self.params.content) > 50:
            preview += "..."
        return f"Write current model.py file: '{preview}'"

    async def execute(self) -> ToolResult:
        mapped = WriteCodeParams(code=self.params.content)
        invocation = WriteCodeInvocation(mapped)
        invocation.bind_file_path(self.file_path or "")
        return await invocation.execute()


class WriteFileTool(BaseDeclarativeTool):
    """Gemini-style write_file tool scoped to the full model file."""

    def __init__(self) -> None:
        schema = make_tool_schema(
            name="write_file",
            description=(
                "Replace the entire current bound `model.py` artifact.\n\n"
                "This is the Gemini-style full rewrite tool. It writes the full visible model script, "
                "including imports and `object_model = build_object_model()`.\n\n"
                "Use this when a targeted `replace` would be awkward or when you intend to rewrite the "
                "whole model file from scratch."
            ),
            parameters={
                "content": {
                    "type": "string",
                    "description": (
                        "Full replacement content for model.py. Include imports, top-level "
                        "build_object_model(), run_tests(), and object_model = build_object_model()."
                    ),
                },
                "path": {
                    "type": "string",
                    "description": (
                        "Optional virtual path for parity with Gemini CLI. "
                        'If provided, use `"model.py"`.'
                    ),
                },
            },
            required=["content"],
        )
        super().__init__("write_file", schema)

    async def build(self, params: dict) -> WriteFileInvocation:
        validated = WriteFileParams(**params)
        if validated.path not in {None, "model.py"}:
            raise ValueError("write_file only supports path='model.py' in this harness")
        return WriteFileInvocation(validated)
