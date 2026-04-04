"""
WriteCode tool - Replace the editable code section in one operation.
"""

from __future__ import annotations

import ast

import aiofiles

from agent.tools.base import (
    BaseDeclarativeTool,
    BoundFileToolInvocation,
    ToolParamsModel,
    ToolResult,
    make_tool_schema,
)
from agent.tools.code_region import (
    find_code_region,
    map_syntax_error_line_to_editable,
    replace_editable_code,
)


class WriteCodeParams(ToolParamsModel):
    """Parameters for write_code tool"""

    code: str


class WriteCodeInvocation(BoundFileToolInvocation[WriteCodeParams, str]):
    """Invocation for replacing editable code"""

    def get_description(self) -> str:
        preview = self.params.code[:50].replace("\n", "\\n")
        if len(self.params.code) > 50:
            preview += "..."
        return f"Rewrite editable code in current target file: '{preview}'"

    async def execute(self) -> ToolResult:
        try:
            if not self.file_path:
                return ToolResult(error="file_path is required")

            async with aiofiles.open(self.file_path, mode="r") as f:
                full_code = await f.read()

            region = find_code_region(full_code)
            if region.has_region:
                missing = self._missing_required_functions(self.params.code)
                if missing:
                    return ToolResult(
                        error=(
                            "write_code must include required top-level functions in the editable section: "
                            + ", ".join(missing)
                        )
                    )

            new_full_code = replace_editable_code(full_code, self.params.code)
            validation = self._validate_python_syntax(new_full_code, self.file_path or "<string>")

            async with aiofiles.open(self.file_path, mode="w") as f:
                await f.write(new_full_code)

            return ToolResult(output="Code rewritten successfully", compilation=validation)
        except FileNotFoundError:
            return ToolResult(error=f"File {self.file_path} not found")
        except Exception as exc:
            return ToolResult(error=f"Error writing code: {str(exc)}")

    def _missing_required_functions(self, editable_code: str) -> list[str]:
        required = ["build_object_model", "run_tests"]
        try:
            tree = ast.parse(editable_code)
        except SyntaxError:
            # Let syntax validation surface this with richer line mapping.
            return []

        defined = {
            node.name
            for node in tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        return [name for name in required if name not in defined]

    def _validate_python_syntax(self, full_code: str, filename: str) -> dict:
        try:
            compile(full_code, filename, "exec")
            return {
                "status": "success",
                "error": None,
            }
        except SyntaxError as exc:
            editable_line = map_syntax_error_line_to_editable(full_code, exc.lineno)
            if editable_line is not None and editable_line != exc.lineno:
                error_msg = f"Syntax error: {exc.msg} (editable line {editable_line}, full line {exc.lineno})"
            else:
                error_msg = f"Syntax error: {exc.msg} (line {exc.lineno})"
            return {
                "status": "error",
                "error": error_msg,
                "error_line": exc.lineno,
                "error_line_editable": editable_line,
            }
        except Exception as exc:
            return {
                "status": "error",
                "error": f"Validation error: {str(exc)}",
            }


class WriteCodeTool(BaseDeclarativeTool):
    """Tool for replacing editable code in a single call"""

    def __init__(self) -> None:
        schema = make_tool_schema(
            name="write_code",
            description=(
                "Replace the entire editable code section in one operation.\n\n"
                "Use this tool when repeated edit_code attempts fail due to exact old_string matching.\n\n"
                "Notes:\n"
                "- In scaffolded files, you should provide the editable section only (not scaffold imports/footer).\n"
                "- The harness validates Python syntax after writing.\n"
                "- In scaffolded files, your code must include top-level build_object_model() and run_tests()."
            ),
            parameters={
                "code": {
                    "type": "string",
                    "description": (
                        "Full replacement content for the editable code section. "
                        "In scaffolded files, include top-level build_object_model() and run_tests()."
                    ),
                }
            },
            required=["code"],
        )
        super().__init__("write_code", schema)

    async def build(self, params: dict) -> WriteCodeInvocation:
        validated = WriteCodeParams(**params)
        return WriteCodeInvocation(validated)
