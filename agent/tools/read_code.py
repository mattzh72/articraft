"""
ReadCode tool - Read a code file.
"""

import aiofiles

from agent.tools.base import (
    BaseDeclarativeTool,
    BoundFileToolInvocation,
    ToolParamsModel,
    ToolResult,
    make_tool_schema,
)
from agent.tools.code_region import extract_editable_code


class ReadCodeParams(ToolParamsModel):
    """Parameters for read_code tool"""


class ReadCodeInvocation(BoundFileToolInvocation[ReadCodeParams, str]):
    """Invocation for reading code file"""

    def get_description(self) -> str:
        return "Read code from the current target file"

    async def execute(self) -> ToolResult:
        try:
            if not self.file_path:
                return ToolResult(error="file_path is required")
            async with aiofiles.open(self.file_path, mode="r") as f:
                full_code = await f.read()
            return ToolResult(output=extract_editable_code(full_code))
        except FileNotFoundError:
            return ToolResult(error=f"File {self.file_path} not found")
        except Exception as exc:
            return ToolResult(error=f"Error reading file: {str(exc)}")


class ReadCodeTool(BaseDeclarativeTool):
    """Tool for reading a code file"""

    def __init__(self) -> None:
        schema = make_tool_schema(
            name="read_code",
            description=(
                "Read the current Python code file.\n\n"
                "Returns the editable user code as a plain string.\n\n"
                "Use this tool when you need the latest exact text for incremental edits with edit_code.\n\n"
                "Notes:\n"
                "- In scaffolded files, this returns only the editable user section.\n\n"
                "Returns: The editable user code as a string."
            ),
            parameters={},
            required=[],
        )
        super().__init__("read_code", schema)

    async def build(self, params: dict) -> ReadCodeInvocation:
        validated = ReadCodeParams(**params)
        return ReadCodeInvocation(validated)
