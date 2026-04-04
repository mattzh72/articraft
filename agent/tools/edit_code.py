"""
EditCode tool - Make precise edits to code files using old_string/new_string
"""

import aiofiles

from agent.tools.base import (
    BaseDeclarativeTool,
    BoundFileToolInvocation,
    ToolParamsModel,
    ToolResult,
    make_tool_schema,
)
from agent.tools.code_region import (
    extract_editable_code,
    map_syntax_error_line_to_editable,
    replace_editable_code,
)


class EditCodeParams(ToolParamsModel):
    """Parameters for edit_code tool"""

    old_string: str
    new_string: str
    replace_all: bool = False


class EditCodeInvocation(BoundFileToolInvocation[EditCodeParams, str]):
    """Invocation for editing code file"""

    def get_description(self) -> str:
        preview = self.params.old_string[:50]
        if len(self.params.old_string) > 50:
            preview += "..."
        return f"Edit current target file: replacing '{preview}'"

    async def execute(self) -> ToolResult:
        try:
            if not self.file_path:
                return ToolResult(error="file_path is required")

            # Load current code
            async with aiofiles.open(self.file_path, mode="r") as f:
                full_code = await f.read()

            editable_code = extract_editable_code(full_code)

            if not self.params.old_string:
                if editable_code.strip():
                    return ToolResult(
                        error=(
                            "old_string cannot be empty unless the editable section is empty. "
                            "Please provide the exact string to replace."
                        )
                    )
                new_editable_code = self.params.new_string
                new_code = replace_editable_code(full_code, new_editable_code)
                validation = self._validate_python_syntax(new_code, self.file_path or "<string>")
                async with aiofiles.open(self.file_path, mode="w") as f:
                    await f.write(new_code)
                return ToolResult(output="Code edited successfully", compilation=validation)

            # Check if old_string exists in editable code
            if self.params.old_string not in editable_code:
                return ToolResult(
                    error="Could not find the old_string in the code. "
                    "Make sure the string matches exactly, including whitespace and indentation. "
                )

            # Count occurrences
            occurrences = editable_code.count(self.params.old_string)
            if occurrences > 1 and not self.params.replace_all:
                return ToolResult(
                    error=f"The old_string appears {occurrences} times in the code. "
                    f"Please provide a longer, unique string that appears only once, "
                    f"or use replace_all=true to replace all occurrences."
                )

            # Perform replacement
            if self.params.replace_all:
                new_editable_code = editable_code.replace(
                    self.params.old_string, self.params.new_string
                )
            else:
                new_editable_code = editable_code.replace(
                    self.params.old_string, self.params.new_string, 1
                )
            new_code = replace_editable_code(full_code, new_editable_code)

            # Validate Python syntax
            validation = self._validate_python_syntax(new_code, self.file_path or "<string>")

            # Save updated code
            async with aiofiles.open(self.file_path, mode="w") as f:
                await f.write(new_code)

            # Build success message
            if self.params.replace_all and occurrences > 1:
                success_msg = f"Code edited successfully (replaced {occurrences} occurrences)"
            else:
                success_msg = "Code edited successfully"

            return ToolResult(output=success_msg, compilation=validation)

        except FileNotFoundError:
            return ToolResult(error=f"File {self.file_path} not found")
        except Exception as e:
            return ToolResult(error=f"Error editing code: {str(e)}")

    def _validate_python_syntax(self, full_code: str, filename: str) -> dict:
        """
        Validate Python syntax by attempting to parse the code.
        """
        try:
            compile(full_code, filename, "exec")
            return {
                "status": "success",
                "error": None,
            }
        except SyntaxError as e:
            editable_line = map_syntax_error_line_to_editable(full_code, e.lineno)
            if editable_line is not None and editable_line != e.lineno:
                error_msg = (
                    f"Syntax error: {e.msg} (editable line {editable_line}, full line {e.lineno})"
                )
            else:
                error_msg = f"Syntax error: {e.msg} (line {e.lineno})"
            return {
                "status": "error",
                "error": error_msg,
                "error_line": e.lineno,
                "error_line_editable": editable_line,
            }
        except Exception as e:
            return {
                "status": "error",
                "error": f"Validation error: {str(e)}",
            }


class EditCodeTool(BaseDeclarativeTool):
    """Tool for making precise edits to code files"""

    def __init__(self):
        schema = make_tool_schema(
            name="edit_code",
            description=(
                "Edit a code file by replacing old_string with new_string.\n\n"
                "In scaffolded files, this edits only the editable user section.\n\n"
                "After every edit, the code is automatically validated for Python syntax. "
                "You will receive immediate feedback on whether the code is valid or contains syntax errors.\n\n"
                "Matching behavior:\n"
                "- old_string must match EXACTLY, including whitespace, indentation, and newlines\n"
                "- Exception: if the editable section is empty, old_string may be an empty string to insert initial code\n"
                "- By default, old_string must appear exactly once in the file\n"
                "- If old_string appears multiple times, you must either provide more context to make it unique, "
                "or set replace_all=true\n"
                "- If exact matching keeps failing, re-read the file and keep edits narrow\n\n"
                "Response format:\n"
                "On success, you receive:\n"
                "- output: 'Code edited successfully' (or 'replaced N occurrences' if replace_all=true)\n"
                "- compilation.status: 'success' or 'error'\n"
                "- compilation.error: error message with line number (if invalid)\n\n"
                "Possible errors:\n"
                "- 'Could not find the old_string in the code' - old_string doesn't match exactly\n"
                "- 'The old_string appears N times' - need longer context or replace_all=true\n"
                "- 'old_string cannot be empty' - must provide text to replace\n"
                "- compilation.error contains syntax errors with line numbers"
            ),
            parameters={
                "old_string": {
                    "type": "string",
                    "description": (
                        "The exact string to find and replace. Must match exactly including whitespace, "
                        "indentation, and newlines. May be empty only when the editable section is empty "
                        "and you are inserting the initial code. Must be unique unless replace_all=true."
                    ),
                },
                "new_string": {
                    "type": "string",
                    "description": (
                        "The replacement string. Can be empty to delete the old_string. "
                        "Must have valid Python syntax and proper indentation."
                    ),
                },
                "replace_all": {
                    "type": "boolean",
                    "description": (
                        "If false (default), old_string must appear exactly once. "
                        "If true, replaces all occurrences. Use for renaming variables or functions."
                    ),
                },
            },
            required=["old_string", "new_string", "replace_all"],
        )
        super().__init__("edit_code", schema)

    async def build(self, params: dict) -> EditCodeInvocation:
        validated = EditCodeParams(**params)
        return EditCodeInvocation(validated)
