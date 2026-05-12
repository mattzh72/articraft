"""
ApplyPatch tool - Apply a Codex-style patch to the current target file.
"""

from __future__ import annotations

from dataclasses import dataclass

import aiofiles

from agent.tools.base import (
    BaseDeclarativeTool,
    BoundFileToolInvocation,
    ToolParamsModel,
    ToolResult,
)
from agent.tools.code_region import map_syntax_error_line_to_editable

APPLY_PATCH_LARK_GRAMMAR = """start: begin_patch hunk+ end_patch
begin_patch: "*** Begin Patch" LF
end_patch: "*** End Patch" LF?

hunk: add_hunk | delete_hunk | update_hunk
add_hunk: "*** Add File: " filename LF add_line+
delete_hunk: "*** Delete File: " filename LF
update_hunk: "*** Update File: " filename LF change_move? change?

filename: /(.+)/
add_line: "+" /(.*)/ LF -> line

change_move: "*** Move to: " filename LF
change: (change_context | change_line)+ eof_line?
change_context: ("@@" | "@@ " /(.+)/) LF
change_line: ("+" | "-" | " ") /(.*)/ LF
eof_line: "*** End of File" LF

%import common.LF
"""


@dataclass(frozen=True)
class PatchHunk:
    header: str | None
    lines: list[str]


class ApplyPatchParams(ToolParamsModel):
    """Parameters for apply_patch tool."""

    input: str


class ApplyPatchInvocation(BoundFileToolInvocation[ApplyPatchParams, str]):
    """Invocation for applying Codex-style patches."""

    def get_description(self) -> str:
        preview = self.params.input[:80].replace("\n", "\\n")
        if len(self.params.input) > 80:
            preview += "..."
        return f"Apply patch to current target file: '{preview}'"

    async def execute(self) -> ToolResult:
        try:
            if not self.file_path:
                return ToolResult(error="file_path is required")
            if not self.params.input.strip():
                return ToolResult(error="input cannot be empty")

            async with aiofiles.open(self.file_path, mode="r") as f:
                full_code = await f.read()

            hunks = _parse_patch(self.params.input)
            new_code = _apply_hunks(full_code, hunks)
            validation = self._validate_python_syntax(new_code, self.file_path)

            async with aiofiles.open(self.file_path, mode="w") as f:
                await f.write(new_code)

            return ToolResult(
                output=f"Patch applied successfully ({len(hunks)} hunks)", compilation=validation
            )
        except FileNotFoundError:
            return ToolResult(error=f"File {self.file_path} not found")
        except ValueError as exc:
            return ToolResult(error=f"Invalid patch: {str(exc)}")
        except Exception as exc:
            return ToolResult(error=f"Error applying patch: {str(exc)}")

    def _validate_python_syntax(self, full_code: str, filename: str) -> dict:
        try:
            compile(full_code, filename, "exec")
            return {"status": "success", "error": None}
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
            return {"status": "error", "error": f"Validation error: {str(exc)}"}


class ApplyPatchFreeformTool(BaseDeclarativeTool):
    """Freeform custom tool variant of apply_patch (Codex parity)."""

    def __init__(self) -> None:
        schema = {
            "type": "custom",
            "name": "apply_patch",
            "description": (
                "Use the `apply_patch` tool to edit the current bound file. "
                "This is a FREEFORM tool, so do not wrap the patch in JSON. "
                "Single-file mode only: do not use `*** Add File`, `*** Delete File`, or `*** Move to`."
            ),
            "format": {
                "type": "grammar",
                "syntax": "lark",
                "definition": APPLY_PATCH_LARK_GRAMMAR,
            },
        }
        super().__init__("apply_patch", schema)

    async def build(self, params: dict) -> ApplyPatchInvocation:
        validated = ApplyPatchParams(**params)
        return ApplyPatchInvocation(validated)


def _parse_patch(patch_text: str) -> list[PatchHunk]:
    lines = patch_text.splitlines()
    if not lines:
        raise ValueError("patch is empty")
    if lines[0].strip() != "*** Begin Patch":
        raise ValueError("missing '*** Begin Patch' header")
    if lines[-1].strip() != "*** End Patch":
        raise ValueError("missing '*** End Patch' footer")

    hunks: list[PatchHunk] = []
    i = 1
    saw_update_file = False

    while i < len(lines) - 1:
        line = lines[i]
        if not line.strip():
            i += 1
            continue

        if line.startswith("*** Add File:"):
            raise ValueError("Add File is not supported in single-file mode")
        if line.startswith("*** Delete File:"):
            raise ValueError("Delete File is not supported in single-file mode")
        if line.startswith("*** Move to:"):
            raise ValueError("Move to is not supported in single-file mode")

        if line.startswith("*** Update File"):
            if saw_update_file:
                raise ValueError("only one Update File block is supported")
            saw_update_file = True
            i += 1

            if i < len(lines) - 1 and lines[i].startswith("*** Move to:"):
                raise ValueError("Move to is not supported in single-file mode")

            while i < len(lines) - 1:
                current = lines[i]
                if current.startswith("*** "):
                    break
                if not current.startswith("@@"):
                    raise ValueError("expected '@@' to start each hunk")

                header = current[2:].strip() or None
                i += 1
                hunk_lines: list[str] = []

                while i < len(lines) - 1:
                    hunk_line = lines[i]
                    if hunk_line == "*** End of File":
                        i += 1
                        break
                    if hunk_line.startswith("@@") or hunk_line.startswith("*** "):
                        break
                    if not hunk_line:
                        raise ValueError(
                            "empty hunk line is invalid; use a prefixed line (' ', '+', or '-')"
                        )
                    if hunk_line[0] not in {" ", "+", "-"}:
                        raise ValueError("hunk lines must start with ' ', '+', or '-'")
                    hunk_lines.append(hunk_line)
                    i += 1

                if not hunk_lines:
                    raise ValueError("hunk has no lines")
                hunks.append(PatchHunk(header=header, lines=hunk_lines))
            continue

        raise ValueError(f"unexpected patch line: {line}")

    if not saw_update_file:
        raise ValueError("missing '*** Update File' block")
    if not hunks:
        raise ValueError("no hunks found")
    return hunks


def _apply_hunks(full_code: str, hunks: list[PatchHunk]) -> str:
    lines = full_code.splitlines()
    had_trailing_newline = full_code.endswith("\n")
    search_start = 0

    for index, hunk in enumerate(hunks, start=1):
        old_block: list[str] = []
        new_block: list[str] = []

        for entry in hunk.lines:
            op = entry[0]
            text = entry[1:]
            if op in {" ", "-"}:
                old_block.append(text)
            if op in {" ", "+"}:
                new_block.append(text)

        if not old_block:
            raise ValueError(
                f"hunk {index} has no context/removal lines; ambiguous insertions are unsupported"
            )

        position = _find_subsequence(lines, old_block, search_start)
        if position is None:
            position = _find_subsequence(lines, old_block, 0)
        if position is None:
            raise ValueError(
                f"hunk {index} could not be matched in the target file with exact text"
            )

        lines = lines[:position] + new_block + lines[position + len(old_block) :]
        search_start = position + len(new_block)

    rewritten = "\n".join(lines)
    if had_trailing_newline and rewritten:
        rewritten += "\n"
    return rewritten


def _find_subsequence(haystack: list[str], needle: list[str], start: int) -> int | None:
    if not needle:
        return max(0, min(start, len(haystack)))
    max_start = len(haystack) - len(needle)
    if max_start < 0:
        return None
    for idx in range(max(0, start), max_start + 1):
        if haystack[idx : idx + len(needle)] == needle:
            return idx
    return None
