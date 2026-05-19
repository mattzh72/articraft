"""
Helpers for operating on the editable user-code region inside scaffolded files.
"""

from __future__ import annotations

from dataclasses import dataclass

USER_CODE_START_MARKER = "# >>> USER_CODE_START"
USER_CODE_END_MARKER = "# >>> USER_CODE_END"


@dataclass(frozen=True)
class CodeRegion:
    has_region: bool
    content_start: int = 0
    content_end: int = 0
    start_line: int = 1
    end_line: int = 1


def find_code_region(full_code: str) -> CodeRegion:
    """
    Locate the editable user-code region.

    If markers are missing/malformed, we treat the entire file as editable.
    """
    start_idx = full_code.find(USER_CODE_START_MARKER)
    if start_idx < 0:
        return CodeRegion(has_region=False, content_start=0, content_end=len(full_code))

    start_line_end = full_code.find("\n", start_idx)
    if start_line_end < 0:
        return CodeRegion(has_region=False, content_start=0, content_end=len(full_code))

    content_start = start_line_end + 1
    end_idx = full_code.find(USER_CODE_END_MARKER, content_start)
    if end_idx < 0:
        return CodeRegion(has_region=False, content_start=0, content_end=len(full_code))

    start_line = full_code.count("\n", 0, content_start) + 1
    end_line = full_code.count("\n", 0, end_idx) + 1
    return CodeRegion(
        has_region=True,
        content_start=content_start,
        content_end=end_idx,
        start_line=start_line,
        end_line=end_line,
    )


def extract_editable_code(full_code: str) -> str:
    region = find_code_region(full_code)
    return full_code[region.content_start : region.content_end]


def replace_editable_code(full_code: str, new_code: str) -> str:
    region = find_code_region(full_code)
    if region.has_region:
        normalized = new_code
        if normalized and not normalized.endswith("\n"):
            normalized += "\n"
        return full_code[: region.content_start] + normalized + full_code[region.content_end :]
    return new_code


def map_syntax_error_line_to_editable(full_code: str, line_no: int | None) -> int | None:
    """
    Convert full-file syntax error line number to editable-region line number.

    Returns None if the error line is unknown or outside the editable region.
    """
    if line_no is None:
        return None

    region = find_code_region(full_code)
    if not region.has_region:
        return line_no
    if line_no < region.start_line or line_no >= region.end_line:
        return None
    return line_no - region.start_line + 1
