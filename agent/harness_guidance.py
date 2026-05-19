from __future__ import annotations

import ast
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from agent.tools.base import ToolResult

MUTATING_TOOL_NAMES = frozenset({"apply_patch", "replace", "write_file"})
EXACT_ELEMENT_KEYWORDS = frozenset(
    {"elem_a", "elem_b", "positive_elem", "negative_elem", "inner_elem", "outer_elem"}
)
BASELINE_QC_CALLS = frozenset(
    {
        "check_model_valid",
        "check_mesh_assets_ready",
        "fail_if_isolated_parts",
        "warn_if_part_contains_disconnected_geometry_islands",
        "fail_if_parts_overlap_in_current_pose",
    }
)


@dataclass(frozen=True)
class CodeContractScan:
    visual_names: tuple[str, ...]
    referenced_exact_names: tuple[str, ...]
    baseline_qc_calls: tuple[str, ...]


def _sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _constant_string(node: ast.AST) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        value = node.value.strip()
        return value or None
    return None


def _call_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Attribute):
        return node.attr
    if isinstance(node, ast.Name):
        return node.id
    return None


class CodeContractVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.visual_names: set[str] = set()
        self.referenced_exact_names: set[str] = set()
        self.baseline_qc_calls: set[str] = set()
        self._function_stack: list[str] = []

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        self._function_stack.append(node.name)
        self.generic_visit(node)
        self._function_stack.pop()

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        self._function_stack.append(node.name)
        self.generic_visit(node)
        self._function_stack.pop()

    def visit_Call(self, node: ast.Call) -> None:
        call_name = _call_name(node.func)
        if call_name == "visual":
            for keyword in node.keywords:
                if keyword.arg != "name":
                    continue
                visual_name = _constant_string(keyword.value)
                if visual_name is not None:
                    self.visual_names.add(visual_name)

        in_run_tests = bool(self._function_stack and self._function_stack[-1] == "run_tests")
        if in_run_tests and call_name in BASELINE_QC_CALLS and not node.args and not node.keywords:
            self.baseline_qc_calls.add(f"{call_name}()")

        if in_run_tests:
            for keyword in node.keywords:
                if keyword.arg not in EXACT_ELEMENT_KEYWORDS:
                    continue
                elem_name = _constant_string(keyword.value)
                if elem_name is not None:
                    self.referenced_exact_names.add(elem_name)

        self.generic_visit(node)


def scan_code_contracts(text: str) -> CodeContractScan | None:
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return None

    visitor = CodeContractVisitor()
    visitor.visit(tree)
    return CodeContractScan(
        visual_names=tuple(sorted(visitor.visual_names)),
        referenced_exact_names=tuple(sorted(visitor.referenced_exact_names)),
        baseline_qc_calls=tuple(sorted(visitor.baseline_qc_calls)),
    )


class GuidanceInjector:
    def __init__(
        self,
        *,
        file_path: str,
        trace_writer: object | None,
        tool_call_name: Callable[[dict], str],
    ) -> None:
        self.file_path = file_path
        self.trace_writer = trace_writer
        self.tool_call_name = tool_call_name
        self._seen_tool_error_sigs: set[str] = set()
        self._seen_exact_geometry_contract_sigs: set[str] = set()
        self._seen_baseline_qc_guidance_sigs: set[str] = set()

    def reset(self) -> None:
        self._seen_exact_geometry_contract_sigs = set()
        self._seen_baseline_qc_guidance_sigs = set()

    def _append_guidance_message(self, conversation: list[dict], content: str) -> None:
        msg = {"role": "user", "content": content}
        conversation.append(msg)
        trace_writer = self.trace_writer
        if trace_writer:
            trace_writer.write_message(msg)

    def _scan_current_code_contracts(self) -> CodeContractScan | None:
        try:
            text = Path(self.file_path).read_text(encoding="utf-8")
        except OSError:
            return None
        return scan_code_contracts(text)

    def _maybe_inject_exact_geometry_contract_guidance(
        self,
        conversation: list[dict],
        *,
        scan: CodeContractScan,
    ) -> bool:
        missing_names = tuple(
            sorted(set(scan.referenced_exact_names).difference(scan.visual_names))
        )
        if not missing_names:
            return False

        sig = _sha256_text(json.dumps(missing_names))
        if sig in self._seen_exact_geometry_contract_sigs:
            return False
        self._seen_exact_geometry_contract_sigs.add(sig)

        joined = ", ".join(repr(name) for name in missing_names)
        self._append_guidance_message(
            conversation,
            "\n".join(
                [
                    "<exact_geometry_contract>",
                    (
                        "- Authored exact checks reference names that are not present in the "
                        f"current file: {joined}."
                    ),
                    "- These names became exact-geometry contracts when `ctx.expect_*` referenced them.",
                    (
                        "- Before more geometry edits, either restore those visual names or "
                        "update/remove the dependent exact checks in the same edit."
                    ),
                    "</exact_geometry_contract>",
                ]
            ),
        )
        return True

    def _maybe_inject_baseline_qc_guidance(
        self,
        conversation: list[dict],
        *,
        scan: CodeContractScan,
    ) -> bool:
        if not scan.baseline_qc_calls:
            return False

        sig = _sha256_text(json.dumps(scan.baseline_qc_calls))
        if sig in self._seen_baseline_qc_guidance_sigs:
            return False
        self._seen_baseline_qc_guidance_sigs.add(sig)

        joined = ", ".join(f"`{name}`" for name in scan.baseline_qc_calls)
        self._append_guidance_message(
            conversation,
            "\n".join(
                [
                    "<baseline_qc_guidance>",
                    (
                        "- `run_tests()` currently reintroduces compiler-owned baseline checks: "
                        f"{joined}."
                    ),
                    "- Leave baseline sanity/QC to `compile_model`.",
                    (
                        "- Keep `run_tests()` for prompt-specific exact checks, targeted pose "
                        "checks, and explicit allowances only."
                    ),
                    "</baseline_qc_guidance>",
                ]
            ),
        )
        return True

    def _turn_had_successful_mutation(
        self,
        tool_calls: list[dict],
        tool_results: list[ToolResult],
    ) -> bool:
        for tool_call, result in zip(tool_calls, tool_results, strict=False):
            if not result.is_success():
                continue
            if self.tool_call_name(tool_call) in MUTATING_TOOL_NAMES:
                return True
        return False

    def maybe_inject_code_contract_guidance(
        self,
        conversation: list[dict],
        *,
        tool_calls: list[dict],
        tool_results: list[ToolResult],
    ) -> None:
        if not self._turn_had_successful_mutation(tool_calls, tool_results):
            return
        scan = self._scan_current_code_contracts()
        if scan is None:
            return
        self._maybe_inject_exact_geometry_contract_guidance(conversation, scan=scan)
        self._maybe_inject_baseline_qc_guidance(conversation, scan=scan)

    def maybe_inject_edit_code_guidance(
        self,
        conversation: list[dict],
        *,
        tool_calls: list[dict],
        tool_results: list[ToolResult],
    ) -> None:
        for tool_call, result in zip(tool_calls, tool_results, strict=False):
            func = tool_call.get("function", {}) if isinstance(tool_call, dict) else {}
            func_name = func.get("name")
            if func_name != "replace":
                continue
            if not getattr(result, "error", None):
                continue
            if "Could not find the old_string in the code" not in result.error:
                continue

            sig = f"{func_name}_old_string_not_found"
            if sig in self._seen_tool_error_sigs:
                return
            self._seen_tool_error_sigs.add(sig)

            self._append_guidance_message(
                conversation,
                (
                    "<edit_retry_guidance>\n"
                    f"- Your last {func_name} failed because `old_string` did not match the file exactly.\n"
                    '- Do NOT guess. Call `read_file(path="model.py")` again, then pick a smaller exact snippet from the current editable code as `old_string` and retry.\n'
                    "- Keep edits surgical.\n"
                    "</edit_retry_guidance>"
                ),
            )
            return
