"""Single run display - minimal log lines, no panels or live widgets."""

import logging
import threading
import time
from typing import Any, Optional

from rich.console import Console
from rich.text import Text

from agent.tui.formatters import (
    format_context_pressure,
    format_cost,
    format_duration,
    format_timestamp,
    format_tokens,
    format_tool_args,
    truncate_text,
)

_ACTIVE_WAIT_DISPLAY: "SingleRunDisplay | None" = None
_ACTIVE_WAIT_DISPLAY_LOCK = threading.Lock()


class LLMWaitAwareStreamHandler(logging.StreamHandler):
    """Clears any active live wait line before emitting a log record."""

    def emit(self, record: logging.LogRecord) -> None:
        display = _active_wait_display()
        if display is not None:
            display.prepare_for_external_output()
        super().emit(record)


def _active_wait_display() -> "SingleRunDisplay | None":
    with _ACTIVE_WAIT_DISPLAY_LOCK:
        return _ACTIVE_WAIT_DISPLAY


def _set_active_wait_display(display: "SingleRunDisplay | None") -> None:
    global _ACTIVE_WAIT_DISPLAY
    with _ACTIVE_WAIT_DISPLAY_LOCK:
        _ACTIVE_WAIT_DISPLAY = display


class SingleRunDisplay:
    """Prints clean, indented log lines as the agent runs.

    Output style (like cargo build / esbuild):

        Turn 1/100 ──────────────────────────────
          llm     39.2K tokens  $0.0047  4.1s
          tool    write_code ✓ 0.3s
                    code: "def build_object_model()..."
          compile ✓ 3.2s

        Turn 2/100 ──────────────────────────────
          llm     3.1K tokens  $0.0061  5.1s
          tool    edit_code ✗ old_string not found
          compile ✓ 2.8s

        Done  2 turns  $0.0108  42.3K tokens  12.4s
    """

    def __init__(
        self,
        console: Console,
        model_id: str,
        thinking_level: str,
        max_turns: int,
        enabled: bool = True,
    ):
        self.console = console
        self.model_id = model_id
        self.thinking_level = thinking_level
        self.max_turns = max_turns
        self.enabled = enabled

        self.start_time = time.time()
        self.total_cost = 0.0
        self.total_tokens = 0
        self.current_turn = 0
        self._llm_wait_stop_event: Optional[threading.Event] = None
        self._llm_wait_thread: Optional[threading.Thread] = None
        self._llm_wait_line_len = 0
        self._llm_wait_lock = threading.Lock()

    def _print_indented_block(self, text: str, *, style: str) -> None:
        """Print a multi-line block with consistent indentation."""
        lines = text.splitlines() or [text]
        for raw_line in lines:
            line = Text()
            line.append("            ", style="dim")
            line.append(raw_line, style=style)
            self.console.print(line)

    def _find_examples_titles(self, result: Any) -> list[str]:
        """Extract returned example titles from a successful find_examples result."""
        if not isinstance(result, list):
            return []
        titles: list[str] = []
        for item in result:
            if not isinstance(item, dict):
                continue
            title = item.get("title")
            if isinstance(title, str) and title.strip():
                label = title.strip()
                if item.get("match_quality") == "weakly_relevant":
                    label = f"{label} [weakly relevant]"
                titles.append(label)
        return titles

    def _extract_tagged_block(self, text: str, tag: str) -> str | None:
        start_token = f"<{tag}>"
        end_token = f"</{tag}>"
        start = text.find(start_token)
        if start < 0:
            return None
        start += len(start_token)
        end = text.find(end_token, start)
        if end < 0:
            return None
        return text[start:end].strip()

    def _compile_block_with_heading(self, tag: str, block: str | None) -> str | None:
        if not isinstance(block, str):
            return None
        cleaned = block.strip()
        if not cleaned:
            return None
        headings = {
            "failures": "Failures (blocking):",
            "warnings": "Warnings (non-blocking):",
            "notes": "Notes (informational):",
            "response_rules": "Suggested next steps:",
        }
        heading = headings.get(tag)
        if heading is None or cleaned.startswith(heading):
            return cleaned
        return f"{heading}\n{cleaned}"

    def _parse_compile_output(
        self,
        result: Any,
        compilation: Optional[dict[str, Any]],
    ) -> tuple[bool, str | None, bool]:
        compile_ok = compilation is not None and compilation.get("status") == "success"
        if not isinstance(result, str):
            if result is None and not compile_ok and compilation and compilation.get("error"):
                return compile_ok, str(compilation["error"]), False
            return compile_ok, None if result is None else str(result), False

        has_warnings = bool(self._extract_tagged_block(result, "warnings"))
        blocks = [
            self._compile_block_with_heading(tag, self._extract_tagged_block(result, tag))
            for tag in ("summary", "failures", "warnings", "notes", "response_rules")
        ]
        visible_blocks = [
            block.strip() for block in blocks if isinstance(block, str) and block.strip()
        ]
        if visible_blocks:
            return compile_ok, "\n\n".join(visible_blocks), has_warnings

        cleaned = result.strip()
        if cleaned:
            return compile_ok, cleaned, has_warnings
        if not compile_ok and compilation and compilation.get("error"):
            return compile_ok, str(compilation["error"]), False
        return compile_ok, None, False

    def _supports_live_timer(self) -> bool:
        stream = getattr(self.console, "file", None)
        if stream is None:
            return False
        isatty = getattr(stream, "isatty", None)
        if not callable(isatty):
            return False
        try:
            return bool(isatty())
        except Exception:
            return False

    def _render_llm_wait_line(self, frame: str, elapsed_seconds: float) -> None:
        stream = getattr(self.console, "file", None)
        if stream is None:
            return
        text = f"  llm     {frame} waiting {format_timestamp(elapsed_seconds)}"
        with self._llm_wait_lock:
            self._llm_wait_line_len = max(self._llm_wait_line_len, len(text))
            stream.write("\r" + text)
            stream.flush()

    def _clear_llm_wait_line(self, *, reset_length: bool = True) -> None:
        stream = getattr(self.console, "file", None)
        if stream is None:
            return
        with self._llm_wait_lock:
            if self._llm_wait_line_len <= 0:
                return
            stream.write("\r" + (" " * self._llm_wait_line_len) + "\r")
            stream.flush()
            if reset_length:
                self._llm_wait_line_len = 0

    def prepare_for_external_output(self) -> None:
        """Clear the live wait line so external output starts on a fresh row."""
        self._clear_llm_wait_line(reset_length=False)

    def start_llm_wait(self) -> None:
        if not self.enabled:
            return
        if not self._supports_live_timer():
            return
        self.stop_llm_wait()

        stop_event = threading.Event()
        self._llm_wait_stop_event = stop_event
        start = time.monotonic()
        _set_active_wait_display(self)

        def _run() -> None:
            frames = ("|", "/", "-", "\\")
            idx = 0
            while not stop_event.is_set():
                self._render_llm_wait_line(frames[idx % len(frames)], time.monotonic() - start)
                idx += 1
                stop_event.wait(0.2)

        self._llm_wait_thread = threading.Thread(target=_run, daemon=True)
        self._llm_wait_thread.start()

    def stop_llm_wait(self) -> None:
        stop_event = self._llm_wait_stop_event
        worker = self._llm_wait_thread
        self._llm_wait_stop_event = None
        self._llm_wait_thread = None
        if stop_event is not None:
            stop_event.set()
        if worker is not None and worker.is_alive():
            worker.join(timeout=0.5)
        if _active_wait_display() is self:
            _set_active_wait_display(None)
        self._clear_llm_wait_line()

    # ── lifecycle ──────────────────────────────────────────────

    def start(self):
        """Print a one-line run header."""
        if not self.enabled:
            return
        line = Text()
        line.append("run ", style="dim")
        line.append(self.model_id, style="bold")
        line.append(f"  thinking={self.thinking_level}", style="dim")
        line.append(f"  max_turns={self.max_turns}", style="dim")
        self.console.print(line)

    def stop(self):
        """Print a summary line."""
        if not self.enabled:
            return
        self.stop_llm_wait()
        elapsed = time.time() - self.start_time
        self.console.print()
        line = Text()
        line.append("done ", style="bold green")
        line.append(f"{self.current_turn} turns", style="bold")
        line.append("  ")
        line.append(format_cost(self.total_cost), style="green")
        line.append("  ")
        line.append(f"{format_tokens(self.total_tokens)} tokens", style="blue")
        line.append("  ")
        line.append(format_duration(elapsed), style="dim")
        self.console.print(line)

    # ── turn boundaries ───────────────────────────────────────

    def start_turn(self, turn_number: int):
        if not self.enabled:
            return
        self.current_turn = turn_number
        self.console.print()
        rule = Text()
        rule.append(f"Turn {turn_number}/{self.max_turns}", style="bold")
        rule.append(" " + "─" * 40, style="dim")
        self.console.print(rule)

    def end_turn(self, success: bool, error: Optional[str] = None):
        if not self.enabled:
            return
        if error:
            self.console.print(Text(f"  error   {error}", style="red"))

    # ── events ────────────────────────────────────────────────

    def add_llm_call(
        self,
        tokens: dict,
        cost: float,
        duration: float,
        context_pressure: dict[str, Any] | None = None,
    ):
        if not self.enabled:
            return
        total = tokens.get("total_tokens", 0)
        self.total_tokens += total
        self.total_cost += cost

        pressure = context_pressure
        if pressure is None:
            pressure = {
                "prompt_tokens": tokens.get("prompt_tokens"),
                "cached_tokens": tokens.get("cached_tokens"),
                "output_tokens": tokens.get("candidates_tokens"),
                "total_tokens": tokens.get("total_tokens"),
            }
        context_text = format_context_pressure(pressure)

        line = Text()
        line.append("  llm     ", style="cyan")
        line.append(f"{format_tokens(total)} tokens", style="bold")
        if context_text:
            line.append("  ")
            line.append(context_text, style="blue")
        line.append(f"  {format_cost(cost)}", style="green")
        line.append(f"  {format_duration(duration)}", style="dim")
        self.console.print(line)

    def add_thinking_summary(self, summary: str):
        if not self.enabled:
            return
        cleaned = (summary or "").strip()
        if not cleaned:
            return
        line = Text()
        line.append("  think   ", style="magenta")
        line.append("summary", style="bold")
        self.console.print(line)
        self._print_indented_block(cleaned, style="magenta")

    def add_compaction_event(
        self,
        *,
        trigger: str,
        estimated_saved_next_input_tokens: int | None,
        billed_cost: float | None,
        previous_response_id_cleared: bool,
        usage_total_tokens: int | None = None,
        estimate_error: str | None = None,
    ) -> None:
        if not self.enabled:
            return

        if billed_cost and billed_cost > 0:
            self.total_cost += billed_cost
        if usage_total_tokens and usage_total_tokens > 0:
            self.total_tokens += usage_total_tokens

        line = Text()
        line.append("  compact ", style="magenta")
        line.append(trigger.replace("_", " "), style="bold")
        if estimated_saved_next_input_tokens is not None:
            line.append("  saved≈", style="dim")
            line.append(format_tokens(estimated_saved_next_input_tokens), style="bold blue")
        else:
            line.append("  saved=", style="dim")
            line.append("?", style="yellow")
        line.append("  billed=", style="dim")
        line.append(format_cost(billed_cost or 0.0), style="green")
        line.append("  prev=", style="dim")
        line.append(
            "cleared" if previous_response_id_cleared else "kept",
            style="bold green" if previous_response_id_cleared else "bold yellow",
        )
        self.console.print(line)

        if estimate_error:
            note = Text()
            note.append("            ", style="dim")
            note.append(truncate_text(f"estimate: {estimate_error}", 80), style="yellow")
            self.console.print(note)

    def add_maintenance_event(
        self,
        event: dict[str, Any],
        *,
        billed_cost: float | None = None,
        usage_total_tokens: int | None = None,
    ) -> None:
        if not self.enabled:
            return

        kind = str(event.get("kind") or event.get("type") or "maintenance")
        if billed_cost and billed_cost > 0:
            self.total_cost += billed_cost
        if usage_total_tokens and usage_total_tokens > 0:
            self.total_tokens += usage_total_tokens

        line = Text()
        line.append("  maint   ", style="magenta")
        line.append(kind.replace("_", " "), style="bold")
        cache_name = event.get("cache_name")
        reason = event.get("reason")
        if isinstance(cache_name, str) and cache_name:
            line.append("  cache=", style="dim")
            line.append(truncate_text(cache_name, 28), style="blue")
        elif isinstance(reason, str) and reason:
            line.append("  reason=", style="dim")
            line.append(reason, style="yellow")
        if billed_cost is not None:
            line.append("  billed=", style="dim")
            line.append(format_cost(billed_cost), style="green")
        self.console.print(line)

    def add_tool_call(
        self,
        tool_name: str,
        args,
        success: bool,
        duration: float,
        result: Any = None,
        compilation: Optional[dict[str, Any]] = None,
        error: Optional[str] = None,
    ):
        if not self.enabled:
            return

        if tool_name == "compile_model":
            compile_success, compile_details, compile_has_warnings = self._parse_compile_output(
                result,
                compilation,
            )
            self.add_compile_result(
                success=compile_success,
                duration=duration,
                details=compile_details,
                has_warnings=compile_has_warnings,
            )
            return

        line = Text()
        line.append("  tool    ", style="yellow")
        line.append(tool_name, style="bold")

        if success:
            line.append(" ✓", style="green")
        else:
            line.append(" ✗", style="red")

        line.append(f" {format_duration(duration)}", style="dim")
        self.console.print(line)

        # Print all formatted args on indented lines so the TUI does not
        # collapse multi-argument tool calls into "... (N more)" summaries.
        formatted = format_tool_args(args)
        if formatted:
            self._print_indented_block("\n".join(formatted), style="dim")

        example_titles = self._find_examples_titles(result) if tool_name == "find_examples" else []
        if example_titles:
            titles_block = "titles:\n" + "\n".join(f"- {title}" for title in example_titles)
            self._print_indented_block(titles_block, style="green")
        elif result is not None:
            result_line = Text()
            result_line.append("            ", style="dim")
            result_line.append(truncate_text(f"result: {result}", 120), style="green")
            self.console.print(result_line)

        if compilation:
            status = compilation.get("status", "unknown")
            comp_line = Text()
            comp_line.append("            ", style="dim")
            comp_line.append(
                truncate_text(f"compilation.status: {status}", 120),
                style="green" if status == "success" else "red",
            )
            self.console.print(comp_line)
            comp_error = compilation.get("error")
            if comp_error:
                comp_error_line = Text()
                comp_error_line.append("            ", style="dim")
                comp_error_line.append(
                    truncate_text(f"compilation.error: {comp_error}", 120),
                    style="red",
                )
                self.console.print(comp_error_line)

        if error:
            err = Text()
            err.append("            ", style="dim")
            err.append(truncate_text(error, 80), style="red")
            self.console.print(err)

    def add_compile_result(
        self,
        success: bool,
        duration: float,
        details: Optional[str] = None,
        has_warnings: bool = False,
    ):
        if not self.enabled:
            return

        line = Text()
        line.append("  compile ", style="blue")
        if success:
            line.append("✓", style="green")
        else:
            line.append("✗", style="red")
        line.append(f" {format_duration(duration)}", style="dim")
        self.console.print(line)

        if details:
            if success and has_warnings:
                detail_style = "yellow"
            else:
                detail_style = "green" if success else "red"
            self._print_indented_block(details, style=detail_style)
