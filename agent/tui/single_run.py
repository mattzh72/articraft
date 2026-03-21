"""Single run display - minimal log lines, no panels or live widgets."""

import threading
import time
from typing import Any, Optional

from rich.console import Console
from rich.text import Text

from agent.tui.formatters import (
    format_cost,
    format_duration,
    format_timestamp,
    format_tokens,
    format_tool_args,
    truncate_text,
)


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

    def _print_indented_block(self, text: str, *, style: str) -> None:
        """Print a multi-line block with consistent indentation."""
        lines = text.splitlines() or [text]
        for raw_line in lines:
            line = Text()
            line.append("            ", style="dim")
            line.append(raw_line, style=style)
            self.console.print(line)

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
        self._llm_wait_line_len = max(self._llm_wait_line_len, len(text))
        stream.write("\r" + text)
        stream.flush()

    def _clear_llm_wait_line(self) -> None:
        stream = getattr(self.console, "file", None)
        if stream is None:
            return
        if self._llm_wait_line_len <= 0:
            return
        stream.write("\r" + (" " * self._llm_wait_line_len) + "\r")
        stream.flush()
        self._llm_wait_line_len = 0

    def start_llm_wait(self) -> None:
        if not self.enabled:
            return
        if not self._supports_live_timer():
            return
        self.stop_llm_wait()

        stop_event = threading.Event()
        self._llm_wait_stop_event = stop_event
        start = time.monotonic()

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

    def add_llm_call(self, tokens: dict, cost: float, duration: float):
        if not self.enabled:
            return
        total = tokens.get("total_tokens", 0)
        self.total_tokens += total
        self.total_cost += cost

        line = Text()
        line.append("  llm     ", style="cyan")
        line.append(f"{format_tokens(total)} tokens", style="bold")
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

        line = Text()
        line.append("  tool    ", style="yellow")
        line.append(tool_name, style="bold")

        if success:
            line.append(" ✓", style="green")
        else:
            line.append(" ✗", style="red")

        line.append(f" {format_duration(duration)}", style="dim")
        self.console.print(line)

        # Print key args on next line, indented
        formatted = format_tool_args(args, max_items=2)
        if formatted:
            detail = Text()
            detail.append("            ", style="dim")
            detail.append(truncate_text(", ".join(formatted), 80), style="dim")
            self.console.print(detail)

        if result is not None:
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
        error: Optional[str] = None,
        warnings: Optional[list[str]] = None,
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

        if error:
            self._print_indented_block(error, style="red")

        if warnings:
            for w in warnings[:3]:
                warn = Text()
                warn.append("            ", style="dim")
                warn.append(truncate_text(w, 80), style="yellow")
                self.console.print(warn)
