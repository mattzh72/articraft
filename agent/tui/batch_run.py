"""Batch run display — minimal log lines, no panels or live widgets."""

import re
import time
from typing import Optional

from rich.console import Console
from rich.text import Text

from agent.tui.formatters import format_cost, format_duration, format_tokens, truncate_text


class BatchRunDisplay:
    """Prints clean log lines for batch experiment progress.

    Output style:

        batch  stand_mixers  model=gemini-3-flash-preview  concurrency=5  20 runs

        start   0001_stand-mixer
        done    0001_stand-mixer  ✓  $0.0234  2m 15s
        start   0003_office-chair
        done    0003_office-chair ✗  timeout after 600s
        ...

        batch done  18/20 succeeded  $0.456  12m 30s
    """

    def __init__(
        self,
        console: Console,
        experiment_name: str,
        total_runs: int,
        concurrency: int,
        model_id: str,
        enabled: bool = True,
    ):
        self.console = console
        self.experiment_name = experiment_name
        self.total_runs = total_runs
        self.concurrency = concurrency
        self.model_id = model_id
        self.enabled = enabled

        self.start_time = time.time()
        self.completed = 0
        self.session_completed = 0
        self.successes = 0
        self.total_cost = 0.0
        self.runs: dict[str, float] = {}  # slug -> start_time
        self.run_numbers: dict[str, int] = {}
        self.run_turns: dict[str, int] = {}
        self.run_metadata: dict[str, dict[str, str]] = {}
        self._next_run_number = 1
        self._checkpoint_every = self._compute_checkpoint_interval()

    def _print_indented_block(self, text: str, *, style: str) -> None:
        lines = text.splitlines() or [text]
        for raw_line in lines:
            err = Text()
            err.append("            ", style="dim")
            err.append(raw_line, style=style)
            self.console.print(err)

    def _print_event(
        self, label: str, message: str, *, label_style: str, message_style: str
    ) -> None:
        line = Text()
        line.append(f"  {label:<8}", style=label_style)
        line.append(message, style=message_style)
        self.console.print(line)

    def _compute_checkpoint_interval(self) -> int:
        if self.total_runs <= 10:
            return 2
        if self.total_runs <= 50:
            return 5
        return 10

    def _print_progress_rule(self):
        rule = Text()
        rule.append(f"Batch {self.completed}/{self.total_runs}", style="bold cyan")
        rule.append(" " + "─" * 40, style="dim")
        self.console.print(rule)

    def _print_checkpoint(self):
        elapsed = time.time() - self.start_time
        active = len(self.runs)
        queued = max(0, self.total_runs - self.completed - active)
        failures = self.completed - self.successes

        completed_per_min = (
            (self.session_completed / elapsed * 60.0)
            if elapsed > 0 and self.session_completed > 0
            else 0.0
        )
        if self.session_completed > 0 and elapsed > 0:
            remaining = active + queued
            eta_seconds = (remaining / self.session_completed) * elapsed
            eta_text = format_duration(eta_seconds)
        else:
            eta_text = "?"

        line = Text()
        line.append("  batch   ", style="bold cyan")
        line.append("ok=", style="dim")
        line.append(f"{self.successes}", style="bold green")
        line.append("  ")
        line.append("fail=", style="dim")
        line.append(f"{failures}", style="bold red" if failures > 0 else "bold green")
        line.append("  ")
        line.append("active=", style="dim")
        line.append(f"{active}", style="bold cyan" if active > 0 else "dim")
        line.append("  ")
        line.append("queued=", style="dim")
        line.append(f"{queued}", style="bold yellow" if queued > 0 else "dim")
        line.append("  ")
        line.append("rate=", style="dim")
        line.append(
            f"{completed_per_min:.2f} runs/min",
            style="bold blue" if completed_per_min > 0 else "dim",
        )
        line.append("  ")
        line.append("eta=", style="dim")
        line.append(eta_text, style="bold magenta" if eta_text != "?" else "yellow")
        self.console.print(line)

    @property
    def failures(self) -> int:
        return self.completed - self.successes

    def _assign_run_number(self, slug: str) -> int:
        match = re.match(r"^(\d+)_", slug)
        if match:
            # Slugs are 0-based indices, but run numbers are user-facing 1-based.
            return int(match.group(1)) + 1
        run_number = self._next_run_number
        self._next_run_number += 1
        return run_number

    def _run_label(self, slug: str) -> str:
        run_number = self.run_numbers.get(slug)
        if run_number is None:
            run_number = self._assign_run_number(slug)
            self.run_numbers[slug] = run_number
        return f"#{run_number:03d}/{self.total_runs}"

    def _metadata_text(self, slug: str) -> str:
        metadata = self.run_metadata.get(slug) or {}
        parts = []
        row_id = metadata.get("row_id")
        provider = metadata.get("provider")
        model_id = metadata.get("model_id")
        thinking_level = metadata.get("thinking_level")
        if row_id:
            parts.append(row_id)
        if provider:
            parts.append(provider)
        if model_id:
            parts.append(model_id)
        if thinking_level:
            parts.append(f"thinking={thinking_level}")
        return "  " + "  ".join(parts) if parts else ""

    def start(self):
        if not self.enabled:
            return
        line = Text()
        line.append("batch ", style="bold cyan")
        line.append(self.experiment_name, style="bold white on blue")
        line.append(f"  model={self.model_id}", style="dim")
        line.append(f"  row_concurrency={self.concurrency}", style="dim")
        line.append(f"  {self.total_runs} runs", style="dim")
        self.console.print(line)
        self.console.print()
        self._print_progress_rule()

    def print_resume_summary(
        self,
        *,
        preserved_successes: int,
        queued_rows: int,
        skipped_rows: int,
    ) -> None:
        if not self.enabled:
            return
        self._print_event(
            "resume",
            (f"preserved={preserved_successes}  queued={queued_rows}  skipped={skipped_rows}"),
            label_style="bold magenta",
            message_style="bold",
        )

    def print_pause_state(self, *, paused: bool, reason: str) -> None:
        if not self.enabled:
            return
        self._print_event(
            "pause" if paused else "resume",
            reason,
            label_style="bold yellow" if paused else "bold green",
            message_style="yellow" if paused else "green",
        )

    def print_finalizing(self, message: str) -> None:
        if not self.enabled:
            return
        self._print_event(
            "finalize",
            message,
            label_style="bold cyan",
            message_style="dim",
        )

    def print_runtime_setting(self, label: str, message: str) -> None:
        if not self.enabled:
            return
        self._print_event(
            label,
            message,
            label_style="bold white",
            message_style="dim",
        )

    def add_compaction_event(
        self,
        slug: str,
        *,
        trigger: str,
        estimated_saved_next_input_tokens: int | None,
        billed_cost: float | None,
        previous_response_id_cleared: bool,
        estimate_error: str | None = None,
    ) -> None:
        if not self.enabled:
            return
        line = Text()
        line.append("  compact ", style="bold magenta")
        line.append(f"[{self._run_label(slug)}] ", style="bold black on magenta")
        line.append(truncate_text(slug, 40), style="bold")
        line.append("  ", style="")
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
            self._print_indented_block(f"estimate: {estimate_error}", style="yellow")

    def add_maintenance_event(
        self,
        slug: str,
        event: dict[str, object],
        *,
        billed_cost: float | None = None,
    ) -> None:
        if not self.enabled:
            return
        line = Text()
        line.append("  maint   ", style="bold magenta")
        line.append(f"[{self._run_label(slug)}] ", style="bold black on magenta")
        line.append(truncate_text(slug, 40), style="bold")
        line.append("  ", style="")
        kind = str(event.get("kind") or event.get("type") or "maintenance")
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

    def stop(self):
        if not self.enabled:
            return
        elapsed = time.time() - self.start_time
        self.console.print()
        line = Text()
        line.append(
            "batch done ",
            style="bold white on green"
            if self.successes == self.completed
            else "bold white on dark_orange",
        )
        line.append(" ", style="")
        line.append(f"{self.successes}", style="bold green")
        line.append("/", style="dim")
        line.append(f"{self.completed}", style="bold cyan")
        line.append(" succeeded", style="bold")
        line.append(f"  {format_cost(self.total_cost)}", style="green")
        line.append(f"  {format_duration(elapsed)}", style="dim")
        if self.session_completed > 0:
            avg_duration = elapsed / self.session_completed
            line.append("  avg=", style="dim")
            line.append(format_duration(avg_duration), style="bold blue")
        self.console.print(line)

    def add_run(
        self,
        slug: str,
        prompt: str,
        *,
        row_id: str | None = None,
        provider: str | None = None,
        model_id: str | None = None,
        thinking_level: str | None = None,
    ):
        if slug not in self.run_numbers:
            self.run_numbers[slug] = self._assign_run_number(slug)
        metadata = {
            key: value
            for key, value in {
                "row_id": row_id,
                "provider": provider,
                "model_id": model_id,
                "thinking_level": thinking_level,
            }.items()
            if value
        }
        if metadata:
            self.run_metadata[slug] = metadata

    def start_run(self, slug: str):
        if not self.enabled:
            return
        self.runs[slug] = time.time()
        self.run_turns[slug] = 0
        line = Text()
        line.append("  start   ", style="bold cyan")
        line.append(f"[{self._run_label(slug)}] ", style="bold black on cyan")
        line.append(truncate_text(slug, 50), style="bold")
        line.append(self._metadata_text(slug), style="dim")
        self.console.print(line)

    def update_run_progress(self, slug: str, turn: int, cost: float):
        if not self.enabled:
            return
        self.run_turns[slug] = turn
        active = len(self.runs)
        queued = max(0, self.total_runs - self.completed - active)
        started_at = self.runs.get(slug)
        elapsed = (time.time() - started_at) if started_at is not None else None

        line = Text()
        line.append("  turn    ", style="bold blue")
        line.append(f"[{self._run_label(slug)}] ", style="bold black on bright_blue")
        line.append(truncate_text(slug, 50), style="bold")
        line.append(self._metadata_text(slug), style="dim")
        line.append("  t=", style="dim")
        line.append(f"{turn}", style="bold yellow")
        if elapsed is not None:
            line.append("  age=", style="dim")
            line.append(format_duration(elapsed), style="bold blue")
        if cost > 0:
            line.append("  spent=", style="dim")
            line.append(format_cost(cost), style="green")
        line.append("  batch ", style="dim")
        line.append("ok=", style="dim")
        line.append(f"{self.successes}", style="bold green")
        line.append(" fail=", style="dim")
        line.append(f"{self.failures}", style="bold red" if self.failures > 0 else "bold green")
        line.append(" done=", style="dim")
        line.append(f"{self.completed}/{self.total_runs}", style="bold cyan")
        line.append(" active=", style="dim")
        line.append(f"{active}", style="bold blue" if active > 0 else "dim")
        line.append(" queued=", style="dim")
        line.append(f"{queued}", style="bold yellow" if queued > 0 else "dim")
        self.console.print(line)

    def complete_run(
        self,
        slug: str,
        success: bool,
        cost: float,
        error: Optional[str] = None,
    ):
        if not self.enabled:
            return

        self.completed += 1
        self.total_cost += cost
        if success:
            self.successes += 1

        now = time.time()
        start_time = self.runs.pop(slug, None)
        last_turn = self.run_turns.pop(slug, 0)
        if start_time is None:
            duration = 0.0
        else:
            self.session_completed += 1
            duration = now - start_time

        line = Text()
        line.append(
            "  done    ",
            style="bold white on green" if success else "bold white on red",
        )
        line.append(
            f"[{self._run_label(slug)}] ",
            style="bold black on green" if success else "bold white on red",
        )
        line.append(truncate_text(slug, 50), style="bold")
        line.append(self._metadata_text(slug), style="dim")
        if success:
            line.append(" ✓", style="green")
        else:
            line.append(" ✗", style="red")
        if last_turn > 0:
            line.append("  t=", style="dim")
            line.append(f"{last_turn}", style="bold yellow")
        line.append(f"  {format_cost(cost)}", style="green" if success else "yellow")
        line.append(f"  {format_duration(duration)}", style="dim")
        line.append("  done=", style="dim")
        line.append(f"{self.completed}/{self.total_runs}", style="bold cyan")
        line.append(" ok=", style="dim")
        line.append(f"{self.successes}", style="bold green")
        line.append(" fail=", style="dim")
        line.append(f"{self.failures}", style="bold red" if self.failures > 0 else "bold green")
        self.console.print(line)

        if error:
            self._print_indented_block(error, style="bold white on red")

        should_checkpoint = self.session_completed > 0 and (
            self.session_completed == 1
            or self.completed == self.total_runs
            or (self.session_completed % self._checkpoint_every == 0)
        )
        if should_checkpoint and self.completed < self.total_runs:
            self._print_progress_rule()
            self._print_checkpoint()
