from __future__ import annotations

import asyncio
import csv
import json
import threading
from pathlib import Path

import pytest

from agent import batch_runner, runner
from agent.models import AgentResult, TerminateReason
from cli.dataset import main as dataset_main
from storage import dataset_workflow
from storage.categories import CategoryStore
from storage.datasets import DatasetStore
from storage.models import CategoryRecord
from storage.queries import StorageQueries
from storage.repo import StorageRepo
from storage.revisions import active_provenance_path
from storage.search import SearchIndex
from viewer.api.store import ViewerStore


def _write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _patch_dataset_tokens(monkeypatch: pytest.MonkeyPatch, *tokens: str) -> None:
    iterator = iter(tokens)
    monkeypatch.setattr(dataset_workflow, "new_dataset_token", lambda: next(iterator))


class SuccessBatchAgent:
    active = 0
    max_active = 0

    def __init__(
        self,
        *,
        file_path: str,
        trace_dir: str | None = None,
        provider: str = "openai",
        model_id: str | None = None,
        **_: object,
    ) -> None:
        self.file_path = Path(file_path)
        self.trace_dir = Path(trace_dir) if trace_dir else None
        self.provider = provider
        self.loaded_system_prompt_path = str(
            runner.resolve_system_prompt_path(
                "designer_system_prompt.txt",
                provider=provider,
                repo_root=Path(__file__).resolve().parents[2],
            )
        )
        self.llm = type("LLM", (), {"model_id": model_id or "gpt-5.5-2026-04-23"})()

    async def __aenter__(self) -> "SuccessBatchAgent":
        return self

    async def __aexit__(self, exc_type: object, exc: object, tb: object) -> None:
        return None

    async def run(self, user_content: object) -> AgentResult:
        type(self).active += 1
        type(self).max_active = max(type(self).max_active, type(self).active)
        try:
            await asyncio.sleep(0.05)
            self.file_path.parent.mkdir(parents=True, exist_ok=True)
            self.file_path.write_text(
                "from __future__ import annotations\n\nobject_model = None\n",
                encoding="utf-8",
            )
            (self.file_path.parent / "assets" / "meshes").mkdir(parents=True, exist_ok=True)
            (self.file_path.parent / "assets" / "meshes" / "part.obj").write_text(
                "# mesh\n",
                encoding="utf-8",
            )
            (self.file_path.parent / "cost.json").write_text(
                json.dumps({"total": {"costs_usd": {"total": 0.1}}}, indent=2),
                encoding="utf-8",
            )
            if self.trace_dir is not None:
                self.trace_dir.mkdir(parents=True, exist_ok=True)
                (self.trace_dir / "trajectory.jsonl").write_text(
                    '{"type":"message","message":{"role":"assistant","content":"done"}}\n',
                    encoding="utf-8",
                )
            return AgentResult(
                success=True,
                reason=TerminateReason.CODE_VALID,
                message="done",
                conversation=[{"role": "user", "content": user_content}],
                final_code=self.file_path.read_text(encoding="utf-8"),
                urdf_xml="<robot name='batch'/>",
                compile_warnings=[],
                turn_count=3,
                tool_call_count=4,
                compile_attempt_count=1,
                usage=None,
            )
        finally:
            type(self).active -= 1


class FlakyBatchAgent(SuccessBatchAgent):
    attempts_by_record: dict[str, int] = {}

    async def run(self, user_content: object) -> AgentResult:
        record_id = self.file_path.parent.name
        self.attempts_by_record[record_id] = self.attempts_by_record.get(record_id, 0) + 1
        if "retry" in str(user_content).lower() and self.attempts_by_record[record_id] == 1:
            type(self).active += 1
            type(self).max_active = max(type(self).max_active, type(self).active)
            try:
                await asyncio.sleep(0.05)
                self.file_path.parent.mkdir(parents=True, exist_ok=True)
                self.file_path.write_text("# failed\n", encoding="utf-8")
                return AgentResult(
                    success=False,
                    reason=TerminateReason.MAX_TURNS,
                    message="synthetic failure",
                    conversation=[{"role": "user", "content": user_content}],
                    final_code=None,
                    urdf_xml=None,
                    compile_warnings=[],
                    turn_count=1,
                    tool_call_count=1,
                    compile_attempt_count=1,
                    usage=None,
                )
            finally:
                type(self).active -= 1
        return await super().run(user_content)


class FailIfRunAgent(SuccessBatchAgent):
    call_count = 0

    async def run(self, user_content: object) -> AgentResult:
        type(self).call_count += 1
        raise AssertionError("resume should not rerun a durably completed row")


class ExplodingBatchAgent(SuccessBatchAgent):
    async def run(self, user_content: object) -> AgentResult:
        if "explode" in str(user_content).lower():
            await asyncio.sleep(0.05)
            raise RuntimeError("synthetic crash")
        return await super().run(user_content)


class RecordingBatchDisplay:
    events: list[str] = []

    def __init__(self, **kwargs: object) -> None:
        type(self).events.append("init")
        return None

    def add_run(self, slug: str, prompt: str, **kwargs: object) -> None:
        return None

    def start(self) -> None:
        type(self).events.append("start")

    def stop(self) -> None:
        type(self).events.append("stop")

    def start_run(self, slug: str) -> None:
        type(self).events.append(f"start_run:{slug}")

    def update_run_progress(self, slug: str, turn: int, cost: float) -> None:
        return None

    def complete_run(self, slug: str, success: bool, cost: float, error: str | None = None) -> None:
        type(self).events.append(f"complete_run:{slug}:{success}")

    def print_resume_summary(
        self,
        *,
        preserved_successes: int,
        queued_rows: int,
        skipped_rows: int,
    ) -> None:
        type(self).events.append(f"resume:{preserved_successes}:{queued_rows}:{skipped_rows}")

    def print_pause_state(self, *, paused: bool, reason: str) -> None:
        type(self).events.append(f"pause:{paused}:{reason}")

    def print_finalizing(self, message: str) -> None:
        type(self).events.append(f"finalize:{message}")


@pytest.fixture(autouse=True)
def reset_agents() -> None:
    SuccessBatchAgent.active = 0
    SuccessBatchAgent.max_active = 0
    FlakyBatchAgent.active = 0
    FlakyBatchAgent.max_active = 0
    FlakyBatchAgent.attempts_by_record = {}
    FailIfRunAgent.call_count = 0
    RecordingBatchDisplay.events = []


def test_build_batch_config_validates_csv_rows(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="hinge", title="Hinge", description="")
    )

    cases = [
        (
            [
                {
                    "row_id": "a",
                    "category_slug": "new_cat",
                    "prompt": "x",
                    "provider": "openai",
                    "model_id": "gpt-5.4",
                    "thinking_level": "high",
                    "max_turns": "10",
                    "sdk_package": "sdk",
                }
            ],
            "without category_title",
        ),
        (
            [
                {
                    "row_id": "dup",
                    "category_slug": "hinge",
                    "category_title": "Hinge",
                    "prompt": "x",
                    "provider": "openai",
                    "model_id": "gpt-5.4",
                    "thinking_level": "high",
                    "max_turns": "10",
                    "sdk_package": "sdk",
                },
                {
                    "row_id": "dup",
                    "category_slug": "hinge",
                    "category_title": "Hinge",
                    "prompt": "y",
                    "provider": "openai",
                    "model_id": "gpt-5.4",
                    "thinking_level": "high",
                    "max_turns": "10",
                    "sdk_package": "sdk",
                },
            ],
            "Duplicate row_id",
        ),
        (
            [
                {
                    "row_id": "a",
                    "category_slug": "hinge",
                    "category_title": "Hinge",
                    "prompt": "x",
                    "provider": "gemini",
                    "model_id": "gpt-5.4",
                    "thinking_level": "high",
                    "max_turns": "10",
                    "sdk_package": "sdk",
                }
            ],
            "provider/model mismatch",
        ),
        (
            [
                {
                    "row_id": "a",
                    "category_slug": "hinge",
                    "category_title": "Wrong",
                    "prompt": "x",
                    "provider": "openai",
                    "model_id": "gpt-5.4",
                    "thinking_level": "high",
                    "max_turns": "10",
                    "sdk_package": "sdk",
                }
            ],
            "category_title mismatch",
        ),
        (
            [
                {
                    "row_id": "a",
                    "category_slug": "hinge",
                    "category_title": "Hinge",
                    "prompt": "x",
                    "provider": "openai",
                    "model_id": "gpt-5.4",
                    "thinking_level": "high",
                    "max_turns": "0",
                    "sdk_package": "sdk",
                }
            ],
            "max_turns > 0",
        ),
    ]

    for index, (rows, message) in enumerate(cases, start=1):
        spec_path = tmp_path / f"invalid_{index}.csv"
        _write_csv(spec_path, rows)
        with pytest.raises(ValueError, match=message):
            batch_runner.build_batch_config(
                repo_root=tmp_path,
                spec_arg=str(spec_path),
                concurrency=2,
                system_prompt_path="designer_system_prompt.txt",
                qc_blurb_path=None,
                resume=False,
                resume_policy="failed_or_pending",
                keep_awake=False,
                pause_file=None,
                pause_poll_seconds=1.0,
                keyboard_pause_enabled=False,
            )


def test_build_batch_config_resolves_auto_concurrency_to_logical_cpu_count(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="hinge", title="Hinge", description="")
    )

    spec_path = tmp_path / "source_specs" / "auto_batch.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": f"row_{index}",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": f"make hinge {index}",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "10",
                "sdk_package": "sdk",
            }
            for index in range(1, 9)
        ],
    )
    monkeypatch.setattr(batch_runner, "_logical_cpu_count", lambda: 6)

    config = batch_runner.build_batch_config(
        repo_root=tmp_path,
        spec_arg=str(spec_path),
        concurrency="auto",
        system_prompt_path="designer_system_prompt.txt",
        qc_blurb_path=None,
        resume=False,
        resume_policy="failed_or_pending",
        keep_awake=False,
        pause_file=None,
        pause_poll_seconds=1.0,
        keyboard_pause_enabled=False,
    )

    assert config.concurrency == 6


def test_build_batch_config_defaults_missing_sdk_package_to_sdk(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="hinge", title="Hinge", description="")
    )

    spec_path = tmp_path / "source_specs" / "default_sdk_batch.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "row_1",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make hinge 1",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "10",
            }
        ],
    )

    config = batch_runner.build_batch_config(
        repo_root=tmp_path,
        spec_arg=str(spec_path),
        concurrency=1,
        system_prompt_path="designer_system_prompt.txt",
        qc_blurb_path=None,
        resume=False,
        resume_policy="failed_or_pending",
        keep_awake=False,
        pause_file=None,
        pause_poll_seconds=1.0,
        keyboard_pause_enabled=False,
    )

    assert len(config.rows) == 1
    assert config.rows[0].sdk_package == "sdk"


def test_build_batch_config_supports_max_cost_default_and_row_overrides(
    tmp_path: Path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="hinge", title="Hinge", description="")
    )

    spec_path = tmp_path / "source_specs" / "max_cost_batch.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "explicit_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "10",
                "max_cost_usd": "1.25",
                "sdk_package": "sdk",
            },
            {
                "row_id": "defaulted_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make another hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "10",
                "sdk_package": "sdk",
            },
        ],
    )

    config = batch_runner.build_batch_config(
        repo_root=tmp_path,
        spec_arg=str(spec_path),
        concurrency=1,
        system_prompt_path="designer_system_prompt.txt",
        max_cost_usd=2.5,
        qc_blurb_path=None,
        resume=False,
        resume_policy="failed_or_pending",
        keep_awake=False,
        pause_file=None,
        pause_poll_seconds=1.0,
        keyboard_pause_enabled=False,
    )

    row_by_id = {row.row_id: row for row in config.rows}
    assert config.max_cost_usd == 2.5
    assert row_by_id["explicit_row"].max_cost_usd == 1.25
    assert row_by_id["defaulted_row"].max_cost_usd == 2.5


def test_build_batch_config_rejects_invalid_max_cost_usd_value(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="hinge", title="Hinge", description="")
    )

    spec_path = tmp_path / "source_specs" / "invalid_max_cost.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "bad_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "10",
                "max_cost_usd": "0",
                "sdk_package": "sdk",
            }
        ],
    )

    with pytest.raises(ValueError, match="max_cost_usd must be > 0"):
        batch_runner.build_batch_config(
            repo_root=tmp_path,
            spec_arg=str(spec_path),
            concurrency=1,
            system_prompt_path="designer_system_prompt.txt",
            qc_blurb_path=None,
            resume=False,
            resume_policy="failed_or_pending",
            keep_awake=False,
            pause_file=None,
            pause_poll_seconds=1.0,
            keyboard_pause_enabled=False,
        )


def test_build_batch_config_rejects_legacy_scaffold_mode_column(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="hinge", title="Hinge", description="")
    )

    spec_path = tmp_path / "source_specs" / "legacy_scaffold_mode.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "bad_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "10",
                "sdk_package": "sdk",
                "scaffold_mode": "strict",
            }
        ],
    )

    with pytest.raises(ValueError, match="Unsupported CSV columns: scaffold_mode"):
        batch_runner.build_batch_config(
            repo_root=tmp_path,
            spec_arg=str(spec_path),
            concurrency=1,
            system_prompt_path="designer_system_prompt.txt",
            qc_blurb_path=None,
            resume=False,
            resume_policy="failed_or_pending",
            keep_awake=False,
            pause_file=None,
            pause_poll_seconds=1.0,
            keyboard_pause_enabled=False,
        )


def test_run_batch_max_cost_cli_override_and_csv_override(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="hinge", title="Hinge", description="")
    )
    spec_path = tmp_path / "source_specs" / "max_cost_cli.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "row_1",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make a hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "max_cost_usd": "1.25",
                "sdk_package": "sdk",
            },
            {
                "row_id": "row_2",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make another hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
        ],
    )

    captured: dict[str, object] = {}

    async def fake_run_dataset_batch(config: object) -> dict[str, object]:
        captured["config_max_cost_usd"] = getattr(config, "max_cost_usd", None)
        captured["row_limits"] = [
            getattr(row, "max_cost_usd", None) for row in getattr(config, "rows", [])
        ]
        return {
            "run_id": "run_mocked",
            "status": "success",
            "prompt_count": 2,
            "success_count": 2,
            "failed_count": 0,
        }

    monkeypatch.setattr(batch_runner, "run_dataset_batch", fake_run_dataset_batch)

    exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "1",
            "--max-cost-usd",
            "3.0",
        ]
    )
    assert exit_code == 0
    assert captured["config_max_cost_usd"] == 3.0
    assert captured["row_limits"] == [1.25, 3.0]


def test_run_dataset_batch_continues_after_over_budget_row(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(batch_runner, "resolve_current_record_author", lambda repo_root: "mattzh72")
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    CategoryStore(repo).save(
        CategoryRecord(schema_version=1, slug="hinge", title="Hinge", description="")
    )
    spec_path = tmp_path / "source_specs" / "over_budget_batch.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "expensive_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "expensive hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "max_cost_usd": "0.5",
                "sdk_package": "sdk",
            },
            {
                "row_id": "cheap_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "cheap hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "max_cost_usd": "0.5",
                "sdk_package": "sdk",
            },
        ],
    )
    config = batch_runner.build_batch_config(
        repo_root=tmp_path,
        spec_arg=str(spec_path),
        concurrency=1,
        system_prompt_path="designer_system_prompt.txt",
        qc_blurb_path=None,
        resume=False,
        resume_policy="failed_or_pending",
        keep_awake=False,
        pause_file=None,
        pause_poll_seconds=1.0,
        keyboard_pause_enabled=False,
    )

    async def fake_execute_single_run(
        *args: object, **kwargs: object
    ) -> runner.RunExecutionOutcome:
        assert kwargs["record_author"] == "mattzh72"
        context = kwargs["context"]
        assert isinstance(context, runner.SingleRunContext)
        prompt_text = str(kwargs["prompt_text"])
        if "expensive" in prompt_text:
            (context.staging_dir / "cost.json").write_text(
                json.dumps({"total": {"costs_usd": {"total": 0.75}}}, indent=2),
                encoding="utf-8",
            )
            return runner.RunExecutionOutcome(
                exit_code=2,
                run_id=context.run_id,
                record_id=context.record_id,
                status="failed",
                message="Cost limit exceeded after turn 2: cumulative $0.750000 exceeded limit $0.500000",
                staging_dir=context.staging_dir,
                turn_count=2,
                tool_call_count=1,
                compile_attempt_count=0,
                provider="openai",
                model_id="gpt-5.4",
                sdk_package="sdk",
            )
        (context.staging_dir / "cost.json").write_text(
            json.dumps({"total": {"costs_usd": {"total": 0.1}}}, indent=2),
            encoding="utf-8",
        )
        return runner.RunExecutionOutcome(
            exit_code=0,
            run_id=context.run_id,
            record_id=context.record_id,
            status="success",
            message=None,
            staging_dir=context.staging_dir,
            turn_count=1,
            tool_call_count=1,
            compile_attempt_count=0,
            provider="openai",
            model_id="gpt-5.4",
            sdk_package="sdk",
        )

    async def fake_finalize(
        *,
        repo: StorageRepo,
        config: batch_runner.BatchRunConfig,
        final_status_by_row: dict[str, str],
    ) -> None:
        return None

    monkeypatch.setattr(batch_runner, "_execute_single_run", fake_execute_single_run)
    monkeypatch.setattr(batch_runner, "_finalize_batch_dataset_artifacts", fake_finalize)
    monkeypatch.setattr(SearchIndex, "rebuild", lambda self: None)

    summary = asyncio.run(batch_runner.run_dataset_batch(config))

    assert summary["success_count"] == 1
    assert summary["failed_count"] == 1
    assert summary["status"] == "failed"

    run_dir = repo.layout.run_dir(config.run_id)
    result_rows = [
        json.loads(line)
        for line in (run_dir / "results.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    result_by_row = {row["row_id"]: row for row in result_rows if row["status"] != "running"}
    assert "Cost limit exceeded" in result_by_row["expensive_row"]["message"]
    assert result_by_row["cheap_row"]["status"] == "success"
    assert config.record_author == "mattzh72"


def test_run_batch_persists_records_and_batch_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", SuccessBatchAgent)
    _patch_dataset_tokens(monkeypatch, "0001", "0001")

    thread_ids: list[int] = []
    manifest_write_calls: list[int] = []
    original_write_success_record = runner._write_success_record
    original_write_dataset_manifest = DatasetStore.write_dataset_manifest

    def wrapped_write_success_record(*args: object, **kwargs: object) -> Path:
        thread_ids.append(threading.get_ident())
        return original_write_success_record(*args, **kwargs)

    def wrapped_write_dataset_manifest(self: DatasetStore) -> Path:
        manifest_write_calls.append(1)
        return original_write_dataset_manifest(self)

    monkeypatch.setattr(runner, "_write_success_record", wrapped_write_success_record)
    monkeypatch.setattr(DatasetStore, "write_dataset_manifest", wrapped_write_dataset_manifest)

    spec_path = tmp_path / "source_specs" / "mixed_batch.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "hinge_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make a hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
            {
                "row_id": "fan_row",
                "category_slug": "fan",
                "category_title": "Fan",
                "prompt": "make a fan",
                "provider": "gemini",
                "model_id": "gemini-3-flash-preview",
                "thinking_level": "low",
                "max_turns": "8",
                "sdk_package": "sdk",
            },
        ],
    )

    exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "2",
        ]
    )
    assert exit_code == 0
    assert SuccessBatchAgent.max_active >= 2
    assert thread_ids
    assert all(thread_id != threading.main_thread().ident for thread_id in thread_ids)
    assert len(manifest_write_calls) == 1

    repo = StorageRepo(tmp_path)
    copied_spec = repo.layout.batch_spec_path("mixed_batch")
    assert copied_spec.exists()

    run_dirs = sorted(path for path in repo.layout.runs_root.iterdir() if path.is_dir())
    assert len(run_dirs) == 1
    run_id = run_dirs[0].name
    run_payload = repo.read_json(repo.layout.run_metadata_path(run_id))
    assert run_payload["run_mode"] == "dataset_batch"
    assert run_payload["provider"] == "mixed"
    assert run_payload["model_id"] == "mixed"
    assert run_payload["sdk_package"] == "sdk"
    assert run_payload["batch_spec_id"] == "mixed_batch"
    assert sorted(run_payload["category_slugs"]) == ["fan", "hinge"]
    assert run_payload["status"] == "success"

    result_rows = [
        json.loads(line)
        for line in repo.layout.run_results_path(run_id).read_text(encoding="utf-8").splitlines()
    ]
    assert len(result_rows) == 2
    assert sorted(row["row_id"] for row in result_rows) == ["fan_row", "hinge_row"]

    hinge_record = repo.read_json(repo.layout.record_metadata_path("rec_hinge_0001"))
    fan_record = repo.read_json(repo.layout.record_metadata_path("rec_fan_0001"))
    assert hinge_record["source"]["run_id"] == run_id
    assert hinge_record["source"]["batch_spec_id"] == "mixed_batch"
    assert hinge_record["source"]["row_id"] == "hinge_row"
    assert hinge_record["source"]["prompt_index"] == 1
    assert fan_record["source"]["row_id"] == "fan_row"
    assert fan_record["sdk_package"] == "sdk"
    hinge_provenance = repo.read_json(
        active_provenance_path(repo, "rec_hinge_0001", record=hinge_record)
    )
    fan_provenance = repo.read_json(active_provenance_path(repo, "rec_fan_0001", record=fan_record))
    assert "scaffold_mode" not in hinge_provenance["prompting"]
    assert "scaffold_mode" not in fan_provenance["prompting"]

    queries = StorageQueries(repo)
    assert run_id in queries.list_run_ids_for_category("hinge")
    assert run_id in queries.list_run_ids_for_category("fan")

    dataset_store = DatasetStore(repo)
    hinge_entry = dataset_store.load_entry("rec_hinge_0001")
    fan_entry = dataset_store.load_entry("rec_fan_0001")
    assert hinge_entry == {
        "schema_version": 1,
        "dataset_id": "ds_hinge_0001",
        "record_id": "rec_hinge_0001",
        "category_slug": "hinge",
        "promoted_at": hinge_entry["promoted_at"],
    }
    assert fan_entry == {
        "schema_version": 1,
        "dataset_id": "ds_fan_0001",
        "record_id": "rec_fan_0001",
        "category_slug": "fan",
        "promoted_at": fan_entry["promoted_at"],
    }
    manifest = repo.read_json(repo.layout.dataset_manifest_path())
    assert manifest == {
        "generated": [
            {"name": "ds_fan_0001", "record_id": "rec_fan_0001"},
            {"name": "ds_hinge_0001", "record_id": "rec_hinge_0001"},
        ]
    }

    hinge_category = CategoryStore(repo).load("hinge")
    fan_category = CategoryStore(repo).load("fan")
    assert hinge_category == {
        "schema_version": 1,
        "slug": "hinge",
        "title": "Hinge",
        "description": "",
    }
    assert fan_category == {
        "schema_version": 1,
        "slug": "fan",
        "title": "Fan",
        "description": "",
    }

    viewer_store = ViewerStore(tmp_path)
    run_detail = viewer_store.runs.load_run_detail(run_id)
    assert run_detail is not None
    assert run_detail.run.status == "success"
    assert len(run_detail.results) == 2
    assert {item.raw["row_id"] for item in run_detail.results} == {"hinge_row", "fan_row"}


def test_run_batch_resume_reuses_allocations_and_only_reruns_failed_rows(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", FlakyBatchAgent)
    _patch_dataset_tokens(monkeypatch, "0001", "0002", "0001")

    spec_path = tmp_path / "source_specs" / "resume_batch.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "ok_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make a hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
            {
                "row_id": "retry_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "retry hinge once",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
            {
                "category_slug": "fan",
                "category_title": "Fan",
                "prompt": "make a fan",
                "provider": "gemini",
                "model_id": "gemini-3-flash-preview",
                "thinking_level": "low",
                "max_turns": "8",
                "sdk_package": "sdk",
            },
        ],
    )

    first_exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "3",
        ]
    )
    assert first_exit_code == 1
    assert FlakyBatchAgent.max_active >= 2

    repo = StorageRepo(tmp_path)
    run_id = next(path.name for path in repo.layout.runs_root.iterdir() if path.is_dir())
    allocations = repo.read_json(repo.layout.run_allocations_path(run_id))
    assert allocations is not None
    allocations_by_row = {row["row_id"]: row for row in allocations["rows"]}
    fan_row_id = next(
        row_id for row_id in allocations_by_row if row_id not in {"ok_row", "retry_row"}
    )
    failed_record_id = allocations_by_row["retry_row"]["record_id"]
    failed_dataset_id = allocations_by_row["retry_row"]["dataset_id"]

    first_results = [
        json.loads(line)
        for line in repo.layout.run_results_path(run_id).read_text(encoding="utf-8").splitlines()
    ]
    assert len(first_results) == 3
    assert next(row for row in first_results if row["row_id"] == "retry_row")["status"] == "failed"

    dataset_store = DatasetStore(repo)
    ok_entry = dataset_store.load_entry(allocations_by_row["ok_row"]["record_id"])
    fan_entry = dataset_store.load_entry(allocations_by_row[fan_row_id]["record_id"])
    failed_entry = dataset_store.load_entry(failed_record_id)
    assert ok_entry is not None
    assert ok_entry["dataset_id"] == allocations_by_row["ok_row"]["dataset_id"]
    assert fan_entry is not None
    assert fan_entry["dataset_id"] == allocations_by_row[fan_row_id]["dataset_id"]
    assert failed_entry is None

    first_manifest = repo.read_json(repo.layout.dataset_manifest_path())
    assert first_manifest == {
        "generated": [
            {
                "name": allocations_by_row[fan_row_id]["dataset_id"],
                "record_id": allocations_by_row[fan_row_id]["record_id"],
            },
            {
                "name": allocations_by_row["ok_row"]["dataset_id"],
                "record_id": allocations_by_row["ok_row"]["record_id"],
            },
        ]
    }
    hinge_category = CategoryStore(repo).load("hinge")
    fan_category = CategoryStore(repo).load("fan")
    assert hinge_category == {
        "schema_version": 1,
        "slug": "hinge",
        "title": "Hinge",
        "description": "",
    }
    assert fan_category == {
        "schema_version": 1,
        "slug": "fan",
        "title": "Fan",
        "description": "",
    }

    second_exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "3",
            "--resume",
        ]
    )
    assert second_exit_code == 0

    second_results = [
        json.loads(line)
        for line in repo.layout.run_results_path(run_id).read_text(encoding="utf-8").splitlines()
    ]
    assert len(second_results) == 3
    retry_row = next(row for row in second_results if row["row_id"] == "retry_row")
    assert retry_row["status"] == "success"
    assert retry_row["record_id"] == failed_record_id
    assert retry_row["dataset_id"] == failed_dataset_id

    retry_state = repo.read_json(repo.layout.run_row_state_path(run_id, "retry_row"))
    assert retry_state["latest_status"] == "success"
    assert retry_state["attempt_count"] == 2

    record = repo.read_json(repo.layout.record_metadata_path(failed_record_id))
    assert record["source"]["row_id"] == "retry_row"
    assert record["source"]["run_id"] == run_id

    retry_entry = dataset_store.load_entry(failed_record_id)
    assert retry_entry is not None
    assert retry_entry["dataset_id"] == failed_dataset_id
    second_manifest = repo.read_json(repo.layout.dataset_manifest_path())
    assert second_manifest == {
        "generated": [
            {
                "name": allocations_by_row[fan_row_id]["dataset_id"],
                "record_id": allocations_by_row[fan_row_id]["record_id"],
            },
            {
                "name": allocations_by_row["ok_row"]["dataset_id"],
                "record_id": allocations_by_row["ok_row"]["record_id"],
            },
            {
                "name": failed_dataset_id,
                "record_id": failed_record_id,
            },
        ]
    }
    hinge_category = CategoryStore(repo).load("hinge")
    assert hinge_category == {
        "schema_version": 1,
        "slug": "hinge",
        "title": "Hinge",
        "description": "",
    }


def test_run_batch_resume_reconciles_durable_success_without_rerunning(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", SuccessBatchAgent)
    _patch_dataset_tokens(monkeypatch, "0001")

    spec_path = tmp_path / "source_specs" / "resume_reconcile.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "ok_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make a hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
        ],
    )

    first_exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "1",
        ]
    )
    assert first_exit_code == 0

    repo = StorageRepo(tmp_path)
    run_id = next(path.name for path in repo.layout.runs_root.iterdir() if path.is_dir())
    row_state_path = repo.layout.run_row_state_path(run_id, "ok_row")
    row_state = repo.read_json(row_state_path)
    assert isinstance(row_state, dict)
    row_state["latest_status"] = "running"
    row_state["latest_error_message"] = None
    row_state["current_attempt_started_at"] = "2026-03-20T00:00:00Z"
    repo.write_json(row_state_path, row_state)

    results_path = repo.layout.run_results_path(run_id)
    result_rows = [
        json.loads(line) for line in results_path.read_text(encoding="utf-8").splitlines()
    ]
    assert len(result_rows) == 1
    result_rows[0]["status"] = "running"
    result_rows[0]["record_dir"] = None
    results_path.write_text(
        "".join(json.dumps(row) + "\n" for row in result_rows),
        encoding="utf-8",
    )

    monkeypatch.setattr(runner, "ArticraftAgent", FailIfRunAgent)

    second_exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "1",
            "--resume",
        ]
    )
    assert second_exit_code == 0
    assert FailIfRunAgent.call_count == 0

    repaired_state = repo.read_json(row_state_path)
    assert repaired_state["latest_status"] == "success"
    assert repaired_state["attempt_count"] == row_state["attempt_count"]

    repaired_rows = [
        json.loads(line) for line in results_path.read_text(encoding="utf-8").splitlines()
    ]
    assert len(repaired_rows) == 1
    assert repaired_rows[0]["status"] == "success"
    assert repaired_rows[0]["record_dir"] == "data/records/rec_hinge_0001"


def test_run_batch_resume_reruns_interrupted_running_rows(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", FlakyBatchAgent)
    _patch_dataset_tokens(monkeypatch, "0001")

    spec_path = tmp_path / "source_specs" / "resume_running.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "retry_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "retry hinge once",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
        ],
    )

    first_exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "1",
        ]
    )
    assert first_exit_code == 1

    repo = StorageRepo(tmp_path)
    run_id = next(path.name for path in repo.layout.runs_root.iterdir() if path.is_dir())
    row_state_path = repo.layout.run_row_state_path(run_id, "retry_row")
    row_state = repo.read_json(row_state_path)
    assert isinstance(row_state, dict)
    row_state["latest_status"] = "running"
    row_state["latest_error_message"] = None
    repo.write_json(row_state_path, row_state)

    results_path = repo.layout.run_results_path(run_id)
    result_rows = [
        json.loads(line) for line in results_path.read_text(encoding="utf-8").splitlines()
    ]
    assert len(result_rows) == 1
    result_rows[0]["status"] = "running"
    result_rows[0]["record_dir"] = None
    results_path.write_text(
        "".join(json.dumps(row) + "\n" for row in result_rows),
        encoding="utf-8",
    )

    second_exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "1",
            "--resume",
        ]
    )
    assert second_exit_code == 0

    repaired_state = repo.read_json(row_state_path)
    assert repaired_state["latest_status"] == "success"
    assert repaired_state["attempt_count"] == 2

    repaired_rows = [
        json.loads(line) for line in results_path.read_text(encoding="utf-8").splitlines()
    ]
    assert len(repaired_rows) == 1
    assert repaired_rows[0]["status"] == "success"


def test_run_batch_unexpected_worker_exception_finalizes_cleanly(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", ExplodingBatchAgent)
    _patch_dataset_tokens(monkeypatch, "0001", "0002")

    spec_path = tmp_path / "source_specs" / "unexpected_worker_error.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "ok_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make a hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
            {
                "row_id": "boom_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "explode this row",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
        ],
    )

    exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "2",
        ]
    )
    assert exit_code == 1

    repo = StorageRepo(tmp_path)
    run_id = next(path.name for path in repo.layout.runs_root.iterdir() if path.is_dir())
    run_payload = repo.read_json(repo.layout.run_metadata_path(run_id))
    assert run_payload["status"] == "failed"

    result_rows = [
        json.loads(line)
        for line in repo.layout.run_results_path(run_id).read_text(encoding="utf-8").splitlines()
    ]
    assert len(result_rows) == 2
    result_by_row_id = {row["row_id"]: row for row in result_rows}
    assert result_by_row_id["ok_row"]["status"] == "success"
    assert result_by_row_id["boom_row"]["status"] == "failed"
    assert "synthetic crash" in result_by_row_id["boom_row"]["message"]

    boom_state = repo.read_json(repo.layout.run_row_state_path(run_id, "boom_row"))
    assert boom_state["latest_status"] == "failed"
    assert "synthetic crash" in boom_state["latest_error_message"]

    manifest = repo.read_json(repo.layout.dataset_manifest_path())
    assert manifest == {
        "generated": [
            {"name": "ds_hinge_0001", "record_id": "rec_hinge_0001"},
        ]
    }
    assert repo.layout.search_index_path().exists()


def test_run_batch_display_stops_after_finalization(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", SuccessBatchAgent)
    monkeypatch.setattr(batch_runner, "BatchRunDisplay", RecordingBatchDisplay)
    _patch_dataset_tokens(monkeypatch, "0001")

    spec_path = tmp_path / "source_specs" / "display_order.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "display_row",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make a hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
        ],
    )

    exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "1",
        ]
    )
    assert exit_code == 0

    finalize_indices = [
        index
        for index, event in enumerate(RecordingBatchDisplay.events)
        if event.startswith("finalize:")
    ]
    assert finalize_indices
    stop_index = RecordingBatchDisplay.events.index("stop")
    assert stop_index > max(finalize_indices)


def test_run_batch_display_uses_single_start_path(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", SuccessBatchAgent)
    monkeypatch.setattr(batch_runner, "BatchRunDisplay", RecordingBatchDisplay)
    _patch_dataset_tokens(monkeypatch, "0001", "0002")

    spec_path = tmp_path / "source_specs" / "display_rows.csv"
    _write_csv(
        spec_path,
        [
            {
                "row_id": "row_1",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make a hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
            {
                "row_id": "row_2",
                "category_slug": "hinge",
                "category_title": "Hinge",
                "prompt": "make another hinge",
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": "12",
                "sdk_package": "sdk",
            },
        ],
    )

    exit_code = dataset_main(
        [
            "--repo-root",
            str(tmp_path),
            "run-batch",
            str(spec_path),
            "--concurrency",
            "1",
        ]
    )
    assert exit_code == 0

    assert "init" in RecordingBatchDisplay.events
    assert any(
        event.startswith("start_run:rec_hinge_0001") for event in RecordingBatchDisplay.events
    )
