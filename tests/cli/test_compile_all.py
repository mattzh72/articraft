from __future__ import annotations

import queue
from pathlib import Path
from types import SimpleNamespace

from cli import compile_all


def test_compile_all_infers_scheduler_class_from_model(tmp_path: Path) -> None:
    model_path = tmp_path / "model.py"
    model_path.write_text(
        "\n".join(
            [
                "from sdk import SlotPatternPanelGeometry, TireGeometry",
                "",
                "panel = SlotPatternPanelGeometry((0.1, 0.05), 0.004, slot_size=(0.02, 0.004), pitch=0.03)",
                "tire = TireGeometry(0.1, 0.03)",
                "",
            ]
        ),
        encoding="utf-8",
    )

    assert compile_all._infer_compile_scheduler_class(model_path) == "SlotPatternPanelGeometry"


def test_compile_all_class_aware_queue_prefers_worker_class() -> None:
    candidates = [
        compile_all.CompileCandidate(
            "panel_light", "forced", scheduler_class="Panel", estimated_compile_seconds=1.0
        ),
        compile_all.CompileCandidate(
            "wheel", "forced", scheduler_class="Wheel", estimated_compile_seconds=8.0
        ),
        compile_all.CompileCandidate(
            "panel_heavy", "forced", scheduler_class="Panel", estimated_compile_seconds=5.0
        ),
    ]
    pending = compile_all._ClassAwareCompileQueue.from_candidates(candidates)

    first = pending.pop(preferred_class="Panel")
    second = pending.pop(preferred_class="Panel")
    third = pending.pop(preferred_class="Panel")

    assert first is not None
    assert second is not None
    assert third is not None
    assert first.record_id == "panel_heavy"
    assert second.record_id == "panel_light"
    assert third.record_id == "wheel"


def test_compile_all_worker_uses_materialization_component(
    tmp_path: Path,
    monkeypatch,
) -> None:
    calls: list[tuple[str, dict[str, object]]] = []

    class FakeViewerStore:
        def __init__(self, repo_root: Path, *, ensure_search_index: bool = True) -> None:
            self.repo_root = repo_root
            self.ensure_search_index = ensure_search_index
            self.materialization = self

        def materialize_record_assets(self, record_id: str, **kwargs: object) -> object:
            calls.append((record_id, dict(kwargs)))
            return SimpleNamespace(compiled=True)

    import viewer.api.store as store_module

    monkeypatch.setattr(store_module, "ViewerStore", FakeViewerStore)

    task_queue: queue.Queue[object] = queue.Queue()
    result_queue: queue.Queue[object] = queue.Queue()
    candidate = compile_all.CompileCandidate(record_id="rec_demo", reason="forced", force=True)
    task_queue.put(candidate)
    task_queue.put(None)

    compile_all._worker_loop(
        7,
        1,
        str(tmp_path),
        False,
        "visual",
        task_queue,
        result_queue,
    )

    assert calls == [
        (
            "rec_demo",
            {
                "force": True,
                "ignore_geom_qc": True,
                "validate": False,
                "target": "visual",
                "use_compile_timeout": False,
            },
        )
    ]
    assert result_queue.get_nowait().event == "started"
    finished = result_queue.get_nowait()
    assert finished.event == "finished"
    assert finished.record_id == "rec_demo"
    assert finished.compiled is True
