from __future__ import annotations

import asyncio
import io
import json
import sys
from contextlib import redirect_stdout
from pathlib import Path
from tempfile import TemporaryDirectory

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from agent import runner
from cli.workbench import main as workbench_main
from tests.agent.test_runtime_storage import FakeAgent


def main() -> None:
    original_agent = runner.ArticraftAgent
    runner.ArticraftAgent = FakeAgent
    try:
        with TemporaryDirectory() as tmpdir:
            repo_root = Path(tmpdir)
            exit_code = asyncio.run(
                runner.run_from_input(
                    "make a cabinet hinge",
                    prompt_text="make a cabinet hinge",
                    display_prompt="make a cabinet hinge",
                    repo_root=repo_root,
                    image_path=None,
                    provider="openai",
                    thinking_level="high",
                    max_turns=30,
                    system_prompt_path="designer_system_prompt.txt",
                    sdk_package="sdk",
                    sdk_docs_mode="full",
                    label="hinge rerun",
                    tags=["hinge"],
                )
            )
            assert exit_code == 0

            record_dir = next((repo_root / "data" / "records").iterdir())
            original_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
            original_run_id = original_record["source"]["run_id"]

            output = io.StringIO()
            with redirect_stdout(output):
                assert (
                    workbench_main(
                        [
                            "--repo-root",
                            str(repo_root),
                            "rerun-record",
                            str(record_dir),
                        ]
                    )
                    == 0
                )

            updated_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
            assert updated_record["record_id"] == record_dir.name
            assert updated_record["source"]["run_id"] != original_run_id
            assert (repo_root / "data" / "cache" / "search.sqlite").exists()

            text = output.getvalue()
            assert f"reran record_id={record_dir.name}" in text
            assert "search_index=" in text
    finally:
        runner.ArticraftAgent = original_agent


if __name__ == "__main__":
    main()
