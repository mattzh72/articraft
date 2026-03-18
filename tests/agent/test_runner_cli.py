from __future__ import annotations

import contextlib
import io
import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from agent import runner


def main() -> None:
    help_stdout = io.StringIO()
    with contextlib.redirect_stdout(help_stdout):
        try:
            runner.main(["--help"])
        except SystemExit as exc:
            assert exc.code == 0
        else:
            raise AssertionError("runner.main(['--help']) should exit via SystemExit(0)")

    help_text = help_stdout.getvalue()
    assert "--prompt" in help_text
    assert "--image" in help_text
    assert "--provider {gemini,openai}" in help_text
    assert "--openai-transport {http,websocket}" in help_text
    assert "--sdk-docs-mode {core,full,none}" in help_text
    assert "--collection {workbench,dataset}" in help_text
    assert "--dataset-id DATASET_ID" in help_text
    assert "--flash" not in help_text
    assert "--hybrid-sdk" not in help_text
    assert "--openai-reasoning-summary" not in help_text
    assert "Currently supported only with --provider openai." not in help_text

    invalid_stderr = io.StringIO()
    with contextlib.redirect_stderr(invalid_stderr):
        try:
            runner.main(["--prompt", "test", "--flash"])
        except SystemExit as exc:
            assert exc.code == 2
        else:
            raise AssertionError("runner.main(['--prompt', 'test', '--flash']) should fail")
    assert "unrecognized arguments: --flash" in invalid_stderr.getvalue()

    dataset_stderr = io.StringIO()
    with contextlib.redirect_stderr(dataset_stderr):
        try:
            runner.main(["--prompt", "test", "--collection", "dataset"])
        except SystemExit as exc:
            assert exc.code == 2
        else:
            raise AssertionError(
                "runner.main(['--prompt', 'test', '--collection', 'dataset']) should fail"
            )
    assert "--dataset-id is required when --collection dataset." in dataset_stderr.getvalue()


if __name__ == "__main__":
    main()
