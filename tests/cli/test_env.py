from __future__ import annotations

from cli.env import bootstrap_env


def test_bootstrap_env_imports_generation_defaults(tmp_path) -> None:
    (tmp_path / ".env.example").write_text(
        "ARTICRAFT_MODEL=\nARTICRAFT_THINKING_LEVEL=\nARTICRAFT_MAX_COST_USD=\n",
        encoding="utf-8",
    )
    values = {
        "ARTICRAFT_MODEL": "gpt-5.5",
        "ARTICRAFT_THINKING_LEVEL": "xhigh",
        "ARTICRAFT_MAX_COST_USD": "1.25",
    }

    created, imported_keys = bootstrap_env(tmp_path, environ=values)

    assert created is True
    assert imported_keys == [
        "ARTICRAFT_MODEL",
        "ARTICRAFT_THINKING_LEVEL",
        "ARTICRAFT_MAX_COST_USD",
    ]
    assert (tmp_path / ".env").read_text(encoding="utf-8") == (
        "ARTICRAFT_MODEL=gpt-5.5\nARTICRAFT_THINKING_LEVEL=xhigh\nARTICRAFT_MAX_COST_USD=1.25\n"
    )
