from __future__ import annotations

from scripts.bootstrap_env import bootstrap_env

OPENAI_API_KEY_NAME = "".join(["OPENAI_API_", "KEY"])
OPENAI_API_KEYS_NAME = "".join(["OPENAI_API_", "KEYS"])
GEMINI_API_KEYS_NAME = "".join(["GEMINI_API_", "KEYS"])


def test_bootstrap_env_imports_supported_keys_from_shell_environment(tmp_path) -> None:
    (tmp_path / ".env.example").write_text(
        f"{OPENAI_API_KEY_NAME}=\n{GEMINI_API_KEYS_NAME}=\nGOOGLE_CLOUD_PROJECT=\n",
        encoding="utf-8",
    )

    created, imported_keys = bootstrap_env(
        tmp_path,
        environ={
            OPENAI_API_KEYS_NAME: "sk-a,sk-b",
            OPENAI_API_KEY_NAME: "sk-primary",
            GEMINI_API_KEYS_NAME: "gem-a,gem-b",
            "GOOGLE_CLOUD_PROJECT": "articraft-dev",
            "GOOGLE_APPLICATION_CREDENTIALS": "/tmp/my creds.json",
            "GOOGLE_CLOUD_LOCATION": "europe-west2",
        },
    )

    assert created is True
    assert imported_keys == [
        OPENAI_API_KEY_NAME,
        GEMINI_API_KEYS_NAME,
        "GOOGLE_CLOUD_PROJECT",
        OPENAI_API_KEYS_NAME,
        "GOOGLE_APPLICATION_CREDENTIALS",
        "GOOGLE_CLOUD_LOCATION",
    ]
    assert (tmp_path / ".env").read_text(encoding="utf-8") == (
        f"{OPENAI_API_KEY_NAME}=sk-primary\n"
        f"{GEMINI_API_KEYS_NAME}=gem-a,gem-b\n"
        "GOOGLE_CLOUD_PROJECT=articraft-dev\n"
        f"{OPENAI_API_KEYS_NAME}=sk-a,sk-b\n"
        "GOOGLE_APPLICATION_CREDENTIALS='/tmp/my creds.json'\n"
        "GOOGLE_CLOUD_LOCATION=europe-west2\n"
    )


def test_bootstrap_env_does_not_overwrite_existing_env_file(tmp_path) -> None:
    (tmp_path / ".env.example").write_text(f"{OPENAI_API_KEY_NAME}=\n", encoding="utf-8")
    (tmp_path / ".env").write_text(f"{OPENAI_API_KEY_NAME}=existing\n", encoding="utf-8")

    created, imported_keys = bootstrap_env(
        tmp_path,
        environ={OPENAI_API_KEY_NAME: "sk-replacement"},
    )

    assert created is False
    assert imported_keys == []
    assert (tmp_path / ".env").read_text(encoding="utf-8") == f"{OPENAI_API_KEY_NAME}=existing\n"
