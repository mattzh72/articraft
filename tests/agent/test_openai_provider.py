from __future__ import annotations

from agent.providers.openai import openai_api_key_from_env, openai_api_keys_from_env


def test_openai_api_keys_from_env_prefers_primary_key_and_dedupes() -> None:
    keys = openai_api_keys_from_env(
        {
            "OPENAI_API_KEY": "sk-primary",
            "OPENAI_API_KEYS": "sk-primary,\nsk-secondary, sk-third, sk-secondary",
        }
    )

    assert keys == ["sk-primary", "sk-secondary", "sk-third"]


def test_openai_api_key_from_env_uses_key_pool_when_primary_missing() -> None:
    key = openai_api_key_from_env({"OPENAI_API_KEYS": "sk-first,sk-second"})

    assert key in {"sk-first", "sk-second"}
