from __future__ import annotations

DEFAULT_MAX_TURNS = 100
GEMINI_3_FLASH_DEFAULT_MAX_TURNS = 250


def is_gemini_3_flash_model(model_id: str | None) -> bool:
    normalized = (model_id or "").strip().lower()
    return normalized.startswith("gemini-3") and "flash" in normalized


def default_max_turns_for_model(model_id: str | None) -> int:
    if is_gemini_3_flash_model(model_id):
        return GEMINI_3_FLASH_DEFAULT_MAX_TURNS
    return DEFAULT_MAX_TURNS


def resolve_max_turns(*, model_id: str | None, max_turns: int | None) -> int:
    if max_turns is None:
        return default_max_turns_for_model(model_id)
    return int(max_turns)
