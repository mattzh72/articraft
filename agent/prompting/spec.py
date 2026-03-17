from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SECTIONS_DIR = ROOT / "sections"
GENERATED_DIR = ROOT / "generated"


@dataclass(frozen=True)
class PromptVariant:
    name: str
    sections: tuple[Path, ...]
    output: Path
    description: str


PROMPT_VARIANTS: tuple[PromptVariant, ...] = (
    PromptVariant(
        name="designer_openai",
        sections=(
            SECTIONS_DIR / "designer_common.md",
            SECTIONS_DIR / "provider_openai.md",
            SECTIONS_DIR / "feedback_rules.md",
            SECTIONS_DIR / "sdk_base.md",
        ),
        output=GENERATED_DIR / "designer_system_prompt_openai.txt",
        description="OpenAI designer prompt for the base SDK.",
    ),
    PromptVariant(
        name="designer_openai_hybrid",
        sections=(
            SECTIONS_DIR / "designer_common.md",
            SECTIONS_DIR / "provider_openai.md",
            SECTIONS_DIR / "feedback_rules.md",
            SECTIONS_DIR / "sdk_hybrid.md",
        ),
        output=GENERATED_DIR / "designer_system_prompt_openai_hybrid.txt",
        description="OpenAI designer prompt for the hybrid SDK.",
    ),
    PromptVariant(
        name="designer_gemini",
        sections=(
            SECTIONS_DIR / "designer_common.md",
            SECTIONS_DIR / "provider_gemini.md",
            SECTIONS_DIR / "feedback_rules.md",
            SECTIONS_DIR / "sdk_base.md",
        ),
        output=GENERATED_DIR / "designer_system_prompt_gemini.txt",
        description="Gemini designer prompt for the base SDK.",
    ),
    PromptVariant(
        name="designer_gemini_hybrid",
        sections=(
            SECTIONS_DIR / "designer_common.md",
            SECTIONS_DIR / "provider_gemini.md",
            SECTIONS_DIR / "feedback_rules.md",
            SECTIONS_DIR / "sdk_hybrid.md",
        ),
        output=GENERATED_DIR / "designer_system_prompt_gemini_hybrid.txt",
        description="Gemini designer prompt for the hybrid SDK.",
    ),
)


def iter_prompt_variants() -> tuple[PromptVariant, ...]:
    return PROMPT_VARIANTS
