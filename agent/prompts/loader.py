from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from agent.workspace_docs import load_sdk_docs_reference as _load_sdk_docs_reference
from sdk._profiles import get_sdk_profile

DESIGNER_PROMPT_NAME = "designer_system_prompt.txt"
OPENAI_DESIGNER_PROMPT_NAME = "designer_system_prompt_openai.txt"
CODEX_CLI_DESIGNER_PROMPT_NAME = "designer_system_prompt_codex_cli.txt"
GEMINI_DESIGNER_PROMPT_NAME = "designer_system_prompt_gemini.txt"
OPENROUTER_DESIGNER_PROMPT_NAME = "designer_system_prompt_openrouter.txt"
ANTHROPIC_DESIGNER_PROMPT_NAME = "designer_system_prompt_anthropic.txt"
DEEPSEEK_DESIGNER_PROMPT_NAME = "designer_system_prompt_deepseek.txt"

PROMPTS_ROOT = Path(__file__).resolve().parent
GENERATED_PROMPTS_DIR = PROMPTS_ROOT / "generated"
SECTIONS_DIR = PROMPTS_ROOT / "sections"


def normalize_sdk_package(sdk_package: str) -> str:
    candidate = str(sdk_package or "sdk").strip().lower()
    return get_sdk_profile(candidate).package_name


def resolve_system_prompt_path(
    prompt_path: str,
    *,
    provider: str | None = None,
    sdk_package: str = "sdk",
    repo_root: Path | None = None,
) -> Path:
    root = repo_root or Path(__file__).resolve().parents[1]
    raw_path = Path(prompt_path)
    if raw_path.is_absolute():
        path = raw_path
    else:
        root_candidate = (root / raw_path).resolve()
        generated_candidate = (GENERATED_PROMPTS_DIR / raw_path.name).resolve()
        if root_candidate.exists() or len(raw_path.parts) > 1:
            path = root_candidate
        else:
            path = generated_candidate
    provider_norm = (provider or "").strip().lower()
    profile = get_sdk_profile(normalize_sdk_package(sdk_package))

    candidates: list[Path] = []
    default_names = {
        DESIGNER_PROMPT_NAME,
        OPENAI_DESIGNER_PROMPT_NAME,
        CODEX_CLI_DESIGNER_PROMPT_NAME,
        GEMINI_DESIGNER_PROMPT_NAME,
        OPENROUTER_DESIGNER_PROMPT_NAME,
        ANTHROPIC_DESIGNER_PROMPT_NAME,
        DEEPSEEK_DESIGNER_PROMPT_NAME,
    }
    profile_prompt_name = profile.prompt_name_for_provider(provider_norm)
    if path.name in default_names and profile_prompt_name is not None:
        candidates.append(path.with_name(profile_prompt_name))
    candidates.append(path)

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def load_system_prompt_text(
    prompt_path: str,
    *,
    provider: str | None = None,
    sdk_package: str = "sdk",
    repo_root: Path | None = None,
) -> tuple[Path, str]:
    prompt_file = resolve_system_prompt_path(
        prompt_path,
        provider=provider,
        sdk_package=sdk_package,
        repo_root=repo_root,
    )
    if not prompt_file.exists():
        raise FileNotFoundError(f"System prompt file not found: {prompt_path}")
    return prompt_file, prompt_file.read_text(encoding="utf-8")


def load_prompt_section_text(section_name: str) -> tuple[Path, str]:
    section_path = (SECTIONS_DIR / section_name).resolve()
    if not section_path.exists():
        raise FileNotFoundError(f"Prompt section file not found: {section_name}")
    return section_path, section_path.read_text(encoding="utf-8")


def load_sdk_docs_reference(
    repo_root: Path,
    *,
    sdk_package: str = "sdk",
) -> str:
    return _load_sdk_docs_reference(
        repo_root,
        sdk_package=sdk_package,
    )


@dataclass(slots=True, frozen=True)
class PromptFiles:
    """Resolved prompt-related files for a run."""

    system_prompt_path: Path
    sdk_docs_context: str
