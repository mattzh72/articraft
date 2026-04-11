from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from agent.workspace_docs import load_sdk_docs_reference as _load_sdk_docs_reference
from sdk._profiles import get_sdk_profile

LEGACY_DESIGNER_PROMPT_NAME = "system_prompt.txt"
DESIGNER_PROMPT_NAME = "designer_system_prompt.txt"
OPENAI_DESIGNER_PROMPT_NAME = "designer_system_prompt_openai.txt"
GEMINI_DESIGNER_PROMPT_NAME = "designer_system_prompt_gemini.txt"

SUPPORTED_SDK_DOCS_MODES = {"full", "core", "none"}
LEGACY_SDK_DOCS_MODE_ALIASES = {
    "legacy_import": "full",
}
PROMPTS_ROOT = Path(__file__).resolve().parent
GENERATED_PROMPTS_DIR = PROMPTS_ROOT / "generated"
SECTIONS_DIR = PROMPTS_ROOT / "sections"
# Back-compat alias for older references.
PROMPTING_ROOT = PROMPTS_ROOT


def normalize_sdk_package(sdk_package: str) -> str:
    candidate = str(sdk_package or "sdk").strip().lower()
    return get_sdk_profile(candidate).package_name


def normalize_sdk_docs_mode(docs_mode: str) -> str:
    candidate = (docs_mode or "full").strip().lower()
    candidate = LEGACY_SDK_DOCS_MODE_ALIASES.get(candidate, candidate)
    if candidate not in SUPPORTED_SDK_DOCS_MODES:
        raise ValueError(
            f"Unsupported SDK docs mode: {docs_mode!r}. "
            f"Expected one of {sorted(SUPPORTED_SDK_DOCS_MODES)!r}."
        )
    return candidate


def resolve_sdk_package_flags(
    *,
    hybrid_sdk: bool = False,
    default_sdk_package: str = "sdk",
) -> str:
    if hybrid_sdk:
        return "sdk"
    return normalize_sdk_package(default_sdk_package)


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
        LEGACY_DESIGNER_PROMPT_NAME,
        DESIGNER_PROMPT_NAME,
        OPENAI_DESIGNER_PROMPT_NAME,
        GEMINI_DESIGNER_PROMPT_NAME,
    }
    profile_prompt_name = profile.prompt_name_for_provider(provider_norm)
    if path.name in default_names and profile_prompt_name is not None:
        candidates.append(path.with_name(profile_prompt_name))
    candidates.append(path)

    if path.name == LEGACY_DESIGNER_PROMPT_NAME:
        candidates.append(path.with_name(DESIGNER_PROMPT_NAME))

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
    docs_mode: str = "full",
) -> str:
    return _load_sdk_docs_reference(
        repo_root,
        sdk_package=sdk_package,
        docs_mode=docs_mode,
    )


def provider_system_prompt_suffix(provider: str, *, sdk_package: str = "sdk") -> str:
    normalize_sdk_package(sdk_package)
    return ""


@dataclass(slots=True, frozen=True)
class PromptFiles:
    """Resolved prompt-related files for a run."""

    system_prompt_path: Path
    sdk_docs_context: str
