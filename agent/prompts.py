from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from sdk._profiles import SUPPORTED_SDK_PACKAGES, get_sdk_profile


LEGACY_DESIGNER_PROMPT_NAME = "system_prompt.txt"
DESIGNER_PROMPT_NAME = "designer_system_prompt.txt"
OPENAI_DESIGNER_PROMPT_NAME = "designer_system_prompt_openai.txt"
HYBRID_OPENAI_DESIGNER_PROMPT_NAME = "designer_system_prompt_openai_hybrid.txt"
GEMINI_DESIGNER_PROMPT_NAME = "designer_system_prompt_gemini.txt"
HYBRID_GEMINI_DESIGNER_PROMPT_NAME = "designer_system_prompt_gemini_hybrid.txt"

SUPPORTED_SDK_DOCS_MODES = {"full", "core", "none"}
PROMPTING_ROOT = Path(__file__).resolve().parent / "prompting"
GENERATED_PROMPTS_DIR = PROMPTING_ROOT / "generated"


def normalize_sdk_package(sdk_package: str) -> str:
    candidate = (sdk_package or "sdk").strip()
    get_sdk_profile(candidate)
    return candidate


def normalize_sdk_docs_mode(docs_mode: str) -> str:
    candidate = (docs_mode or "full").strip().lower()
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
        return "sdk_hybrid"
    return normalize_sdk_package(default_sdk_package)


def _select_sdk_doc_paths(
    repo_root: Path,
    *,
    sdk_package: str,
    docs_mode: str,
) -> list[Path]:
    profile = get_sdk_profile(normalize_sdk_package(sdk_package))
    mode = normalize_sdk_docs_mode(docs_mode)
    if mode == "none":
        return []

    selected: list[Path] = []
    for rel_path in profile.docs_for_mode(mode):
        path = repo_root / rel_path
        if path.exists():
            selected.append(path)
    return selected


def load_sdk_docs_reference(
    repo_root: Path,
    *,
    sdk_package: str = "sdk",
    docs_mode: str = "full",
) -> str:
    package = normalize_sdk_package(sdk_package)
    mode = normalize_sdk_docs_mode(docs_mode)
    doc_paths = _select_sdk_doc_paths(
        repo_root,
        sdk_package=package,
        docs_mode=mode,
    )
    if not doc_paths:
        return ""

    parts: list[str] = []
    parts.append("\n\n# SDK Documentation (read-only)\n")
    parts.append(
        f"The selected SDK package for this run is `{package}`. "
        f"When you write imports, import from `{package}`.\n"
    )
    parts.append(
        "The content below is the SDK user documentation available in this repository. "
        "Use it for recommended usage patterns and best practices.\n"
    )
    if mode == "core":
        parts.append(
            "Only the core SDK docs subset is injected for speed. "
            "It focuses on quickstart, testing, and package-specific essentials.\n"
        )
    for path in doc_paths:
        rel = path.relative_to(repo_root).as_posix()
        content = path.read_text(encoding="utf-8")
        parts.append(f"\n## {rel}\n````markdown\n{content}\n````\n")

    return "".join(parts)


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
        HYBRID_OPENAI_DESIGNER_PROMPT_NAME,
        GEMINI_DESIGNER_PROMPT_NAME,
        HYBRID_GEMINI_DESIGNER_PROMPT_NAME,
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


def provider_system_prompt_suffix(provider: str, *, sdk_package: str = "sdk") -> str:
    normalize_sdk_package(sdk_package)
    return ""


@dataclass(slots=True, frozen=True)
class PromptFiles:
    """Resolved prompt-related files for a run."""

    system_prompt_path: Path
    sdk_docs_context: str
