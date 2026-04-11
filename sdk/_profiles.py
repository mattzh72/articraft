from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

OPENAI_DESIGNER_PROMPT_NAME = "designer_system_prompt_openai.txt"
GEMINI_DESIGNER_PROMPT_NAME = "designer_system_prompt_gemini.txt"
ScaffoldMode = Literal["lite", "strict"]
DEFAULT_SCAFFOLD_MODE: ScaffoldMode = "lite"
LEGACY_SCAFFOLD_MODE: ScaffoldMode = "strict"


@dataclass(slots=True, frozen=True)
class SdkProfile:
    package_name: str
    scaffold_paths: dict[ScaffoldMode, Path]
    docs_full: tuple[Path, ...]
    docs_core: tuple[Path, ...]
    openai_prompt_name: str
    gemini_prompt_name: str

    def scaffold_path_for_mode(self, scaffold_mode: ScaffoldMode) -> Path:
        try:
            return self.scaffold_paths[scaffold_mode]
        except KeyError as exc:
            raise ValueError(f"Unsupported scaffold mode: {scaffold_mode!r}") from exc

    def docs_for_mode(self, docs_mode: str) -> tuple[Path, ...]:
        if docs_mode == "full":
            return self.docs_full
        if docs_mode == "core":
            return self.docs_core
        if docs_mode == "none":
            return ()
        raise ValueError(f"Unsupported SDK docs mode: {docs_mode!r}")

    def prompt_name_for_provider(self, provider: str | None) -> str | None:
        provider_norm = (provider or "").strip().lower()
        if provider_norm == "openai":
            return self.openai_prompt_name
        if provider_norm == "gemini":
            return self.gemini_prompt_name
        return None


_COMMON_DOCS = (
    Path("sdk/_docs/common/00_quickstart.md"),
    Path("sdk/_docs/common/10_errors.md"),
    Path("sdk/_docs/common/20_core_types.md"),
    Path("sdk/_docs/common/30_articulated_object.md"),
    Path("sdk/_docs/common/40_assets.md"),
    Path("sdk/_docs/common/50_placement.md"),
    Path("sdk/_docs/common/70_probe_tooling.md"),
    Path("sdk/_docs/common/80_testing.md"),
)

_BASE_DOCS = (
    Path("sdk/_docs/base/40_mesh_geometry.md"),
    Path("sdk/_docs/base/45_wires.md"),
    Path("sdk/_docs/base/46_section_lofts.md"),
)

_CADQUERY_DOCS = (
    Path("sdk/_docs/cadquery/35_cadquery.md"),
    Path("sdk/_docs/cadquery/36_cadquery_primer.md"),
    Path("sdk/_docs/cadquery/37_cadquery_workplane.md"),
    Path("sdk/_docs/cadquery/38_cadquery_sketch.md"),
    Path("sdk/_docs/cadquery/39_cadquery_assembly.md"),
    Path("sdk/_docs/cadquery/39d_cadquery_gears.md"),
    Path("sdk/_docs/cadquery/39b_cadquery_free_function.md"),
    Path("sdk/_docs/cadquery/39c_cadquery_api_ref.md"),
)

_SDK_PACKAGE_ALIASES = {
    "": "sdk",
    "base": "sdk",
    "sdk": "sdk",
    "sdk_hybrid": "sdk",
}


SDK_PROFILES: dict[str, SdkProfile] = {
    "sdk": SdkProfile(
        package_name="sdk",
        scaffold_paths={
            "lite": Path("scaffold_lite.py"),
            "strict": Path("scaffold.py"),
        },
        docs_full=_COMMON_DOCS[:4] + _BASE_DOCS + _CADQUERY_DOCS + _COMMON_DOCS[4:],
        docs_core=(
            Path("sdk/_docs/common/00_quickstart.md"),
            Path("sdk/_docs/cadquery/35_cadquery.md"),
            Path("sdk/_docs/common/70_probe_tooling.md"),
            Path("sdk/_docs/common/80_testing.md"),
        ),
        openai_prompt_name=OPENAI_DESIGNER_PROMPT_NAME,
        gemini_prompt_name=GEMINI_DESIGNER_PROMPT_NAME,
    ),
}


SUPPORTED_SDK_PACKAGES = frozenset(SDK_PROFILES)
SUPPORTED_SCAFFOLD_MODES = frozenset({"lite", "strict"})


def normalize_scaffold_mode(scaffold_mode: str) -> ScaffoldMode:
    candidate = str(scaffold_mode or "").strip().lower()
    if candidate in SUPPORTED_SCAFFOLD_MODES:
        return candidate  # type: ignore[return-value]
    raise ValueError(
        f"Unsupported scaffold mode: {scaffold_mode!r}. "
        f"Expected one of {sorted(SUPPORTED_SCAFFOLD_MODES)!r}."
    )


def get_sdk_profile(package_name: str) -> SdkProfile:
    normalized = _SDK_PACKAGE_ALIASES.get(str(package_name or "").strip().lower())
    if normalized is None:
        raise ValueError(
            f"Unsupported SDK package: {package_name!r}. "
            f"Expected one of {sorted(SUPPORTED_SDK_PACKAGES)!r}."
        )
    try:
        return SDK_PROFILES[normalized]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported SDK package: {package_name!r}. "
            f"Expected one of {sorted(SUPPORTED_SDK_PACKAGES)!r}."
        ) from exc
