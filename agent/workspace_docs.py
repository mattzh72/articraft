from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from sdk._profiles import get_sdk_profile

_SDK_ROUTER_SOURCE = Path("sdk/_docs/common/00_quickstart.md")
_SDK_ROUTER_VIRTUAL_PATH = "docs/sdk/references/quickstart.md"
_MODEL_VIRTUAL_PATH = "model.py"
_DEFAULT_PRELOAD_PATHS = (
    "docs/sdk/references/quickstart.md",
    "docs/sdk/references/probe-tooling.md",
    "docs/sdk/references/testing.md",
)


@dataclass(frozen=True)
class VirtualWorkspaceFile:
    virtual_path: str
    disk_path: Path | None = None
    content: str | None = None

    def read_text(self) -> str:
        if self.content is not None:
            return self.content
        if self.disk_path is None:
            raise FileNotFoundError(f"No backing content for {self.virtual_path}")
        return self.disk_path.read_text(encoding="utf-8")


@dataclass(frozen=True)
class DocsBundle:
    router: VirtualWorkspaceFile
    files_by_path: dict[str, VirtualWorkspaceFile]

    def read_text(self, virtual_path: str) -> str:
        return self.resolve(virtual_path).read_text()

    def resolve(self, virtual_path: str) -> VirtualWorkspaceFile:
        normalized = normalize_virtual_workspace_path(virtual_path)
        try:
            return self.files_by_path[normalized]
        except KeyError as exc:
            raise FileNotFoundError(f"Unknown virtual docs path: {normalized}") from exc

    def default_read_virtual_paths(self) -> tuple[str, ...]:
        return _DEFAULT_PRELOAD_PATHS


@dataclass(frozen=True)
class VirtualWorkspace:
    model_file_path: Path
    docs_bundle: DocsBundle

    def resolve(self, virtual_path: str) -> VirtualWorkspaceFile:
        normalized = normalize_virtual_workspace_path(virtual_path)
        if normalized == _MODEL_VIRTUAL_PATH:
            return VirtualWorkspaceFile(
                virtual_path=_MODEL_VIRTUAL_PATH,
                disk_path=self.model_file_path,
            )
        return self.docs_bundle.resolve(normalized)


def normalize_virtual_workspace_path(path: str) -> str:
    candidate = (path or "").strip().replace("\\", "/")
    if not candidate:
        raise ValueError("path must not be empty")
    if candidate.startswith("/"):
        raise ValueError("path must be a virtual workspace path, not an absolute path")
    parts = [part for part in candidate.split("/") if part not in {"", "."}]
    if not parts:
        raise ValueError("path must not be empty")
    if any(part == ".." for part in parts):
        raise ValueError("path must not contain '..'")
    return "/".join(parts)


def build_virtual_workspace(
    repo_root: Path,
    *,
    model_file_path: Path,
    sdk_package: str,
) -> VirtualWorkspace:
    return VirtualWorkspace(
        model_file_path=model_file_path,
        docs_bundle=load_sdk_docs_bundle(repo_root, sdk_package=sdk_package),
    )


def load_sdk_docs_reference(
    repo_root: Path,
    *,
    sdk_package: str = "sdk",
    docs_mode: str = "full",
) -> str:
    # Detailed docs now live behind `read_file(path=...)`, so the preloaded bundle stays small
    # regardless of the legacy docs_mode setting.
    del docs_mode
    bundle = load_sdk_docs_bundle(repo_root, sdk_package=sdk_package)
    preload_paths = bundle.default_read_virtual_paths()

    parts = [
        "\n\n# Workspace Documentation (read-only)\n",
        "The virtual workspace exposes `model.py` as the editable artifact script and `docs/` "
        "as read-only SDK guidance.\n",
        "`docs/sdk/references/quickstart.md` is the preloaded SDK entrypoint and reference index.\n",
        "Use `read_file(path=...)` with these virtual paths when you need exact text.\n",
    ]
    for virtual_path in preload_paths:
        parts.append(f"\n## {virtual_path}\n````markdown\n{bundle.read_text(virtual_path)}\n````\n")
    return "".join(parts)


def load_sdk_docs_bundle(repo_root: Path, *, sdk_package: str) -> DocsBundle:
    router = _load_router_document(repo_root)
    files_by_path: dict[str, VirtualWorkspaceFile] = {
        router.router.virtual_path: router.router,
        **_build_sdk_reference_files(repo_root, sdk_package=sdk_package),
    }
    return DocsBundle(
        router=router.router,
        files_by_path=files_by_path,
    )


@dataclass(frozen=True)
class _LoadedRouter:
    router: VirtualWorkspaceFile


def _load_router_document(repo_root: Path) -> _LoadedRouter:
    path = repo_root / _SDK_ROUTER_SOURCE
    if not path.exists():
        raise FileNotFoundError(f"SDK quickstart document not found: {path}")
    return _LoadedRouter(
        router=VirtualWorkspaceFile(
            virtual_path=_SDK_ROUTER_VIRTUAL_PATH,
            disk_path=path,
        ),
    )


def _build_sdk_reference_files(
    repo_root: Path,
    *,
    sdk_package: str,
) -> dict[str, VirtualWorkspaceFile]:
    profile = get_sdk_profile(sdk_package)
    files: dict[str, VirtualWorkspaceFile] = {}

    for rel_path in profile.docs_full:
        rel_str = rel_path.as_posix()
        virtual_suffix = _DOC_PATH_ALIASES.get(rel_str)
        if virtual_suffix is None:
            continue
        files[f"docs/sdk/{virtual_suffix}"] = VirtualWorkspaceFile(
            virtual_path=f"docs/sdk/{virtual_suffix}",
            disk_path=repo_root / rel_path,
        )
    return files


def _resolve_sdk_docs_relative_path(path: str) -> str:
    normalized = normalize_virtual_workspace_path(path)
    if normalized.startswith("docs/"):
        return normalized
    if normalized == _MODEL_VIRTUAL_PATH:
        return normalized
    return f"docs/sdk/{normalized}"


_DOC_PATH_ALIASES = {
    "sdk/_docs/common/00_quickstart.md": "references/quickstart.md",
    "sdk/_docs/common/10_errors.md": "references/errors.md",
    "sdk/_docs/common/20_core_types.md": "references/core-types.md",
    "sdk/_docs/common/30_articulated_object.md": "references/articulated-object.md",
    "sdk/_docs/common/40_assets.md": "references/assets.md",
    "sdk/_docs/common/50_placement.md": "references/placement.md",
    "sdk/_docs/common/70_probe_tooling.md": "references/probe-tooling.md",
    "sdk/_docs/common/80_testing.md": "references/testing.md",
    "sdk/_docs/base/40_mesh_geometry.md": "references/geometry/mesh-geometry.md",
    "sdk/_docs/base/45_wires.md": "references/geometry/wires.md",
    "sdk/_docs/base/46_section_lofts.md": "references/geometry/section-lofts.md",
    "sdk/_docs/cadquery/35_cadquery.md": "references/cadquery/overview.md",
    "sdk/_docs/cadquery/36_cadquery_primer.md": "references/cadquery/primer.md",
    "sdk/_docs/cadquery/37_cadquery_workplane.md": "references/cadquery/workplane.md",
    "sdk/_docs/cadquery/38_cadquery_sketch.md": "references/cadquery/sketch.md",
    "sdk/_docs/cadquery/39_cadquery_assembly.md": "references/cadquery/assembly.md",
    "sdk/_docs/cadquery/39d_cadquery_gears.md": "references/cadquery/gears.md",
    "sdk/_docs/cadquery/39b_cadquery_free_function.md": "references/cadquery/free-functions.md",
    "sdk/_docs/cadquery/39c_cadquery_api_ref.md": "references/cadquery/api-ref.md",
}
