from __future__ import annotations

import contextlib
import contextvars
import hashlib
import os
import re
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Iterator, Optional, Union

from .errors import ValidationError

if TYPE_CHECKING:
    from .types import Mesh

_LEGACY_MESH_PREFIX = "meshes/"
_DEFAULT_MESH_SUBDIR = "assets/meshes"
_CURRENT_ASSET_SESSION: contextvars.ContextVar[AssetSession | None] = contextvars.ContextVar(
    "articraft_asset_session",
    default=None,
)


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    return _sha256_bytes(path.read_bytes())


def _short_digest(digest: str) -> str:
    return digest[:12]


def _normalize_mesh_subdir(mesh_subdir: str) -> str:
    normalized = str(mesh_subdir or _DEFAULT_MESH_SUBDIR).strip().strip("/\\")
    if not normalized:
        raise ValueError("mesh_subdir must be non-empty")
    return normalized


def _slugify_asset_name(name: str) -> str:
    raw = str(name or "").strip()
    if not raw:
        raise ValidationError("Managed mesh name must be non-empty")
    stem = Path(raw).stem if Path(raw).suffix else raw
    cleaned = re.sub(r"[^a-z0-9_]+", "-", stem.lower())
    slug = cleaned.strip("-_")
    if not slug:
        raise ValidationError(f"Managed mesh name produced an empty slug: {name!r}")
    return slug


def _candidate_input_names(name: str) -> tuple[str, ...]:
    raw = str(name or "").strip()
    if not raw:
        raise ValidationError("Input mesh name must be non-empty")
    if Path(raw).name != raw:
        raise ValidationError(f"Input mesh name must not include path segments: {name!r}")
    if raw.lower().endswith(".obj"):
        return (raw,)
    return (raw, f"{raw}.obj")


def _infer_inputs_root_from_script_path(script_path: Path) -> Path | None:
    script_path = Path(script_path).resolve()
    parent = script_path.parent

    direct_inputs = parent / "inputs"
    if direct_inputs.exists():
        return direct_inputs

    parts = script_path.parts
    for index in range(len(parts) - 2):
        if parts[index : index + 2] != ("data", "records"):
            continue
        if index + 3 >= len(parts):
            continue
        record_id = parts[index + 2]
        if record_id and parts[index + 3] == script_path.name:
            repo_root = Path(*parts[:index]) if index > 0 else Path(parts[0])
            return repo_root / "data" / "records" / record_id / "inputs"

    for index in range(len(parts) - 5):
        if parts[index : index + 4] != ("data", "cache", "runs", parts[index + 3]):
            continue
        if index + 6 >= len(parts):
            continue
        if parts[index + 4] != "staging":
            continue
        record_id = parts[index + 5]
        repo_root = Path(*parts[:index]) if index > 0 else Path(parts[0])
        return repo_root / "data" / "records" / record_id / "inputs"

    return None


@dataclass(slots=True, frozen=True)
class ManagedMeshInfo:
    logical_name: str
    slug: str
    ref: str
    path: Path
    digest: str


@dataclass(slots=True)
class AssetSession:
    root: Path
    inputs_root: Path | None = None
    mesh_subdir: str = _DEFAULT_MESH_SUBDIR
    _managed_meshes: dict[tuple[str, str], ManagedMeshInfo] = field(
        default_factory=dict, init=False, repr=False
    )
    _refs: dict[str, ManagedMeshInfo] = field(default_factory=dict, init=False, repr=False)
    _temp_dir: tempfile.TemporaryDirectory[str] | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        self.root = Path(self.root).resolve()
        self.mesh_subdir = _normalize_mesh_subdir(self.mesh_subdir)
        self.inputs_root = (
            Path(self.inputs_root).resolve() if self.inputs_root is not None else None
        )

    @classmethod
    def from_script(cls, script_path: str | Path) -> AssetSession:
        resolved_script = Path(script_path).resolve()
        return cls(
            resolved_script.parent,
            inputs_root=_infer_inputs_root_from_script_path(resolved_script),
        )

    @classmethod
    def ephemeral(cls) -> AssetSession:
        tmp = tempfile.TemporaryDirectory(prefix="articraft-assets-")
        session = cls(Path(tmp.name))
        session._temp_dir = tmp
        return session

    @property
    def asset_root(self) -> Path:
        return self.root

    @property
    def mesh_dir(self) -> Path:
        return self.root / self.mesh_subdir

    def ensure_mesh_dir(self) -> Path:
        self.mesh_dir.mkdir(parents=True, exist_ok=True)
        return self.mesh_dir

    def mesh_ref(self, filename: str | Path) -> str:
        path = Path(filename)
        if path.is_absolute():
            try:
                rel = path.resolve().relative_to(self.root)
            except ValueError:
                return path.as_posix()
        else:
            normalized = path.as_posix()
            if normalized.startswith(f"{self.mesh_subdir}/"):
                rel = path
            elif normalized.startswith(_LEGACY_MESH_PREFIX):
                rel = Path(self.mesh_subdir) / normalized[len(_LEGACY_MESH_PREFIX) :]
            else:
                rel = Path(self.mesh_subdir) / path
        return rel.as_posix()

    def mesh_path(
        self,
        filename: str | Path,
        *,
        ensure_dir: bool = True,
    ) -> Path:
        path = Path(filename)
        if not path.is_absolute():
            normalized = path.as_posix()
            if normalized.startswith(f"{self.mesh_subdir}/"):
                path = self.root / path
            elif normalized.startswith(_LEGACY_MESH_PREFIX):
                path = self.mesh_dir / normalized[len(_LEGACY_MESH_PREFIX) :]
            else:
                path = self.mesh_dir / path
        if ensure_dir:
            path.parent.mkdir(parents=True, exist_ok=True)
        return path

    def managed_mesh_ref(self, logical_name: str, *, suffix: str = ".obj") -> str:
        return f"{self.mesh_subdir}/{_slugify_asset_name(logical_name)}{suffix}"

    def _managed_mesh_path(
        self, logical_name: str, *, suffix: str = ".obj"
    ) -> tuple[str, Path, str]:
        slug = _slugify_asset_name(logical_name)
        ref = f"{self.mesh_subdir}/{slug}{suffix}"
        path = (self.root / ref).resolve()
        return ref, path, slug

    def _managed_mesh_info(
        self,
        logical_name: str,
        *,
        slug: str,
        ref: str,
        path: Path,
        digest: str,
    ) -> ManagedMeshInfo:
        return ManagedMeshInfo(
            logical_name=logical_name,
            slug=slug,
            ref=ref,
            path=path,
            digest=digest,
        )

    def _managed_mesh_conflict_candidates(
        self,
        slug: str,
        digest: str,
        *,
        suffix: str = ".obj",
    ) -> Iterator[tuple[str, Path, int]]:
        seen_lengths: set[int] = set()
        for length in (12, 16, 20, len(digest)):
            resolved_length = min(length, len(digest))
            if resolved_length in seen_lengths:
                continue
            seen_lengths.add(resolved_length)
            ref = f"{self.mesh_subdir}/{slug}--{digest[:resolved_length]}{suffix}"
            yield ref, (self.root / ref).resolve(), resolved_length

    def _resolve_managed_mesh_target(
        self,
        logical_name: str,
        *,
        slug: str,
        digest: str,
        payload: bytes,
        suffix: str = ".obj",
    ) -> tuple[str, Path]:
        base_ref = f"{self.mesh_subdir}/{slug}{suffix}"
        base_path = (self.root / base_ref).resolve()
        ref_owner = self._refs.get(base_ref)
        if ref_owner is not None:
            if ref_owner.digest == digest:
                return base_ref, base_path
        else:
            if base_path.exists():
                existing_digest = _sha256_file(base_path)
                if existing_digest == digest:
                    return base_ref, base_path
                # Treat on-disk conflicts without an in-session owner as stale prior-run files.
                base_path.parent.mkdir(parents=True, exist_ok=True)
                base_path.write_bytes(payload)
                return base_ref, base_path
            base_path.parent.mkdir(parents=True, exist_ok=True)
            base_path.write_bytes(payload)
            return base_ref, base_path

        for ref, path, digest_length in self._managed_mesh_conflict_candidates(
            slug,
            digest,
            suffix=suffix,
        ):
            ref_owner = self._refs.get(ref)
            if ref_owner is not None:
                if ref_owner.digest == digest:
                    return ref, path
                continue
            if path.exists():
                existing_digest = _sha256_file(path)
                if existing_digest == digest:
                    return ref, path
                if digest_length < len(digest):
                    continue
                raise ValidationError(
                    "Managed mesh content-addressed path conflict for "
                    f"{logical_name!r}: ref={ref!r} path={path} "
                    f"existing_digest={_short_digest(existing_digest)} "
                    f"new_digest={_short_digest(digest)}"
                )
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(payload)
            return ref, path

        raise ValidationError(
            f"Failed to allocate managed mesh path for {logical_name!r}: "
            f"slug={slug!r} digest={_short_digest(digest)}"
        )

    def _dedupe_or_write(
        self,
        logical_name: str,
        *,
        payload: bytes,
        suffix: str = ".obj",
    ) -> ManagedMeshInfo:
        if suffix.lower() != ".obj":
            raise ValidationError(f"Managed meshes must use .obj payloads (got: {suffix!r})")

        digest = _sha256_bytes(payload)
        existing = self._managed_meshes.get((logical_name, digest))
        if existing is not None:
            return existing

        _base_ref, _base_path, slug = self._managed_mesh_path(logical_name, suffix=suffix)
        ref, path = self._resolve_managed_mesh_target(
            logical_name,
            slug=slug,
            digest=digest,
            payload=payload,
            suffix=suffix,
        )
        info = self._managed_mesh_info(
            logical_name,
            slug=slug,
            ref=ref,
            path=path,
            digest=digest,
        )
        self._managed_meshes[(logical_name, digest)] = info
        self._refs.setdefault(ref, info)
        return info

    def register_mesh_text(
        self,
        logical_name: str,
        payload: str,
        *,
        suffix: str = ".obj",
    ) -> ManagedMeshInfo:
        return self._dedupe_or_write(logical_name, payload=payload.encode("utf-8"), suffix=suffix)

    def register_mesh_file(self, logical_name: str, source: Path) -> ManagedMeshInfo:
        source_path = Path(source).resolve()
        if not source_path.exists() or not source_path.is_file():
            raise ValidationError(f"Managed mesh source file not found: {source_path}")
        if source_path.suffix.lower() != ".obj":
            raise ValidationError(
                f"Only OBJ meshes are supported for managed assets (got: {source_path.name!r})"
            )
        return self._dedupe_or_write(
            logical_name,
            payload=source_path.read_bytes(),
            suffix=source_path.suffix.lower(),
        )

    def input_mesh_source(self, name: str) -> Path:
        if self.inputs_root is None:
            raise ValidationError(f"No managed input catalog is available for {name!r}")
        for candidate in _candidate_input_names(name):
            path = (self.inputs_root / candidate).resolve()
            try:
                path.relative_to(self.inputs_root.resolve())
            except ValueError as exc:
                raise ValidationError(f"Invalid managed input mesh name: {name!r}") from exc
            if path.exists() and path.is_file():
                if path.suffix.lower() != ".obj":
                    raise ValidationError(
                        f"Only OBJ input meshes are supported (got: {path.name!r})"
                    )
                return path
        raise ValidationError(f"Managed input mesh not found: {name!r}")

    def register_input_mesh(self, logical_name: str) -> ManagedMeshInfo:
        return self.register_mesh_file(logical_name, self.input_mesh_source(logical_name))


@dataclass(frozen=True)
class AssetContext:
    root: Path
    mesh_subdir: str = _DEFAULT_MESH_SUBDIR

    def __post_init__(self) -> None:
        object.__setattr__(self, "root", Path(self.root).resolve())
        object.__setattr__(self, "mesh_subdir", _normalize_mesh_subdir(self.mesh_subdir))

    @classmethod
    def from_script(cls, script_path: str | Path) -> AssetContext:
        return cls(Path(script_path).resolve().parent)

    @property
    def asset_root(self) -> Path:
        return self.root

    @property
    def mesh_dir(self) -> Path:
        return self.root / self.mesh_subdir

    def ensure_mesh_dir(self) -> Path:
        self.mesh_dir.mkdir(parents=True, exist_ok=True)
        return self.mesh_dir

    def mesh_path(
        self,
        filename: str | Path,
        *,
        ensure_dir: bool = True,
    ) -> Path:
        return AssetSession(self.root, mesh_subdir=self.mesh_subdir).mesh_path(
            filename,
            ensure_dir=ensure_dir,
        )

    def mesh_ref(self, filename: str | Path) -> str:
        return AssetSession(self.root, mesh_subdir=self.mesh_subdir).mesh_ref(filename)


AssetContextLike = Union["AssetContext", "AssetSession", str, Path]


def current_asset_session() -> AssetSession | None:
    return _CURRENT_ASSET_SESSION.get()


def get_active_asset_session(*, create_if_missing: bool = False) -> AssetSession | None:
    session = current_asset_session()
    if session is not None:
        return session
    if not create_if_missing:
        return None
    session = AssetSession.ephemeral()
    _CURRENT_ASSET_SESSION.set(session)
    return session


@contextlib.contextmanager
def activate_asset_session(session: AssetSession) -> Iterator[AssetSession]:
    token = _CURRENT_ASSET_SESSION.set(session)
    try:
        yield session
    finally:
        _CURRENT_ASSET_SESSION.reset(token)


def asset_session_for_script(script_path: str | Path) -> AssetSession:
    return AssetSession.from_script(script_path)


def coerce_asset_context(value: AssetContextLike | None) -> Optional[AssetContext]:
    if value is None:
        return None
    if isinstance(value, AssetContext):
        return value
    if isinstance(value, AssetSession):
        return AssetContext(value.asset_root, mesh_subdir=value.mesh_subdir)
    return AssetContext(Path(value))


def resolve_asset_context(
    assets: AssetContextLike | None,
    *owners: object,
) -> Optional[AssetContext]:
    resolved = coerce_asset_context(assets)
    if resolved is not None:
        return resolved

    for owner in owners:
        owner_assets = getattr(owner, "assets", None)
        resolved = coerce_asset_context(owner_assets)
        if resolved is not None:
            return resolved

    session = current_asset_session()
    if session is not None:
        return AssetContext(session.asset_root, mesh_subdir=session.mesh_subdir)

    return None


def resolve_asset_root(
    asset_root: AssetContextLike | None,
    *owners: object,
) -> Optional[Path]:
    resolved = resolve_asset_context(asset_root, *owners)
    if resolved is not None:
        return resolved.asset_root
    return None


def resolve_mesh_path(
    filename: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None,
    ensure_dir: bool = False,
) -> Path:
    path = Path(filename)
    if path.is_absolute():
        return path.resolve()

    asset_ctx = resolve_asset_context(assets)
    if asset_ctx is not None:
        return asset_ctx.mesh_path(path, ensure_dir=ensure_dir).resolve()

    session = current_asset_session()
    if session is not None:
        return session.mesh_path(path, ensure_dir=ensure_dir).resolve()

    raise ValidationError(
        f"Mesh path is relative but no managed asset session or asset_root is available: "
        f"{os.fspath(filename)!r}"
    )


def mesh_from_input(name: str) -> Mesh:
    from .types import Mesh

    session = get_active_asset_session(create_if_missing=True)
    if session is None:
        raise ValidationError("Failed to create a managed asset session")
    info = session.register_input_mesh(name)
    return Mesh(
        filename=info.ref,
        name=info.logical_name,
        materialized_path=info.path.as_posix(),
    )


__all__ = [
    "AssetContext",
    "AssetContextLike",
    "AssetSession",
    "ManagedMeshInfo",
    "activate_asset_session",
    "asset_session_for_script",
    "coerce_asset_context",
    "current_asset_session",
    "get_active_asset_session",
    "mesh_from_input",
    "resolve_asset_context",
    "resolve_asset_root",
    "resolve_mesh_path",
]
