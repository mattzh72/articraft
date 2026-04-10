from __future__ import annotations

import hashlib
import json
import os
import shutil
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
from typing import Any

from sdk._core.v0.assets import (
    AssetContextLike,
    AssetSession,
    get_active_asset_session,
    resolve_asset_context,
)
from sdk._core.v0.types import Mesh
from sdk._dependencies import require_cadquery

Vec3 = tuple[float, float, float]
AABB = tuple[Vec3, Vec3]
_CADQUERY_MESH_CACHE_VERSION = "v1"


@dataclass(frozen=True)
class CadQueryMeshExport:
    mesh: Mesh
    local_aabb: AABB
    center_xyz: Vec3
    size_xyz: Vec3


@dataclass(frozen=True)
class _CadQueryMeshCacheMetadata:
    cache_key: str
    local_aabb: AABB


def _require_cadquery() -> Any:
    return require_cadquery(feature="CadQuery support in `sdk`")


def _vector_xyz(value: Any) -> Vec3:
    if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
        return (float(value.x), float(value.y), float(value.z))
    if isinstance(value, (tuple, list)) and len(value) == 3:
        return (float(value[0]), float(value[1]), float(value[2]))
    raise TypeError(f"Unsupported CadQuery vertex type: {type(value).__name__}")


def _coerce_shape(model: object, cq: Any) -> object:
    if isinstance(model, cq.Assembly):
        try:
            return model.toCompound()
        except Exception as exc:
            raise TypeError("Failed to convert CadQuery Assembly into a compound shape") from exc

    if isinstance(model, cq.Workplane):
        try:
            values = list(model.vals())
        except Exception as exc:
            raise TypeError("CadQuery Workplane did not produce an exportable shape") from exc
        if not values:
            raise TypeError("CadQuery Workplane produced no exportable shape")
        if not all(isinstance(value, cq.Shape) for value in values):
            raise TypeError("CadQuery Workplane produced non-shape objects")
        if len(values) == 1:
            return values[0]
        try:
            return cq.Compound.makeCompound(values)
        except Exception as exc:
            raise TypeError(
                "Failed to combine CadQuery Workplane objects into a compound shape"
            ) from exc

    if isinstance(model, cq.Shape):
        return model

    raise TypeError(
        "Unsupported CadQuery model type. Expected cadquery.Shape, "
        "cadquery.Workplane, or cadquery.Assembly."
    )


def _coerce_shape_list(model: object, cq: Any) -> list[object]:
    if isinstance(model, cq.Assembly):
        return _assembly_component_shapes(model, cq)

    if isinstance(model, cq.Workplane):
        try:
            values = list(model.vals())
        except Exception as exc:
            raise TypeError("CadQuery Workplane did not produce an exportable shape") from exc
        if not values:
            raise TypeError("CadQuery Workplane produced no exportable shape")
        if not all(isinstance(value, cq.Shape) for value in values):
            raise TypeError("CadQuery Workplane produced non-shape objects")
        shapes: list[object] = []
        for value in values:
            shapes.extend(_split_shape_components(value))
        return shapes

    if isinstance(model, cq.Shape):
        return _split_shape_components(model)

    raise TypeError(
        "Unsupported CadQuery model type. Expected cadquery.Shape, "
        "cadquery.Workplane, or cadquery.Assembly."
    )


def _split_shape_components(shape: object) -> list[object]:
    solids_getter = getattr(shape, "Solids", None)
    if callable(solids_getter):
        try:
            solids = [solid for solid in solids_getter() if solid is not None]
        except Exception:
            solids = []
        if len(solids) > 1:
            return list(solids)
        if len(solids) == 1:
            return [solids[0]]
    return [shape]


def _apply_shape_location(shape: object, location: object) -> object:
    for method_name in ("located", "locate"):
        method = getattr(shape, method_name, None)
        if not callable(method):
            continue
        try:
            return method(location)
        except Exception:
            continue
    raise TypeError("CadQuery shape could not be located for component export")


def _assembly_component_shapes(
    assembly: object, cq: Any, *, parent_location: object | None = None
) -> list[object]:
    base_location = parent_location if parent_location is not None else cq.Location()
    assembly_location = getattr(assembly, "loc", None)
    if assembly_location is None:
        current_location = base_location
    else:
        current_location = base_location * assembly_location

    shapes: list[object] = []
    obj = getattr(assembly, "obj", None)
    if obj is not None:
        for shape in _coerce_shape_list(obj, cq):
            shapes.append(_apply_shape_location(shape, current_location))

    children = getattr(assembly, "children", ())
    for child in children:
        shapes.extend(_assembly_component_shapes(child, cq, parent_location=current_location))
    return shapes


def _tessellate_cadquery_shape(
    shape: object,
    *,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> tuple[list[Vec3], list[tuple[int, int, int]]]:
    try:
        vertices_raw, triangles_raw = shape.tessellate(
            float(tolerance),
            float(angular_tolerance),
        )
    except TypeError:
        vertices_raw, triangles_raw = shape.tessellate(float(tolerance))

    scale = float(unit_scale)
    vertices = [tuple(coord * scale for coord in _vector_xyz(vertex)) for vertex in vertices_raw]
    triangles = [(int(face[0]), int(face[1]), int(face[2])) for face in triangles_raw]

    if not vertices:
        raise ValueError("CadQuery tessellation produced no vertices")
    if not triangles:
        raise ValueError("CadQuery tessellation produced no faces")

    return vertices, triangles


def tessellate_cadquery(
    model: object,
    *,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> tuple[list[Vec3], list[tuple[int, int, int]]]:
    cq = _require_cadquery()
    shape = _coerce_shape(model, cq)
    return _tessellate_cadquery_shape(
        shape,
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
        unit_scale=unit_scale,
    )


def _export_friendly_mesh_filename(filename: str) -> str:
    if not isinstance(filename, str) or not filename:
        return filename

    normalized = filename.replace("\\", "/")
    if normalized.startswith("assets/meshes/"):
        return normalized
    if normalized.startswith("meshes/"):
        return normalized

    try:
        path = Path(filename)
    except Exception:
        return filename

    parts = list(path.parts)
    asset_meshes_indices = [
        i for i in range(len(parts) - 1) if parts[i] == "assets" and parts[i + 1] == "meshes"
    ]
    if asset_meshes_indices:
        return Path(*parts[asset_meshes_indices[-1] :]).as_posix()

    meshes_indices = [i for i, part in enumerate(parts) if part == "meshes"]
    if not meshes_indices:
        return filename

    return Path(*parts[meshes_indices[-1] :]).as_posix()


def _resolve_export_path(
    filename: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None,
) -> Path:
    path = Path(filename)
    asset_ctx = resolve_asset_context(assets)
    if asset_ctx is not None and not path.is_absolute():
        return asset_ctx.mesh_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _looks_like_legacy_export_name(value: str | os.PathLike[str]) -> bool:
    name = os.fspath(value)
    if not name:
        return False
    if name.startswith(("assets/meshes/", "meshes/")):
        return True
    try:
        path = Path(name)
    except Exception:
        return False
    if path.is_absolute():
        return True
    return len(path.parts) > 1


def _managed_export_session(assets: AssetContextLike | None) -> AssetSession:
    if isinstance(assets, AssetSession):
        return assets
    asset_ctx = resolve_asset_context(assets)
    current = get_active_asset_session(create_if_missing=False)
    if asset_ctx is not None:
        if (
            current is not None
            and current.asset_root == asset_ctx.asset_root
            and current.mesh_subdir == asset_ctx.mesh_subdir
        ):
            return current
        return AssetSession(asset_ctx.asset_root, mesh_subdir=asset_ctx.mesh_subdir)
    if current is not None:
        return current
    session = get_active_asset_session(create_if_missing=True)
    if session is None:
        raise RuntimeError("Failed to create a managed asset session")
    return session


def _component_export_paths(
    filename: str | os.PathLike[str],
    *,
    component_count: int,
    assets: AssetContextLike | None,
) -> list[Path]:
    path = _resolve_export_path(filename, assets=assets)
    if component_count <= 1:
        return [path]

    suffix = path.suffix or ".obj"
    stem = path.stem if path.suffix else path.name
    return [
        path.parent / f"{stem}__component_{index:03d}{suffix}"
        for index in range(1, component_count + 1)
    ]


def _component_export_names(
    name: str | os.PathLike[str],
    *,
    component_count: int,
) -> list[str]:
    raw = os.fspath(name)
    if component_count <= 1:
        return [raw]
    stem = Path(raw).stem if Path(raw).suffix else raw
    return [f"{stem}__component_{index:03d}" for index in range(1, component_count + 1)]


def _cache_root_for_export(
    mesh_path: Path,
    *,
    assets: AssetContextLike | None,
) -> Path:
    asset_ctx = resolve_asset_context(assets)
    if asset_ctx is not None:
        return asset_ctx.asset_root / "assets" / ".cadquery_cache"
    return mesh_path.parent / ".cadquery_cache"


def _cache_mesh_path(cache_root: Path, cache_key: str) -> Path:
    return cache_root / f"{cache_key}.obj"


def _cache_metadata_path(cache_root: Path, cache_key: str) -> Path:
    return cache_root / f"{cache_key}.json"


def _export_metadata_path(mesh_path: Path) -> Path:
    return Path(f"{mesh_path.as_posix()}.cadquery.json")


def _coerce_vec3(value: Any) -> Vec3 | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return None


def _parse_cache_metadata(payload: Any) -> _CadQueryMeshCacheMetadata | None:
    if not isinstance(payload, dict):
        return None
    cache_key = payload.get("cache_key")
    local_aabb = payload.get("local_aabb")
    if not isinstance(cache_key, str) or not cache_key.strip():
        return None
    if not isinstance(local_aabb, (list, tuple)) or len(local_aabb) != 2:
        return None
    local_min = _coerce_vec3(local_aabb[0])
    local_max = _coerce_vec3(local_aabb[1])
    if local_min is None or local_max is None:
        return None
    return _CadQueryMeshCacheMetadata(
        cache_key=cache_key,
        local_aabb=(local_min, local_max),
    )


def _read_cache_metadata(path: Path) -> _CadQueryMeshCacheMetadata | None:
    if not path.exists() or not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return _parse_cache_metadata(payload)


def _write_cache_metadata(path: Path, metadata: _CadQueryMeshCacheMetadata) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "cache_key": metadata.cache_key,
        "local_aabb": [
            list(metadata.local_aabb[0]),
            list(metadata.local_aabb[1]),
        ],
        "version": _CADQUERY_MESH_CACHE_VERSION,
    }
    path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")


def _serialize_shape_for_cache(shape: object) -> bytes:
    for exporter_name in ("exportBin", "exportBrep"):
        exporter = getattr(shape, exporter_name, None)
        if not callable(exporter):
            continue
        buffer = BytesIO()
        try:
            exporter(buffer)
        except Exception:
            continue
        payload = buffer.getvalue()
        if payload:
            return payload
    raise ValueError("CadQuery shape could not be serialized for cache key generation")


def _cadquery_mesh_cache_key(
    shape: object,
    *,
    tolerance: float,
    angular_tolerance: float,
    unit_scale: float,
    cq: Any,
) -> str:
    digest = hashlib.sha256()
    digest.update(_CADQUERY_MESH_CACHE_VERSION.encode("utf-8"))
    digest.update(
        json.dumps(
            {
                "angular_tolerance": float(angular_tolerance),
                "cadquery_version": str(getattr(cq, "__version__", "")),
                "tolerance": float(tolerance),
                "unit_scale": float(unit_scale),
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    )
    digest.update(b"\0")
    digest.update(_serialize_shape_for_cache(shape))
    return digest.hexdigest()


def _copy_cached_mesh(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)


def _write_obj(
    path: Path,
    *,
    vertices: list[Vec3],
    triangles: list[tuple[int, int, int]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines: list[str] = []
    for x, y, z in vertices:
        lines.append(f"v {x:.6f} {y:.6f} {z:.6f}")
    for a, b, c in triangles:
        lines.append(f"f {a + 1} {b + 1} {c + 1}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _local_aabb_from_vertices(vertices: list[Vec3]) -> AABB:
    mn = (
        min(v[0] for v in vertices),
        min(v[1] for v in vertices),
        min(v[2] for v in vertices),
    )
    mx = (
        max(v[0] for v in vertices),
        max(v[1] for v in vertices),
        max(v[2] for v in vertices),
    )
    return mn, mx


def _aabb_center(aabb: AABB) -> Vec3:
    (mn, mx) = aabb
    return (
        (float(mn[0]) + float(mx[0])) * 0.5,
        (float(mn[1]) + float(mx[1])) * 0.5,
        (float(mn[2]) + float(mx[2])) * 0.5,
    )


def _aabb_size(aabb: AABB) -> Vec3:
    (mn, mx) = aabb
    return (
        float(mx[0]) - float(mn[0]),
        float(mx[1]) - float(mn[1]),
        float(mx[2]) - float(mn[2]),
    )


def cadquery_local_aabb(
    model: object,
    *,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> AABB:
    vertices, _triangles = tessellate_cadquery(
        model,
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
        unit_scale=unit_scale,
    )
    return _local_aabb_from_vertices(vertices)


def export_cadquery_mesh(
    model: object,
    name: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None = None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> CadQueryMeshExport:
    logical_name = os.fspath(name)
    legacy_mode = _looks_like_legacy_export_name(logical_name)
    asset_ctx = resolve_asset_context(assets)
    if legacy_mode:
        path = _resolve_export_path(logical_name, assets=asset_ctx)
        mesh_ref = _export_friendly_mesh_filename(path.as_posix())
        mesh_name = Path(logical_name).stem if Path(logical_name).stem else None
        asset_owner: AssetContextLike | None = asset_ctx
        cache_probe_path = path
    else:
        session = _managed_export_session(assets)
        mesh_name = logical_name
        asset_owner = session
        cache_probe_path = session.mesh_path("__cadquery_cache_probe__.obj", ensure_dir=False)
    cq = _require_cadquery()
    shape = _coerce_shape(model, cq)
    cache_key = _cadquery_mesh_cache_key(
        shape,
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
        unit_scale=unit_scale,
        cq=cq,
    )
    cache_root = _cache_root_for_export(cache_probe_path, assets=asset_owner)
    cache_mesh_path = _cache_mesh_path(cache_root, cache_key)
    cache_metadata_path = _cache_metadata_path(cache_root, cache_key)
    cached_metadata = _read_cache_metadata(cache_metadata_path)
    if (
        cache_mesh_path.exists()
        and cached_metadata is not None
        and cached_metadata.cache_key == cache_key
    ):
        metadata = cached_metadata
    else:
        vertices, triangles = _tessellate_cadquery_shape(
            shape,
            tolerance=tolerance,
            angular_tolerance=angular_tolerance,
            unit_scale=unit_scale,
        )
        _write_obj(cache_mesh_path, vertices=vertices, triangles=triangles)
        local_aabb = _local_aabb_from_vertices(vertices)
        metadata = _CadQueryMeshCacheMetadata(cache_key=cache_key, local_aabb=local_aabb)
        _write_cache_metadata(cache_metadata_path, metadata)

    if legacy_mode:
        destination_metadata_path = _export_metadata_path(path)
        destination_metadata = _read_cache_metadata(destination_metadata_path)
        if (
            path.exists()
            and destination_metadata is not None
            and destination_metadata.cache_key == cache_key
        ):
            local_aabb = destination_metadata.local_aabb
            return CadQueryMeshExport(
                mesh=Mesh(
                    filename=mesh_ref,
                    name=mesh_name,
                    materialized_path=path.as_posix(),
                ),
                local_aabb=local_aabb,
                center_xyz=_aabb_center(local_aabb),
                size_xyz=_aabb_size(local_aabb),
            )
        _copy_cached_mesh(cache_mesh_path, path)
        _write_cache_metadata(destination_metadata_path, metadata)
        return CadQueryMeshExport(
            mesh=Mesh(
                filename=mesh_ref,
                name=mesh_name,
                materialized_path=path.as_posix(),
            ),
            local_aabb=metadata.local_aabb,
            center_xyz=_aabb_center(metadata.local_aabb),
            size_xyz=_aabb_size(metadata.local_aabb),
        )

    info = session.register_mesh_file(logical_name, cache_mesh_path)
    destination_metadata_path = _export_metadata_path(info.path)
    destination_metadata = _read_cache_metadata(destination_metadata_path)
    if destination_metadata is None or destination_metadata.cache_key != cache_key:
        _write_cache_metadata(destination_metadata_path, metadata)
    return CadQueryMeshExport(
        mesh=Mesh(
            filename=info.ref,
            name=mesh_name,
            materialized_path=info.path.as_posix(),
        ),
        local_aabb=metadata.local_aabb,
        center_xyz=_aabb_center(metadata.local_aabb),
        size_xyz=_aabb_size(metadata.local_aabb),
    )


def save_cadquery_obj(
    model: object,
    name: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None = None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> Path:
    export = export_cadquery_mesh(
        model,
        name,
        assets=assets,
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
        unit_scale=unit_scale,
    )
    materialized_path = export.mesh.materialized_path
    if materialized_path is None:
        raise RuntimeError("CadQuery export did not record a materialized mesh path")
    return Path(materialized_path)


def export_cadquery_components(
    model: object,
    name: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None = None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> list[CadQueryMeshExport]:
    cq = _require_cadquery()
    shapes = _coerce_shape_list(model, cq)
    if not shapes:
        raise TypeError("CadQuery model produced no exportable component shapes")
    export_targets: list[str | Path]
    if _looks_like_legacy_export_name(name):
        export_targets = _component_export_paths(
            name,
            component_count=len(shapes),
            assets=assets,
        )
    else:
        export_targets = _component_export_names(name, component_count=len(shapes))
    return [
        export_cadquery_mesh(
            shape,
            export_target,
            assets=assets,
            tolerance=tolerance,
            angular_tolerance=angular_tolerance,
            unit_scale=unit_scale,
        )
        for shape, export_target in zip(shapes, export_targets, strict=True)
    ]


def mesh_from_cadquery(
    model: object,
    name: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None = None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> Mesh:
    export = export_cadquery_mesh(
        model,
        name,
        assets=assets,
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
        unit_scale=unit_scale,
    )
    return export.mesh


def mesh_components_from_cadquery(
    model: object,
    name: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None = None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> list[Mesh]:
    return [
        export.mesh
        for export in export_cadquery_components(
            model,
            name,
            assets=assets,
            tolerance=tolerance,
            angular_tolerance=angular_tolerance,
            unit_scale=unit_scale,
        )
    ]
