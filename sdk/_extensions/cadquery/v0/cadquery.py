from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from sdk._core.v0.assets import AssetContextLike, resolve_asset_context
from sdk._core.v0.errors import ValidationError
from sdk._core.v0.types import Mesh

Vec3 = tuple[float, float, float]
AABB = tuple[Vec3, Vec3]


@dataclass(frozen=True)
class CadQueryMeshExport:
    mesh: Mesh
    mesh_path: Path
    local_aabb: AABB
    center_xyz: Vec3
    size_xyz: Vec3


def _require_cadquery() -> Any:
    try:
        import cadquery as cq
    except Exception as exc:
        raise RuntimeError(
            "CadQuery support requires the optional `cadquery` dependency. "
            "Install project dependencies again after updating `pyproject.toml`."
        ) from exc
    return cq


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
            shape = model.val()
        except Exception as exc:
            raise TypeError("CadQuery Workplane did not produce an exportable shape") from exc
        if shape is None:
            raise TypeError("CadQuery Workplane produced no exportable shape")
        return shape

    if isinstance(model, cq.Shape):
        return model

    raise TypeError(
        "Unsupported CadQuery model type. Expected cadquery.Shape, "
        "cadquery.Workplane, or cadquery.Assembly."
    )


def tessellate_cadquery(
    model: object,
    *,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> tuple[list[Vec3], list[tuple[int, int, int]]]:
    cq = _require_cadquery()
    shape = _coerce_shape(model, cq)

    try:
        vertices_raw, triangles_raw = shape.tessellate(
            float(tolerance),
            float(angular_tolerance),
        )
    except TypeError:
        vertices_raw, triangles_raw = shape.tessellate(float(tolerance))

    scale = float(unit_scale)
    vertices = [
        tuple(coord * scale for coord in _vector_xyz(vertex))
        for vertex in vertices_raw
    ]
    triangles = [
        (int(face[0]), int(face[1]), int(face[2]))
        for face in triangles_raw
    ]

    if not vertices:
        raise ValueError("CadQuery tessellation produced no vertices")
    if not triangles:
        raise ValueError("CadQuery tessellation produced no faces")

    return vertices, triangles


def _export_friendly_mesh_filename(filename: str) -> str:
    if not isinstance(filename, str) or not filename:
        return filename

    normalized = filename.replace("\\", "/")
    if normalized.startswith("meshes/"):
        return normalized

    try:
        path = Path(filename)
    except Exception:
        return filename

    parts = list(path.parts)
    meshes_indices = [i for i, part in enumerate(parts) if part == "meshes"]
    if not meshes_indices:
        return filename

    return Path(*parts[meshes_indices[-1]:]).as_posix()


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
    filename: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None = None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> CadQueryMeshExport:
    path = _resolve_export_path(filename, assets=assets)
    vertices, triangles = tessellate_cadquery(
        model,
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
        unit_scale=unit_scale,
    )
    _write_obj(path, vertices=vertices, triangles=triangles)
    local_aabb = _local_aabb_from_vertices(vertices)
    return CadQueryMeshExport(
        mesh=Mesh(filename=_export_friendly_mesh_filename(path.as_posix())),
        mesh_path=path,
        local_aabb=local_aabb,
        center_xyz=_aabb_center(local_aabb),
        size_xyz=_aabb_size(local_aabb),
    )


def save_cadquery_obj(
    model: object,
    filename: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None = None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> Path:
    export = export_cadquery_mesh(
        model,
        filename,
        assets=assets,
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
        unit_scale=unit_scale,
    )
    return export.mesh_path


def mesh_from_cadquery(
    model: object,
    filename: str | os.PathLike[str],
    *,
    assets: AssetContextLike | None = None,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
    unit_scale: float = 1.0,
) -> Mesh:
    export = export_cadquery_mesh(
        model,
        filename,
        assets=assets,
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
        unit_scale=unit_scale,
    )
    return export.mesh
