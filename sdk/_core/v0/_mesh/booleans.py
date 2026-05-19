from __future__ import annotations

import os
from math import sqrt
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple, Union

import manifold3d as _m3d
import numpy as np

from ..assets import get_active_asset_session
from ..types import Mesh
from .common import (
    _EPS,
    _OBJ_QUANT_STEP,
    Face,
    Vec2,
    Vec3,
    _ensure_ccw,
    _polygon_area,
    _polygon_centroid,
    _primitive_source_geometry,
    _primitive_source_transform,
    _profile_points_2d,
    _set_manifold_provenance,
    _triangulate_polygon_with_holes,
    _v_add,
    _v_cross,
    _v_dot,
    _v_norm,
    _v_normalize,
    _v_scale,
    _v_sub,
)
from .primitives import BoxGeometry, LoftGeometry, MeshGeometry


def cut_opening_on_face(
    shell_geometry: MeshGeometry,
    *,
    face: str,
    opening_profile: Iterable[Tuple[float, float]],
    depth: float,
    offset: Tuple[float, float] = (0.0, 0.0),
    taper: float = 0.0,
) -> MeshGeometry:
    """
    Add an opening "throat" on a mesh AABB face.

    This helper does not perform boolean subtraction. It creates the internal
    side walls for a recessed opening and merges them into ``shell_geometry``.
    It works best when the target face is already open (shell-like models).
    """

    if not isinstance(shell_geometry, MeshGeometry):
        raise TypeError("shell_geometry must be MeshGeometry")
    if not shell_geometry.vertices:
        raise ValueError("shell_geometry has no vertices")

    d = float(depth)
    if d <= 0:
        raise ValueError("depth must be positive")

    f = (face or "").strip().lower()
    if len(f) != 2 or f[0] not in "+-" or f[1] not in "xyz":
        raise ValueError("face must be one of: '+x', '-x', '+y', '-y', '+z', '-z'")

    points = _ensure_ccw(_profile_points_2d(opening_profile))
    ox = float(offset[0])
    oy = float(offset[1])
    taper = float(taper)
    if abs(taper) >= 0.95:
        raise ValueError("abs(taper) must be < 0.95")

    center = _polygon_centroid(points)
    scale = 1.0 - taper
    inner_points = [
        (center[0] + (x - center[0]) * scale, center[1] + (y - center[1]) * scale)
        for (x, y) in points
    ]

    xs = [v[0] for v in shell_geometry.vertices]
    ys = [v[1] for v in shell_geometry.vertices]
    zs = [v[2] for v in shell_geometry.vertices]
    bounds = {
        "x": (min(xs), max(xs)),
        "y": (min(ys), max(ys)),
        "z": (min(zs), max(zs)),
    }

    axis = f[1]
    sign = 1.0 if f[0] == "+" else -1.0
    face_plane = bounds[axis][1] if sign > 0 else bounds[axis][0]

    def map_uv(u: float, v: float, depth_local: float) -> Vec3:
        inward = -sign * depth_local
        if axis == "x":
            return (face_plane + inward, u + ox, v + oy)
        if axis == "y":
            return (u + ox, face_plane + inward, v + oy)
        return (u + ox, v + oy, face_plane + inward)

    p0 = [map_uv(u, v, 0.0) for (u, v) in points]
    p1 = [map_uv(u, v, d) for (u, v) in inner_points]
    shell_geometry.merge(LoftGeometry([p0, p1], cap=False, closed=True))
    return shell_geometry


def _manifold_from_geometry(geometry: MeshGeometry, *, name: str = "geometry"):
    if not isinstance(geometry, MeshGeometry):
        raise TypeError(f"{name} must be MeshGeometry")
    manifold_source = getattr(geometry, "_manifold_source", None)
    if manifold_source is not None:
        return manifold_source

    verts = np.asarray(geometry.vertices, dtype=np.float32)
    faces = np.asarray(geometry.faces, dtype=np.uint32)
    if verts.size == 0 or faces.size == 0:
        return _m3d.Manifold()
    if verts.ndim != 2 or verts.shape[1] != 3:
        raise ValueError(f"{name} vertices must have shape (N, 3)")
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(f"{name} faces must have shape (M, 3)")

    mesh = _m3d.Mesh(verts, faces)
    manifold = _m3d.Manifold(mesh)
    status = manifold.status()
    if status != _m3d.Error.NoError:
        raise ValueError(f"{name} is not a valid manifold solid for boolean ops (status={status})")
    return manifold


def _quantize_scalar(value: float, *, step: float) -> int:
    return int(round(float(value) / float(step)))


def _canonical_plane(
    normal: Vec3, offset: float, *, step: float
) -> Tuple[Tuple[int, int, int, int], Vec3, float]:
    n = _v_normalize(normal)
    if n[2] < -_EPS or (
        abs(n[2]) <= _EPS and (n[1] < -_EPS or (abs(n[1]) <= _EPS and n[0] < -_EPS))
    ):
        n = (-n[0], -n[1], -n[2])
        offset = -offset
    key = (
        _quantize_scalar(n[0], step=step),
        _quantize_scalar(n[1], step=step),
        _quantize_scalar(n[2], step=step),
        _quantize_scalar(offset, step=step),
    )
    return key, n, float(offset)


def _plane_basis(normal: Vec3) -> Tuple[Vec3, Vec3]:
    reference = (1.0, 0.0, 0.0) if abs(normal[0]) < 0.8 else (0.0, 1.0, 0.0)
    u_axis = _v_normalize(_v_cross(reference, normal))
    v_axis = _v_cross(normal, u_axis)
    return u_axis, v_axis


def _project_point_to_plane(point: Vec3, origin: Vec3, u_axis: Vec3, v_axis: Vec3) -> Vec2:
    delta = _v_sub(point, origin)
    return (_v_dot(delta, u_axis), _v_dot(delta, v_axis))


def _lift_point_from_plane(point: Vec2, origin: Vec3, u_axis: Vec3, v_axis: Vec3) -> Vec3:
    return _v_add(origin, _v_add(_v_scale(u_axis, point[0]), _v_scale(v_axis, point[1])))


def _point_inside_manifold(manifold, point: Vec3, *, probe_size: float) -> bool:
    probe = _m3d.Manifold.cube((probe_size, probe_size, probe_size), center=True).translate(point)
    return not (manifold ^ probe).is_empty()


def _geometry_from_manifold(manifold) -> MeshGeometry:
    if manifold is None or manifold.is_empty():
        return MeshGeometry()

    out_mesh = manifold.to_mesh()
    vp = np.asarray(out_mesh.vert_properties, dtype=np.float64)
    tri = np.asarray(out_mesh.tri_verts, dtype=np.int64)
    if vp.size == 0 or tri.size == 0:
        return MeshGeometry()

    plane_step = max(_OBJ_QUANT_STEP, float(manifold.get_tolerance()) * 4.0)
    grouped_triangles: dict[Tuple[int, int, int, int], List[Tuple[Vec3, Vec3, Vec3]]] = {}
    grouped_normals: dict[Tuple[int, int, int, int], Vec3] = {}
    grouped_offsets: dict[Tuple[int, int, int, int], float] = {}

    for face in tri:
        points = tuple((float(vp[idx][0]), float(vp[idx][1]), float(vp[idx][2])) for idx in face)
        normal = _v_cross(_v_sub(points[1], points[0]), _v_sub(points[2], points[0]))
        if _v_norm(normal) <= _EPS:
            continue
        plane_offset = _v_dot(_v_normalize(normal), points[0])
        key, plane_normal, canonical_offset = _canonical_plane(
            normal, plane_offset, step=plane_step
        )
        grouped_triangles.setdefault(key, []).append(points)
        grouped_normals.setdefault(key, plane_normal)
        grouped_offsets.setdefault(key, canonical_offset)

    if not grouped_triangles:
        return MeshGeometry()

    bounds = manifold.bounding_box()
    bbox_diag = sqrt(
        (float(bounds[3]) - float(bounds[0])) ** 2
        + (float(bounds[4]) - float(bounds[1])) ** 2
        + (float(bounds[5]) - float(bounds[2])) ** 2
    )
    sample_offset = max(
        float(manifold.get_tolerance()) * 12.0, bbox_diag * 2.0e-4, _OBJ_QUANT_STEP * 32.0
    )
    probe_size = max(sample_offset * 0.1, _OBJ_QUANT_STEP * 8.0)

    output_vertices: List[Vec3] = []
    output_faces: List[Face] = []
    vertex_index: dict[Tuple[int, int, int], int] = {}

    def add_output_vertex(point: Vec3) -> int:
        key = (
            _quantize_scalar(point[0], step=_OBJ_QUANT_STEP),
            _quantize_scalar(point[1], step=_OBJ_QUANT_STEP),
            _quantize_scalar(point[2], step=_OBJ_QUANT_STEP),
        )
        idx = vertex_index.get(key)
        if idx is not None:
            return idx
        idx = len(output_vertices)
        vertex_index[key] = idx
        output_vertices.append(point)
        return idx

    for key, triangles in grouped_triangles.items():
        plane_normal = grouped_normals[key]
        plane_origin = _v_scale(plane_normal, grouped_offsets[key])
        u_axis, v_axis = _plane_basis(plane_normal)

        projected_triangles: List[List[Vec2]] = []
        for triangle in triangles:
            projected = [
                _project_point_to_plane(point, plane_origin, u_axis, v_axis) for point in triangle
            ]
            if _polygon_area(projected) < 0.0:
                projected = [projected[0], projected[2], projected[1]]
            projected_triangles.append(projected)

        cross_section = _m3d.CrossSection(projected_triangles, _m3d.FillRule.Positive)
        cross_section = cross_section.simplify(plane_step * 0.25)

        for piece in cross_section.decompose():
            contours = [
                [(float(point[0]), float(point[1])) for point in contour]
                for contour in piece.to_polygons()
                if len(contour) >= 3
            ]
            if not contours:
                continue

            outer = max(
                (contour for contour in contours if _polygon_area(contour) > 0.0),
                key=lambda contour: abs(_polygon_area(contour)),
                default=None,
            )
            if outer is None:
                continue

            holes = [
                contour
                for contour in contours
                if contour is not outer and _polygon_area(contour) < 0.0
            ]
            flat_ring, piece_triangles = _triangulate_polygon_with_holes(outer, holes)
            if not piece_triangles:
                continue

            sample_face = piece_triangles[0]
            sample_points = [
                _lift_point_from_plane(flat_ring[idx], plane_origin, u_axis, v_axis)
                for idx in sample_face
            ]
            sample_centroid = (
                (sample_points[0][0] + sample_points[1][0] + sample_points[2][0]) / 3.0,
                (sample_points[0][1] + sample_points[1][1] + sample_points[2][1]) / 3.0,
                (sample_points[0][2] + sample_points[1][2] + sample_points[2][2]) / 3.0,
            )
            plus_sample = _v_add(sample_centroid, _v_scale(plane_normal, sample_offset))
            minus_sample = _v_add(sample_centroid, _v_scale(plane_normal, -sample_offset))
            inside_plus = _point_inside_manifold(manifold, plus_sample, probe_size=probe_size)
            inside_minus = _point_inside_manifold(manifold, minus_sample, probe_size=probe_size)
            if inside_plus == inside_minus:
                continue

            reverse = inside_plus and not inside_minus
            output_indices = [
                add_output_vertex(_lift_point_from_plane(point, plane_origin, u_axis, v_axis))
                for point in flat_ring
            ]
            for a, b, c in piece_triangles:
                if reverse:
                    output_faces.append((output_indices[a], output_indices[c], output_indices[b]))
                else:
                    output_faces.append((output_indices[a], output_indices[b], output_indices[c]))

    geometry = MeshGeometry(vertices=output_vertices, faces=output_faces)
    _set_manifold_provenance(geometry, manifold)
    return geometry


def _is_expected_boolean_fallback_error(exc: BaseException) -> bool:
    if isinstance(exc, (ValueError, RuntimeError)):
        return True
    module_name = type(exc).__module__.lower()
    return module_name.startswith("manifold")


def boolean_union(a: MeshGeometry, b: MeshGeometry) -> MeshGeometry:
    """
    Compute a solid boolean union of two meshes.

    Both inputs must be manifold solids. This is an MVP wrapper for clean geometry.
    """
    ma = _manifold_from_geometry(a, name="a")
    mb = _manifold_from_geometry(b, name="b")
    return _geometry_from_manifold(ma + mb)


def _boolean_union_many(geometries: Sequence[MeshGeometry]) -> MeshGeometry:
    combined = _m3d.Manifold()
    for idx, geometry in enumerate(geometries):
        combined = combined + _manifold_from_geometry(geometry, name=f"geometry[{idx}]")
    return _geometry_from_manifold(combined)


def boolean_difference(a: MeshGeometry, b: MeshGeometry) -> MeshGeometry:
    """
    Compute a solid boolean difference: ``a - b``.

    Both inputs must be manifold solids. This is an MVP wrapper for clean geometry.
    """
    ma = _manifold_from_geometry(a, name="a")
    mb = _manifold_from_geometry(b, name="b")
    return _geometry_from_manifold(ma - mb)


def boolean_intersection(a: MeshGeometry, b: MeshGeometry) -> MeshGeometry:
    """
    Compute a solid boolean intersection of two meshes.

    Both inputs must be manifold solids. This is an MVP wrapper for clean geometry.
    """
    ma = _manifold_from_geometry(a, name="a")
    mb = _manifold_from_geometry(b, name="b")
    return _geometry_from_manifold(ma ^ mb)


def mesh_from_geometry(
    geometry: MeshGeometry,
    name: Union[str, os.PathLike[str]],
) -> Mesh:
    name_s = os.fspath(name)
    if _looks_like_legacy_mesh_filename(name_s):
        geometry.save_obj(name_s)
        resolved_path = Path(name_s).resolve()
        return Mesh(
            filename=_export_friendly_mesh_filename(name_s),
            name=resolved_path.stem,
            source_geometry=_primitive_source_geometry(geometry),
            source_transform=_primitive_source_transform(geometry),
            materialized_path=resolved_path.as_posix(),
        )

    session = get_active_asset_session(create_if_missing=True)
    if session is None:
        raise RuntimeError("Failed to create a managed asset session")
    info = session.register_mesh_text(name_s, geometry.to_obj())
    return Mesh(
        filename=info.ref,
        name=info.logical_name,
        source_geometry=_primitive_source_geometry(geometry),
        source_transform=_primitive_source_transform(geometry),
        materialized_path=info.path.as_posix(),
    )


def _looks_like_legacy_mesh_filename(value: str) -> bool:
    if not value:
        return False
    if value.startswith(("assets/meshes/", "meshes/")):
        return True
    try:
        path = Path(value)
    except Exception:
        return False
    if path.is_absolute():
        return True
    return len(path.parts) > 1


def _export_friendly_mesh_filename(filename: str) -> str:
    """
    Convert a disk path into a portable export filename when possible.

    Prefer canonical `assets/meshes/...` paths when the file lives under that subtree.
    Fall back to legacy `meshes/...` relative paths only when the file was actually written there.
    Otherwise, return the input unchanged.
    """

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
        i = asset_meshes_indices[-1]
        return Path(*parts[i:]).as_posix()

    meshes_indices = [i for i, p in enumerate(parts) if p == "meshes"]
    if not meshes_indices:
        return filename

    i = meshes_indices[-1]
    rel = Path(*parts[i:]).as_posix()
    return rel
