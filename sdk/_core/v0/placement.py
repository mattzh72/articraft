from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Literal, Optional, Tuple, Union

from ._mesh_provenance import decompose_mesh_primitive_provenance
from .assets import AssetContext, resolve_asset_root, resolve_mesh_path
from .errors import ValidationError
from .geometry_qc import (
    AABB,
    _axis_angle_matrix,
    _geometry_local_aabb,
    _mat4_mul,
    _mat4_vec3,
    _origin_to_mat4,
    _transform_aabb,
    link_local_aabbs,
)
from .mesh import (
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    _ensure_ccw,
    _profile_points_2d,
    _triangulate_polygon,
)
from .types import Box, Cylinder, Geometry, Mat4, Mesh, Origin, Part, Sphere, Visual

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]
SurfaceSubject = Union[Part, Visual, Geometry]
MeshSubject = Union[MeshGeometry, Mesh]
SurfaceWrapMapping = Literal["auto", "intrinsic", "nearest"]
_EPS = 1e-9
_TRIMESH_CACHE: dict[Path, tuple[tuple[int, int], object]] = {}
logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SurfaceFrame:
    point: Vec3
    normal: Vec3
    tangent_u: Vec3
    tangent_v: Vec3


@dataclass(frozen=True)
class _SurfaceHit:
    point: Vec3
    normal: Vec3
    distance: float


def _identity4() -> Mat4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _as_pair(values: object, *, name: str) -> tuple[float, float]:
    try:
        if len(values) != 2:  # type: ignore[arg-type]
            raise ValidationError(f"{name} must have 2 elements")
        return (float(values[0]), float(values[1]))  # type: ignore[index]
    except TypeError as exc:
        raise ValidationError(f"{name} must have 2 elements") from exc


def _as_vec3(values: object, *, name: str) -> Vec3:
    try:
        if len(values) != 3:  # type: ignore[arg-type]
            raise ValidationError(f"{name} must have 3 elements")
        return (
            float(values[0]),  # type: ignore[index]
            float(values[1]),  # type: ignore[index]
            float(values[2]),  # type: ignore[index]
        )
    except TypeError as exc:
        raise ValidationError(f"{name} must have 3 elements") from exc


def _aabb_union(a: list[AABB]) -> AABB:
    if not a:
        raise ValidationError("Cannot union empty AABB list")
    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]
    for mn, mx in a:
        for i in range(3):
            mins[i] = min(mins[i], mn[i])
            maxs[i] = max(maxs[i], mx[i])
    return (mins[0], mins[1], mins[2]), (maxs[0], maxs[1], maxs[2])


def _aabb_center(aabb: AABB) -> Vec3:
    (mn, mx) = aabb
    return ((mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5)


def _aabb_size(aabb: AABB) -> Vec3:
    (mn, mx) = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _dot(a: Vec3, b: Vec3) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: Vec3) -> float:
    return math.sqrt(_dot(v, v))


def _normalize(v: Vec3, *, name: str) -> Vec3:
    n = _norm(v)
    if n <= _EPS:
        raise ValidationError(f"{name} must be non-zero")
    return (v[0] / n, v[1] / n, v[2] / n)


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(max(float(value), float(lo)), float(hi))


def _mat3_from_mat4(mat: Mat4) -> Mat3:
    return (
        (float(mat[0][0]), float(mat[0][1]), float(mat[0][2])),
        (float(mat[1][0]), float(mat[1][1]), float(mat[1][2])),
        (float(mat[2][0]), float(mat[2][1]), float(mat[2][2])),
    )


def _mat3_transpose(mat: Mat3) -> Mat3:
    return (
        (mat[0][0], mat[1][0], mat[2][0]),
        (mat[0][1], mat[1][1], mat[2][1]),
        (mat[0][2], mat[1][2], mat[2][2]),
    )


def _mat3_vec3(mat: Mat3, vec: Vec3) -> Vec3:
    return (
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    )


def _mat3_mul(a: Mat3, b: Mat3) -> Mat3:
    rows: list[Vec3] = []
    for i in range(3):
        row = []
        for j in range(3):
            row.append(a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j])
        rows.append((row[0], row[1], row[2]))
    return (rows[0], rows[1], rows[2])


def _mat3_from_columns(a: Vec3, b: Vec3, c: Vec3) -> Mat3:
    return (
        (a[0], b[0], c[0]),
        (a[1], b[1], c[1]),
        (a[2], b[2], c[2]),
    )


def _mat3_to_rpy(mat: Mat3) -> Vec3:
    pitch = math.asin(_clamp(-mat[2][0], -1.0, 1.0))
    cp = math.cos(pitch)
    if abs(cp) > 1e-8:
        roll = math.atan2(mat[2][1], mat[2][2])
        yaw = math.atan2(mat[1][0], mat[0][0])
        return (roll, pitch, yaw)

    # Gimbal-lock fallback for ZYX convention.
    roll = 0.0
    if pitch >= 0.0:
        yaw = math.atan2(-mat[0][1], mat[1][1])
    else:
        yaw = math.atan2(mat[0][1], mat[1][1])
    return (roll, pitch, yaw)


def _mat4_inverse_rigid(tf: Mat4) -> Mat4:
    rot = _mat3_from_mat4(tf)
    rot_t = _mat3_transpose(rot)
    trans = (float(tf[0][3]), float(tf[1][3]), float(tf[2][3]))
    inv_trans = _mat3_vec3(rot_t, (-trans[0], -trans[1], -trans[2]))
    return (
        (rot_t[0][0], rot_t[0][1], rot_t[0][2], inv_trans[0]),
        (rot_t[1][0], rot_t[1][1], rot_t[1][2], inv_trans[1]),
        (rot_t[2][0], rot_t[2][1], rot_t[2][2], inv_trans[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def _mat4_transform_direction(tf: Mat4, vec: Vec3) -> Vec3:
    rot = _mat3_from_mat4(tf)
    return _mat3_vec3(rot, vec)


def _resolve_asset_root_for_subject(
    asset_root: Optional[Union[Path, str, AssetContext]],
    subject: SurfaceSubject,
) -> Optional[Path]:
    if isinstance(subject, Part):
        return resolve_asset_root(asset_root, subject)
    if asset_root is None:
        return None
    if isinstance(asset_root, AssetContext):
        return asset_root.asset_root
    return Path(asset_root)


def _iter_subject_items(
    subject: SurfaceSubject,
    *,
    prefer_collisions: bool,
) -> list[tuple[Geometry, Mat4]]:
    if isinstance(subject, Part):
        if prefer_collisions and subject.collisions:
            items = list(subject.collisions)
        elif subject.visuals:
            items = list(subject.visuals)
        else:
            items = list(subject.collisions)
        return [
            (item.geometry, _origin_to_mat4(getattr(item, "origin", Origin())))
            for item in items
            if getattr(item, "geometry", None) is not None
        ]

    if isinstance(subject, Visual):
        return [(subject.geometry, _origin_to_mat4(subject.origin))]

    return [(subject, _identity4())]


def _expand_mesh_provenance(geometry: Geometry, tf: Mat4) -> tuple[Geometry, Mat4]:
    if isinstance(geometry, Mesh):
        decomposed = decompose_mesh_primitive_provenance(geometry)
        if decomposed is not None:
            primitive, local_tf = decomposed
            return primitive, _mat4_mul(tf, local_tf)
    return geometry, tf


def _subject_world_aabb(
    subject: SurfaceSubject,
    *,
    asset_root: Optional[Path],
    prefer_collisions: bool,
) -> AABB:
    aabbs: list[AABB] = []
    obj_cache: dict[Path, AABB] = {}
    for geometry, tf in _iter_subject_items(subject, prefer_collisions=prefer_collisions):
        geometry, tf = _expand_mesh_provenance(geometry, tf)
        local_aabb = _geometry_local_aabb(geometry, asset_root=asset_root, _obj_cache=obj_cache)
        aabbs.append(_transform_aabb(local_aabb, tf))
    if not aabbs:
        raise ValidationError("subject has no geometry to compute AABB")
    return _aabb_union(aabbs)


def link_local_aabb(
    link: Part,
    *,
    asset_root: Optional[Union[Path, str, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> Optional[AABB]:
    root = resolve_asset_root(asset_root, link)
    aabbs = link_local_aabbs(
        link,
        asset_root=root,
        prefer_collisions=prefer_collisions,
    )
    if not aabbs:
        return None
    return _aabb_union(aabbs)


def part_local_aabb(
    part: Part,
    *,
    asset_root: Optional[Union[Path, str, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> Optional[AABB]:
    return link_local_aabb(
        part,
        asset_root=asset_root,
        prefer_collisions=prefer_collisions,
    )


def align_centers_xy(child_aabb: AABB, parent_aabb: AABB) -> Origin:
    cx, cy, _ = _aabb_center(child_aabb)
    px, py, _ = _aabb_center(parent_aabb)
    return Origin(xyz=(px - cx, py - cy, 0.0))


def place_on_top(
    child_aabb: AABB,
    parent_aabb: AABB,
    *,
    clearance: float = 0.0,
    align_xy: bool = True,
) -> Origin:
    (cmin, _cmax) = child_aabb
    (_pmin, pmax) = parent_aabb
    dz = float(pmax[2]) + float(clearance) - float(cmin[2])
    dx = 0.0
    dy = 0.0
    if align_xy:
        cx, cy, _ = _aabb_center(child_aabb)
        px, py, _ = _aabb_center(parent_aabb)
        dx = px - cx
        dy = py - cy
    return Origin(xyz=(dx, dy, dz))


def place_in_front_of(
    child_aabb: AABB,
    parent_aabb: AABB,
    *,
    gap: float = 0.0,
    align_yz: bool = True,
) -> Origin:
    (cmin, _cmax) = child_aabb
    (_pmin, pmax) = parent_aabb
    dx = float(pmax[0]) + float(gap) - float(cmin[0])
    dy = 0.0
    dz = 0.0
    if align_yz:
        cx, cy, cz = _aabb_center(child_aabb)
        px, py, pz = _aabb_center(parent_aabb)
        dy = py - cy
        dz = pz - cz
    return Origin(xyz=(dx, dy, dz))


# Mapping from face string to (axis_index, sign, rpy).
# The rpy rotates the child's local +Z axis to point along the face outward normal.
_FACE_TABLE: dict[str, tuple[int, float, Vec3]] = {
    "+x": (0, +1.0, (0.0, math.pi / 2.0, 0.0)),
    "-x": (0, -1.0, (0.0, -math.pi / 2.0, 0.0)),
    "+y": (1, +1.0, (-math.pi / 2.0, 0.0, 0.0)),
    "-y": (1, -1.0, (math.pi / 2.0, 0.0, 0.0)),
    "+z": (2, +1.0, (0.0, 0.0, 0.0)),
    "-z": (2, -1.0, (math.pi, 0.0, 0.0)),
}

# For each face, which two parent-frame axes span the face.
# Order: (first_tangent_index, second_tangent_index).
_FACE_TANGENT_AXES: dict[str, tuple[int, int]] = {
    "+x": (1, 2),
    "-x": (1, 2),
    "+y": (0, 2),
    "-y": (0, 2),
    "+z": (0, 1),
    "-z": (0, 1),
}


def place_on_face(
    parent_link: Part,
    face: str,
    *,
    face_pos: tuple[float, float] = (0.0, 0.0),
    proud: float = 0.0,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> Origin:
    """
    Compute an ``Origin`` that places a child part on a face of the parent.

    The returned origin includes both the translation (``xyz``) and rotation
    (``rpy``) so that the child's local +Z axis points outward from the
    chosen face. This eliminates the need to manually compute rpy rotations
    and per-axis offsets when mounting controls, buttons, or panels on the
    side of a body.

    Parameters
    ----------
    parent_link:
        The parent ``Part`` whose geometry defines the mounting surface.
    face:
        Which face of the parent's AABB to use.
        One of ``"+x"``, ``"-x"``, ``"+y"``, ``"-y"``, ``"+z"``, ``"-z"``.
    face_pos:
        Position on the face surface, given as ``(a, b)`` in the parent's
        coordinate frame along the two tangent axes of the face:

        * ``±x`` faces: ``face_pos = (y, z)``
        * ``±y`` faces: ``face_pos = (x, z)``
        * ``±z`` faces: ``face_pos = (x, y)``
    proud:
        Distance the child origin is pushed outward from the face surface.
        With ``proud=0`` the child origin sits exactly on the surface.
        Set this to half the child's thickness to make the child geometry
        sit flush, or slightly more to make it visually proud.
    asset_root:
        Root directory for resolving relative mesh filenames.
    prefer_collisions:
        Whether to use collision geometry (preferred) or visual geometry
        when computing the parent's AABB.

    Returns
    -------
    Origin
        An origin with the correct ``xyz`` and ``rpy`` for mounting
        on the specified face.
    """
    face_key = face.strip().lower()
    if face_key not in _FACE_TABLE:
        raise ValidationError(
            f"Invalid face {face!r}; expected one of: {', '.join(sorted(_FACE_TABLE))}"
        )

    axis_idx, sign, rpy = _FACE_TABLE[face_key]
    tang_a, tang_b = _FACE_TANGENT_AXES[face_key]
    face_pos_pair = _as_pair(face_pos, name="face_pos")

    root = resolve_asset_root(asset_root, parent_link)
    aabbs = link_local_aabbs(
        parent_link,
        asset_root=root,
        prefer_collisions=prefer_collisions,
    )
    if not aabbs:
        raise ValidationError(f"Parent link {parent_link.name!r} has no geometry to compute AABB")
    aabb = _aabb_union(aabbs)
    (mn, mx) = aabb

    surface = float(mx[axis_idx]) if sign > 0 else float(mn[axis_idx])
    normal_coord = surface + sign * float(proud)

    xyz = [0.0, 0.0, 0.0]
    xyz[axis_idx] = normal_coord
    xyz[tang_a] = face_pos_pair[0]
    xyz[tang_b] = face_pos_pair[1]

    return Origin(xyz=(xyz[0], xyz[1], xyz[2]), rpy=rpy)


def place_on_face_uv(
    parent_link: Part,
    face: str,
    *,
    uv: tuple[float, float] = (0.5, 0.5),
    uv_margin: float | tuple[float, float] = 0.0,
    proud: float = 0.0,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> Origin:
    """
    Like :func:`place_on_face`, but address the face using normalized coordinates.

    ``uv`` is interpreted on the chosen face's tangential axes and mapped into
    the parent's AABB bounds:

    - u=0 is the min bound along the first tangent axis; u=1 is the max bound.
    - v=0 is the min bound along the second tangent axis; v=1 is the max bound.

    This is significantly less error-prone than hand-picking absolute
    ``face_pos`` values when the parent dimensions are changing.

    Parameters
    ----------
    uv:
        Normalized face coordinates. Values outside [0,1] are clamped.
    uv_margin:
        Fractional margin applied to each axis before mapping, keeping placements
        away from sharp edges. Either a single float for both axes or ``(mu, mv)``.
    """
    face_key = face.strip().lower()
    if face_key not in _FACE_TABLE:
        raise ValidationError(
            f"Invalid face {face!r}; expected one of: {', '.join(sorted(_FACE_TABLE))}"
        )

    axis_idx, sign, rpy = _FACE_TABLE[face_key]
    tang_a, tang_b = _FACE_TANGENT_AXES[face_key]

    if isinstance(uv_margin, tuple):
        mu, mv = _as_pair(uv_margin, name="uv_margin")
    else:
        mu = mv = float(uv_margin)
    mu = max(0.0, min(0.49, mu))
    mv = max(0.0, min(0.49, mv))

    u_raw, v_raw = _as_pair(uv, name="uv")
    u = max(0.0, min(1.0, u_raw))
    v = max(0.0, min(1.0, v_raw))
    u = mu + (1.0 - 2.0 * mu) * u
    v = mv + (1.0 - 2.0 * mv) * v

    root = resolve_asset_root(asset_root, parent_link)
    aabbs = link_local_aabbs(
        parent_link,
        asset_root=root,
        prefer_collisions=prefer_collisions,
    )
    if not aabbs:
        raise ValidationError(f"Parent link {parent_link.name!r} has no geometry to compute AABB")
    (mn, mx) = _aabb_union(aabbs)

    surface = float(mx[axis_idx]) if sign > 0 else float(mn[axis_idx])
    normal_coord = surface + sign * float(proud)

    ta0, ta1 = float(mn[tang_a]), float(mx[tang_a])
    tb0, tb1 = float(mn[tang_b]), float(mx[tang_b])
    face_pos = (ta0 + (ta1 - ta0) * u, tb0 + (tb1 - tb0) * v)

    xyz = [0.0, 0.0, 0.0]
    xyz[axis_idx] = normal_coord
    xyz[tang_a] = float(face_pos[0])
    xyz[tang_b] = float(face_pos[1])

    return Origin(xyz=(xyz[0], xyz[1], xyz[2]), rpy=rpy)


def _axis_vector(axis: str) -> Vec3:
    axis_key = axis.strip().lower()
    table = {
        "+x": (1.0, 0.0, 0.0),
        "-x": (-1.0, 0.0, 0.0),
        "+y": (0.0, 1.0, 0.0),
        "-y": (0.0, -1.0, 0.0),
        "+z": (0.0, 0.0, 1.0),
        "-z": (0.0, 0.0, -1.0),
    }
    if axis_key not in table:
        raise ValidationError(f"Invalid axis {axis!r}; expected one of: {', '.join(sorted(table))}")
    return table[axis_key]


def _child_local_basis_for_axis(axis: str) -> Mat3:
    axis_key = axis.strip().lower()
    basis = {
        "+z": ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        "-z": ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)),
        "+x": ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
        "-x": ((0.0, 1.0, 0.0), (0.0, 0.0, -1.0), (-1.0, 0.0, 0.0)),
        "+y": ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        "-y": ((0.0, 0.0, -1.0), (1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    }
    if axis_key not in basis:
        raise ValidationError(f"Invalid axis {axis!r}; expected one of: {', '.join(sorted(basis))}")
    return basis[axis_key]


def _build_surface_tangents(normal: Vec3, up_hint: Vec3) -> tuple[Vec3, Vec3]:
    up = _normalize(up_hint, name="up_hint")
    projected = (
        up[0] - normal[0] * _dot(up, normal),
        up[1] - normal[1] * _dot(up, normal),
        up[2] - normal[2] * _dot(up, normal),
    )
    if _norm(projected) <= _EPS:
        fallback = (1.0, 0.0, 0.0)
        if abs(_dot(fallback, normal)) >= 0.95:
            fallback = (0.0, 1.0, 0.0)
        projected = _cross(fallback, normal)
    tangent_u = _normalize(projected, name="surface tangent")
    tangent_v = _normalize(_cross(normal, tangent_u), name="surface bitangent")
    return tangent_u, tangent_v


def _rotation_for_surface_frame(
    frame: SurfaceFrame,
    *,
    child_axis: str,
    spin: float,
) -> Mat3:
    local_basis = _child_local_basis_for_axis(child_axis)
    world_basis = _mat3_from_columns(frame.tangent_u, frame.tangent_v, frame.normal)
    local_basis_mat = _mat3_from_columns(
        local_basis[0],
        local_basis[1],
        local_basis[2],
    )
    base = _mat3_mul(world_basis, _mat3_transpose(local_basis_mat))
    if abs(float(spin)) <= _EPS:
        return base
    spin_world = _axis_angle_matrix(frame.normal, float(spin))
    return _mat3_mul(spin_world, base)


def _resolve_mesh_path(mesh: Mesh, asset_root: Optional[Path]) -> Path:
    return resolve_mesh_path(mesh.filename, assets=asset_root)


def _mesh_cache_fingerprint(mesh_path: Path) -> tuple[int, int]:
    try:
        stat = mesh_path.stat()
    except FileNotFoundError as exc:
        raise ValidationError(f"Mesh file not found: {mesh_path}") from exc
    return int(stat.st_mtime_ns), int(stat.st_size)


def _load_trimesh_mesh(mesh: Mesh, *, asset_root: Optional[Path]):
    mesh_path = _resolve_mesh_path(mesh, asset_root)
    fingerprint = _mesh_cache_fingerprint(mesh_path)
    cached_entry = _TRIMESH_CACHE.get(mesh_path)
    cached = None
    if cached_entry is not None and cached_entry[0] == fingerprint:
        cached = cached_entry[1]

    if cached is None:
        try:
            import trimesh
        except Exception as exc:  # pragma: no cover - import failure is environment-specific
            raise RuntimeError(
                "Surface mesh placement requires the optional 'trimesh' dependency"
            ) from exc
        loaded = trimesh.load_mesh(mesh_path, force="mesh")
        if not isinstance(loaded, trimesh.Trimesh):
            raise ValidationError(
                f"Expected mesh geometry at {mesh_path}, got {type(loaded).__name__}"
            )
        _TRIMESH_CACHE[mesh_path] = (fingerprint, loaded)
        cached = loaded
    if mesh.scale is None:
        return cached
    scaled = cached.copy()
    scaled.apply_scale(mesh.scale)
    return scaled


def _closest_point_on_box(point: Vec3, size: Vec3) -> tuple[Vec3, Vec3]:
    hx, hy, hz = float(size[0]) * 0.5, float(size[1]) * 0.5, float(size[2]) * 0.5
    x, y, z = point
    clamped = (
        _clamp(x, -hx, hx),
        _clamp(y, -hy, hy),
        _clamp(z, -hz, hz),
    )
    inside = abs(x) <= hx and abs(y) <= hy and abs(z) <= hz
    if inside:
        face_dist = (hx - abs(x), hy - abs(y), hz - abs(z))
        axis = min(range(3), key=lambda idx: face_dist[idx])
        sign = 1.0 if point[axis] >= 0.0 else -1.0
        nearest = [x, y, z]
        nearest[axis] = (hx, hy, hz)[axis] * sign
        normal = [0.0, 0.0, 0.0]
        normal[axis] = sign
        return (nearest[0], nearest[1], nearest[2]), (normal[0], normal[1], normal[2])

    dx = x - clamped[0]
    dy = y - clamped[1]
    dz = z - clamped[2]
    axis = max(range(3), key=lambda idx: abs((dx, dy, dz)[idx]))
    delta = (dx, dy, dz)[axis]
    sign = 1.0 if delta >= 0.0 else -1.0
    normal = [0.0, 0.0, 0.0]
    normal[axis] = sign
    return clamped, (normal[0], normal[1], normal[2])


def _closest_point_on_cylinder(point: Vec3, radius: float, length: float) -> tuple[Vec3, Vec3]:
    x, y, z = point
    half = float(length) * 0.5
    radial = math.hypot(x, y)
    if radial <= _EPS:
        rx, ry = 1.0, 0.0
    else:
        rx, ry = x / radial, y / radial

    side = (rx * radius, ry * radius, _clamp(z, -half, half))
    side_d2 = (x - side[0]) ** 2 + (y - side[1]) ** 2 + (z - side[2]) ** 2

    def cap_candidate(sign: float) -> tuple[Vec3, float]:
        cap_z = sign * half
        if radial <= radius:
            cap_xy = (x, y)
        else:
            cap_xy = (rx * radius, ry * radius)
        cap = (cap_xy[0], cap_xy[1], cap_z)
        d2 = (x - cap[0]) ** 2 + (y - cap[1]) ** 2 + (z - cap[2]) ** 2
        return cap, d2

    top, top_d2 = cap_candidate(+1.0)
    bottom, bottom_d2 = cap_candidate(-1.0)
    if side_d2 <= top_d2 and side_d2 <= bottom_d2:
        return side, (rx, ry, 0.0)
    if top_d2 <= bottom_d2:
        return top, (0.0, 0.0, 1.0)
    return bottom, (0.0, 0.0, -1.0)


def _query_mesh_surface_local(
    mesh: Mesh, point_local: Vec3, *, asset_root: Optional[Path]
) -> _SurfaceHit:
    local_mesh = _load_trimesh_mesh(mesh, asset_root=asset_root)
    try:
        query = local_mesh.nearest.on_surface([point_local])
    except ModuleNotFoundError as exc:  # pragma: no cover - depends on native environment
        if exc.name == "rtree":
            raise RuntimeError(
                "Mesh surface placement requires the optional 'rtree' dependency"
            ) from exc
        raise
    closest = query[0][0]
    distance = float(query[1][0])
    face_index = int(query[2][0])
    normal = tuple(float(v) for v in local_mesh.face_normals[face_index])
    to_query = (
        float(point_local[0]) - float(closest[0]),
        float(point_local[1]) - float(closest[1]),
        float(point_local[2]) - float(closest[2]),
    )
    if _dot(normal, to_query) < 0.0:
        normal = (-normal[0], -normal[1], -normal[2])
    normal = _normalize(normal, name="mesh face normal")
    return _SurfaceHit(
        point=(float(closest[0]), float(closest[1]), float(closest[2])),
        normal=normal,
        distance=distance,
    )


def _query_surface_on_element(
    geometry: Geometry,
    tf: Mat4,
    *,
    point_hint: Vec3,
    asset_root: Optional[Path],
) -> _SurfaceHit:
    geometry, tf = _expand_mesh_provenance(geometry, tf)
    inv_tf = _mat4_inverse_rigid(tf)
    point_local = _mat4_vec3(inv_tf, point_hint)

    if isinstance(geometry, Sphere):
        radius = float(geometry.radius)
        direction = point_local
        if _norm(direction) <= _EPS:
            direction = (1.0, 0.0, 0.0)
        normal_local = _normalize(direction, name="sphere query direction")
        surface_local = (
            normal_local[0] * radius,
            normal_local[1] * radius,
            normal_local[2] * radius,
        )
        distance = _norm(
            (
                point_local[0] - surface_local[0],
                point_local[1] - surface_local[1],
                point_local[2] - surface_local[2],
            )
        )
    elif isinstance(geometry, Box):
        surface_local, normal_local = _closest_point_on_box(point_local, geometry.size)
        distance = _norm(
            (
                point_local[0] - surface_local[0],
                point_local[1] - surface_local[1],
                point_local[2] - surface_local[2],
            )
        )
    elif isinstance(geometry, Cylinder):
        surface_local, normal_local = _closest_point_on_cylinder(
            point_local,
            radius=float(geometry.radius),
            length=float(geometry.length),
        )
        distance = _norm(
            (
                point_local[0] - surface_local[0],
                point_local[1] - surface_local[1],
                point_local[2] - surface_local[2],
            )
        )
    elif isinstance(geometry, Mesh):
        hit = _query_mesh_surface_local(geometry, point_local, asset_root=asset_root)
        surface_local = hit.point
        normal_local = hit.normal
        distance = hit.distance
    else:
        raise ValidationError(f"Unsupported surface geometry type: {type(geometry).__name__}")

    point_world = _mat4_vec3(tf, surface_local)
    normal_world = _normalize(
        _mat4_transform_direction(tf, normal_local),
        name="surface normal",
    )
    return _SurfaceHit(point=point_world, normal=normal_world, distance=distance)


def _query_subject_surface_at_point(
    subject: SurfaceSubject,
    *,
    point_hint: Vec3,
    asset_root: Optional[Union[str, Path, AssetContext]],
    prefer_collisions: bool,
) -> _SurfaceHit:
    root = _resolve_asset_root_for_subject(asset_root, subject)
    best: Optional[_SurfaceHit] = None
    for geometry, tf in _iter_subject_items(subject, prefer_collisions=prefer_collisions):
        hit = _query_surface_on_element(geometry, tf, point_hint=point_hint, asset_root=root)
        if best is None or hit.distance < best.distance:
            best = hit
    if best is None:
        raise ValidationError("target has no geometry to query")
    return best


def surface_frame(
    target: SurfaceSubject,
    *,
    point_hint: Optional[Vec3] = None,
    direction: Optional[Vec3] = None,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = False,
    up_hint: Vec3 = (0.0, 0.0, 1.0),
) -> SurfaceFrame:
    if (point_hint is None) == (direction is None):
        raise ValidationError("Exactly one of point_hint or direction must be provided")

    root = _resolve_asset_root_for_subject(asset_root, target)
    query_point = point_hint
    if direction is not None:
        dir_world = _normalize(direction, name="direction")
        aabb = _subject_world_aabb(target, asset_root=root, prefer_collisions=prefer_collisions)
        center = _aabb_center(aabb)
        size = _aabb_size(aabb)
        radius = max(_norm(size), 1e-3)
        query_point = (
            center[0] + dir_world[0] * radius * 4.0,
            center[1] + dir_world[1] * radius * 4.0,
            center[2] + dir_world[2] * radius * 4.0,
        )

    assert query_point is not None
    best = _query_subject_surface_at_point(
        target,
        point_hint=query_point,
        asset_root=root,
        prefer_collisions=prefer_collisions,
    )

    tangent_u, tangent_v = _build_surface_tangents(best.normal, up_hint)
    return SurfaceFrame(
        point=best.point,
        normal=best.normal,
        tangent_u=tangent_u,
        tangent_v=tangent_v,
    )


def _support_min_projection_on_element(
    geometry: Geometry,
    tf: Mat4,
    *,
    axis: Vec3,
    asset_root: Optional[Path],
) -> float:
    geometry, tf = _expand_mesh_provenance(geometry, tf)
    center = (float(tf[0][3]), float(tf[1][3]), float(tf[2][3]))
    rot = _mat3_from_mat4(tf)
    axis_local = _mat3_vec3(_mat3_transpose(rot), axis)

    if isinstance(geometry, Sphere):
        return _dot(center, axis) - float(geometry.radius)
    if isinstance(geometry, Box):
        hx, hy, hz = (
            float(geometry.size[0]) * 0.5,
            float(geometry.size[1]) * 0.5,
            float(geometry.size[2]) * 0.5,
        )
        half = hx * abs(axis_local[0]) + hy * abs(axis_local[1]) + hz * abs(axis_local[2])
        return _dot(center, axis) - half
    if isinstance(geometry, Cylinder):
        radial = math.hypot(axis_local[0], axis_local[1])
        half = float(geometry.radius) * radial + (float(geometry.length) * 0.5) * abs(axis_local[2])
        return _dot(center, axis) - half
    if isinstance(geometry, Mesh):
        local_mesh = _load_trimesh_mesh(geometry, asset_root=asset_root)
        min_proj = float("inf")
        for vertex in local_mesh.vertices:
            local_vertex = (float(vertex[0]), float(vertex[1]), float(vertex[2]))
            world_vertex = _mat4_vec3(tf, local_vertex)
            min_proj = min(min_proj, _dot(world_vertex, axis))
        if min_proj == float("inf"):
            raise ValidationError("mesh geometry has no vertices")
        return min_proj
    raise ValidationError(f"Unsupported child geometry type: {type(geometry).__name__}")


def _subject_min_projection(
    subject: SurfaceSubject,
    *,
    axis: Vec3,
    asset_root: Optional[Path],
    prefer_collisions: bool,
) -> float:
    projections = [
        _support_min_projection_on_element(
            geometry,
            tf,
            axis=axis,
            asset_root=asset_root,
        )
        for geometry, tf in _iter_subject_items(subject, prefer_collisions=prefer_collisions)
    ]
    if not projections:
        raise ValidationError("child has no geometry to compute support distance")
    return min(projections)


def _mesh_geometry_from_subject(
    mesh: MeshSubject,
    *,
    asset_root: Optional[Union[str, Path, AssetContext]],
) -> MeshGeometry:
    if isinstance(mesh, MeshGeometry):
        return mesh.copy()
    if isinstance(mesh, Mesh):
        local_mesh = _load_trimesh_mesh(
            mesh,
            asset_root=_resolve_asset_root_for_subject(asset_root, mesh),
        )
        vertices = [
            (float(vertex[0]), float(vertex[1]), float(vertex[2])) for vertex in local_mesh.vertices
        ]
        faces = [(int(face[0]), int(face[1]), int(face[2])) for face in local_mesh.faces]
        return MeshGeometry(vertices=vertices, faces=faces)
    raise ValidationError(f"mesh must be a MeshGeometry or Mesh, got {type(mesh).__name__}")


def _normalize_surface_wrap_mapping(mapping: SurfaceWrapMapping | str) -> SurfaceWrapMapping:
    mapping_key = str(mapping).strip().lower()
    if mapping_key not in {"auto", "intrinsic", "nearest"}:
        raise ValidationError("mapping must be one of: 'auto', 'intrinsic', 'nearest'")
    return mapping_key  # type: ignore[return-value]


def _resolve_surface_max_edge(
    *,
    surface_max_edge: Optional[float],
    max_edge: Optional[float],
) -> Optional[float]:
    if surface_max_edge is None:
        return max_edge
    if max_edge is None:
        return surface_max_edge
    if abs(float(surface_max_edge) - float(max_edge)) > 1e-12:
        raise ValidationError("Provide only one of surface_max_edge or max_edge")
    return surface_max_edge


def _subdivide_mesh_geometry_to_size(
    geometry: MeshGeometry,
    *,
    max_edge: Optional[float],
) -> MeshGeometry:
    if max_edge is None:
        return geometry
    max_edge_value = float(max_edge)
    if max_edge_value <= 0.0:
        raise ValidationError("max_edge must be positive")
    if not geometry.vertices or not geometry.faces:
        return geometry
    try:
        import trimesh
    except Exception as exc:  # pragma: no cover - import failure is environment-specific
        raise RuntimeError(
            "Mesh surface wrapping with max_edge requires the optional 'trimesh' dependency"
        ) from exc
    vertices, faces = trimesh.remesh.subdivide_to_size(
        geometry.vertices,
        geometry.faces,
        max_edge=max_edge_value,
    )
    return MeshGeometry(
        vertices=[(float(vertex[0]), float(vertex[1]), float(vertex[2])) for vertex in vertices],
        faces=[(int(face[0]), int(face[1]), int(face[2])) for face in faces],
    )


def _normalize_profile_points(
    profile: Iterable[tuple[float, float]],
) -> list[tuple[float, float]]:
    try:
        return _ensure_ccw(_profile_points_2d(profile))
    except ValueError as exc:
        raise ValidationError(str(exc)) from exc


def _normalize_hole_profiles(
    hole_profiles: Iterable[Iterable[tuple[float, float]]],
) -> list[list[tuple[float, float]]]:
    holes: list[list[tuple[float, float]]] = []
    for hole_profile in hole_profiles:
        holes.append(_normalize_profile_points(hole_profile))
    return holes


def _fallback_profile_solid(
    profile: list[tuple[float, float]],
    *,
    hole_profiles: list[list[tuple[float, float]]],
    thickness: float,
) -> MeshGeometry:
    if hole_profiles:
        solid = ExtrudeWithHolesGeometry(
            profile,
            hole_profiles,
            thickness,
            cap=True,
            center=False,
            closed=True,
        )
    else:
        solid = ExtrudeGeometry.from_z0(profile, thickness, cap=True, closed=True)
    solid.translate(0.0, 0.0, -float(thickness))
    return solid


def _point_distance_to_segment_xy(
    point: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> tuple[float, float]:
    seg_x = float(b[0]) - float(a[0])
    seg_y = float(b[1]) - float(a[1])
    seg_len2 = seg_x * seg_x + seg_y * seg_y
    if seg_len2 <= _EPS:
        dx = float(point[0]) - float(a[0])
        dy = float(point[1]) - float(a[1])
        return math.hypot(dx, dy), 0.0
    rel_x = float(point[0]) - float(a[0])
    rel_y = float(point[1]) - float(a[1])
    t = _clamp((rel_x * seg_x + rel_y * seg_y) / seg_len2, 0.0, 1.0)
    closest = (float(a[0]) + seg_x * t, float(a[1]) + seg_y * t)
    return math.hypot(float(point[0]) - closest[0], float(point[1]) - closest[1]), t


def _boundary_loop_for_planar_mesh(
    geometry: MeshGeometry,
    *,
    boundary_profile: list[tuple[float, float]],
) -> list[int]:
    edge_counts: dict[tuple[int, int], int] = {}
    for a, b, c in geometry.faces:
        for u, v in ((a, b), (b, c), (c, a)):
            key = (u, v) if u < v else (v, u)
            edge_counts[key] = edge_counts.get(key, 0) + 1

    boundary_vertices = sorted(
        {index for edge, count in edge_counts.items() if count == 1 for index in edge}
    )
    if not boundary_vertices:
        raise ValidationError("profile mesh has no boundary loop")

    cumulative = [0.0]
    for i, a in enumerate(boundary_profile):
        b = boundary_profile[(i + 1) % len(boundary_profile)]
        cumulative.append(
            cumulative[-1] + math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        )

    perimeter = cumulative[-1]
    if perimeter <= _EPS:
        raise ValidationError("profile perimeter must be non-zero")

    ordered = []
    for index in boundary_vertices:
        point = (float(geometry.vertices[index][0]), float(geometry.vertices[index][1]))
        best_distance = float("inf")
        best_arclength = 0.0
        for segment_index, a in enumerate(boundary_profile):
            b = boundary_profile[(segment_index + 1) % len(boundary_profile)]
            distance, t = _point_distance_to_segment_xy(point, a, b)
            if distance + 1e-9 < best_distance:
                best_distance = distance
                best_arclength = (
                    cumulative[segment_index]
                    + (cumulative[segment_index + 1] - cumulative[segment_index]) * t
                )
        if best_distance > 1e-5:
            raise ValidationError("profile mesh boundary deviated from source profile")
        if best_arclength >= perimeter - 1e-8:
            best_arclength = 0.0
        ordered.append((best_arclength, index))

    ordered.sort(key=lambda item: item[0])
    loop = [index for _, index in ordered]

    area = 0.0
    for i, index in enumerate(loop):
        x1, y1, _ = geometry.vertices[index]
        x2, y2, _ = geometry.vertices[loop[(i + 1) % len(loop)]]
        area += x1 * y2 - x2 * y1
    if area < 0.0:
        loop.reverse()
    return loop


def _planar_profile_cap(
    profile: Iterable[tuple[float, float]],
    *,
    max_edge: Optional[float],
) -> MeshGeometry:
    points = _normalize_profile_points(profile)
    try:
        faces = _triangulate_polygon(points)
    except ValueError as exc:
        raise ValidationError(str(exc)) from exc

    cap = MeshGeometry(
        vertices=[(float(x), float(y), 0.0) for x, y in points],
        faces=[(int(a), int(b), int(c)) for a, b, c in faces],
    )
    if not cap.faces:
        raise ValidationError("profile triangulation produced no faces")
    return _subdivide_mesh_geometry_to_size(cap, max_edge=max_edge)


def _solid_from_planar_profile_cap(
    cap: MeshGeometry,
    *,
    boundary_profile: list[tuple[float, float]],
    thickness: float,
) -> MeshGeometry:
    thickness_value = float(thickness)
    if thickness_value <= 0.0:
        raise ValidationError("thickness must be positive")

    loop = _boundary_loop_for_planar_mesh(cap, boundary_profile=boundary_profile)
    vertex_count = len(cap.vertices)
    solid = MeshGeometry(
        vertices=[
            *[(float(x), float(y), 0.0) for x, y, _ in cap.vertices],
            *[(float(x), float(y), -thickness_value) for x, y, _ in cap.vertices],
        ],
        faces=[],
    )

    for a, b, c in cap.faces:
        solid.add_face(a, b, c)

    offset = vertex_count
    for a, b, c in cap.faces:
        solid.add_face(offset + c, offset + b, offset + a)

    for i, a in enumerate(loop):
        b = loop[(i + 1) % len(loop)]
        solid.add_face(a, b, offset + b)
        solid.add_face(a, offset + b, offset + a)
    return solid


def _subject_single_world_sphere(
    subject: SurfaceSubject,
    *,
    prefer_collisions: bool,
) -> tuple[Vec3, float] | None:
    items = _iter_subject_items(subject, prefer_collisions=prefer_collisions)
    if len(items) != 1:
        return None
    geometry, tf = _expand_mesh_provenance(items[0][0], items[0][1])
    if not isinstance(geometry, Sphere):
        return None
    center = _mat4_vec3(tf, (0.0, 0.0, 0.0))
    return center, float(geometry.radius)


def _subject_single_world_cylinder(
    subject: SurfaceSubject,
    *,
    prefer_collisions: bool,
) -> tuple[Vec3, Vec3, float, float] | None:
    items = _iter_subject_items(subject, prefer_collisions=prefer_collisions)
    if len(items) != 1:
        return None
    geometry, tf = _expand_mesh_provenance(items[0][0], items[0][1])
    if not isinstance(geometry, Cylinder):
        return None
    center = _mat4_vec3(tf, (0.0, 0.0, 0.0))
    axis_world = _normalize(
        _mat4_transform_direction(tf, (0.0, 0.0, 1.0)),
        name="cylinder axis",
    )
    return center, axis_world, float(geometry.radius), float(geometry.length)


def _wrap_mesh_onto_sphere(
    geometry: MeshGeometry,
    *,
    frame: SurfaceFrame,
    sphere_center: Vec3,
    sphere_radius: float,
    child_axis: str,
    visible_relief: float,
    spin: float,
) -> MeshGeometry:
    rot = _rotation_for_surface_frame(frame, child_axis=child_axis, spin=float(spin))
    axis_local = _axis_vector(child_axis)
    outer_support = max(_dot(vertex, axis_local) for vertex in geometry.vertices)

    wrapped_vertices: list[Vec3] = []
    for vertex in geometry.vertices:
        local_proj = _dot(vertex, axis_local)
        depth_behind = outer_support - local_proj
        in_plane_local = (
            float(vertex[0]) - axis_local[0] * local_proj,
            float(vertex[1]) - axis_local[1] * local_proj,
            float(vertex[2]) - axis_local[2] * local_proj,
        )
        tangent_offset = _mat3_vec3(rot, in_plane_local)
        tangent_distance = _norm(tangent_offset)
        if tangent_distance <= _EPS:
            surface_normal = frame.normal
        else:
            tangent_dir = (
                tangent_offset[0] / tangent_distance,
                tangent_offset[1] / tangent_distance,
                tangent_offset[2] / tangent_distance,
            )
            angle = tangent_distance / float(sphere_radius)
            surface_normal = _normalize(
                (
                    math.cos(angle) * frame.normal[0] + math.sin(angle) * tangent_dir[0],
                    math.cos(angle) * frame.normal[1] + math.sin(angle) * tangent_dir[1],
                    math.cos(angle) * frame.normal[2] + math.sin(angle) * tangent_dir[2],
                ),
                name="sphere wrap normal",
            )
        surface_point = (
            sphere_center[0] + surface_normal[0] * float(sphere_radius),
            sphere_center[1] + surface_normal[1] * float(sphere_radius),
            sphere_center[2] + surface_normal[2] * float(sphere_radius),
        )
        wrapped_vertices.append(
            (
                surface_point[0] + surface_normal[0] * (float(visible_relief) - depth_behind),
                surface_point[1] + surface_normal[1] * (float(visible_relief) - depth_behind),
                surface_point[2] + surface_normal[2] * (float(visible_relief) - depth_behind),
            )
        )

    return MeshGeometry(vertices=wrapped_vertices, faces=list(geometry.faces))


def _wrap_mesh_onto_cylinder(
    geometry: MeshGeometry,
    *,
    frame: SurfaceFrame,
    cylinder_center: Vec3,
    cylinder_axis: Vec3,
    cylinder_radius: float,
    cylinder_length: float,
    child_axis: str,
    visible_relief: float,
    spin: float,
) -> MeshGeometry | None:
    if abs(_dot(frame.normal, cylinder_axis)) >= 0.2:
        return None

    rel_anchor = (
        frame.point[0] - cylinder_center[0],
        frame.point[1] - cylinder_center[1],
        frame.point[2] - cylinder_center[2],
    )
    anchor_axial = _dot(rel_anchor, cylinder_axis)
    anchor_radial = (
        rel_anchor[0] - cylinder_axis[0] * anchor_axial,
        rel_anchor[1] - cylinder_axis[1] * anchor_axial,
        rel_anchor[2] - cylinder_axis[2] * anchor_axial,
    )
    if _norm(anchor_radial) <= _EPS:
        return None

    anchor_normal = _normalize(anchor_radial, name="cylinder wrap normal")
    circum_axis = _normalize(
        _cross(cylinder_axis, anchor_normal),
        name="cylinder circumferential tangent",
    )
    half_length = float(cylinder_length) * 0.5
    rot = _rotation_for_surface_frame(frame, child_axis=child_axis, spin=float(spin))
    axis_local = _axis_vector(child_axis)
    outer_support = max(_dot(vertex, axis_local) for vertex in geometry.vertices)

    wrapped_vertices: list[Vec3] = []
    for vertex in geometry.vertices:
        local_proj = _dot(vertex, axis_local)
        depth_behind = outer_support - local_proj
        in_plane_local = (
            float(vertex[0]) - axis_local[0] * local_proj,
            float(vertex[1]) - axis_local[1] * local_proj,
            float(vertex[2]) - axis_local[2] * local_proj,
        )
        tangent_offset = _mat3_vec3(rot, in_plane_local)
        circum_distance = _dot(tangent_offset, circum_axis)
        axial_distance = _dot(tangent_offset, cylinder_axis)
        axial = anchor_axial + axial_distance
        if abs(axial) > half_length + 1e-9:
            return None

        angle = circum_distance / float(cylinder_radius)
        turn = _axis_angle_matrix(cylinder_axis, angle)
        surface_normal = _normalize(
            _mat3_vec3(turn, anchor_normal),
            name="cylinder wrap rotated normal",
        )
        surface_point = (
            cylinder_center[0]
            + cylinder_axis[0] * axial
            + surface_normal[0] * float(cylinder_radius),
            cylinder_center[1]
            + cylinder_axis[1] * axial
            + surface_normal[1] * float(cylinder_radius),
            cylinder_center[2]
            + cylinder_axis[2] * axial
            + surface_normal[2] * float(cylinder_radius),
        )
        wrapped_vertices.append(
            (
                surface_point[0] + surface_normal[0] * (float(visible_relief) - depth_behind),
                surface_point[1] + surface_normal[1] * (float(visible_relief) - depth_behind),
                surface_point[2] + surface_normal[2] * (float(visible_relief) - depth_behind),
            )
        )

    return MeshGeometry(vertices=wrapped_vertices, faces=list(geometry.faces))


def _wrap_mesh_onto_surface_nearest(
    geometry: MeshGeometry,
    *,
    frame: SurfaceFrame,
    target: SurfaceSubject,
    child_axis: str,
    visible_relief: float,
    spin: float,
    asset_root: Optional[Union[str, Path, AssetContext]],
    prefer_collisions: bool,
) -> MeshGeometry:
    rot = _rotation_for_surface_frame(frame, child_axis=child_axis, spin=float(spin))
    axis_local = _axis_vector(child_axis)
    outer_support = max(_dot(vertex, axis_local) for vertex in geometry.vertices)

    wrapped_vertices: list[Vec3] = []
    for vertex in geometry.vertices:
        local_proj = _dot(vertex, axis_local)
        depth_behind = outer_support - local_proj
        in_plane_local = (
            float(vertex[0]) - axis_local[0] * local_proj,
            float(vertex[1]) - axis_local[1] * local_proj,
            float(vertex[2]) - axis_local[2] * local_proj,
        )
        plane_offset = _mat3_vec3(rot, in_plane_local)
        query_point = (
            frame.point[0] + plane_offset[0],
            frame.point[1] + plane_offset[1],
            frame.point[2] + plane_offset[2],
        )
        hit = _query_subject_surface_at_point(
            target,
            point_hint=query_point,
            asset_root=asset_root,
            prefer_collisions=prefer_collisions,
        )
        wrapped_vertices.append(
            (
                hit.point[0] + hit.normal[0] * (float(visible_relief) - depth_behind),
                hit.point[1] + hit.normal[1] * (float(visible_relief) - depth_behind),
                hit.point[2] + hit.normal[2] * (float(visible_relief) - depth_behind),
            )
        )

    return MeshGeometry(vertices=wrapped_vertices, faces=list(geometry.faces))


def wrap_mesh_onto_surface(
    mesh: MeshSubject,
    target: SurfaceSubject,
    *,
    point_hint: Optional[Vec3] = None,
    direction: Optional[Vec3] = None,
    child_axis: str = "+z",
    visible_relief: float = 0.0,
    mapping: SurfaceWrapMapping | str = "auto",
    surface_max_edge: Optional[float] = None,
    max_edge: Optional[float] = None,
    spin: float = 0.0,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = False,
    up_hint: Vec3 = (0.0, 0.0, 1.0),
) -> MeshGeometry:
    """
    Conform a mesh onto a target surface and return a baked ``MeshGeometry``.

    This differs from :func:`place_on_surface`, which only returns a rigid
    transform. Use this helper when the child must follow curvature, like a
    sticker on a globe or label on a cylindrical shell.

    ``mapping`` controls how the target is interpreted:

    - ``"auto"`` prefers intrinsic analytic wraps when supported, otherwise falls
      back to nearest-surface projection.
    - ``"intrinsic"`` requires an intrinsic backend and raises if unsupported.
    - ``"nearest"`` always uses nearest-surface projection.
    """
    mapping_mode = _normalize_surface_wrap_mapping(mapping)
    frame = surface_frame(
        target,
        point_hint=point_hint,
        direction=direction,
        asset_root=asset_root,
        prefer_collisions=prefer_collisions,
        up_hint=up_hint,
    )
    resolved_max_edge = _resolve_surface_max_edge(
        surface_max_edge=surface_max_edge,
        max_edge=max_edge,
    )
    geometry = _mesh_geometry_from_subject(mesh, asset_root=asset_root)
    geometry = _subdivide_mesh_geometry_to_size(geometry, max_edge=resolved_max_edge)
    if not geometry.vertices:
        raise ValidationError("mesh has no vertices")
    if mapping_mode != "nearest":
        sphere_info = _subject_single_world_sphere(
            target,
            prefer_collisions=prefer_collisions,
        )
        if sphere_info is not None:
            sphere_center, sphere_radius = sphere_info
            return _wrap_mesh_onto_sphere(
                geometry,
                frame=frame,
                sphere_center=sphere_center,
                sphere_radius=sphere_radius,
                child_axis=child_axis,
                visible_relief=float(visible_relief),
                spin=float(spin),
            )

        cylinder_info = _subject_single_world_cylinder(
            target,
            prefer_collisions=prefer_collisions,
        )
        if cylinder_info is not None:
            cylinder_center, cylinder_axis, cylinder_radius, cylinder_length = cylinder_info
            intrinsic = _wrap_mesh_onto_cylinder(
                geometry,
                frame=frame,
                cylinder_center=cylinder_center,
                cylinder_axis=cylinder_axis,
                cylinder_radius=cylinder_radius,
                cylinder_length=cylinder_length,
                child_axis=child_axis,
                visible_relief=float(visible_relief),
                spin=float(spin),
            )
            if intrinsic is not None:
                return intrinsic

        if mapping_mode == "intrinsic":
            raise ValidationError(
                "Intrinsic surface wrapping is only supported for spheres and cylinder sidewalls"
            )

    return _wrap_mesh_onto_surface_nearest(
        geometry,
        frame=frame,
        target=target,
        child_axis=child_axis,
        visible_relief=float(visible_relief),
        spin=float(spin),
        asset_root=asset_root,
        prefer_collisions=prefer_collisions,
    )


def wrap_profile_onto_surface(
    profile: Iterable[tuple[float, float]],
    target: SurfaceSubject,
    *,
    thickness: float,
    hole_profiles: Iterable[Iterable[tuple[float, float]]] = (),
    point_hint: Optional[Vec3] = None,
    direction: Optional[Vec3] = None,
    visible_relief: float = 0.0,
    mapping: SurfaceWrapMapping | str = "auto",
    surface_max_edge: Optional[float] = None,
    max_edge: Optional[float] = None,
    spin: float = 0.0,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = False,
    up_hint: Vec3 = (0.0, 0.0, 1.0),
) -> MeshGeometry:
    """
    Wrap a 2D profile with thickness directly onto a target surface.

    The profile is interpreted in the local XY plane, with its visible face on
    local ``z=0`` and thickness extending inward along ``-z``. This matches the
    common "sticker" or "thin plaque" case without requiring callers to build
    and remesh a solid themselves. ``hole_profiles`` can be used for through-cut
    features such as badges, vents, or label windows. Hole profiles currently
    take the extrude-based fallback path directly instead of the planar-cap path.
    """
    thickness_value = float(thickness)
    if thickness_value <= 0.0:
        raise ValidationError("thickness must be positive")
    resolved_max_edge = _resolve_surface_max_edge(
        surface_max_edge=surface_max_edge,
        max_edge=max_edge,
    )
    points = _normalize_profile_points(profile)
    holes = _normalize_hole_profiles(hole_profiles)
    if holes:
        logger.warning(
            "wrap_profile_onto_surface using extrude fallback because hole_profiles were provided: hole_count=%s thickness=%.6g",
            len(holes),
            thickness_value,
        )
        fallback = _fallback_profile_solid(
            points,
            hole_profiles=holes,
            thickness=thickness_value,
        )
        return wrap_mesh_onto_surface(
            fallback,
            target,
            point_hint=point_hint,
            direction=direction,
            child_axis="+z",
            visible_relief=visible_relief,
            mapping=mapping,
            surface_max_edge=resolved_max_edge,
            spin=spin,
            asset_root=asset_root,
            prefer_collisions=prefer_collisions,
            up_hint=up_hint,
        )

    cap = _planar_profile_cap(points, max_edge=resolved_max_edge)
    try:
        solid = _solid_from_planar_profile_cap(
            cap,
            boundary_profile=points,
            thickness=thickness_value,
        )
        return wrap_mesh_onto_surface(
            solid,
            target,
            point_hint=point_hint,
            direction=direction,
            child_axis="+z",
            visible_relief=visible_relief,
            mapping=mapping,
            spin=spin,
            asset_root=asset_root,
            prefer_collisions=prefer_collisions,
            up_hint=up_hint,
        )
    except ValidationError as exc:
        message = str(exc)
        if "profile mesh boundary" not in message:
            raise
        logger.warning(
            "wrap_profile_onto_surface using extrude fallback after planar cap reconstruction failed: %s",
            message,
        )

    fallback = _fallback_profile_solid(
        points,
        hole_profiles=[],
        thickness=thickness_value,
    )
    return wrap_mesh_onto_surface(
        fallback,
        target,
        point_hint=point_hint,
        direction=direction,
        child_axis="+z",
        visible_relief=visible_relief,
        mapping=mapping,
        surface_max_edge=resolved_max_edge,
        spin=spin,
        asset_root=asset_root,
        prefer_collisions=prefer_collisions,
        up_hint=up_hint,
    )


def place_on_surface(
    child: SurfaceSubject,
    target: SurfaceSubject,
    *,
    point_hint: Optional[Vec3] = None,
    direction: Optional[Vec3] = None,
    child_axis: str = "+z",
    clearance: float = 0.0,
    spin: float = 0.0,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = False,
    child_prefer_collisions: bool = False,
    up_hint: Vec3 = (0.0, 0.0, 1.0),
) -> Origin:
    """
    Rigidly place a child on a target surface using a tangent-frame mount.

    This returns only a transform. It does not deform the child to follow
    curvature. For decals, labels, or other geometry that must conform to a
    curved surface, use :func:`wrap_mesh_onto_surface` instead.
    """
    frame = surface_frame(
        target,
        point_hint=point_hint,
        direction=direction,
        asset_root=asset_root,
        prefer_collisions=prefer_collisions,
        up_hint=up_hint,
    )
    child_root = _resolve_asset_root_for_subject(asset_root, child)
    child_axis_vec = _axis_vector(child_axis)
    mount_proj = _subject_min_projection(
        child,
        axis=child_axis_vec,
        asset_root=child_root,
        prefer_collisions=child_prefer_collisions,
    )
    offset = float(clearance) - float(mount_proj)
    xyz = (
        frame.point[0] + frame.normal[0] * offset,
        frame.point[1] + frame.normal[1] * offset,
        frame.point[2] + frame.normal[2] * offset,
    )
    rot = _rotation_for_surface_frame(frame, child_axis=child_axis, spin=float(spin))
    return Origin(xyz=xyz, rpy=_mat3_to_rpy(rot))


def proud_for_flush_mount(
    child_link: Part,
    *,
    axis: str = "z",
    clearance: float = 0.0,
    asset_root: Optional[Union[str, Path, AssetContext]] = None,
    prefer_collisions: bool = True,
) -> float:
    """
    Compute a ``proud`` distance that makes a face-mounted child sit flush.

    When you mount a child using ``place_on_face(..., proud=0)``, the child
    origin sits on the parent face. If the child's geometry is centered about
    its link origin, half the geometry will be embedded in the parent.

    This helper returns half the child's thickness along the selected local axis
    (plus optional clearance), which is typically the correct ``proud`` for a
    flush mount when using ``place_on_face``.
    """
    axis_key = axis.strip().lower()
    axis_idx = {"x": 0, "y": 1, "z": 2}.get(axis_key)
    if axis_idx is None:
        raise ValidationError(f"Invalid axis {axis!r}; expected 'x', 'y', or 'z'.")

    root = resolve_asset_root(asset_root, child_link)
    aabb = link_local_aabb(child_link, asset_root=root, prefer_collisions=prefer_collisions)
    if aabb is None:
        raise ValidationError(f"Child link {child_link.name!r} has no geometry to compute AABB")

    (mn, mx) = aabb
    thickness = float(mx[axis_idx]) - float(mn[axis_idx])
    return 0.5 * thickness + float(clearance)
