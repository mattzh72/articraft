from __future__ import annotations

import itertools
import math
import os
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .assets import resolve_asset_root, resolve_mesh_path
from .errors import ValidationError
from .exact_collisions import compile_object_model_with_exact_collisions
from .types import ArticulationType, Box, Cylinder, Mesh, Origin, Part, Sphere

Vec3 = Tuple[float, float, float]
AABB = Tuple[Vec3, Vec3]
Mat3 = Tuple[Vec3, Vec3, Vec3]
Mat4 = Tuple[Tuple[float, float, float, float], ...]

_EPS = 1e-9


def _dot(a: Vec3, b: Vec3) -> float:
    return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _aabb_center(aabb: AABB) -> Vec3:
    (mn, mx) = aabb
    return ((mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5)


def _aabb_size(aabb: AABB) -> Vec3:
    (mn, mx) = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


@dataclass(frozen=True)
class _OBB:
    center: Vec3
    axes: Mat3  # orthonormal axes in world coordinates
    half_extents: Vec3


def _mat4_axes(mat: Mat4) -> Mat3:
    # Columns of the rotation part represent the world direction of the local basis axes.
    ax = (mat[0][0], mat[1][0], mat[2][0])
    ay = (mat[0][1], mat[1][1], mat[2][1])
    az = (mat[0][2], mat[1][2], mat[2][2])
    return (_normalize(ax), _normalize(ay), _normalize(az))


def _obb_from_local_aabb_and_tf(local_aabb: AABB, tf: Mat4) -> _OBB:
    (mn, mx) = local_aabb
    center_local = ((mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5)
    half = ((mx[0] - mn[0]) * 0.5, (mx[1] - mn[1]) * 0.5, (mx[2] - mn[2]) * 0.5)
    return _OBB(center=_mat4_vec3(tf, center_local), axes=_mat4_axes(tf), half_extents=half)


def _obb_overlaps(a: _OBB, b: _OBB) -> bool:
    """
    Separating Axis Theorem OBB-vs-OBB overlap test.

    This is still conservative with respect to the true underlying geometry (especially meshes),
    but substantially less conservative than using world-axis AABBs for rotated parts.
    """

    # Rotation matrix expressing B in A's frame: R[i][j] = Ai dot Bj.
    r = [[_dot(a.axes[i], b.axes[j]) for j in range(3)] for i in range(3)]
    abs_r = [[abs(r[i][j]) + _EPS for j in range(3)] for i in range(3)]

    # Translation in A's frame.
    t_world = _sub(b.center, a.center)
    t = [_dot(t_world, a.axes[i]) for i in range(3)]

    a_e = a.half_extents
    b_e = b.half_extents

    # Test axes L = A0, A1, A2
    for i in range(3):
        ra = a_e[i]
        rb = b_e[0] * abs_r[i][0] + b_e[1] * abs_r[i][1] + b_e[2] * abs_r[i][2]
        if abs(t[i]) > ra + rb:
            return False

    # Test axes L = B0, B1, B2
    for j in range(3):
        ra = a_e[0] * abs_r[0][j] + a_e[1] * abs_r[1][j] + a_e[2] * abs_r[2][j]
        rb = b_e[j]
        tj = t[0] * r[0][j] + t[1] * r[1][j] + t[2] * r[2][j]
        if abs(tj) > ra + rb:
            return False

    # Test axis L = Ai x Bj
    for i in range(3):
        i1 = (i + 1) % 3
        i2 = (i + 2) % 3
        for j in range(3):
            j1 = (j + 1) % 3
            j2 = (j + 2) % 3
            ra = a_e[i1] * abs_r[i2][j] + a_e[i2] * abs_r[i1][j]
            rb = b_e[j1] * abs_r[i][j2] + b_e[j2] * abs_r[i][j1]
            t_ij = abs(t[i2] * r[i1][j] - t[i1] * r[i2][j])
            if t_ij > ra + rb:
                return False

    return True


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> Mat3:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mat3_vec3(mat: Mat3, vec: Vec3) -> Vec3:
    x, y, z = vec
    return (
        mat[0][0] * x + mat[0][1] * y + mat[0][2] * z,
        mat[1][0] * x + mat[1][1] * y + mat[1][2] * z,
        mat[2][0] * x + mat[2][1] * y + mat[2][2] * z,
    )


def _mat4_mul(a: Mat4, b: Mat4) -> Mat4:
    out: list[list[float]] = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = (
                a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j]
            )
    return tuple(tuple(row) for row in out)  # type: ignore[return-value]


def _mat4_vec3(mat: Mat4, vec: Vec3) -> Vec3:
    x, y, z = vec
    tx = mat[0][0] * x + mat[0][1] * y + mat[0][2] * z + mat[0][3]
    ty = mat[1][0] * x + mat[1][1] * y + mat[1][2] * z + mat[1][3]
    tz = mat[2][0] * x + mat[2][1] * y + mat[2][2] * z + mat[2][3]
    return (tx, ty, tz)


def _identity4() -> Mat4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _origin_to_mat4(origin: Origin) -> Mat4:
    ox, oy, oz = origin.xyz
    rr, rp, ry = origin.rpy
    r = _rpy_matrix(float(rr), float(rp), float(ry))
    return (
        (r[0][0], r[0][1], r[0][2], float(ox)),
        (r[1][0], r[1][1], r[1][2], float(oy)),
        (r[2][0], r[2][1], r[2][2], float(oz)),
        (0.0, 0.0, 0.0, 1.0),
    )


def _normalize(v: Vec3) -> Vec3:
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    n = math.sqrt(x * x + y * y + z * z)
    if n <= 0:
        return (0.0, 0.0, 0.0)
    return (x / n, y / n, z / n)


def _axis_angle_matrix(axis: Vec3, angle: float) -> Mat3:
    ax = _normalize(axis)
    if abs(ax[0]) <= _EPS and abs(ax[1]) <= _EPS and abs(ax[2]) <= _EPS:
        raise ValidationError("Articulation axis must be non-zero")
    x, y, z = ax
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def _mat3_to_mat4(rot: Mat3, *, translation: Vec3 = (0.0, 0.0, 0.0)) -> Mat4:
    tx, ty, tz = translation
    return (
        (rot[0][0], rot[0][1], rot[0][2], float(tx)),
        (rot[1][0], rot[1][1], rot[1][2], float(ty)),
        (rot[2][0], rot[2][1], rot[2][2], float(tz)),
        (0.0, 0.0, 0.0, 1.0),
    )


def _aabb_intersection_depth(a: AABB, b: AABB) -> Vec3:
    (amin_x, amin_y, amin_z), (amax_x, amax_y, amax_z) = a
    (bmin_x, bmin_y, bmin_z), (bmax_x, bmax_y, bmax_z) = b
    dx = min(amax_x, bmax_x) - max(amin_x, bmin_x)
    dy = min(amax_y, bmax_y) - max(amin_y, bmin_y)
    dz = min(amax_z, bmax_z) - max(amin_z, bmin_z)
    return (dx, dy, dz)


def _aabb_intersection_volume(a: AABB, b: AABB) -> float:
    dx, dy, dz = _aabb_intersection_depth(a, b)
    if dx <= 0 or dy <= 0 or dz <= 0:
        return 0.0
    return float(dx * dy * dz)


def _aabbs_touch_or_overlap(a: AABB, b: AABB, *, tol: float) -> bool:
    (amin_x, amin_y, amin_z), (amax_x, amax_y, amax_z) = a
    (bmin_x, bmin_y, bmin_z), (bmax_x, bmax_y, bmax_z) = b
    return not (
        (amax_x < bmin_x - tol)
        or (bmax_x < amin_x - tol)
        or (amax_y < bmin_y - tol)
        or (bmax_y < amin_y - tol)
        or (amax_z < bmin_z - tol)
        or (bmax_z < amin_z - tol)
    )


def _aabb_separation_distance(a: AABB, b: AABB) -> float:
    (amin_x, amin_y, amin_z), (amax_x, amax_y, amax_z) = a
    (bmin_x, bmin_y, bmin_z), (bmax_x, bmax_y, bmax_z) = b
    dx = max(0.0, bmin_x - amax_x, amin_x - bmax_x)
    dy = max(0.0, bmin_y - amax_y, amin_y - bmax_y)
    dz = max(0.0, bmin_z - amax_z, amin_z - bmax_z)
    return float(math.sqrt(dx * dx + dy * dy + dz * dz))


def _obj_aabb(path: Path) -> AABB:
    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")
    found = False
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if not line.startswith("v "):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
        except ValueError:
            continue
        found = True
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        min_z = min(min_z, z)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
        max_z = max(max_z, z)

    if not found:
        raise ValidationError(f"OBJ contains no vertices: {path} (is this a valid .obj file?)")
    return (min_x, min_y, min_z), (max_x, max_y, max_z)


@dataclass(frozen=True)
class GeometryOverlap:
    pose_index: int
    pose: Dict[str, float]
    link_a: str
    link_b: str
    elem_a: int
    elem_b: int
    overlap_depth: Vec3
    overlap_volume: float
    aabb_a: AABB
    aabb_b: AABB
    elem_a_name: Optional[str] = None
    elem_b_name: Optional[str] = None
    elem_a_origin: Optional[Origin] = None
    elem_b_origin: Optional[Origin] = None
    elem_a_geometry: Optional[str] = None
    elem_b_geometry: Optional[str] = None


@dataclass(frozen=True)
class UnsupportedPartFinding:
    pose_index: int
    pose: Dict[str, float]
    part: str
    nearest_part: Optional[str]
    min_distance: Optional[float]
    contact_tol: float
    backend: str
    parts: Tuple[str, ...] = ()
    root_parts: Tuple[str, ...] = ()


@dataclass(frozen=True)
class JointOriginDistanceFinding:
    joint: str
    parent: str
    child: str
    parent_distance: float
    child_distance: float
    tol: float


@dataclass(frozen=True)
class PartGeometryConnectivityFinding:
    part: str
    connected: int
    total: int
    disconnected: Tuple[str, ...]
    contact_tol: float


@dataclass(frozen=True)
class _MeshComponentCollision:
    local_aabb: AABB
    shape: object
    index: int
    total: int


@dataclass(frozen=True)
class _CompiledCollisionEntry:
    item: object
    collision_obj: object
    aabb: AABB
    name: Optional[str]
    geometry_name: str
    origin: Origin


def _transform_aabb(aabb: AABB, tf: Mat4) -> AABB:
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    corners = [(x, y, z) for x in (min_x, max_x) for y in (min_y, max_y) for z in (min_z, max_z)]
    tx0 = ty0 = tz0 = float("inf")
    tx1 = ty1 = tz1 = float("-inf")
    for c in corners:
        x, y, z = _mat4_vec3(tf, c)
        tx0 = min(tx0, x)
        ty0 = min(ty0, y)
        tz0 = min(tz0, z)
        tx1 = max(tx1, x)
        ty1 = max(ty1, y)
        tz1 = max(tz1, z)
    return (tx0, ty0, tz0), (tx1, ty1, tz1)


def _geometry_local_aabb(
    geometry: object,
    *,
    asset_root: Optional[Path],
    _obj_cache: Dict[Path, AABB],
) -> AABB:

    if isinstance(geometry, Box):
        sx, sy, sz = geometry.size
        return (-sx / 2.0, -sy / 2.0, -sz / 2.0), (sx / 2.0, sy / 2.0, sz / 2.0)
    if isinstance(geometry, Cylinder):
        r = float(geometry.radius)
        length = float(geometry.length)
        return (-r, -r, -length / 2.0), (r, r, length / 2.0)
    if isinstance(geometry, Sphere):
        r = float(geometry.radius)
        return (-r, -r, -r), (r, r, r)
    if isinstance(geometry, Mesh):
        filename = geometry.filename
        if not isinstance(filename, (str, os.PathLike)):
            raise ValidationError("Mesh geometry filename is missing")
        filename = os.fspath(filename)
        if not filename:
            raise ValidationError("Mesh geometry filename is missing")

        mesh_path = resolve_mesh_path(filename, assets=asset_root)
        if not mesh_path.exists():
            raise ValidationError(f"Mesh file not found: {mesh_path}")
        if mesh_path.suffix.lower() != ".obj":
            raise ValidationError(
                f"Only OBJ meshes are supported for geometry QC (got: {mesh_path.name!r}). "
                "Convert the mesh to .obj or disable the overlap check."
            )

        aabb = _obj_cache.get(mesh_path)
        if aabb is None:
            aabb = _obj_aabb(mesh_path)
            _obj_cache[mesh_path] = aabb

        local_min, local_max = aabb
        if geometry.scale:
            sx, sy, sz = geometry.scale
            local_min = (local_min[0] * sx, local_min[1] * sy, local_min[2] * sz)
            local_max = (local_max[0] * sx, local_max[1] * sy, local_max[2] * sz)
        return local_min, local_max

    raise ValidationError(f"Unsupported geometry type: {type(geometry).__name__}")


def part_local_aabbs(
    part: Part,
    *,
    asset_root: Optional[Path] = None,
    prefer_collisions: bool = True,
    _obj_cache: Optional[Dict[Path, AABB]] = None,
) -> List[AABB]:
    obj_cache: Dict[Path, AABB] = _obj_cache if _obj_cache is not None else {}
    geoms: list[object] = []
    if prefer_collisions and part.collisions:
        geoms = list(part.collisions)
    else:
        geoms = list(part.visuals)

    aabbs: list[AABB] = []
    for item in geoms:
        origin = getattr(item, "origin", Origin())
        geometry = getattr(item, "geometry", None)
        if geometry is None:
            continue
        local_aabb = _geometry_local_aabb(geometry, asset_root=asset_root, _obj_cache=obj_cache)
        aabbs.append(_transform_aabb(local_aabb, _origin_to_mat4(origin)))
    return aabbs


def link_local_aabbs(
    link: Part,
    *,
    asset_root: Optional[Path] = None,
    prefer_collisions: bool = True,
    _obj_cache: Optional[Dict[Path, AABB]] = None,
) -> List[AABB]:
    return part_local_aabbs(
        link,
        asset_root=asset_root,
        prefer_collisions=prefer_collisions,
        _obj_cache=_obj_cache,
    )


def part_world_aabb(
    part: Part,
    link_world_tf: Mat4,
    *,
    asset_root: Optional[Path] = None,
    prefer_collisions: bool = True,
    _obj_cache: Optional[Dict[Path, AABB]] = None,
) -> Optional[AABB]:
    local = part_local_aabbs(
        part,
        asset_root=asset_root,
        prefer_collisions=prefer_collisions,
        _obj_cache=_obj_cache,
    )
    if not local:
        return None

    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]
    for aabb in local:
        waabb = _transform_aabb(aabb, link_world_tf)
        (mn, mx) = waabb
        for i in range(3):
            mins[i] = min(mins[i], mn[i])
            maxs[i] = max(maxs[i], mx[i])
    return (mins[0], mins[1], mins[2]), (maxs[0], maxs[1], maxs[2])


def link_world_aabb(
    link: Part,
    link_world_tf: Mat4,
    *,
    asset_root: Optional[Path] = None,
    prefer_collisions: bool = True,
    _obj_cache: Optional[Dict[Path, AABB]] = None,
) -> Optional[AABB]:
    return part_world_aabb(
        link,
        link_world_tf,
        asset_root=asset_root,
        prefer_collisions=prefer_collisions,
        _obj_cache=_obj_cache,
    )


def _mat4_rotation_translation(
    mat: Mat4,
) -> tuple[tuple[tuple[float, float, float], ...], tuple[float, float, float]]:
    rot = (
        (float(mat[0][0]), float(mat[0][1]), float(mat[0][2])),
        (float(mat[1][0]), float(mat[1][1]), float(mat[1][2])),
        (float(mat[2][0]), float(mat[2][1]), float(mat[2][2])),
    )
    trans = (float(mat[0][3]), float(mat[1][3]), float(mat[2][3]))
    return rot, trans


def _load_fcl_mesh(
    mesh_path: Path,
    *,
    cache: Dict[tuple[Path, Optional[Vec3]], object],
    scale: Optional[Vec3] = None,
) -> object:
    cache_key = (mesh_path, None if scale is None else tuple(float(v) for v in scale))
    cached = cache.get(cache_key)
    if cached is not None:
        return cached

    import fcl
    import trimesh

    loaded = trimesh.load_mesh(mesh_path, force="mesh")
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValidationError(f"Expected mesh geometry at {mesh_path}, got {type(loaded).__name__}")
    if loaded.vertices.size == 0:
        raise ValidationError(f"OBJ contains no vertices: {mesh_path}")
    if loaded.faces.size == 0:
        raise ValidationError(f"OBJ contains no triangles: {mesh_path}")
    if scale is not None:
        loaded = loaded.copy()
        loaded.apply_scale(scale)
    model = fcl.BVHModel()
    model.beginModel(len(loaded.vertices), len(loaded.faces))
    model.addSubModel(loaded.vertices, loaded.faces)
    model.endModel()
    cache[cache_key] = model
    return model


def _mesh_component_suffix(index: int) -> str:
    return f"__component_{index + 1:03d}"


def _mesh_component_name(name: object, *, index: int, total: int) -> Optional[str]:
    base_name = str(name).strip() if isinstance(name, str) and str(name).strip() else None
    if total <= 1:
        return base_name
    if base_name is None:
        return f"mesh{_mesh_component_suffix(index)}"
    return f"{base_name}{_mesh_component_suffix(index)}"


def _mesh_local_aabb(mesh: object) -> AABB:
    bounds = getattr(mesh, "bounds", None)
    if bounds is None:
        raise ValidationError("Mesh component is missing bounds")
    return (
        (float(bounds[0][0]), float(bounds[0][1]), float(bounds[0][2])),
        (float(bounds[1][0]), float(bounds[1][1]), float(bounds[1][2])),
    )


def _build_fcl_bvh_model(mesh: object) -> object:
    import fcl

    vertices = getattr(mesh, "vertices", None)
    faces = getattr(mesh, "faces", None)
    if vertices is None or getattr(vertices, "size", 0) == 0:
        raise ValidationError("Mesh component contains no vertices")
    if faces is None or getattr(faces, "size", 0) == 0:
        raise ValidationError("Mesh component contains no triangles")
    model = fcl.BVHModel()
    model.beginModel(len(vertices), len(faces))
    model.addSubModel(vertices, faces)
    model.endModel()
    return model


def _load_fcl_mesh_components(
    mesh_path: Path,
    *,
    cache: Dict[tuple[Path, Optional[Vec3]], object],
    scale: Optional[Vec3] = None,
) -> tuple[_MeshComponentCollision, ...]:
    cache_key = (
        "components",
        mesh_path,
        None if scale is None else tuple(float(v) for v in scale),
    )
    cached = cache.get(cache_key)
    if isinstance(cached, tuple):
        return cached

    import trimesh

    loaded = trimesh.load_mesh(mesh_path, force="mesh")
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValidationError(f"Expected mesh geometry at {mesh_path}, got {type(loaded).__name__}")
    if loaded.vertices.size == 0:
        raise ValidationError(f"OBJ contains no vertices: {mesh_path}")
    if loaded.faces.size == 0:
        raise ValidationError(f"OBJ contains no triangles: {mesh_path}")
    if scale is not None:
        loaded = loaded.copy()
        loaded.apply_scale(scale)

    raw_components = list(loaded.split(only_watertight=False))
    components = [
        component
        for component in raw_components
        if getattr(component, "vertices", None) is not None
        and getattr(component.vertices, "size", 0) > 0
        and getattr(component, "faces", None) is not None
        and getattr(component.faces, "size", 0) > 0
    ]
    if not components:
        components = [loaded]

    total = len(components)
    resolved = tuple(
        _MeshComponentCollision(
            local_aabb=_mesh_local_aabb(component),
            shape=_build_fcl_bvh_model(component),
            index=index,
            total=total,
        )
        for index, component in enumerate(components)
    )
    cache[cache_key] = resolved
    return resolved


def _collision_object_from_shape(shape: object, *, elem_tf: Mat4) -> object:
    import fcl

    rot, trans = _mat4_rotation_translation(elem_tf)
    return fcl.CollisionObject(shape, fcl.Transform(rot, trans))


def _collision_object_from_geometry(
    geometry: object,
    *,
    elem_tf: Mat4,
    asset_root: Optional[Path],
    mesh_cache: Dict[tuple[Path, Optional[Vec3]], object],
) -> object:
    import fcl

    if isinstance(geometry, Box):
        shape = fcl.Box(*[float(v) for v in geometry.size])
    elif isinstance(geometry, Cylinder):
        shape = fcl.Cylinder(float(geometry.radius), float(geometry.length))
    elif isinstance(geometry, Sphere):
        shape = fcl.Sphere(float(geometry.radius))
    elif isinstance(geometry, Mesh):
        filename = os.fspath(geometry.filename)
        mesh_path = resolve_mesh_path(filename, assets=asset_root)
        if mesh_path.suffix.lower() != ".obj":
            raise ValidationError(
                f"Only OBJ meshes are supported for geometry QC (got: {mesh_path.name!r})."
            )
        scale = None
        if geometry.scale is not None:
            scale = tuple(float(v) for v in geometry.scale)
        shape = _load_fcl_mesh(mesh_path, cache=mesh_cache, scale=scale)
    else:
        raise ValidationError(f"Unsupported geometry type: {type(geometry).__name__}")

    return _collision_object_from_shape(shape, elem_tf=elem_tf)


def _point_collision_object(point: Vec3, *, radius: float = 1e-9) -> object:
    import fcl

    rot = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    return fcl.CollisionObject(
        fcl.Sphere(float(radius)),
        fcl.Transform(rot, (float(point[0]), float(point[1]), float(point[2]))),
    )


def _collision_pair_metrics(obj_a: object, obj_b: object) -> tuple[bool, float]:
    import fcl

    collision_request = fcl.CollisionRequest()
    collision_result = fcl.CollisionResult()
    if int(fcl.collide(obj_a, obj_b, collision_request, collision_result)) > 0:
        return True, 0.0

    distance_request = fcl.DistanceRequest()
    distance_result = fcl.DistanceResult()
    return False, float(fcl.distance(obj_a, obj_b, distance_request, distance_result))


def _compiled_part_collision_entries(
    part: Part,
    *,
    asset_root: Optional[Path],
    mesh_cache: Dict[tuple[Path, Optional[Vec3]], object],
    part_tf: Optional[Mat4] = None,
) -> list[_CompiledCollisionEntry]:
    resolved_part_tf = _identity4() if part_tf is None else part_tf
    obj_cache: Dict[Path, AABB] = {}
    entries: list[_CompiledCollisionEntry] = []
    for item in list(getattr(part, "collisions", []) or []):
        geometry = getattr(item, "geometry", None)
        if geometry is None:
            continue
        origin = getattr(item, "origin", Origin())
        elem_tf = _mat4_mul(resolved_part_tf, _origin_to_mat4(origin))
        geometry_name = type(geometry).__name__
        if isinstance(geometry, Mesh):
            filename = os.fspath(geometry.filename)
            mesh_path = resolve_mesh_path(filename, assets=asset_root)
            scale = None
            if geometry.scale is not None:
                scale = tuple(float(v) for v in geometry.scale)
            for component in _load_fcl_mesh_components(
                mesh_path,
                cache=mesh_cache,
                scale=scale,
            ):
                entries.append(
                    _CompiledCollisionEntry(
                        item=item,
                        collision_obj=_collision_object_from_shape(
                            component.shape, elem_tf=elem_tf
                        ),
                        aabb=_transform_aabb(component.local_aabb, elem_tf),
                        name=_mesh_component_name(
                            getattr(item, "name", None),
                            index=component.index,
                            total=component.total,
                        ),
                        geometry_name=geometry_name,
                        origin=origin,
                    )
                )
            continue

        local_aabb = _geometry_local_aabb(
            geometry,
            asset_root=asset_root,
            _obj_cache=obj_cache,
        )
        entries.append(
            _CompiledCollisionEntry(
                item=item,
                collision_obj=_collision_object_from_geometry(
                    geometry,
                    elem_tf=elem_tf,
                    asset_root=asset_root,
                    mesh_cache=mesh_cache,
                ),
                aabb=_transform_aabb(local_aabb, elem_tf),
                name=getattr(item, "name", None),
                geometry_name=geometry_name,
                origin=origin,
            )
        )
    return entries


def compute_part_world_transforms(
    model: object,
    joint_positions: Dict[str, object],
) -> Dict[str, Mat4]:
    links = getattr(model, "parts", None)
    if not isinstance(links, list):
        links = getattr(model, "links", None)
    joints = getattr(model, "articulations", None)
    if not isinstance(joints, list):
        joints = getattr(model, "joints", None)
    if not isinstance(links, list) or not isinstance(joints, list):
        raise ValidationError("model must have .parts and .articulations lists")

    link_names = [getattr(link, "name", None) for link in links]
    if not all(isinstance(n, str) for n in link_names):
        raise ValidationError("model.parts must have string names")
    if len(set(link_names)) != len(link_names):
        raise ValidationError("model.parts must have unique names")
    link_name_set = set(link_names)  # type: ignore[arg-type]

    child_set: set[str] = set()
    parent_to_children: Dict[str, List[object]] = {name: [] for name in link_name_set}  # type: ignore[arg-type]
    for j in joints:
        parent = getattr(j, "parent", None)
        child = getattr(j, "child", None)
        if not isinstance(parent, str) or not isinstance(child, str):
            raise ValidationError("articulations must have string parent/child names")
        if parent not in link_name_set:
            raise ValidationError(f"Articulation references missing parent part {parent!r}")
        if child not in link_name_set:
            raise ValidationError(f"Articulation references missing child part {child!r}")
        if child in child_set:
            raise ValidationError(f"Part {child!r} has multiple parent articulations")
        parent_to_children.setdefault(parent, []).append(j)
        child_set.add(child)

    roots = sorted(link_name_set - child_set)
    if not roots:
        raise ValidationError(
            "Articulated object has no root part (cycle or every part is a child)"
        )
    if len(roots) > 1:
        raise ValidationError(
            "Articulated object must have exactly one root part for world-frame "
            f"computations; found multiple root parts: {roots}"
        )

    root = roots[0]
    world: Dict[str, Mat4] = {root: _identity4()}
    resolved_joint_positions = _resolve_joint_positions(model, joint_positions)
    visited: set[str] = set()
    queue: List[str] = [root]

    while queue:
        parent_link = queue.pop(0)
        if parent_link in visited:
            continue
        visited.add(parent_link)
        parent_tf = world[parent_link]
        for joint in parent_to_children.get(parent_link, []):
            child = getattr(joint, "child", None)
            if not isinstance(child, str):
                continue
            joint_origin = getattr(joint, "origin", Origin())
            joint_tf = _origin_to_mat4(joint_origin)

            motion_tf = _identity4()
            joint_type = getattr(joint, "articulation_type", None)
            axis = getattr(joint, "axis", (0.0, 0.0, 1.0))
            pose_value = resolved_joint_positions.get(
                getattr(joint, "name", ""), _joint_zero_pose_value(joint)
            )

            if joint_type in (ArticulationType.REVOLUTE, ArticulationType.CONTINUOUS):
                q = float(pose_value)
                rot = _axis_angle_matrix(_normalize(axis), q)
                motion_tf = _mat3_to_mat4(rot)
            elif joint_type == ArticulationType.PRISMATIC:
                q = float(pose_value)
                ax = _normalize(axis)
                motion_tf = _mat3_to_mat4(
                    ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                    translation=(ax[0] * q, ax[1] * q, ax[2] * q),
                )
            elif joint_type == ArticulationType.FLOATING:
                motion_tf = _origin_to_mat4(pose_value)
            elif joint_type == ArticulationType.FIXED:
                motion_tf = _identity4()
            else:
                motion_tf = _identity4()

            child_tf = _mat4_mul(_mat4_mul(parent_tf, joint_tf), motion_tf)
            if child in world:
                raise ValidationError(f"Part {child!r} was reached multiple times")
            world[child] = child_tf
            queue.append(child)

    if visited != link_name_set:
        missing = sorted(link_name_set - visited)
        raise ValidationError(
            "Articulated object contains unreachable parts (cycle or broken articulations): "
            f"{missing}"
        )
    return world


def _coerce_origin_pose_value(value: object, *, joint_name: str) -> Origin:
    if isinstance(value, Origin):
        return value
    xyz = getattr(value, "xyz", None)
    rpy = getattr(value, "rpy", None)
    if xyz is not None and rpy is not None:
        try:
            return Origin(xyz=xyz, rpy=rpy)
        except Exception as exc:  # pragma: no cover - defensive conversion guard
            raise ValidationError(
                f"Articulation {joint_name!r} expects an Origin pose value"
            ) from exc
    raise ValidationError(f"Articulation {joint_name!r} expects an Origin pose value")


def _joint_zero_pose_value(joint: object) -> object:
    if getattr(joint, "articulation_type", None) == ArticulationType.FLOATING:
        return Origin()
    return 0.0


def _resolve_joint_positions(
    model: object, joint_positions: Dict[str, object]
) -> Dict[str, object]:
    joints = getattr(model, "articulations", None)
    if not isinstance(joints, list):
        joints = getattr(model, "joints", None)
    if not isinstance(joints, list):
        raise ValidationError("model must have an .articulations list")

    joint_lookup = {
        name: joint for joint in joints if isinstance((name := getattr(joint, "name", None)), str)
    }
    resolved: Dict[str, object] = {}
    resolving: list[str] = []

    def resolve_joint(name: str) -> object:
        if name in resolved:
            return resolved[name]
        joint = joint_lookup.get(name)
        if joint is None:
            raise ValidationError(f"Unknown articulation: {name!r}")
        if name in resolving:
            cycle_start = resolving.index(name)
            cycle = resolving[cycle_start:] + [name]
            raise ValidationError(f"Mimic cycle detected: {' -> '.join(cycle)}")

        resolving.append(name)
        mimic = getattr(joint, "mimic", None)
        if mimic is not None:
            if name in joint_positions:
                raise ValidationError(
                    f"Articulation {name!r} is mimic-driven and cannot be posed directly"
                )
            source_name = str(getattr(mimic, "joint", "")).strip()
            source_value = resolve_joint(source_name)
            if isinstance(source_value, Origin):
                raise ValidationError(
                    f"Articulation {name!r} mimic source {source_name!r} must use scalar poses"
                )
            value = float(source_value) * float(getattr(mimic, "multiplier", 1.0)) + float(
                getattr(mimic, "offset", 0.0)
            )
        else:
            value = _coerce_joint_pose_value(
                joint,
                joint_positions.get(name, _joint_zero_pose_value(joint)),
            )
        resolving.pop()
        resolved[name] = value
        return value

    for name in joint_lookup:
        resolve_joint(name)
    return resolved


def _coerce_joint_pose_value(joint: object, value: object) -> object:
    joint_name = str(getattr(joint, "name", ""))
    joint_type = getattr(joint, "articulation_type", None)
    if joint_type == ArticulationType.FLOATING:
        return _coerce_origin_pose_value(value, joint_name=joint_name)
    if isinstance(value, Origin) or (hasattr(value, "xyz") and hasattr(value, "rpy")):
        raise ValidationError(
            f"Articulation {joint_name!r} expects a scalar pose value, not Origin(...)"
        )
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"Articulation {joint_name!r} expects a scalar pose value") from exc


def _pose_value_key(value: object) -> object:
    if isinstance(value, Origin):
        return ("origin", value.xyz, value.rpy)
    return float(value)


def _pose_value_is_zero(value: object) -> bool:
    if isinstance(value, Origin):
        return all(abs(float(v)) <= 1e-12 for v in (*value.xyz, *value.rpy))
    return abs(float(value)) <= 1e-12


def compute_link_world_transforms(
    model: object,
    joint_positions: Dict[str, object],
) -> Dict[str, Mat4]:
    return compute_part_world_transforms(model, joint_positions)


def _joint_sample_values(joint: object) -> List[object]:
    joint_type = getattr(joint, "articulation_type", None)
    motion_limits = getattr(joint, "motion_limits", None)
    if joint_type == ArticulationType.FIXED:
        return [0.0]

    meta = getattr(joint, "meta", None)
    raw = meta.get("qc_samples", meta.get("qc_sample_values")) if isinstance(meta, dict) else None

    if joint_type == ArticulationType.FLOATING:
        values: list[Origin] = [Origin()]
        if isinstance(raw, (list, tuple)) and raw:
            for v in raw[:32]:
                try:
                    pose_value = _coerce_joint_pose_value(joint, v)
                except ValidationError:
                    continue
                if isinstance(pose_value, Origin) and pose_value not in values:
                    values.append(pose_value)
        return list(values)

    if isinstance(raw, (list, tuple)) and raw:
        values: list[float] = [0.0]
        for v in raw[:32]:
            if isinstance(v, (int, float)):
                values.append(float(v))
        out: list[float] = []
        for v in values:
            if not any(abs(v - u) <= 1e-9 for u in out):
                out.append(v)
        return list(out)

    if joint_type == ArticulationType.CONTINUOUS:
        return [0.0, -math.pi / 2.0, math.pi / 2.0, math.pi]

    lower = getattr(motion_limits, "lower", None) if motion_limits is not None else None
    upper = getattr(motion_limits, "upper", None) if motion_limits is not None else None
    values: list[float] = [0.0]
    if isinstance(lower, (int, float)):
        values.append(float(lower))
    if isinstance(upper, (int, float)):
        values.append(float(upper))
    if isinstance(lower, (int, float)) and isinstance(upper, (int, float)):
        values.append(0.5 * (float(lower) + float(upper)))
    out: list[float] = []
    for v in values:
        if not any(abs(v - u) <= 1e-9 for u in out):
            out.append(v)
    return list(out)


def generate_pose_samples(
    model: object,
    *,
    max_samples: int = 256,
    seed: int = 0,
) -> List[Dict[str, object]]:
    max_samples = int(max_samples)
    if max_samples <= 0:
        raise ValidationError("max_samples must be positive")

    joints = getattr(model, "articulations", None)
    if not isinstance(joints, list):
        joints = getattr(model, "joints", None)
    if not isinstance(joints, list):
        raise ValidationError("model must have an .articulations list")

    joint_names: list[str] = []
    per_joint: list[list[object]] = []
    for j in joints:
        name = getattr(j, "name", None)
        if not isinstance(name, str):
            continue
        if getattr(j, "mimic", None) is not None:
            continue
        joint_names.append(name)
        per_joint.append(_joint_sample_values(j))

    if not joint_names:
        return [{}]

    total = 1
    for vals in per_joint:
        total *= max(1, len(vals))

    poses: list[dict[str, object]] = []
    seen: set[tuple[object, ...]] = set()

    def add_pose(values: Sequence[object]) -> None:
        if len(poses) >= max_samples:
            return
        key = tuple(_pose_value_key(values[i]) for i in range(len(joint_names)))
        if key in seen:
            return
        seen.add(key)
        poses.append({joint_names[i]: values[i] for i in range(len(joint_names))})

    if total <= max_samples:
        for combo in itertools.product(*per_joint):
            add_pose(combo)
        return poses

    zero = [vals[0] for vals in per_joint]
    add_pose(zero)
    for i, vals in enumerate(per_joint):
        if len(poses) >= max_samples:
            break
        for v in vals:
            if _pose_value_is_zero(v):
                continue
            one = list(zero)
            one[i] = v
            add_pose(one)

    rng = random.Random(int(seed))
    while len(poses) < max_samples and len(seen) < total:
        sample = [rng.choice(vals) for vals in per_joint]
        add_pose(sample)
    return poses


def _find_collision_overlaps_fcl_for_poses(
    compiled_model: object,
    *,
    asset_root: Optional[Path],
    source_model: Optional[object] = None,
    poses: Sequence[Dict[str, float]],
    overlap_tol: float,
    overlap_volume_tol: float,
    allowed_pairs: Optional[Iterable[Tuple[str, str]]],
) -> List[GeometryOverlap]:
    import fcl

    resolved_asset_root = resolve_asset_root(
        asset_root,
        compiled_model,
        source_model if source_model is not None else compiled_model,
    )
    links = getattr(compiled_model, "parts", None)
    joints = getattr(compiled_model, "articulations", None)
    if not isinstance(links, list) or not isinstance(joints, list):
        raise ValidationError("model must have .parts and .articulations lists")

    allowed: set[tuple[str, str]] = set()
    if allowed_pairs:
        for a, b in allowed_pairs:
            allowed.add((a, b))
            allowed.add((b, a))

    overlaps: list[GeometryOverlap] = []
    mesh_cache: Dict[tuple[Path, Optional[Vec3]], object] = {}
    collision_request = fcl.CollisionRequest()

    for pose_index, pose in enumerate(poses):
        tf = compute_part_world_transforms(compiled_model, pose)
        link_elem_bounds: Dict[str, List[tuple[AABB, object, object]]] = {}
        for link in links:
            name = getattr(link, "name", None)
            if not isinstance(name, str):
                continue
            world_tf = tf.get(name, _identity4())
            bounds: list[tuple[AABB, object, object]] = []
            for entry in _compiled_part_collision_entries(
                link,
                asset_root=resolved_asset_root,
                mesh_cache=mesh_cache,
                part_tf=world_tf,
            ):
                bounds.append((entry.aabb, entry.collision_obj, entry))
            if bounds:
                link_elem_bounds[name] = bounds

        names = sorted(link_elem_bounds.keys())
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a = names[i]
                b = names[j]
                if (a, b) in allowed:
                    continue
                for ia, (aabb_a, obj_a, item_a) in enumerate(link_elem_bounds[a]):
                    for ib, (aabb_b, obj_b, item_b) in enumerate(link_elem_bounds[b]):
                        depth = _aabb_intersection_depth(aabb_a, aabb_b)
                        volume = _aabb_intersection_volume(aabb_a, aabb_b)
                        if not (
                            depth[0] > overlap_tol
                            and depth[1] > overlap_tol
                            and depth[2] > overlap_tol
                        ):
                            continue
                        if volume <= overlap_volume_tol:
                            continue
                        result = fcl.CollisionResult()
                        collided = fcl.collide(obj_a, obj_b, collision_request, result)
                        if int(collided) <= 0:
                            continue
                        overlaps.append(
                            GeometryOverlap(
                                pose_index=pose_index,
                                pose=dict(pose),
                                link_a=a,
                                link_b=b,
                                elem_a=ia,
                                elem_b=ib,
                                overlap_depth=depth,
                                overlap_volume=volume,
                                aabb_a=aabb_a,
                                aabb_b=aabb_b,
                                elem_a_name=item_a.name,
                                elem_b_name=item_b.name,
                                elem_a_origin=item_a.origin,
                                elem_b_origin=item_b.origin,
                                elem_a_geometry=item_a.geometry_name,
                                elem_b_geometry=item_b.geometry_name,
                            )
                        )
    return overlaps


def _find_collision_overlaps_fcl(
    model: object,
    *,
    asset_root: Optional[Path],
    max_pose_samples: int,
    overlap_tol: float,
    overlap_volume_tol: float,
    allowed_pairs: Optional[Iterable[Tuple[str, str]]],
    seed: int,
    validate_model: bool = True,
) -> List[GeometryOverlap]:
    compiled_model = compile_object_model_with_exact_collisions(
        model,  # type: ignore[arg-type]
        asset_root=asset_root,
        validate=validate_model,
    )
    poses = generate_pose_samples(compiled_model, max_samples=max_pose_samples, seed=int(seed))
    return _find_collision_overlaps_fcl_for_poses(
        compiled_model,
        asset_root=asset_root,
        source_model=model,
        poses=poses,
        overlap_tol=overlap_tol,
        overlap_volume_tol=overlap_volume_tol,
        allowed_pairs=allowed_pairs,
    )


def _resolve_model_links_and_joints(model: object) -> tuple[list[object], list[object]]:
    links = getattr(model, "parts", None)
    if not isinstance(links, list):
        links = getattr(model, "links", None)
    joints = getattr(model, "articulations", None)
    if not isinstance(joints, list):
        joints = getattr(model, "joints", None)
    if not isinstance(links, list) or not isinstance(joints, list):
        raise ValidationError("model must have .parts and .articulations lists")
    return links, joints


def _pair_key(a: str, b: str) -> tuple[str, str]:
    return (a, b) if a <= b else (b, a)


def _root_part_names(names: Sequence[str], joints: Sequence[object]) -> Tuple[str, ...]:
    name_set = {str(name) for name in names}
    child_names = {
        str(child)
        for joint in joints
        if isinstance((child := getattr(joint, "child", None)), str) and child in name_set
    }
    roots = tuple(sorted(name for name in name_set if name not in child_names))
    if roots:
        return roots
    if not name_set:
        return ()
    return (sorted(name_set)[0],)


def _connected_components(
    names: Sequence[str],
    adjacency: Dict[str, set[str]],
) -> List[Tuple[str, ...]]:
    remaining = set(names)
    components: list[tuple[str, ...]] = []
    while remaining:
        start = min(remaining)
        stack = [start]
        component: list[str] = []
        remaining.remove(start)
        while stack:
            current = stack.pop()
            component.append(current)
            for neighbor in sorted(adjacency.get(current, set())):
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                stack.append(neighbor)
        components.append(tuple(sorted(component)))
    return components


def _unsupported_component_findings(
    names: Sequence[str],
    *,
    root_parts: Sequence[str],
    support_pairs: Iterable[Tuple[str, str]],
    pair_min_distances: Dict[Tuple[str, str], float],
    pose_index: int,
    pose: Dict[str, float],
    contact_tol: float,
    backend: str,
) -> List[UnsupportedPartFinding]:
    unique_names = tuple(sorted({str(name) for name in names}))
    if len(unique_names) <= 1:
        return []

    adjacency: Dict[str, set[str]] = {name: set() for name in unique_names}
    for raw_a, raw_b in support_pairs:
        a, b = _pair_key(str(raw_a), str(raw_b))
        if a == b or a not in adjacency or b not in adjacency:
            continue
        adjacency[a].add(b)
        adjacency[b].add(a)

    components = _connected_components(unique_names, adjacency)
    root_set = {str(name) for name in root_parts}
    grounded_parts: set[str] = set()
    for component in components:
        if any(name in root_set for name in component):
            grounded_parts.update(component)
    if not grounded_parts and unique_names:
        grounded_parts.add(unique_names[0])

    findings: list[UnsupportedPartFinding] = []
    for component in components:
        if any(name in grounded_parts for name in component):
            continue

        nearest_part: Optional[str] = None
        min_distance: Optional[float] = None
        for name in component:
            for grounded_name in grounded_parts:
                if name == grounded_name:
                    continue
                distance = pair_min_distances.get(_pair_key(name, grounded_name))
                if distance is None:
                    continue
                if (
                    min_distance is None
                    or distance < float(min_distance)
                    or (
                        math.isclose(distance, float(min_distance), abs_tol=_EPS)
                        and isinstance(nearest_part, str)
                        and grounded_name < nearest_part
                    )
                    or (nearest_part is None and math.isfinite(distance))
                ):
                    min_distance = float(distance)
                    nearest_part = grounded_name

        findings.append(
            UnsupportedPartFinding(
                pose_index=pose_index,
                pose=dict(pose),
                part=component[0],
                nearest_part=nearest_part,
                min_distance=min_distance,
                contact_tol=float(contact_tol),
                backend=backend,
                parts=component,
                root_parts=tuple(sorted(root_set)),
            )
        )

    return findings


def _unsupported_parts_from_world_aabbs(
    part_aabbs: Dict[str, AABB],
    *,
    pose_index: int,
    pose: Dict[str, float],
    contact_tol: float,
    backend: str,
    root_parts: Sequence[str] = (),
) -> List[UnsupportedPartFinding]:
    names = sorted(part_aabbs.keys())
    if len(names) <= 1:
        return []

    support_pairs: set[tuple[str, str]] = set()
    pair_min_distances: dict[tuple[str, str], float] = {}

    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a = names[i]
            b = names[j]
            aabb_a = part_aabbs[a]
            aabb_b = part_aabbs[b]
            if _aabbs_touch_or_overlap(aabb_a, aabb_b, tol=contact_tol):
                support_pairs.add((a, b))
                dist = 0.0
            else:
                dist = _aabb_separation_distance(aabb_a, aabb_b)
            pair_min_distances[_pair_key(a, b)] = float(dist)

    return _unsupported_component_findings(
        names,
        root_parts=tuple(root_parts) if root_parts else _root_part_names(names, ()),
        support_pairs=support_pairs,
        pair_min_distances=pair_min_distances,
        pose_index=pose_index,
        pose=pose,
        contact_tol=float(contact_tol),
        backend=backend,
    )


def _find_unsupported_parts_physical(
    model: object,
    *,
    asset_root: Optional[Path],
    max_pose_samples: int,
    contact_tol: float,
    seed: int,
    validate_model: bool = True,
) -> List[UnsupportedPartFinding]:
    compiled_model = compile_object_model_with_exact_collisions(
        model,  # type: ignore[arg-type]
        asset_root=asset_root,
        validate=validate_model,
    )
    resolved_asset_root = resolve_asset_root(asset_root, compiled_model, model)
    links, joints = _resolve_model_links_and_joints(compiled_model)
    if len(links) <= 1:
        return []

    try:
        import fcl  # type: ignore[import-not-found]
    except Exception as exc:
        raise ValidationError(
            "find_unsupported_parts(...) requires FCL support for contact queries."
        ) from exc

    poses = generate_pose_samples(compiled_model, max_samples=max_pose_samples, seed=int(seed))
    mesh_cache: Dict[tuple[Path, Optional[Vec3]], object] = {}
    findings: list[UnsupportedPartFinding] = []

    collision_request = fcl.CollisionRequest()
    if not all(hasattr(fcl, attr) for attr in ("distance", "DistanceRequest", "DistanceResult")):
        raise ValidationError(
            "find_unsupported_parts(...) requires FCL distance queries "
            "(`distance`, `DistanceRequest`, `DistanceResult`)."
        )

    for pose_index, pose in enumerate(poses):
        tf = compute_part_world_transforms(compiled_model, pose)
        link_elem_bounds: Dict[str, List[tuple[AABB, object]]] = {}
        for link in links:
            name = getattr(link, "name", None)
            if not isinstance(name, str):
                continue
            world_tf = tf.get(name, _identity4())
            bounds: list[tuple[AABB, object]] = []
            for entry in _compiled_part_collision_entries(
                link,
                asset_root=resolved_asset_root,
                mesh_cache=mesh_cache,
                part_tf=world_tf,
            ):
                bounds.append((entry.aabb, entry.collision_obj))
            if bounds:
                link_elem_bounds[name] = bounds

        names = sorted(link_elem_bounds.keys())
        if len(names) <= 1:
            continue

        root_parts = _root_part_names(names, joints)
        support_pairs: set[tuple[str, str]] = set()
        pair_min_distances: dict[tuple[str, str], float] = {}

        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a = names[i]
                b = names[j]
                pair_supported = False
                pair_min_distance: Optional[float] = None
                for aabb_a, obj_a in link_elem_bounds[a]:
                    for aabb_b, obj_b in link_elem_bounds[b]:
                        result = fcl.CollisionResult()
                        collided = fcl.collide(obj_a, obj_b, collision_request, result)
                        if int(collided) > 0:
                            pair_supported = True
                            pair_min_distance = 0.0
                            break

                        dist_request = fcl.DistanceRequest()
                        dist_result = fcl.DistanceResult()
                        dist = float(fcl.distance(obj_a, obj_b, dist_request, dist_result))
                        if pair_min_distance is None or dist < pair_min_distance:
                            pair_min_distance = dist
                        if dist <= contact_tol:
                            pair_supported = True
                            pair_min_distance = dist
                            break
                    if pair_supported:
                        break

                if pair_supported:
                    support_pairs.add((a, b))

                if pair_min_distance is not None:
                    pair_min_distances[_pair_key(a, b)] = float(pair_min_distance)

        findings.extend(
            _unsupported_component_findings(
                names,
                root_parts=root_parts,
                support_pairs=support_pairs,
                pair_min_distances=pair_min_distances,
                pose_index=pose_index,
                pose=pose,
                contact_tol=float(contact_tol),
                backend="fcl",
            )
        )

    return findings


def find_joint_origin_distance_findings(
    model: object,
    *,
    asset_root: Optional[Path] = None,
    tol: float = 0.015,
    validate_model: bool = True,
) -> List[JointOriginDistanceFinding]:
    compiled_model = compile_object_model_with_exact_collisions(
        model,  # type: ignore[arg-type]
        asset_root=asset_root,
        validate=validate_model,
    )
    resolved_asset_root = resolve_asset_root(asset_root, compiled_model, model)
    links, joints = _resolve_model_links_and_joints(compiled_model)
    if not links or not joints:
        return []

    links_by_name = {
        name: link for link in links if isinstance((name := getattr(link, "name", None)), str)
    }
    mesh_cache: Dict[tuple[Path, Optional[Vec3]], object] = {}
    findings: list[JointOriginDistanceFinding] = []
    point_radius = 1e-9

    for joint in joints:
        joint_name = getattr(joint, "name", None)
        parent_name = getattr(joint, "parent", None)
        child_name = getattr(joint, "child", None)
        if (
            not isinstance(joint_name, str)
            or not isinstance(parent_name, str)
            or not isinstance(child_name, str)
        ):
            continue
        parent_part = links_by_name.get(parent_name)
        child_part = links_by_name.get(child_name)
        if parent_part is None or child_part is None:
            continue

        origin = getattr(joint, "origin", Origin())
        parent_point = (
            float(origin.xyz[0]),
            float(origin.xyz[1]),
            float(origin.xyz[2]),
        )
        child_point = (0.0, 0.0, 0.0)
        parent_probe = _point_collision_object(parent_point, radius=point_radius)
        child_probe = _point_collision_object(child_point, radius=point_radius)

        parent_entries = _compiled_part_collision_entries(
            parent_part,
            asset_root=resolved_asset_root,
            mesh_cache=mesh_cache,
        )
        child_entries = _compiled_part_collision_entries(
            child_part,
            asset_root=resolved_asset_root,
            mesh_cache=mesh_cache,
        )
        if not parent_entries or not child_entries:
            continue

        parent_distance = min(
            _collision_pair_metrics(parent_probe, entry.collision_obj)[1]
            for entry in parent_entries
        )
        child_distance = min(
            _collision_pair_metrics(child_probe, entry.collision_obj)[1] for entry in child_entries
        )
        if parent_distance > float(tol) or child_distance > float(tol):
            findings.append(
                JointOriginDistanceFinding(
                    joint=joint_name,
                    parent=parent_name,
                    child=child_name,
                    parent_distance=float(parent_distance),
                    child_distance=float(child_distance),
                    tol=float(tol),
                )
            )
    return findings


def find_part_geometry_connectivity_findings(
    model: object,
    *,
    asset_root: Optional[Path] = None,
    contact_tol: float = 1e-6,
    validate_model: bool = True,
) -> List[PartGeometryConnectivityFinding]:
    compiled_model = compile_object_model_with_exact_collisions(
        model,  # type: ignore[arg-type]
        asset_root=asset_root,
        validate=validate_model,
    )
    resolved_asset_root = resolve_asset_root(asset_root, compiled_model, model)
    links, _joints = _resolve_model_links_and_joints(compiled_model)
    if not links:
        return []

    mesh_cache: Dict[tuple[Path, Optional[Vec3]], object] = {}
    findings: list[PartGeometryConnectivityFinding] = []

    for link in links:
        part_name = getattr(link, "name", None)
        if not isinstance(part_name, str):
            continue
        entries = _compiled_part_collision_entries(
            link,
            asset_root=resolved_asset_root,
            mesh_cache=mesh_cache,
        )
        if len(entries) <= 1:
            continue

        remaining = set(range(len(entries)))
        connected_groups: list[set[int]] = []
        while remaining:
            current_group = {next(iter(remaining))}
            queue = list(current_group)
            while queue:
                current = queue.pop()
                current_obj = entries[current].collision_obj
                for idx, candidate in enumerate(entries):
                    if idx in current_group:
                        continue
                    collided, distance = _collision_pair_metrics(
                        current_obj, candidate.collision_obj
                    )
                    if collided or distance <= float(contact_tol):
                        current_group.add(idx)
                        queue.append(idx)
            connected_groups.append(current_group)
            remaining.difference_update(current_group)

        largest_group = max(connected_groups, key=lambda group: (len(group), -min(group)))
        if len(largest_group) == len(entries):
            continue

        disconnected: list[str] = []
        for idx, entry in enumerate(entries):
            if idx in largest_group:
                continue
            item_name = entry.name
            geometry_name = entry.geometry_name
            if isinstance(item_name, str) and item_name:
                disconnected.append(f"{item_name}:{geometry_name}")
            else:
                disconnected.append(f"#{idx}:{geometry_name}")
        findings.append(
            PartGeometryConnectivityFinding(
                part=part_name,
                connected=len(largest_group),
                total=len(entries),
                disconnected=tuple(disconnected),
                contact_tol=float(contact_tol),
            )
        )
    return findings


def find_unsupported_parts(
    model: object,
    *,
    asset_root: Optional[Path] = None,
    max_pose_samples: int = 1,
    contact_tol: float = 1e-6,
    seed: int = 0,
    validate_model: bool = True,
) -> List[UnsupportedPartFinding]:
    """
    Find support-disconnected floating components across sampled poses.

    Parts are grouped by exact physical contact / near-contact into connected
    components. Any component that has no path to a rooted body part is
    reported, so a floating multi-part island such as `lid + top_vent` is
    flagged even when those parts still touch each other.
    """
    return _find_unsupported_parts_physical(
        model,
        asset_root=asset_root,
        max_pose_samples=max_pose_samples,
        contact_tol=float(contact_tol),
        seed=int(seed),
        validate_model=validate_model,
    )


def find_geometry_overlaps(
    model: object,
    *,
    asset_root: Optional[Path] = None,
    max_pose_samples: int = 256,
    overlap_tol: float = 1e-3,
    overlap_volume_tol: float = 0.0,
    allowed_pairs: Optional[Iterable[Tuple[str, str]]] = None,
    seed: int = 0,
    validate_model: bool = True,
) -> List[GeometryOverlap]:
    return _find_collision_overlaps_fcl(
        model,
        asset_root=asset_root,
        max_pose_samples=max_pose_samples,
        overlap_tol=overlap_tol,
        overlap_volume_tol=overlap_volume_tol,
        allowed_pairs=allowed_pairs,
        seed=seed,
        validate_model=validate_model,
    )


def find_geometry_overlaps_in_poses(
    model: object,
    *,
    poses: Sequence[Dict[str, float]],
    asset_root: Optional[Path] = None,
    overlap_tol: float = 1e-3,
    overlap_volume_tol: float = 0.0,
    allowed_pairs: Optional[Iterable[Tuple[str, str]]] = None,
    validate_model: bool = True,
) -> List[GeometryOverlap]:
    if not poses:
        return []
    compiled_model = compile_object_model_with_exact_collisions(
        model,  # type: ignore[arg-type]
        asset_root=asset_root,
        validate=validate_model,
    )
    return _find_collision_overlaps_fcl_for_poses(
        compiled_model,
        asset_root=asset_root,
        source_model=model,
        poses=poses,
        overlap_tol=overlap_tol,
        overlap_volume_tol=overlap_volume_tol,
        allowed_pairs=allowed_pairs,
    )


def validate_no_geometry_overlaps(
    model: object,
    *,
    asset_root: Optional[Path] = None,
    max_pose_samples: int = 256,
    overlap_tol: float = 1e-3,
    overlap_volume_tol: float = 0.0,
    allowed_pairs: Optional[Iterable[Tuple[str, str]]] = None,
    seed: int = 0,
    validate_model: bool = True,
) -> None:
    overlaps = find_geometry_overlaps(
        model,
        asset_root=asset_root,
        max_pose_samples=max_pose_samples,
        overlap_tol=overlap_tol,
        overlap_volume_tol=overlap_volume_tol,
        allowed_pairs=allowed_pairs,
        seed=int(seed),
        validate_model=validate_model,
    )
    if not overlaps:
        return

    worst = max(
        overlaps,
        key=lambda o: min(o.overlap_depth[0], o.overlap_depth[1], o.overlap_depth[2]),
    )
    min_depth = min(worst.overlap_depth[0], worst.overlap_depth[1], worst.overlap_depth[2])
    pose_preview = ", ".join(f"{k}={v:.4g}" for k, v in sorted(worst.pose.items()))

    # Heuristic "what to move" hint: separate world-axis AABBs along the least-overlapping axis.
    a_ctr = _aabb_center(worst.aabb_a)
    b_ctr = _aabb_center(worst.aabb_b)
    dx, dy, dz = worst.overlap_depth
    axis = "x"
    overlap_amt = float(dx)
    if float(dy) < overlap_amt:
        axis, overlap_amt = "y", float(dy)
    if float(dz) < overlap_amt:
        axis, overlap_amt = "z", float(dz)

    sign = 1.0
    if axis == "x" and b_ctr[0] < a_ctr[0]:
        sign = -1.0
    if axis == "y" and b_ctr[1] < a_ctr[1]:
        sign = -1.0
    if axis == "z" and b_ctr[2] < a_ctr[2]:
        sign = -1.0
    suggested_move = sign * (overlap_amt + overlap_tol)
    raise ValidationError(
        "Geometry overlap check failed: part AABBs intersect at sampled articulation poses. "
        f"worst_pair=({worst.link_a!r}[{worst.elem_a}], {worst.link_b!r}[{worst.elem_b}]) "
        f"min_overlap_depth={min_depth:.4g}m overlap_volume={worst.overlap_volume:.4g}m^3 "
        f"pose_index={worst.pose_index} pose=({pose_preview}). "
        f"elem_a=(name={worst.elem_a_name!r} origin_xyz={getattr(worst.elem_a_origin, 'xyz', None)} "
        f"origin_rpy={getattr(worst.elem_a_origin, 'rpy', None)} geom={worst.elem_a_geometry}) "
        f"elem_b=(name={worst.elem_b_name!r} origin_xyz={getattr(worst.elem_b_origin, 'xyz', None)} "
        f"origin_rpy={getattr(worst.elem_b_origin, 'rpy', None)} geom={worst.elem_b_geometry}). "
        f"hint: try moving {worst.link_b!r} by ~{suggested_move:.4g}m along world {axis} "
        "(or adjust the responsible articulation origin/limits or element origins). "
        "Note: this QC is conservative (bounding boxes); concave mesh collisions can yield false positives. "
        "Fix by adjusting articulation origins/limits and/or visual element origins so parts do not interpenetrate "
        "in the rest pose or across the articulated range."
    )


def default_overlap_tol_from_env() -> float:
    raw = os.environ.get(
        "OBJECT_GEOMETRY_OVERLAP_TOL", os.environ.get("URDF_GEOMETRY_OVERLAP_TOL", "0.005")
    )
    try:
        return float(raw)
    except ValueError:
        return 0.005


def default_overlap_volume_tol_from_env() -> float:
    raw = os.environ.get(
        "OBJECT_GEOMETRY_OVERLAP_VOLUME_TOL",
        os.environ.get("URDF_GEOMETRY_OVERLAP_VOLUME_TOL", "5e-7"),
    )
    try:
        return float(raw)
    except ValueError:
        return 5e-7


def default_contact_tol_from_env() -> float:
    raw = os.environ.get(
        "OBJECT_GEOMETRY_CONTACT_TOL",
        os.environ.get("URDF_GEOMETRY_CONTACT_TOL", "1e-6"),
    )
    try:
        return float(raw)
    except ValueError:
        return 1e-6
