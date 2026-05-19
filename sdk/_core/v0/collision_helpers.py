from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple, Union

from .assets import AssetContext, resolve_asset_root
from .errors import ValidationError
from .geometry_qc import part_local_aabbs
from .types import Box, Collision, Cylinder, Origin, Part, Sphere

Vec3 = Tuple[float, float, float]


def _normalize(v: Vec3) -> Vec3:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n <= 0.0:
        raise ValueError("Cannot normalize zero vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def _rpy_matrix(rpy: Vec3) -> tuple[Vec3, Vec3, Vec3]:
    roll, pitch, yaw = rpy
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _rotate_vec(rpy: Vec3, vec: Vec3) -> Vec3:
    rot = _rpy_matrix(rpy)
    x, y, z = vec
    return (
        rot[0][0] * x + rot[0][1] * y + rot[0][2] * z,
        rot[1][0] * x + rot[1][1] * y + rot[1][2] * z,
        rot[2][0] * x + rot[2][1] * y + rot[2][2] * z,
    )


def _rpy_from_z_axis(direction: Vec3) -> Vec3:
    dx, dy, dz = _normalize(direction)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return (0.0, pitch, yaw)


def add_collisions(part: Part, collisions: Iterable[Collision]) -> None:
    """Append an iterable of collisions to a part."""
    for c in collisions:
        part.collisions.append(c)


def _aabb_union(aabbs: Sequence[Tuple[Vec3, Vec3]]) -> Tuple[Vec3, Vec3]:
    if not aabbs:
        raise ValidationError("Cannot union an empty AABB set")
    return (
        min(aabb[0][0] for aabb in aabbs),
        min(aabb[0][1] for aabb in aabbs),
        min(aabb[0][2] for aabb in aabbs),
    ), (
        max(aabb[1][0] for aabb in aabbs),
        max(aabb[1][1] for aabb in aabbs),
        max(aabb[1][2] for aabb in aabbs),
    )


def _aabbs_touch_or_overlap(a: Tuple[Vec3, Vec3], b: Tuple[Vec3, Vec3], *, tol: float) -> bool:
    for axis in range(3):
        if a[1][axis] + tol < b[0][axis]:
            return False
        if b[1][axis] + tol < a[0][axis]:
            return False
    return True


def _padding_xyz(padding: Union[float, Vec3]) -> Vec3:
    if isinstance(padding, (int, float)):
        p = float(padding)
        if p < 0.0:
            raise ValidationError("padding must be non-negative")
        return (p, p, p)
    if len(padding) != 3:
        raise ValidationError("padding must be a float or a 3-tuple")
    px, py, pz = float(padding[0]), float(padding[1]), float(padding[2])
    if px < 0.0 or py < 0.0 or pz < 0.0:
        raise ValidationError("padding values must be non-negative")
    return (px, py, pz)


def _visual_element_aabbs(
    part: Part,
    *,
    asset_root: Optional[Union[str, AssetContext]],
) -> List[Tuple[Vec3, Vec3]]:
    root = resolve_asset_root(asset_root, part)
    obj_cache = {}
    aabbs: List[Tuple[Vec3, Vec3]] = []
    for visual in list(part.visuals or []):
        geometry = getattr(visual, "geometry", None)
        if geometry is None:
            continue
        local_aabb = part_local_aabbs(
            Part(
                name=f"{part.name}_tmp",
                visuals=[visual],
                assets=part.assets,
            ),
            asset_root=root,
            prefer_collisions=False,
            _obj_cache=obj_cache,
        )
        aabbs.extend(local_aabb)
    return aabbs


def _cluster_aabbs(aabbs: Sequence[Tuple[Vec3, Vec3]], *, tol: float) -> List[Tuple[Vec3, Vec3]]:
    if not aabbs:
        return []
    remaining = list(range(len(aabbs)))
    clusters: List[List[int]] = []
    while remaining:
        seed = remaining.pop()
        cluster = [seed]
        queue = [seed]
        while queue:
            current = queue.pop()
            next_remaining: List[int] = []
            for idx in remaining:
                if _aabbs_touch_or_overlap(aabbs[current], aabbs[idx], tol=tol):
                    cluster.append(idx)
                    queue.append(idx)
                else:
                    next_remaining.append(idx)
            remaining = next_remaining
        clusters.append(cluster)
    return [_aabb_union([aabbs[idx] for idx in cluster]) for cluster in clusters]


def _box_collision_from_aabb(
    aabb: Tuple[Vec3, Vec3],
    *,
    padding: Vec3,
    min_size: float,
    name: str,
) -> Collision:
    (mn, mx) = aabb
    px, py, pz = padding
    sx = max(float(min_size), float(mx[0] - mn[0]) + 2.0 * px)
    sy = max(float(min_size), float(mx[1] - mn[1]) + 2.0 * py)
    sz = max(float(min_size), float(mx[2] - mn[2]) + 2.0 * pz)
    center = (
        (float(mn[0]) + float(mx[0])) * 0.5,
        (float(mn[1]) + float(mx[1])) * 0.5,
        (float(mn[2]) + float(mx[2])) * 0.5,
    )
    return Collision(
        geometry=Box((sx, sy, sz)),
        origin=Origin(xyz=center),
        name=name,
    )


def autocollide_part(
    part: Part,
    *,
    mode: str = "clustered",
    asset_root: Optional[Union[str, AssetContext]] = None,
    replace: bool = True,
    cluster_tol: float = 0.005,
    padding: Union[float, Vec3] = 0.0,
    min_size: float = 1e-4,
    name_prefix: str = "auto_collision",
) -> List[Collision]:
    """
    Derive conservative box collisions from visible geometry.

    ``clustered`` is the preferred mode because it preserves disconnected
    subassemblies as separate collision islands instead of collapsing the entire
    part into one union box.
    """

    mode_key = str(mode or "").strip().lower()
    if mode_key not in {"per_visual", "clustered", "single"}:
        raise ValidationError("mode must be one of: 'per_visual', 'clustered', 'single'")
    if float(cluster_tol) < 0.0:
        raise ValidationError("cluster_tol must be non-negative")
    if float(min_size) <= 0.0:
        raise ValidationError("min_size must be positive")

    visual_aabbs = _visual_element_aabbs(part, asset_root=asset_root)
    if not visual_aabbs:
        raise ValidationError(
            f"part {part.name!r} has no visual geometry to derive collisions from"
        )

    if mode_key == "per_visual":
        target_aabbs = list(visual_aabbs)
    elif mode_key == "single":
        target_aabbs = [_aabb_union(visual_aabbs)]
    else:
        target_aabbs = _cluster_aabbs(visual_aabbs, tol=float(cluster_tol))

    pad = _padding_xyz(padding)
    collisions = [
        _box_collision_from_aabb(
            aabb,
            padding=pad,
            min_size=float(min_size),
            name=f"{name_prefix}_{idx}",
        )
        for idx, aabb in enumerate(target_aabbs, start=1)
    ]
    if replace:
        part.collisions = collisions
    else:
        part.collisions.extend(collisions)
    return collisions


def make_capsule_collisions(
    *,
    radius: float,
    length: float,
    axis: str = "z",
    name_prefix: str = "capsule",
    origin: Optional[Origin] = None,
) -> List[Collision]:
    """
    Approximate a capsule (cylinder + two spheres) using primitive collisions.

    - `length` is the cylinder segment length between the sphere centers.
    - Total end-to-end length is `length + 2*radius`.
    - `axis` selects the capsule's main axis in the part frame: 'x' | 'y' | 'z'.
    """

    r = float(radius)
    segment_length = float(length)
    if r <= 0 or segment_length <= 0:
        raise ValueError("radius and length must be positive")
    ax = (axis or "z").strip().lower()
    if ax not in {"x", "y", "z"}:
        raise ValueError("axis must be one of: 'x', 'y', 'z'")

    base = origin or Origin()
    axis_local: dict[str, Vec3] = {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
    }
    axis_world = _rotate_vec(base.rpy, axis_local[ax])
    offset = tuple(component * (segment_length * 0.5) for component in axis_world)
    cyl_rpy = _rpy_from_z_axis(axis_world)
    s1_xyz = (
        base.xyz[0] - offset[0],
        base.xyz[1] - offset[1],
        base.xyz[2] - offset[2],
    )
    s2_xyz = (
        base.xyz[0] + offset[0],
        base.xyz[1] + offset[1],
        base.xyz[2] + offset[2],
    )

    return [
        Collision(
            geometry=Cylinder(radius=r, length=segment_length),
            origin=Origin(xyz=base.xyz, rpy=cyl_rpy),
            name=f"{name_prefix}_cyl",
        ),
        Collision(
            geometry=Sphere(radius=r),
            origin=Origin(xyz=s1_xyz, rpy=base.rpy),
            name=f"{name_prefix}_s1",
        ),
        Collision(
            geometry=Sphere(radius=r),
            origin=Origin(xyz=s2_xyz, rpy=base.rpy),
            name=f"{name_prefix}_s2",
        ),
    ]


def make_hollow_box_collisions(
    *,
    inner_size: Sequence[float],
    wall_thickness: float,
    floor_thickness: float = 0.0,
    name_prefix: str = "hollow_box",
) -> List[Collision]:
    """
    Create a simple open-top rectangular container using 4 wall boxes (+ optional floor).

    The interior spans:
    - x ∈ [-inner_x/2, +inner_x/2]
    - y ∈ [-inner_y/2, +inner_y/2]
    - z ∈ [0, inner_z]

    Use this for bowls/cups/bins where contents should be allowed inside the cavity.
    """

    inner_x, inner_y, inner_z = (float(inner_size[0]), float(inner_size[1]), float(inner_size[2]))
    t = float(wall_thickness)
    ft = float(floor_thickness)
    if inner_x <= 0 or inner_y <= 0 or inner_z <= 0:
        raise ValueError("inner_size must be positive")
    if t <= 0:
        raise ValueError("wall_thickness must be positive")
    if ft < 0:
        raise ValueError("floor_thickness must be >= 0")

    walls: List[Collision] = []

    # Side walls.
    walls.append(
        Collision(
            geometry=Box((t, inner_y + 2 * t, inner_z)),
            origin=Origin(xyz=(inner_x * 0.5 + t * 0.5, 0.0, inner_z * 0.5)),
            name=f"{name_prefix}_wall_xp",
        )
    )
    walls.append(
        Collision(
            geometry=Box((t, inner_y + 2 * t, inner_z)),
            origin=Origin(xyz=(-(inner_x * 0.5 + t * 0.5), 0.0, inner_z * 0.5)),
            name=f"{name_prefix}_wall_xn",
        )
    )
    walls.append(
        Collision(
            geometry=Box((inner_x, t, inner_z)),
            origin=Origin(xyz=(0.0, inner_y * 0.5 + t * 0.5, inner_z * 0.5)),
            name=f"{name_prefix}_wall_yp",
        )
    )
    walls.append(
        Collision(
            geometry=Box((inner_x, t, inner_z)),
            origin=Origin(xyz=(0.0, -(inner_y * 0.5 + t * 0.5), inner_z * 0.5)),
            name=f"{name_prefix}_wall_yn",
        )
    )

    # Optional floor.
    if ft > 0:
        walls.append(
            Collision(
                geometry=Box((inner_x + 2 * t, inner_y + 2 * t, ft)),
                origin=Origin(xyz=(0.0, 0.0, ft * 0.5)),
                name=f"{name_prefix}_floor",
            )
        )

    return walls


def make_hollow_cylinder_wall_collisions(
    *,
    inner_radius: float,
    height: float,
    wall_thickness: float,
    segments: int = 12,
    floor_thickness: float = 0.0,
    name_prefix: str = "hollow_cyl",
) -> List[Collision]:
    """
    Approximate a hollow cylinder's walls using `segments` rotated boxes (+ optional floor).

    Interior:
    - radial distance <= inner_radius
    - z ∈ [0, height]

    This is useful for bowls/cups where contents/attachments should occupy the interior.
    """

    r = float(inner_radius)
    h = float(height)
    t = float(wall_thickness)
    n = max(3, int(segments))
    ft = float(floor_thickness)
    if r <= 0 or h <= 0:
        raise ValueError("inner_radius and height must be positive")
    if t <= 0:
        raise ValueError("wall_thickness must be positive")
    if ft < 0:
        raise ValueError("floor_thickness must be >= 0")

    # Chord length of the inscribed polygon edge.
    chord = 2.0 * r * math.sin(math.pi / n)
    # Slightly enlarge tangential span to avoid gaps.
    span = chord + 0.5 * t

    out: List[Collision] = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        cx = (r + t * 0.5) * math.cos(theta)
        cy = (r + t * 0.5) * math.sin(theta)
        out.append(
            Collision(
                geometry=Box((t, span, h)),
                origin=Origin(xyz=(cx, cy, h * 0.5), rpy=(0.0, 0.0, theta)),
                name=f"{name_prefix}_wall_{i}",
            )
        )

    if ft > 0:
        out.append(
            Collision(
                geometry=Cylinder(radius=r + t, length=ft),
                origin=Origin(xyz=(0.0, 0.0, ft * 0.5)),
                name=f"{name_prefix}_floor",
            )
        )

    return out
