from __future__ import annotations

import math
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Set, Tuple, Union

from ..assets import resolve_asset_root, resolve_mesh_path
from ..errors import ValidationError
from ..exact_collisions import compile_object_model_with_exact_collisions
from ..geometry_qc import (
    GeometryOverlap,
    _coerce_joint_pose_value,
    _collision_object_from_geometry,
    _identity4,
    _mat4_mul,
    _origin_to_mat4,
    compute_part_world_transforms,
    default_contact_tol_from_env,
    default_overlap_tol_from_env,
    default_overlap_volume_tol_from_env,
    find_geometry_overlaps,
    find_geometry_overlaps_in_poses,
    find_joint_origin_distance_findings,
    find_part_geometry_connectivity_findings,
    find_unsupported_parts,
    generate_pose_samples,
    part_world_aabb,
)
from ..types import ArticulationType, Box, Cylinder, Mesh, Origin, Sphere

Vec3 = Tuple[float, float, float]
AABB = Tuple[Vec3, Vec3]
PoseValue = Union[float, Origin]

_JOINT_ORIGIN_TOL_DEFAULT = 0.015
_JOINT_ORIGIN_TOL_MAX = 0.15
_JOINT_ORIGIN_TOL_DECIMALS = 3


@dataclass(frozen=True)
class TestFailure:
    name: str
    details: str


@dataclass(frozen=True)
class AllowedOverlap:
    link_a: str
    link_b: str
    reason: str
    elem_a: Optional[str] = None
    elem_b: Optional[str] = None


@dataclass(frozen=True)
class TestReport:
    passed: bool
    checks_run: int
    checks: Tuple[str, ...]
    failures: Tuple[TestFailure, ...]
    warnings: Tuple[str, ...] = ()
    allowances: Tuple[str, ...] = ()
    allowed_isolated_parts: Tuple[str, ...] = ()
    allowed_overlaps: Tuple[AllowedOverlap, ...] = ()


@dataclass(frozen=True)
class _CoplanarFinding:
    link_a: str
    link_b: str
    relation: str
    risk: str
    pose_index: int
    pose: Dict[str, object]
    axis_name: str
    overlap_axis_0: str
    overlap_axis_1: str
    face_a: str
    face_b: str
    plane_delta: float
    overlap_0: float
    overlap_1: float
    area: float
    overlap_ratio: float
    thin_pair: bool
    elem_a_name: Optional[str]
    elem_a_geometry: Optional[str]
    elem_b_name: Optional[str]
    elem_b_geometry: Optional[str]


@dataclass(frozen=True)
class _ExactElement:
    part_name: str
    index: int
    elem_name: Optional[str]
    geometry_name: str
    geometry: object
    origin: Origin
    tf: object
    collision_obj: object


def _parse_xyz(xyz: str) -> Optional[Vec3]:
    parts = (xyz or "").strip().split()
    if len(parts) != 3:
        return None
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return None


def _aabb_center(aabb: AABB) -> Vec3:
    (mn, mx) = aabb
    return ((mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5)


def _pair_key(name_a: str, name_b: str) -> Tuple[str, str]:
    return (name_a, name_b) if name_a <= name_b else (name_b, name_a)


def _aabbs_touch_or_overlap(a: AABB, b: AABB, *, tol: float) -> bool:
    for axis in range(3):
        if a[1][axis] + tol < b[0][axis]:
            return False
        if b[1][axis] + tol < a[0][axis]:
            return False
    return True


def _format_overlap_element(index: int, name: Optional[str], geometry: Optional[str]) -> str:
    geometry_name = geometry or "?"
    if isinstance(name, str) and name:
        return f"#{index} {name!r}:{geometry_name}"
    return f"#{index} <unnamed>:{geometry_name}"


def _overlap_rank(overlap: GeometryOverlap) -> tuple[float, float]:
    return (
        float(min(overlap.overlap_depth[0], overlap.overlap_depth[1], overlap.overlap_depth[2])),
        float(overlap.overlap_volume),
    )


def _format_unsupported_part_finding(finding: object) -> str:
    part = getattr(finding, "part", None)
    parts = getattr(finding, "parts", ())
    nearest_part = getattr(finding, "nearest_part", None)
    min_distance = getattr(finding, "min_distance", None)
    pose_index = getattr(finding, "pose_index", None)
    pose = getattr(finding, "pose", None)
    backend = getattr(finding, "backend", None)
    root_parts = getattr(finding, "root_parts", ())

    component_parts = tuple(
        str(item).strip() for item in parts if isinstance(item, str) and str(item).strip()
    )
    if not component_parts and isinstance(part, str) and part.strip():
        component_parts = (part.strip(),)

    rooted_parts = tuple(
        str(item).strip() for item in root_parts if isinstance(item, str) and str(item).strip()
    )

    gap_text = "unknown"
    if isinstance(min_distance, (int, float)) and math.isfinite(float(min_distance)):
        gap_text = f"{float(min_distance):.4g}m"

    pose_preview = ""
    if isinstance(pose, dict):
        pairs: list[str] = []
        for key, value in sorted(pose.items()):
            try:
                pairs.append(f"{key}={float(value):.4g}")
            except Exception:
                pairs.append(f"{key}={value}")
        pose_preview = ", ".join(pairs)

    pose_label = "at rest pose"
    if pose_preview:
        pose_label = f"at pose ({pose_preview})"
    if isinstance(pose_index, int):
        pose_label = f"pose_index={pose_index} {pose_label}"

    if len(component_parts) == 1:
        subject = f"part {component_parts[0]!r}"
    elif component_parts:
        subject = f"floating group {list(component_parts)!r}"
    else:
        subject = "part <unknown>"

    if len(rooted_parts) == 1:
        grounded_body = f"the grounded body rooted at {rooted_parts[0]!r}"
    elif rooted_parts:
        grounded_body = f"the grounded body rooted at {list(rooted_parts)!r}"
    else:
        grounded_body = "the grounded body"

    return (
        f"- {pose_label}, {subject} is disconnected from {grounded_body}; "
        f"nearest_grounded_part={nearest_part!r}; approx_gap={gap_text}; backend={backend}"
    )


def _unsupported_finding_parts(finding: object) -> tuple[str, ...]:
    parts = getattr(finding, "parts", ())
    component_parts = tuple(
        str(item).strip() for item in parts if isinstance(item, str) and str(item).strip()
    )
    if component_parts:
        return component_parts
    part = getattr(finding, "part", None)
    if isinstance(part, str) and part.strip():
        return (part.strip(),)
    return ()


def _truncate_decimal(value: float, *, decimals: int) -> float:
    scale = 10**decimals
    return math.trunc(float(value) * scale) / scale


def _normalize_joint_origin_tol(value: float) -> float:
    tol = float(value)
    if not math.isfinite(tol) or tol < 0.0:
        return tol
    tol = _truncate_decimal(tol, decimals=_JOINT_ORIGIN_TOL_DECIMALS)
    return min(tol, _JOINT_ORIGIN_TOL_MAX)


def _normalize_axis_name(
    world_axis: Union[str, Sequence[float]],
) -> Tuple[Optional[str], float, Optional[str]]:
    if isinstance(world_axis, str):
        axis_key = world_axis.strip().lower()
        if axis_key in ("x", "y", "z"):
            return axis_key, 1.0, None
        return None, 1.0, "world_axis must be one of: x, y, z"

    if isinstance(world_axis, Sequence) and len(world_axis) == 3:
        try:
            comps = tuple(float(v) for v in world_axis)
        except (TypeError, ValueError):
            return None, 1.0, "world_axis must be one of: x, y, z"

        abs_comps = [abs(v) for v in comps]
        dominant = max(abs_comps)
        if dominant <= 1e-12:
            return None, 1.0, "world_axis must be one of: x, y, z"
        axis_index = abs_comps.index(dominant)
        if any(i != axis_index and abs_comps[i] > 1e-6 for i in range(3)):
            return None, 1.0, "world_axis must be one of: x, y, z"
        axis_key = ("x", "y", "z")[axis_index]
        axis_sign = 1.0 if comps[axis_index] >= 0.0 else -1.0
        return axis_key, axis_sign, None

    return None, 1.0, "world_axis must be one of: x, y, z"


def _normalize_direction_name(
    direction: Union[str, float, int],
) -> Tuple[Optional[str], Optional[str]]:
    if isinstance(direction, str):
        dir_key = direction.strip().lower()
        if dir_key in ("positive", "negative"):
            return dir_key, None
        try:
            direction = float(dir_key)
        except ValueError:
            return None, "direction must be either 'positive' or 'negative'"
    try:
        direction_f = float(direction)
    except (TypeError, ValueError):
        return None, "direction must be either 'positive' or 'negative'"
    if abs(direction_f) <= 1e-12:
        return None, "direction must be non-zero"
    return ("positive" if direction_f > 0.0 else "negative"), None


def _normalize_axes_spec(
    axes: Union[str, Sequence[str]],
    *,
    field_name: str,
) -> Tuple[Tuple[str, ...], Optional[str]]:
    if isinstance(axes, str):
        tokens = [ch for ch in axes.strip().lower() if not ch.isspace() and ch != ","]
    elif isinstance(axes, Sequence):
        tokens = []
        for axis in axes:
            token = str(axis).strip().lower()
            if len(token) != 1:
                return (), f"{field_name} must contain only axis names 'x', 'y', or 'z'"
            tokens.append(token)
    else:
        return (), f"{field_name} must contain only axis names 'x', 'y', or 'z'"

    if not tokens:
        return (), f"{field_name} must include at least one axis"

    seen: set[str] = set()
    normalized: list[str] = []
    for token in tokens:
        if token not in {"x", "y", "z"}:
            return (), f"{field_name} must contain only axis names 'x', 'y', or 'z'"
        if token in seen:
            continue
        seen.add(token)
        normalized.append(token)
    if not normalized:
        return (), f"{field_name} must include at least one axis"
    return tuple(normalized), None


def _axes_label(axes: Sequence[str]) -> str:
    return "".join(str(axis) for axis in axes)


def _axis_index(axis: str) -> int:
    return {"x": 0, "y": 1, "z": 2}[axis]


def _point_distance_on_axes(a: Vec3, b: Vec3, *, axes: Sequence[str]) -> float:
    total = 0.0
    for axis in axes:
        idx = _axis_index(axis)
        delta = float(a[idx]) - float(b[idx])
        total += delta * delta
    return total**0.5


def _aabb_axis_overlap(aabb_a: AABB, aabb_b: AABB, *, axis: str) -> float:
    idx = _axis_index(axis)
    return min(float(aabb_a[1][idx]), float(aabb_b[1][idx])) - max(
        float(aabb_a[0][idx]), float(aabb_b[0][idx])
    )


def _aabb_axis_span(aabb: AABB, *, axis: str) -> float:
    idx = _axis_index(axis)
    return max(0.0, float(aabb[1][idx]) - float(aabb[0][idx]))


def _aabb_axis_gap(
    positive_aabb: AABB,
    negative_aabb: AABB,
    *,
    axis: str,
) -> float:
    idx = _axis_index(axis)
    return float(positive_aabb[0][idx]) - float(negative_aabb[1][idx])


def _aabb_axis_separation(aabb_a: AABB, aabb_b: AABB, *, axis: str) -> float:
    idx = _axis_index(axis)
    lower_gap = float(aabb_b[0][idx]) - float(aabb_a[1][idx])
    upper_gap = float(aabb_a[0][idx]) - float(aabb_b[1][idx])
    return max(0.0, lower_gap, upper_gap)


def _dot_vec3(a: Vec3, b: Vec3) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def _axis_unit(axis: str) -> Vec3:
    return {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}[axis]


def _transform_aabb(aabb: AABB, tf: object) -> AABB:
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    corners = [(x, y, z) for x in (min_x, max_x) for y in (min_y, max_y) for z in (min_z, max_z)]
    tx0 = ty0 = tz0 = float("inf")
    tx1 = ty1 = tz1 = float("-inf")
    for cx, cy, cz in corners:
        x = float(tf[0][0]) * cx + float(tf[0][1]) * cy + float(tf[0][2]) * cz + float(tf[0][3])  # type: ignore[index]
        y = float(tf[1][0]) * cx + float(tf[1][1]) * cy + float(tf[1][2]) * cz + float(tf[1][3])  # type: ignore[index]
        z = float(tf[2][0]) * cx + float(tf[2][1]) * cy + float(tf[2][2]) * cz + float(tf[2][3])  # type: ignore[index]
        tx0 = min(tx0, x)
        ty0 = min(ty0, y)
        tz0 = min(tz0, z)
        tx1 = max(tx1, x)
        ty1 = max(ty1, y)
        tz1 = max(tz1, z)
    return (tx0, ty0, tz0), (tx1, ty1, tz1)


def _named_ref(value: object, *, kind: str) -> str:
    if isinstance(value, str):
        name = value
    else:
        name = getattr(value, "name", None)
        if not isinstance(name, str):
            raise ValidationError(f"{kind} must be a name or object with a .name")
    name = name.strip()
    if not name:
        raise ValidationError(f"{kind} must be non-empty")
    return name


__all__ = [name for name in globals() if not name.startswith("__")]
