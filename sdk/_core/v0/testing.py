from __future__ import annotations

import math
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Set, Tuple, Union

from .assets import resolve_asset_root, resolve_mesh_path
from .errors import ValidationError
from .exact_collisions import compile_object_model_with_exact_collisions
from .geometry_qc import (
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
from .types import ArticulationType, Box, Cylinder, Mesh, Origin, Sphere

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
class TestReport:
    passed: bool
    checks_run: int
    checks: Tuple[str, ...]
    failures: Tuple[TestFailure, ...]
    warnings: Tuple[str, ...] = ()
    allowances: Tuple[str, ...] = ()
    allowed_isolated_parts: Tuple[str, ...] = ()


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


@dataclass
class TestContext:
    model: object
    asset_root: Optional[Union[str, Path]] = None
    seed: int = 0

    checks_run: int = 0
    _checks: List[str] = field(default_factory=list)
    _failures: List[TestFailure] = field(default_factory=list)
    _warnings: List[str] = field(default_factory=list)
    _allow_pairs: Dict[Tuple[str, str], str] = field(default_factory=dict)
    _allowances: List[str] = field(default_factory=list)
    _allow_isolated_parts: Dict[str, str] = field(default_factory=dict)
    _allow_elems: List[Tuple[Tuple[str, str], Optional[str], Optional[str], str]] = field(
        default_factory=list, repr=False
    )
    _allow_coplanar_pairs: Dict[Tuple[str, str], str] = field(default_factory=dict)
    _allow_coplanar_elems: List[Tuple[Tuple[str, str], Optional[str], Optional[str], str]] = field(
        default_factory=list, repr=False
    )

    _pose: Dict[str, float] = field(default_factory=dict)
    _world_tfs_cache: Optional[Dict[str, object]] = None
    _parts_by_name_cache: Optional[Dict[str, object]] = field(default=None, repr=False)
    _mesh_aabb_cache: Dict[Path, AABB] = field(default_factory=dict, repr=False)
    _part_local_aabbs_cache: Dict[str, Tuple[AABB, ...]] = field(default_factory=dict, repr=False)
    _part_world_aabb_cache: Dict[str, Optional[AABB]] = field(default_factory=dict, repr=False)
    _exact_compiled_model_cache: Optional[object] = field(default=None, repr=False)
    _exact_local_elements_cache: Dict[str, Tuple[_ExactElement, ...]] = field(
        default_factory=dict, repr=False
    )
    _exact_world_elements_cache: Optional[Dict[str, Tuple[_ExactElement, ...]]] = field(
        default=None, repr=False
    )
    _exact_mesh_cache: Dict[tuple[Path, Optional[Vec3]], object] = field(
        default_factory=dict, repr=False
    )
    _mesh_vertex_cache: Dict[Path, Tuple[Vec3, ...]] = field(default_factory=dict, repr=False)
    _warned_deprecations: Set[str] = field(default_factory=set, repr=False)

    def report(self) -> TestReport:
        failures = tuple(self._failures)
        return TestReport(
            passed=not failures,
            checks_run=int(self.checks_run),
            checks=tuple(self._checks),
            failures=failures,
            warnings=tuple(self._warnings),
            allowances=tuple(self._allowances),
            allowed_isolated_parts=tuple(self._allow_isolated_parts),
        )

    def _record(self, name: str, ok: bool, details: str = "") -> bool:
        self.checks_run += 1
        self._checks.append(name)
        if not ok:
            self._failures.append(TestFailure(name=name, details=details))
        return ok

    def _record_warning_check(self, name: str, ok: bool, details: str = "") -> bool:
        self.checks_run += 1
        self._checks.append(name)
        if not ok:
            text = str(details or "").strip()
            if text:
                self._warnings.append(f"{name}: {text}")
            else:
                self._warnings.append(name)
        return ok

    def check(self, name: str, ok: bool, details: str = "") -> bool:
        """Record a custom boolean check."""
        return self._record(str(name), bool(ok), str(details or ""))

    def fail(self, name: str, details: str) -> bool:
        """Convenience helper to record a failure."""
        return self._record(str(name), False, str(details or ""))

    def warn(self, text: str) -> None:
        if text:
            self._warnings.append(str(text))

    @staticmethod
    def _overlap_name_aliases(name: Optional[str]) -> set[str]:
        if not isinstance(name, str):
            return set()
        cleaned = name.strip()
        if not cleaned:
            return set()
        aliases = {cleaned}
        prefix, sep, suffix = cleaned.rpartition("__component_")
        if sep and suffix.isdigit() and prefix:
            aliases.add(prefix)
        return aliases

    def _overlap_is_allowed(self, overlap: GeometryOverlap) -> bool:
        key = _pair_key(overlap.link_a, overlap.link_b)
        for allowed_key, elem_a, elem_b, _reason in self._allow_elems:
            if allowed_key != key:
                continue
            if elem_a is None and elem_b is None:
                return True
            aliases_a = self._overlap_name_aliases(overlap.elem_a_name)
            aliases_b = self._overlap_name_aliases(overlap.elem_b_name)
            if (elem_a in aliases_a and elem_b in aliases_b) or (
                elem_a in aliases_b and elem_b in aliases_a
            ):
                return True
        return False

    def _asset_root(self) -> Path:
        resolved = resolve_asset_root(self.asset_root, self.model)
        if resolved is not None:
            return resolved
        return Path(".").resolve()

    def _parts_by_name(self) -> Dict[str, object]:
        if self._parts_by_name_cache is None:
            parts = getattr(self.model, "parts", None)
            if not isinstance(parts, list):
                self._parts_by_name_cache = {}
            else:
                self._parts_by_name_cache = {
                    name: part
                    for part in parts
                    if isinstance((name := getattr(part, "name", None)), str)
                }
        return self._parts_by_name_cache

    def _part_by_name(self, part_name: str) -> Optional[object]:
        return self._parts_by_name().get(part_name)

    def _part_local_aabbs(
        self,
        part: object,
    ) -> Tuple[AABB, ...]:
        from .geometry_qc import part_local_aabbs  # local import

        part_name = _named_ref(part, kind="part")
        cached = self._part_local_aabbs_cache.get(part_name)
        if cached is not None:
            return cached

        part_obj = self._part_by_name(part_name)
        if part_obj is None:
            return ()

        cached = tuple(
            part_local_aabbs(
                part_obj,
                asset_root=self._asset_root(),
                prefer_collisions=False,
                _obj_cache=self._mesh_aabb_cache,
            )
        )
        self._part_local_aabbs_cache[part_name] = cached
        return cached

    def _part_geometry_items_with_local_aabbs(
        self,
        part: object,
    ) -> List[Tuple[Optional[str], Optional[str], AABB]]:
        part_name = _named_ref(part, kind="part")
        part_obj = self._part_by_name(part_name)
        if part_obj is None:
            return []

        raw_items = list(getattr(part_obj, "visuals", []) or [])
        items = [item for item in raw_items if getattr(item, "geometry", None) is not None]
        aabbs = self._part_local_aabbs(part_name)

        out: List[Tuple[Optional[str], Optional[str], AABB]] = []
        for idx, local_aabb in enumerate(aabbs):
            item = items[idx] if idx < len(items) else None
            out.append(
                (
                    getattr(item, "name", None) if item is not None else None,
                    type(getattr(item, "geometry", None)).__name__ if item is not None else None,
                    local_aabb,
                )
            )
        return out

    def _compiled_exact_model(self) -> object:
        if self._exact_compiled_model_cache is None:
            self._exact_compiled_model_cache = compile_object_model_with_exact_collisions(
                self.model,
                asset_root=self._asset_root(),
            )
        return self._exact_compiled_model_cache

    def _compiled_exact_part(self, part_name: str) -> Optional[object]:
        compiled_model = self._compiled_exact_model()
        get_part = getattr(compiled_model, "get_part", None)
        if callable(get_part):
            try:
                return get_part(part_name)
            except Exception:
                return None
        parts = getattr(compiled_model, "parts", None)
        if not isinstance(parts, list):
            return None
        for part in parts:
            if getattr(part, "name", None) == part_name:
                return part
        return None

    def _build_exact_elements_for_part(
        self,
        part_name: str,
        *,
        world: bool,
        world_tfs: Optional[Dict[str, object]] = None,
    ) -> Tuple[_ExactElement, ...]:
        compiled_part = self._compiled_exact_part(part_name)
        if compiled_part is None:
            return ()
        asset_root = self._asset_root()
        part_tf = _identity4()
        if world:
            if world_tfs is None:
                world_tfs = compute_part_world_transforms(
                    self._compiled_exact_model(),
                    dict(self._pose),
                )
            candidate_tf = world_tfs.get(part_name)
            if candidate_tf is not None:
                part_tf = candidate_tf  # type: ignore[assignment]
        elements: list[_ExactElement] = []
        for index, item in enumerate(list(getattr(compiled_part, "collisions", []) or [])):
            geometry = getattr(item, "geometry", None)
            if geometry is None:
                continue
            local_tf = _origin_to_mat4(getattr(item, "origin", Origin()))
            elem_tf = _mat4_mul(part_tf, local_tf)
            collision_obj = _collision_object_from_geometry(
                geometry,
                elem_tf=elem_tf,
                asset_root=asset_root,
                mesh_cache=self._exact_mesh_cache,
            )
            elements.append(
                _ExactElement(
                    part_name=part_name,
                    index=index,
                    elem_name=getattr(item, "name", None),
                    geometry_name=type(geometry).__name__,
                    geometry=geometry,
                    origin=getattr(item, "origin", Origin()),
                    tf=elem_tf,
                    collision_obj=collision_obj,
                )
            )
        return tuple(elements)

    def _exact_local_elements(self, part: object) -> Tuple[_ExactElement, ...]:
        part_name = _named_ref(part, kind="part")
        cached = self._exact_local_elements_cache.get(part_name)
        if cached is not None:
            return cached
        cached = self._build_exact_elements_for_part(part_name, world=False)
        self._exact_local_elements_cache[part_name] = cached
        return cached

    def _exact_world_elements(self) -> Dict[str, Tuple[_ExactElement, ...]]:
        if self._exact_world_elements_cache is None:
            world_tfs = compute_part_world_transforms(
                self._compiled_exact_model(), dict(self._pose)
            )
            compiled_model = self._compiled_exact_model()
            parts = getattr(compiled_model, "parts", None)
            built: Dict[str, Tuple[_ExactElement, ...]] = {}
            if isinstance(parts, list):
                for part in parts:
                    part_name = getattr(part, "name", None)
                    if not isinstance(part_name, str):
                        continue
                    built[part_name] = self._build_exact_elements_for_part(
                        part_name,
                        world=True,
                        world_tfs=world_tfs,
                    )
            self._exact_world_elements_cache = built
        return self._exact_world_elements_cache

    def _resolve_exact_elements(
        self,
        part: object,
        *,
        elem: Optional[object] = None,
        kind_prefix: str,
    ) -> Tuple[Optional[Tuple[_ExactElement, ...]], Optional[str], Optional[str], Optional[str]]:
        part_name = _named_ref(part, kind=f"{kind_prefix}_link")
        elements = self._exact_world_elements().get(part_name, ())
        if not elements:
            return None, part_name, None, f"missing exact geometry for {part_name!r}"
        if elem is None:
            return elements, part_name, None, None
        elem_label = kind_prefix if kind_prefix.startswith("elem_") else f"{kind_prefix}_elem"
        elem_name = _named_ref(elem, kind=elem_label)
        matched = tuple(item for item in elements if item.elem_name == elem_name)
        if not matched:
            return (
                None,
                part_name,
                elem_name,
                f"missing exact geometry for {elem_label}={elem_name!r} on {part_name!r}",
            )
        return matched, part_name, elem_name, None

    def _warn_deprecated_helper(self, helper_name: str, replacement: str) -> None:
        if helper_name in self._warned_deprecations:
            return
        self._warned_deprecations.add(helper_name)
        self.warn(
            f"DEPRECATED: {helper_name} uses legacy AABB-envelope semantics. "
            f"Use {replacement} for exact visual-geometry checks."
        )

    def _warn_deprecated_default_helper(self, helper_name: str, guidance: str) -> None:
        key = f"default:{helper_name}"
        if key in self._warned_deprecations:
            return
        self._warned_deprecations.add(key)
        self.warn(
            f"DEPRECATED AS DEFAULT: {helper_name} is no longer recommended as a blanket "
            f"scaffold heuristic. {guidance}"
        )

    def _mesh_vertices(self, geometry: Mesh) -> Tuple[Vec3, ...]:
        mesh_path = resolve_mesh_path(geometry.filename, assets=self._asset_root())
        cached = self._mesh_vertex_cache.get(mesh_path)
        if cached is not None:
            return cached
        vertices: list[Vec3] = []
        for line in mesh_path.read_text(encoding="utf-8", errors="ignore").splitlines():
            if not line.startswith("v "):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError:
                continue
        if not vertices:
            raise ValidationError(f"OBJ contains no vertices: {mesh_path}")
        cached = tuple(vertices)
        self._mesh_vertex_cache[mesh_path] = cached
        return cached

    def _element_projection_interval(
        self, element: _ExactElement, *, axis: str
    ) -> Tuple[float, float]:
        axis_vec = _axis_unit(axis)
        tf = element.tf
        center = (float(tf[0][3]), float(tf[1][3]), float(tf[2][3]))  # type: ignore[index]
        center_proj = _dot_vec3(center, axis_vec)
        basis_x = (float(tf[0][0]), float(tf[1][0]), float(tf[2][0]))  # type: ignore[index]
        basis_y = (float(tf[0][1]), float(tf[1][1]), float(tf[2][1]))  # type: ignore[index]
        basis_z = (float(tf[0][2]), float(tf[1][2]), float(tf[2][2]))  # type: ignore[index]
        geometry = element.geometry

        if isinstance(geometry, Box):
            hx = float(geometry.size[0]) * 0.5
            hy = float(geometry.size[1]) * 0.5
            hz = float(geometry.size[2]) * 0.5
            extent = (
                hx * abs(_dot_vec3(axis_vec, basis_x))
                + hy * abs(_dot_vec3(axis_vec, basis_y))
                + hz * abs(_dot_vec3(axis_vec, basis_z))
            )
            return center_proj - extent, center_proj + extent

        if isinstance(geometry, Sphere):
            radius = float(geometry.radius)
            return center_proj - radius, center_proj + radius

        if isinstance(geometry, Cylinder):
            axis_dot = max(-1.0, min(1.0, _dot_vec3(axis_vec, basis_z)))
            half_length = 0.5 * float(geometry.length) * abs(axis_dot)
            radial = float(geometry.radius) * math.sqrt(max(0.0, 1.0 - axis_dot * axis_dot))
            extent = half_length + radial
            return center_proj - extent, center_proj + extent

        if isinstance(geometry, Mesh):
            local_axis = (
                float(tf[0][0]) * axis_vec[0]
                + float(tf[1][0]) * axis_vec[1]
                + float(tf[2][0]) * axis_vec[2],
                float(tf[0][1]) * axis_vec[0]
                + float(tf[1][1]) * axis_vec[1]
                + float(tf[2][1]) * axis_vec[2],
                float(tf[0][2]) * axis_vec[0]
                + float(tf[1][2]) * axis_vec[1]
                + float(tf[2][2]) * axis_vec[2],
            )
            min_proj = float("inf")
            max_proj = float("-inf")
            sx, sy, sz = geometry.scale if geometry.scale is not None else (1.0, 1.0, 1.0)
            for vx, vy, vz in self._mesh_vertices(geometry):
                px = float(vx) * float(sx)
                py = float(vy) * float(sy)
                pz = float(vz) * float(sz)
                proj = center_proj + px * local_axis[0] + py * local_axis[1] + pz * local_axis[2]
                min_proj = min(min_proj, proj)
                max_proj = max(max_proj, proj)
            return min_proj, max_proj

        raise ValidationError(f"Unsupported geometry type: {type(geometry).__name__}")

    def _elements_projection_interval(
        self,
        elements: Sequence[_ExactElement],
        *,
        axis: str,
    ) -> Tuple[float, float]:
        if not elements:
            raise ValidationError("exact-geometry target has no elements")
        mins: list[float] = []
        maxs: list[float] = []
        for element in elements:
            elem_min, elem_max = self._element_projection_interval(element, axis=axis)
            mins.append(elem_min)
            maxs.append(elem_max)
        return min(mins), max(maxs)

    def _exact_pair_distance(
        self,
        elements_a: Sequence[_ExactElement],
        elements_b: Sequence[_ExactElement],
    ) -> Tuple[float, bool]:
        import fcl

        min_distance = float("inf")
        collision_request = fcl.CollisionRequest()
        distance_request = fcl.DistanceRequest()
        for elem_a in elements_a:
            for elem_b in elements_b:
                if elem_a.part_name == elem_b.part_name and elem_a.index == elem_b.index:
                    continue
                collision_result = fcl.CollisionResult()
                if (
                    int(
                        fcl.collide(
                            elem_a.collision_obj,
                            elem_b.collision_obj,
                            collision_request,
                            collision_result,
                        )
                    )
                    > 0
                ):
                    return 0.0, True
                distance_result = fcl.DistanceResult()
                distance = float(
                    fcl.distance(
                        elem_a.collision_obj,
                        elem_b.collision_obj,
                        distance_request,
                        distance_result,
                    )
                )
                min_distance = min(min_distance, distance)
        if min_distance == float("inf"):
            return 0.0, True
        return min_distance, False

    def allow_overlap(
        self,
        link_a: object,
        link_b: object,
        *,
        reason: str,
        elem_a: Optional[object] = None,
        elem_b: Optional[object] = None,
    ) -> None:
        """
        Allow a specific overlap with justification.

        By default, allowances apply at the link-pair level. If you name visual elements
        (via `part.visual(..., name=...)`), you can scope an allowance to a specific
        element pair by passing `elem_a`/`elem_b`.
        """

        a = _named_ref(link_a, kind="link_a")
        b = _named_ref(link_b, kind="link_b")
        r = (reason or "").strip()
        if not r:
            raise ValueError("allow_overlap requires a non-empty reason")
        key = (a, b) if a <= b else (b, a)
        self._allow_pairs[key] = r

        ea = None if elem_a is None else _named_ref(elem_a, kind="elem_a")
        eb = None if elem_b is None else _named_ref(elem_b, kind="elem_b")
        if ea or eb:
            self._allowances.append(
                f"allow_overlap({key[0]!r}, {key[1]!r}, elem_a={ea!r}, elem_b={eb!r}): {r}"
            )
        else:
            self._allowances.append(f"allow_overlap({key[0]!r}, {key[1]!r}): {r}")
        self._allow_elems.append((key, ea, eb, r))

    def allow_isolated_part(
        self,
        part: object,
        *,
        reason: str,
    ) -> None:
        """Allow a specific part to remain isolated in the checked pose."""

        part_name = _named_ref(part, kind="part")
        r = (reason or "").strip()
        if not r:
            raise ValueError("allow_isolated_part requires a non-empty reason")
        self._allow_isolated_parts[part_name] = r
        self._allowances.append(f"allow_isolated_part({part_name!r}): {r}")

    def allow_coplanar_surfaces(
        self,
        link_a: object,
        link_b: object,
        *,
        reason: str,
        elem_a: Optional[object] = None,
        elem_b: Optional[object] = None,
    ) -> None:
        """Allow a specific coplanar-surface finding with justification."""

        a = _named_ref(link_a, kind="link_a")
        b = _named_ref(link_b, kind="link_b")
        r = (reason or "").strip()
        if not r:
            raise ValueError("allow_coplanar_surfaces requires a non-empty reason")
        key = _pair_key(a, b)
        self._allow_coplanar_pairs[key] = r

        ea = None if elem_a is None else _named_ref(elem_a, kind="elem_a")
        eb = None if elem_b is None else _named_ref(elem_b, kind="elem_b")
        if ea or eb:
            self._allowances.append(
                f"allow_coplanar_surfaces({key[0]!r}, {key[1]!r}, elem_a={ea!r}, elem_b={eb!r}): {r}"
            )
        else:
            self._allowances.append(f"allow_coplanar_surfaces({key[0]!r}, {key[1]!r}): {r}")
        self._allow_coplanar_elems.append((key, ea, eb, r))

    @contextmanager
    def pose(
        self,
        joint_positions: Optional[Dict[object, object]] = None,
        **kwargs: object,
    ) -> Iterator[None]:
        prev = dict(self._pose)
        merged: Dict[str, PoseValue] = {}
        raw_joints = getattr(self.model, "articulations", None)
        if not isinstance(raw_joints, list):
            raw_joints = getattr(self.model, "joints", None)
        joint_lookup = {
            str(getattr(joint, "name")): joint
            for joint in list(raw_joints or [])
            if isinstance(getattr(joint, "name", None), str)
        }
        if joint_positions:
            for key, value in joint_positions.items():
                joint_name = _named_ref(key, kind="joint")
                joint = joint_lookup.get(joint_name)
                if joint is None:
                    raise ValidationError(f"Unknown joint: {joint_name!r}")
                merged[joint_name] = _coerce_joint_pose_value(joint, value)
        for key, value in kwargs.items():
            joint_name = str(key)
            joint = joint_lookup.get(joint_name)
            if joint is None:
                raise ValidationError(f"Unknown joint: {joint_name!r}")
            merged[joint_name] = _coerce_joint_pose_value(joint, value)
        self._pose = merged
        self._world_tfs_cache = None
        self._part_world_aabb_cache.clear()
        self._exact_world_elements_cache = None
        try:
            yield
        finally:
            self._pose = prev
            self._world_tfs_cache = None
            self._part_world_aabb_cache.clear()
            self._exact_world_elements_cache = None

    def _world_tfs(self) -> Dict[str, object]:
        if self._world_tfs_cache is None:
            self._world_tfs_cache = compute_part_world_transforms(self.model, dict(self._pose))
        return self._world_tfs_cache

    def part_world_position(self, part: object) -> Optional[Vec3]:
        part_name = _named_ref(part, kind="part")
        tf = self._world_tfs().get(part_name)
        if tf is None:
            return None
        try:
            return (float(tf[0][3]), float(tf[1][3]), float(tf[2][3]))  # type: ignore[index]
        except Exception:
            return None

    def link_world_position(self, link: object) -> Optional[Vec3]:
        return self.part_world_position(link)

    def part_world_aabb(self, part: object) -> Optional[AABB]:
        part_name = _named_ref(part, kind="part")
        if part_name in self._part_world_aabb_cache:
            return self._part_world_aabb_cache[part_name]

        part_obj = self._part_by_name(part_name)
        if part_obj is None:
            return None

        tf = self._world_tfs().get(part_name)
        if tf is None:
            return None
        world_aabb = part_world_aabb(
            part_obj,
            tf,  # type: ignore[arg-type]
            asset_root=self._asset_root(),
            prefer_collisions=False,
            _obj_cache=self._mesh_aabb_cache,
        )
        self._part_world_aabb_cache[part_name] = world_aabb
        return world_aabb

    def link_world_aabb(self, link: object) -> Optional[AABB]:
        return self.part_world_aabb(link)

    def part_element_world_aabb(
        self,
        part: object,
        *,
        elem: object,
    ) -> Optional[AABB]:
        part_name = _named_ref(part, kind="part")
        elem_name = _named_ref(elem, kind="elem")

        tf = self._world_tfs().get(part_name)
        if tf is None:
            return None

        for (
            candidate_name,
            _geometry_name,
            local_aabb,
        ) in self._part_geometry_items_with_local_aabbs(
            part_name,
        ):
            if candidate_name == elem_name:
                return _transform_aabb(local_aabb, tf)
        return None

    # ---- General checks ----------------------------------------------------

    def check_model_valid(self) -> bool:
        try:
            validate = getattr(self.model, "validate", None)
            if callable(validate):
                validate(strict=True)
                return self._record("check_model_valid", True)
            return self._record("check_model_valid", False, "model has no .validate()")
        except Exception as exc:
            return self._record("check_model_valid", False, f"{type(exc).__name__}: {exc}")

    def check_mesh_assets_ready(self) -> bool:
        links = getattr(self.model, "parts", None)
        if not isinstance(links, list):
            return self._record(
                "check_mesh_assets_ready", False, "model.parts missing or not a list"
            )
        missing: List[str] = []
        for link in links:
            for field_name in ("visuals",):
                elems = getattr(link, field_name, None)
                if not isinstance(elems, list):
                    continue
                for elem in elems:
                    geom = getattr(elem, "geometry", None)
                    filename = getattr(geom, "filename", None)
                    if filename is None:
                        continue
                    filename_text = str(filename)
                    if not filename_text:
                        continue
                    try:
                        resolved = resolve_mesh_path(
                            filename_text,
                            assets=self.asset_root or getattr(self.model, "assets", None),
                        )
                    except Exception:
                        resolved = None
                    if resolved is None or not resolved.exists():
                        missing.append(str(filename_text if resolved is None else resolved))
        if missing:
            preview = "\n".join(missing[:12])
            more = "" if len(missing) <= 12 else f"\n... ({len(missing) - 12} more)"
            return self._record(
                "check_mesh_assets_ready", False, f"Missing mesh assets:\n{preview}{more}"
            )
        return self._record("check_mesh_assets_ready", True)

    def check_mesh_files_exist(self) -> bool:
        self._warn_deprecated_helper(
            "check_mesh_files_exist()",
            "check_mesh_assets_ready()",
        )
        return self.check_mesh_assets_ready()

    def _check_articulation_origin_far_from_geometry_impl(
        self,
        *,
        tol: float,
        reason: Optional[str],
        check_name: str,
        warn_only: bool = False,
    ) -> bool:
        record = self._record_warning_check if warn_only else self._record
        tol_f = _normalize_joint_origin_tol(tol)
        if not math.isfinite(tol_f) or tol_f < 0.0:
            return record(check_name, False, "tol must be finite and non-negative")
        r = (reason or "").strip()
        if tol_f > _JOINT_ORIGIN_TOL_DEFAULT and r:
            self.warn(f"Relaxed articulation-origin tolerance in use: tol={tol_f:.4g}. Reason: {r}")

        findings = find_joint_origin_distance_findings(
            self.model,
            asset_root=self._asset_root(),
            tol=tol_f,
        )

        failures: List[str] = []
        for finding in findings:
            failures.append(
                f"joint={finding.joint!r} parent={finding.parent!r} child={finding.child!r} "
                f"dist_parent={finding.parent_distance:.4g} "
                f"dist_child={finding.child_distance:.4g} tol={finding.tol:.4g}"
            )

        if failures:
            preview = "\n".join(failures[:10])
            more = "" if len(failures) <= 10 else f"\n... ({len(failures) - 10} more)"
            return record(
                check_name,
                False,
                f"Articulation origin(s) far from geometry:\n{preview}{more}",
            )
        return record(check_name, True)

    def fail_if_articulation_origin_far_from_geometry(
        self,
        *,
        tol: float = _JOINT_ORIGIN_TOL_DEFAULT,
        reason: Optional[str] = None,
        name: Optional[str] = None,
    ) -> bool:
        tol_f = _normalize_joint_origin_tol(tol)
        return self._check_articulation_origin_far_from_geometry_impl(
            tol=tol_f,
            reason=reason,
            check_name=name or f"fail_if_articulation_origin_far_from_geometry(tol={tol_f:.4g})",
        )

    def warn_if_articulation_origin_far_from_geometry(
        self,
        *,
        tol: float = _JOINT_ORIGIN_TOL_DEFAULT,
        reason: Optional[str] = None,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_default_helper(
            "warn_if_articulation_origin_far_from_geometry(...)",
            "Use prompt-specific exact `expect_*` checks for mount/placement realism, "
            "and investigate any finding with `probe_model` before relaxing thresholds.",
        )
        tol_f = _normalize_joint_origin_tol(tol)
        return self._check_articulation_origin_far_from_geometry_impl(
            tol=tol_f,
            reason=reason,
            check_name=name or f"warn_if_articulation_origin_far_from_geometry(tol={tol_f:.4g})",
            warn_only=True,
        )

    def fail_if_part_contains_disconnected_geometry_islands(
        self,
        *,
        tol: float = 1e-6,
        name: Optional[str] = None,
    ) -> bool:
        return self._check_part_contains_disconnected_geometry_islands_impl(
            tol=tol,
            check_name=name,
            warn_only=False,
        )

    def warn_if_part_contains_disconnected_geometry_islands(
        self,
        *,
        tol: float = 1e-6,
        name: Optional[str] = None,
    ) -> bool:
        return self._check_part_contains_disconnected_geometry_islands_impl(
            tol=tol,
            check_name=name,
            warn_only=True,
            warn_prefix="warn_if_part_contains_disconnected_geometry_islands",
        )

    def _check_part_contains_disconnected_geometry_islands_impl(
        self,
        *,
        tol: float,
        check_name: Optional[str],
        warn_only: bool,
        warn_prefix: str = "warn_if_part_contains_disconnected_geometry_islands",
    ) -> bool:
        record = self._record_warning_check if warn_only else self._record
        if float(tol) < 0.0:
            prefix = (
                warn_prefix if warn_only else "fail_if_part_contains_disconnected_geometry_islands"
            )
            return record(
                check_name or f"{prefix}(tol={float(tol):.4g})",
                False,
                "tol must be non-negative",
            )

        findings = find_part_geometry_connectivity_findings(
            self.model,
            asset_root=self._asset_root(),
            contact_tol=float(tol),
        )

        failures: List[str] = []
        for finding in findings:
            disconnected = ", ".join(finding.disconnected)
            failures.append(
                f"part={finding.part!r} connected={finding.connected}/{finding.total} "
                f"disconnected=[{disconnected}]"
            )

        prefix = warn_prefix if warn_only else "fail_if_part_contains_disconnected_geometry_islands"
        resolved_name = check_name or f"{prefix}(tol={float(tol):.4g})"
        if failures:
            preview = "\n".join(failures[:10])
            more = "" if len(failures) <= 10 else f"\n... ({len(failures) - 10} more)"
            return record(
                resolved_name, False, f"Disconnected geometry islands detected:\n{preview}{more}"
            )
        return record(resolved_name, True)

    def fail_if_isolated_parts(
        self,
        *,
        max_pose_samples: int = 1,
        contact_tol: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        sample_count = int(max_pose_samples)
        if sample_count < 1:
            return self._record(
                name or "fail_if_isolated_parts(samples=0)",
                False,
                "max_pose_samples must be >= 1",
            )

        resolved_contact_tol = (
            default_contact_tol_from_env() if contact_tol is None else float(contact_tol)
        )
        if resolved_contact_tol < 0.0:
            return self._record(
                name
                or (
                    "fail_if_isolated_parts("
                    f"samples={sample_count},contact_tol={resolved_contact_tol:.4g})"
                ),
                False,
                "contact_tol must be non-negative",
            )

        if name is not None:
            resolved_name = name
        elif max_pose_samples == 1 and contact_tol is None:
            resolved_name = "fail_if_isolated_parts()"
        else:
            resolved_name = (
                "fail_if_isolated_parts("
                f"samples={sample_count},contact_tol={resolved_contact_tol:.4g})"
            )

        findings = find_unsupported_parts(
            self.model,
            asset_root=self._asset_root(),
            max_pose_samples=sample_count,
            contact_tol=resolved_contact_tol,
            seed=int(self.seed),
        )
        if not findings:
            return self._record(resolved_name, True)

        allowed_parts = set(self._allow_isolated_parts)
        allowed_findings = [
            finding
            for finding in findings
            if (member_parts := _unsupported_finding_parts(finding))
            and all(name in allowed_parts for name in member_parts)
        ]
        remaining_findings = [
            finding
            for finding in findings
            if not (
                (member_parts := _unsupported_finding_parts(finding))
                and all(name in allowed_parts for name in member_parts)
            )
        ]

        if allowed_findings:
            allowed_names = sorted(
                {
                    part_name
                    for finding in allowed_findings
                    for part_name in _unsupported_finding_parts(finding)
                }
            )
            preview = "\n".join(
                _format_unsupported_part_finding(finding) for finding in allowed_findings[:10]
            )
            more = (
                "" if len(allowed_findings) <= 10 else f"\n... ({len(allowed_findings) - 10} more)"
            )
            self.warn(
                "Isolated parts detected but allowed by justification: "
                f"{len(allowed_names)} part(s) [{', '.join(repr(name) for name in allowed_names)}].\n"
                f"{preview}{more}"
            )

        if not remaining_findings:
            return self._record(resolved_name, True)

        preview = "\n".join(
            _format_unsupported_part_finding(finding) for finding in remaining_findings[:10]
        )
        more = (
            "" if len(remaining_findings) <= 10 else f"\n... ({len(remaining_findings) - 10} more)"
        )
        return self._record(
            resolved_name,
            False,
            "Isolated parts detected "
            "(floating support-disconnected component groups from the grounded body; "
            f"samples={sample_count}, contact_tol={resolved_contact_tol:.4g}):\n"
            f"{preview}{more}",
        )

    def fail_if_parts_overlap_in_current_pose(
        self,
        *,
        overlap_tol: Optional[float] = None,
        overlap_volume_tol: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        resolved_overlap_tol = (
            default_overlap_tol_from_env() if overlap_tol is None else float(overlap_tol)
        )
        resolved_overlap_volume_tol = (
            default_overlap_volume_tol_from_env()
            if overlap_volume_tol is None
            else float(overlap_volume_tol)
        )
        if name is not None:
            resolved_name = name
        elif overlap_tol is None and overlap_volume_tol is None:
            resolved_name = "fail_if_parts_overlap_in_current_pose()"
        else:
            resolved_name = (
                "fail_if_parts_overlap_in_current_pose("
                f"overlap_tol={resolved_overlap_tol:.4g},"
                f"overlap_volume_tol={resolved_overlap_volume_tol:.4g})"
            )

        overlaps = find_geometry_overlaps_in_poses(
            self.model,
            asset_root=self._asset_root(),
            poses=[dict(self._pose)],
            overlap_tol=resolved_overlap_tol,
            overlap_volume_tol=resolved_overlap_volume_tol,
        )

        representative_by_pair: Dict[Tuple[str, str], GeometryOverlap] = {}
        allowed_pairs: Set[Tuple[str, str]] = set()
        allowed_overlap_count = 0

        for overlap in overlaps:
            key = _pair_key(overlap.link_a, overlap.link_b)
            if self._overlap_is_allowed(overlap):
                allowed_pairs.add(key)
                allowed_overlap_count += 1
                continue
            current = representative_by_pair.get(key)
            if current is None or _overlap_rank(overlap) > _overlap_rank(current):
                representative_by_pair[key] = overlap

        if representative_by_pair:
            details = []
            for key in sorted(representative_by_pair.keys()):
                overlap = representative_by_pair[key]
                details.append(
                    f"pair=({key[0]!r},{key[1]!r}) "
                    f"depth=({overlap.overlap_depth[0]:.4g},{overlap.overlap_depth[1]:.4g},{overlap.overlap_depth[2]:.4g}) "
                    f"min_depth={min(overlap.overlap_depth):.4g} vol={overlap.overlap_volume:.4g} "
                    f"elem_a={_format_overlap_element(overlap.elem_a, overlap.elem_a_name, overlap.elem_a_geometry)} "
                    f"elem_b={_format_overlap_element(overlap.elem_b, overlap.elem_b_name, overlap.elem_b_geometry)} "
                    f"pose={overlap.pose}"
                )

            preview = "\n".join(details[:8])
            more = "" if len(details) <= 8 else f"\n... ({len(details) - 8} more)"
            return self._record(
                resolved_name,
                False,
                "Part overlaps detected "
                f"(overlap_tol={resolved_overlap_tol:.4g}, "
                f"overlap_volume_tol={resolved_overlap_volume_tol:.4g}):\n"
                f"{preview}{more}",
            )

        if allowed_overlap_count > 0:
            self.warn(
                "Overlaps detected but allowed by justification: "
                f"{allowed_overlap_count} overlaps across {len(allowed_pairs)} part pair(s)."
            )
        return self._record(resolved_name, True)

    def fail_if_parts_overlap_in_sampled_poses(
        self,
        *,
        max_pose_samples: int = 128,
        overlap_tol: Optional[float] = None,
        overlap_volume_tol: Optional[float] = None,
        ignore_adjacent: bool = False,
        ignore_fixed: bool = True,
        name: Optional[str] = None,
    ) -> bool:
        return self._check_overlaps_impl(
            max_pose_samples=max_pose_samples,
            overlap_tol=overlap_tol,
            overlap_volume_tol=overlap_volume_tol,
            ignore_adjacent=ignore_adjacent,
            ignore_fixed=ignore_fixed,
            check_name=name,
            warn_only=False,
        )

    def fail_if_articulation_overlaps(
        self,
        *,
        max_pose_samples: int = 128,
        overlap_tol: Optional[float] = None,
        overlap_volume_tol: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        return self._check_overlaps_impl(
            max_pose_samples=max_pose_samples,
            overlap_tol=overlap_tol,
            overlap_volume_tol=overlap_volume_tol,
            ignore_adjacent=False,
            ignore_fixed=False,
            check_name=name,
            warn_only=False,
            pair_scope=self._articulation_overlap_pair_scope(),
            default_prefix="fail_if_articulation_overlaps",
        )

    def warn_if_articulation_overlaps(
        self,
        *,
        max_pose_samples: int = 128,
        overlap_tol: Optional[float] = None,
        overlap_volume_tol: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        return self._check_overlaps_impl(
            max_pose_samples=max_pose_samples,
            overlap_tol=overlap_tol,
            overlap_volume_tol=overlap_volume_tol,
            ignore_adjacent=False,
            ignore_fixed=False,
            check_name=name,
            warn_only=True,
            pair_scope=self._articulation_overlap_pair_scope(),
            default_prefix="warn_if_articulation_overlaps",
        )

    def warn_if_overlaps(
        self,
        *,
        max_pose_samples: int = 128,
        overlap_tol: Optional[float] = None,
        overlap_volume_tol: Optional[float] = None,
        ignore_adjacent: bool = False,
        ignore_fixed: bool = True,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_default_helper(
            "warn_if_overlaps(...)",
            "Use prompt-specific exact `expect_*` checks for attachment and clearance first; "
            "add this only when a broad overlap sensor answers a specific uncertainty.",
        )
        return self._check_overlaps_impl(
            max_pose_samples=max_pose_samples,
            overlap_tol=overlap_tol,
            overlap_volume_tol=overlap_volume_tol,
            ignore_adjacent=ignore_adjacent,
            ignore_fixed=ignore_fixed,
            check_name=name,
            warn_only=True,
        )

    def _relation_ignored_pairs(
        self,
        *,
        ignore_adjacent: bool,
        ignore_fixed: bool,
    ) -> List[Tuple[str, str]]:
        ignored_pairs: List[Tuple[str, str]] = []
        if ignore_adjacent or ignore_fixed:
            joints = getattr(self.model, "articulations", None)
            if isinstance(joints, list):
                for joint in joints:
                    parent = getattr(joint, "parent", None)
                    child = getattr(joint, "child", None)
                    if not isinstance(parent, str) or not isinstance(child, str):
                        continue
                    joint_type = getattr(joint, "articulation_type", None)
                    if ignore_adjacent or (ignore_fixed and joint_type == ArticulationType.FIXED):
                        ignored_pairs.append((parent, child))
        return ignored_pairs

    def _articulation_overlap_pair_scope(self) -> Set[Tuple[str, str]]:
        scoped_pairs: Set[Tuple[str, str]] = set()
        joints = getattr(self.model, "articulations", None)
        if not isinstance(joints, list):
            return scoped_pairs
        for joint in joints:
            parent = getattr(joint, "parent", None)
            child = getattr(joint, "child", None)
            if not isinstance(parent, str) or not isinstance(child, str):
                continue
            joint_type = getattr(joint, "articulation_type", None)
            if joint_type not in {
                ArticulationType.REVOLUTE,
                ArticulationType.PRISMATIC,
                ArticulationType.CONTINUOUS,
            }:
                continue
            scoped_pairs.add(_pair_key(parent, child))
        return scoped_pairs

    def _check_overlaps_impl(
        self,
        *,
        max_pose_samples: int,
        overlap_tol: Optional[float],
        overlap_volume_tol: Optional[float],
        ignore_adjacent: bool,
        ignore_fixed: bool,
        check_name: Optional[str],
        warn_only: bool,
        pair_scope: Optional[Set[Tuple[str, str]]] = None,
        default_prefix: Optional[str] = None,
    ) -> bool:
        prefix = default_prefix or (
            "warn_if_overlaps" if warn_only else "fail_if_parts_overlap_in_sampled_poses"
        )
        if check_name is not None:
            resolved_name = check_name
        elif pair_scope is not None:
            resolved_name = f"{prefix}(samples={int(max_pose_samples)})"
        else:
            resolved_name = (
                f"{prefix}("
                f"samples={int(max_pose_samples)},"
                f"ignore_adjacent={bool(ignore_adjacent)},"
                f"ignore_fixed={bool(ignore_fixed)})"
            )
        record = self._record_warning_check if warn_only else self._record
        allowed_pairs = self._relation_ignored_pairs(
            ignore_adjacent=ignore_adjacent,
            ignore_fixed=ignore_fixed,
        )
        resolved_overlap_tol = (
            default_overlap_tol_from_env() if overlap_tol is None else float(overlap_tol)
        )
        resolved_overlap_volume_tol = (
            default_overlap_volume_tol_from_env()
            if overlap_volume_tol is None
            else float(overlap_volume_tol)
        )

        pair_relations: Dict[Tuple[str, str], str] = {}
        joints = getattr(self.model, "articulations", None)
        if isinstance(joints, list):
            for joint in joints:
                parent = getattr(joint, "parent", None)
                child = getattr(joint, "child", None)
                if not isinstance(parent, str) or not isinstance(child, str):
                    continue
                joint_type = getattr(joint, "articulation_type", None)
                key = _pair_key(parent, child)
                if isinstance(joint_type, ArticulationType):
                    pair_relations[key] = f"adjacent-{joint_type.value.lower()}"
                else:
                    pair_relations[key] = "adjacent"

        overlaps = find_geometry_overlaps(
            self.model,
            asset_root=self._asset_root(),
            max_pose_samples=int(max_pose_samples),
            overlap_tol=resolved_overlap_tol,
            overlap_volume_tol=resolved_overlap_volume_tol,
            allowed_pairs=allowed_pairs or None,
            seed=int(self.seed),
        )

        relevant_overlaps = overlaps
        if pair_scope is not None:
            relevant_overlaps = [
                overlap
                for overlap in overlaps
                if _pair_key(overlap.link_a, overlap.link_b) in pair_scope
            ]

        remaining: List[str] = []
        for o in relevant_overlaps:
            key = _pair_key(o.link_a, o.link_b)
            if self._overlap_is_allowed(o):
                continue
            relation = pair_relations.get(key, "unrelated")
            remaining.append(
                f"relation={relation} pair=({o.link_a!r},{o.link_b!r}) pose_index={o.pose_index} "
                f"depth=({o.overlap_depth[0]:.4g},{o.overlap_depth[1]:.4g},{o.overlap_depth[2]:.4g}) "
                f"min_depth={min(o.overlap_depth):.4g} vol={o.overlap_volume:.4g} "
                f"elem_a={_format_overlap_element(o.elem_a, o.elem_a_name, o.elem_a_geometry)} "
                f"elem_b={_format_overlap_element(o.elem_b, o.elem_b_name, o.elem_b_geometry)} "
                f"pose={o.pose}"
            )

        if remaining:
            preview = "\n".join(remaining[:8])
            more = "" if len(remaining) <= 8 else f"\n... ({len(remaining) - 8} more)"
            return record(
                resolved_name,
                False,
                "Overlaps detected "
                f"(overlap_tol={resolved_overlap_tol:.4g}, "
                f"overlap_volume_tol={resolved_overlap_volume_tol:.4g}):\n"
                f"{preview}{more}",
            )

        if relevant_overlaps and (self._allow_pairs or self._allow_elems):
            self.warn(
                f"Overlaps detected but allowed by justification: {len(relevant_overlaps)} overlaps."
            )
        return record(resolved_name, True)

    def warn_if_coplanar_surfaces(
        self,
        *,
        max_pose_samples: int = 32,
        plane_tol: float = 0.001,
        min_overlap: float = 0.02,
        min_overlap_ratio: float = 0.35,
        ignore_adjacent: bool = True,
        ignore_fixed: bool = True,
        name: Optional[str] = None,
    ) -> bool:
        resolved_name = name or (
            f"warn_if_coplanar_surfaces("
            f"samples={int(max_pose_samples)},"
            f"plane_tol={float(plane_tol):.4g},"
            f"min_overlap={float(min_overlap):.4g},"
            f"min_overlap_ratio={float(min_overlap_ratio):.4g},"
            f"ignore_adjacent={bool(ignore_adjacent)},"
            f"ignore_fixed={bool(ignore_fixed)})"
        )
        record = self._record_warning_check

        if int(max_pose_samples) <= 0:
            return record(resolved_name, False, "max_pose_samples must be positive")
        plane_tol_f = float(plane_tol)
        if not math.isfinite(plane_tol_f) or plane_tol_f < 0.0:
            return record(resolved_name, False, "plane_tol must be finite and non-negative")
        min_overlap_f = float(min_overlap)
        if not math.isfinite(min_overlap_f) or min_overlap_f < 0.0:
            return record(resolved_name, False, "min_overlap must be finite and non-negative")
        min_overlap_ratio_f = float(min_overlap_ratio)
        if (
            not math.isfinite(min_overlap_ratio_f)
            or min_overlap_ratio_f < 0.0
            or min_overlap_ratio_f > 1.0
        ):
            return record(
                resolved_name,
                False,
                "min_overlap_ratio must be finite and between 0 and 1",
            )

        links = getattr(self.model, "parts", None)
        joints = getattr(self.model, "articulations", None)
        if not isinstance(links, list) or not isinstance(joints, list):
            return record(resolved_name, False, "model.parts/articulations missing")

        pair_relations: Dict[Tuple[str, str], str] = {}
        ignored_pairs: set[Tuple[str, str]] = set()
        for joint in joints:
            parent = getattr(joint, "parent", None)
            child = getattr(joint, "child", None)
            if not isinstance(parent, str) or not isinstance(child, str):
                continue
            joint_type = getattr(joint, "articulation_type", None)
            key = _pair_key(parent, child)
            if isinstance(joint_type, ArticulationType):
                relation = f"adjacent-{joint_type.value.lower()}"
            else:
                relation = "adjacent"
            pair_relations[key] = relation
            if ignore_adjacent or (ignore_fixed and joint_type == ArticulationType.FIXED):
                ignored_pairs.add(key)

        def is_coplanar_allowed(finding: _CoplanarFinding) -> bool:
            key = _pair_key(finding.link_a, finding.link_b)
            if key not in self._allow_coplanar_pairs:
                return False
            for allowed_key, elem_a, elem_b, _reason in self._allow_coplanar_elems:
                if allowed_key != key:
                    continue
                if elem_a is None and elem_b is None:
                    return True
                names = (finding.elem_a_name, finding.elem_b_name)
                if (elem_a in names) and (elem_b in names):
                    return True
            return False

        risk_rank = {"low": 0, "medium": 1, "high": 2}

        def classify_risk(
            *,
            relation: str,
            plane_delta: float,
            overlap_ratio: float,
            thin_pair: bool,
        ) -> str:
            if relation.startswith("adjacent-"):
                return "low"
            strong_plane = plane_delta <= max(plane_tol_f * 0.25, 1e-5)
            strong_overlap = overlap_ratio >= max(min_overlap_ratio_f + 0.25, 0.75)
            moderate_overlap = overlap_ratio >= max(min_overlap_ratio_f + 0.1, 0.5)
            if strong_plane and strong_overlap and thin_pair:
                return "high"
            if (strong_plane and moderate_overlap) or (strong_overlap and thin_pair):
                return "medium"
            return "low"

        def better_finding(candidate: _CoplanarFinding, current: _CoplanarFinding) -> bool:
            if risk_rank[candidate.risk] != risk_rank[current.risk]:
                return risk_rank[candidate.risk] > risk_rank[current.risk]
            if abs(candidate.plane_delta - current.plane_delta) > 1e-9:
                return candidate.plane_delta < current.plane_delta
            if abs(candidate.overlap_ratio - current.overlap_ratio) > 1e-9:
                return candidate.overlap_ratio > current.overlap_ratio
            return candidate.area > current.area

        try:
            poses = generate_pose_samples(
                self.model,
                max_samples=int(max_pose_samples),
                seed=int(self.seed),
            )
        except Exception as exc:
            return record(resolved_name, False, str(exc))

        findings_by_pair: Dict[Tuple[str, str], _CoplanarFinding] = {}
        for pose_index, pose in enumerate(poses):
            world_tfs = compute_part_world_transforms(self.model, dict(pose))
            part_elem_bounds: Dict[str, List[Tuple[Optional[str], Optional[str], AABB]]] = {}
            for link in links:
                link_name = getattr(link, "name", None)
                if not isinstance(link_name, str):
                    continue
                world_tf = world_tfs.get(link_name)
                if world_tf is None:
                    continue
                elem_bounds = self._part_geometry_items_with_local_aabbs(link_name)
                if not elem_bounds:
                    continue
                part_elem_bounds[link_name] = [
                    (elem_name, elem_geometry, _transform_aabb(local_aabb, world_tf))
                    for elem_name, elem_geometry, local_aabb in elem_bounds
                ]

            names = sorted(part_elem_bounds.keys())
            for i in range(len(names)):
                for j in range(i + 1, len(names)):
                    link_a = names[i]
                    link_b = names[j]
                    key = _pair_key(link_a, link_b)
                    if key in ignored_pairs:
                        continue
                    relation = pair_relations.get(key, "unrelated")
                    best_pair: Optional[_CoplanarFinding] = None
                    for elem_a_name, elem_a_geometry, aabb_a in part_elem_bounds[link_a]:
                        for elem_b_name, elem_b_geometry, aabb_b in part_elem_bounds[link_b]:
                            for axis_name in ("x", "y", "z"):
                                orth_axes = tuple(
                                    axis for axis in ("x", "y", "z") if axis != axis_name
                                )
                                overlap_0 = _aabb_axis_overlap(aabb_a, aabb_b, axis=orth_axes[0])
                                overlap_1 = _aabb_axis_overlap(aabb_a, aabb_b, axis=orth_axes[1])
                                if overlap_0 < min_overlap_f or overlap_1 < min_overlap_f:
                                    continue

                                axis_idx = _axis_index(axis_name)
                                face_pairs = (
                                    (
                                        "min",
                                        "min",
                                        abs(
                                            float(aabb_a[0][axis_idx]) - float(aabb_b[0][axis_idx])
                                        ),
                                    ),
                                    (
                                        "min",
                                        "max",
                                        abs(
                                            float(aabb_a[0][axis_idx]) - float(aabb_b[1][axis_idx])
                                        ),
                                    ),
                                    (
                                        "max",
                                        "min",
                                        abs(
                                            float(aabb_a[1][axis_idx]) - float(aabb_b[0][axis_idx])
                                        ),
                                    ),
                                    (
                                        "max",
                                        "max",
                                        abs(
                                            float(aabb_a[1][axis_idx]) - float(aabb_b[1][axis_idx])
                                        ),
                                    ),
                                )
                                face_a, face_b, plane_delta = min(
                                    face_pairs, key=lambda item: item[2]
                                )
                                if plane_delta > plane_tol_f:
                                    continue

                                area = overlap_0 * overlap_1
                                face_area_a = _aabb_axis_span(
                                    aabb_a, axis=orth_axes[0]
                                ) * _aabb_axis_span(aabb_a, axis=orth_axes[1])
                                face_area_b = _aabb_axis_span(
                                    aabb_b, axis=orth_axes[0]
                                ) * _aabb_axis_span(aabb_b, axis=orth_axes[1])
                                smaller_face_area = max(min(face_area_a, face_area_b), 1e-9)
                                overlap_ratio = area / smaller_face_area
                                if overlap_ratio < min_overlap_ratio_f:
                                    continue

                                thickness_a = _aabb_axis_span(aabb_a, axis=axis_name)
                                thickness_b = _aabb_axis_span(aabb_b, axis=axis_name)
                                in_plane_min_a = max(
                                    min(
                                        _aabb_axis_span(aabb_a, axis=orth_axes[0]),
                                        _aabb_axis_span(aabb_a, axis=orth_axes[1]),
                                    ),
                                    1e-9,
                                )
                                in_plane_min_b = max(
                                    min(
                                        _aabb_axis_span(aabb_b, axis=orth_axes[0]),
                                        _aabb_axis_span(aabb_b, axis=orth_axes[1]),
                                    ),
                                    1e-9,
                                )
                                thin_pair = (
                                    thickness_a / in_plane_min_a <= 0.12
                                    and thickness_b / in_plane_min_b <= 0.12
                                )
                                candidate = _CoplanarFinding(
                                    link_a=link_a,
                                    link_b=link_b,
                                    relation=relation,
                                    risk=classify_risk(
                                        relation=relation,
                                        plane_delta=plane_delta,
                                        overlap_ratio=overlap_ratio,
                                        thin_pair=thin_pair,
                                    ),
                                    pose_index=pose_index,
                                    pose=dict(pose),
                                    axis_name=axis_name,
                                    overlap_axis_0=orth_axes[0],
                                    overlap_axis_1=orth_axes[1],
                                    face_a=face_a,
                                    face_b=face_b,
                                    plane_delta=plane_delta,
                                    overlap_0=overlap_0,
                                    overlap_1=overlap_1,
                                    area=area,
                                    overlap_ratio=overlap_ratio,
                                    thin_pair=thin_pair,
                                    elem_a_name=elem_a_name,
                                    elem_a_geometry=elem_a_geometry,
                                    elem_b_name=elem_b_name,
                                    elem_b_geometry=elem_b_geometry,
                                )
                                if best_pair is None or better_finding(candidate, best_pair):
                                    best_pair = candidate

                    if best_pair is None:
                        continue
                    if is_coplanar_allowed(best_pair):
                        continue
                    current = findings_by_pair.get(key)
                    if current is None or better_finding(best_pair, current):
                        findings_by_pair[key] = best_pair

        if findings_by_pair:
            max_risk = max(findings_by_pair.values(), key=lambda item: risk_rank[item.risk]).risk
            headline = (
                "Low-confidence coplanar-surface hints detected"
                if max_risk == "low"
                else "Coplanar or nearly coplanar surfaces detected"
            )
            findings = [
                f"risk={candidate.risk} relation={candidate.relation} "
                f"pair=({link_a!r},{link_b!r}) pose_index={candidate.pose_index} "
                f"axis={candidate.axis_name} faces=({candidate.face_a},{candidate.face_b}) "
                f"plane_delta={candidate.plane_delta:.4g} "
                f"{candidate.overlap_axis_0}_overlap={candidate.overlap_0:.4g} "
                f"{candidate.overlap_axis_1}_overlap={candidate.overlap_1:.4g} "
                f"overlap_ratio={candidate.overlap_ratio:.4g} thin_pair={candidate.thin_pair} "
                f"elem_a={candidate.elem_a_name!r}:{candidate.elem_a_geometry or '?'} "
                f"elem_b={candidate.elem_b_name!r}:{candidate.elem_b_geometry or '?'} "
                f"pose={candidate.pose}"
                for (link_a, link_b), candidate in sorted(findings_by_pair.items())
            ]
            preview = "\n".join(findings[:8])
            more = "" if len(findings) <= 8 else f"\n... ({len(findings) - 8} more)"
            return record(
                resolved_name,
                False,
                f"{headline} "
                f"(max_risk={max_risk}; warning-tier heuristic; adjacent flush mounts can be intentional):\n"
                f"{preview}{more}",
            )
        return record(resolved_name, True)

    # ---- Intent checks (pose-aware) ----------------------------------------

    def expect_origin_distance(
        self,
        link_a: object,
        link_b: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        min_dist: float = 0.0,
        max_dist: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_origin_distance({link_a_name},{link_b_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)
        min_dist_f = float(min_dist)
        if min_dist_f < 0.0:
            return self._record(check_name, False, "min_dist must be >= 0")
        max_dist_f = None if max_dist is None else float(max_dist)
        if max_dist_f is not None and max_dist_f < min_dist_f:
            return self._record(check_name, False, "max_dist must be >= min_dist")

        pa = self.link_world_position(link_a)
        pb = self.link_world_position(link_b)
        if pa is None or pb is None:
            return self._record(check_name, False, "missing link world position(s)")

        dist = _point_distance_on_axes(pa, pb, axes=axes_key)
        ok = dist >= min_dist_f
        if max_dist_f is not None:
            ok = ok and dist <= max_dist_f
        upper_txt = "inf" if max_dist_f is None else f"{max_dist_f:.4g}"
        return self._record(
            check_name,
            ok,
            f"origin_dist[{_axes_label(axes_key)}]({link_a_name!r},{link_b_name!r})={dist:.4g} "
            f"min_dist={min_dist_f:.4g} max_dist={upper_txt} pose={self._pose}",
        )

    def expect_origin_gap(
        self,
        positive_link: object,
        negative_link: object,
        *,
        axis: str,
        min_gap: float = 0.0,
        max_gap: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        positive_name = _named_ref(positive_link, kind="positive_link")
        negative_name = _named_ref(negative_link, kind="negative_link")
        axis_key, axis_sign, axis_err = _normalize_axis_name(axis)
        check_name = (
            name or f"expect_origin_gap({positive_name},{negative_name},axis={axis_key or axis})"
        )
        if axis_err is not None or axis_key is None or axis_sign < 0.0:
            return self._record(check_name, False, axis_err or "axis must be one of: x, y, z")
        min_gap_f = float(min_gap)
        max_gap_f = None if max_gap is None else float(max_gap)
        if max_gap_f is not None and max_gap_f < min_gap_f:
            return self._record(check_name, False, "max_gap must be >= min_gap")

        positive_pos = self.link_world_position(positive_link)
        negative_pos = self.link_world_position(negative_link)
        if positive_pos is None or negative_pos is None:
            return self._record(check_name, False, "missing link world position(s)")

        idx = _axis_index(axis_key)
        gap = float(positive_pos[idx]) - float(negative_pos[idx])
        ok = gap >= min_gap_f
        if max_gap_f is not None:
            ok = ok and gap <= max_gap_f
        upper_txt = "inf" if max_gap_f is None else f"{max_gap_f:.4g}"
        return self._record(
            check_name,
            ok,
            f"origin_gap_{axis_key}={gap:.4g} min_gap={min_gap_f:.4g} max_gap={upper_txt} pose={self._pose}",
        )

    def expect_contact(
        self,
        link_a: object,
        link_b: object,
        *,
        contact_tol: float = 1e-6,
        elem_a: Optional[object] = None,
        elem_b: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        check_name = name or f"expect_contact({link_a_name},{link_b_name})"
        contact_tol_f = float(contact_tol)
        if contact_tol_f < 0.0:
            return self._record(check_name, False, "contact_tol must be >= 0")

        elements_a, _resolved_a, elem_a_name, error_a = self._resolve_exact_elements(
            link_a,
            elem=elem_a,
            kind_prefix="elem_a",
        )
        elements_b, _resolved_b, elem_b_name, error_b = self._resolve_exact_elements(
            link_b,
            elem=elem_b,
            kind_prefix="elem_b",
        )
        if error_a or error_b or elements_a is None or elements_b is None:
            errors = [item for item in (error_a, error_b) if item]
            return self._record(check_name, False, "; ".join(errors))

        min_distance, collided = self._exact_pair_distance(elements_a, elements_b)
        ok = collided or min_distance <= contact_tol_f
        return self._record(
            check_name,
            ok,
            f"min_distance={min_distance:.4g} contact_tol={contact_tol_f:.4g} "
            f"elem_a={elem_a_name!r} elem_b={elem_b_name!r} pose={self._pose}",
        )

    def expect_gap(
        self,
        positive_link: object,
        negative_link: object,
        *,
        axis: str,
        min_gap: Optional[float] = None,
        max_gap: Optional[float] = None,
        max_penetration: Optional[float] = None,
        positive_elem: Optional[object] = None,
        negative_elem: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        positive_name = _named_ref(positive_link, kind="positive_link")
        negative_name = _named_ref(negative_link, kind="negative_link")
        axis_key, axis_sign, axis_err = _normalize_axis_name(axis)
        check_name = name or f"expect_gap({positive_name},{negative_name},axis={axis_key or axis})"
        if axis_err is not None or axis_key is None or axis_sign < 0.0:
            return self._record(check_name, False, axis_err or "axis must be one of: x, y, z")

        positive_elements, _resolved_positive, positive_elem_name, positive_error = (
            self._resolve_exact_elements(
                positive_link,
                elem=positive_elem,
                kind_prefix="positive",
            )
        )
        negative_elements, _resolved_negative, negative_elem_name, negative_error = (
            self._resolve_exact_elements(
                negative_link,
                elem=negative_elem,
                kind_prefix="negative",
            )
        )
        if (
            positive_error
            or negative_error
            or positive_elements is None
            or negative_elements is None
        ):
            errors = [item for item in (positive_error, negative_error) if item]
            return self._record(check_name, False, "; ".join(errors))

        positive_min, _positive_max = self._elements_projection_interval(
            positive_elements,
            axis=axis_key,
        )
        _negative_min, negative_max = self._elements_projection_interval(
            negative_elements,
            axis=axis_key,
        )

        if min_gap is None:
            max_penetration_f = 0.0 if max_penetration is None else float(max_penetration)
            min_gap_f = -max_penetration_f
        else:
            min_gap_f = float(min_gap)
            if max_penetration is not None:
                max_penetration_f = float(max_penetration)
                expected_min_gap = -max_penetration_f
                if abs(expected_min_gap - min_gap_f) > 1e-9:
                    self.warn(
                        "expect_gap received both min_gap and max_penetration with different bounds; "
                        "using min_gap as the lower bound."
                    )
            else:
                max_penetration_f = max(0.0, -min_gap_f)
        max_gap_f = None if max_gap is None else float(max_gap)
        if max_gap_f is not None and max_gap_f < min_gap_f:
            return self._record(check_name, False, "max_gap must be >= min_gap")

        gap = positive_min - negative_max
        ok = gap >= min_gap_f
        if max_gap_f is not None:
            ok = ok and gap <= max_gap_f
        upper_txt = "inf" if max_gap_f is None else f"{max_gap_f:.4g}"
        return self._record(
            check_name,
            ok,
            f"gap_{axis_key}={gap:.4g} min_gap={min_gap_f:.4g} max_gap={upper_txt} "
            f"max_penetration={max_penetration_f:.4g} "
            f"positive_elem={positive_elem_name!r} negative_elem={negative_elem_name!r} "
            f"pose={self._pose}",
        )

    def expect_overlap(
        self,
        link_a: object,
        link_b: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        min_overlap: float = 0.0,
        elem_a: Optional[object] = None,
        elem_b: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_overlap({link_a_name},{link_b_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        elements_a, _resolved_a, elem_a_name, error_a = self._resolve_exact_elements(
            link_a,
            elem=elem_a,
            kind_prefix="elem_a",
        )
        elements_b, _resolved_b, elem_b_name, error_b = self._resolve_exact_elements(
            link_b,
            elem=elem_b,
            kind_prefix="elem_b",
        )
        if error_a or error_b or elements_a is None or elements_b is None:
            errors = [item for item in (error_a, error_b) if item]
            return self._record(check_name, False, "; ".join(errors))

        min_overlap_f = float(min_overlap)
        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            min_a, max_a = self._elements_projection_interval(elements_a, axis=axis_name)
            min_b, max_b = self._elements_projection_interval(elements_b, axis=axis_name)
            overlap = min(max_a, max_b) - max(min_a, min_b)
            axis_ok = overlap >= min_overlap_f
            ok = ok and axis_ok
            axis_details.append(f"overlap_{axis_name}={overlap:.4g}")

        return self._record(
            check_name,
            ok,
            " ".join(axis_details)
            + f" min_overlap={min_overlap_f:.4g} elem_a={elem_a_name!r} elem_b={elem_b_name!r} pose={self._pose}",
        )

    def expect_within(
        self,
        inner_link: object,
        outer_link: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        margin: float = 0.0,
        inner_elem: Optional[object] = None,
        outer_elem: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        inner_name = _named_ref(inner_link, kind="inner_link")
        outer_name = _named_ref(outer_link, kind="outer_link")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name or f"expect_within({inner_name},{outer_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        inner_elements, _resolved_inner, inner_elem_name, inner_error = (
            self._resolve_exact_elements(
                inner_link,
                elem=inner_elem,
                kind_prefix="inner",
            )
        )
        outer_elements, _resolved_outer, outer_elem_name, outer_error = (
            self._resolve_exact_elements(
                outer_link,
                elem=outer_elem,
                kind_prefix="outer",
            )
        )
        if inner_error or outer_error or inner_elements is None or outer_elements is None:
            errors = [item for item in (inner_error, outer_error) if item]
            return self._record(check_name, False, "; ".join(errors))

        margin_f = float(margin)
        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            inner_min, inner_max = self._elements_projection_interval(
                inner_elements, axis=axis_name
            )
            outer_min, outer_max = self._elements_projection_interval(
                outer_elements, axis=axis_name
            )
            axis_ok = inner_min >= outer_min - margin_f and inner_max <= outer_max + margin_f
            ok = ok and axis_ok
            axis_details.append(
                f"{axis_name}=({inner_min:.4g},{inner_max:.4g}) in ({outer_min:.4g},{outer_max:.4g})"
            )

        return self._record(
            check_name,
            ok,
            f"inner={inner_name!r} outer={outer_name!r} axes={_axes_label(axes_key)} "
            f"margin={margin_f:.4g} inner_elem={inner_elem_name!r} outer_elem={outer_elem_name!r} "
            + " ".join(axis_details)
            + f" pose={self._pose}",
        )

    def expect_aabb_within(
        self,
        inner_link: object,
        outer_link: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        margin: float = 0.0,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper("expect_aabb_within(...)", "expect_within(...)")
        inner_name = _named_ref(inner_link, kind="inner_link")
        outer_name = _named_ref(outer_link, kind="outer_link")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_aabb_within({inner_name},{outer_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        inner = self.link_world_aabb(inner_link)
        outer = self.link_world_aabb(outer_link)
        if inner is None or outer is None:
            return self._record(check_name, False, "missing link AABB(s)")

        margin_f = float(margin)
        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            idx = _axis_index(axis_name)
            inner_min = float(inner[0][idx])
            inner_max = float(inner[1][idx])
            outer_min = float(outer[0][idx])
            outer_max = float(outer[1][idx])
            axis_ok = inner_min >= outer_min - margin_f and inner_max <= outer_max + margin_f
            ok = ok and axis_ok
            axis_details.append(
                f"{axis_name}=({inner_min:.4g},{inner_max:.4g}) in ({outer_min:.4g},{outer_max:.4g})"
            )

        return self._record(
            check_name,
            ok,
            f"inner={inner_name!r} outer={outer_name!r} axes={_axes_label(axes_key)} margin={margin_f:.4g} "
            + " ".join(axis_details)
            + f" pose={self._pose}",
        )

    def expect_aabb_gap(
        self,
        positive_link: object,
        negative_link: object,
        *,
        axis: str,
        min_gap: Optional[float] = None,
        max_gap: Optional[float] = None,
        max_penetration: Optional[float] = None,
        positive_elem: Optional[object] = None,
        negative_elem: Optional[object] = None,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper("expect_aabb_gap(...)", "expect_gap(...)")
        """
        Check the signed directional gap between two links or two named elements.

        The measured gap is:

            positive.min[axis] - negative.max[axis]

        along the requested positive world axis. This makes the check directional:
        `expect_aabb_gap("lid", "base", axis="z")` means "how far is the bottom
        of the lid above the top of the base?", while swapping the arguments asks
        a different question and flips the sign convention.

        Parameters:
          - positive_link / negative_link: the objects on the positive-side and
            negative-side of the tested axis
          - axis: one of "x", "y", or "z"; must be the positive axis direction
          - min_gap: lower bound on the signed gap; when omitted, derive it from
            `max_penetration`
          - max_gap: upper bound on the signed gap
          - max_penetration: convenience way to express the allowed overlap depth;
            this becomes a lower bound of `-max_penetration`
          - positive_elem / negative_elem: optional named geometry items within
            each link; when provided, the check uses those element AABBs instead
            of the whole-link union AABBs
          - name: optional explicit check name for reporting
        """
        positive_name = _named_ref(positive_link, kind="positive_link")
        negative_name = _named_ref(negative_link, kind="negative_link")
        axis_key, axis_sign, axis_err = _normalize_axis_name(axis)
        check_name = (
            name or f"expect_aabb_gap({positive_name},{negative_name},axis={axis_key or axis})"
        )
        if axis_err is not None or axis_key is None or axis_sign < 0.0:
            return self._record(check_name, False, axis_err or "axis must be one of: x, y, z")

        positive_elem_name = (
            None if positive_elem is None else _named_ref(positive_elem, kind="positive_elem")
        )
        negative_elem_name = (
            None if negative_elem is None else _named_ref(negative_elem, kind="negative_elem")
        )

        positive_aabb = (
            self.link_world_aabb(positive_link)
            if positive_elem_name is None
            else self.part_element_world_aabb(
                positive_link,
                elem=positive_elem_name,
            )
        )
        negative_aabb = (
            self.link_world_aabb(negative_link)
            if negative_elem_name is None
            else self.part_element_world_aabb(
                negative_link,
                elem=negative_elem_name,
            )
        )
        if positive_aabb is None or negative_aabb is None:
            missing: list[str] = []
            if positive_aabb is None:
                if positive_elem_name is None:
                    missing.append(f"missing link AABB for {positive_name!r}")
                else:
                    missing.append(
                        f"missing element AABB for positive_elem={positive_elem_name!r} on {positive_name!r}"
                    )
            if negative_aabb is None:
                if negative_elem_name is None:
                    missing.append(f"missing link AABB for {negative_name!r}")
                else:
                    missing.append(
                        f"missing element AABB for negative_elem={negative_elem_name!r} on {negative_name!r}"
                    )
            return self._record(check_name, False, "; ".join(missing))

        if min_gap is None:
            max_penetration_f = 0.0 if max_penetration is None else float(max_penetration)
            min_gap_f = -max_penetration_f
        else:
            min_gap_f = float(min_gap)
            if max_penetration is not None:
                max_penetration_f = float(max_penetration)
                expected_min_gap = -max_penetration_f
                if abs(expected_min_gap - min_gap_f) > 1e-9:
                    self.warn(
                        "expect_aabb_gap received both min_gap and max_penetration with different bounds; "
                        "using min_gap as the lower bound."
                    )
            else:
                max_penetration_f = max(0.0, -min_gap_f)
        max_gap_f = None if max_gap is None else float(max_gap)
        if max_gap_f is not None and max_gap_f < min_gap_f:
            return self._record(check_name, False, "max_gap must be >= min_gap")

        gap = _aabb_axis_gap(positive_aabb, negative_aabb, axis=axis_key)
        ok = gap >= min_gap_f
        if max_gap_f is not None:
            ok = ok and gap <= max_gap_f
        upper_txt = "inf" if max_gap_f is None else f"{max_gap_f:.4g}"
        return self._record(
            check_name,
            ok,
            f"gap_{axis_key}={gap:.4g} min_gap={min_gap_f:.4g} max_gap={upper_txt} "
            f"max_penetration={max_penetration_f:.4g} "
            f"positive_elem={positive_elem_name!r} negative_elem={negative_elem_name!r} "
            f"pose={self._pose}",
        )

    def expect_aabb_overlap(
        self,
        link_a: object,
        link_b: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        min_overlap: float = 0.0,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper("expect_aabb_overlap(...)", "expect_overlap(...)")
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_aabb_overlap({link_a_name},{link_b_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        aabb_a = self.link_world_aabb(link_a)
        aabb_b = self.link_world_aabb(link_b)
        if aabb_a is None or aabb_b is None:
            return self._record(check_name, False, "missing link AABB(s)")

        min_overlap_f = float(min_overlap)
        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            overlap = _aabb_axis_overlap(aabb_a, aabb_b, axis=axis_name)
            axis_ok = overlap >= min_overlap_f
            ok = ok and axis_ok
            axis_details.append(f"overlap_{axis_name}={overlap:.4g}")

        return self._record(
            check_name,
            ok,
            " ".join(axis_details) + f" min_overlap={min_overlap_f:.4g} pose={self._pose}",
        )

    def expect_aabb_contact(
        self,
        link_a: object,
        link_b: object,
        *,
        axes: Union[str, Sequence[str]] = "xyz",
        contact_tol: float = 0.0,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper("expect_aabb_contact(...)", "expect_contact(...)")
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_aabb_contact({link_a_name},{link_b_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        contact_tol_f = float(contact_tol)
        if contact_tol_f < 0.0:
            return self._record(check_name, False, "contact_tol must be >= 0")

        aabb_a = self.link_world_aabb(link_a)
        aabb_b = self.link_world_aabb(link_b)
        if aabb_a is None or aabb_b is None:
            return self._record(check_name, False, "missing link AABB(s)")

        ok = True
        axis_details: list[str] = []
        for axis_name in axes_key:
            sep = _aabb_axis_separation(aabb_a, aabb_b, axis=axis_name)
            axis_ok = sep <= contact_tol_f
            ok = ok and axis_ok
            axis_details.append(f"sep_{axis_name}={sep:.4g}")

        return self._record(
            check_name,
            ok,
            " ".join(axis_details) + f" contact_tol={contact_tol_f:.4g} pose={self._pose}",
        )

    def expect_joint_motion_axis(
        self,
        joint: object,
        link: object,
        *,
        world_axis: Union[str, Sequence[float]],
        direction: Union[str, float, int],
        min_delta: float = 0.0,
        q0: Optional[float] = None,
        q1: Optional[float] = None,
        name: Optional[str] = None,
    ) -> bool:
        self._warn_deprecated_helper(
            "expect_joint_motion_axis(...)",
            "pose-specific exact contact/gap/overlap checks",
        )
        """
        Check that moving one joint from q0 -> q1 moves a link center along a world axis
        in the expected sign direction.

        This is useful for catching hinge/pivot sign mistakes, such as a trash-can lid
        rotating inward instead of outward.

        Parameters:
          - world_axis: one of "x", "y", "z"
          - direction: "positive" or "negative"
          - min_delta: minimum magnitude along the chosen axis in the requested direction
          - q0/q1: optional explicit samples; when omitted, derive from joint limits
                   (continuous joints default to 0 -> pi/2)
        """

        j_name = _named_ref(joint, kind="joint")
        l_name = _named_ref(link, kind="link")
        axis_key, axis_sign, axis_err = _normalize_axis_name(world_axis)
        if axis_err is not None or axis_key is None:
            return self._record(
                name or f"expect_joint_motion_axis({j_name},{l_name},{world_axis},{direction})",
                False,
                axis_err or "world_axis must be one of: x, y, z",
            )
        dir_key, dir_err = _normalize_direction_name(direction)
        if dir_err is not None or dir_key is None:
            return self._record(
                name or f"expect_joint_motion_axis({j_name},{l_name},{axis_key},{direction})",
                False,
                dir_err or "direction must be either 'positive' or 'negative'",
            )
        if axis_sign < 0.0:
            dir_key = "negative" if dir_key == "positive" else "positive"
        check_name = name or f"expect_joint_motion_axis({j_name},{l_name},{axis_key},{dir_key})"

        axis_index = {"x": 0, "y": 1, "z": 2}[axis_key]

        min_delta_f = float(min_delta)
        if min_delta_f < 0.0:
            return self._record(check_name, False, "min_delta must be >= 0")

        joint_obj = None
        get_joint = getattr(self.model, "get_articulation", None)
        if not callable(get_joint):
            get_joint = getattr(self.model, "get_joint", None)
        if callable(get_joint):
            try:
                joint_obj = get_joint(j_name)
            except Exception:
                joint_obj = None
        if joint_obj is None:
            joints = getattr(self.model, "articulations", None)
            if not isinstance(joints, list):
                joints = getattr(self.model, "joints", None)
            if isinstance(joints, list):
                for j in joints:
                    if getattr(j, "name", None) == j_name:
                        joint_obj = j
                        break
        if joint_obj is None:
            return self._record(check_name, False, f"joint {j_name!r} not found")

        if q0 is None or q1 is None:
            jt = getattr(joint_obj, "joint_type", None)
            lim = getattr(joint_obj, "limit", None)
            if jt == ArticulationType.CONTINUOUS:
                default_q0 = 0.0
                default_q1 = 1.5707963267948966  # pi/2
            else:
                lower = None if lim is None else getattr(lim, "lower", None)
                upper = None if lim is None else getattr(lim, "upper", None)
                if lower is None or upper is None:
                    return self._record(
                        check_name,
                        False,
                        f"joint {j_name!r} has no finite lower/upper limits; provide q0 and q1 explicitly",
                    )
                default_q0 = float(lower)
                default_q1 = float(upper)
        else:
            default_q0 = 0.0
            default_q1 = 0.0

        q0_f = float(default_q0 if q0 is None else q0)
        q1_f = float(default_q1 if q1 is None else q1)
        if abs(q1_f - q0_f) <= 1e-12:
            return self._record(
                check_name, False, f"q0 and q1 must differ (q0={q0_f:.6g}, q1={q1_f:.6g})"
            )

        base_pose = dict(self._pose)
        pose0 = dict(base_pose)
        pose1 = dict(base_pose)
        pose0[j_name] = q0_f
        pose1[j_name] = q1_f

        with self.pose(pose0):
            aabb0 = self.link_world_aabb(l_name)
        with self.pose(pose1):
            aabb1 = self.link_world_aabb(l_name)
        if aabb0 is None or aabb1 is None:
            return self._record(check_name, False, f"missing AABB for link {l_name!r}")

        c0 = _aabb_center(aabb0)
        c1 = _aabb_center(aabb1)
        delta = float(c1[axis_index]) - float(c0[axis_index])

        if dir_key == "positive":
            ok = delta >= min_delta_f
            expect_txt = f"delta_{axis_key} >= {min_delta_f:.4g}"
        else:
            ok = delta <= -min_delta_f
            expect_txt = f"delta_{axis_key} <= {-min_delta_f:.4g}"

        details = (
            f"{expect_txt}; got delta_{axis_key}={delta:.4g} "
            f"for joint={j_name!r} link={l_name!r} q0={q0_f:.4g} q1={q1_f:.4g} "
            f"center0=({c0[0]:.4g},{c0[1]:.4g},{c0[2]:.4g}) "
            f"center1=({c1[0]:.4g},{c1[1]:.4g},{c1[2]:.4g}) pose_base={base_pose}"
        )
        if not ok and getattr(joint_obj, "joint_type", None) == ArticulationType.REVOLUTE:
            axis = getattr(joint_obj, "axis", None)
            details += (
                f" | Right-hand rule hint: positive rotation follows the right-hand rule around axis={axis}. "
                "If motion is reversed, flip the joint axis sign."
            )

        return self._record(check_name, ok, details)
