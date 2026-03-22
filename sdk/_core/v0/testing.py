from __future__ import annotations

import math
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Set, Tuple, Union

from .assets import resolve_asset_root
from .errors import ValidationError
from .generated_collisions import compile_object_model_with_generated_collisions
from .geometry_qc import (
    compute_part_world_transforms,
    default_overlap_tol_from_env,
    default_overlap_volume_tol_from_env,
    find_geometry_overlaps,
    generate_pose_samples,
    part_world_aabb,
)
from .types import ArticulationType, Origin

Vec3 = Tuple[float, float, float]
AABB = Tuple[Vec3, Vec3]

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


@dataclass(frozen=True)
class _CoplanarFinding:
    link_a: str
    link_b: str
    relation: str
    risk: str
    pose_index: int
    pose: Dict[str, float]
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


def _parse_xyz(xyz: str) -> Optional[Vec3]:
    parts = (xyz or "").strip().split()
    if len(parts) != 3:
        return None
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return None


def _aabb_union(aabbs: Sequence[AABB]) -> Optional[AABB]:
    if not aabbs:
        return None
    mnx = min(a[0][0] for a in aabbs)
    mny = min(a[0][1] for a in aabbs)
    mnz = min(a[0][2] for a in aabbs)
    mxx = max(a[1][0] for a in aabbs)
    mxy = max(a[1][1] for a in aabbs)
    mxz = max(a[1][2] for a in aabbs)
    return (mnx, mny, mnz), (mxx, mxy, mxz)


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


def _normalize_geometry_source(value: str, *, field_name: str) -> str:
    source = str(value or "").strip().lower()
    if source not in {"collision", "visual"}:
        raise ValidationError(f"{field_name} must be either 'collision' or 'visual'")
    return source


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
    geometry_source: str = "collision"
    seed: int = 0

    checks_run: int = 0
    _checks: List[str] = field(default_factory=list)
    _failures: List[TestFailure] = field(default_factory=list)
    _warnings: List[str] = field(default_factory=list)
    _allow_pairs: Dict[Tuple[str, str], str] = field(default_factory=dict)
    _allowances: List[str] = field(default_factory=list)
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
    _compiled_collision_model: Optional[object] = field(default=None, repr=False)
    _compiled_collision_error: Optional[Exception] = field(default=None, repr=False)
    _collision_parts_by_name_cache: Optional[Dict[str, object]] = field(default=None, repr=False)
    _mesh_aabb_cache: Dict[Path, AABB] = field(default_factory=dict, repr=False)
    _part_local_aabbs_cache: Dict[Tuple[str, bool], Tuple[AABB, ...]] = field(
        default_factory=dict, repr=False
    )
    _part_world_aabb_cache: Dict[Tuple[str, str], Optional[AABB]] = field(
        default_factory=dict, repr=False
    )

    def __post_init__(self) -> None:
        self.geometry_source = _normalize_geometry_source(
            self.geometry_source,
            field_name="geometry_source",
        )
        try:
            self._compiled_collision_model = compile_object_model_with_generated_collisions(
                self.model,  # type: ignore[arg-type]
                asset_root=self.asset_root,
            )
        except Exception as exc:
            self._compiled_collision_model = None
            self._compiled_collision_error = exc

    def report(self) -> TestReport:
        failures = tuple(self._failures)
        return TestReport(
            passed=not failures,
            checks_run=int(self.checks_run),
            checks=tuple(self._checks),
            failures=failures,
            warnings=tuple(self._warnings),
            allowances=tuple(self._allowances),
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

    def _collision_parts_by_name(self) -> Dict[str, object]:
        if self._collision_parts_by_name_cache is None:
            parts = getattr(self._compiled_collision_model, "parts", None)
            if not isinstance(parts, list):
                self._collision_parts_by_name_cache = {}
            else:
                self._collision_parts_by_name_cache = {
                    name: part
                    for part in parts
                    if isinstance((name := getattr(part, "name", None)), str)
                }
        return self._collision_parts_by_name_cache

    def _part_local_aabbs(
        self,
        part: object,
        *,
        prefer_collisions: bool,
    ) -> Tuple[AABB, ...]:
        from .geometry_qc import part_local_aabbs  # local import

        part_name = _named_ref(part, kind="part")
        cache_key = (part_name, bool(prefer_collisions))
        cached = self._part_local_aabbs_cache.get(cache_key)
        if cached is not None:
            return cached

        if prefer_collisions and self._compiled_collision_model is not None:
            part_obj = self._collision_parts_by_name().get(part_name)
        else:
            if prefer_collisions and self._compiled_collision_error is not None:
                raise ValidationError(str(self._compiled_collision_error))
            part_obj = self._part_by_name(part_name)
        if part_obj is None:
            return ()

        cached = tuple(
            part_local_aabbs(
                part_obj,
                asset_root=self._asset_root(),
                prefer_collisions=prefer_collisions,
                _obj_cache=self._mesh_aabb_cache,
            )
        )
        self._part_local_aabbs_cache[cache_key] = cached
        return cached

    def _part_geometry_items_with_local_aabbs(
        self,
        part: object,
        *,
        use: str,
    ) -> List[Tuple[Optional[str], Optional[str], AABB]]:
        part_name = _named_ref(part, kind="part")
        use_key = _normalize_geometry_source(use, field_name="use")
        prefer_collisions = use_key == "collision"

        if prefer_collisions and self._compiled_collision_model is not None:
            part_obj = self._collision_parts_by_name().get(part_name)
        else:
            if prefer_collisions and self._compiled_collision_error is not None:
                raise ValidationError(str(self._compiled_collision_error))
            part_obj = self._part_by_name(part_name)
        if part_obj is None:
            return []

        if prefer_collisions and getattr(part_obj, "collisions", None):
            raw_items = list(getattr(part_obj, "collisions", []) or [])
        else:
            raw_items = list(getattr(part_obj, "visuals", []) or [])
        items = [item for item in raw_items if getattr(item, "geometry", None) is not None]
        aabbs = self._part_local_aabbs(part_name, prefer_collisions=prefer_collisions)

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

    def allow_overlap(
        self,
        link_a: object,
        link_b: object,
        *,
        reason: str,
        elem_a: Optional[str] = None,
        elem_b: Optional[str] = None,
    ) -> None:
        """
        Allow a specific overlap with justification.

        By default, allowances apply at the link-pair level. If you name collision elements
        (via `link.collision(..., name=...)`), you can scope an allowance to a specific
        element pair by passing `elem_a`/`elem_b`.
        """

        a = _named_ref(link_a, kind="link_a")
        b = _named_ref(link_b, kind="link_b")
        r = (reason or "").strip()
        if not r:
            raise ValueError("allow_overlap requires a non-empty reason")
        key = (a, b) if a <= b else (b, a)
        self._allow_pairs[key] = r

        ea = (str(elem_a) if elem_a is not None else "").strip() or None
        eb = (str(elem_b) if elem_b is not None else "").strip() or None
        if ea or eb:
            self._allowances.append(
                f"allow_overlap({key[0]!r}, {key[1]!r}, elem_a={ea!r}, elem_b={eb!r}): {r}"
            )
        else:
            self._allowances.append(f"allow_overlap({key[0]!r}, {key[1]!r}): {r}")
        self._allow_elems.append((key, ea, eb, r))

    def allow_coplanar_surfaces(
        self,
        link_a: object,
        link_b: object,
        *,
        reason: str,
        elem_a: Optional[str] = None,
        elem_b: Optional[str] = None,
    ) -> None:
        """Allow a specific coplanar-surface finding with justification."""

        a = _named_ref(link_a, kind="link_a")
        b = _named_ref(link_b, kind="link_b")
        r = (reason or "").strip()
        if not r:
            raise ValueError("allow_coplanar_surfaces requires a non-empty reason")
        key = _pair_key(a, b)
        self._allow_coplanar_pairs[key] = r

        ea = (str(elem_a) if elem_a is not None else "").strip() or None
        eb = (str(elem_b) if elem_b is not None else "").strip() or None
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
        joint_positions: Optional[Dict[object, float]] = None,
        **kwargs: float,
    ) -> Iterator[None]:
        prev = dict(self._pose)
        merged: Dict[str, float] = {}
        if joint_positions:
            merged.update(
                {_named_ref(k, kind="joint"): float(v) for k, v in joint_positions.items()}
            )
        merged.update({str(k): float(v) for k, v in kwargs.items()})
        self._pose = merged
        self._world_tfs_cache = None
        self._part_world_aabb_cache.clear()
        try:
            yield
        finally:
            self._pose = prev
            self._world_tfs_cache = None
            self._part_world_aabb_cache.clear()

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

    def part_world_aabb(
        self,
        part: object,
        *,
        use: Optional[str] = None,
    ) -> Optional[AABB]:
        part_name = _named_ref(part, kind="part")

        use_key = (
            self.geometry_source
            if use is None
            else _normalize_geometry_source(use, field_name="use")
        )
        cache_key = (part_name, use_key)
        if cache_key in self._part_world_aabb_cache:
            return self._part_world_aabb_cache[cache_key]

        prefer = use_key == "collision"
        if prefer and self._compiled_collision_model is not None:
            part_obj = self._collision_parts_by_name().get(part_name)
        else:
            if prefer and self._compiled_collision_error is not None:
                raise ValidationError(str(self._compiled_collision_error))
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
            prefer_collisions=prefer,
            _obj_cache=self._mesh_aabb_cache,
        )
        self._part_world_aabb_cache[cache_key] = world_aabb
        return world_aabb

    def link_world_aabb(
        self,
        link: object,
        *,
        use: Optional[str] = None,
    ) -> Optional[AABB]:
        return self.part_world_aabb(link, use=use)

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

    def check_mesh_files_exist(self) -> bool:
        links = getattr(self.model, "parts", None)
        if not isinstance(links, list):
            return self._record(
                "check_mesh_files_exist", False, "model.parts missing or not a list"
            )
        if self._compiled_collision_error is not None:
            return self._record(
                "check_mesh_files_exist",
                False,
                f"{type(self._compiled_collision_error).__name__}: {self._compiled_collision_error}",
            )
        root = self._asset_root()
        missing: List[str] = []
        collision_parts = (
            self._collision_parts_by_name() if self._compiled_collision_model is not None else {}
        )
        for link in links:
            compiled_link = collision_parts.get(getattr(link, "name", None))
            for field_name in ("visuals",):
                elems = getattr(link, field_name, None)
                if not isinstance(elems, list):
                    continue
                for elem in elems:
                    geom = getattr(elem, "geometry", None)
                    filename = getattr(geom, "filename", None)
                    if filename is None:
                        continue
                    filename = str(filename)
                    if not filename:
                        continue
                    p = Path(filename)
                    if not p.is_absolute():
                        p = (root / p).resolve()
                    if not p.exists():
                        missing.append(str(p))
            if compiled_link is not None:
                elems = getattr(compiled_link, "collisions", None)
                if isinstance(elems, list):
                    for elem in elems:
                        geom = getattr(elem, "geometry", None)
                        filename = getattr(geom, "filename", None)
                        if filename is None:
                            continue
                        p = Path(str(filename))
                        if not p.is_absolute():
                            p = (root / p).resolve()
                        if not p.exists():
                            missing.append(str(p))
        if missing:
            preview = "\n".join(missing[:12])
            more = "" if len(missing) <= 12 else f"\n... ({len(missing) - 12} more)"
            return self._record(
                "check_mesh_files_exist", False, f"Missing mesh files:\n{preview}{more}"
            )
        return self._record("check_mesh_files_exist", True)

    def _check_joint_origin_near_geometry_impl(
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
            self.warn(f"Relaxed joint-origin tolerance in use: tol={tol_f:.4g}. Reason: {r}")

        links = getattr(self.model, "parts", None)
        joints = getattr(self.model, "articulations", None)
        if not isinstance(links, list) or not isinstance(joints, list):
            return record(check_name, False, "model.parts/articulations missing")

        link_to_local: Dict[str, List[AABB]] = {}
        for link in links:
            link_name = getattr(link, "name", None)
            if not isinstance(link_name, str):
                continue
            aabbs = self._part_local_aabbs(
                link,
                prefer_collisions=self.geometry_source == "collision",
            )
            if aabbs:
                link_to_local[link_name] = list(aabbs)

        def point_aabb_distance(point: Vec3, aabb: AABB) -> float:
            (mn, mx) = aabb
            px, py, pz = point
            dx = 0.0
            if px < mn[0]:
                dx = mn[0] - px
            elif px > mx[0]:
                dx = px - mx[0]
            dy = 0.0
            if py < mn[1]:
                dy = mn[1] - py
            elif py > mx[1]:
                dy = py - mx[1]
            dz = 0.0
            if pz < mn[2]:
                dz = mn[2] - pz
            elif pz > mx[2]:
                dz = pz - mx[2]
            return (dx * dx + dy * dy + dz * dz) ** 0.5

        failures: List[str] = []
        for joint in joints:
            parent = getattr(joint, "parent", None)
            child = getattr(joint, "child", None)
            if not isinstance(parent, str) or not isinstance(child, str):
                continue
            parent_aabbs = link_to_local.get(parent)
            child_aabbs = link_to_local.get(child)
            if not parent_aabbs or not child_aabbs:
                continue

            org = getattr(joint, "origin", Origin())
            parent_point = getattr(org, "xyz", (0.0, 0.0, 0.0))
            if not (isinstance(parent_point, tuple) and len(parent_point) == 3):
                parent_point = (0.0, 0.0, 0.0)
            parent_point = (float(parent_point[0]), float(parent_point[1]), float(parent_point[2]))

            parent_dist = min(point_aabb_distance(parent_point, aabb) for aabb in parent_aabbs)
            child_dist = min(point_aabb_distance((0.0, 0.0, 0.0), aabb) for aabb in child_aabbs)
            if parent_dist > tol_f or child_dist > tol_f:
                failures.append(
                    f"joint={getattr(joint, 'name', None)!r} parent={parent!r} child={child!r} "
                    f"dist_parent={parent_dist:.4g} dist_child={child_dist:.4g} tol={tol_f:.4g}"
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

    def check_joint_origin_near_geometry(
        self,
        *,
        tol: float = _JOINT_ORIGIN_TOL_DEFAULT,
        reason: Optional[str] = None,
        name: Optional[str] = None,
    ) -> bool:
        tol_f = _normalize_joint_origin_tol(tol)
        return self._check_joint_origin_near_geometry_impl(
            tol=tol_f,
            reason=reason,
            check_name=name or f"check_joint_origin_near_geometry(tol={tol_f:.4g})",
        )

    def check_articulation_origin_near_geometry(
        self,
        *,
        tol: float = _JOINT_ORIGIN_TOL_DEFAULT,
        reason: Optional[str] = None,
        name: Optional[str] = None,
    ) -> bool:
        tol_f = _normalize_joint_origin_tol(tol)
        return self._check_joint_origin_near_geometry_impl(
            tol=tol_f,
            reason=reason,
            check_name=name or f"check_articulation_origin_near_geometry(tol={tol_f:.4g})",
        )

    def warn_if_joint_origin_near_geometry(
        self,
        *,
        tol: float = _JOINT_ORIGIN_TOL_DEFAULT,
        reason: Optional[str] = None,
        name: Optional[str] = None,
    ) -> bool:
        tol_f = _normalize_joint_origin_tol(tol)
        return self._check_joint_origin_near_geometry_impl(
            tol=tol_f,
            reason=reason,
            check_name=name or f"warn_if_joint_origin_near_geometry(tol={tol_f:.4g})",
            warn_only=True,
        )

    def warn_if_articulation_origin_near_geometry(
        self,
        *,
        tol: float = _JOINT_ORIGIN_TOL_DEFAULT,
        reason: Optional[str] = None,
        name: Optional[str] = None,
    ) -> bool:
        tol_f = _normalize_joint_origin_tol(tol)
        return self._check_joint_origin_near_geometry_impl(
            tol=tol_f,
            reason=reason,
            check_name=name or f"warn_if_articulation_origin_near_geometry(tol={tol_f:.4g})",
            warn_only=True,
        )

    def check_visual_collision_alignment(
        self,
        *,
        max_center_dist: float = 0.03,
        max_size_rel_err: float = 0.35,
        name: Optional[str] = None,
    ) -> bool:
        links = getattr(self.model, "parts", None)
        if not isinstance(links, list):
            return self._record(
                name or "check_visual_collision_alignment",
                False,
                "model.parts missing or not a list",
            )

        failures: List[str] = []
        for part in links:
            part_name = getattr(part, "name", None)
            if not isinstance(part_name, str) or not getattr(part, "visuals", None):
                continue

            visual_aabbs = self._part_local_aabbs(
                part,
                prefer_collisions=False,
            )
            collision_aabbs = self._part_local_aabbs(
                part,
                prefer_collisions=True,
            )
            if not visual_aabbs or not collision_aabbs:
                continue

            visual_union = _aabb_union(visual_aabbs)
            collision_union = _aabb_union(collision_aabbs)
            if visual_union is None or collision_union is None:
                continue

            visual_center = _aabb_center(visual_union)
            collision_center = _aabb_center(collision_union)
            center_dist = (
                (visual_center[0] - collision_center[0]) ** 2
                + (visual_center[1] - collision_center[1]) ** 2
                + (visual_center[2] - collision_center[2]) ** 2
            ) ** 0.5

            visual_size = (
                visual_union[1][0] - visual_union[0][0],
                visual_union[1][1] - visual_union[0][1],
                visual_union[1][2] - visual_union[0][2],
            )
            collision_size = (
                collision_union[1][0] - collision_union[0][0],
                collision_union[1][1] - collision_union[0][1],
                collision_union[1][2] - collision_union[0][2],
            )
            denom = max(1e-9, max(collision_size))
            size_rel_err = (
                (
                    (visual_size[0] - collision_size[0]) ** 2
                    + (visual_size[1] - collision_size[1]) ** 2
                    + (visual_size[2] - collision_size[2]) ** 2
                )
                ** 0.5
            ) / denom

            if center_dist > float(max_center_dist) or size_rel_err > float(max_size_rel_err):
                failures.append(
                    f"part={part_name!r} center_dist={center_dist:.4g} "
                    f"size_visual={tuple(round(v, 4) for v in visual_size)} "
                    f"size_collision={tuple(round(v, 4) for v in collision_size)}"
                )

        if failures:
            preview = "\n".join(failures[:10])
            more = "" if len(failures) <= 10 else f"\n... ({len(failures) - 10} more)"
            return self._record(
                name or "check_visual_collision_alignment",
                False,
                f"Visual/collision drift detected:\n{preview}{more}",
            )
        return self._record(name or "check_visual_collision_alignment", True)

    def check_part_geometry_connected(
        self,
        *,
        use: str = "visual",
        tol: float = 0.005,
        name: Optional[str] = None,
    ) -> bool:
        return self._check_part_geometry_connected_impl(
            use=use,
            tol=tol,
            check_name=name,
            warn_only=False,
        )

    def warn_if_part_geometry_connected(
        self,
        *,
        use: str = "visual",
        tol: float = 0.005,
        name: Optional[str] = None,
    ) -> bool:
        return self._check_part_geometry_connected_impl(
            use=use,
            tol=tol,
            check_name=name,
            warn_only=True,
            warn_prefix="warn_if_part_geometry_connected",
        )

    def warn_if_part_geometry_disconnected(
        self,
        *,
        use: str = "visual",
        tol: float = 0.005,
        name: Optional[str] = None,
    ) -> bool:
        return self._check_part_geometry_connected_impl(
            use=use,
            tol=tol,
            check_name=name,
            warn_only=True,
            warn_prefix="warn_if_part_geometry_disconnected",
        )

    def _check_part_geometry_connected_impl(
        self,
        *,
        use: str,
        tol: float,
        check_name: Optional[str],
        warn_only: bool,
        warn_prefix: str = "warn_if_part_geometry_connected",
    ) -> bool:
        record = self._record_warning_check if warn_only else self._record
        use_key = _normalize_geometry_source(use, field_name="use")
        if float(tol) < 0.0:
            prefix = warn_prefix if warn_only else "check_part_geometry_connected"
            return record(
                check_name or f"{prefix}(use={use_key},tol={float(tol):.4g})",
                False,
                "tol must be non-negative",
            )

        links = getattr(self.model, "parts", None)
        if not isinstance(links, list):
            prefix = warn_prefix if warn_only else "check_part_geometry_connected"
            return record(
                check_name or f"{prefix}(use={use_key},tol={float(tol):.4g})",
                False,
                "model.parts missing or not a list",
            )

        failures: List[str] = []
        for part in links:
            part_name = getattr(part, "name", None)
            if not isinstance(part_name, str):
                continue
            aabbs = self._part_local_aabbs(
                part,
                prefer_collisions=use_key == "collision",
            )
            if len(aabbs) <= 1:
                continue

            visited = set([0])
            queue = [0]
            while queue:
                current = queue.pop()
                for idx in range(len(aabbs)):
                    if idx in visited:
                        continue
                    if _aabbs_touch_or_overlap(aabbs[current], aabbs[idx], tol=float(tol)):
                        visited.add(idx)
                        queue.append(idx)
            if len(visited) != len(aabbs):
                failures.append(
                    f"part={part_name!r} connected={len(visited)}/{len(aabbs)} use={use_key}"
                )

        prefix = warn_prefix if warn_only else "check_part_geometry_connected"
        resolved_name = check_name or f"{prefix}(use={use_key},tol={float(tol):.4g})"
        if failures:
            preview = "\n".join(failures[:10])
            more = "" if len(failures) <= 10 else f"\n... ({len(failures) - 10} more)"
            return record(
                resolved_name, False, f"Disconnected geometry islands detected:\n{preview}{more}"
            )
        return record(resolved_name, True)

    def check_no_overlaps(
        self,
        *,
        max_pose_samples: int = 128,
        overlap_tol: Optional[float] = None,
        overlap_volume_tol: Optional[float] = None,
        ignore_adjacent: bool = False,
        ignore_fixed: bool = True,
    ) -> bool:
        return self._check_overlaps_impl(
            max_pose_samples=max_pose_samples,
            overlap_tol=overlap_tol,
            overlap_volume_tol=overlap_volume_tol,
            ignore_adjacent=ignore_adjacent,
            ignore_fixed=ignore_fixed,
            check_name=None,
            warn_only=False,
        )

    def check_articulation_overlaps(
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
            default_prefix="check_articulation_overlaps",
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
        prefix = default_prefix or ("warn_if_overlaps" if warn_only else "check_no_overlaps")
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

        def format_element(index: int, name: Optional[str], geometry: Optional[str]) -> str:
            geometry_name = geometry or "?"
            if isinstance(name, str) and name:
                return f"#{index} {name!r}:{geometry_name}"
            return f"#{index} <unnamed>:{geometry_name}"

        overlaps = find_geometry_overlaps(
            self.model,
            asset_root=self._asset_root(),
            geometry_source=self.geometry_source,
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
            key = (o.link_a, o.link_b) if o.link_a <= o.link_b else (o.link_b, o.link_a)
            allowed = False
            # Element-scoped allowances, if any.
            for k, ea, eb, _r in self._allow_elems:
                if k != key:
                    continue
                if ea is None and eb is None:
                    allowed = True
                    break
                # Commutative match on element names when present.
                names = (o.elem_a_name, o.elem_b_name)
                if (ea in names) and (eb in names):
                    allowed = True
                    break
            if allowed:
                continue
            relation = pair_relations.get(key, "unrelated")
            remaining.append(
                f"relation={relation} pair=({o.link_a!r},{o.link_b!r}) pose_index={o.pose_index} "
                f"depth=({o.overlap_depth[0]:.4g},{o.overlap_depth[1]:.4g},{o.overlap_depth[2]:.4g}) "
                f"min_depth={min(o.overlap_depth):.4g} vol={o.overlap_volume:.4g} "
                f"elem_a={format_element(o.elem_a, o.elem_a_name, o.elem_a_geometry)} "
                f"elem_b={format_element(o.elem_b, o.elem_b_name, o.elem_b_geometry)} "
                f"pose={o.pose}"
            )

        if remaining:
            preview = "\n".join(remaining[:8])
            more = "" if len(remaining) <= 8 else f"\n... ({len(remaining) - 8} more)"
            return record(
                resolved_name,
                False,
                "Overlaps detected "
                f"(geometry_source={self.geometry_source}, "
                f"overlap_tol={resolved_overlap_tol:.4g}, "
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
        use: str = "visual",
        plane_tol: float = 0.001,
        min_overlap: float = 0.02,
        min_overlap_ratio: float = 0.35,
        ignore_adjacent: bool = True,
        ignore_fixed: bool = True,
        name: Optional[str] = None,
    ) -> bool:
        use_key = _normalize_geometry_source(use, field_name="use")
        resolved_name = name or (
            f"warn_if_coplanar_surfaces("
            f"samples={int(max_pose_samples)},"
            f"use={use_key},"
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
                elem_bounds = self._part_geometry_items_with_local_aabbs(link_name, use=use_key)
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

    def expect_aabb_within(
        self,
        inner_link: object,
        outer_link: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        margin: float = 0.0,
        inner_use: str = "collision",
        outer_use: str = "collision",
        name: Optional[str] = None,
    ) -> bool:
        inner_name = _named_ref(inner_link, kind="inner_link")
        outer_name = _named_ref(outer_link, kind="outer_link")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_aabb_within({inner_name},{outer_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        inner = self.link_world_aabb(inner_link, use=inner_use)
        outer = self.link_world_aabb(outer_link, use=outer_use)
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
        positive_use: str = "collision",
        negative_use: str = "collision",
        name: Optional[str] = None,
    ) -> bool:
        positive_name = _named_ref(positive_link, kind="positive_link")
        negative_name = _named_ref(negative_link, kind="negative_link")
        axis_key, axis_sign, axis_err = _normalize_axis_name(axis)
        check_name = (
            name or f"expect_aabb_gap({positive_name},{negative_name},axis={axis_key or axis})"
        )
        if axis_err is not None or axis_key is None or axis_sign < 0.0:
            return self._record(check_name, False, axis_err or "axis must be one of: x, y, z")

        positive_aabb = self.link_world_aabb(positive_link, use=positive_use)
        negative_aabb = self.link_world_aabb(negative_link, use=negative_use)
        if positive_aabb is None or negative_aabb is None:
            return self._record(check_name, False, "missing link AABB(s)")

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
            f"max_penetration={max_penetration_f:.4g} pose={self._pose}",
        )

    def expect_aabb_overlap(
        self,
        link_a: object,
        link_b: object,
        *,
        axes: Union[str, Sequence[str]] = "xy",
        min_overlap: float = 0.0,
        use_a: str = "collision",
        use_b: str = "collision",
        name: Optional[str] = None,
    ) -> bool:
        link_a_name = _named_ref(link_a, kind="link_a")
        link_b_name = _named_ref(link_b, kind="link_b")
        axes_key, axes_err = _normalize_axes_spec(axes, field_name="axes")
        check_name = (
            name
            or f"expect_aabb_overlap({link_a_name},{link_b_name},axes={_axes_label(axes_key) or axes})"
        )
        if axes_err is not None:
            return self._record(check_name, False, axes_err)

        aabb_a = self.link_world_aabb(link_a, use=use_a)
        aabb_b = self.link_world_aabb(link_b, use=use_b)
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
        use_a: str = "collision",
        use_b: str = "collision",
        name: Optional[str] = None,
    ) -> bool:
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

        aabb_a = self.link_world_aabb(link_a, use=use_a)
        aabb_b = self.link_world_aabb(link_b, use=use_b)
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
        use: str = "collision",
        name: Optional[str] = None,
    ) -> bool:
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
            aabb0 = self.link_world_aabb(l_name, use=use)
        with self.pose(pose1):
            aabb1 = self.link_world_aabb(l_name, use=use)
        if aabb0 is None or aabb1 is None:
            return self._record(
                check_name, False, f"missing AABB for link {l_name!r} using {use!r} geometry"
            )

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
