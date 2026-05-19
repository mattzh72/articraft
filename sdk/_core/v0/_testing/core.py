from __future__ import annotations

import math
from contextlib import contextmanager
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

from ..geometry_qc import part_local_aabbs
from ..geometry_qc import part_world_aabb as compute_part_world_aabb
from .common import (
    AABB,
    AllowedOverlap,
    Box,
    Cylinder,
    GeometryOverlap,
    Mesh,
    Origin,
    PoseValue,
    Sphere,
    TestFailure,
    TestReport,
    ValidationError,
    Vec3,
    _axis_unit,
    _coerce_joint_pose_value,
    _collision_object_from_geometry,
    _dot_vec3,
    _ExactElement,
    _identity4,
    _mat4_mul,
    _named_ref,
    _origin_to_mat4,
    _pair_key,
    _transform_aabb,
    compile_object_model_with_exact_collisions,
    compute_part_world_transforms,
    resolve_asset_root,
    resolve_mesh_path,
)


class TestContextCoreMixin:
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
            allowed_overlaps=tuple(
                AllowedOverlap(
                    link_a=pair[0],
                    link_b=pair[1],
                    elem_a=elem_a,
                    elem_b=elem_b,
                    reason=reason,
                )
                for pair, elem_a, elem_b, reason in self._allow_elems
            ),
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
                validate=not self._model_validated_strict,
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
                if getattr(joint, "mimic", None) is not None:
                    raise ValidationError(
                        f"Articulation {joint_name!r} is mimic-driven and cannot be posed directly"
                    )
                merged[joint_name] = _coerce_joint_pose_value(joint, value)
        for key, value in kwargs.items():
            joint_name = str(key)
            joint = joint_lookup.get(joint_name)
            if joint is None:
                raise ValidationError(f"Unknown joint: {joint_name!r}")
            if getattr(joint, "mimic", None) is not None:
                raise ValidationError(
                    f"Articulation {joint_name!r} is mimic-driven and cannot be posed directly"
                )
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
        world_aabb = compute_part_world_aabb(
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
