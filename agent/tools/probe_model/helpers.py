from __future__ import annotations

import math
import statistics
from collections.abc import Iterable, Sequence
from contextlib import AbstractContextManager
from typing import Any

from sdk._core.v0.geometry_qc import generate_pose_samples


class ProbeLookupError(LookupError):
    pass


def _as_list3(values: Sequence[float]) -> list[float]:
    return [float(values[0]), float(values[1]), float(values[2])]


def _normalize_aabb(aabb: Any) -> dict[str, list[float]] | None:
    if aabb is None:
        return None
    mn, mx = aabb
    return {
        "min": _as_list3(mn),
        "max": _as_list3(mx),
    }


def _aabb_center(aabb: Any) -> list[float] | None:
    if aabb is None:
        return None
    mn, mx = aabb
    return [
        0.5 * (float(mn[0]) + float(mx[0])),
        0.5 * (float(mn[1]) + float(mx[1])),
        0.5 * (float(mn[2]) + float(mx[2])),
    ]


def _aabb_dims(aabb: Any) -> list[float] | None:
    if aabb is None:
        return None
    mn, mx = aabb
    return [
        float(mx[0]) - float(mn[0]),
        float(mx[1]) - float(mn[1]),
        float(mx[2]) - float(mn[2]),
    ]


def _transform_point(tf: Any, point: Sequence[float]) -> list[float] | None:
    try:
        x = float(point[0])
        y = float(point[1])
        z = float(point[2])
        return [
            float(tf[0][0]) * x + float(tf[0][1]) * y + float(tf[0][2]) * z + float(tf[0][3]),
            float(tf[1][0]) * x + float(tf[1][1]) * y + float(tf[1][2]) * z + float(tf[1][3]),
            float(tf[2][0]) * x + float(tf[2][1]) * y + float(tf[2][2]) * z + float(tf[2][3]),
        ]
    except Exception:
        return None


def _normalize_axes(value: str | Sequence[str]) -> list[str]:
    if isinstance(value, str):
        raw = [char for char in value if char.strip()]
    else:
        raw = [str(item).strip() for item in value]
    axes = [axis for axis in raw if axis in {"x", "y", "z"}]
    if not axes:
        raise ValueError("axes must contain at least one of x, y, z")
    deduped: list[str] = []
    for axis in axes:
        if axis not in deduped:
            deduped.append(axis)
    return deduped


def _coerce_float_map(values: dict[str, float] | None) -> dict[str, float]:
    if not values:
        return {}
    return {str(key): float(value) for key, value in values.items()}


class ProbeSession:
    def __init__(self, object_model: object, ctx: object) -> None:
        self.object_model = object_model
        self.ctx = ctx
        self._parts = list(getattr(object_model, "parts", []) or [])
        raw_joints = getattr(object_model, "articulations", None)
        if not isinstance(raw_joints, list):
            raw_joints = getattr(object_model, "joints", None)
        self._joints = list(raw_joints or [])
        self._part_by_name = {
            str(getattr(part, "name")): part
            for part in self._parts
            if isinstance(getattr(part, "name", None), str)
        }
        self._joint_by_name = {
            str(getattr(joint, "name")): joint
            for joint in self._joints
            if isinstance(getattr(joint, "name", None), str)
        }
        self._visual_owner_by_id: dict[int, object] = {}
        self._visual_by_part_and_name: dict[tuple[str, str], object] = {}
        for part in self._parts:
            part_name = getattr(part, "name", None)
            visuals = list(getattr(part, "visuals", []) or [])
            if not isinstance(part_name, str):
                continue
            for visual in visuals:
                self._visual_owner_by_id[id(visual)] = part
                visual_name = getattr(visual, "name", None)
                if isinstance(visual_name, str):
                    self._visual_by_part_and_name[(part_name, visual_name)] = visual

    def catalog(self) -> dict[str, list[str]]:
        return {
            "core": [
                "object_model",
                "ctx",
                "emit",
                "pose",
                "part",
                "joint",
                "visual",
                "parts",
                "joints",
                "visuals",
                "name",
            ],
            "measurement": [
                "aabb",
                "dims",
                "center",
                "position",
                "projection",
                "summary",
                "pair_report",
                "gap_report",
                "overlap_report",
                "within_report",
                "contact_report",
                "mount_report",
                "containment_report",
                "alignment_report",
            ],
            "review": [
                "sample_poses",
                "nearest_neighbors",
                "find_clearance_risks",
                "find_floating_parts",
                "layout_report",
                "grid_report",
                "symmetry_report",
            ],
        }

    def part(self, name: str) -> object:
        part = self._part_by_name.get(str(name))
        if part is None:
            raise ProbeLookupError(f"Unknown part: {name!r}")
        return part

    def joint(self, name: str) -> object:
        joint = self._joint_by_name.get(str(name))
        if joint is None:
            raise ProbeLookupError(f"Unknown articulation: {name!r}")
        return joint

    def visual(self, part_name: str, visual_name: str) -> object:
        visual = self._visual_by_part_and_name.get((str(part_name), str(visual_name)))
        if visual is None:
            raise ProbeLookupError(f"Unknown visual: part={part_name!r} visual={visual_name!r}")
        return visual

    def parts(self) -> list[object]:
        return list(self._parts)

    def joints(self) -> list[object]:
        return list(self._joints)

    def visuals(self, part_or_name: object) -> list[object]:
        part = self._resolve_part(part_or_name)
        return list(getattr(part, "visuals", []) or [])

    def name(self, obj: object) -> str:
        value = getattr(obj, "name", None)
        if isinstance(value, str) and value:
            return value
        owner = self._visual_owner_by_id.get(id(obj))
        if owner is not None:
            owner_name = getattr(owner, "name", "unknown")
            visual_name = getattr(obj, "name", None)
            return f"{owner_name}:{visual_name or type(obj).__name__}"
        return type(obj).__name__

    def pose(
        self, mapping: dict[str, float] | None = None, **kwargs: float
    ) -> AbstractContextManager:
        merged = _coerce_float_map(mapping)
        merged.update({str(key): float(value) for key, value in kwargs.items()})
        return self.ctx.pose(merged)

    def aabb(self, obj: object) -> dict[str, list[float]] | None:
        target = self._resolve_target(obj)
        if target["visual"] is not None:
            world_aabb = self.ctx.part_element_world_aabb(target["part"], elem=target["visual"])
        else:
            world_aabb = self.ctx.part_world_aabb(target["part"])
        return _normalize_aabb(world_aabb)

    def dims(self, obj: object) -> list[float] | None:
        target = self._resolve_target(obj)
        if target["visual"] is not None:
            world_aabb = self.ctx.part_element_world_aabb(target["part"], elem=target["visual"])
        else:
            world_aabb = self.ctx.part_world_aabb(target["part"])
        return _aabb_dims(world_aabb)

    def center(self, obj: object) -> list[float] | None:
        target = self._resolve_target(obj)
        if target["visual"] is not None:
            world_aabb = self.ctx.part_element_world_aabb(target["part"], elem=target["visual"])
        else:
            world_aabb = self.ctx.part_world_aabb(target["part"])
        return _aabb_center(world_aabb)

    def position(self, obj: object) -> list[float] | None:
        if self._is_joint(obj):
            return self._joint_world_position(obj)
        target = self._resolve_target(obj)
        if target["visual"] is None:
            pos = self.ctx.part_world_position(target["part"])
            return None if pos is None else _as_list3(pos)
        return self.center(obj)

    def projection(self, obj: object, axis_or_axes: str | Sequence[str]) -> dict[str, Any]:
        target = self._resolve_target(obj)
        elements, _, _, error = self.ctx._resolve_exact_elements(
            target["part"],
            elem=target["visual"],
            kind_prefix="probe",
        )
        if error or elements is None:
            raise ProbeLookupError(error or "missing exact geometry")
        axes = _normalize_axes(axis_or_axes)
        intervals = {
            axis: [float(v) for v in self.ctx._elements_projection_interval(elements, axis=axis)]
            for axis in axes
        }
        return {
            "target": self._target_ref(target["part"], target["visual"]),
            "metric_kind": "exact_projection",
            "intervals": intervals,
        }

    def summary(self, obj: object) -> dict[str, Any]:
        if self._is_joint(obj):
            limits = getattr(obj, "motion_limits", None)
            lower = getattr(limits, "lower", None)
            upper = getattr(limits, "upper", None)
            return {
                "kind": "articulation",
                "name": self.name(obj),
                "type": str(getattr(obj, "type", getattr(obj, "joint_type", "unknown"))),
                "axis": list(getattr(obj, "axis", ()) or ()),
                "limits": {
                    "lower": None if lower is None else float(lower),
                    "upper": None if upper is None else float(upper),
                    "effort": None
                    if getattr(limits, "effort", None) is None
                    else float(getattr(limits, "effort")),
                    "velocity": None
                    if getattr(limits, "velocity", None) is None
                    else float(getattr(limits, "velocity")),
                },
            }
        target = self._resolve_target(obj)
        parent_name = None if target["visual"] is None else self.name(target["part"])
        return {
            "kind": "visual" if target["visual"] is not None else "part",
            "name": self.name(target["visual"] or target["part"]),
            "parent_part": parent_name,
            "position": self.position(target["visual"] or target["part"]),
            "center": self.center(target["visual"] or target["part"]),
            "dims": self.dims(target["visual"] or target["part"]),
            "aabb": self.aabb(target["visual"] or target["part"]),
            "visual_names": [self.name(visual) for visual in self.visuals(target["part"])]
            if target["visual"] is None
            else None,
        }

    def pair_report(
        self,
        a: object,
        b: object,
        *,
        elem_a: object | None = None,
        elem_b: object | None = None,
    ) -> dict[str, Any]:
        target_a = self._resolve_target(a, elem=elem_a)
        target_b = self._resolve_target(b, elem=elem_b)
        elements_a, _, _, error_a = self.ctx._resolve_exact_elements(
            target_a["part"],
            elem=target_a["visual"],
            kind_prefix="a",
        )
        elements_b, _, _, error_b = self.ctx._resolve_exact_elements(
            target_b["part"],
            elem=target_b["visual"],
            kind_prefix="b",
        )
        if error_a or elements_a is None:
            raise ProbeLookupError(error_a or "missing exact geometry for a")
        if error_b or elements_b is None:
            raise ProbeLookupError(error_b or "missing exact geometry for b")

        min_distance, collided = self.ctx._exact_pair_distance(elements_a, elements_b)
        report: dict[str, Any] = {
            "metric_kind": "exact_pair",
            "a": self._target_ref(target_a["part"], target_a["visual"]),
            "b": self._target_ref(target_b["part"], target_b["visual"]),
            "collided": bool(collided),
            "min_distance": float(min_distance),
            "intervals": {},
        }
        for axis in ("x", "y", "z"):
            min_a, max_a = self.ctx._elements_projection_interval(elements_a, axis=axis)
            min_b, max_b = self.ctx._elements_projection_interval(elements_b, axis=axis)
            overlap = min(float(max_a), float(max_b)) - max(float(min_a), float(min_b))
            signed_gap = max(float(min_a) - float(max_b), float(min_b) - float(max_a))
            report["intervals"][axis] = {
                "a": [float(min_a), float(max_a)],
                "b": [float(min_b), float(max_b)],
            }
            report[f"gap_{axis}"] = float(signed_gap)
            report[f"overlap_{axis}"] = max(0.0, float(overlap))
        center_a = self.center(target_a["visual"] or target_a["part"])
        center_b = self.center(target_b["visual"] or target_b["part"])
        if center_a is not None and center_b is not None:
            report["center_delta"] = [
                float(center_a[0] - center_b[0]),
                float(center_a[1] - center_b[1]),
                float(center_a[2] - center_b[2]),
            ]
        return report

    def gap_report(
        self,
        positive: object,
        negative: object,
        axis: str,
        *,
        positive_elem: object | None = None,
        negative_elem: object | None = None,
    ) -> dict[str, Any]:
        axis_name = _normalize_axes(axis)[0]
        positive_target = self._resolve_target(positive, elem=positive_elem)
        negative_target = self._resolve_target(negative, elem=negative_elem)
        positive_elements, _, _, positive_error = self.ctx._resolve_exact_elements(
            positive_target["part"],
            elem=positive_target["visual"],
            kind_prefix="positive",
        )
        negative_elements, _, _, negative_error = self.ctx._resolve_exact_elements(
            negative_target["part"],
            elem=negative_target["visual"],
            kind_prefix="negative",
        )
        if positive_error or positive_elements is None:
            raise ProbeLookupError(positive_error or "missing exact geometry for positive target")
        if negative_error or negative_elements is None:
            raise ProbeLookupError(negative_error or "missing exact geometry for negative target")
        positive_min, positive_max = self.ctx._elements_projection_interval(
            positive_elements,
            axis=axis_name,
        )
        negative_min, negative_max = self.ctx._elements_projection_interval(
            negative_elements,
            axis=axis_name,
        )
        gap = float(positive_min) - float(negative_max)
        return {
            "metric_kind": "projection_gap",
            "axis": axis_name,
            "positive": self._target_ref(positive_target["part"], positive_target["visual"]),
            "negative": self._target_ref(negative_target["part"], negative_target["visual"]),
            "gap": gap,
            "intervals": {
                "positive": [float(positive_min), float(positive_max)],
                "negative": [float(negative_min), float(negative_max)],
            },
        }

    def overlap_report(
        self,
        a: object,
        b: object,
        *,
        axes: str | Sequence[str] = "xy",
        elem_a: object | None = None,
        elem_b: object | None = None,
    ) -> dict[str, Any]:
        pair = self.pair_report(a, b, elem_a=elem_a, elem_b=elem_b)
        axis_names = _normalize_axes(axes)
        overlaps = {axis: float(pair[f"overlap_{axis}"]) for axis in axis_names}
        return {
            "metric_kind": "projection_overlap",
            "a": pair["a"],
            "b": pair["b"],
            "axes": axis_names,
            "overlap": overlaps,
            "min_overlap": min(overlaps.values()) if overlaps else None,
        }

    def within_report(
        self,
        inner: object,
        outer: object,
        *,
        axes: str | Sequence[str] = "xy",
        inner_elem: object | None = None,
        outer_elem: object | None = None,
    ) -> dict[str, Any]:
        inner_target = self._resolve_target(inner, elem=inner_elem)
        outer_target = self._resolve_target(outer, elem=outer_elem)
        inner_elements, _, _, inner_error = self.ctx._resolve_exact_elements(
            inner_target["part"],
            elem=inner_target["visual"],
            kind_prefix="inner",
        )
        outer_elements, _, _, outer_error = self.ctx._resolve_exact_elements(
            outer_target["part"],
            elem=outer_target["visual"],
            kind_prefix="outer",
        )
        if inner_error or inner_elements is None:
            raise ProbeLookupError(inner_error or "missing exact geometry for inner target")
        if outer_error or outer_elements is None:
            raise ProbeLookupError(outer_error or "missing exact geometry for outer target")
        margins: dict[str, dict[str, float]] = {}
        within = True
        for axis in _normalize_axes(axes):
            inner_min, inner_max = self.ctx._elements_projection_interval(inner_elements, axis=axis)
            outer_min, outer_max = self.ctx._elements_projection_interval(outer_elements, axis=axis)
            lower_margin = float(inner_min) - float(outer_min)
            upper_margin = float(outer_max) - float(inner_max)
            axis_within = lower_margin >= 0.0 and upper_margin >= 0.0
            within = within and axis_within
            margins[axis] = {
                "lower_margin": lower_margin,
                "upper_margin": upper_margin,
                "within": axis_within,
            }
        return {
            "metric_kind": "projection_within",
            "inner": self._target_ref(inner_target["part"], inner_target["visual"]),
            "outer": self._target_ref(outer_target["part"], outer_target["visual"]),
            "axes": _normalize_axes(axes),
            "within": within,
            "margins": margins,
        }

    def contact_report(
        self,
        a: object,
        b: object,
        *,
        elem_a: object | None = None,
        elem_b: object | None = None,
        contact_tol: float = 1e-6,
    ) -> dict[str, Any]:
        pair = self.pair_report(a, b, elem_a=elem_a, elem_b=elem_b)
        contact = bool(pair["collided"]) or float(pair["min_distance"]) <= float(contact_tol)
        return {
            "metric_kind": "exact_contact",
            "a": pair["a"],
            "b": pair["b"],
            "contact": contact,
            "contact_tol": float(contact_tol),
            "min_distance": pair["min_distance"],
            "collided": pair["collided"],
        }

    def mount_report(
        self,
        child: object,
        parent: object,
        *,
        elem_a: object | None = None,
        elem_b: object | None = None,
    ) -> dict[str, Any]:
        pair = self.pair_report(child, parent, elem_a=elem_a, elem_b=elem_b)
        within_xy = self.within_report(
            child,
            parent,
            axes="xy",
            inner_elem=elem_a,
            outer_elem=elem_b,
        )
        gap_z = self.gap_report(
            child,
            parent,
            "z",
            positive_elem=elem_a,
            negative_elem=elem_b,
        )
        child_center = self.center(elem_a or child)
        parent_center = self.center(elem_b or parent)
        center_offset_xy = None
        if child_center is not None and parent_center is not None:
            center_offset_xy = [
                float(child_center[0] - parent_center[0]),
                float(child_center[1] - parent_center[1]),
            ]
        looks_mounted = bool(within_xy["within"]) and float(gap_z["gap"]) <= 0.02
        return {
            "metric_kind": "mount_review",
            "child": pair["a"],
            "parent": pair["b"],
            "looks_mounted": looks_mounted,
            "within_xy": within_xy,
            "gap_z": gap_z,
            "pair": pair,
            "center_offset_xy": center_offset_xy,
        }

    def containment_report(
        self,
        inner: object,
        outer: object,
        *,
        axes: str | Sequence[str] = "xy",
    ) -> dict[str, Any]:
        return self.within_report(inner, outer, axes=axes)

    def alignment_report(self, a: object, b: object) -> dict[str, Any]:
        center_a = self.center(a)
        center_b = self.center(b)
        if center_a is None or center_b is None:
            raise ProbeLookupError("alignment_report requires world-space centers")
        delta = [
            float(center_a[0] - center_b[0]),
            float(center_a[1] - center_b[1]),
            float(center_a[2] - center_b[2]),
        ]
        return {
            "metric_kind": "center_alignment",
            "a": self._target_ref(*self._target_tuple(a)),
            "b": self._target_ref(*self._target_tuple(b)),
            "delta": delta,
            "abs_delta": [abs(value) for value in delta],
        }

    def sample_poses(self, max_samples: int = 32, seed: int = 0) -> list[dict[str, float]]:
        return generate_pose_samples(
            self.object_model,
            max_samples=int(max_samples),
            seed=int(seed),
        )

    def nearest_neighbors(
        self,
        obj: object,
        *,
        candidates: Iterable[object] | None = None,
        limit: int = 5,
    ) -> list[dict[str, Any]]:
        base_part, base_visual = self._target_tuple(obj)
        base_target = self._target_ref(base_part, base_visual)
        pool = (
            list(candidates)
            if candidates is not None
            else [part for part in self._parts if part is not base_part]
        )
        reports: list[dict[str, Any]] = []
        for candidate in pool:
            report = self.pair_report(obj, candidate)
            reports.append(
                {
                    "target": base_target,
                    "candidate": report["b"],
                    "collided": report["collided"],
                    "min_distance": report["min_distance"],
                    "gap_x": report["gap_x"],
                    "gap_y": report["gap_y"],
                    "gap_z": report["gap_z"],
                }
            )
        reports.sort(key=lambda item: (0 if item["collided"] else 1, float(item["min_distance"])))
        return reports[: max(1, int(limit))]

    def find_clearance_risks(
        self,
        *,
        limit: int = 10,
        parts: Iterable[object] | None = None,
    ) -> list[dict[str, Any]]:
        part_list = self._coerce_parts(parts)
        findings: list[dict[str, Any]] = []
        for index, part_a in enumerate(part_list):
            for part_b in part_list[index + 1 :]:
                pair = self.pair_report(part_a, part_b)
                findings.append(
                    {
                        "a": pair["a"],
                        "b": pair["b"],
                        "collided": pair["collided"],
                        "min_distance": pair["min_distance"],
                        "gap_x": pair["gap_x"],
                        "gap_y": pair["gap_y"],
                        "gap_z": pair["gap_z"],
                    }
                )
        findings.sort(key=lambda item: (0 if item["collided"] else 1, float(item["min_distance"])))
        return findings[: max(1, int(limit))]

    def find_floating_parts(
        self,
        *,
        limit: int = 10,
        parts: Iterable[object] | None = None,
    ) -> list[dict[str, Any]]:
        part_list = self._coerce_parts(parts)
        findings: list[dict[str, Any]] = []
        for part in part_list:
            neighbors = self.nearest_neighbors(
                part, candidates=[p for p in part_list if p is not part], limit=1
            )
            nearest = neighbors[0] if neighbors else None
            findings.append(
                {
                    "part": self._target_ref(part, None),
                    "nearest": nearest,
                    "suspicious": nearest is None
                    or (not nearest["collided"] and float(nearest["min_distance"]) > 0.05),
                }
            )
        findings.sort(
            key=lambda item: (
                0 if item["suspicious"] else 1,
                -1.0 if item["nearest"] is None else -float(item["nearest"]["min_distance"]),
            )
        )
        return findings[: max(1, int(limit))]

    def layout_report(self, items: Iterable[object], *, axis: str = "x") -> dict[str, Any]:
        axis_name = _normalize_axes(axis)[0]
        item_list = list(items)
        centers = [self.center(item) for item in item_list]
        if any(center is None for center in centers):
            raise ProbeLookupError("layout_report requires measurable centers")
        typed_centers = [center for center in centers if center is not None]
        axis_index = {"x": 0, "y": 1, "z": 2}[axis_name]
        ordered = sorted(
            [
                {
                    "name": self.name(item),
                    "center": center,
                    "coordinate": float(center[axis_index]),
                }
                for item, center in zip(item_list, typed_centers, strict=False)
            ],
            key=lambda item: item["coordinate"],
        )
        coordinates = [float(item["coordinate"]) for item in ordered]
        pitches = [
            float(coordinates[index + 1] - coordinates[index])
            for index in range(len(coordinates) - 1)
        ]
        pitch_mean = statistics.fmean(pitches) if pitches else None
        pitch_stdev = statistics.pstdev(pitches) if len(pitches) > 1 else 0.0
        outliers: list[int] = []
        if pitches and pitch_mean is not None and pitch_stdev > 1e-9:
            for index, pitch in enumerate(pitches):
                if abs(pitch - pitch_mean) > 2.0 * pitch_stdev:
                    outliers.append(index)
        return {
            "metric_kind": "layout_review",
            "axis": axis_name,
            "count": len(ordered),
            "ordered": ordered,
            "span": None if len(coordinates) < 2 else float(coordinates[-1] - coordinates[0]),
            "pitches": pitches,
            "pitch_mean": None if pitch_mean is None else float(pitch_mean),
            "pitch_min": min(pitches) if pitches else None,
            "pitch_max": max(pitches) if pitches else None,
            "pitch_stdev": float(pitch_stdev),
            "outlier_indices": outliers,
        }

    def grid_report(
        self, items: Iterable[object], *, axes: str | Sequence[str] = "xy"
    ) -> dict[str, Any]:
        axis_names = _normalize_axes(axes)[:2]
        if len(axis_names) != 2:
            raise ValueError("grid_report requires exactly two axes")
        item_list = list(items)
        centers = [self.center(item) for item in item_list]
        if any(center is None for center in centers):
            raise ProbeLookupError("grid_report requires measurable centers")
        typed_centers = [center for center in centers if center is not None]
        indices = {"x": 0, "y": 1, "z": 2}
        axis_values = {
            axis: [float(center[indices[axis]]) for center in typed_centers] for axis in axis_names
        }
        clusters = {axis: self._cluster_coordinates(axis_values[axis]) for axis in axis_names}
        return {
            "metric_kind": "grid_review",
            "axes": axis_names,
            "count": len(typed_centers),
            "rows": len(clusters[axis_names[1]]),
            "cols": len(clusters[axis_names[0]]),
            "axis_clusters": {
                axis: [[float(value) for value in cluster] for cluster in clusters[axis]]
                for axis in axis_names
            },
        }

    def symmetry_report(self, items: Iterable[object], *, axis: str = "x") -> dict[str, Any]:
        axis_name = _normalize_axes(axis)[0]
        axis_index = {"x": 0, "y": 1, "z": 2}[axis_name]
        item_list = list(items)
        resolved = [
            {
                "name": self.name(item),
                "center": self.center(item),
            }
            for item in item_list
        ]
        if any(item["center"] is None for item in resolved):
            raise ProbeLookupError("symmetry_report requires measurable centers")
        centers = [item["center"] for item in resolved if item["center"] is not None]
        axis_coords = [float(center[axis_index]) for center in centers]
        midplane = 0.5 * (min(axis_coords) + max(axis_coords))
        remaining = resolved.copy()
        pairs: list[dict[str, Any]] = []
        unmatched: list[str] = []
        max_offset = 0.0
        while remaining:
            current = remaining.pop(0)
            current_center = current["center"]
            assert current_center is not None
            mirrored_coord = 2.0 * midplane - float(current_center[axis_index])
            best_index = None
            best_error = float("inf")
            for index, candidate in enumerate(remaining):
                candidate_center = candidate["center"]
                assert candidate_center is not None
                mirror_error = abs(float(candidate_center[axis_index]) - mirrored_coord)
                other_error = math.sqrt(
                    sum(
                        (float(candidate_center[i]) - float(current_center[i])) ** 2
                        for i in range(3)
                        if i != axis_index
                    )
                )
                total_error = mirror_error + other_error
                if total_error < best_error:
                    best_error = total_error
                    best_index = index
            if best_index is None or best_error > 0.05:
                unmatched.append(str(current["name"]))
                continue
            candidate = remaining.pop(best_index)
            max_offset = max(max_offset, float(best_error))
            pairs.append(
                {
                    "a": current["name"],
                    "b": candidate["name"],
                    "mirror_error": float(best_error),
                }
            )
        return {
            "metric_kind": "symmetry_review",
            "axis": axis_name,
            "midplane": float(midplane),
            "pairs": pairs,
            "unmatched": unmatched,
            "max_mirror_offset": float(max_offset),
        }

    def build_namespace(self, *, emit: object) -> dict[str, Any]:
        return {
            "object_model": self.object_model,
            "ctx": self.ctx,
            "emit": emit,
            "pose": self.pose,
            "part": self.part,
            "joint": self.joint,
            "visual": self.visual,
            "parts": self.parts,
            "joints": self.joints,
            "visuals": self.visuals,
            "name": self.name,
            "aabb": self.aabb,
            "dims": self.dims,
            "center": self.center,
            "position": self.position,
            "projection": self.projection,
            "summary": self.summary,
            "pair_report": self.pair_report,
            "gap_report": self.gap_report,
            "overlap_report": self.overlap_report,
            "within_report": self.within_report,
            "contact_report": self.contact_report,
            "mount_report": self.mount_report,
            "containment_report": self.containment_report,
            "alignment_report": self.alignment_report,
            "layout_report": self.layout_report,
            "grid_report": self.grid_report,
            "symmetry_report": self.symmetry_report,
            "sample_poses": self.sample_poses,
            "nearest_neighbors": self.nearest_neighbors,
            "find_clearance_risks": self.find_clearance_risks,
            "find_floating_parts": self.find_floating_parts,
            "catalog": self.catalog,
        }

    def _resolve_part(self, value: object) -> object:
        if isinstance(value, str):
            return self.part(value)
        if id(value) in self._visual_owner_by_id:
            return self._visual_owner_by_id[id(value)]
        if value in self._parts:
            return value
        part_name = getattr(value, "name", None)
        if isinstance(part_name, str) and part_name in self._part_by_name:
            return self._part_by_name[part_name]
        raise ProbeLookupError(f"Expected a part or visual, got {type(value).__name__}")

    def _resolve_joint(self, value: object) -> object:
        if isinstance(value, str):
            joint = self._joint_by_name.get(value)
            if joint is not None:
                return joint
        if value in self._joints:
            return value
        joint_name = getattr(value, "name", None)
        if isinstance(joint_name, str) and joint_name in self._joint_by_name:
            return self._joint_by_name[joint_name]
        raise ProbeLookupError(f"Expected an articulation, got {type(value).__name__}")

    def _joint_world_position(self, value: object) -> list[float] | None:
        joint = self._resolve_joint(value)
        origin = getattr(joint, "origin", None)
        xyz = getattr(origin, "xyz", None)
        if not isinstance(xyz, (tuple, list)) or len(xyz) != 3:
            return None
        parent = getattr(joint, "parent", None)
        if parent is None:
            return None
        parent_part = self._resolve_part(parent)
        parent_tf = self.ctx._world_tfs().get(self.name(parent_part))
        return None if parent_tf is None else _transform_point(parent_tf, xyz)

    def _resolve_visual_ref(self, part: object, elem: object | None) -> object | None:
        if elem is None:
            return None
        if isinstance(elem, str):
            return self.visual(self.name(part), elem)
        owner = self._visual_owner_by_id.get(id(elem))
        if owner is part:
            return elem
        if owner is not None:
            raise ProbeLookupError(
                f"Visual {self.name(elem)!r} belongs to {self.name(owner)!r}, "
                f"not {self.name(part)!r}"
            )
        raise ProbeLookupError(f"Expected a visual for {self.name(part)!r}")

    def _resolve_target(
        self, value: object, *, elem: object | None = None
    ) -> dict[str, object | None]:
        if id(value) in self._visual_owner_by_id:
            part = self._visual_owner_by_id[id(value)]
            visual = value
            if elem is not None:
                visual = self._resolve_visual_ref(part, elem)
            return {"part": part, "visual": visual}
        part = self._resolve_part(value)
        visual = self._resolve_visual_ref(part, elem)
        return {"part": part, "visual": visual}

    def _target_tuple(self, value: object) -> tuple[object, object | None]:
        target = self._resolve_target(value)
        return target["part"], target["visual"]

    def _target_ref(self, part: object, visual: object | None) -> dict[str, str]:
        ref = {"part": self.name(part)}
        if visual is not None:
            ref["visual"] = self.name(visual)
        return ref

    def _is_joint(self, value: object) -> bool:
        return value in self._joints or (
            isinstance(getattr(value, "name", None), str)
            and getattr(value, "name", None) in self._joint_by_name
        )

    def _coerce_parts(self, values: Iterable[object] | None) -> list[object]:
        if values is None:
            return list(self._parts)
        return [self._resolve_part(value) for value in values]

    def _cluster_coordinates(self, coordinates: Sequence[float]) -> list[list[float]]:
        if not coordinates:
            return []
        ordered = sorted(float(value) for value in coordinates)
        if len(ordered) == 1:
            return [[ordered[0]]]
        diffs = [ordered[index + 1] - ordered[index] for index in range(len(ordered) - 1)]
        positive_diffs = [diff for diff in diffs if diff > 1e-9]
        tol = 1e-6 if not positive_diffs else max(1e-6, 0.25 * statistics.median(positive_diffs))
        clusters: list[list[float]] = [[ordered[0]]]
        for value in ordered[1:]:
            if abs(value - statistics.fmean(clusters[-1])) <= tol:
                clusters[-1].append(value)
            else:
                clusters.append([value])
        return clusters
