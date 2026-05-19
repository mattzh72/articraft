from __future__ import annotations

import logging
import sys
from dataclasses import dataclass
from math import acos, atan2, cos, isfinite, pi, sin, sqrt, tan
from typing import Iterable, List, Optional, Sequence, Tuple

from .booleans import _boolean_union_many, _is_expected_boolean_fallback_error
from .common import (
    _EPS,
    Vec2,
    Vec3,
    _clamp,
    _ensure_ccw,
    _profile_points_2d,
    _v_add,
    _v_cross,
    _v_dot,
    _v_lerp,
    _v_norm,
    _v_normalize,
    _v_scale,
    _v_sub,
    sample_arc_3d,
    sample_catmull_rom_spline_3d,
    sample_cubic_bezier_spline_3d,
)
from .primitives import (
    CylinderGeometry,
    LoftGeometry,
    MeshGeometry,
    SphereGeometry,
)

logger = logging.getLogger(__name__)


def _tube_network_compat_hooks():
    """
    Resolve legacy monkeypatch hooks exposed through ``sdk._core.v0.mesh``.

    Older internal tests and downstream debug code patch these private helpers
    on the public compatibility facade. Keep that narrow behavior here while
    the implementation lives in this domain module.
    """

    compat_module = sys.modules.get("sdk._core.v0.mesh")
    return (
        getattr(compat_module, "_boolean_union_many", _boolean_union_many),
        getattr(
            compat_module,
            "_is_expected_boolean_fallback_error",
            _is_expected_boolean_fallback_error,
        ),
        getattr(
            compat_module,
            "_tube_network_visual_mesh_from_paths",
            _tube_network_visual_mesh_from_paths,
        ),
    )


class SweepGeometry(MeshGeometry):
    def __init__(
        self,
        profile: Iterable[Tuple[float, float]],
        path: Iterable[Tuple[float, float, float]],
        *,
        cap: bool = False,
        closed: bool = True,
    ):
        super().__init__()
        points = _ensure_ccw(_profile_points_2d(profile))
        if not closed:
            cap = False

        path_points = [(float(x), float(y), float(z)) for (x, y, z) in path]
        if len(path_points) < 2:
            raise ValueError("Sweep requires at least two path points")

        profiles = [[(x + px, y + py, pz) for (x, y) in points] for (px, py, pz) in path_points]
        self.merge(LoftGeometry(profiles, cap=cap, closed=closed))


def _dedupe_path_points(path: List[Vec3]) -> List[Vec3]:
    if not path:
        return []
    out = [path[0]]
    for p in path[1:]:
        if _v_norm(_v_sub(p, out[-1])) > _EPS:
            out.append(p)
    return out


def _compute_path_tangents(path: List[Vec3]) -> List[Vec3]:
    n = len(path)
    if n < 2:
        raise ValueError("Pipe path requires at least two distinct points")
    tangents: List[Vec3] = []
    for i in range(n):
        if i == 0:
            d = _v_sub(path[1], path[0])
        elif i == n - 1:
            d = _v_sub(path[-1], path[-2])
        else:
            d = _v_sub(path[i + 1], path[i - 1])
        tangents.append(_v_normalize(d))
    return tangents


def _initial_frame(tangent: Vec3, up_hint: Vec3) -> Tuple[Vec3, Vec3]:
    up = _v_normalize(up_hint)
    if abs(_v_dot(up, tangent)) > 0.95:
        up = (0.0, 1.0, 0.0) if abs(tangent[1]) < 0.95 else (1.0, 0.0, 0.0)
    n = _v_cross(up, tangent)
    if _v_norm(n) <= _EPS:
        n = _v_cross((0.0, 0.0, 1.0), tangent)
    n = _v_normalize(n)
    b = _v_normalize(_v_cross(tangent, n))
    return (n, b)


@dataclass
class _WireCorner:
    incoming_end: Vec3
    outgoing_start: Vec3
    bridge: List[Vec3]


def _append_distinct_path_point(points: List[Vec3], point: Vec3, *, tol: float = _EPS) -> None:
    p = (float(point[0]), float(point[1]), float(point[2]))
    if not points or _v_norm(_v_sub(points[-1], p)) > tol:
        points.append(p)


def _preprocess_wire_polyline_points(
    points: Iterable[Vec3],
    *,
    min_segment_length: float,
    closed_path: bool,
) -> List[Vec3]:
    raw = [(float(x), float(y), float(z)) for (x, y, z) in points]
    if len(raw) < 2:
        raise ValueError("WirePolylineGeometry requires at least two input points")

    tol = max(float(min_segment_length), _EPS)
    out: List[Vec3] = [raw[0]]
    for point in raw[1:]:
        if _v_norm(_v_sub(point, out[-1])) >= tol:
            out.append(point)

    if len(out) < 2:
        raise ValueError("Too few distinct points after dedupe")

    if closed_path:
        if _v_norm(_v_sub(out[0], out[-1])) <= tol:
            out[-1] = out[0]
        else:
            out.append(out[0])
        # Closed loops need at least three distinct vertices.
        if len(out) < 4:
            raise ValueError("Closed path requires at least three distinct points")

    return out


def _wire_corner_from_three_points(
    prev_point: Vec3,
    corner_point: Vec3,
    next_point: Vec3,
    *,
    corner_mode: str,
    corner_radius: float,
    corner_segments: int,
) -> _WireCorner:
    incoming = _v_sub(corner_point, prev_point)
    outgoing = _v_sub(next_point, corner_point)
    len_in = _v_norm(incoming)
    len_out = _v_norm(outgoing)

    if len_in <= _EPS or len_out <= _EPS:
        if corner_mode == "fillet" and corner_radius > 0.0:
            raise ValueError("Impossible fillet: corner has zero-length segment")
        return _WireCorner(corner_point, corner_point, [corner_point])

    u = _v_scale(incoming, 1.0 / len_in)
    v = _v_scale(outgoing, 1.0 / len_out)
    theta = acos(_clamp(_v_dot(_v_scale(u, -1.0), v), -1.0, 1.0))

    # Straight-through and U-turn corners fall back to miter behavior.
    if theta <= 1e-6 or abs(pi - theta) <= 1e-6:
        return _WireCorner(corner_point, corner_point, [corner_point])
    if corner_mode == "miter" or corner_radius <= 0.0:
        return _WireCorner(corner_point, corner_point, [corner_point])

    tan_half = tan(theta * 0.5)
    if abs(tan_half) <= _EPS:
        return _WireCorner(corner_point, corner_point, [corner_point])

    target_trim = corner_radius * tan_half
    if not isfinite(target_trim):
        target_trim = float("inf")
    trim = min(target_trim, 0.49 * min(len_in, len_out))
    if trim <= _EPS:
        return _WireCorner(corner_point, corner_point, [corner_point])

    a = _v_sub(corner_point, _v_scale(u, trim))
    b = _v_add(corner_point, _v_scale(v, trim))

    if corner_mode == "bevel":
        return _WireCorner(a, b, [a, b])

    radius_eff = trim / tan_half
    if radius_eff <= _EPS:
        return _WireCorner(corner_point, corner_point, [corner_point])

    bisector = _v_add(_v_scale(u, -1.0), v)
    if _v_norm(bisector) <= _EPS:
        return _WireCorner(corner_point, corner_point, [corner_point])
    bisector = _v_normalize(bisector)

    sin_half = sin(theta * 0.5)
    if abs(sin_half) <= _EPS:
        return _WireCorner(corner_point, corner_point, [corner_point])
    center_dist = radius_eff / sin_half
    center = _v_add(corner_point, _v_scale(bisector, center_dist))

    axis = _v_cross(u, v)
    if _v_norm(axis) <= _EPS:
        return _WireCorner(corner_point, corner_point, [corner_point])
    axis = _v_normalize(axis)

    ra = _v_sub(a, center)
    rb = _v_sub(b, center)
    if _v_norm(ra) <= _EPS or _v_norm(rb) <= _EPS:
        return _WireCorner(corner_point, corner_point, [corner_point])
    ra_u = _v_normalize(ra)
    rb_u = _v_normalize(rb)

    arc_angle = acos(_clamp(_v_dot(ra_u, rb_u), -1.0, 1.0))
    if _v_dot(axis, _v_cross(ra_u, rb_u)) < 0.0:
        arc_angle = -arc_angle
    if abs(arc_angle) <= _EPS:
        return _WireCorner(corner_point, corner_point, [corner_point])

    arc_points = sample_arc_3d(
        start_point=a,
        center=center,
        normal=axis,
        angle=arc_angle,
        segments=max(2, int(corner_segments)),
    )
    arc_points[0] = a
    arc_points[-1] = b
    return _WireCorner(a, b, arc_points)


def _build_wire_centerline(
    points: List[Vec3],
    *,
    closed_path: bool,
    corner_mode: str,
    corner_radius: float,
    corner_segments: int,
) -> List[Vec3]:
    if not closed_path:
        if len(points) < 2:
            raise ValueError("Wire path requires at least two points")
        if len(points) == 2:
            return list(points)

        out: List[Vec3] = []
        _append_distinct_path_point(out, points[0])
        for i in range(1, len(points) - 1):
            corner = _wire_corner_from_three_points(
                points[i - 1],
                points[i],
                points[i + 1],
                corner_mode=corner_mode,
                corner_radius=corner_radius,
                corner_segments=corner_segments,
            )
            _append_distinct_path_point(out, corner.incoming_end)
            for point in corner.bridge[1:]:
                _append_distinct_path_point(out, point)
        _append_distinct_path_point(out, points[-1])
        return out

    ring = points[:-1]
    if len(ring) < 3:
        raise ValueError("Closed path requires at least three distinct points")

    corners: List[_WireCorner] = []
    n = len(ring)
    for i in range(n):
        corners.append(
            _wire_corner_from_three_points(
                ring[(i - 1) % n],
                ring[i],
                ring[(i + 1) % n],
                corner_mode=corner_mode,
                corner_radius=corner_radius,
                corner_segments=corner_segments,
            )
        )

    out: List[Vec3] = []
    _append_distinct_path_point(out, corners[0].outgoing_start)
    for i in range(n):
        j = (i + 1) % n
        corner = corners[j]
        _append_distinct_path_point(out, corner.incoming_end)
        for point in corner.bridge[1:]:
            _append_distinct_path_point(out, point)
    if _v_norm(_v_sub(out[0], out[-1])) > _EPS:
        out.append(out[0])
    else:
        out[-1] = out[0]
    return out


def _wire_circle_profile(radius: float, radial_segments: int) -> List[Vec2]:
    return [
        (
            float(radius) * cos(2.0 * pi * i / radial_segments),
            float(radius) * sin(2.0 * pi * i / radial_segments),
        )
        for i in range(radial_segments)
    ]


def _rpy_from_z_axis(direction: Vec3) -> Vec3:
    d = _v_normalize(direction)
    yaw = atan2(d[1], d[0])
    pitch = atan2(sqrt(d[0] * d[0] + d[1] * d[1]), d[2])
    return (0.0, pitch, yaw)


class PipeGeometry(MeshGeometry):
    """
    Sweep a 2D profile along a 3D path using transported local frames.

    Unlike `SweepGeometry`, profile orientation follows the path tangent,
    which is important for wire-like and tube-like parts.
    """

    def __init__(
        self,
        profile: Iterable[Tuple[float, float]],
        path: Iterable[Tuple[float, float, float]],
        *,
        cap: bool = False,
        closed: bool = True,
        path_closed: bool = False,
        up_hint: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    ):
        super().__init__()
        raw_points = _profile_points_2d(profile)
        points = _ensure_ccw(raw_points) if closed else raw_points
        if not closed:
            cap = False
        path_points = [(float(x), float(y), float(z)) for (x, y, z) in path]
        path_points = _dedupe_path_points(path_points)
        if len(path_points) < 2:
            raise ValueError("Pipe requires at least two distinct path points")
        if path_closed:
            if _v_norm(_v_sub(path_points[0], path_points[-1])) > _EPS:
                path_points.append(path_points[0])
            cap = False

        tangents = _compute_path_tangents(path_points)
        n0, b0 = _initial_frame(
            tangents[0], (float(up_hint[0]), float(up_hint[1]), float(up_hint[2]))
        )
        normals: List[Vec3] = [n0]
        binormals: List[Vec3] = [b0]
        for i in range(1, len(path_points)):
            t = tangents[i]
            prev_n = normals[-1]
            prev_b = binormals[-1]
            # Parallel-transport-ish update: keep normal close to previous frame
            # while orthogonalizing against the new tangent.
            n = _v_sub(prev_n, _v_scale(t, _v_dot(prev_n, t)))
            if _v_norm(n) <= _EPS:
                n = _v_cross(prev_b, t)
            if _v_norm(n) <= _EPS:
                fallback = (0.0, 1.0, 0.0) if abs(t[1]) < 0.95 else (1.0, 0.0, 0.0)
                n = _v_cross(fallback, t)
            n = _v_normalize(n)
            b = _v_normalize(_v_cross(t, n))
            normals.append(n)
            binormals.append(b)
        if path_closed:
            normals[-1] = normals[0]
            binormals[-1] = binormals[0]

        profiles: List[List[Vec3]] = []
        for i, p in enumerate(path_points):
            n = normals[i]
            b = binormals[i]
            ring: List[Vec3] = []
            for x, y in points:
                ring.append(
                    (
                        p[0] + x * n[0] + y * b[0],
                        p[1] + x * n[1] + y * b[1],
                        p[2] + x * n[2] + y * b[2],
                    )
                )
            profiles.append(ring)
        ring_count = len(points)
        if ring_count < 2:
            raise ValueError("Pipe profile must contain at least 2 points")

        ring_indices: List[List[int]] = []
        for ring in profiles:
            idxs: List[int] = []
            for x, y, z in ring:
                idxs.append(self.add_vertex(x, y, z))
            ring_indices.append(idxs)

        seg_count = ring_count if closed else (ring_count - 1)
        for i in range(len(ring_indices) - 1):
            r0 = ring_indices[i]
            r1 = ring_indices[i + 1]
            for j in range(seg_count):
                j2 = (j + 1) % ring_count
                a = r0[j]
                b = r0[j2]
                c = r1[j2]
                d = r1[j]
                self.add_face(a, b, c)
                self.add_face(a, c, d)

        if cap and closed and ring_count >= 3:
            first = profiles[0]
            last = profiles[-1]
            c0 = (
                sum(v[0] for v in first) / ring_count,
                sum(v[1] for v in first) / ring_count,
                sum(v[2] for v in first) / ring_count,
            )
            c1 = (
                sum(v[0] for v in last) / ring_count,
                sum(v[1] for v in last) / ring_count,
                sum(v[2] for v in last) / ring_count,
            )
            i0 = self.add_vertex(*c0)
            i1 = self.add_vertex(*c1)
            r0 = ring_indices[0]
            r1 = ring_indices[-1]
            for j in range(ring_count):
                j2 = (j + 1) % ring_count
                self.add_face(i0, r0[j2], r0[j])
                self.add_face(i1, r1[j], r1[j2])


class ArcPipeGeometry(MeshGeometry):
    """
    Advanced compatibility helper: profile swept over a circular 3D arc.

    Prefer `sample_arc_3d(...)` + `PipeGeometry(...)` or the higher-level wire
    helpers for new code.
    """

    def __init__(
        self,
        profile: Iterable[Tuple[float, float]],
        *,
        start_point: Vec3,
        center: Vec3,
        normal: Vec3,
        angle: float,
        arc_segments: int = 20,
        cap: bool = False,
        closed: bool = True,
        up_hint: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    ):
        super().__init__()
        path = sample_arc_3d(
            start_point=start_point,
            center=center,
            normal=normal,
            angle=angle,
            segments=arc_segments,
        )
        self.merge(PipeGeometry(profile, path, cap=cap, closed=closed, up_hint=up_hint))


class WirePolylineGeometry(MeshGeometry):
    """
    Compatibility class-form wire primitive built from centerline points.

    Prefer `wire_from_points(...)` for new code. This class remains available
    when the explicit class form is useful.
    """

    def __init__(
        self,
        points: Iterable[Tuple[float, float, float]],
        *,
        radius: float,
        radial_segments: int = 16,
        closed_path: bool = False,
        cap_ends: bool = False,
        corner_mode: str = "fillet",
        corner_radius: float = 0.0,
        corner_segments: int = 8,
        up_hint: Tuple[float, float, float] = (0.0, 0.0, 1.0),
        min_segment_length: float = 1e-6,
    ):
        super().__init__()
        radius = float(radius)
        radial_segments = int(radial_segments)
        closed_path = bool(closed_path)
        corner_mode = str(corner_mode).strip().lower()
        corner_radius = float(corner_radius)
        corner_segments = int(corner_segments)
        up_hint_v = (float(up_hint[0]), float(up_hint[1]), float(up_hint[2]))
        min_segment_length = float(min_segment_length)

        if radius <= 0.0:
            raise ValueError("radius must be > 0")
        if radial_segments < 6:
            raise ValueError("radial_segments must be >= 6")
        if corner_mode not in {"fillet", "miter", "bevel"}:
            raise ValueError("corner_mode must be one of: fillet, miter, bevel")
        if corner_radius < 0.0:
            raise ValueError("corner_radius must be >= 0")
        if corner_mode == "fillet" and corner_radius > 0.0 and corner_segments < 2:
            raise ValueError("corner_segments must be >= 2 when fillets are active")
        if _v_norm(up_hint_v) <= _EPS:
            raise ValueError("up_hint must be non-zero")
        if min_segment_length <= 0.0:
            raise ValueError("min_segment_length must be > 0")

        preprocessed = _preprocess_wire_polyline_points(
            points,
            min_segment_length=min_segment_length,
            closed_path=closed_path,
        )
        centerline = _build_wire_centerline(
            preprocessed,
            closed_path=closed_path,
            corner_mode=corner_mode,
            corner_radius=corner_radius,
            corner_segments=corner_segments,
        )
        centerline = _dedupe_path_points(centerline)
        if len(centerline) < 2:
            raise ValueError("Too few distinct points after corner preprocessing")

        profile = _wire_circle_profile(radius, radial_segments)
        self.merge(
            PipeGeometry(
                profile,
                centerline,
                cap=bool(cap_ends and not closed_path),
                closed=True,
                path_closed=closed_path,
                up_hint=up_hint_v,
            )
        )


def wire_from_points(
    points: Iterable[Tuple[float, float, float]],
    *,
    radius: float,
    radial_segments: int = 16,
    closed_path: bool = False,
    cap_ends: bool = False,
    corner_mode: str = "fillet",
    corner_radius: float = 0.0,
    corner_segments: int = 8,
    up_hint: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    min_segment_length: float = 1e-6,
) -> MeshGeometry:
    """Build one tube/wire mesh from a centerline path and radius."""
    return WirePolylineGeometry(
        points=points,
        radius=radius,
        radial_segments=radial_segments,
        closed_path=closed_path,
        cap_ends=cap_ends,
        corner_mode=corner_mode,
        corner_radius=corner_radius,
        corner_segments=corner_segments,
        up_hint=up_hint,
        min_segment_length=min_segment_length,
    )


def _sample_supported_spline_path(
    points: Iterable[Vec3],
    *,
    spline: str,
    samples_per_segment: int,
    closed_spline: bool,
    alpha: float,
    min_segment_length: float,
) -> List[Vec3]:
    spline_key = str(spline or "catmull_rom").strip().lower().replace("-", "_")
    if spline_key in {"catmullrom", "catmull_rom"}:
        sampled = sample_catmull_rom_spline_3d(
            list(points),
            samples_per_segment=int(samples_per_segment),
            closed=bool(closed_spline),
            alpha=float(alpha),
        )
    elif spline_key in {"bezier", "cubic_bezier", "cubicbezier"}:
        sampled = sample_cubic_bezier_spline_3d(
            list(points),
            samples_per_segment=int(samples_per_segment),
        )
        if bool(closed_spline) and _v_norm(_v_sub(sampled[0], sampled[-1])) > max(
            float(min_segment_length), _EPS
        ):
            raise ValueError(
                "closed_spline=True with spline='bezier' requires the sampled "
                "Bezier chain to end where it starts"
            )
    else:
        raise ValueError("spline must be one of: 'catmull_rom', 'bezier'")

    return _preprocess_wire_polyline_points(
        sampled,
        min_segment_length=float(min_segment_length),
        closed_path=bool(closed_spline),
    )


def tube_from_spline_points(
    points: Iterable[Vec3],
    *,
    radius: float,
    samples_per_segment: int = 12,
    closed_spline: bool = False,
    spline: str = "catmull_rom",
    alpha: float = 0.5,
    radial_segments: int = 16,
    cap_ends: bool = True,
    up_hint: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    min_segment_length: float = 1e-6,
) -> MeshGeometry:
    """
    Fit a spline through points, then build a circular tube along that path.

    This is the default "click points, fit a nice handle/tube" helper.
    Supported spline families: ``catmull_rom`` and chained cubic ``bezier``.
    """
    centerline = _sample_supported_spline_path(
        points,
        spline=spline,
        samples_per_segment=samples_per_segment,
        closed_spline=closed_spline,
        alpha=alpha,
        min_segment_length=min_segment_length,
    )
    return wire_from_points(
        centerline,
        radius=float(radius),
        radial_segments=int(radial_segments),
        closed_path=bool(closed_spline),
        cap_ends=bool(cap_ends),
        corner_mode="miter",
        up_hint=up_hint,
        min_segment_length=min_segment_length,
    )


def sweep_profile_along_spline(
    points: Iterable[Vec3],
    *,
    profile: Iterable[Vec2],
    samples_per_segment: int = 12,
    closed_spline: bool = False,
    spline: str = "catmull_rom",
    alpha: float = 0.5,
    cap_profile: bool = True,
    up_hint: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    min_segment_length: float = 1e-6,
) -> MeshGeometry:
    """
    Fit a spline through points, then sweep a 2D profile along the sampled path.

    The profile is treated as a closed outline. Use `tube_from_spline_points(...)`
    for the common circular-tube case. Supported spline families:
    ``catmull_rom`` and chained cubic ``bezier``.
    """
    centerline = _sample_supported_spline_path(
        points,
        spline=spline,
        samples_per_segment=samples_per_segment,
        closed_spline=closed_spline,
        alpha=alpha,
        min_segment_length=min_segment_length,
    )
    return PipeGeometry(
        profile=profile,
        path=centerline,
        cap=bool(cap_profile and not closed_spline),
        closed=True,
        path_closed=bool(closed_spline),
        up_hint=up_hint,
    )


def _vec3_quant_key(point: Vec3, *, tol: float) -> Tuple[int, int, int]:
    scale = 1.0 / max(float(tol), _EPS)
    return (
        int(round(point[0] * scale)),
        int(round(point[1] * scale)),
        int(round(point[2] * scale)),
    )


def _coerce_tube_network_paths(paths: Iterable[Iterable[Vec3]]) -> List[List[Vec3]]:
    out: List[List[Vec3]] = []
    for path in paths:
        pts = [(float(x), float(y), float(z)) for (x, y, z) in path]
        if len(pts) < 2:
            raise ValueError(
                "tube_network_from_paths requires each path to contain at least two points"
            )
        out.append(pts)
    if not out:
        raise ValueError("tube_network_from_paths requires at least one path")
    return out


def _is_closed_network_path(path: Sequence[Vec3], *, tol: float) -> bool:
    return len(path) >= 3 and _v_norm(_v_sub(path[0], path[-1])) <= max(float(tol), _EPS)


def _collect_shared_network_nodes(paths: Sequence[Sequence[Vec3]], *, tol: float) -> List[Vec3]:
    counts: dict[Tuple[int, int, int], int] = {}
    canonical: dict[Tuple[int, int, int], Vec3] = {}
    for path in paths:
        seen: set[Tuple[int, int, int]] = set()
        for point in path:
            key = _vec3_quant_key(point, tol=tol)
            canonical.setdefault(key, (float(point[0]), float(point[1]), float(point[2])))
            seen.add(key)
        for key in seen:
            counts[key] = counts.get(key, 0) + 1
    return [canonical[key] for key, count in counts.items() if count > 1]


def _segment_cylinder_geometry(
    start: Vec3,
    end: Vec3,
    *,
    radius: float,
    radial_segments: int,
) -> Optional[MeshGeometry]:
    segment = _v_sub(end, start)
    length = _v_norm(segment)
    if length <= _EPS:
        return None
    mid = _v_lerp(start, end, 0.5)
    _roll, pitch, yaw = _rpy_from_z_axis(segment)
    return (
        CylinderGeometry(
            radius=float(radius),
            height=length,
            radial_segments=max(6, int(radial_segments)),
            closed=True,
        )
        .rotate_y(pitch)
        .rotate_z(yaw)
        .translate(*mid)
    )


def _tube_network_visual_mesh_from_paths(
    path_list: Sequence[Sequence[Vec3]],
    *,
    radius: float,
    radial_segments: int,
    cap_ends: bool,
    corner_mode: str,
    corner_radius: float,
    corner_segments: int,
    up_hint: Tuple[float, float, float],
    min_segment_length: float,
    shared_node_radius: float,
) -> MeshGeometry:
    out = MeshGeometry()
    for path in path_list:
        closed_path = _is_closed_network_path(path, tol=min_segment_length)
        out.merge(
            wire_from_points(
                path,
                radius=radius,
                radial_segments=radial_segments,
                closed_path=closed_path,
                cap_ends=cap_ends,
                corner_mode=corner_mode,
                corner_radius=corner_radius,
                corner_segments=corner_segments,
                up_hint=up_hint,
                min_segment_length=min_segment_length,
            )
        )

    if shared_node_radius > 0.0:
        width_segments = max(12, int(radial_segments))
        height_segments = max(8, int(radial_segments) // 2)
        for point in _collect_shared_network_nodes(path_list, tol=min_segment_length):
            out.merge(
                SphereGeometry(
                    radius=shared_node_radius,
                    width_segments=width_segments,
                    height_segments=height_segments,
                ).translate(*point)
            )
    return out


def tube_network_from_paths(
    paths: Iterable[Iterable[Vec3]],
    *,
    radius: float,
    radial_segments: int = 16,
    cap_ends: bool = True,
    corner_mode: str = "fillet",
    corner_radius: float = 0.0,
    corner_segments: int = 8,
    up_hint: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    min_segment_length: float = 1e-6,
    shared_node_radius: Optional[float] = None,
) -> MeshGeometry:
    """
    Build a tubular frame/network from multiple centerline paths.

    This helper aims to return a watertight solid by boolean-unioning closed
    segment cylinders and optional node spheres. If the boolean backend is not
    available, it falls back to the older merge-based visual mesh construction.
    """
    radius = float(radius)
    node_radius = radius if shared_node_radius is None else float(shared_node_radius)
    tol = max(float(min_segment_length), _EPS)
    path_list = _coerce_tube_network_paths(paths)

    if radius <= 0.0:
        raise ValueError("radius must be > 0")
    if node_radius < 0.0:
        raise ValueError("shared_node_radius must be >= 0")

    solids: List[MeshGeometry] = []
    for path in path_list:
        closed_path = _is_closed_network_path(path, tol=tol)
        preprocessed = _preprocess_wire_polyline_points(
            path,
            min_segment_length=min_segment_length,
            closed_path=closed_path,
        )
        centerline = _build_wire_centerline(
            preprocessed,
            closed_path=closed_path,
            corner_mode=corner_mode,
            corner_radius=corner_radius,
            corner_segments=corner_segments,
        )
        centerline = _dedupe_path_points(centerline)
        for i in range(len(centerline) - 1):
            segment_geom = _segment_cylinder_geometry(
                centerline[i],
                centerline[i + 1],
                radius=radius,
                radial_segments=radial_segments,
            )
            if segment_geom is not None:
                solids.append(segment_geom)

    if node_radius > 0.0 and cap_ends:
        width_segments = max(12, int(radial_segments))
        height_segments = max(8, int(radial_segments) // 2)
        node_keys: set[Tuple[int, int, int]] = set()
        for path in path_list:
            for point in path:
                node_keys.add(_vec3_quant_key(point, tol=tol))
        canonical = {
            _vec3_quant_key(point, tol=tol): (float(point[0]), float(point[1]), float(point[2]))
            for path in path_list
            for point in path
        }
        for key in sorted(node_keys):
            solids.append(
                SphereGeometry(
                    radius=node_radius,
                    width_segments=width_segments,
                    height_segments=height_segments,
                ).translate(*canonical[key])
            )

    if not solids:
        raise ValueError("tube_network_from_paths produced no solid segments")

    boolean_union_many, expected_fallback_error, visual_fallback = _tube_network_compat_hooks()

    try:
        out = boolean_union_many(solids)
    except Exception as exc:
        if not expected_fallback_error(exc):
            raise
        logger.warning(
            "tube_network_from_paths falling back to visual mesh after boolean union failure: %s: %s",
            type(exc).__name__,
            exc,
        )
        out = visual_fallback(
            path_list,
            radius=radius,
            radial_segments=radial_segments,
            cap_ends=cap_ends,
            corner_mode=corner_mode,
            corner_radius=corner_radius,
            corner_segments=corner_segments,
            up_hint=up_hint,
            min_segment_length=min_segment_length,
            shared_node_radius=node_radius if cap_ends else 0.0,
        )
    return out
