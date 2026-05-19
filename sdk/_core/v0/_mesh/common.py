from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from math import acos, asin, atan2, cos, isfinite, pi, sin, sqrt, tan
from pathlib import Path
from typing import Iterable, List, Literal, Optional, Sequence, Tuple, Union

import manifold3d as _m3d
import numpy as np

from sdk._dependencies import require_cadquery

from ..assets import get_active_asset_session
from ..types import Box, Cylinder, Mesh, Sphere

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]
Face = Tuple[int, int, int]
Mat4 = Tuple[Tuple[float, float, float, float], ...]
LatheCapMode = Literal["flat", "round"]
_EPS = 1e-9
_OBJ_COORD_DECIMALS = 6
_OBJ_QUANT_STEP = 10.0 ** (-_OBJ_COORD_DECIMALS)
_BRIDGE_EPSILON = 0.1 * _OBJ_QUANT_STEP
logger = logging.getLogger(__name__)


def _identity_mat4() -> Mat4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _mat4_mul(a: Mat4, b: Mat4) -> Mat4:
    rows: list[tuple[float, float, float, float]] = []
    for i in range(4):
        row = []
        for j in range(4):
            row.append(sum(float(a[i][k]) * float(b[k][j]) for k in range(4)))
        rows.append((row[0], row[1], row[2], row[3]))
    return (rows[0], rows[1], rows[2], rows[3])


def _set_primitive_provenance(geometry: "MeshGeometry", primitive: Box | Cylinder | Sphere) -> None:
    setattr(geometry, "_primitive_source", primitive)
    setattr(geometry, "_primitive_transform", _identity_mat4())


def _clear_primitive_provenance(geometry: "MeshGeometry") -> None:
    if hasattr(geometry, "_primitive_source"):
        delattr(geometry, "_primitive_source")
    if hasattr(geometry, "_primitive_transform"):
        delattr(geometry, "_primitive_transform")


def _copy_primitive_provenance(src: "MeshGeometry", dst: "MeshGeometry") -> None:
    primitive = _primitive_source_geometry(src)
    if primitive is None:
        return
    setattr(dst, "_primitive_source", primitive)
    transform = _primitive_source_transform(src)
    if transform is not None:
        setattr(dst, "_primitive_transform", transform)


def _set_manifold_provenance(geometry: "MeshGeometry", manifold) -> None:
    setattr(geometry, "_manifold_source", manifold)


def _clear_manifold_provenance(geometry: "MeshGeometry") -> None:
    if hasattr(geometry, "_manifold_source"):
        delattr(geometry, "_manifold_source")


def _copy_manifold_provenance(src: "MeshGeometry", dst: "MeshGeometry") -> None:
    manifold = getattr(src, "_manifold_source", None)
    if manifold is not None:
        setattr(dst, "_manifold_source", manifold)


def _prepend_primitive_transform(geometry: "MeshGeometry", transform: Mat4) -> None:
    primitive = _primitive_source_geometry(geometry)
    if primitive is None:
        return
    current = _primitive_source_transform(geometry) or _identity_mat4()
    setattr(geometry, "_primitive_transform", _mat4_mul(transform, current))


def _primitive_source_geometry(geometry: "MeshGeometry") -> Box | Cylinder | Sphere | None:
    primitive = getattr(geometry, "_primitive_source", None)
    if isinstance(primitive, (Box, Cylinder, Sphere)):
        return primitive
    return None


def _primitive_source_transform(geometry: "MeshGeometry") -> Mat4 | None:
    transform = getattr(geometry, "_primitive_transform", None)
    if isinstance(transform, tuple) and len(transform) == 4:
        return transform  # type: ignore[return-value]
    return None


def _clone_primitive_geometry(geometry: Box | Cylinder | Sphere) -> Box | Cylinder | Sphere:
    if isinstance(geometry, Box):
        return Box(tuple(float(v) for v in geometry.size))
    if isinstance(geometry, Cylinder):
        return Cylinder(radius=float(geometry.radius), length=float(geometry.length))
    return Sphere(radius=float(geometry.radius))


def _mat4_vec3(mat: Mat4, vec: Vec3) -> Vec3:
    x, y, z = vec
    return (
        float(mat[0][0]) * x + float(mat[0][1]) * y + float(mat[0][2]) * z + float(mat[0][3]),
        float(mat[1][0]) * x + float(mat[1][1]) * y + float(mat[1][2]) * z + float(mat[1][3]),
        float(mat[2][0]) * x + float(mat[2][1]) * y + float(mat[2][2]) * z + float(mat[2][3]),
    )


def _rpy_from_matrix(
    mat: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]],
) -> Vec3:
    pitch = asin(max(-1.0, min(1.0, -float(mat[2][0]))))
    cp = cos(pitch)
    if abs(cp) > 1e-6:
        roll = atan2(float(mat[2][1]), float(mat[2][2]))
        yaw = atan2(float(mat[1][0]), float(mat[0][0]))
    else:
        roll = 0.0
        yaw = atan2(-float(mat[0][1]), float(mat[1][1]))
    return (roll, pitch, yaw)


def sample_cubic_bezier_spline_2d(
    control_points: Sequence[Vec2],
    *,
    samples_per_segment: int = 12,
) -> List[Vec2]:
    """
    Sample a C1-continuous cubic Bezier spline in 2D.

    The control-point layout follows chained cubic segments:
    ``[P0, P1, P2, P3, P4, P5, P6, ...]`` where each additional segment
    contributes three points and starts at the previous segment's end.

    This is useful for "rail" curves that drive non-uniform lofts.
    """
    pts = [(float(x), float(y)) for (x, y) in control_points]
    if len(pts) < 4:
        raise ValueError("Bezier spline requires at least 4 control points")
    if (len(pts) - 1) % 3 != 0:
        raise ValueError("Bezier spline control points must satisfy (n-1) % 3 == 0")

    seg_count = (len(pts) - 1) // 3
    steps = max(2, int(samples_per_segment))

    def eval_seg(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: float) -> Vec2:
        u = 1.0 - t
        b0 = u * u * u
        b1 = 3.0 * u * u * t
        b2 = 3.0 * u * t * t
        b3 = t * t * t
        x = b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0]
        y = b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]
        return (x, y)

    out: List[Vec2] = []
    for seg in range(seg_count):
        i = seg * 3
        p0, p1, p2, p3 = pts[i], pts[i + 1], pts[i + 2], pts[i + 3]
        for j in range(steps):
            if seg > 0 and j == 0:
                continue
            t = float(j) / float(steps)
            out.append(eval_seg(p0, p1, p2, p3, t))
    out.append(pts[-1])
    return out


def sample_cubic_bezier_spline_3d(
    control_points: Sequence[Vec3],
    *,
    samples_per_segment: int = 12,
) -> List[Vec3]:
    """
    Sample a C1-continuous cubic Bezier spline in 3D.

    Control-point layout follows chained cubic segments:
    ``[P0, P1, P2, P3, P4, P5, P6, ...]``.
    """
    pts = [(float(x), float(y), float(z)) for (x, y, z) in control_points]
    if len(pts) < 4:
        raise ValueError("Bezier spline requires at least 4 control points")
    if (len(pts) - 1) % 3 != 0:
        raise ValueError("Bezier spline control points must satisfy (n-1) % 3 == 0")

    seg_count = (len(pts) - 1) // 3
    steps = max(2, int(samples_per_segment))

    def eval_seg(p0: Vec3, p1: Vec3, p2: Vec3, p3: Vec3, t: float) -> Vec3:
        u = 1.0 - t
        b0 = u * u * u
        b1 = 3.0 * u * u * t
        b2 = 3.0 * u * t * t
        b3 = t * t * t
        x = b0 * p0[0] + b1 * p1[0] + b2 * p2[0] + b3 * p3[0]
        y = b0 * p0[1] + b1 * p1[1] + b2 * p2[1] + b3 * p3[1]
        z = b0 * p0[2] + b1 * p1[2] + b2 * p2[2] + b3 * p3[2]
        return (x, y, z)

    out: List[Vec3] = []
    for seg in range(seg_count):
        i = seg * 3
        p0, p1, p2, p3 = pts[i], pts[i + 1], pts[i + 2], pts[i + 3]
        for j in range(steps):
            if seg > 0 and j == 0:
                continue
            t = float(j) / float(steps)
            out.append(eval_seg(p0, p1, p2, p3, t))
    out.append(pts[-1])
    return out


def _tuple_lerp(a: Sequence[float], b: Sequence[float], t: float) -> Tuple[float, ...]:
    return tuple(float(xa) + (float(xb) - float(xa)) * float(t) for xa, xb in zip(a, b))


def _tuple_extrapolate(a: Sequence[float], b: Sequence[float]) -> Tuple[float, ...]:
    return tuple(2.0 * float(xa) - float(xb) for xa, xb in zip(a, b))


def _tuple_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return sqrt(sum((float(xa) - float(xb)) * (float(xa) - float(xb)) for xa, xb in zip(a, b)))


def _catmull_rom_interp(
    a: Sequence[float],
    b: Sequence[float],
    ta: float,
    tb: float,
    t: float,
) -> Tuple[float, ...]:
    if abs(tb - ta) <= _EPS:
        return tuple(float(v) for v in a)
    wa = (tb - t) / (tb - ta)
    wb = (t - ta) / (tb - ta)
    return tuple(wa * float(va) + wb * float(vb) for va, vb in zip(a, b))


def _sample_catmull_rom_segment(
    p0: Sequence[float],
    p1: Sequence[float],
    p2: Sequence[float],
    p3: Sequence[float],
    *,
    steps: int,
    alpha: float,
) -> List[Tuple[float, ...]]:
    def tj(ti: float, pa: Sequence[float], pb: Sequence[float]) -> float:
        dist = max(_tuple_distance(pa, pb), _EPS)
        return ti + dist**alpha

    t0 = 0.0
    t1 = tj(t0, p0, p1)
    t2 = tj(t1, p1, p2)
    t3 = tj(t2, p2, p3)

    out: List[Tuple[float, ...]] = []
    for j in range(steps):
        t = t1 + (t2 - t1) * (float(j) / float(steps))
        a1 = _catmull_rom_interp(p0, p1, t0, t1, t)
        a2 = _catmull_rom_interp(p1, p2, t1, t2, t)
        a3 = _catmull_rom_interp(p2, p3, t2, t3, t)
        b1 = _catmull_rom_interp(a1, a2, t0, t2, t)
        b2 = _catmull_rom_interp(a2, a3, t1, t3, t)
        out.append(_catmull_rom_interp(b1, b2, t1, t2, t))
    return out


def _sample_catmull_rom_spline(
    points: Sequence[Sequence[float]],
    *,
    samples_per_segment: int,
    closed: bool,
    alpha: float,
) -> List[Tuple[float, ...]]:
    pts = [tuple(float(v) for v in point) for point in points]
    if len(pts) < 2:
        raise ValueError("Catmull-Rom spline requires at least two points")
    if not (0.0 <= float(alpha) <= 1.0):
        raise ValueError("Catmull-Rom alpha must be in [0, 1]")

    dedup: List[Tuple[float, ...]] = [pts[0]]
    for point in pts[1:]:
        if _tuple_distance(point, dedup[-1]) > _EPS:
            dedup.append(point)

    if len(dedup) < 2:
        raise ValueError("Catmull-Rom spline requires at least two distinct points")

    steps = max(2, int(samples_per_segment))

    if closed:
        ring = list(dedup)
        if _tuple_distance(ring[0], ring[-1]) <= _EPS:
            ring.pop()
        if len(ring) < 3:
            raise ValueError("Closed Catmull-Rom spline requires at least three distinct points")

        out: List[Tuple[float, ...]] = []
        n = len(ring)
        for i in range(n):
            seg = _sample_catmull_rom_segment(
                ring[(i - 1) % n],
                ring[i],
                ring[(i + 1) % n],
                ring[(i + 2) % n],
                steps=steps,
                alpha=float(alpha),
            )
            out.extend(seg)
        if not out:
            raise ValueError("Closed Catmull-Rom spline produced no samples")
        out.append(out[0])
        return out

    if len(dedup) == 2:
        out = [_tuple_lerp(dedup[0], dedup[1], float(j) / float(steps)) for j in range(steps)]
        out.append(dedup[-1])
        return out

    extended = [
        _tuple_extrapolate(dedup[0], dedup[1]),
        *dedup,
        _tuple_extrapolate(dedup[-1], dedup[-2]),
    ]
    out = []
    for i in range(len(dedup) - 1):
        seg = _sample_catmull_rom_segment(
            extended[i],
            extended[i + 1],
            extended[i + 2],
            extended[i + 3],
            steps=steps,
            alpha=float(alpha),
        )
        out.extend(seg)
    out.append(dedup[-1])
    return out


def sample_catmull_rom_spline_2d(
    points: Sequence[Vec2],
    *,
    samples_per_segment: int = 12,
    closed: bool = False,
    alpha: float = 0.5,
) -> List[Vec2]:
    """
    Sample a centripetal Catmull-Rom spline that passes through the input points.

    This is the recommended helper when you want a smooth spline fitted through
    clicked/estimated points instead of chained Bezier control handles.
    """
    sampled = _sample_catmull_rom_spline(
        points,
        samples_per_segment=samples_per_segment,
        closed=bool(closed),
        alpha=float(alpha),
    )
    return [(float(x), float(y)) for (x, y) in sampled]


def sample_catmull_rom_spline_3d(
    points: Sequence[Vec3],
    *,
    samples_per_segment: int = 12,
    closed: bool = False,
    alpha: float = 0.5,
) -> List[Vec3]:
    """
    Sample a centripetal Catmull-Rom spline that passes through the input points.

    This is the recommended helper for tube centerlines and other fitted 3D paths.
    """
    sampled = _sample_catmull_rom_spline(
        points,
        samples_per_segment=samples_per_segment,
        closed=bool(closed),
        alpha=float(alpha),
    )
    return [(float(x), float(y), float(z)) for (x, y, z) in sampled]


def _v_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _v_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _v_cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _v_norm(v: Vec3) -> float:
    return sqrt(_v_dot(v, v))


def _v_normalize(v: Vec3) -> Vec3:
    n = _v_norm(v)
    if n <= _EPS:
        raise ValueError("Cannot normalize near-zero vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def _v_rotate_rodrigues(v: Vec3, axis: Vec3, angle: float) -> Vec3:
    k = _v_normalize(axis)
    c = cos(float(angle))
    s = sin(float(angle))
    term1 = _v_scale(v, c)
    term2 = _v_scale(_v_cross(k, v), s)
    term3 = _v_scale(k, _v_dot(k, v) * (1.0 - c))
    return _v_add(_v_add(term1, term2), term3)


def _v_lerp(a: Vec3, b: Vec3, t: float) -> Vec3:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _clamp(x: float, low: float, high: float) -> float:
    return max(low, min(high, x))


def sample_arc_3d(
    *,
    start_point: Vec3,
    center: Vec3,
    normal: Vec3,
    angle: float,
    segments: int = 16,
) -> List[Vec3]:
    """
    Sample a circular 3D arc from `start_point` around axis (`center`, `normal`).

    `angle` is in radians and follows right-hand rule around `normal`.
    """
    start = (float(start_point[0]), float(start_point[1]), float(start_point[2]))
    ctr = (float(center[0]), float(center[1]), float(center[2]))
    n = _v_normalize((float(normal[0]), float(normal[1]), float(normal[2])))
    steps = max(2, int(segments))

    radius_vec = _v_sub(start, ctr)
    # Arc radius vector must be perpendicular to axis.
    axial = abs(_v_dot(_v_normalize(radius_vec), n)) if _v_norm(radius_vec) > _EPS else 1.0
    if _v_norm(radius_vec) <= _EPS or axial > 1.0 - 1e-6:
        raise ValueError("start_point must be away from center and not collinear with normal")

    out: List[Vec3] = []
    for i in range(steps + 1):
        t = float(i) / float(steps)
        rv = _v_rotate_rodrigues(radius_vec, n, float(angle) * t)
        out.append(_v_add(ctr, rv))
    return out


@dataclass
class WirePath:
    points: List[Vec3]

    def __init__(self, start: Vec3):
        s = (float(start[0]), float(start[1]), float(start[2]))
        self.points = [s]

    @classmethod
    def from_points(cls, points: Iterable[Vec3]) -> "WirePath":
        pts = [(float(x), float(y), float(z)) for (x, y, z) in points]
        if len(pts) < 1:
            raise ValueError("WirePath.from_points requires at least one point")
        out = cls(pts[0])
        for p in pts[1:]:
            out.line_to(p)
        return out

    def _append_if_distinct(self, p: Vec3) -> None:
        last = self.points[-1]
        if _v_norm(_v_sub(last, p)) > _EPS:
            self.points.append((float(p[0]), float(p[1]), float(p[2])))

    def line_to(self, point: Vec3) -> "WirePath":
        p = (float(point[0]), float(point[1]), float(point[2]))
        self._append_if_distinct(p)
        return self

    def line_by(self, dx: float, dy: float, dz: float) -> "WirePath":
        cur = self.points[-1]
        self._append_if_distinct((cur[0] + float(dx), cur[1] + float(dy), cur[2] + float(dz)))
        return self

    def bezier_to(
        self,
        control1: Vec3,
        control2: Vec3,
        end: Vec3,
        *,
        samples: int = 12,
    ) -> "WirePath":
        p0 = self.points[-1]
        pts = sample_cubic_bezier_spline_3d(
            [p0, control1, control2, end],
            samples_per_segment=max(2, int(samples)),
        )
        for p in pts[1:]:
            self._append_if_distinct(p)
        return self

    def arc(
        self,
        *,
        center: Vec3,
        normal: Vec3,
        angle: float,
        segments: int = 16,
    ) -> "WirePath":
        pts = sample_arc_3d(
            start_point=self.points[-1],
            center=center,
            normal=normal,
            angle=angle,
            segments=segments,
        )
        for p in pts[1:]:
            self._append_if_distinct(p)
        return self

    def extend(self, points: Iterable[Vec3]) -> "WirePath":
        for p in points:
            self.line_to(p)
        return self

    def to_points(self) -> List[Vec3]:
        return list(self.points)


def superellipse_side_loft(
    sections: Sequence[Tuple[float, float, float, float]],
    *,
    exponents: Union[float, Sequence[float]] = 2.8,
    segments: int = 56,
    cap: bool = True,
    closed: bool = True,
    min_height: float = 1e-4,
    min_width: float = 1e-4,
) -> "MeshGeometry":
    """
    Build an organic casing from side-profile rails.

    Parameters
    ----------
    sections:
        Ordered tuples ``(y, z_min, z_max, width)`` in the *final* frame.
        The loft axis is +Y, and each cross-section is a superellipse in XZ.
    exponents:
        Superellipse exponent(s). A single float applies to all sections, or
        pass one value per section.

    Notes
    -----
    Internally this uses ``LoftGeometry``'s XY-planar profile convention and
    rotates into XZ/Y world orientation. This avoids repeated ad-hoc axis
    remapping in object scripts.
    """
    if len(sections) < 2:
        raise ValueError("superellipse_side_loft requires at least two sections")

    if isinstance(exponents, (int, float)):
        expo_list = [float(exponents)] * len(sections)
    else:
        expo_list = [float(v) for v in exponents]
        if len(expo_list) != len(sections):
            raise ValueError("exponents must match section count")

    profiles: List[List[Vec3]] = []
    segs = max(12, int(segments))
    min_h = max(1e-9, float(min_height))
    min_w = max(1e-9, float(min_width))

    for i, sec in enumerate(sections):
        y, z_min, z_max, width = sec
        y = float(y)
        z0 = float(z_min)
        z1 = float(z_max)
        if z1 < z0:
            z0, z1 = z1, z0

        height = max(min_h, z1 - z0)
        width = max(min_w, float(width))
        z_center = 0.5 * (z0 + z1)
        exponent = max(0.2, float(expo_list[i]))

        prof_xy = superellipse_profile(width, height, exponent=exponent, segments=segs)
        # Map final (x, y_section, z) into LoftGeometry's expected frame:
        # pre-rotate point is (x, -(z), y_section), then rotate_x(-90deg).
        prof_xyz = [(x, -(z_local + z_center), y) for (x, z_local) in prof_xy]
        profiles.append(prof_xyz)

    from .primitives import LoftGeometry

    geom = LoftGeometry(profiles, cap=cap, closed=closed)
    geom.rotate_x(-pi / 2.0)
    return geom


def split_superellipse_side_loft(
    sections: Sequence[Tuple[float, float, float, float]],
    *,
    split_y: float,
    exponents: Union[float, Sequence[float]] = 2.8,
    segments: int = 56,
    cap: bool = True,
    closed: bool = True,
    min_height: float = 1e-4,
    min_width: float = 1e-4,
) -> Tuple["MeshGeometry", "MeshGeometry", Tuple[float, float, float, float]]:
    """
    Split one continuous side-loft shell into two watertight halves at ``split_y``.

    This function is intended for articulated shells (for example a stand-mixer
    neck/head) where both links should share exactly the same seam profile. The
    seam rail is interpolated from neighboring source rails when needed.

    Returns
    -------
    tuple
        ``(rear_geom, front_geom, seam_section)`` where ``seam_section`` is the
        rail tuple ``(y, z_min, z_max, width)`` shared by both outputs.
    """
    raw = [(float(y), float(z0), float(z1), float(w)) for (y, z0, z1, w) in sections]
    if len(raw) < 3:
        raise ValueError("split_superellipse_side_loft requires at least three sections")

    raw.sort(key=lambda row: row[0])
    split = float(split_y)
    y_min = raw[0][0]
    y_max = raw[-1][0]
    if not (y_min < split < y_max):
        raise ValueError(f"split_y must be strictly inside section range ({y_min}, {y_max})")

    if isinstance(exponents, (int, float)):
        expo_list = [float(exponents)] * len(raw)
    else:
        expo_list = [float(v) for v in exponents]
        if len(expo_list) != len(raw):
            raise ValueError("exponents must match section count")

    # Find seam section and aligned exponent.
    seam_tol = 1e-9
    seam_section: Optional[Tuple[float, float, float, float]] = None
    seam_exponent: Optional[float] = None
    seam_idx: Optional[int] = None
    for i, row in enumerate(raw):
        if abs(row[0] - split) <= seam_tol:
            seam_section = row
            seam_exponent = expo_list[i]
            seam_idx = i
            break

    if seam_section is None:
        insert_i = None
        for i in range(len(raw) - 1):
            y0 = raw[i][0]
            y1 = raw[i + 1][0]
            if y0 <= split <= y1:
                insert_i = i
                break
        if insert_i is None:
            raise ValueError("Unable to locate split interval")

        y0, z00, z01, w0 = raw[insert_i]
        y1, z10, z11, w1 = raw[insert_i + 1]
        if abs(y1 - y0) <= seam_tol:
            raise ValueError("Adjacent sections have identical y; cannot interpolate seam")
        t = (split - y0) / (y1 - y0)

        z0 = z00 + (z10 - z00) * t
        z1 = z01 + (z11 - z01) * t
        w = w0 + (w1 - w0) * t
        seam_section = (split, z0, z1, w)
        seam_exponent = expo_list[insert_i] + (expo_list[insert_i + 1] - expo_list[insert_i]) * t
        seam_idx = insert_i

    assert seam_section is not None
    assert seam_exponent is not None
    assert seam_idx is not None

    # Build aligned rear/front rails with shared seam.
    lower_sections: List[Tuple[float, float, float, float]] = []
    lower_exp: List[float] = []
    upper_sections: List[Tuple[float, float, float, float]] = []
    upper_exp: List[float] = []

    # Existing seam at raw[seam_idx].
    if abs(raw[seam_idx][0] - split) <= seam_tol:
        for i in range(seam_idx + 1):
            lower_sections.append(raw[i])
            lower_exp.append(expo_list[i])
        for i in range(seam_idx, len(raw)):
            upper_sections.append(raw[i])
            upper_exp.append(expo_list[i])
    else:
        for i in range(seam_idx + 1):
            lower_sections.append(raw[i])
            lower_exp.append(expo_list[i])
        lower_sections.append(seam_section)
        lower_exp.append(seam_exponent)

        upper_sections.append(seam_section)
        upper_exp.append(seam_exponent)
        for i in range(seam_idx + 1, len(raw)):
            upper_sections.append(raw[i])
            upper_exp.append(expo_list[i])

    if len(lower_sections) < 2 or len(upper_sections) < 2:
        raise ValueError("split produced an invalid half with fewer than two sections")

    rear_geom = superellipse_side_loft(
        lower_sections,
        exponents=lower_exp,
        segments=segments,
        cap=cap,
        closed=closed,
        min_height=min_height,
        min_width=min_width,
    )
    front_geom = superellipse_side_loft(
        upper_sections,
        exponents=upper_exp,
        segments=segments,
        cap=cap,
        closed=closed,
        min_height=min_height,
        min_width=min_width,
    )
    return rear_geom, front_geom, seam_section


def resample_side_sections(
    sections: Sequence[Tuple[float, float, float, float]],
    *,
    samples_per_span: int = 2,
    smooth_passes: int = 0,
    min_height: float = 1e-4,
    min_width: float = 1e-4,
) -> List[Tuple[float, float, float, float]]:
    """
    Densify and optionally smooth side-loft sections.

    This helper is useful when source rails are sparse/noisy (for example when
    they are estimated from mesh slices). It linearly resamples each span in Y,
    then applies a light 3-point smoothing kernel to ``z_min``, ``z_max``, and
    ``width`` while keeping ``y`` fixed and monotonic.
    """
    raw = [(float(y), float(z0), float(z1), float(w)) for (y, z0, z1, w) in sections]
    if len(raw) < 2:
        raise ValueError("resample_side_sections requires at least two sections")

    raw.sort(key=lambda row: row[0])

    # Merge duplicate y samples by averaging rails.
    merged: List[Tuple[float, float, float, float]] = []
    for row in raw:
        if merged and abs(row[0] - merged[-1][0]) <= 1e-9:
            y0, a0, a1, aw = merged[-1]
            merged[-1] = (
                y0,
                0.5 * (a0 + row[1]),
                0.5 * (a1 + row[2]),
                0.5 * (aw + row[3]),
            )
        else:
            merged.append(row)

    if len(merged) < 2:
        y0, z0, z1, w = merged[0]
        height = max(float(min_height), z1 - z0)
        return [(y0, z0, z0 + height, max(float(min_width), w))]

    spans = max(1, int(samples_per_span))

    def lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * t

    dense: List[Tuple[float, float, float, float]] = []
    for idx in range(len(merged) - 1):
        a = merged[idx]
        b = merged[idx + 1]
        for j in range(spans):
            if idx > 0 and j == 0:
                continue
            t = float(j) / float(spans)
            dense.append(
                (
                    lerp(a[0], b[0], t),
                    lerp(a[1], b[1], t),
                    lerp(a[2], b[2], t),
                    lerp(a[3], b[3], t),
                )
            )
    dense.append(merged[-1])

    passes = max(0, int(smooth_passes))
    if passes > 0 and len(dense) >= 3:
        for _ in range(passes):
            smoothed: List[Tuple[float, float, float, float]] = [dense[0]]
            for idx in range(1, len(dense) - 1):
                left = dense[idx - 1]
                cur = dense[idx]
                right = dense[idx + 1]
                smoothed.append(
                    (
                        cur[0],
                        0.25 * left[1] + 0.50 * cur[1] + 0.25 * right[1],
                        0.25 * left[2] + 0.50 * cur[2] + 0.25 * right[2],
                        0.25 * left[3] + 0.50 * cur[3] + 0.25 * right[3],
                    )
                )
            smoothed.append(dense[-1])
            dense = smoothed

    min_h = max(1e-9, float(min_height))
    min_w = max(1e-9, float(min_width))
    out: List[Tuple[float, float, float, float]] = []
    for y, z0, z1, w in dense:
        low = float(min(z0, z1))
        high = float(max(z0, z1))
        if high - low < min_h:
            center = 0.5 * (low + high)
            low = center - 0.5 * min_h
            high = center + 0.5 * min_h
        out.append((float(y), low, high, max(min_w, float(w))))
    return out


def superellipse_profile(
    width: float,
    height: float,
    exponent: float = 2.6,
    *,
    segments: int = 48,
) -> List[Vec2]:
    """
    Return a CCW superellipse outline centered at the origin.

    Equation: |x/a|^n + |y/b|^n = 1
    - n=2.0 gives an ellipse
    - larger n approaches a rounded rectangle
    """
    width = float(width)
    height = float(height)
    exponent = float(exponent)
    segments = max(12, int(segments))

    if width <= 0 or height <= 0:
        raise ValueError("width/height must be positive")
    if exponent <= 0:
        raise ValueError("exponent must be positive")

    a = width * 0.5
    b = height * 0.5
    power = 2.0 / exponent

    pts: List[Vec2] = []
    for i in range(segments):
        t = 2.0 * pi * i / segments
        ct = cos(t)
        st = sin(t)
        x = a * (abs(ct) ** power)
        y = b * (abs(st) ** power)
        if ct < 0:
            x = -x
        if st < 0:
            y = -y
        pts.append((x, y))
    return pts


def rounded_rect_profile(
    width: float,
    height: float,
    radius: float,
    *,
    corner_segments: int = 6,
) -> List[Vec2]:
    """
    Return a CCW 2D rounded-rectangle outline centered at the origin.

    The profile is suitable for :class:`ExtrudeGeometry` or as the base shape
    for constructing :class:`LoftGeometry` profiles.
    """
    width = float(width)
    height = float(height)
    radius = float(radius)
    if width <= 0 or height <= 0:
        raise ValueError("width/height must be positive")
    corner_segments = max(1, int(corner_segments))
    max_r = 0.5 * min(width, height)
    if radius < 0 or radius > max_r:
        raise ValueError(f"radius must be within [0, {max_r}]")

    if radius <= 1e-12:
        hw = width * 0.5
        hh = height * 0.5
        return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]

    hw = width * 0.5
    hh = height * 0.5
    r = radius

    def arc(cx: float, cy: float, a0: float, a1: float) -> List[Vec2]:
        pts: List[Vec2] = []
        for i in range(corner_segments + 1):
            t = i / corner_segments
            a = a0 + (a1 - a0) * t
            pts.append((cx + r * cos(a), cy + r * sin(a)))
        return pts

    # Start on +X edge (bottom-right corner) and go CCW.
    pts: List[Vec2] = []
    pts.extend(arc(hw - r, -hh + r, -pi / 2.0, 0.0))  # bottom-right
    pts.extend(arc(hw - r, hh - r, 0.0, pi / 2.0)[1:])  # top-right
    pts.extend(arc(-hw + r, hh - r, pi / 2.0, pi)[1:])  # top-left
    pts.extend(arc(-hw + r, -hh + r, pi, 3.0 * pi / 2.0)[1:])  # bottom-left
    return pts


def _points_match_2d(a: Vec2, b: Vec2, *, tol: float = 1e-9) -> bool:
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def _points_match_3d(a: Vec3, b: Vec3, *, tol: float = 1e-9) -> bool:
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol and abs(a[2] - b[2]) <= tol


def _polygon_area(points: List[Vec2]) -> float:
    area = 0.0
    count = len(points)
    for i in range(count):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % count]
        area += x1 * y2 - x2 * y1
    return area * 0.5


def _ensure_ccw(points: List[Vec2]) -> List[Vec2]:
    area = _polygon_area(points)
    if abs(area) <= 1e-9:
        raise ValueError("Profile area must be non-zero")
    if area < 0:
        return list(reversed(points))
    return points


def _profile_points_2d(profile: Iterable[Tuple[float, float]]) -> List[Vec2]:
    points = [(float(x), float(y)) for (x, y) in profile]
    cleaned: List[Vec2] = []
    for pt in points:
        if not cleaned or not _points_match_2d(cleaned[-1], pt):
            cleaned.append(pt)
    if len(cleaned) >= 2 and _points_match_2d(cleaned[0], cleaned[-1]):
        cleaned = cleaned[:-1]
    if len(cleaned) < 3:
        raise ValueError("Profile must have at least 3 unique points")
    return cleaned


def _polyline_points_2d(profile: Iterable[Tuple[float, float]]) -> List[Vec2]:
    points = [(float(x), float(y)) for (x, y) in profile]
    cleaned: List[Vec2] = []
    for pt in points:
        if not cleaned or not _points_match_2d(cleaned[-1], pt):
            cleaned.append(pt)
    if len(cleaned) < 2:
        raise ValueError("Profile must have at least 2 unique points")
    return cleaned


def _lathe_cap_connector(
    start: Vec2,
    end: Vec2,
    *,
    start_tangent: Vec2,
    end_tangent: Vec2,
    mode: LatheCapMode,
    samples: int,
) -> List[Vec2]:
    if _points_match_2d(start, end):
        return [start]
    if mode == "flat":
        return [start, end]
    if mode != "round":
        raise ValueError("Lathe cap mode must be 'flat' or 'round'")

    chord_dx = end[0] - start[0]
    chord_dz = end[1] - start[1]
    chord = sqrt(chord_dx * chord_dx + chord_dz * chord_dz)
    if chord <= _EPS:
        return [start]

    start_len = sqrt(start_tangent[0] * start_tangent[0] + start_tangent[1] * start_tangent[1])
    end_len = sqrt(end_tangent[0] * end_tangent[0] + end_tangent[1] * end_tangent[1])
    if start_len <= _EPS or end_len <= _EPS:
        return [start, end]

    handle = min(chord * 0.75, start_len * 0.5, end_len * 0.5)
    if handle <= _EPS:
        return [start, end]

    p1 = (
        start[0] + start_tangent[0] * (handle / start_len),
        start[1] + start_tangent[1] * (handle / start_len),
    )
    p2 = (
        end[0] - end_tangent[0] * (handle / end_len),
        end[1] - end_tangent[1] * (handle / end_len),
    )
    return sample_cubic_bezier_spline_2d(
        [start, p1, p2, end],
        samples_per_segment=max(3, int(samples)),
    )


def _lathe_shell_profile(
    outer_profile: Iterable[Tuple[float, float]],
    inner_profile: Iterable[Tuple[float, float]],
    *,
    start_cap: LatheCapMode,
    end_cap: LatheCapMode,
    lip_samples: int,
) -> List[Vec2]:
    outer = _polyline_points_2d(outer_profile)
    inner = _polyline_points_2d(inner_profile)

    same_alignment = _tuple_distance(outer[0], inner[0]) + _tuple_distance(outer[-1], inner[-1])
    flipped_alignment = _tuple_distance(outer[0], inner[-1]) + _tuple_distance(outer[-1], inner[0])
    if flipped_alignment < same_alignment:
        inner = list(reversed(inner))

    end_connector = _lathe_cap_connector(
        outer[-1],
        inner[-1],
        start_tangent=(outer[-1][0] - outer[-2][0], outer[-1][1] - outer[-2][1]),
        end_tangent=(inner[-2][0] - inner[-1][0], inner[-2][1] - inner[-1][1]),
        mode=end_cap,
        samples=lip_samples,
    )
    start_connector = _lathe_cap_connector(
        inner[0],
        outer[0],
        start_tangent=(inner[0][0] - inner[1][0], inner[0][1] - inner[1][1]),
        end_tangent=(outer[1][0] - outer[0][0], outer[1][1] - outer[0][1]),
        mode=start_cap,
        samples=lip_samples,
    )

    profile = list(outer)
    profile.extend(end_connector[1:])
    profile.extend(reversed(inner[:-1]))
    profile.extend(start_connector[1:])
    return profile


def _profile_points_3d(profile: Iterable[Tuple[float, float, float]]) -> List[Vec3]:
    points = [(float(x), float(y), float(z)) for (x, y, z) in profile]
    cleaned: List[Vec3] = []
    for pt in points:
        if not cleaned or not _points_match_3d(cleaned[-1], pt):
            cleaned.append(pt)
    if len(cleaned) >= 2 and _points_match_3d(cleaned[0], cleaned[-1]):
        cleaned = cleaned[:-1]
    if len(cleaned) < 2:
        raise ValueError("Profile must have at least 2 unique points")
    return cleaned


def _cross_z(a: Vec2, b: Vec2, c: Vec2) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _point_in_triangle(p: Vec2, a: Vec2, b: Vec2, c: Vec2) -> bool:
    eps = 1e-9
    c1 = _cross_z(a, b, p)
    c2 = _cross_z(b, c, p)
    c3 = _cross_z(c, a, p)
    return (c1 > eps and c2 > eps and c3 > eps) or (c1 < -eps and c2 < -eps and c3 < -eps)


def _manifold_triangulate_polygons(polygons: Sequence[List[Vec2]]) -> Optional[List[Face]]:
    if not polygons:
        return []
    try:
        triangles = _m3d.triangulate(polygons, epsilon=float(_OBJ_QUANT_STEP))
    except Exception:
        return None
    return [
        (int(tri[0]), int(tri[1]), int(tri[2])) for tri in np.asarray(triangles, dtype=np.int64)
    ]


def _triangulate_polygon_earclip(points: List[Vec2]) -> List[Face]:
    if len(points) < 3:
        return []

    indices = list(range(len(points)))
    triangles: List[Face] = []
    guard = 0
    max_guard = len(points) * len(points)

    while len(indices) > 3:
        ear_found = False
        for i in range(len(indices)):
            i_prev = indices[i - 1]
            i_curr = indices[i]
            i_next = indices[(i + 1) % len(indices)]

            if _cross_z(points[i_prev], points[i_curr], points[i_next]) <= 0:
                continue

            ear = True
            for j in indices:
                if j in (i_prev, i_curr, i_next):
                    continue
                if _point_in_triangle(points[j], points[i_prev], points[i_curr], points[i_next]):
                    ear = False
                    break

            if ear:
                triangles.append((i_prev, i_curr, i_next))
                del indices[i]
                ear_found = True
                break

        guard += 1
        if not ear_found or guard > max_guard:
            raise ValueError("Failed to triangulate profile; ensure it is simple")

    triangles.append((indices[0], indices[1], indices[2]))
    return triangles


def _triangulate_polygon(points: List[Vec2]) -> List[Face]:
    if len(points) < 3:
        return []

    triangles = _manifold_triangulate_polygons([points])
    if triangles is not None:
        return triangles
    return _triangulate_polygon_earclip(points)


def _point_on_segment(p: Vec2, a: Vec2, b: Vec2, *, tol: float = 1e-9) -> bool:
    px, py = p
    ax, ay = a
    bx, by = b
    cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
    if abs(cross) > tol:
        return False
    dot = (px - ax) * (bx - ax) + (py - ay) * (by - ay)
    if dot < -tol:
        return False
    sq_len = (bx - ax) * (bx - ax) + (by - ay) * (by - ay)
    if dot - sq_len > tol:
        return False
    return True


def _segments_intersect(a: Vec2, b: Vec2, c: Vec2, d: Vec2, *, tol: float = 1e-9) -> bool:
    def orient(p: Vec2, q: Vec2, r: Vec2) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)

    if (o1 > tol and o2 < -tol or o1 < -tol and o2 > tol) and (
        o3 > tol and o4 < -tol or o3 < -tol and o4 > tol
    ):
        return True

    if abs(o1) <= tol and _point_on_segment(c, a, b, tol=tol):
        return True
    if abs(o2) <= tol and _point_on_segment(d, a, b, tol=tol):
        return True
    if abs(o3) <= tol and _point_on_segment(a, c, d, tol=tol):
        return True
    if abs(o4) <= tol and _point_on_segment(b, c, d, tol=tol):
        return True
    return False


def _point_in_polygon(point: Vec2, polygon: List[Vec2]) -> bool:
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        a = polygon[i]
        b = polygon[(i + 1) % n]
        if _point_on_segment(point, a, b):
            return True
        yi = a[1]
        yj = b[1]
        intersects = (yi > y) != (yj > y)
        if not intersects:
            continue
        x_on_edge = a[0] + (y - yi) * (b[0] - a[0]) / (yj - yi)
        if x_on_edge >= x:
            inside = not inside
    return inside


def _polygon_centroid(points: List[Vec2]) -> Vec2:
    sx = 0.0
    sy = 0.0
    n = max(1, len(points))
    for x, y in points:
        sx += x
        sy += y
    return (sx / float(n), sy / float(n))


def _reduce_redundant_vertices(points: List[Vec2], *, tol: float = 1e-9) -> List[Vec2]:
    out: List[Vec2] = []
    for pt in points:
        if not out or not _points_match_2d(out[-1], pt, tol=tol):
            out.append(pt)
    if len(out) >= 2 and _points_match_2d(out[0], out[-1], tol=tol):
        out.pop()
    if len(out) < 4:
        return out

    changed = True
    while changed and len(out) >= 4:
        changed = False
        keep: List[Vec2] = []
        n = len(out)
        for i in range(n):
            prev_pt = out[(i - 1) % n]
            cur_pt = out[i]
            next_pt = out[(i + 1) % n]
            if abs(_cross_z(prev_pt, cur_pt, next_pt)) <= tol and _point_on_segment(
                cur_pt, prev_pt, next_pt, tol=tol
            ):
                changed = True
                continue
            keep.append(cur_pt)
        out = keep
    return out


def _bridge_hole_into_outer(
    outer_ring: List[Vec2],
    hole_ring_cw: List[Vec2],
    *,
    all_holes_ccw: List[List[Vec2]],
) -> List[Vec2]:
    if len(outer_ring) < 3 or len(hole_ring_cw) < 3:
        raise ValueError("Cannot bridge degenerate polygon/hole")

    hi = max(range(len(hole_ring_cw)), key=lambda i: (hole_ring_cw[i][0], -hole_ring_cw[i][1]))
    hp = hole_ring_cw[hi]

    candidate_outer = sorted(
        range(len(outer_ring)),
        key=lambda i: (outer_ring[i][0] - hp[0]) ** 2 + (outer_ring[i][1] - hp[1]) ** 2,
    )

    def bridge_visible(oi: int) -> bool:
        op = outer_ring[oi]
        for i in range(len(outer_ring)):
            a = outer_ring[i]
            b = outer_ring[(i + 1) % len(outer_ring)]
            if _points_match_2d(a, op) or _points_match_2d(b, op):
                continue
            if _segments_intersect(op, hp, a, b):
                return False

        for i in range(len(hole_ring_cw)):
            a = hole_ring_cw[i]
            b = hole_ring_cw[(i + 1) % len(hole_ring_cw)]
            if _points_match_2d(a, hp) or _points_match_2d(b, hp):
                continue
            if _segments_intersect(op, hp, a, b):
                return False

        for h in all_holes_ccw:
            for i in range(len(h)):
                a = h[i]
                b = h[(i + 1) % len(h)]
                if _points_match_2d(a, hp) or _points_match_2d(b, hp):
                    continue
                if _segments_intersect(op, hp, a, b):
                    return False

        for t in (0.2, 0.5, 0.8):
            sample = (op[0] + (hp[0] - op[0]) * t, op[1] + (hp[1] - op[1]) * t)
            if not _point_in_polygon(sample, outer_ring):
                return False
            for h in all_holes_ccw:
                if _point_in_polygon(sample, h):
                    return False
        return True

    oi: Optional[int] = None
    for idx in candidate_outer:
        if bridge_visible(idx):
            oi = idx
            break
    if oi is None:
        raise ValueError("Failed to bridge polygon hole; try simpler/non-overlapping profiles")

    hole_path = hole_ring_cw[hi:] + hole_ring_cw[:hi]
    op = outer_ring[oi]
    on = outer_ring[(oi + 1) % len(outer_ring)]
    edge_dx = on[0] - op[0]
    edge_dy = on[1] - op[1]
    edge_len = (edge_dx * edge_dx + edge_dy * edge_dy) ** 0.5
    if edge_len <= 1e-12:
        eps_dir = (1.0, 0.0)
    else:
        eps_dir = (edge_dx / edge_len, edge_dy / edge_len)

    hprev = hole_ring_cw[(hi - 1) % len(hole_ring_cw)]
    hole_dx = hprev[0] - hp[0]
    hole_dy = hprev[1] - hp[1]
    hole_len = (hole_dx * hole_dx + hole_dy * hole_dy) ** 0.5
    if hole_len <= 1e-12:
        hole_eps_dir = (1.0, 0.0)
    else:
        hole_eps_dir = (hole_dx / hole_len, hole_dy / hole_len)

    # Duplicate both bridge endpoints with sub-quantization offsets. This keeps
    # the stitched polygon triangulable while collapsing to a watertight seam
    # after OBJ coordinate quantization.
    hp2 = (hp[0] + hole_eps_dir[0] * _BRIDGE_EPSILON, hp[1] + hole_eps_dir[1] * _BRIDGE_EPSILON)
    op2 = (op[0] + eps_dir[0] * _BRIDGE_EPSILON, op[1] + eps_dir[1] * _BRIDGE_EPSILON)

    merged = list(outer_ring[: oi + 1]) + list(hole_path) + [hp2, op2] + list(outer_ring[oi + 1 :])
    return _reduce_redundant_vertices(merged)


def _triangulate_polygon_with_holes(
    outer_profile: List[Vec2],
    hole_profiles: List[List[Vec2]],
) -> Tuple[List[Vec2], List[Face]]:
    outer = _ensure_ccw(_profile_points_2d(outer_profile))
    holes_ccw = [_ensure_ccw(_profile_points_2d(h)) for h in hole_profiles]

    if not holes_ccw:
        return outer, _triangulate_polygon(outer)

    for h in holes_ccw:
        c = _polygon_centroid(h)
        if not _point_in_polygon(c, outer):
            raise ValueError("Hole profile must lie inside outer profile")

    holes_cw = [list(reversed(h)) for h in holes_ccw]
    flat_ring = list(outer)
    for h in holes_cw:
        flat_ring.extend(h)

    triangles = _manifold_triangulate_polygons([outer] + holes_cw)
    if triangles is not None:
        return flat_ring, triangles

    holes_cw.sort(key=lambda h: max(pt[0] for pt in h), reverse=True)

    stitched = list(outer)
    for hole_cw in holes_cw:
        stitched = _bridge_hole_into_outer(stitched, hole_cw, all_holes_ccw=holes_ccw)

    stitched = _reduce_redundant_vertices(stitched)
    if _polygon_area(stitched) < 0:
        stitched = list(reversed(stitched))
    return stitched, _triangulate_polygon(stitched)


__all__ = [name for name in globals() if not name.startswith("__")]
