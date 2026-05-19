from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, pi, sin
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple, Union

from ..types import Box, Cylinder, Sphere
from .common import (
    _EPS,
    _OBJ_COORD_DECIMALS,
    Face,
    LatheCapMode,
    Mat4,
    Vec2,
    Vec3,
    _clear_manifold_provenance,
    _clear_primitive_provenance,
    _copy_manifold_provenance,
    _copy_primitive_provenance,
    _ensure_ccw,
    _lathe_shell_profile,
    _point_in_polygon,
    _points_match_2d,
    _polygon_area,
    _polygon_centroid,
    _polyline_points_2d,
    _prepend_primitive_transform,
    _profile_points_2d,
    _profile_points_3d,
    _set_primitive_provenance,
    _triangulate_polygon,
    _triangulate_polygon_with_holes,
    _v_normalize,
)


@dataclass
class MeshGeometry:
    vertices: List[Vec3] = field(default_factory=list)
    faces: List[Face] = field(default_factory=list)

    def copy(self) -> "MeshGeometry":
        copied = MeshGeometry(
            vertices=[tuple(vertex) for vertex in self.vertices],
            faces=[tuple(face) for face in self.faces],
        )
        _copy_primitive_provenance(self, copied)
        _copy_manifold_provenance(self, copied)
        return copied

    def clone(self) -> "MeshGeometry":
        return self.copy()

    def add_vertex(self, x: float, y: float, z: float) -> int:
        _clear_manifold_provenance(self)
        self.vertices.append((float(x), float(y), float(z)))
        return len(self.vertices) - 1

    def add_face(self, a: int, b: int, c: int) -> None:
        _clear_manifold_provenance(self)
        self.faces.append((int(a), int(b), int(c)))

    def merge(self, other: "MeshGeometry") -> "MeshGeometry":
        offset = len(self.vertices)
        self.vertices.extend(other.vertices)
        self.faces.extend((a + offset, b + offset, c + offset) for (a, b, c) in other.faces)
        _clear_primitive_provenance(self)
        _clear_manifold_provenance(self)
        return self

    def translate(self, dx: float, dy: float, dz: float) -> "MeshGeometry":
        self.vertices = [(x + dx, y + dy, z + dz) for (x, y, z) in self.vertices]
        transform: Mat4 = (
            (1.0, 0.0, 0.0, float(dx)),
            (0.0, 1.0, 0.0, float(dy)),
            (0.0, 0.0, 1.0, float(dz)),
            (0.0, 0.0, 0.0, 1.0),
        )
        _prepend_primitive_transform(self, transform)
        _clear_manifold_provenance(self)
        return self

    def scale(
        self, sx: float, sy: Optional[float] = None, sz: Optional[float] = None
    ) -> "MeshGeometry":
        if sy is None:
            sy = sx
        if sz is None:
            sz = sx
        self.vertices = [(x * sx, y * sy, z * sz) for (x, y, z) in self.vertices]
        _prepend_primitive_transform(
            self,
            (
                (float(sx), 0.0, 0.0, 0.0),
                (0.0, float(sy), 0.0, 0.0),
                (0.0, 0.0, float(sz), 0.0),
                (0.0, 0.0, 0.0, 1.0),
            ),
        )
        _clear_manifold_provenance(self)
        return self

    def rotate(
        self,
        axis: Sequence[float],
        angle: float,
        *,
        origin: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> "MeshGeometry":
        kx, ky, kz = _v_normalize((float(axis[0]), float(axis[1]), float(axis[2])))
        ox, oy, oz = float(origin[0]), float(origin[1]), float(origin[2])
        c = cos(angle)
        s = sin(angle)
        t = 1.0 - c

        r00 = t * kx * kx + c
        r01 = t * kx * ky - s * kz
        r02 = t * kx * kz + s * ky
        r10 = t * ky * kx + s * kz
        r11 = t * ky * ky + c
        r12 = t * ky * kz - s * kx
        r20 = t * kz * kx - s * ky
        r21 = t * kz * ky + s * kx
        r22 = t * kz * kz + c

        tx = ox - (r00 * ox + r01 * oy + r02 * oz)
        ty = oy - (r10 * ox + r11 * oy + r12 * oz)
        tz = oz - (r20 * ox + r21 * oy + r22 * oz)
        transform: Mat4 = (
            (float(r00), float(r01), float(r02), float(tx)),
            (float(r10), float(r11), float(r12), float(ty)),
            (float(r20), float(r21), float(r22), float(tz)),
            (0.0, 0.0, 0.0, 1.0),
        )

        self.vertices = [
            (
                r00 * x + r01 * y + r02 * z + tx,
                r10 * x + r11 * y + r12 * z + ty,
                r20 * x + r21 * y + r22 * z + tz,
            )
            for (x, y, z) in self.vertices
        ]
        _prepend_primitive_transform(self, transform)
        _clear_manifold_provenance(self)
        return self

    def rotate_x(self, angle: float) -> "MeshGeometry":
        return self.rotate((1.0, 0.0, 0.0), angle)

    def rotate_y(self, angle: float) -> "MeshGeometry":
        return self.rotate((0.0, 1.0, 0.0), angle)

    def rotate_z(self, angle: float) -> "MeshGeometry":
        return self.rotate((0.0, 0.0, 1.0), angle)

    def to_obj(self) -> str:
        lines = ["o mesh"]
        for x, y, z in self.vertices:
            lines.append(
                f"v {x:.{_OBJ_COORD_DECIMALS}f} {y:.{_OBJ_COORD_DECIMALS}f} {z:.{_OBJ_COORD_DECIMALS}f}"
            )
        for a, b, c in self.faces:
            lines.append(f"f {a + 1} {b + 1} {c + 1}")
        lines.append("")
        return "\n".join(lines)

    def save_obj(self, path: Union[str, Path]) -> None:
        out_path = Path(path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(self.to_obj())


def _coerce_positive_radii3(
    value: Union[float, Sequence[float]],
    *,
    name: str,
) -> Vec3:
    if isinstance(value, (int, float)):
        radius = float(value)
        if radius <= 0.0:
            raise ValueError(f"{name} must be positive")
        return (radius, radius, radius)
    if len(value) != 3:
        raise ValueError(f"{name} must be a positive float or a 3-sequence")
    rx = float(value[0])
    ry = float(value[1])
    rz = float(value[2])
    if rx <= 0.0 or ry <= 0.0 or rz <= 0.0:
        raise ValueError(f"{name} values must be positive")
    return (rx, ry, rz)


class BoxGeometry(MeshGeometry):
    def __init__(self, size: Sequence[float]):
        super().__init__()
        x, y, z = (float(size[0]), float(size[1]), float(size[2]))
        primitive = Box((x, y, z))
        _set_primitive_provenance(self, primitive)
        hx, hy, hz = x / 2.0, y / 2.0, z / 2.0
        verts = [
            (-hx, -hy, -hz),
            (hx, -hy, -hz),
            (hx, hy, -hz),
            (-hx, hy, -hz),
            (-hx, -hy, hz),
            (hx, -hy, hz),
            (hx, hy, hz),
            (-hx, hy, hz),
        ]
        self.vertices = verts
        faces = [
            (0, 1, 2),
            (0, 2, 3),
            (4, 6, 5),
            (4, 7, 6),
            (0, 4, 5),
            (0, 5, 1),
            (1, 5, 6),
            (1, 6, 2),
            (2, 6, 7),
            (2, 7, 3),
            (3, 7, 4),
            (3, 4, 0),
        ]
        self.faces = faces


class CylinderGeometry(MeshGeometry):
    def __init__(
        self,
        radius: float,
        height: float,
        *,
        radial_segments: int = 24,
        closed: bool = True,
    ):
        super().__init__()
        radius = float(radius)
        height = float(height)
        radial_segments = max(3, int(radial_segments))
        primitive = Cylinder(radius=radius, length=height)
        _set_primitive_provenance(self, primitive)
        half = height / 2.0

        top_indices = []
        bottom_indices = []
        for i in range(radial_segments):
            theta = 2 * pi * i / radial_segments
            x = radius * cos(theta)
            y = radius * sin(theta)
            top_indices.append(self.add_vertex(x, y, half))
            bottom_indices.append(self.add_vertex(x, y, -half))

        for i in range(radial_segments):
            i2 = (i + 1) % radial_segments
            a = top_indices[i]
            b = bottom_indices[i]
            c = bottom_indices[i2]
            d = top_indices[i2]
            self.add_face(a, b, c)
            self.add_face(a, c, d)

        if closed:
            top_center = self.add_vertex(0.0, 0.0, half)
            bottom_center = self.add_vertex(0.0, 0.0, -half)
            for i in range(radial_segments):
                i2 = (i + 1) % radial_segments
                self.add_face(top_center, top_indices[i], top_indices[i2])
                self.add_face(bottom_center, bottom_indices[i2], bottom_indices[i])


class ConeGeometry(MeshGeometry):
    def __init__(
        self,
        radius: float,
        height: float,
        *,
        radial_segments: int = 24,
        closed: bool = True,
    ):
        super().__init__()
        radius = float(radius)
        height = float(height)
        radial_segments = max(3, int(radial_segments))
        half = height / 2.0
        apex = self.add_vertex(0.0, 0.0, half)
        ring_indices = []
        for i in range(radial_segments):
            theta = 2 * pi * i / radial_segments
            x = radius * cos(theta)
            y = radius * sin(theta)
            ring_indices.append(self.add_vertex(x, y, -half))

        for i in range(radial_segments):
            i2 = (i + 1) % radial_segments
            self.add_face(apex, ring_indices[i], ring_indices[i2])

        if closed:
            base_center = self.add_vertex(0.0, 0.0, -half)
            for i in range(radial_segments):
                i2 = (i + 1) % radial_segments
                self.add_face(base_center, ring_indices[i2], ring_indices[i])


class SphereGeometry(MeshGeometry):
    def __init__(
        self,
        radius: float,
        *,
        width_segments: int = 24,
        height_segments: int = 16,
    ):
        super().__init__()
        radius = float(radius)
        width_segments = max(3, int(width_segments))
        height_segments = max(2, int(height_segments))
        primitive = Sphere(radius=radius)
        _set_primitive_provenance(self, primitive)

        for iy in range(height_segments + 1):
            v = iy / height_segments
            phi = pi * v
            for ix in range(width_segments + 1):
                u = ix / width_segments
                theta = 2 * pi * u
                x = radius * sin(phi) * cos(theta)
                y = radius * sin(phi) * sin(theta)
                z = radius * cos(phi)
                self.add_vertex(x, y, z)

        verts_per_row = width_segments + 1
        for iy in range(height_segments):
            for ix in range(width_segments):
                a = iy * verts_per_row + ix
                b = a + verts_per_row
                c = b + 1
                d = a + 1
                if iy != 0:
                    self.add_face(a, b, d)
                if iy != height_segments - 1:
                    self.add_face(b, c, d)


class DomeGeometry(MeshGeometry):
    def __init__(
        self,
        radius: Union[float, Sequence[float]],
        *,
        radial_segments: int = 24,
        height_segments: int = 12,
        closed: bool = True,
    ):
        super().__init__()
        rx, ry, rz = _coerce_positive_radii3(radius, name="radius")
        radial_segments = max(3, int(radial_segments))
        height_segments = max(1, int(height_segments))

        for iy in range(height_segments + 1):
            v = iy / height_segments
            phi = 0.5 * pi * v
            sin_phi = sin(phi)
            cos_phi = cos(phi)
            z = rz * cos_phi
            for ix in range(radial_segments + 1):
                u = ix / radial_segments
                theta = 2.0 * pi * u
                x = rx * sin_phi * cos(theta)
                y = ry * sin_phi * sin(theta)
                self.add_vertex(x, y, z)

        verts_per_row = radial_segments + 1
        for iy in range(height_segments):
            for ix in range(radial_segments):
                a = iy * verts_per_row + ix
                b = a + verts_per_row
                c = b + 1
                d = a + 1
                if iy != 0:
                    self.add_face(a, b, d)
                self.add_face(b, c, d)

        if closed:
            base_center = self.add_vertex(0.0, 0.0, 0.0)
            base_offset = height_segments * verts_per_row
            for ix in range(radial_segments):
                self.add_face(base_center, base_offset + ix, base_offset + ix + 1)


class CapsuleGeometry(MeshGeometry):
    def __init__(
        self,
        radius: float,
        length: float,
        *,
        radial_segments: int = 24,
        height_segments: int = 8,
    ):
        super().__init__()
        radius = float(radius)
        length = float(length)
        radial_segments = max(3, int(radial_segments))
        height_segments = max(2, int(height_segments))

        if radius <= 0.0:
            raise ValueError("radius must be positive")
        if length < 0.0:
            raise ValueError("length must be non-negative")

        if length <= _EPS:
            self.merge(
                SphereGeometry(
                    radius,
                    width_segments=radial_segments,
                    height_segments=max(4, 2 * height_segments),
                )
            )
            return

        half = length * 0.5
        rows: List[Tuple[float, float]] = []

        for iy in range(height_segments + 1):
            v = iy / height_segments
            phi = 0.5 * pi * v
            rows.append((radius * sin(phi), half + radius * cos(phi)))

        for iy in range(height_segments + 1):
            v = iy / height_segments
            phi = 0.5 * pi + 0.5 * pi * v
            rows.append((radius * sin(phi), -half + radius * cos(phi)))

        for ring_radius, z in rows:
            for ix in range(radial_segments + 1):
                u = ix / radial_segments
                theta = 2.0 * pi * u
                x = ring_radius * cos(theta)
                y = ring_radius * sin(theta)
                self.add_vertex(x, y, z)

        verts_per_row = radial_segments + 1
        segment_count = len(rows) - 1
        for iy in range(segment_count):
            for ix in range(radial_segments):
                a = iy * verts_per_row + ix
                b = a + verts_per_row
                c = b + 1
                d = a + 1
                if iy != 0:
                    self.add_face(a, b, d)
                if iy != segment_count - 1:
                    self.add_face(b, c, d)


class TorusGeometry(MeshGeometry):
    def __init__(
        self,
        radius: float,
        tube: float,
        *,
        radial_segments: int = 16,
        tubular_segments: int = 32,
    ):
        super().__init__()
        radius = float(radius)
        tube = float(tube)
        radial_segments = max(3, int(radial_segments))
        tubular_segments = max(3, int(tubular_segments))

        for j in range(radial_segments + 1):
            v = 2 * pi * j / radial_segments
            cv = cos(v)
            sv = sin(v)
            for i in range(tubular_segments + 1):
                u = 2 * pi * i / tubular_segments
                cu = cos(u)
                su = sin(u)
                x = (radius + tube * cv) * cu
                y = (radius + tube * cv) * su
                z = tube * sv
                self.add_vertex(x, y, z)

        verts_per_row = tubular_segments + 1
        for j in range(radial_segments):
            for i in range(tubular_segments):
                a = j * verts_per_row + i
                b = a + verts_per_row
                c = b + 1
                d = a + 1
                self.add_face(a, b, d)
                self.add_face(b, c, d)


class LatheGeometry(MeshGeometry):
    @classmethod
    def from_shell_profiles(
        cls,
        outer_profile: Iterable[Tuple[float, float]],
        inner_profile: Iterable[Tuple[float, float]],
        *,
        segments: int = 32,
        start_cap: LatheCapMode = "flat",
        end_cap: LatheCapMode = "flat",
        lip_samples: int = 6,
    ) -> "LatheGeometry":
        profile = _lathe_shell_profile(
            outer_profile,
            inner_profile,
            start_cap=start_cap,
            end_cap=end_cap,
            lip_samples=lip_samples,
        )
        return cls(profile, segments=segments, closed=True)

    def __init__(
        self,
        profile: Iterable[Tuple[float, float]],
        *,
        segments: int = 32,
        closed: bool = True,
    ):
        super().__init__()
        segments = max(3, int(segments))
        points = _profile_points_2d(profile) if closed else _polyline_points_2d(profile)
        if closed:
            points = _ensure_ccw(points)

        normalized_points: List[Vec2] = []
        for r, z in points:
            if r < -_EPS:
                raise ValueError("Lathe profile radii must be non-negative")
            normalized_points.append((0.0 if abs(r) <= _EPS else float(r), float(z)))
        points = normalized_points

        ring_indices: List[List[int]] = []
        for r, z in points:
            if r <= _EPS:
                axis_vertex = self.add_vertex(0.0, 0.0, z)
                ring_indices.append([axis_vertex] * segments)
                continue

            ring: List[int] = []
            for i in range(segments):
                theta = 2 * pi * i / segments
                ring.append(self.add_vertex(r * cos(theta), r * sin(theta), z))
            ring_indices.append(ring)

        def add_face_if_non_degenerate(a: int, b: int, c: int) -> None:
            if a == b or b == c or c == a:
                return
            self.add_face(a, b, c)

        edge_count = len(points) if closed else len(points) - 1
        for j in range(edge_count):
            j2 = (j + 1) % len(points)
            r0 = points[j][0]
            r1 = points[j2][0]
            if r0 <= _EPS and r1 <= _EPS:
                continue

            ring0 = ring_indices[j]
            ring1 = ring_indices[j2]
            for i in range(segments):
                i2 = (i + 1) % segments
                a = ring0[i]
                b = ring0[i2]
                c = ring1[i2]
                d = ring1[i]
                add_face_if_non_degenerate(a, b, d)
                add_face_if_non_degenerate(b, c, d)


class LoftGeometry(MeshGeometry):
    def __init__(
        self,
        profiles: Iterable[Iterable[Tuple[float, float, float]]],
        *,
        cap: bool = True,
        closed: bool = True,
    ):
        super().__init__()
        profile_list = [_profile_points_3d(profile) for profile in profiles]
        if len(profile_list) < 2:
            raise ValueError("Loft requires at least two profiles")

        ring_count = len(profile_list[0])
        if closed and ring_count < 3:
            raise ValueError("Closed loft profiles must have at least 3 points")

        for profile in profile_list:
            if len(profile) != ring_count:
                raise ValueError("Loft profiles must have the same point count")

        profile_xy = [(x, y) for (x, y, _z) in profile_list[0]]
        base_area = _polygon_area(profile_xy)
        if abs(base_area) <= 1e-9:
            raise ValueError(
                "Loft profile area must be non-zero in XY projection; "
                "profiles should usually be closed XY loops at constant z"
            )

        if base_area < 0:
            profile_list = [list(reversed(profile)) for profile in profile_list]
            base_area = -base_area

        for idx, profile in enumerate(profile_list):
            area = _polygon_area([(x, y) for (x, y, _z) in profile])
            if abs(area) <= 1e-9:
                raise ValueError(
                    "Loft profile area must be non-zero in XY projection; "
                    "profiles should usually be closed XY loops at constant z"
                )
            if area * base_area < 0:
                profile_list[idx] = list(reversed(profile))

        for profile in profile_list:
            for x, y, z in profile:
                self.add_vertex(x, y, z)

        segment_count = ring_count if closed else ring_count - 1
        for i in range(len(profile_list) - 1):
            offset0 = i * ring_count
            offset1 = (i + 1) * ring_count
            for j in range(segment_count):
                j2 = (j + 1) % ring_count
                a = offset0 + j
                b = offset0 + j2
                c = offset1 + j2
                d = offset1 + j
                self.add_face(a, b, c)
                self.add_face(a, c, d)

        if cap and closed:
            first_profile = profile_list[0]
            last_profile = profile_list[-1]
            first_xy = [(x, y) for (x, y, _z) in first_profile]
            last_xy = [(x, y) for (x, y, _z) in last_profile]

            z_spread = max(z for (_x, _y, z) in first_profile) - min(
                z for (_x, _y, z) in first_profile
            )
            if z_spread > 1e-6:
                raise ValueError("Loft caps require planar profiles")

            z_spread = max(z for (_x, _y, z) in last_profile) - min(
                z for (_x, _y, z) in last_profile
            )
            if z_spread > 1e-6:
                raise ValueError("Loft caps require planar profiles")

            triangles = _triangulate_polygon(first_xy)
            for a, b, c in triangles:
                self.add_face(c, b, a)

            offset_last = (len(profile_list) - 1) * ring_count
            triangles = _triangulate_polygon(last_xy)
            for a, b, c in triangles:
                self.add_face(offset_last + a, offset_last + b, offset_last + c)


class ExtrudeGeometry(MeshGeometry):
    @classmethod
    def centered(
        cls,
        profile: Iterable[Tuple[float, float]],
        height: float,
        *,
        cap: bool = True,
        closed: bool = True,
    ) -> "ExtrudeGeometry":
        return cls(profile, height, cap=cap, center=True, closed=closed)

    @classmethod
    def from_z0(
        cls,
        profile: Iterable[Tuple[float, float]],
        height: float,
        *,
        cap: bool = True,
        closed: bool = True,
    ) -> "ExtrudeGeometry":
        return cls(profile, height, cap=cap, center=False, closed=closed)

    def __init__(
        self,
        profile: Iterable[Tuple[float, float]],
        height: float,
        *,
        cap: bool = True,
        center: bool = True,
        closed: bool = True,
    ):
        super().__init__()
        height = float(height)
        if height <= 0:
            raise ValueError("Extrude height must be positive")

        points = _ensure_ccw(_profile_points_2d(profile))
        if not closed:
            cap = False

        z0 = -height / 2.0 if center else 0.0
        z1 = z0 + height

        profiles = [
            [(x, y, z0) for (x, y) in points],
            [(x, y, z1) for (x, y) in points],
        ]
        self.merge(LoftGeometry(profiles, cap=cap, closed=closed))


class ExtrudeWithHolesGeometry(MeshGeometry):
    """
    Extrude a 2D outer profile with one or more through-holes.

    Notes
    -----
    - This is a lightweight CAD-like helper for panel/filter parts.
    - For capped, closed solids, cap triangulation uses the shared manifold-backed
      polygon triangulation path when available.
    """

    def __init__(
        self,
        outer_profile: Iterable[Tuple[float, float]],
        hole_profiles: Iterable[Iterable[Tuple[float, float]]],
        height: float,
        *,
        cap: bool = True,
        center: bool = True,
        closed: bool = True,
    ):
        super().__init__()
        height = float(height)
        if height <= 0:
            raise ValueError("Extrude height must be positive")

        outer = _ensure_ccw(_profile_points_2d(outer_profile))
        holes = [_ensure_ccw(_profile_points_2d(h)) for h in hole_profiles]
        if not closed:
            cap = False

        for hole in holes:
            c = _polygon_centroid(hole)
            if not _point_in_polygon(c, outer):
                raise ValueError("Hole profile must lie inside outer profile")

        if not holes:
            self.merge(
                ExtrudeGeometry(
                    outer,
                    height,
                    cap=cap,
                    center=center,
                    closed=closed,
                )
            )
            return

        z0 = -height / 2.0 if center else 0.0
        z1 = z0 + height

        # Exterior side walls.
        outer_loft = LoftGeometry(
            [[(x, y, z0) for (x, y) in outer], [(x, y, z1) for (x, y) in outer]],
            cap=False,
            closed=closed,
        )
        outer_offset = len(self.vertices)
        self.merge(outer_loft)
        bottom_cap_indices = [outer_offset + i for i in range(len(outer))]
        top_cap_indices = [outer_offset + len(outer) + i for i in range(len(outer))]

        # Interior side walls for each hole.
        holes_cw: List[List[Vec2]] = []
        for hole in holes:
            hole_cw = list(reversed(hole))
            hole_loft = LoftGeometry(
                [[(x, y, z0) for (x, y) in hole_cw], [(x, y, z1) for (x, y) in hole_cw]],
                cap=False,
                closed=closed,
            )
            hole_offset = len(self.vertices)
            self.merge(hole_loft)
            bottom_cap_indices.extend(hole_offset + i for i in range(len(hole_cw)))
            top_cap_indices.extend(hole_offset + len(hole_cw) + i for i in range(len(hole_cw)))
            holes_cw.append(hole_cw)

        if not (cap and closed):
            return

        cap_ring, cap_triangles = _triangulate_polygon_with_holes(outer, holes)
        expected_cap_ring = list(outer)
        for hole_cw in holes_cw:
            expected_cap_ring.extend(hole_cw)

        reuse_sidewall_rims = len(cap_ring) == len(expected_cap_ring) and all(
            _points_match_2d(actual, expected)
            for actual, expected in zip(cap_ring, expected_cap_ring)
        )
        if reuse_sidewall_rims:
            bottom_idx = bottom_cap_indices
            top_idx = top_cap_indices
        else:
            bottom_idx = [self.add_vertex(x, y, z0) for (x, y) in cap_ring]
            top_idx = [self.add_vertex(x, y, z1) for (x, y) in cap_ring]

        for a, b, c in cap_triangles:
            # Bottom points downward, top points upward.
            self.add_face(bottom_idx[c], bottom_idx[b], bottom_idx[a])
            self.add_face(top_idx[a], top_idx[b], top_idx[c])


def _translated_profile(profile: List[Vec2], dx: float, dy: float) -> List[Vec2]:
    return [(x + dx, y + dy) for (x, y) in profile]


def _mesh_bounds(geometry: "MeshGeometry") -> tuple[Vec3, Vec3]:
    if not geometry.vertices:
        raise ValueError("MeshGeometry has no vertices")
    xs = [vertex[0] for vertex in geometry.vertices]
    ys = [vertex[1] for vertex in geometry.vertices]
    zs = [vertex[2] for vertex in geometry.vertices]
    return (
        (min(xs), min(ys), min(zs)),
        (max(xs), max(ys), max(zs)),
    )


def _normalize_pitch_2d(
    pitch: Union[float, Sequence[float]],
    *,
    name: str = "pitch",
) -> tuple[float, float]:
    if isinstance(pitch, Sequence) and not isinstance(pitch, (str, bytes)):
        values = list(pitch)
        if len(values) != 2:
            raise ValueError(f"{name} must be a positive scalar or a 2-tuple")
        pitch_x = float(values[0])
        pitch_y = float(values[1])
    else:
        pitch_x = float(pitch)
        pitch_y = float(pitch)
    if pitch_x <= 0.0 or pitch_y <= 0.0:
        raise ValueError(f"{name} values must be positive")
    return (pitch_x, pitch_y)


def _centered_axis_positions(limit: float, pitch: float) -> list[float]:
    if limit < -1e-9:
        return []
    if limit <= 1e-9:
        return [0.0]
    count = int((2.0 * limit) / pitch) + 1
    if count <= 0:
        return []
    return [((index - (count - 1) * 0.5) * pitch) for index in range(count)]


def _mesh_geometry_shifted_to_z0(geometry: "MeshGeometry") -> "MeshGeometry":
    shifted = geometry.copy()
    mins, _maxs = _mesh_bounds(shifted)
    shifted.translate(0.0, 0.0, -mins[2])
    return shifted


def _mesh_geometry_shifted_to_axis0(geometry: "MeshGeometry", axis: int) -> "MeshGeometry":
    shifted = geometry.copy()
    mins, _maxs = _mesh_bounds(shifted)
    delta = [0.0, 0.0, 0.0]
    delta[int(axis)] = -mins[int(axis)]
    shifted.translate(delta[0], delta[1], delta[2])
    return shifted


def _adopt_mesh_geometry(target: "MeshGeometry", geometry: "MeshGeometry") -> None:
    target.vertices = [tuple(vertex) for vertex in geometry.vertices]
    target.faces = [tuple(face) for face in geometry.faces]
    _copy_manifold_provenance(geometry, target)
