from __future__ import annotations

from contextlib import suppress
from math import cos, pi, sin
from typing import Sequence, Union

import numpy as np

from .common import (
    Vec2,
    _set_manifold_provenance,
    rounded_rect_profile,
    superellipse_profile,
)
from .primitives import MeshGeometry


def _cq_polyline_wire(cq_module, points: Sequence[Vec2], plane: str = "XY"):
    return cq_module.Workplane(plane).polyline(points).close()


def _sample_ellipse_profile(width: float, height: float, *, segments: int = 64) -> list[Vec2]:
    rx = float(width) * 0.5
    ry = float(height) * 0.5
    return [
        (rx * cos(2.0 * pi * index / float(segments)), ry * sin(2.0 * pi * index / float(segments)))
        for index in range(segments)
    ]


def _sample_rect_profile(width: float, height: float) -> list[Vec2]:
    hw = float(width) * 0.5
    hh = float(height) * 0.5
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def _shape_profile_points_2d(
    shape: str,
    size: Sequence[float],
    *,
    corner_radius: float = 0.0,
    segments: int = 64,
) -> list[Vec2]:
    width = float(size[0])
    height = float(size[1])
    if width <= 0.0 or height <= 0.0:
        raise ValueError("shape size values must be positive")
    if shape == "rect":
        return _sample_rect_profile(width, height)
    if shape == "rounded_rect":
        return rounded_rect_profile(
            width, height, max(0.0, corner_radius), corner_segments=max(4, segments // 16)
        )
    if shape == "circle":
        diameter = min(width, height)
        return _sample_ellipse_profile(diameter, diameter, segments=segments)
    if shape == "ellipse":
        return _sample_ellipse_profile(width, height, segments=segments)
    if shape == "superellipse":
        return superellipse_profile(width, height, exponent=2.8, segments=segments)
    raise ValueError(f"Unsupported shape {shape!r}")


def _shape_size_from_wall(
    inner_size: Sequence[float],
    wall: Union[float, Sequence[float]],
) -> tuple[float, float]:
    inner_w = float(inner_size[0])
    inner_h = float(inner_size[1])
    if isinstance(wall, (int, float)):
        wall_left = wall_right = wall_bottom = wall_top = float(wall)
    else:
        if len(wall) != 4:
            raise ValueError("wall must be a float or a 4-sequence")
        wall_left, wall_right, wall_bottom, wall_top = (float(value) for value in wall)
    if min(wall_left, wall_right, wall_bottom, wall_top) < 0.0:
        raise ValueError("wall values must be non-negative")
    return (
        inner_w + wall_left + wall_right,
        inner_h + wall_bottom + wall_top,
    )


def _cq_ring_solid(
    cq_module,
    outer_points: Sequence[Vec2],
    inner_points: Sequence[Vec2],
    depth: float,
    *,
    center: bool = True,
):
    shape = _cq_polyline_wire(cq_module, outer_points).extrude(float(depth) * 0.5, both=center)
    inner_cut = _cq_polyline_wire(cq_module, inner_points).extrude(
        float(depth) + max(0.002, float(depth) * 0.5),
        both=center,
    )
    return shape.cut(inner_cut)


def _cq_annulus_x(cq_module, outer_radius: float, inner_radius: float, width: float):
    return (
        cq_module.Workplane("YZ")
        .circle(float(outer_radius))
        .circle(max(float(inner_radius), 1.0e-4))
        .extrude(float(width) * 0.5, both=True)
    )


def _centered_pattern_positions(count: int, spacing: float) -> list[float]:
    if count <= 0:
        return []
    if count == 1:
        return [0.0]
    origin = -0.5 * float(spacing) * float(count - 1)
    return [origin + float(index) * float(spacing) for index in range(count)]


def _rounded_slot_profile(length: float, width: float, *, segments: int = 10) -> list[Vec2]:
    radius = width * 0.5
    straight = max(length - width, 0.0)
    half = straight * 0.5
    points: list[Vec2] = []
    for index in range(segments + 1):
        theta = -pi * 0.5 + pi * (index / float(segments))
        points.append((half + radius * cos(theta), radius * sin(theta)))
    for index in range(segments + 1):
        theta = pi * 0.5 + pi * (index / float(segments))
        points.append((-half + radius * cos(theta), radius * sin(theta)))
    return points


def _loft_between_radii_z(
    cq_module,
    radii_and_offsets: Sequence[tuple[float, float]],
):
    wp = None
    previous_offset = 0.0
    for radius, offset in radii_and_offsets:
        if wp is None:
            wp = cq_module.Workplane("XY").workplane(offset=offset).circle(radius)
        else:
            wp = wp.workplane(offset=offset - previous_offset).circle(radius)
        previous_offset = offset
    if wp is None:
        raise ValueError("No loft sections were provided")
    return wp.loft(combine=True, ruled=False)


def _cut_with_pattern(
    shape,
    cutters: Sequence[object],
):
    for cutter in cutters:
        shape = shape.cut(cutter)
    return shape


def _mesh_geometry_from_cadquery_model(
    model: object,
    *,
    tolerance: float = 0.001,
    angular_tolerance: float = 0.1,
) -> "MeshGeometry":
    import trimesh

    shape = model.val() if hasattr(model, "val") else model
    if hasattr(shape, "fix"):
        with suppress(Exception):
            shape = shape.fix()

    try:
        vertices_raw, triangles_raw = shape.tessellate(
            float(tolerance),
            float(angular_tolerance),
        )
    except TypeError:
        vertices_raw, triangles_raw = shape.tessellate(float(tolerance))

    vertices = [
        (float(vertex.x), float(vertex.y), float(vertex.z))
        if hasattr(vertex, "x")
        else (float(vertex[0]), float(vertex[1]), float(vertex[2]))
        for vertex in vertices_raw
    ]
    faces = [(int(face[0]), int(face[1]), int(face[2])) for face in triangles_raw]
    if not vertices or not faces:
        raise ValueError("CadQuery tessellation produced an empty mesh")

    # CadQuery tessellation can duplicate shared vertices per face. Normalize the
    # triangle soup so downstream manifold checks and OBJ export see one closed mesh.
    mesh = trimesh.Trimesh(vertices=np.asarray(vertices), faces=np.asarray(faces), process=True)
    if len(mesh.vertices) == 0 or len(mesh.faces) == 0:
        raise ValueError("CadQuery tessellation normalization produced an empty mesh")

    geometry = MeshGeometry(
        vertices=[tuple(float(coord) for coord in vertex) for vertex in mesh.vertices],
        faces=[tuple(int(index) for index in face) for face in mesh.faces],
    )
    try:
        from .booleans import _manifold_from_geometry

        _set_manifold_provenance(
            geometry,
            _manifold_from_geometry(geometry, name="cadquery_tessellation"),
        )
    except ValueError:
        pass
    return geometry


__all__ = [name for name in globals() if not name.startswith("__")]
