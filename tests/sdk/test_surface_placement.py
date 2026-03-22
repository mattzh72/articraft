from __future__ import annotations

import logging
import math
from pathlib import Path

import pytest

from sdk import (
    Box,
    BoxGeometry,
    Cylinder,
    Mesh,
    MeshGeometry,
    Origin,
    Part,
    Sphere,
    SurfaceFrame,
    ValidationError,
    Visual,
    mesh_from_geometry,
    part_local_aabb,
    place_on_surface,
    sample_catmull_rom_spline_2d,
    surface_frame,
    wrap_mesh_onto_surface,
    wrap_profile_onto_surface,
)
from sdk._core.v0 import placement as placement_module


def _rpy_matrix(origin: Origin) -> tuple[tuple[float, float, float], ...]:
    roll, pitch, yaw = origin.rpy
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


def _rotate(
    mat: tuple[tuple[float, float, float], ...], vec: tuple[float, float, float]
) -> tuple[float, float, float]:
    x, y, z = vec
    return (
        mat[0][0] * x + mat[0][1] * y + mat[0][2] * z,
        mat[1][0] * x + mat[1][1] * y + mat[1][2] * z,
        mat[2][0] * x + mat[2][1] * y + mat[2][2] * z,
    )


def _assert_vec_close(
    actual: tuple[float, float, float], expected: tuple[float, float, float], *, tol: float = 1e-6
) -> None:
    for a, b in zip(actual, expected):
        assert abs(a - b) <= tol


def _radius(vec: tuple[float, float, float]) -> float:
    return math.sqrt(vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2)


def _spherical_log_map(
    point: tuple[float, float, float],
    *,
    center: tuple[float, float, float],
    radius: float,
    frame: SurfaceFrame,
) -> tuple[float, float]:
    normal = (
        (point[0] - center[0]) / radius,
        (point[1] - center[1]) / radius,
        (point[2] - center[2]) / radius,
    )
    cos_angle = max(-1.0, min(1.0, sum(a * b for a, b in zip(frame.normal, normal))))
    angle = math.acos(cos_angle)
    if angle <= 1e-12:
        return (0.0, 0.0)
    tangent = (
        normal[0] - cos_angle * frame.normal[0],
        normal[1] - cos_angle * frame.normal[1],
        normal[2] - cos_angle * frame.normal[2],
    )
    tangent_norm = math.sqrt(sum(v * v for v in tangent))
    tangent_dir = (
        tangent[0] / tangent_norm,
        tangent[1] / tangent_norm,
        tangent[2] / tangent_norm,
    )
    distance = radius * angle
    return (
        distance * sum(a * b for a, b in zip(tangent_dir, frame.tangent_u)),
        distance * sum(a * b for a, b in zip(tangent_dir, frame.tangent_v)),
    )


def _cylindrical_recover_xy(
    point: tuple[float, float, float],
    *,
    radius: float,
    frame: SurfaceFrame,
) -> tuple[float, float]:
    radial = (point[0], point[1], 0.0)
    radial_norm = math.sqrt(radial[0] ** 2 + radial[1] ** 2)
    radial_dir = (radial[0] / radial_norm, radial[1] / radial_norm, 0.0)
    anchor_radial = (frame.normal[0], frame.normal[1], 0.0)
    anchor_radial_norm = math.sqrt(anchor_radial[0] ** 2 + anchor_radial[1] ** 2)
    anchor_radial_dir = (
        anchor_radial[0] / anchor_radial_norm,
        anchor_radial[1] / anchor_radial_norm,
        0.0,
    )
    sin_angle = anchor_radial_dir[0] * radial_dir[1] - anchor_radial_dir[1] * radial_dir[0]
    cos_angle = anchor_radial_dir[0] * radial_dir[0] + anchor_radial_dir[1] * radial_dir[1]
    circum = radius * math.atan2(sin_angle, cos_angle)
    axial = point[2] - frame.point[2]
    return (axial, -circum)


def test_place_on_surface_mounts_centered_child_flush_to_sphere() -> None:
    origin = place_on_surface(
        child=Box((0.020, 0.030, 0.004)),
        target=Sphere(radius=1.0),
        direction=(1.0, 0.0, 0.0),
        child_axis="+z",
    )

    _assert_vec_close(origin.xyz, (1.002, 0.0, 0.0))
    rotated = _rotate(_rpy_matrix(origin), (0.0, 0.0, 1.0))
    _assert_vec_close(rotated, (1.0, 0.0, 0.0))


def test_surface_frame_respects_visual_origin_offset() -> None:
    target = Visual(
        geometry=Sphere(radius=1.0),
        origin=Origin(xyz=(0.5, -0.25, 0.1)),
    )

    frame = surface_frame(
        target,
        point_hint=(2.5, -0.25, 0.1),
    )

    assert isinstance(frame, SurfaceFrame)
    _assert_vec_close(frame.point, (1.5, -0.25, 0.1))
    _assert_vec_close(frame.normal, (1.0, 0.0, 0.0))


def test_surface_frame_uses_mesh_proximity_backend(tmp_path: Path) -> None:
    mesh: Mesh = mesh_from_geometry(BoxGeometry((2.0, 2.0, 2.0)), tmp_path / "box.obj")

    frame = surface_frame(
        mesh,
        point_hint=(1.8, 0.2, -0.1),
    )

    _assert_vec_close(frame.point, (1.0, 0.2, -0.1), tol=1e-4)
    _assert_vec_close(frame.normal, (1.0, 0.0, 0.0), tol=1e-4)


def test_surface_frame_uses_scaled_source_geometry_provenance() -> None:
    target = Mesh(
        filename="dummy.obj",
        scale=(2.0, 2.0, 2.0),
        source_geometry=Box((1.0, 1.0, 1.0)),
    )

    frame = surface_frame(
        target,
        direction=(0.0, 0.0, 1.0),
    )

    _assert_vec_close(frame.point, (0.0, 0.0, 1.0))

    origin = place_on_surface(
        child=Box((1.0, 1.0, 1.0)),
        target=target,
        direction=(0.0, 0.0, 1.0),
        child_axis="+z",
    )

    _assert_vec_close(origin.xyz, (0.0, 0.0, 1.5))


def test_mesh_helpers_resolve_legacy_mesh_prefix(tmp_path: Path) -> None:
    mesh_from_geometry(
        BoxGeometry((2.0, 4.0, 6.0)),
        tmp_path / "assets" / "meshes" / "box.obj",
    )
    mesh = Mesh(filename="meshes/box.obj")

    aabb = part_local_aabb(
        Part("body", visuals=[Visual(mesh)]),
        asset_root=tmp_path,
        prefer_collisions=False,
    )

    assert aabb == ((-1.0, -2.0, -3.0), (1.0, 2.0, 3.0))

    loaded = placement_module._load_trimesh_mesh(mesh, asset_root=tmp_path)
    assert tuple(float(v) for v in loaded.extents) == (2.0, 4.0, 6.0)


def test_place_on_surface_uses_mesh_target_for_flush_offset(tmp_path: Path) -> None:
    target_mesh: Mesh = mesh_from_geometry(BoxGeometry((2.0, 2.0, 2.0)), tmp_path / "target.obj")

    origin = place_on_surface(
        child=Box((0.100, 0.100, 0.100)),
        target=target_mesh,
        point_hint=(1.8, 0.0, 0.0),
        child_axis="+z",
        clearance=0.010,
    )

    _assert_vec_close(origin.xyz, (1.06, 0.0, 0.0), tol=1e-4)


def test_surface_mesh_cache_refreshes_when_obj_changes(tmp_path: Path) -> None:
    placement_module._TRIMESH_CACHE.clear()

    mesh_path = tmp_path / "assets" / "meshes" / "box.obj"
    mesh_from_geometry(BoxGeometry((1.0, 1.0, 1.0)), mesh_path)
    mesh = Mesh(filename="assets/meshes/box.obj")

    loaded1 = placement_module._load_trimesh_mesh(mesh, asset_root=tmp_path)
    mesh_from_geometry(BoxGeometry((2.0, 2.0, 2.0)), mesh_path)
    loaded2 = placement_module._load_trimesh_mesh(mesh, asset_root=tmp_path)

    assert tuple(float(v) for v in loaded1.extents) == (1.0, 1.0, 1.0)
    assert tuple(float(v) for v in loaded2.extents) == (2.0, 2.0, 2.0)
    assert loaded1 is not loaded2

    placement_module._TRIMESH_CACHE.clear()


def test_wrap_mesh_onto_surface_conforms_visible_face_to_sphere() -> None:
    geom = MeshGeometry(
        vertices=[
            (-0.10, 0.0, 0.0),
            (0.10, 0.0, 0.0),
            (-0.10, 0.0, -0.02),
            (0.10, 0.0, -0.02),
        ],
        faces=[(0, 1, 2), (1, 3, 2)],
    )

    wrapped = wrap_mesh_onto_surface(
        geom,
        Sphere(radius=1.0),
        direction=(1.0, 0.0, 0.0),
        child_axis="+z",
        visible_relief=0.0,
    )

    assert isinstance(wrapped, MeshGeometry)
    outer = wrapped.vertices[:2]
    inner = wrapped.vertices[2:]
    for vertex in outer:
        radius = math.sqrt(vertex[0] ** 2 + vertex[1] ** 2 + vertex[2] ** 2)
        assert abs(radius - 1.0) <= 1e-6
    for outer_vertex, inner_vertex in zip(outer, inner):
        outer_radius = math.sqrt(outer_vertex[0] ** 2 + outer_vertex[1] ** 2 + outer_vertex[2] ** 2)
        inner_radius = math.sqrt(inner_vertex[0] ** 2 + inner_vertex[1] ** 2 + inner_vertex[2] ** 2)
        assert inner_radius < outer_radius
    assert outer[0][2] < 0.0
    assert outer[1][2] > 0.0


def test_wrap_mesh_onto_surface_uses_mesh_target(tmp_path: Path) -> None:
    target_mesh: Mesh = mesh_from_geometry(BoxGeometry((2.0, 2.0, 2.0)), tmp_path / "target.obj")
    geom = MeshGeometry(
        vertices=[
            (-0.05, -0.05, 0.01),
            (0.05, -0.05, 0.01),
            (0.05, 0.05, 0.01),
            (-0.05, 0.05, 0.01),
            (-0.05, -0.05, -0.01),
            (0.05, -0.05, -0.01),
            (0.05, 0.05, -0.01),
            (-0.05, 0.05, -0.01),
        ],
        faces=[(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)],
    )

    wrapped = wrap_mesh_onto_surface(
        geom,
        target_mesh,
        point_hint=(1.2, 0.0, 0.0),
        child_axis="+z",
        visible_relief=0.0,
    )

    outer = wrapped.vertices[:4]
    for vertex in outer:
        assert abs(vertex[0] - 1.0) <= 1e-4


def test_wrap_mesh_onto_surface_subdivides_large_patch_with_max_edge() -> None:
    geom = MeshGeometry(
        vertices=[
            (-0.25, -0.20, 0.0),
            (0.25, -0.20, 0.0),
            (0.00, 0.28, 0.0),
        ],
        faces=[(0, 1, 2)],
    )

    wrapped = wrap_mesh_onto_surface(
        geom,
        Sphere(radius=1.0),
        direction=(1.0, 0.0, 0.0),
        child_axis="+z",
        visible_relief=0.0,
        max_edge=0.05,
    )

    assert len(wrapped.faces) > 1
    for a, b, c in wrapped.faces:
        centroid = (
            (wrapped.vertices[a][0] + wrapped.vertices[b][0] + wrapped.vertices[c][0]) / 3.0,
            (wrapped.vertices[a][1] + wrapped.vertices[b][1] + wrapped.vertices[c][1]) / 3.0,
            (wrapped.vertices[a][2] + wrapped.vertices[b][2] + wrapped.vertices[c][2]) / 3.0,
        )
        assert _radius(centroid) >= 0.998


def test_wrap_mesh_onto_surface_preserves_shape_on_sphere() -> None:
    geom = MeshGeometry(
        vertices=[
            (-0.03, -0.01, 0.0),
            (0.02, -0.015, 0.0),
            (0.015, 0.012, 0.0),
            (-0.025, 0.008, 0.0),
        ],
        faces=[(0, 1, 2), (0, 2, 3)],
    )
    direction = (0.12, -0.97, 0.21)
    frame = surface_frame(
        Sphere(radius=1.0),
        direction=direction,
    )

    wrapped = wrap_mesh_onto_surface(
        geom,
        Sphere(radius=1.0),
        direction=direction,
        child_axis="+z",
        visible_relief=0.0,
    )

    recovered = [
        _spherical_log_map(vertex, center=(0.0, 0.0, 0.0), radius=1.0, frame=frame)
        for vertex in wrapped.vertices
    ]
    expected = [(vertex[0], vertex[1]) for vertex in geom.vertices]
    for actual, target in zip(recovered, expected):
        assert abs(actual[0] - target[0]) <= 1e-6
        assert abs(actual[1] - target[1]) <= 1e-6


def test_wrap_profile_onto_surface_preserves_visible_profile_on_sphere() -> None:
    profile = [
        (-0.03, -0.01),
        (0.02, -0.015),
        (0.015, 0.012),
        (-0.025, 0.008),
    ]
    direction = (0.12, -0.97, 0.21)
    frame = surface_frame(
        Sphere(radius=1.0),
        direction=direction,
    )

    wrapped = wrap_profile_onto_surface(
        profile,
        Sphere(radius=1.0),
        thickness=0.02,
        direction=direction,
        mapping="intrinsic",
        visible_relief=0.0,
    )

    recovered = [
        _spherical_log_map(vertex, center=(0.0, 0.0, 0.0), radius=1.0, frame=frame)
        for vertex in wrapped.vertices[: len(profile)]
    ]
    for actual, target in zip(recovered, profile):
        assert abs(actual[0] - target[0]) <= 1e-6
        assert abs(actual[1] - target[1]) <= 1e-6

    outer = wrapped.vertices[: len(wrapped.vertices) // 2]
    inner = wrapped.vertices[len(wrapped.vertices) // 2 :]
    assert len(outer) == len(inner)
    for outer_vertex, inner_vertex in zip(outer, inner):
        assert abs(_radius(outer_vertex) - 1.0) <= 1e-6
        assert _radius(inner_vertex) < _radius(outer_vertex)


def test_wrap_profile_onto_surface_subdivides_only_profile_cap() -> None:
    profile = [
        (-0.25, -0.20),
        (0.25, -0.20),
        (0.0, 0.28),
    ]

    wrapped = wrap_profile_onto_surface(
        profile,
        Sphere(radius=1.0),
        thickness=0.02,
        direction=(1.0, 0.0, 0.0),
        mapping="intrinsic",
        visible_relief=0.0,
        surface_max_edge=0.05,
    )

    assert len(wrapped.faces) > 8
    outer_vertex_count = len(wrapped.vertices) // 2
    outer_faces = [
        (a, b, c)
        for a, b, c in wrapped.faces
        if a < outer_vertex_count and b < outer_vertex_count and c < outer_vertex_count
    ]
    assert outer_faces
    for a, b, c in outer_faces:
        centroid = (
            (wrapped.vertices[a][0] + wrapped.vertices[b][0] + wrapped.vertices[c][0]) / 3.0,
            (wrapped.vertices[a][1] + wrapped.vertices[b][1] + wrapped.vertices[c][1]) / 3.0,
            (wrapped.vertices[a][2] + wrapped.vertices[b][2] + wrapped.vertices[c][2]) / 3.0,
        )
        assert _radius(centroid) >= 0.998


def test_wrap_profile_onto_surface_supports_hole_profiles(
    caplog: pytest.LogCaptureFixture,
) -> None:
    outer = [
        (-0.04, -0.04),
        (0.04, -0.04),
        (0.04, 0.04),
        (-0.04, 0.04),
    ]
    hole = [
        (-0.012, -0.012),
        (0.012, -0.012),
        (0.012, 0.012),
        (-0.012, 0.012),
    ]
    frame = surface_frame(
        Sphere(radius=1.0),
        direction=(1.0, 0.0, 0.0),
    )

    with caplog.at_level(logging.WARNING):
        wrapped = wrap_profile_onto_surface(
            outer,
            Sphere(radius=1.0),
            thickness=0.015,
            hole_profiles=[hole],
            direction=(1.0, 0.0, 0.0),
            mapping="intrinsic",
            visible_relief=0.0,
            surface_max_edge=0.03,
        )

    visible_points = [
        _spherical_log_map(vertex, center=(0.0, 0.0, 0.0), radius=1.0, frame=frame)
        for vertex in wrapped.vertices
        if abs(_radius(vertex) - 1.0) <= 1e-6
    ]
    assert visible_points

    expected_points = outer + hole
    for expected in expected_points:
        assert (
            min(
                math.hypot(actual[0] - expected[0], actual[1] - expected[1])
                for actual in visible_points
            )
            <= 1e-6
        )
    assert "hole_profiles were provided" in caplog.text


def test_wrap_profile_onto_surface_logs_boundary_fallback(
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    def fail_planar_cap(*args, **kwargs):
        raise ValidationError("profile mesh boundary could not be reconstructed")

    monkeypatch.setattr(placement_module, "_solid_from_planar_profile_cap", fail_planar_cap)

    with caplog.at_level(logging.WARNING):
        wrapped = wrap_profile_onto_surface(
            [
                (-0.04, -0.04),
                (0.04, -0.04),
                (0.04, 0.04),
                (-0.04, 0.04),
            ],
            Sphere(radius=1.0),
            thickness=0.015,
            direction=(1.0, 0.0, 0.0),
            mapping="intrinsic",
            visible_relief=0.0,
            surface_max_edge=0.03,
        )

    assert wrapped.vertices
    assert "after planar cap reconstruction failed" in caplog.text


def test_wrap_profile_onto_surface_handles_complex_profile_robustly() -> None:
    points = [
        (-0.040, 0.010),
        (-0.030, 0.025),
        (-0.010, 0.036),
        (0.014, 0.034),
        (0.035, 0.026),
        (0.045, 0.010),
        (0.040, -0.004),
        (0.020, -0.012),
        (0.012, -0.032),
        (-0.004, -0.040),
        (-0.018, -0.024),
        (-0.032, -0.012),
    ]
    profile = sample_catmull_rom_spline_2d(
        points,
        samples_per_segment=10,
        closed=True,
    )

    wrapped = wrap_profile_onto_surface(
        profile,
        Sphere(radius=1.0),
        thickness=0.016,
        direction=(0.8, 0.3, 0.52),
        mapping="intrinsic",
        visible_relief=0.0,
        surface_max_edge=0.006,
    )

    radii = [_radius(vertex) for vertex in wrapped.vertices]
    assert len(wrapped.faces) > 1000
    assert max(radii) >= 0.999999
    assert min(radii) < 0.999


def test_wrap_mesh_onto_surface_preserves_shape_on_cylinder_sidewall() -> None:
    geom = MeshGeometry(
        vertices=[
            (-0.03, -0.01, 0.0),
            (0.02, -0.015, 0.0),
            (0.015, 0.012, 0.0),
            (-0.025, 0.008, 0.0),
        ],
        faces=[(0, 1, 2), (0, 2, 3)],
    )
    frame = surface_frame(
        Cylinder(radius=1.0, length=4.0),
        direction=(1.0, 0.0, 0.0),
    )

    wrapped = wrap_mesh_onto_surface(
        geom,
        Cylinder(radius=1.0, length=4.0),
        direction=(1.0, 0.0, 0.0),
        child_axis="+z",
        mapping="intrinsic",
        visible_relief=0.0,
    )

    recovered = [
        _cylindrical_recover_xy(vertex, radius=1.0, frame=frame) for vertex in wrapped.vertices
    ]
    expected = [(vertex[0], vertex[1]) for vertex in geom.vertices]
    for actual, target in zip(recovered, expected):
        assert abs(actual[0] - target[0]) <= 1e-6
        assert abs(actual[1] - target[1]) <= 1e-6


def test_wrap_mesh_onto_surface_intrinsic_mapping_rejects_unsupported_target(
    tmp_path: Path,
) -> None:
    target_mesh: Mesh = mesh_from_geometry(BoxGeometry((2.0, 2.0, 2.0)), tmp_path / "target.obj")
    geom = MeshGeometry(
        vertices=[(-0.05, -0.05, 0.0), (0.05, -0.05, 0.0), (0.0, 0.05, 0.0)],
        faces=[(0, 1, 2)],
    )

    with pytest.raises(ValidationError, match="Intrinsic surface wrapping"):
        wrap_mesh_onto_surface(
            geom,
            target_mesh,
            point_hint=(1.2, 0.0, 0.0),
            mapping="intrinsic",
        )


def test_wrap_mesh_onto_surface_rejects_conflicting_edge_settings() -> None:
    geom = MeshGeometry(
        vertices=[(-0.05, -0.05, 0.0), (0.05, -0.05, 0.0), (0.0, 0.05, 0.0)],
        faces=[(0, 1, 2)],
    )

    with pytest.raises(ValidationError, match="surface_max_edge or max_edge"):
        wrap_mesh_onto_surface(
            geom,
            Sphere(radius=1.0),
            direction=(1.0, 0.0, 0.0),
            surface_max_edge=0.05,
            max_edge=0.10,
        )
