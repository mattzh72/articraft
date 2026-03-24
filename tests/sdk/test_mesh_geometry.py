from __future__ import annotations

import logging
from math import pi

import pytest

import sdk
from sdk._core.v0 import mesh as mesh_module


def _bounds(
    geom: sdk.MeshGeometry,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    xs = [v[0] for v in geom.vertices]
    ys = [v[1] for v in geom.vertices]
    zs = [v[2] for v in geom.vertices]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def test_dome_geometry_builds_closed_hemisphere() -> None:
    geom = sdk.DomeGeometry(0.4, radial_segments=18, height_segments=9)

    assert len(geom.vertices) > 0
    assert len(geom.faces) > 0

    mins, maxs = _bounds(geom)
    assert mins[0] <= -0.39
    assert maxs[0] >= 0.39
    assert mins[1] <= -0.39
    assert maxs[1] >= 0.39
    assert mins[2] >= -1e-9
    assert maxs[2] >= 0.399


def test_dome_geometry_supports_ellipsoidal_radii() -> None:
    geom = sdk.DomeGeometry((0.30, 0.20, 0.10), radial_segments=16, height_segments=8)

    mins, maxs = _bounds(geom)
    assert mins[0] <= -0.29
    assert maxs[0] >= 0.29
    assert mins[1] <= -0.19
    assert maxs[1] >= 0.19
    assert mins[2] >= -1e-9
    assert maxs[2] >= 0.099


def test_capsule_geometry_builds_sphere_capped_cylinder() -> None:
    geom = sdk.CapsuleGeometry(0.05, 0.20, radial_segments=18, height_segments=8)

    assert len(geom.vertices) > 0
    assert len(geom.faces) > 0

    mins, maxs = _bounds(geom)
    assert mins[0] <= -0.049
    assert maxs[0] >= 0.049
    assert mins[1] <= -0.049
    assert maxs[1] >= 0.049
    assert mins[2] <= -0.149
    assert maxs[2] >= 0.149


def test_rotate_supports_arbitrary_axis_rotation() -> None:
    geom = sdk.BoxGeometry((0.20, 0.10, 0.06)).rotate((0.0, 0.0, 1.0), pi / 2.0)

    mins, maxs = _bounds(geom)
    assert mins[0] == pytest.approx(-0.05, abs=1e-9)
    assert maxs[0] == pytest.approx(0.05, abs=1e-9)
    assert mins[1] == pytest.approx(-0.10, abs=1e-9)
    assert maxs[1] == pytest.approx(0.10, abs=1e-9)
    assert mins[2] == pytest.approx(-0.03, abs=1e-9)
    assert maxs[2] == pytest.approx(0.03, abs=1e-9)


def test_rotate_supports_custom_origin() -> None:
    geom = sdk.MeshGeometry(vertices=[(2.0, 0.0, 0.0)], faces=[])

    geom.rotate((0.0, 0.0, 1.0), pi, origin=(1.0, 0.0, 0.0))

    x, y, z = geom.vertices[0]
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert z == pytest.approx(0.0, abs=1e-9)


def test_lathe_geometry_builds_closed_manifold_solid() -> None:
    geom = sdk.LatheGeometry(
        [
            (0.0, -0.20),
            (0.05, -0.20),
            (0.05, 0.20),
            (0.0, 0.20),
        ],
        segments=24,
    )

    mesh_module._manifold_from_geometry(geom, name="lathe")

    mins, maxs = _bounds(geom)
    assert mins[0] <= -0.049
    assert maxs[0] >= 0.049
    assert mins[1] <= -0.049
    assert maxs[1] >= 0.049
    assert mins[2] == pytest.approx(-0.20, abs=1e-6)
    assert maxs[2] == pytest.approx(0.20, abs=1e-6)


def test_lathe_geometry_supports_boolean_difference() -> None:
    outer = sdk.LatheGeometry(
        [
            (0.0, -0.20),
            (0.06, -0.20),
            (0.06, 0.20),
            (0.0, 0.20),
        ],
        segments=28,
    )
    inner = sdk.LatheGeometry(
        [
            (0.0, -0.16),
            (0.03, -0.16),
            (0.03, 0.16),
            (0.0, 0.16),
        ],
        segments=28,
    )

    shell = sdk.boolean_difference(outer, inner)

    mesh_module._manifold_from_geometry(shell, name="lathe_shell")

    mins, maxs = _bounds(shell)
    assert mins[0] <= -0.059
    assert maxs[0] >= 0.059
    assert mins[2] == pytest.approx(-0.20, abs=1e-6)
    assert maxs[2] == pytest.approx(0.20, abs=1e-6)


def test_lathe_geometry_closed_false_preserves_open_surface_mode() -> None:
    geom = sdk.LatheGeometry(
        [
            (0.04, -0.10),
            (0.04, 0.10),
        ],
        segments=20,
        closed=False,
    )

    with pytest.raises(ValueError, match="not a valid manifold solid"):
        mesh_module._manifold_from_geometry(geom, name="open_lathe")


def test_mesh_from_geometry_preserves_general_rotation_transform(tmp_path) -> None:
    mesh = sdk.mesh_from_geometry(
        sdk.BoxGeometry((0.20, 0.10, 0.06)).rotate((0.0, 0.0, 1.0), pi / 2.0),
        tmp_path / "assets" / "meshes" / "rotated_box.obj",
    )

    assert mesh.filename == "assets/meshes/rotated_box.obj"
    assert isinstance(mesh.source_geometry, sdk.Box)
    assert mesh.source_transform is not None

    row0, row1, row2, row3 = mesh.source_transform
    assert row0[0] == pytest.approx(0.0, abs=1e-9)
    assert row0[1] == pytest.approx(-1.0, abs=1e-9)
    assert row1[0] == pytest.approx(1.0, abs=1e-9)
    assert row1[1] == pytest.approx(0.0, abs=1e-9)
    assert row2[2] == pytest.approx(1.0, abs=1e-9)
    assert row3 == (0.0, 0.0, 0.0, 1.0)


def test_tube_from_spline_points_supports_bezier_paths() -> None:
    geom = sdk.tube_from_spline_points(
        [
            (0.00, 0.00, 0.00),
            (0.08, 0.00, 0.12),
            (0.16, 0.00, 0.12),
            (0.24, 0.00, 0.00),
        ],
        radius=0.01,
        spline="bezier",
        samples_per_segment=18,
        radial_segments=14,
    )

    assert len(geom.vertices) > 0
    assert len(geom.faces) > 0

    mins, maxs = _bounds(geom)
    assert mins[0] <= 0.001
    assert maxs[0] >= 0.239
    assert maxs[2] >= 0.08


def test_sweep_profile_along_spline_supports_cubic_bezier_alias() -> None:
    geom = sdk.sweep_profile_along_spline(
        [
            (0.00, 0.00, 0.00),
            (0.05, 0.04, 0.08),
            (0.15, -0.04, 0.08),
            (0.20, 0.00, 0.00),
        ],
        profile=sdk.rounded_rect_profile(0.02, 0.01, radius=0.002),
        spline="cubic_bezier",
        samples_per_segment=16,
    )

    assert len(geom.vertices) > 0
    assert len(geom.faces) > 0

    mins, maxs = _bounds(geom)
    assert mins[0] <= 0.001
    assert maxs[0] >= 0.199
    assert maxs[2] >= 0.05


def test_closed_bezier_spline_requires_closed_curve() -> None:
    with pytest.raises(ValueError, match="end where it starts"):
        sdk.tube_from_spline_points(
            [
                (0.00, 0.00, 0.00),
                (0.08, 0.00, 0.12),
                (0.16, 0.00, 0.12),
                (0.24, 0.00, 0.00),
            ],
            radius=0.01,
            spline="bezier",
            closed_spline=True,
        )


def test_tube_network_from_paths_logs_visual_mesh_fallback(
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    sentinel = sdk.MeshGeometry(
        vertices=[(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.1, 0.0)],
        faces=[(0, 1, 2)],
    )

    def fail_boolean_union(_geometries):
        raise ValueError("geometry[0] is not a valid manifold solid")

    monkeypatch.setattr(mesh_module, "_boolean_union_many", fail_boolean_union)
    monkeypatch.setattr(
        mesh_module,
        "_tube_network_visual_mesh_from_paths",
        lambda *args, **kwargs: sentinel,
    )

    with caplog.at_level(logging.WARNING):
        geometry = mesh_module.tube_network_from_paths(
            [[(0.0, 0.0, 0.0), (0.0, 0.0, 0.2)]],
            radius=0.02,
        )

    assert geometry is sentinel
    assert "falling back to visual mesh" in caplog.text
