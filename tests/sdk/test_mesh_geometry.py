from __future__ import annotations

import hashlib
import logging
from collections import defaultdict, deque
from math import pi
from pathlib import Path

import pytest

import sdk
from sdk._core.v0 import mesh as mesh_module
from sdk._core.v0.assets import AssetSession, activate_asset_session


def _bounds(
    geom: sdk.MeshGeometry,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    xs = [v[0] for v in geom.vertices]
    ys = [v[1] for v in geom.vertices]
    zs = [v[2] for v in geom.vertices]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def _managed_mesh_suffix(geometry: sdk.MeshGeometry) -> str:
    return hashlib.sha256(geometry.to_obj().encode("utf-8")).hexdigest()[:12]


def _component_count(geom: sdk.MeshGeometry) -> int:
    adjacency: dict[int, set[int]] = defaultdict(set)
    for a, b, c in geom.faces:
        adjacency[a].update((b, c))
        adjacency[b].update((a, c))
        adjacency[c].update((a, b))

    seen: set[int] = set()
    count = 0
    for start in range(len(geom.vertices)):
        if start in seen:
            continue
        count += 1
        queue: deque[int] = deque([start])
        seen.add(start)
        while queue:
            current = queue.popleft()
            for neighbor in adjacency[current]:
                if neighbor in seen:
                    continue
                seen.add(neighbor)
                queue.append(neighbor)
    return count


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


def test_lathe_geometry_from_shell_profiles_supports_flat_lips() -> None:
    geom = sdk.LatheGeometry.from_shell_profiles(
        [
            (0.02, -0.10),
            (0.05, -0.08),
            (0.09, 0.04),
        ],
        [
            (0.00, -0.09),
            (0.03, -0.07),
            (0.07, 0.04),
        ],
        segments=24,
        start_cap="flat",
        end_cap="flat",
    )

    mesh_module._manifold_from_geometry(geom, name="flat_shell")

    mins, maxs = _bounds(geom)
    assert mins[0] <= -0.089
    assert maxs[0] >= 0.089
    assert mins[2] == pytest.approx(-0.10, abs=1e-6)
    assert maxs[2] == pytest.approx(0.04, abs=1e-6)


def test_lathe_geometry_from_shell_profiles_supports_rounded_lips() -> None:
    flat = sdk.LatheGeometry.from_shell_profiles(
        [
            (0.03, -0.14),
            (0.07, -0.10),
            (0.11, 0.00),
        ],
        [
            (0.00, -0.12),
            (0.04, -0.08),
            (0.08, 0.00),
        ],
        segments=28,
        end_cap="flat",
    )
    rounded = sdk.LatheGeometry.from_shell_profiles(
        [
            (0.03, -0.14),
            (0.07, -0.10),
            (0.11, 0.00),
        ],
        [
            (0.00, -0.12),
            (0.04, -0.08),
            (0.08, 0.00),
        ],
        segments=28,
        end_cap="round",
        lip_samples=8,
    )

    mesh_module._manifold_from_geometry(rounded, name="rounded_shell")

    assert len(rounded.vertices) > len(flat.vertices)
    assert len(rounded.faces) > len(flat.faces)

    flat_max_z = max(z for (_x, _y, z) in flat.vertices)
    rounded_max_z = max(z for (_x, _y, z) in rounded.vertices)
    assert rounded_max_z > flat_max_z


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


def test_mesh_from_geometry_uses_managed_logical_names_without_paths() -> None:
    mesh = sdk.mesh_from_geometry(
        sdk.BoxGeometry((0.20, 0.10, 0.06)),
        "door_panel",
    )

    assert mesh.filename == "assets/meshes/door_panel.obj"
    assert mesh.name == "door_panel"
    assert mesh.materialized_path is not None
    assert Path(mesh.materialized_path).exists()


def test_mesh_from_input_uses_managed_input_catalog(tmp_path) -> None:
    inputs_dir = tmp_path / "inputs"
    inputs_dir.mkdir(parents=True, exist_ok=True)
    (inputs_dir / "plate.obj").write_text(
        "\n".join(
            [
                "o plate",
                "v 0 0 0",
                "v 1 0 0",
                "v 0 1 0",
                "f 1 2 3",
                "",
            ]
        ),
        encoding="utf-8",
    )

    session = AssetSession(tmp_path, inputs_root=inputs_dir)
    with activate_asset_session(session):
        mesh = sdk.mesh_from_input("plate")

    assert mesh.filename == "assets/meshes/plate.obj"
    assert mesh.materialized_path is not None
    assert Path(mesh.materialized_path).exists()


def test_mesh_from_geometry_dedupes_same_logical_name_and_geometry_in_session(tmp_path) -> None:
    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        first = sdk.mesh_from_geometry(sdk.BoxGeometry((0.20, 0.10, 0.06)), "shared_name")
        second = sdk.mesh_from_geometry(sdk.BoxGeometry((0.20, 0.10, 0.06)), "shared_name")

    assert first.filename == "assets/meshes/shared_name.obj"
    assert second.filename == first.filename
    assert second.materialized_path == first.materialized_path


def test_mesh_from_geometry_allocates_suffix_for_same_name_different_geometry_in_session(
    tmp_path,
) -> None:
    base_geometry = sdk.BoxGeometry((0.20, 0.10, 0.06))
    alternate_geometry = sdk.BoxGeometry((0.30, 0.10, 0.06))
    expected_suffix = _managed_mesh_suffix(alternate_geometry)

    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        base_mesh = sdk.mesh_from_geometry(base_geometry, "shared_name")
        alternate_mesh = sdk.mesh_from_geometry(alternate_geometry, "shared_name")

    assert base_mesh.filename == "assets/meshes/shared_name.obj"
    assert alternate_mesh.filename == f"assets/meshes/shared_name--{expected_suffix}.obj"
    assert alternate_mesh.materialized_path is not None
    assert Path(str(alternate_mesh.materialized_path)).exists()


def test_mesh_from_geometry_dedupes_same_slug_and_geometry_across_logical_names(tmp_path) -> None:
    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        first = sdk.mesh_from_geometry(sdk.BoxGeometry((0.20, 0.10, 0.06)), "door panel")
        second = sdk.mesh_from_geometry(sdk.BoxGeometry((0.20, 0.10, 0.06)), "door-panel")

    assert first.filename == "assets/meshes/door-panel.obj"
    assert second.filename == first.filename
    assert second.materialized_path == first.materialized_path
    assert first.name == "door panel"
    assert second.name == "door-panel"


def test_mesh_from_geometry_allocates_suffix_for_same_slug_different_geometry(tmp_path) -> None:
    alternate_geometry = sdk.BoxGeometry((0.30, 0.10, 0.06))
    expected_suffix = _managed_mesh_suffix(alternate_geometry)

    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        base_mesh = sdk.mesh_from_geometry(sdk.BoxGeometry((0.20, 0.10, 0.06)), "door panel")
        alternate_mesh = sdk.mesh_from_geometry(alternate_geometry, "door-panel")

    assert base_mesh.filename == "assets/meshes/door-panel.obj"
    assert alternate_mesh.filename == f"assets/meshes/door-panel--{expected_suffix}.obj"


def test_mesh_from_geometry_overwrites_stale_managed_mesh_from_prior_session(tmp_path) -> None:
    first_session = AssetSession(tmp_path)
    with activate_asset_session(first_session):
        first_mesh = sdk.mesh_from_geometry(sdk.BoxGeometry((0.20, 0.10, 0.06)), "shared_name")

    mesh_path = Path(str(first_mesh.materialized_path))
    first_bytes = mesh_path.read_bytes()

    second_session = AssetSession(tmp_path)
    with activate_asset_session(second_session):
        second_mesh = sdk.mesh_from_geometry(sdk.BoxGeometry((0.30, 0.10, 0.06)), "shared_name")

    assert second_mesh.filename == "assets/meshes/shared_name.obj"
    assert Path(str(second_mesh.materialized_path)) == mesh_path
    assert mesh_path.read_bytes() != first_bytes


def test_mesh_from_geometry_allocates_deterministic_suffix_for_same_payload(tmp_path) -> None:
    geometry = sdk.BoxGeometry((0.30, 0.10, 0.06))

    first_session = AssetSession(tmp_path / "a")
    with activate_asset_session(first_session):
        sdk.mesh_from_geometry(sdk.BoxGeometry((0.20, 0.10, 0.06)), "shared_name")
        first_conflict = sdk.mesh_from_geometry(geometry, "shared_name")

    second_session = AssetSession(tmp_path / "b")
    with activate_asset_session(second_session):
        sdk.mesh_from_geometry(sdk.BoxGeometry((0.20, 0.10, 0.06)), "shared_name")
        second_conflict = sdk.mesh_from_geometry(geometry, "shared_name")

    assert first_conflict.filename == second_conflict.filename
    assert (
        first_conflict.filename
        == f"assets/meshes/shared_name--{_managed_mesh_suffix(geometry)}.obj"
    )


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


def test_extrude_with_holes_geometry_builds_manifold_panel() -> None:
    geom = sdk.ExtrudeWithHolesGeometry(
        sdk.rounded_rect_profile(0.12, 0.08, radius=0.006),
        [sdk.rounded_rect_profile(0.095, 0.0108, radius=0.002)],
        0.01,
    )

    mesh_module._manifold_from_geometry(geom, name="panel_with_hole")

    assert len(geom.vertices) > 0
    assert len(geom.faces) > 0


def test_legacy_internal_louver_panel_geometry_builds_closed_panel_with_expected_bounds() -> None:
    geom = mesh_module.LouverPanelGeometry(
        (0.12, 0.08),
        0.01,
        frame=0.01,
        slat_pitch=0.02,
        slat_width=0.008,
        slat_angle_deg=30.0,
        corner_radius=0.006,
    )

    mesh_module._manifold_from_geometry(geom, name="louver_panel")

    assert len(geom.vertices) > 0
    assert len(geom.faces) > 0
    assert _component_count(geom) == 1

    mins, maxs = _bounds(geom)
    assert mins[0] == pytest.approx(-0.06, abs=1e-6)
    assert maxs[0] == pytest.approx(0.06, abs=1e-6)
    assert mins[1] == pytest.approx(-0.04, abs=1e-6)
    assert maxs[1] == pytest.approx(0.04, abs=1e-6)
    assert mins[2] == pytest.approx(-0.005, abs=1e-6)
    assert maxs[2] == pytest.approx(0.005, abs=1e-6)


def test_legacy_internal_louver_panel_geometry_exports_clean_single_body_mesh(
    tmp_path: Path,
) -> None:
    import trimesh

    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        mesh = sdk.mesh_from_geometry(
            mesh_module.LouverPanelGeometry(
                (0.12, 0.08),
                0.01,
                frame=0.01,
                slat_pitch=0.02,
                slat_width=0.008,
                slat_angle_deg=30.0,
                corner_radius=0.006,
            ),
            "louver_panel",
        )

    exported = trimesh.load_mesh(mesh.materialized_path, force="mesh")
    assert exported.is_watertight
    assert exported.body_count == 1
    assert len(exported.split(only_watertight=False)) == 1

    mins, maxs = exported.bounds
    assert mins[0] == pytest.approx(-0.06, abs=1e-3)
    assert maxs[0] == pytest.approx(0.06, abs=1e-3)
    assert mins[1] == pytest.approx(-0.04, abs=1e-3)
    assert maxs[1] == pytest.approx(0.04, abs=1e-3)
    assert mins[2] == pytest.approx(-0.005, abs=1e-3)
    assert maxs[2] == pytest.approx(0.005, abs=1e-3)


def test_sdk_does_not_expose_louver_panel_geometry() -> None:
    assert "LouverPanelGeometry" not in sdk.__all__
    with pytest.raises(AttributeError, match="no longer exposes"):
        getattr(sdk, "LouverPanelGeometry")


def test_legacy_internal_louver_panel_geometry_requires_at_least_one_slat_row() -> None:
    with pytest.raises(ValueError, match="No louver rows fit panel"):
        mesh_module.LouverPanelGeometry(
            (0.12, 0.03),
            0.01,
            frame=0.01,
            slat_pitch=0.02,
            slat_width=0.008,
        )


def test_boolean_union_exports_clean_single_body_mesh(tmp_path: Path) -> None:
    import trimesh

    left = sdk.BoxGeometry((1.0, 1.0, 1.0)).translate(-0.2, 0.0, 0.0)
    right = sdk.BoxGeometry((1.0, 1.0, 1.0)).translate(0.2, 0.0, 0.0)

    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        mesh = sdk.mesh_from_geometry(sdk.boolean_union(left, right), "boolean_union_box")

    exported = trimesh.load_mesh(mesh.materialized_path, force="mesh")
    assert exported.is_watertight
    assert exported.body_count == 1
    assert len(exported.split(only_watertight=False)) == 1

    mins, maxs = exported.bounds
    assert mins[0] == pytest.approx(-0.7, abs=1e-3)
    assert maxs[0] == pytest.approx(0.7, abs=1e-3)
    assert mins[1] == pytest.approx(-0.5, abs=1e-3)
    assert maxs[1] == pytest.approx(0.5, abs=1e-3)
    assert mins[2] == pytest.approx(-0.5, abs=1e-3)
    assert maxs[2] == pytest.approx(0.5, abs=1e-3)


def test_boolean_difference_exports_clean_single_body_mesh(tmp_path: Path) -> None:
    import trimesh

    outer = sdk.BoxGeometry((0.18, 0.10, 0.026))
    inner = sdk.BoxGeometry((0.174, 0.094, 0.03))
    shell = sdk.boolean_difference(outer, inner)

    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        mesh = sdk.mesh_from_geometry(shell, "boolean_difference_shell")

    exported = trimesh.load_mesh(mesh.materialized_path, force="mesh")
    assert exported.is_watertight
    assert exported.body_count == 1
    assert len(exported.split(only_watertight=False)) == 1

    mins, maxs = exported.bounds
    assert mins[0] == pytest.approx(-0.09, abs=1e-3)
    assert maxs[0] == pytest.approx(0.09, abs=1e-3)
    assert mins[1] == pytest.approx(-0.05, abs=1e-3)
    assert maxs[1] == pytest.approx(0.05, abs=1e-3)
    assert mins[2] == pytest.approx(-0.013, abs=1e-3)
    assert maxs[2] == pytest.approx(0.013, abs=1e-3)


def test_vent_grille_geometry_builds_with_expected_bounds(tmp_path: Path) -> None:
    import trimesh

    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        mesh = sdk.mesh_from_geometry(
            sdk.VentGrilleGeometry(
                (0.18, 0.10),
                frame=0.012,
                face_thickness=0.004,
                duct_depth=0.026,
                duct_wall=0.003,
                slat_pitch=0.018,
                slat_width=0.009,
                slat_angle_deg=35.0,
                corner_radius=0.006,
            ),
            "vent_grille",
        )

    exported = trimesh.load_mesh(mesh.materialized_path, force="mesh")
    assert exported.is_watertight
    assert exported.body_count == 1
    assert len(exported.split(only_watertight=False)) == 1

    mins, maxs = exported.bounds
    assert mins[0] == pytest.approx(-0.09, abs=1e-3)
    assert maxs[0] == pytest.approx(0.09, abs=1e-3)
    assert mins[1] == pytest.approx(-0.05, abs=1e-3)
    assert maxs[1] == pytest.approx(0.05, abs=1e-3)
    assert mins[2] < -0.025
    assert maxs[2] > 0.0


def test_vent_grille_geometry_requires_at_least_one_slat_row() -> None:
    with pytest.raises(ValueError, match="No slat rows fit panel"):
        sdk.VentGrilleGeometry(
            (0.18, 0.03),
            frame=0.008,
            slat_pitch=0.018,
            slat_width=0.009,
        )


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
