from __future__ import annotations

import hashlib
import logging
from collections import defaultdict, deque
from math import pi
from pathlib import Path

import pytest

import sdk
import sdk.v0 as sdk_v0
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


def _load_exported_mesh(mesh_path: str | Path):
    import trimesh

    return trimesh.load_mesh(mesh_path, force="mesh")


def _assert_clean_export(
    tmp_path: Path,
    geometry: sdk.MeshGeometry,
    logical_name: str,
):
    session = AssetSession(tmp_path)
    with activate_asset_session(session):
        mesh = sdk.mesh_from_geometry(geometry, logical_name)

    exported = _load_exported_mesh(mesh.materialized_path)
    assert exported.is_watertight
    assert exported.body_count == 1
    assert len(exported.split(only_watertight=False)) == 1
    return exported


def _build_perforated_panel_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.PerforatedPanelGeometry(
        (0.16, 0.10),
        0.004,
        hole_diameter=0.006,
        pitch=(0.012, 0.012),
        frame=0.010,
        corner_radius=0.004,
        stagger=True,
        center=center,
    )


def _build_slot_pattern_panel_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.SlotPatternPanelGeometry(
        (0.18, 0.09),
        0.004,
        slot_size=(0.024, 0.006),
        pitch=(0.032, 0.016),
        frame=0.010,
        corner_radius=0.004,
        slot_angle_deg=18.0,
        stagger=True,
        center=center,
    )


def _build_clevis_bracket_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.ClevisBracketGeometry(
        (0.08, 0.04, 0.06),
        gap_width=0.032,
        bore_diameter=0.012,
        bore_center_z=0.038,
        base_thickness=0.012,
        corner_radius=0.003,
        center=center,
    )


def _build_pivot_fork_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.PivotForkGeometry(
        (0.08, 0.05, 0.05),
        gap_width=0.034,
        bore_diameter=0.010,
        bore_center_z=0.028,
        bridge_thickness=0.012,
        corner_radius=0.002,
        center=center,
    )


def _build_trunnion_yoke_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.TrunnionYokeGeometry(
        (0.12, 0.05, 0.08),
        span_width=0.060,
        trunnion_diameter=0.016,
        trunnion_center_z=0.050,
        base_thickness=0.014,
        corner_radius=0.003,
        center=center,
    )


def _build_fan_rotor_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.FanRotorGeometry(
        0.070,
        0.020,
        5,
        thickness=0.010,
        blade_pitch_deg=24.0,
        blade_sweep_deg=14.0,
        center=center,
    )


def _build_blower_wheel_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.BlowerWheelGeometry(
        0.080,
        0.040,
        0.050,
        18,
        blade_thickness=0.004,
        blade_sweep_deg=25.0,
        center=center,
    )


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


@pytest.mark.parametrize(
    ("builder", "logical_name", "expected_mins", "expected_maxs", "tol"),
    [
        pytest.param(
            _build_perforated_panel_geometry,
            "perforated_panel",
            (-0.080, -0.050, -0.002),
            (0.080, 0.050, 0.002),
            1e-3,
            id="perforated-panel",
        ),
        pytest.param(
            _build_slot_pattern_panel_geometry,
            "slot_pattern_panel",
            (-0.090, -0.045, -0.002),
            (0.090, 0.045, 0.002),
            1e-3,
            id="slot-pattern-panel",
        ),
        pytest.param(
            _build_clevis_bracket_geometry,
            "clevis_bracket",
            (-0.040, -0.020, -0.030),
            (0.040, 0.020, 0.030),
            1e-3,
            id="clevis-bracket",
        ),
        pytest.param(
            _build_pivot_fork_geometry,
            "pivot_fork",
            (-0.040, -0.025, -0.025),
            (0.040, 0.025, 0.025),
            1e-3,
            id="pivot-fork",
        ),
        pytest.param(
            _build_trunnion_yoke_geometry,
            "trunnion_yoke",
            (-0.060, -0.025, -0.040),
            (0.060, 0.025, 0.040),
            1e-3,
            id="trunnion-yoke",
        ),
        pytest.param(
            _build_fan_rotor_geometry,
            "fan_rotor",
            (-0.060, -0.065, -0.0038),
            (0.067, 0.067, 0.0050),
            2.5e-3,
            id="fan-rotor",
        ),
        pytest.param(
            _build_blower_wheel_geometry,
            "blower_wheel",
            (-0.080, -0.080, -0.025),
            (0.080, 0.080, 0.025),
            1.5e-3,
            id="blower-wheel",
        ),
    ],
)
def test_new_mesh_geometry_helpers_export_clean_single_body_meshes(
    tmp_path: Path,
    builder,
    logical_name: str,
    expected_mins: tuple[float, float, float],
    expected_maxs: tuple[float, float, float],
    tol: float,
) -> None:
    geometry = builder()

    mesh_module._manifold_from_geometry(geometry, name=logical_name)
    exported = _assert_clean_export(tmp_path, geometry, logical_name)

    mins, maxs = exported.bounds
    for axis, expected in enumerate(expected_mins):
        assert mins[axis] == pytest.approx(expected, abs=tol)
    for axis, expected in enumerate(expected_maxs):
        assert maxs[axis] == pytest.approx(expected, abs=tol)


@pytest.mark.parametrize(
    ("builder", "name"),
    [
        pytest.param(
            _build_perforated_panel_geometry, "PerforatedPanelGeometry", id="perforated-panel"
        ),
        pytest.param(_build_blower_wheel_geometry, "BlowerWheelGeometry", id="blower-wheel"),
    ],
)
def test_new_mesh_geometry_helpers_support_center_false_z0_mount_frames(builder, name: str) -> None:
    geometry = builder(center=False)
    mins, _maxs = _bounds(geometry)
    assert mins[2] == pytest.approx(0.0, abs=1e-6), name


@pytest.mark.parametrize(
    ("builder", "message"),
    [
        pytest.param(
            lambda: sdk.PerforatedPanelGeometry(
                (0.12, 0.08),
                0.004,
                hole_diameter=0.008,
                pitch=0.012,
                frame=0.05,
            ),
            "frame",
            id="perforated-frame-too-large",
        ),
        pytest.param(
            lambda: sdk.PerforatedPanelGeometry(
                (0.12, 0.08),
                0.004,
                hole_diameter=0.008,
                pitch=0.008,
                frame=0.01,
            ),
            "pitch must be greater than hole_diameter",
            id="perforated-pitch-too-small",
        ),
        pytest.param(
            lambda: sdk.PerforatedPanelGeometry(
                (0.12, 0.08),
                0.004,
                hole_diameter=0.050,
                pitch=(0.060, 0.060),
                frame=0.020,
            ),
            "leave no usable perforation area",
            id="perforated-no-rows",
        ),
        pytest.param(
            lambda: sdk.SlotPatternPanelGeometry(
                (0.14, 0.08),
                0.004,
                slot_size=(0.005, 0.006),
                pitch=0.015,
            ),
            "slot_size\\[0\\] must be greater than or equal",
            id="slot-invalid-size",
        ),
        pytest.param(
            lambda: sdk.SlotPatternPanelGeometry(
                (0.14, 0.08),
                0.004,
                slot_size=(0.020, 0.006),
                pitch=(0.020, 0.012),
            ),
            "rotated slot envelope",
            id="slot-pitch-too-small",
        ),
        pytest.param(
            lambda: sdk.SlotPatternPanelGeometry(
                (0.14, 0.05),
                0.004,
                slot_size=(0.040, 0.032),
                pitch=(0.050, 0.050),
                frame=0.01,
            ),
            "leave no usable slot area",
            id="slot-no-rows",
        ),
    ],
)
def test_new_panel_geometry_helpers_validate_invalid_inputs(builder, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        builder()


@pytest.mark.parametrize(
    ("builder", "message"),
    [
        pytest.param(
            lambda: sdk.ClevisBracketGeometry(
                (0.08, 0.04, 0.06),
                gap_width=0.08,
                bore_diameter=0.012,
                bore_center_z=0.038,
                base_thickness=0.012,
            ),
            "gap_width",
            id="clevis-invalid-gap",
        ),
        pytest.param(
            lambda: sdk.ClevisBracketGeometry(
                (0.08, 0.04, 0.06),
                gap_width=0.032,
                bore_diameter=0.050,
                bore_center_z=0.038,
                base_thickness=0.012,
            ),
            "bore_diameter",
            id="clevis-invalid-bore",
        ),
        pytest.param(
            lambda: sdk.ClevisBracketGeometry(
                (0.08, 0.04, 0.06),
                gap_width=0.032,
                bore_diameter=0.012,
                bore_center_z=0.014,
                base_thickness=0.012,
            ),
            "bore_center_z",
            id="clevis-invalid-bore-center",
        ),
        pytest.param(
            lambda: sdk.PivotForkGeometry(
                (0.08, 0.05, 0.05),
                gap_width=0.08,
                bore_diameter=0.01,
                bore_center_z=0.028,
                bridge_thickness=0.012,
            ),
            "gap_width",
            id="pivot-fork-invalid-gap",
        ),
        pytest.param(
            lambda: sdk.PivotForkGeometry(
                (0.08, 0.05, 0.05),
                gap_width=0.034,
                bore_diameter=0.050,
                bore_center_z=0.028,
                bridge_thickness=0.012,
            ),
            "bore_diameter",
            id="pivot-fork-invalid-bore",
        ),
        pytest.param(
            lambda: sdk.PivotForkGeometry(
                (0.08, 0.05, 0.05),
                gap_width=0.034,
                bore_diameter=0.01,
                bore_center_z=0.004,
                bridge_thickness=0.012,
            ),
            "bore_center_z",
            id="pivot-fork-invalid-bore-center",
        ),
        pytest.param(
            lambda: sdk.TrunnionYokeGeometry(
                (0.12, 0.05, 0.08),
                span_width=0.12,
                trunnion_diameter=0.016,
                trunnion_center_z=0.05,
                base_thickness=0.014,
            ),
            "span_width",
            id="trunnion-yoke-invalid-span",
        ),
        pytest.param(
            lambda: sdk.TrunnionYokeGeometry(
                (0.12, 0.05, 0.08),
                span_width=0.060,
                trunnion_diameter=0.050,
                trunnion_center_z=0.05,
                base_thickness=0.014,
            ),
            "trunnion_diameter",
            id="trunnion-yoke-invalid-diameter",
        ),
        pytest.param(
            lambda: sdk.TrunnionYokeGeometry(
                (0.12, 0.05, 0.08),
                span_width=0.060,
                trunnion_diameter=0.016,
                trunnion_center_z=0.018,
                base_thickness=0.014,
            ),
            "trunnion_center_z",
            id="trunnion-yoke-invalid-center",
        ),
    ],
)
def test_new_bracket_geometry_helpers_validate_invalid_inputs(builder, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        builder()


@pytest.mark.parametrize(
    ("builder", "message"),
    [
        pytest.param(
            lambda: sdk.FanRotorGeometry(0.07, 0.02, 1, thickness=0.01),
            "blade_count",
            id="fan-invalid-blade-count",
        ),
        pytest.param(
            lambda: sdk.FanRotorGeometry(0.07, 0.07, 5, thickness=0.01),
            "hub_radius",
            id="fan-invalid-hub-radius",
        ),
        pytest.param(
            lambda: sdk.FanRotorGeometry(0.07, 0.02, 5, thickness=0.01, blade_root_chord=0.20),
            "blade chords",
            id="fan-invalid-chord",
        ),
        pytest.param(
            lambda: sdk.BlowerWheelGeometry(0.08, 0.04, 0.05, 1, blade_thickness=0.004),
            "blade_count",
            id="blower-invalid-blade-count",
        ),
        pytest.param(
            lambda: sdk.BlowerWheelGeometry(0.08, 0.08, 0.05, 18, blade_thickness=0.004),
            "inner_radius",
            id="blower-invalid-inner-radius",
        ),
        pytest.param(
            lambda: sdk.BlowerWheelGeometry(0.08, 0.04, 0.05, 18, blade_thickness=0.050),
            "blade_thickness",
            id="blower-invalid-blade-thickness",
        ),
    ],
)
def test_new_fan_geometry_helpers_validate_invalid_inputs(builder, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        builder()


@pytest.mark.parametrize(
    "name",
    [
        "PerforatedPanelGeometry",
        "SlotPatternPanelGeometry",
        "ClevisBracketGeometry",
        "PivotForkGeometry",
        "TrunnionYokeGeometry",
        "FanRotorGeometry",
        "BlowerWheelGeometry",
    ],
)
def test_sdk_and_v0_expose_new_mesh_geometry_helpers(name: str) -> None:
    assert name in sdk.__all__
    assert name in sdk_v0.__all__
    assert getattr(sdk, name) is not None
    assert getattr(sdk_v0, name) is not None


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
