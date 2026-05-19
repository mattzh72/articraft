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
        (0.08, 0.05),
        0.004,
        hole_diameter=0.006,
        pitch=(0.018, 0.018),
        frame=0.008,
        corner_radius=0.003,
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


def _build_vent_grille_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.VentGrilleGeometry(
        (0.18, 0.10),
        frame=0.012,
        face_thickness=0.004,
        duct_depth=0.026,
        duct_wall=0.003,
        slat_pitch=0.018,
        slat_width=0.009,
        slat_angle_deg=30.0,
        corner_radius=0.006,
        slats=sdk.VentGrilleSlats(
            profile="airfoil",
            direction="down",
            divider_count=2,
            divider_width=0.004,
        ),
        frame_profile=sdk.VentGrilleFrame(style="beveled", depth=0.0012),
        mounts=sdk.VentGrilleMounts(style="holes", inset=0.010, hole_diameter=0.0032),
        sleeve=sdk.VentGrilleSleeve(style="full"),
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
        7,
        thickness=0.010,
        blade_pitch_deg=31.0,
        blade_sweep_deg=24.0,
        blade=sdk.FanRotorBlade(
            shape="scimitar",
            tip_pitch_deg=12.0,
            camber=0.16,
        ),
        hub=sdk.FanRotorHub(
            style="spinner",
            bore_diameter=0.005,
        ),
        shroud=sdk.FanRotorShroud(
            thickness=0.004,
            depth=0.012,
            clearance=0.0015,
            lip_depth=0.002,
        ),
        center=center,
    )


def _build_blower_wheel_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.BlowerWheelGeometry(
        0.080,
        0.040,
        0.050,
        8,
        blade_thickness=0.004,
        blade_sweep_deg=25.0,
        center=center,
    )


def _build_knob_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.KnobGeometry(
        0.042,
        0.024,
        body_style="skirted",
        top_diameter=0.034,
        skirt=sdk.KnobSkirt(0.052, 0.006, flare=0.08),
        grip=sdk.KnobGrip(style="fluted", count=8, depth=0.0014),
        indicator=sdk.KnobIndicator(
            style="line",
            mode="engraved",
            depth=0.0008,
            angle_deg=20.0,
        ),
        bore=sdk.KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
        center=center,
    )


def _build_bezel_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.BezelGeometry(
        (0.080, 0.050),
        (0.110, 0.080),
        0.012,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.006,
        outer_corner_radius=0.010,
        center=center,
    )


def _build_wheel_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.WheelGeometry(
        0.080,
        0.026,
        rim=sdk.WheelRim(
            inner_radius=0.056,
            flange_height=0.006,
            flange_thickness=0.003,
        ),
        hub=sdk.WheelHub(
            radius=0.020,
            width=0.020,
            cap_style="protruding",
            bolt_pattern=sdk.BoltPattern(
                count=3,
                circle_diameter=0.026,
                hole_diameter=0.003,
            ),
        ),
        face=sdk.WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.001),
        spokes=sdk.WheelSpokes(
            style="straight",
            count=3,
            thickness=0.003,
            window_radius=0.008,
        ),
        bore=sdk.WheelBore(style="round", diameter=0.010),
        center=center,
    )


def _build_tire_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.TireGeometry(
        0.100,
        0.034,
        inner_radius=0.074,
        carcass=sdk.TireCarcass(belt_width_ratio=0.66, sidewall_bulge=0.08),
        tread=sdk.TireTread(
            style="circumferential",
            depth=0.004,
            count=2,
            land_ratio=0.58,
        ),
        grooves=(sdk.TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
        sidewall=sdk.TireSidewall(style="rounded", bulge=0.06),
        shoulder=sdk.TireShoulder(width=0.004, radius=0.003),
        center=center,
    )


def _build_barrel_hinge_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.BarrelHingeGeometry(
        0.090,
        leaf_width_a=0.024,
        leaf_width_b=0.020,
        leaf_thickness=0.0024,
        pin_diameter=0.003,
        knuckle_count=5,
        holes_a=sdk.HingeHolePattern(
            style="round",
            count=3,
            diameter=0.0032,
            edge_margin=0.010,
        ),
        holes_b=sdk.HingeHolePattern(
            style="slotted",
            count=2,
            slot_size=(0.007, 0.003),
            edge_margin=0.012,
        ),
        center=center,
    )


def _build_piano_hinge_geometry(*, center: bool = True) -> sdk.MeshGeometry:
    return sdk.PianoHingeGeometry(
        0.180,
        leaf_width_a=0.016,
        leaf_width_b=0.014,
        leaf_thickness=0.0018,
        pin_diameter=0.0025,
        knuckle_pitch=0.012,
        holes_a=sdk.HingeHolePattern(
            style="round",
            count=5,
            diameter=0.0022,
            edge_margin=0.010,
        ),
        holes_b=sdk.HingeHolePattern(
            style="round",
            count=5,
            diameter=0.0022,
            edge_margin=0.010,
        ),
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


def test_vent_grille_geometry_supports_custom_profiles_mounts_and_dividers(tmp_path: Path) -> None:
    exported = _assert_clean_export(tmp_path, _build_vent_grille_geometry(), "vent_grille_custom")

    mins, maxs = exported.bounds
    assert mins[0] == pytest.approx(-0.09, abs=1e-3)
    assert maxs[0] == pytest.approx(0.09, abs=1e-3)
    assert mins[1] == pytest.approx(-0.05, abs=1e-3)
    assert maxs[1] == pytest.approx(0.05, abs=1e-3)
    assert mins[2] <= -0.027
    assert maxs[2] >= 0.001


def test_vent_grille_geometry_requires_at_least_one_slat_row() -> None:
    with pytest.raises(ValueError, match="No slat rows fit panel"):
        sdk.VentGrilleGeometry(
            (0.18, 0.03),
            frame=0.008,
            slat_pitch=0.018,
            slat_width=0.009,
        )


def test_vent_grille_geometry_supports_center_false_z0_mount_frame() -> None:
    geometry = _build_vent_grille_geometry(center=False)
    mins, _maxs = _bounds(geometry)
    assert mins[2] == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize(
    ("builder", "logical_name", "expected_mins", "expected_maxs", "tol"),
    [
        pytest.param(
            _build_vent_grille_geometry,
            "vent_grille",
            (-0.090, -0.050, -0.027),
            (0.090, 0.050, 0.002),
            2.0e-3,
            id="vent-grille",
        ),
        pytest.param(
            _build_perforated_panel_geometry,
            "perforated_panel",
            (-0.040, -0.025, -0.002),
            (0.040, 0.025, 0.002),
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
            (-0.0685, -0.0685, -0.0060),
            (0.0685, 0.0685, 0.0080),
            3.0e-3,
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
        pytest.param(
            _build_knob_geometry,
            "knob",
            (-0.028, -0.028, -0.018),
            (0.028, 0.028, 0.012),
            2.0e-3,
            id="knob",
        ),
        pytest.param(
            _build_bezel_geometry,
            "bezel",
            (-0.055, -0.040, -0.006),
            (0.055, 0.040, 0.006),
            1.5e-3,
            id="bezel",
        ),
        pytest.param(
            _build_wheel_geometry,
            "wheel",
            (-0.0130, -0.080, -0.080),
            (0.0132, 0.080, 0.080),
            2.5e-3,
            id="wheel",
        ),
        pytest.param(
            _build_tire_geometry,
            "tire",
            (-0.017, -0.100, -0.100),
            (0.017, 0.100, 0.100),
            2.5e-3,
            id="tire",
        ),
        pytest.param(
            _build_barrel_hinge_geometry,
            "barrel_hinge",
            (-0.0260, -0.0027, -0.045),
            (0.0220, 0.0027, 0.045),
            2.0e-3,
            id="barrel-hinge",
        ),
        pytest.param(
            _build_piano_hinge_geometry,
            "piano_hinge",
            (-0.0175, -0.0020, -0.090),
            (0.0155, 0.0020, 0.090),
            2.0e-3,
            id="piano-hinge",
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


def test_slot_pattern_panel_exports_real_through_slot(tmp_path: Path) -> None:
    geometry = sdk.SlotPatternPanelGeometry(
        (0.10, 0.05),
        0.004,
        slot_size=(0.030, 0.010),
        pitch=(0.20, 0.20),
        frame=0.010,
        corner_radius=0.0,
    )

    exported = _assert_clean_export(tmp_path, geometry, "single_slot_panel")

    assert exported.euler_number == 0
    assert exported.bounds[0] == pytest.approx((-0.05, -0.025, -0.002), abs=1e-6)
    assert exported.bounds[1] == pytest.approx((0.05, 0.025, 0.002), abs=1e-6)


@pytest.mark.parametrize(
    ("builder", "name"),
    [
        pytest.param(
            _build_perforated_panel_geometry, "PerforatedPanelGeometry", id="perforated-panel"
        ),
        pytest.param(
            _build_slot_pattern_panel_geometry,
            "SlotPatternPanelGeometry",
            id="slot-pattern-panel",
        ),
        pytest.param(_build_knob_geometry, "KnobGeometry", id="knob"),
        pytest.param(_build_bezel_geometry, "BezelGeometry", id="bezel"),
        pytest.param(_build_fan_rotor_geometry, "FanRotorGeometry", id="fan-rotor"),
        pytest.param(_build_barrel_hinge_geometry, "BarrelHingeGeometry", id="barrel-hinge"),
        pytest.param(_build_piano_hinge_geometry, "PianoHingeGeometry", id="piano-hinge"),
        pytest.param(_build_blower_wheel_geometry, "BlowerWheelGeometry", id="blower-wheel"),
    ],
)
def test_new_mesh_geometry_helpers_support_center_false_z0_mount_frames(builder, name: str) -> None:
    geometry = builder(center=False)
    mins, _maxs = _bounds(geometry)
    assert mins[2] == pytest.approx(0.0, abs=1e-6), name


@pytest.mark.parametrize(
    ("builder", "name"),
    [
        pytest.param(_build_wheel_geometry, "WheelGeometry", id="wheel"),
        pytest.param(_build_tire_geometry, "TireGeometry", id="tire"),
    ],
)
def test_new_rotational_geometry_helpers_support_center_false_x0_mount_frames(
    builder, name: str
) -> None:
    geometry = builder(center=False)
    mins, _maxs = _bounds(geometry)
    assert mins[0] == pytest.approx(0.0, abs=1e-6), name


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
        pytest.param(
            lambda: sdk.VentGrilleGeometry(
                (0.18, 0.10),
                mounts=sdk.VentGrilleMounts(style="holes", hole_diameter=0.030),
            ),
            "mounts.hole_diameter",
            id="vent-hole-too-large",
        ),
        pytest.param(
            lambda: sdk.VentGrilleGeometry(
                (0.18, 0.10),
                slats=sdk.VentGrilleSlats(divider_count=8, divider_width=0.020),
            ),
            "leave no usable openings",
            id="vent-dividers-too-dense",
        ),
        pytest.param(
            lambda: sdk.VentGrilleGeometry(
                (0.18, 0.10),
                sleeve=sdk.VentGrilleSleeve(style="short", depth=-0.01),
            ),
            "sleeve.depth",
            id="vent-invalid-sleeve-depth",
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
            lambda: sdk.FanRotorGeometry(
                0.07,
                0.02,
                5,
                thickness=0.01,
                hub=sdk.FanRotorHub(style="domed", bore_diameter=0.04),
            ),
            "hub.bore_diameter",
            id="fan-invalid-bore",
        ),
        pytest.param(
            lambda: sdk.FanRotorGeometry(
                0.07,
                0.02,
                5,
                thickness=0.01,
                blade=sdk.FanRotorBlade(shape="scimitar", tip_clearance=0.003),
                shroud=sdk.FanRotorShroud(thickness=0.004),
            ),
            "tip_clearance cannot be used with shroud",
            id="fan-invalid-tip-clearance-with-shroud",
        ),
        pytest.param(
            lambda: sdk.FanRotorGeometry(
                0.07,
                0.02,
                5,
                thickness=0.01,
                blade=sdk.FanRotorBlade(tip_pitch_deg=86.0),
            ),
            "tip_pitch_deg",
            id="fan-invalid-tip-pitch",
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
    ("builder", "expected"),
    [
        pytest.param(
            lambda: sdk.KnobGeometry(
                0.038,
                0.022,
                body_style="domed",
                top_feature=sdk.KnobTopFeature(
                    style="top_insert",
                    diameter=0.016,
                    height=0.0018,
                ),
                grip=sdk.KnobGrip(
                    style="diamond_knurl",
                    count=8,
                    depth=0.0011,
                    helix_angle_deg=28.0,
                ),
                indicator=sdk.KnobIndicator(
                    style="wedge",
                    mode="raised",
                    angle_deg=35.0,
                ),
                body_reliefs=(
                    sdk.KnobRelief(
                        style="top_recess",
                        width=0.014,
                        depth=0.0018,
                    ),
                ),
                bore=sdk.KnobBore(
                    style="splined",
                    diameter=0.006,
                    spline_count=10,
                    spline_depth=0.0007,
                ),
            ),
            "knob",
            id="knob-detailed-variant",
        ),
        pytest.param(
            lambda: sdk.KnobGeometry(
                0.036,
                0.021,
                body_style="faceted",
                base_diameter=0.039,
                top_diameter=0.028,
                edge_radius=0.0008,
                grip=sdk.KnobGrip(
                    style="ribbed",
                    count=6,
                    depth=0.0010,
                    width=0.0018,
                ),
                indicator=sdk.KnobIndicator(
                    style="wedge",
                    mode="raised",
                    angle_deg=10.0,
                ),
                top_feature=sdk.KnobTopFeature(
                    style="top_insert",
                    diameter=0.012,
                    height=0.0012,
                ),
            ),
            "knob-faceted",
            id="knob-faceted-variant",
        ),
        pytest.param(
            lambda: sdk.KnobGeometry(
                0.040,
                0.024,
                body_style="lobed",
                base_diameter=0.028,
                top_diameter=0.036,
                crown_radius=0.0012,
                body_reliefs=(
                    sdk.KnobRelief(
                        style="top_recess",
                        width=0.014,
                        depth=0.0016,
                    ),
                ),
                bore=sdk.KnobBore(style="round", diameter=0.008),
            ),
            "knob-lobed",
            id="knob-lobed-variant",
        ),
        pytest.param(
            lambda: sdk.BezelGeometry(
                (0.056, 0.056),
                (0.084, 0.084),
                0.010,
                opening_shape="circle",
                outer_shape="circle",
                face=sdk.BezelFace(style="chamfered", chamfer=0.0012),
                flange=sdk.BezelFlange(width=0.004, thickness=0.002, offset=0.001),
                mounts=sdk.BezelMounts(
                    style="rear_flange",
                    hole_count=4,
                    hole_diameter=0.003,
                    setback=0.004,
                ),
                edge_features=(
                    sdk.BezelEdgeFeature(
                        style="notch",
                        edge="bottom",
                        size=0.004,
                        extent=0.014,
                    ),
                    sdk.BezelEdgeFeature(
                        style="groove",
                        edge="right",
                        size=0.0012,
                        extent=0.040,
                    ),
                ),
            ),
            "bezel",
            id="bezel-detailed-variant",
        ),
        pytest.param(
            lambda: sdk.WheelGeometry(
                0.100,
                0.032,
                rim=sdk.WheelRim(
                    inner_radius=0.070,
                    flange_height=0.008,
                    flange_thickness=0.003,
                ),
                hub=sdk.WheelHub(
                    radius=0.024,
                    width=0.024,
                    cap_style="recessed",
                ),
                face=sdk.WheelFace(dish_depth=0.004, front_inset=0.003),
                spokes=sdk.WheelSpokes(
                    style="mesh",
                    count=3,
                    thickness=0.0028,
                    window_radius=0.007,
                ),
                bore=sdk.WheelBore(style="keyed", diameter=0.010, key_width=0.0025),
            ),
            "wheel",
            id="wheel-detailed-variant",
        ),
        pytest.param(
            lambda: sdk.TireGeometry(
                0.130,
                0.044,
                inner_radius=0.096,
                tread=sdk.TireTread(
                    style="block",
                    depth=0.005,
                    count=6,
                    land_ratio=0.52,
                ),
                grooves=(
                    sdk.TireGroove(center_offset=-0.010, width=0.004, depth=0.0025),
                    sdk.TireGroove(center_offset=0.010, width=0.004, depth=0.0025),
                ),
                sidewall=sdk.TireSidewall(style="square", bulge=0.0),
                shoulder=sdk.TireShoulder(width=0.004, radius=0.003),
            ),
            "tire",
            id="tire-detailed-variant",
        ),
        pytest.param(
            lambda: sdk.BarrelHingeGeometry(
                0.080,
                leaf_width_a=0.020,
                leaf_width_b=0.018,
                leaf_thickness=0.0022,
                pin_diameter=0.0028,
                knuckle_outer_diameter=0.0048,
                knuckle_count=5,
                open_angle_deg=120.0,
                holes_a=sdk.HingeHolePattern(
                    style="round",
                    count=3,
                    diameter=0.003,
                    edge_margin=0.008,
                ),
                holes_b=sdk.HingeHolePattern(
                    style="slotted",
                    count=2,
                    slot_size=(0.006, 0.0028),
                    edge_margin=0.010,
                ),
                pin=sdk.HingePinStyle(
                    head_style="button",
                    head_height=0.001,
                    head_diameter=0.0046,
                    exposed_end=0.0008,
                ),
            ),
            "barrel-hinge",
            id="barrel-hinge-detailed-variant",
        ),
    ],
)
def test_new_detail_option_variants_build_nonempty_meshes(builder, expected: str) -> None:
    geometry = builder()
    assert len(geometry.vertices) > 0, expected
    assert len(geometry.faces) > 0, expected
    assert _component_count(geometry) == 1, expected


def test_wheel_and_tire_helpers_can_be_sized_compatibly() -> None:
    wheel = sdk.WheelGeometry(
        0.100,
        0.032,
        rim=sdk.WheelRim(inner_radius=0.072, flange_height=0.008, flange_thickness=0.003),
        hub=sdk.WheelHub(radius=0.023, width=0.022),
        spokes=sdk.WheelSpokes(style="straight", count=6, thickness=0.003),
        bore=sdk.WheelBore(style="round", diameter=0.010),
    )
    tire = sdk.TireGeometry(
        0.126,
        0.038,
        inner_radius=0.101,
        tread=sdk.TireTread(style="circumferential", depth=0.003, count=3),
    )

    wheel_mins, wheel_maxs = _bounds(wheel)
    tire_mins, tire_maxs = _bounds(tire)
    assert abs(tire_mins[0]) >= abs(wheel_mins[0]) - 1e-6
    assert tire_maxs[0] >= wheel_maxs[0] - 1e-6
    assert tire_maxs[1] > wheel_maxs[1]
    assert tire_maxs[2] > wheel_maxs[2]


@pytest.mark.parametrize(
    ("builder", "message"),
    [
        pytest.param(
            lambda: sdk.KnobGeometry(
                0.040,
                0.022,
                grip=sdk.KnobGrip(style="fluted", count=1, depth=0.001),
            ),
            "KnobGrip.count",
            id="knob-invalid-grip-count",
        ),
        pytest.param(
            lambda: sdk.KnobGeometry(
                0.040,
                0.022,
                side_draft_deg=46.0,
            ),
            "side_draft_deg",
            id="knob-invalid-side-draft",
        ),
        pytest.param(
            lambda: sdk.KnobGeometry(
                0.040,
                0.022,
                bore=sdk.KnobBore(style="round", diameter=0.080),
            ),
            "KnobBore diameter",
            id="knob-invalid-bore",
        ),
        pytest.param(
            lambda: sdk.BezelGeometry((0.080, 0.050), (0.070, 0.060), 0.010),
            "opening_size",
            id="bezel-invalid-opening",
        ),
        pytest.param(
            lambda: sdk.BezelGeometry(
                (0.080, 0.050),
                (0.100, 0.070),
                0.010,
                recess=sdk.BezelRecess(depth=0.003, inset=0.030),
            ),
            "recess wall leaves no outer frame material",
            id="bezel-invalid-recess",
        ),
        pytest.param(
            lambda: sdk.WheelGeometry(
                0.100,
                0.030,
                rim=sdk.WheelRim(inner_radius=0.100),
            ),
            "WheelRim.inner_radius",
            id="wheel-invalid-rim-inner-radius",
        ),
        pytest.param(
            lambda: sdk.WheelGeometry(
                0.100,
                0.030,
                hub=sdk.WheelHub(radius=0.096, width=0.020),
            ),
            "WheelHub dimensions",
            id="wheel-invalid-hub-radius",
        ),
        pytest.param(
            lambda: sdk.WheelGeometry(
                0.100,
                0.030,
                hub=sdk.WheelHub(radius=0.020, width=0.020),
                bore=sdk.WheelBore(style="round", diameter=0.050),
            ),
            "WheelBore\\.diameter",
            id="wheel-invalid-bore",
        ),
        pytest.param(
            lambda: sdk.TireGeometry(0.120, 0.040, inner_radius=0.120),
            "inner_radius",
            id="tire-invalid-inner-radius",
        ),
        pytest.param(
            lambda: sdk.TireGeometry(
                0.120,
                0.040,
                grooves=(sdk.TireGroove(center_offset=0.0, width=-0.003, depth=0.002),),
            ),
            "TireGroove",
            id="tire-invalid-groove",
        ),
        pytest.param(
            lambda: sdk.BarrelHingeGeometry(
                0.080,
                leaf_width_a=0.020,
                leaf_thickness=0.002,
                pin_diameter=0.003,
                knuckle_count=2,
            ),
            "knuckle_count",
            id="barrel-hinge-invalid-knuckle-count",
        ),
        pytest.param(
            lambda: sdk.BarrelHingeGeometry(
                0.080,
                leaf_width_a=0.020,
                leaf_thickness=0.002,
                pin_diameter=0.005,
                knuckle_outer_diameter=0.004,
            ),
            "pin_diameter",
            id="barrel-hinge-invalid-pin-diameter",
        ),
        pytest.param(
            lambda: sdk.PianoHingeGeometry(
                0.120,
                leaf_width_a=0.016,
                leaf_thickness=0.0018,
                pin_diameter=0.0025,
                knuckle_pitch=0.0,
            ),
            "knuckle_pitch",
            id="piano-hinge-invalid-pitch",
        ),
    ],
)
def test_new_knob_bezel_wheel_tire_and_hinge_helpers_validate_invalid_inputs(
    builder, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        builder()


@pytest.mark.parametrize(
    "name",
    [
        "KnobGeometry",
        "BezelGeometry",
        "WheelGeometry",
        "TireGeometry",
        "BarrelHingeGeometry",
        "PianoHingeGeometry",
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


@pytest.mark.parametrize(
    "name",
    [
        "KnobSkirt",
        "KnobGrip",
        "KnobIndicator",
        "KnobTopFeature",
        "KnobBore",
        "KnobRelief",
        "BezelFace",
        "BezelRecess",
        "BezelVisor",
        "BezelFlange",
        "BezelMounts",
        "BezelCutout",
        "BezelEdgeFeature",
        "WheelRim",
        "WheelHub",
        "WheelFace",
        "WheelSpokes",
        "WheelBore",
        "WheelFlange",
        "BoltPattern",
        "FanRotorBlade",
        "FanRotorHub",
        "FanRotorShroud",
        "VentGrilleSlats",
        "VentGrilleFrame",
        "VentGrilleMounts",
        "VentGrilleSleeve",
        "TireCarcass",
        "TireTread",
        "TireGroove",
        "TireSidewall",
        "TireShoulder",
        "HingeHolePattern",
        "HingePinStyle",
    ],
)
def test_sdk_and_v0_expose_new_mesh_geometry_option_dataclasses(name: str) -> None:
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
