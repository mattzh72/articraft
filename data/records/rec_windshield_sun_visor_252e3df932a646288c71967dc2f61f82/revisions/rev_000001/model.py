from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_windshield_sun_visor")

    shell_dark = model.material("shell_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.09, 0.09, 0.10, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.38, 0.40, 0.42, 1.0))
    hardware = model.material("hardware", rgba=(0.72, 0.74, 0.76, 1.0))
    index_paint = model.material("index_paint", rgba=(0.84, 0.85, 0.82, 1.0))

    roof_mount = model.part("roof_mount")

    mount_outer = rounded_rect_profile(0.168, 0.084, 0.014)
    slot_profile = rounded_rect_profile(0.030, 0.010, 0.005)
    mount_plate = ExtrudeWithHolesGeometry(
        mount_outer,
        [
            _offset_profile(slot_profile, dx=-0.040),
            _offset_profile(slot_profile, dx=0.040),
        ],
        height=0.004,
        center=True,
    )
    roof_mount.visual(
        mesh_from_geometry(mount_plate, "visor_roof_mount_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=trim_grey,
        name="mount_plate",
    )
    roof_mount.visual(
        Box((0.074, 0.050, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=plastic_black,
        name="pedestal",
    )
    roof_mount.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=hardware,
        name="bearing_face",
    )
    roof_mount.visual(
        Box((0.066, 0.032, 0.004)),
        origin=Origin(xyz=(0.040, 0.018, 0.002)),
        material=trim_grey,
        name="roof_datum_plane",
    )
    roof_mount.visual(
        Box((0.028, 0.010, 0.004)),
        origin=Origin(xyz=(0.020, -0.020, 0.002)),
        material=trim_grey,
        name="index_land",
    )
    for mark_name, mark_x, mark_len in (
        ("index_mark_center", 0.020, 0.010),
        ("index_mark_minus", 0.014, 0.007),
        ("index_mark_plus", 0.026, 0.007),
    ):
        roof_mount.visual(
            Box((0.002, mark_len, 0.001)),
            origin=Origin(xyz=(mark_x, -0.020, 0.0045)),
            material=index_paint,
            name=mark_name,
        )
    roof_mount.inertial = Inertial.from_geometry(
        Box((0.168, 0.084, 0.036)),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    swing_carrier = model.part("swing_carrier")
    swing_carrier.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=hardware,
        name="swing_washer",
    )
    swing_carrier.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=hardware,
        name="swing_spindle",
    )
    swing_carrier.visual(
        Box((0.038, 0.020, 0.020)),
        origin=Origin(xyz=(0.019, -0.010, -0.010)),
        material=plastic_black,
        name="carrier_block",
    )
    swing_carrier.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(
            xyz=(0.018, -0.002, -0.006),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="pitch_head",
    )
    swing_carrier.visual(
        Box((0.014, 0.004, 0.004)),
        origin=Origin(xyz=(0.030, -0.022, -0.004)),
        material=index_paint,
        name="secondary_pointer",
    )
    swing_carrier.inertial = Inertial.from_geometry(
        Box((0.038, 0.024, 0.020)),
        mass=0.22,
        origin=Origin(xyz=(0.019, -0.010, -0.010)),
    )

    visor_assembly = model.part("visor_assembly")
    visor_assembly.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(
            xyz=(0.0, 0.012, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="primary_barrel",
    )
    visor_assembly.visual(
        Box((0.042, 0.034, 0.016)),
        origin=Origin(xyz=(0.020, 0.023, -0.008)),
        material=plastic_black,
        name="hinge_arm",
    )

    panel_geometry = ExtrudeGeometry.centered(
        rounded_rect_profile(0.172, 0.336, 0.022),
        0.018,
    )
    visor_assembly.visual(
        mesh_from_geometry(panel_geometry, "visor_panel_shell"),
        origin=Origin(xyz=(0.086, 0.172, -0.009)),
        material=shell_dark,
        name="panel_shell",
    )
    visor_assembly.visual(
        Box((0.052, 0.018, 0.003)),
        origin=Origin(xyz=(0.040, 0.026, -0.0015)),
        material=trim_grey,
        name="panel_top_datum",
    )
    visor_assembly.visual(
        Box((0.016, 0.140, 0.006)),
        origin=Origin(xyz=(0.165, 0.170, -0.009)),
        material=trim_grey,
        name="edge_stiffener",
    )
    for mark_name, mark_x, mark_len in (
        ("pitch_mark_zero", 0.010, 0.010),
        ("pitch_mark_mid", 0.018, 0.007),
        ("pitch_mark_low", 0.026, 0.007),
    ):
        visor_assembly.visual(
            Box((0.002, mark_len, 0.0015)),
            origin=Origin(xyz=(mark_x, 0.030, -0.00075)),
            material=index_paint,
            name=mark_name,
        )
    visor_assembly.inertial = Inertial.from_geometry(
        Box((0.172, 0.340, 0.022)),
        mass=0.95,
        origin=Origin(xyz=(0.086, 0.170, -0.009)),
    )

    model.articulation(
        "roof_to_swing",
        ArticulationType.REVOLUTE,
        parent=roof_mount,
        child=swing_carrier,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(76.0),
        ),
    )
    model.articulation(
        "swing_to_visor",
        ArticulationType.REVOLUTE,
        parent=swing_carrier,
        child=visor_assembly,
        origin=Origin(xyz=(0.018, 0.0, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_mount = object_model.get_part("roof_mount")
    swing_carrier = object_model.get_part("swing_carrier")
    visor_assembly = object_model.get_part("visor_assembly")
    swing = object_model.get_articulation("roof_to_swing")
    pitch = object_model.get_articulation("swing_to_visor")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "secondary_axis_is_vertical_for_positive_side_swing",
        tuple(swing.axis) == (0.0, 0.0, -1.0),
        f"Expected roof_to_swing axis (0, 0, -1), got {swing.axis}.",
    )
    ctx.check(
        "primary_axis_is_drop_pitch_for_positive_deploy",
        tuple(pitch.axis) == (0.0, 1.0, 0.0),
        f"Expected swing_to_visor axis (0, 1, 0), got {pitch.axis}.",
    )

    ctx.expect_contact(
        swing_carrier,
        roof_mount,
        elem_a="swing_washer",
        elem_b="bearing_face",
        name="secondary_pivot_stack_is_seated",
    )
    ctx.expect_contact(
        visor_assembly,
        swing_carrier,
        elem_a="primary_barrel",
        elem_b="pitch_head",
        name="primary_pivot_stack_is_seated",
    )

    with ctx.pose({swing: 0.0, pitch: 0.0}):
        ctx.expect_gap(
            roof_mount,
            visor_assembly,
            axis="z",
            positive_elem="roof_datum_plane",
            negative_elem="panel_top_datum",
            min_gap=0.004,
            max_gap=0.0085,
            name="stowed_roof_gap_is_controlled",
        )
        ctx.expect_overlap(
            roof_mount,
            visor_assembly,
            axes="xy",
            elem_a="roof_datum_plane",
            elem_b="panel_top_datum",
            min_overlap=0.012,
            name="stowed_datum_pads_remain_aligned",
        )
        stowed_panel = ctx.part_element_world_aabb(visor_assembly, elem="panel_shell")

    with ctx.pose({swing: 0.0, pitch: math.radians(96.0)}):
        dropped_panel = ctx.part_element_world_aabb(visor_assembly, elem="panel_shell")
        mount_aabb = ctx.part_world_aabb(roof_mount)

    if stowed_panel is None or dropped_panel is None or mount_aabb is None:
        ctx.fail("pitch_pose_measurements_available", "Could not measure visor panel or roof mount AABBs.")
    else:
        stowed_center = _aabb_center(stowed_panel)
        dropped_center = _aabb_center(dropped_panel)
        ctx.check(
            "primary_pitch_lowers_panel",
            dropped_center[2] < stowed_center[2] - 0.060,
            (
                f"Expected panel center to drop by at least 0.060 m, "
                f"got {stowed_center[2] - dropped_center[2]:.4f} m."
            ),
        )
        ctx.check(
            "deployed_panel_hangs_below_mount",
            dropped_panel[0][2] < mount_aabb[0][2] - 0.120,
            (
                f"Expected deployed panel bottom below mount by > 0.120 m, "
                f"got {mount_aabb[0][2] - dropped_panel[0][2]:.4f} m."
            ),
        )

    with ctx.pose({swing: 0.0, pitch: math.radians(96.0)}):
        front_panel = ctx.part_element_world_aabb(visor_assembly, elem="panel_shell")
    with ctx.pose({swing: math.radians(72.0), pitch: math.radians(96.0)}):
        side_panel = ctx.part_element_world_aabb(visor_assembly, elem="panel_shell")

    if front_panel is None or side_panel is None:
        ctx.fail("swing_pose_measurements_available", "Could not measure side-swing visor pose.")
    else:
        front_center = _aabb_center(front_panel)
        side_center = _aabb_center(side_panel)
        ctx.check(
            "secondary_pivot_swings_panel_sideways",
            side_center[0] > front_center[0] + 0.070 and side_center[1] < front_center[1] - 0.050,
            (
                "Expected the secondary pivot to rotate the deployed panel toward the side window; "
                f"front center={front_center}, side center={side_center}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
