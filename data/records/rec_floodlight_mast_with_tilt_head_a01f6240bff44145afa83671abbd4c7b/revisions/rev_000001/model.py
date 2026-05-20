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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(width: float, height: float, radius: float, x: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="theatrical_fresnel_flood")

    base_black = model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    stand_black = model.material("stand_black", rgba=(0.14, 0.14, 0.15, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    housing_black = model.material("housing_black", rgba=(0.16, 0.16, 0.17, 1.0))
    knob_black = model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.82, 0.90, 0.96, 0.55))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.42, 0.42, 0.030, corner_segments=10), 0.024),
        "square_floor_base",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=base_black,
        name="base_plate",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.155, -0.155),
            (-0.155, 0.155),
            (0.155, -0.155),
            (0.155, 0.155),
        )
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.003)),
            material=rubber,
            name=f"foot_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, 0.030)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    outer_column = model.part("outer_column")
    outer_column.visual(
        Cylinder(radius=0.045, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_steel,
        name="base_receiver",
    )
    outer_column.visual(
        Box((0.086, 0.086, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=dark_steel,
        name="lower_socket_housing",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.035, -0.035),
            (-0.035, 0.035),
            (0.035, -0.035),
            (0.035, 0.035),
        )
    ):
        outer_column.visual(
            Box((0.018, 0.018, 0.690)),
            origin=Origin(xyz=(x_pos, y_pos, 0.465)),
            material=stand_black,
            name=f"corner_rail_{index}",
        )
    outer_column.visual(
        Box((0.088, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.035, 0.785)),
        material=dark_steel,
        name="rear_clamp_bar",
    )
    outer_column.visual(
        Box((0.088, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.035, 0.785)),
        material=dark_steel,
        name="front_clamp_bar",
    )
    outer_column.visual(
        Box((0.014, 0.088, 0.020)),
        origin=Origin(xyz=(-0.035, 0.0, 0.785)),
        material=dark_steel,
        name="left_clamp_bar",
    )
    outer_column.visual(
        Box((0.014, 0.088, 0.020)),
        origin=Origin(xyz=(0.035, 0.0, 0.785)),
        material=dark_steel,
        name="right_clamp_bar",
    )
    outer_column.visual(
        Box((0.010, 0.040, 0.040)),
        origin=Origin(xyz=(-0.028, 0.0, 0.785)),
        material=dark_steel,
        name="left_guide_jaw",
    )
    outer_column.visual(
        Box((0.010, 0.040, 0.040)),
        origin=Origin(xyz=(0.028, 0.0, 0.785)),
        material=dark_steel,
        name="right_guide_jaw",
    )
    outer_column.visual(
        Box((0.040, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.028, 0.785)),
        material=dark_steel,
        name="rear_guide_jaw",
    )
    outer_column.visual(
        Box((0.040, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.028, 0.785)),
        material=dark_steel,
        name="front_guide_jaw",
    )
    outer_column.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.060, 0.0, 0.785), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="height_lock_knob",
    )
    outer_column.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.042, 0.0, 0.785), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="lock_spindle",
    )
    outer_column.inertial = Inertial.from_geometry(
        Box((0.110, 0.110, 0.804)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.402)),
    )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Cylinder(radius=0.023, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=aluminum,
        name="telescoping_riser",
    )
    inner_column.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=dark_steel,
        name="top_boss",
    )
    inner_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.029, length=0.720),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )

    spigot = model.part("spigot")
    spigot.visual(
        Cylinder(radius=0.029, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="spigot_shoulder",
    )
    spigot.visual(
        Cylinder(radius=0.014, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=aluminum,
        name="spigot_pin",
    )
    spigot.visual(
        Box((0.026, 0.036, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=dark_steel,
        name="tilt_block",
    )
    spigot.visual(
        Box((0.016, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, -0.017, 0.126)),
        material=dark_steel,
        name="left_ear",
    )
    spigot.visual(
        Box((0.016, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.017, 0.126)),
        material=dark_steel,
        name="right_ear",
    )
    spigot.inertial = Inertial.from_geometry(
        Box((0.060, 0.040, 0.150)),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    head_yoke = model.part("head_yoke")
    head_yoke.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="tilt_trunnion",
    )
    head_yoke.visual(
        Box((0.042, 0.024, 0.055)),
        origin=Origin(xyz=(0.026, 0.0, 0.022)),
        material=dark_steel,
        name="yoke_stem",
    )
    head_yoke.visual(
        Box((0.128, 0.250, 0.018)),
        origin=Origin(xyz=(0.089, 0.0, 0.050)),
        material=dark_steel,
        name="yoke_bridge",
    )
    head_yoke.visual(
        Box((0.018, 0.022, 0.230)),
        origin=Origin(xyz=(0.145, -0.117, 0.160)),
        material=dark_steel,
        name="left_yoke_arm",
    )
    head_yoke.visual(
        Box((0.018, 0.022, 0.230)),
        origin=Origin(xyz=(0.145, 0.117, 0.160)),
        material=dark_steel,
        name="right_yoke_arm",
    )
    head_yoke.visual(
        Cylinder(radius=0.016, length=0.028),
        origin=Origin(xyz=(0.145, -0.139, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="left_tilt_knob",
    )
    head_yoke.visual(
        Cylinder(radius=0.016, length=0.028),
        origin=Origin(xyz=(0.145, 0.139, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="right_tilt_knob",
    )

    housing_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.188, 0.178, 0.018, -0.095),
                _yz_section(0.236, 0.226, 0.028, 0.015),
                _yz_section(0.218, 0.208, 0.024, 0.110),
                _yz_section(0.194, 0.182, 0.018, 0.125),
            ]
        ),
        "fresnel_head_housing",
    )
    bezel_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.102, -0.016), (0.102, 0.016)],
            [(0.088, -0.016), (0.088, 0.016)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        "fresnel_bezel_ring",
    )
    head_yoke.visual(
        housing_shell_mesh,
        origin=Origin(xyz=(0.175, 0.0, 0.160)),
        material=housing_black,
        name="housing_shell",
    )
    head_yoke.visual(
        bezel_ring_mesh,
        origin=Origin(xyz=(0.078, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    head_yoke.visual(
        Cylinder(radius=0.088, length=0.008),
        origin=Origin(xyz=(0.077, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="fresnel_lens",
    )
    head_yoke.visual(
        Box((0.024, 0.180, 0.186)),
        origin=Origin(xyz=(0.296, 0.0, 0.160)),
        material=housing_black,
        name="rear_door",
    )
    head_yoke.visual(
        Box((0.090, 0.120, 0.018)),
        origin=Origin(xyz=(0.210, 0.0, 0.280)),
        material=dark_steel,
        name="top_vent_cap",
    )
    head_yoke.visual(
        Box((0.110, 0.014, 0.018)),
        origin=Origin(xyz=(0.170, 0.0, 0.276)),
        material=dark_steel,
        name="carry_handle",
    )
    head_yoke.inertial = Inertial.from_geometry(
        Box((0.310, 0.300, 0.310)),
        mass=6.5,
        origin=Origin(xyz=(0.155, 0.0, 0.155)),
    )

    model.articulation(
        "base_to_outer_column",
        ArticulationType.FIXED,
        parent=base,
        child=outer_column,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )
    model.articulation(
        "outer_to_inner_column",
        ArticulationType.PRISMATIC,
        parent=outer_column,
        child=inner_column,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.15,
            lower=0.0,
            upper=0.450,
        ),
    )
    model.articulation(
        "inner_to_spigot",
        ArticulationType.FIXED,
        parent=inner_column,
        child=spigot,
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
    )
    model.articulation(
        "spigot_to_head",
        ArticulationType.REVOLUTE,
        parent=spigot,
        child=head_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=-1.05,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    outer_column = object_model.get_part("outer_column")
    inner_column = object_model.get_part("inner_column")
    spigot = object_model.get_part("spigot")
    head_yoke = object_model.get_part("head_yoke")

    outer_to_inner = object_model.get_articulation("outer_to_inner_column")
    spigot_to_head = object_model.get_articulation("spigot_to_head")

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

    ctx.check("base_present", base.name == "base", "Base part missing.")
    ctx.check("outer_column_present", outer_column.name == "outer_column", "Outer column missing.")
    ctx.check("inner_column_present", inner_column.name == "inner_column", "Inner column missing.")
    ctx.check("spigot_present", spigot.name == "spigot", "Spigot part missing.")
    ctx.check("head_yoke_present", head_yoke.name == "head_yoke", "Head/yoke assembly missing.")

    ctx.check(
        "column_slide_axis_is_vertical",
        outer_to_inner.axis == (0.0, 0.0, 1.0),
        f"Expected vertical prismatic axis, got {outer_to_inner.axis}.",
    )
    ctx.check(
        "head_tilt_axis_is_horizontal",
        spigot_to_head.axis == (0.0, 1.0, 0.0),
        f"Expected horizontal y-axis tilt, got {spigot_to_head.axis}.",
    )

    ctx.expect_contact(base, outer_column, contact_tol=1e-4, name="base_supports_outer_column")
    ctx.expect_contact(outer_column, inner_column, contact_tol=1e-4, name="inner_column_guided_by_outer")
    ctx.expect_contact(inner_column, spigot, contact_tol=1e-4, name="spigot_seated_on_inner_column")
    ctx.expect_contact(spigot, head_yoke, contact_tol=1e-4, name="head_knuckle_seated_in_spigot_ears")

    ctx.expect_origin_distance(
        outer_column,
        inner_column,
        axes="xy",
        max_dist=0.001,
        name="telescoping_columns_share_centerline",
    )
    ctx.expect_within(
        inner_column,
        outer_column,
        axes="xy",
        margin=0.006,
        name="inner_column_stays_within_outer_column_plan",
    )
    ctx.expect_gap(
        head_yoke,
        base,
        axis="z",
        min_gap=0.70,
        name="fixture_head_stays_well_above_base",
    )

    spigot_rest = ctx.part_world_position(spigot)
    assert spigot_rest is not None
    with ctx.pose({outer_to_inner: 0.350}):
        spigot_extended = ctx.part_world_position(spigot)
        assert spigot_extended is not None
        ctx.check(
            "column_extension_moves_upward",
            abs(spigot_extended[0] - spigot_rest[0]) < 1e-6
            and abs(spigot_extended[1] - spigot_rest[1]) < 1e-6
            and spigot_extended[2] > spigot_rest[2] + 0.34,
            f"Expected pure upward extension, got rest={spigot_rest}, extended={spigot_extended}.",
        )
        ctx.expect_origin_distance(outer_column, inner_column, axes="xy", max_dist=0.001)

    housing_rest = ctx.part_element_world_aabb(head_yoke, elem="housing_shell")
    assert housing_rest is not None
    housing_rest_center = tuple((housing_rest[0][i] + housing_rest[1][i]) * 0.5 for i in range(3))
    with ctx.pose({spigot_to_head: 0.65}):
        housing_tilted = ctx.part_element_world_aabb(head_yoke, elem="housing_shell")
        assert housing_tilted is not None
        housing_tilted_center = tuple((housing_tilted[0][i] + housing_tilted[1][i]) * 0.5 for i in range(3))
        ctx.check(
            "head_tilt_changes_xz_but_not_y",
            abs(housing_tilted_center[1] - housing_rest_center[1]) < 0.002
            and (
                abs(housing_tilted_center[0] - housing_rest_center[0]) > 0.03
                or abs(housing_tilted_center[2] - housing_rest_center[2]) > 0.03
            ),
            f"Expected y-axis tilt motion, got rest={housing_rest_center}, tilted={housing_tilted_center}.",
        )
        ctx.expect_contact(spigot, head_yoke, contact_tol=1e-4)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
