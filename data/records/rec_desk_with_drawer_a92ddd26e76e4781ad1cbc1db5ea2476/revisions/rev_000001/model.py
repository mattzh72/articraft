from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="height_adjustable_standing_desk")

    wood = model.material("wood_oak", rgba=(0.69, 0.56, 0.39, 1.0))
    black_steel = model.material("black_steel", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.18, 0.19, 0.20, 1.0))

    worktop_size = (1.60, 0.75, 0.035)
    leg_x_offset = 0.55
    upper_stage_size = (0.078, 0.050, 0.65)
    lower_stage_outer = (0.090, 0.062, 0.50)
    lower_wall = 0.006
    leg_rest_drop = -0.20
    leg_travel = 0.35
    foot_size = (0.090, 0.700, 0.030)

    drawer_body_width = 0.450
    drawer_body_depth = 0.300
    drawer_side_thickness = 0.010
    drawer_height = 0.046
    drawer_front_width = 0.458
    drawer_front_thickness = 0.018
    drawer_front_height = 0.058
    drawer_closed_y = 0.115
    drawer_top_z = -0.006
    drawer_travel = 0.22

    foot_profile = rounded_rect_profile(foot_size[0], foot_size[1], 0.035)
    foot_geom = ExtrudeGeometry.from_z0(foot_profile, foot_size[2]).translate(0.0, 0.0, -foot_size[2])
    foot_mesh = mesh_from_geometry(foot_geom, "desk_foot_base")

    worktop = model.part("worktop")
    worktop.visual(
        Box(worktop_size),
        origin=Origin(xyz=(0.0, 0.0, worktop_size[2] / 2.0)),
        material=wood,
        name="top_panel",
    )
    worktop.inertial = Inertial.from_geometry(
        Box(worktop_size),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, worktop_size[2] / 2.0)),
    )

    apron_frame = model.part("apron_frame")
    apron_frame.visual(
        Box((0.230, 0.050, 0.040)),
        origin=Origin(xyz=(-0.385, 0.250, -0.020)),
        material=black_steel,
        name="front_left_beam",
    )
    apron_frame.visual(
        Box((0.230, 0.050, 0.040)),
        origin=Origin(xyz=(0.385, 0.250, -0.020)),
        material=black_steel,
        name="front_right_beam",
    )
    apron_frame.visual(
        Box((1.000, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, -0.180, -0.020)),
        material=black_steel,
        name="rear_beam",
    )
    apron_frame.visual(
        Box((0.040, 0.430, 0.040)),
        origin=Origin(xyz=(-0.480, 0.010, -0.020)),
        material=black_steel,
        name="left_side_beam",
    )
    apron_frame.visual(
        Box((0.040, 0.430, 0.040)),
        origin=Origin(xyz=(0.480, 0.010, -0.020)),
        material=black_steel,
        name="right_side_beam",
    )
    apron_frame.inertial = Inertial.from_geometry(
        Box((1.000, 0.480, 0.040)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.020, -0.020)),
    )

    model.articulation(
        "worktop_to_apron",
        ArticulationType.FIXED,
        parent=worktop,
        child=apron_frame,
        origin=Origin(),
    )

    def build_leg(prefix: str, x_offset: float) -> None:
        upper = model.part(f"{prefix}_upper_stage")
        upper.visual(
            Box(upper_stage_size),
            origin=Origin(xyz=(0.0, 0.0, -upper_stage_size[2] / 2.0)),
            material=dark_graphite,
            name="upper_tube",
        )
        upper.inertial = Inertial.from_geometry(
            Box(upper_stage_size),
            mass=4.0,
            origin=Origin(xyz=(0.0, 0.0, -upper_stage_size[2] / 2.0)),
        )

        model.articulation(
            f"apron_to_{prefix}_upper",
            ArticulationType.FIXED,
            parent=apron_frame,
            child=upper,
            origin=Origin(xyz=(x_offset, 0.0, 0.0)),
        )

        lower = model.part(f"{prefix}_lower_stage")
        lower.visual(
            Box((lower_stage_outer[0], lower_wall, lower_stage_outer[2])),
            origin=Origin(xyz=(0.0, (lower_stage_outer[1] - lower_wall) / 2.0, -lower_stage_outer[2] / 2.0)),
            material=black_steel,
            name="front_wall",
        )
        lower.visual(
            Box((lower_stage_outer[0], lower_wall, lower_stage_outer[2])),
            origin=Origin(xyz=(0.0, -(lower_stage_outer[1] - lower_wall) / 2.0, -lower_stage_outer[2] / 2.0)),
            material=black_steel,
            name="rear_wall",
        )
        lower.visual(
            Box((lower_wall, lower_stage_outer[1] - 2.0 * lower_wall, lower_stage_outer[2])),
            origin=Origin(xyz=(-(lower_stage_outer[0] - lower_wall) / 2.0, 0.0, -lower_stage_outer[2] / 2.0)),
            material=black_steel,
            name="left_wall",
        )
        lower.visual(
            Box((lower_wall, lower_stage_outer[1] - 2.0 * lower_wall, lower_stage_outer[2])),
            origin=Origin(xyz=((lower_stage_outer[0] - lower_wall) / 2.0, 0.0, -lower_stage_outer[2] / 2.0)),
            material=black_steel,
            name="right_wall",
        )
        lower.inertial = Inertial.from_geometry(
            Box(lower_stage_outer),
            mass=5.0,
            origin=Origin(xyz=(0.0, 0.0, -lower_stage_outer[2] / 2.0)),
        )

        model.articulation(
            f"{prefix}_lift",
            ArticulationType.PRISMATIC,
            parent=upper,
            child=lower,
            origin=Origin(xyz=(0.0, 0.0, leg_rest_drop)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1200.0,
                velocity=0.08,
                lower=0.0,
                upper=leg_travel,
            ),
        )

        foot = model.part(f"{prefix}_foot")
        foot.visual(
            foot_mesh,
            material=black_steel,
            name="foot_plate",
        )
        foot.inertial = Inertial.from_geometry(
            Box(foot_size),
            mass=5.5,
            origin=Origin(xyz=(0.0, 0.0, -foot_size[2] / 2.0)),
        )

        model.articulation(
            f"{prefix}_lower_to_foot",
            ArticulationType.FIXED,
            parent=lower,
            child=foot,
            origin=Origin(xyz=(0.0, 0.0, -lower_stage_outer[2])),
        )

    build_leg("left", -leg_x_offset)
    build_leg("right", leg_x_offset)

    for side, x_offset in (("left", -0.231), ("right", 0.231)):
        rail = model.part(f"{side}_drawer_rail")
        rail.visual(
            Box((0.012, 0.300, 0.046)),
            origin=Origin(xyz=(0.0, 0.0, -0.023)),
            material=dark_graphite,
            name="rail_body",
        )
        rail.inertial = Inertial.from_geometry(
            Box((0.012, 0.300, 0.046)),
            mass=0.45,
            origin=Origin(xyz=(0.0, 0.0, -0.023)),
        )
        model.articulation(
            f"apron_to_{side}_rail",
            ArticulationType.FIXED,
            parent=apron_frame,
            child=rail,
            origin=Origin(xyz=(x_offset, 0.110, 0.0)),
        )

    drawer = model.part("pencil_drawer")
    drawer.visual(
        Box((drawer_body_width - 2.0 * drawer_side_thickness, drawer_body_depth, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=dark_graphite,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((drawer_side_thickness, drawer_body_depth, drawer_height)),
        origin=Origin(xyz=(-(drawer_body_width - drawer_side_thickness) / 2.0, 0.0, -drawer_height / 2.0)),
        material=dark_graphite,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((drawer_side_thickness, drawer_body_depth, drawer_height)),
        origin=Origin(xyz=((drawer_body_width - drawer_side_thickness) / 2.0, 0.0, -drawer_height / 2.0)),
        material=dark_graphite,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((drawer_body_width - 2.0 * drawer_side_thickness, drawer_side_thickness, drawer_height)),
        origin=Origin(xyz=(0.0, -drawer_body_depth / 2.0, -drawer_height / 2.0)),
        material=dark_graphite,
        name="drawer_back",
    )
    drawer.visual(
        Box((drawer_front_width, drawer_front_thickness, drawer_front_height)),
        origin=Origin(
            xyz=(0.0, drawer_body_depth / 2.0 + drawer_front_thickness / 2.0, -drawer_front_height / 2.0),
        ),
        material=wood,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.160, 0.012, 0.012)),
        origin=Origin(
            xyz=(0.0, drawer_body_depth / 2.0 + drawer_front_thickness + 0.006, -0.016),
        ),
        material=black_steel,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_front_width, drawer_body_depth + drawer_front_thickness, drawer_front_height)),
        mass=3.2,
        origin=Origin(xyz=(0.0, drawer_front_thickness / 2.0, -drawer_front_height / 2.0)),
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=apron_frame,
        child=drawer,
        origin=Origin(xyz=(0.0, drawer_closed_y, drawer_top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=drawer_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    worktop = object_model.get_part("worktop")
    apron_frame = object_model.get_part("apron_frame")
    left_upper = object_model.get_part("left_upper_stage")
    right_upper = object_model.get_part("right_upper_stage")
    left_lower = object_model.get_part("left_lower_stage")
    right_lower = object_model.get_part("right_lower_stage")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")
    left_rail = object_model.get_part("left_drawer_rail")
    right_rail = object_model.get_part("right_drawer_rail")
    drawer = object_model.get_part("pencil_drawer")

    left_lift = object_model.get_articulation("left_lift")
    right_lift = object_model.get_articulation("right_lift")
    drawer_slide = object_model.get_articulation("drawer_slide")

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
        "leg_axes_are_vertical_prismatic",
        left_lift.axis == (0.0, 0.0, -1.0)
        and right_lift.axis == (0.0, 0.0, -1.0)
        and left_lift.motion_limits is not None
        and right_lift.motion_limits is not None
        and left_lift.motion_limits.upper == 0.35
        and right_lift.motion_limits.upper == 0.35,
        details=f"left={left_lift.axis} right={right_lift.axis}",
    )
    ctx.check(
        "drawer_axis_is_front_back",
        drawer_slide.axis == (0.0, 1.0, 0.0)
        and drawer_slide.motion_limits is not None
        and drawer_slide.motion_limits.upper == 0.22,
        details=f"drawer axis={drawer_slide.axis}",
    )

    ctx.expect_contact(worktop, apron_frame, contact_tol=1e-4)
    ctx.expect_contact(worktop, left_upper, contact_tol=1e-4)
    ctx.expect_contact(worktop, right_upper, contact_tol=1e-4)
    ctx.expect_contact(left_upper, left_lower, contact_tol=1e-4)
    ctx.expect_contact(right_upper, right_lower, contact_tol=1e-4)
    ctx.expect_contact(left_lower, left_foot, contact_tol=1e-4)
    ctx.expect_contact(right_lower, right_foot, contact_tol=1e-4)
    ctx.expect_contact(worktop, left_rail, contact_tol=1e-4)
    ctx.expect_contact(worktop, right_rail, contact_tol=1e-4)
    ctx.expect_contact(drawer, left_rail, contact_tol=1e-4)
    ctx.expect_contact(drawer, right_rail, contact_tol=1e-4)

    ctx.expect_within(drawer, worktop, axes="x", margin=0.01)
    ctx.expect_gap(worktop, drawer, axis="z", min_gap=0.005, max_gap=0.008)
    ctx.expect_gap(worktop, left_foot, axis="z", min_gap=0.68, max_gap=0.70)
    ctx.expect_gap(worktop, right_foot, axis="z", min_gap=0.68, max_gap=0.70)

    left_foot_rest = ctx.part_world_position(left_foot)
    right_foot_rest = ctx.part_world_position(right_foot)
    drawer_rest = ctx.part_world_position(drawer)
    assert left_foot_rest is not None
    assert right_foot_rest is not None
    assert drawer_rest is not None

    with ctx.pose({left_lift: 0.30, right_lift: 0.30}):
        left_foot_extended = ctx.part_world_position(left_foot)
        right_foot_extended = ctx.part_world_position(right_foot)
        assert left_foot_extended is not None
        assert right_foot_extended is not None
        ctx.check(
            "desk_lifts_evenly",
            left_foot_extended[2] < left_foot_rest[2] - 0.28
            and right_foot_extended[2] < right_foot_rest[2] - 0.28
            and abs(left_foot_extended[2] - right_foot_extended[2]) < 1e-6,
            details=f"left={left_foot_extended} right={right_foot_extended}",
        )
        ctx.expect_contact(left_upper, left_lower, contact_tol=1e-4)
        ctx.expect_contact(right_upper, right_lower, contact_tol=1e-4)

    with ctx.pose({drawer_slide: 0.22}):
        drawer_open = ctx.part_world_position(drawer)
        assert drawer_open is not None
        ctx.check(
            "drawer_opens_forward",
            drawer_open[1] > drawer_rest[1] + 0.18,
            details=f"rest={drawer_rest} open={drawer_open}",
        )
        ctx.expect_contact(drawer, left_rail, contact_tol=1e-4)
        ctx.expect_contact(drawer, right_rail, contact_tol=1e-4)
        ctx.expect_within(drawer, worktop, axes="x", margin=0.01)
        ctx.expect_gap(worktop, drawer, axis="z", min_gap=0.005, max_gap=0.008)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
