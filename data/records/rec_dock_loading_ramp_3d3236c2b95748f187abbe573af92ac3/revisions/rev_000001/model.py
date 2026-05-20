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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pit_style_dock_leveler")

    concrete = model.material("concrete", rgba=(0.55, 0.55, 0.57, 1.0))
    steel = model.material("steel", rgba=(0.28, 0.30, 0.34, 1.0))
    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    axle_dark = model.material("axle_dark", rgba=(0.16, 0.16, 0.18, 1.0))

    pit_inner_length = 2.20
    pit_inner_width = 2.00
    side_wall_thickness = 0.10
    front_rear_wall_thickness = 0.12
    pit_depth = 0.60
    floor_thickness = 0.06

    hinge_axis_z = -0.095

    platform_length = 2.10
    platform_width = 1.92
    platform_top_thickness = 0.014
    rear_axle_radius = 0.026

    lip_length = 0.40
    lip_width = 1.90
    lip_plate_thickness = 0.016
    lip_pin_radius = 0.022

    floor = model.part("pit_floor")
    floor.visual(
        Box(
            (
                pit_inner_length + 2.0 * front_rear_wall_thickness,
                pit_inner_width + 2.0 * side_wall_thickness,
                floor_thickness,
            )
        ),
        origin=Origin(xyz=(0.0, 0.0, -(pit_depth + floor_thickness / 2.0))),
        material=concrete,
        name="floor_slab",
    )
    floor.inertial = Inertial.from_geometry(
        Box(
            (
                pit_inner_length + 2.0 * front_rear_wall_thickness,
                pit_inner_width + 2.0 * side_wall_thickness,
                floor_thickness,
            )
        ),
        mass=850.0,
        origin=Origin(xyz=(0.0, 0.0, -(pit_depth + floor_thickness / 2.0))),
    )

    left_wall = model.part("left_wall")
    left_wall.visual(
        Box((pit_inner_length, side_wall_thickness, pit_depth)),
        origin=Origin(xyz=(0.0, 0.0, -pit_depth / 2.0)),
        material=concrete,
        name="left_wall_shell",
    )
    left_wall.inertial = Inertial.from_geometry(
        Box((pit_inner_length, side_wall_thickness, pit_depth)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, -pit_depth / 2.0)),
    )
    model.articulation(
        "floor_to_left_wall",
        ArticulationType.FIXED,
        parent=floor,
        child=left_wall,
        origin=Origin(xyz=(0.0, pit_inner_width / 2.0 + side_wall_thickness / 2.0, 0.0)),
    )

    right_wall = model.part("right_wall")
    right_wall.visual(
        Box((pit_inner_length, side_wall_thickness, pit_depth)),
        origin=Origin(xyz=(0.0, 0.0, -pit_depth / 2.0)),
        material=concrete,
        name="right_wall_shell",
    )
    right_wall.inertial = Inertial.from_geometry(
        Box((pit_inner_length, side_wall_thickness, pit_depth)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, -pit_depth / 2.0)),
    )
    model.articulation(
        "floor_to_right_wall",
        ArticulationType.FIXED,
        parent=floor,
        child=right_wall,
        origin=Origin(xyz=(0.0, -(pit_inner_width / 2.0 + side_wall_thickness / 2.0), 0.0)),
    )

    rear_wall = model.part("rear_wall")
    rear_wall.visual(
        Box((front_rear_wall_thickness, side_wall_thickness, pit_depth)),
        origin=Origin(xyz=(0.0, pit_inner_width / 2.0 + side_wall_thickness / 2.0, -pit_depth / 2.0)),
        material=concrete,
        name="rear_left_column",
    )
    rear_wall.visual(
        Box((front_rear_wall_thickness, side_wall_thickness, pit_depth)),
        origin=Origin(xyz=(0.0, -(pit_inner_width / 2.0 + side_wall_thickness / 2.0), -pit_depth / 2.0)),
        material=concrete,
        name="rear_right_column",
    )
    rear_wall.visual(
        Box((front_rear_wall_thickness, pit_inner_width + 2.0 * side_wall_thickness, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -0.52)),
        material=concrete,
        name="rear_base_beam",
    )
    rear_wall.visual(
        Box((0.06, pit_inner_width + 2.0 * side_wall_thickness, 0.08)),
        origin=Origin(xyz=(0.03, 0.0, hinge_axis_z)),
        material=steel,
        name="rear_hinge_bar",
    )
    rear_wall.visual(
        Cylinder(radius=rear_axle_radius, length=pit_inner_width - 0.02),
        origin=Origin(xyz=(0.02, 0.0, hinge_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_dark,
        name="rear_axle",
    )
    rear_wall.inertial = Inertial.from_geometry(
        Box((front_rear_wall_thickness, pit_inner_width + 2.0 * side_wall_thickness, pit_depth)),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, pit_depth / 2.0)),
    )
    model.articulation(
        "floor_to_rear_wall",
        ArticulationType.FIXED,
        parent=floor,
        child=rear_wall,
        origin=Origin(
            xyz=(
                -(pit_inner_length / 2.0 + front_rear_wall_thickness / 2.0),
                0.0,
                0.0,
            )
        ),
    )

    front_sill = model.part("front_sill")
    front_sill.visual(
        Box((front_rear_wall_thickness, pit_inner_width + 2.0 * side_wall_thickness, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, -0.52)),
        material=concrete,
        name="front_sill_shell",
    )
    front_sill.inertial = Inertial.from_geometry(
        Box((front_rear_wall_thickness, pit_inner_width + 2.0 * side_wall_thickness, 0.16)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, -0.52)),
    )
    model.articulation(
        "floor_to_front_sill",
        ArticulationType.FIXED,
        parent=floor,
        child=front_sill,
        origin=Origin(
            xyz=(
                pit_inner_length / 2.0 + front_rear_wall_thickness / 2.0,
                0.0,
                0.0,
            )
        ),
    )

    platform = model.part("platform")
    for idx, lug_y in enumerate((-0.68, 0.0, 0.68), start=1):
        platform.visual(
            Box((0.10, 0.26, 0.08)),
            origin=Origin(xyz=(0.05, lug_y, 0.0)),
            material=steel,
            name=f"rear_hinge_lug_{idx}",
        )
    platform.visual(
        Box((platform_length, platform_width, platform_top_thickness)),
        origin=Origin(xyz=(0.08 + platform_length / 2.0, 0.0, 0.088)),
        material=steel,
        name="deck_plate",
    )
    platform.visual(
        Box((0.14, platform_width, 0.09)),
        origin=Origin(xyz=(0.11, 0.0, 0.045)),
        material=steel,
        name="rear_cross_beam",
    )
    for idx, beam_y in enumerate((-0.58, 0.0, 0.58), start=1):
        platform.visual(
            Box((1.88, 0.14, 0.07)),
            origin=Origin(xyz=(1.12, beam_y, 0.035)),
            material=steel,
            name=f"stringer_{idx}",
        )
    platform.visual(
        Box((0.10, platform_width, 0.06)),
        origin=Origin(xyz=(2.02, 0.0, 0.045)),
        material=steel,
        name="front_cross_beam",
    )
    for idx, pin_y in enumerate((-(platform_width / 2.0 - 0.01), platform_width / 2.0 - 0.01), start=1):
        platform.visual(
            Cylinder(radius=lip_pin_radius, length=0.04),
            origin=Origin(xyz=(2.18, pin_y, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=axle_dark,
            name=f"lip_pin_stub_{idx}",
        )
    platform.inertial = Inertial.from_geometry(
        Box((2.18, platform_width, 0.12)),
        mass=620.0,
        origin=Origin(xyz=(1.09, 0.0, 0.04)),
    )
    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_wall,
        child=platform,
        origin=Origin(xyz=(front_rear_wall_thickness / 2.0, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.8,
            lower=math.radians(-3.0),
            upper=math.radians(32.0),
        ),
    )

    lip = model.part("lip")
    lip_plate_profile = [
        (0.0, 0.0),
        (0.0, 0.016),
        (0.12, 0.016),
        (0.36, 0.010),
        (0.36, 0.004),
        (0.30, 0.0),
    ]
    lip_plate_geom = ExtrudeGeometry(lip_plate_profile, lip_width, center=True)
    lip_plate_geom.rotate_x(math.pi / 2.0)
    lip_plate_mesh = mesh_from_geometry(lip_plate_geom, "lip_plate_wedge")
    lip.visual(
        Box((0.05, 0.07, 0.04)),
        origin=Origin(xyz=(0.04, -(lip_width / 2.0 - 0.02), 0.0)),
        material=galvanized,
        name="hinge_ear_1",
    )
    lip.visual(
        Box((0.05, 0.07, 0.04)),
        origin=Origin(xyz=(0.04, lip_width / 2.0 - 0.02, 0.0)),
        material=galvanized,
        name="hinge_ear_2",
    )
    lip.visual(
        Box((0.14, lip_width, 0.012)),
        origin=Origin(xyz=(0.10, 0.0, -0.006)),
        material=galvanized,
        name="hinge_leaf",
    )
    lip.visual(
        lip_plate_mesh,
        origin=Origin(xyz=(0.04, 0.0, -0.022)),
        material=galvanized,
        name="lip_plate",
    )
    for idx, stiffener_y in enumerate((-0.55, 0.0, 0.55), start=1):
        lip.visual(
            Box((0.24, 0.14, 0.040)),
            origin=Origin(xyz=(0.20, stiffener_y, -0.036)),
            material=galvanized,
            name=f"lip_stiffener_{idx}",
        )
    lip.inertial = Inertial.from_geometry(
        Box((0.41, lip_width, 0.06)),
        mass=95.0,
        origin=Origin(xyz=(0.205, 0.0, -0.020)),
    )
    model.articulation(
        "lip_hinge",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=lip,
        origin=Origin(xyz=(2.18, 0.0, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3500.0,
            velocity=1.5,
            lower=math.radians(-75.0),
            upper=math.radians(10.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    floor = object_model.get_part("pit_floor")
    left_wall = object_model.get_part("left_wall")
    right_wall = object_model.get_part("right_wall")
    rear_wall = object_model.get_part("rear_wall")
    front_sill = object_model.get_part("front_sill")
    platform = object_model.get_part("platform")
    lip = object_model.get_part("lip")

    rear_hinge = object_model.get_articulation("rear_hinge")
    lip_hinge = object_model.get_articulation("lip_hinge")

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
    ctx.allow_overlap(
        lip,
        platform,
        elem_a="hinge_ear_1",
        elem_b="lip_pin_stub_1",
        reason="Left hinge ear is intentionally captured by the platform pin stub.",
    )
    ctx.allow_overlap(
        lip,
        platform,
        elem_a="hinge_ear_2",
        elem_b="lip_pin_stub_2",
        reason="Right hinge ear is intentionally captured by the platform pin stub.",
    )
    ctx.allow_overlap(
        platform,
        rear_wall,
        elem_a="rear_hinge_lug_1",
        elem_b="rear_hinge_bar",
        reason="Rear hinge lug swings in a recessed hinge pocket around the rear axle support bar.",
    )
    ctx.allow_overlap(
        platform,
        rear_wall,
        elem_a="rear_hinge_lug_2",
        elem_b="rear_hinge_bar",
        reason="Rear hinge lug swings in a recessed hinge pocket around the rear axle support bar.",
    )
    ctx.allow_overlap(
        platform,
        rear_wall,
        elem_a="rear_hinge_lug_3",
        elem_b="rear_hinge_bar",
        reason="Rear hinge lug swings in a recessed hinge pocket around the rear axle support bar.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_wall, floor, name="left_wall_on_floor")
    ctx.expect_contact(right_wall, floor, name="right_wall_on_floor")
    ctx.expect_contact(rear_wall, floor, name="rear_wall_on_floor")
    ctx.expect_contact(front_sill, floor, name="front_sill_on_floor")
    ctx.expect_contact(left_wall, rear_wall, name="left_wall_to_rear_wall")
    ctx.expect_contact(right_wall, rear_wall, name="right_wall_to_rear_wall")
    ctx.expect_contact(left_wall, front_sill, name="left_wall_to_front_sill")
    ctx.expect_contact(right_wall, front_sill, name="right_wall_to_front_sill")
    ctx.expect_contact(platform, rear_wall, name="platform_on_rear_axle")
    ctx.expect_contact(lip, platform, name="lip_on_platform_pin")

    ctx.expect_overlap(platform, floor, axes="xy", min_overlap=1.80, name="platform_spans_pit")
    ctx.expect_overlap(lip, platform, axes="y", min_overlap=1.85, name="lip_matches_platform_width")
    ctx.expect_gap(platform, floor, axis="z", min_gap=0.46, max_gap=0.70, name="platform_above_pit_floor")
    ctx.expect_gap(left_wall, platform, axis="y", min_gap=0.02, max_gap=0.08, name="platform_clear_of_left_wall")
    ctx.expect_gap(platform, right_wall, axis="y", min_gap=0.02, max_gap=0.08, name="platform_clear_of_right_wall")

    ctx.check(
        "rear_hinge_axis_orientation",
        tuple(rear_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"unexpected rear hinge axis {rear_hinge.axis}",
    )
    ctx.check(
        "lip_hinge_axis_orientation",
        tuple(lip_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"unexpected lip hinge axis {lip_hinge.axis}",
    )

    platform_rest_aabb = ctx.part_world_aabb(platform)
    lip_rest_aabb = ctx.part_world_aabb(lip)
    assert platform_rest_aabb is not None
    assert lip_rest_aabb is not None

    rear_limits = rear_hinge.motion_limits
    assert rear_limits is not None
    assert rear_limits.lower is not None
    assert rear_limits.upper is not None

    with ctx.pose({rear_hinge: rear_limits.lower}):
        platform_low_aabb = ctx.part_world_aabb(platform)
        assert platform_low_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="rear_hinge_lower_no_floating")
        ctx.expect_contact(platform, rear_wall, name="rear_hinge_lower_contact")
        ctx.check(
            "rear_hinge_lower_front_drops",
            platform_low_aabb[1][2] < platform_rest_aabb[1][2] - 0.015,
            details=f"expected lowered platform max z below rest; rest={platform_rest_aabb}, lower={platform_low_aabb}",
        )

    with ctx.pose({rear_hinge: rear_limits.upper}):
        platform_high_aabb = ctx.part_world_aabb(platform)
        assert platform_high_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="rear_hinge_upper_no_floating")
        ctx.expect_contact(platform, rear_wall, name="rear_hinge_upper_contact")
        ctx.check(
            "rear_hinge_upper_front_rises",
            platform_high_aabb[1][2] > platform_rest_aabb[1][2] + 0.25,
            details=f"expected raised platform max z above rest; rest={platform_rest_aabb}, upper={platform_high_aabb}",
        )

    lip_limits = lip_hinge.motion_limits
    assert lip_limits is not None
    assert lip_limits.lower is not None
    assert lip_limits.upper is not None

    with ctx.pose({lip_hinge: lip_limits.lower}):
        lip_retracted_aabb = ctx.part_world_aabb(lip)
        assert lip_retracted_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="lip_hinge_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="lip_hinge_lower_no_floating")
        ctx.expect_contact(lip, platform, name="lip_hinge_lower_contact")
        ctx.check(
            "lip_hinge_lower_retracts_upward",
            lip_retracted_aabb[1][2] > lip_rest_aabb[1][2] + 0.25,
            details=f"expected lower-limit lip pose to swing up above the bridge pose; rest={lip_rest_aabb}, lower={lip_retracted_aabb}",
        )

    with ctx.pose({lip_hinge: lip_limits.upper}):
        lip_hanging_aabb = ctx.part_world_aabb(lip)
        assert lip_hanging_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="lip_hinge_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="lip_hinge_upper_no_floating")
        ctx.expect_contact(lip, platform, name="lip_hinge_upper_contact")
        ctx.check(
            "lip_hinge_upper_drops_below_bridge_pose",
            lip_hanging_aabb[0][2] < lip_rest_aabb[0][2] - 0.04,
            details=f"expected upper-limit lip pose to hang below the bridge pose; rest={lip_rest_aabb}, upper={lip_hanging_aabb}",
        )

    with ctx.pose({rear_hinge: math.radians(14.0), lip_hinge: math.radians(10.0)}):
        deployed_lip_aabb = ctx.part_world_aabb(lip)
        platform_deployed_aabb = ctx.part_world_aabb(platform)
        assert deployed_lip_aabb is not None
        assert platform_deployed_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="service_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="service_pose_no_floating")
        ctx.expect_contact(lip, platform, name="service_pose_lip_contact")
        ctx.check(
            "deployed_lip_projects_forward",
            deployed_lip_aabb[1][0] > platform_deployed_aabb[1][0] + 0.25,
            details=f"expected lip to project well beyond platform front edge; platform={platform_deployed_aabb}, lip={deployed_lip_aabb}",
        )
        ctx.check(
            "deployed_lip_stays_near_platform_top_plane",
            deployed_lip_aabb[1][2] <= platform_deployed_aabb[1][2] + 0.03,
            details=f"expected bridge lip upper edge to stay near platform top plane; platform={platform_deployed_aabb}, lip={deployed_lip_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
