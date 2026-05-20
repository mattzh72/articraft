from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


MAST_HEIGHT = 1.42
CHANNEL_WIDTH = 0.10
CHANNEL_DEPTH = 0.09
CHANNEL_WALL = 0.014
RAIL_HALF_SPAN = 0.22
MAST_OVERALL_WIDTH = 2.0 * RAIL_HALF_SPAN + CHANNEL_WIDTH
CROSSHEAD_HEIGHT = 0.07
BOTTOM_TIE_HEIGHT = 0.10
BOTTOM_TIE_WIDTH = 0.50
CROSS_MEMBER_Y = -0.05
LIFT_BASE_Z = 0.17
LIFT_TRAVEL = 0.55

CARRIAGE_HEIGHT = 0.44
CARRIAGE_SIDE_WIDTH = 0.072
CARRIAGE_SIDE_DEPTH = 0.024
CARRIAGE_SIDE_X = 0.206
CARRIAGE_BAR_WIDTH = 0.34
CARRIAGE_TOP_BAR_HEIGHT = 0.04
CARRIAGE_TOP_BAR_BOTTOM = 0.36
CARRIAGE_MID_BAR_HEIGHT = 0.04
CARRIAGE_MID_BAR_BOTTOM = 0.22
CARRIAGE_BOTTOM_BAR_HEIGHT = 0.06
CARRIAGE_BOTTOM_BAR_BOTTOM = 0.09
GUIDE_BLOCK_THICKNESS = 0.014
GUIDE_BLOCK_DEPTH = CHANNEL_DEPTH - 2.0 * CHANNEL_WALL
GUIDE_BLOCK_X = 0.249
GUIDE_BLOCK_HEIGHT = 0.13
GUIDE_LOWER_Z = 0.02
GUIDE_UPPER_Z = 0.26

BACKREST_HEIGHT = 0.40
BACKREST_BOTTOM_BAR_HEIGHT = 0.025
BACKREST_TOP_BAR_HEIGHT = 0.03
BACKREST_DEPTH = 0.012
BACKREST_MOUNT_DEPTH = 0.014
BACKREST_OFFSET_Y = 0.022

FORK_SPACING_HALF = 0.14
FORK_WIDTH = 0.05
FORK_LENGTH = 0.42
FORK_TINE_HEIGHT = 0.05
FORK_SHANK_DEPTH = 0.045
FORK_SHANK_HEIGHT = 0.20
FORK_MOUNT_Y = 0.02
FORK_MOUNT_Z = 0.03


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_forklift_mast")

    mast_material = model.material("mast_steel", rgba=(0.25, 0.27, 0.29, 1.0))
    carriage_material = model.material("carriage_orange", rgba=(0.92, 0.46, 0.12, 1.0))
    fork_material = model.material("fork_steel", rgba=(0.14, 0.14, 0.15, 1.0))

    mast = model.part("mast")

    left_channel_x = -RAIL_HALF_SPAN
    right_channel_x = RAIL_HALF_SPAN
    flange_y = CHANNEL_DEPTH / 2.0 - CHANNEL_WALL / 2.0
    left_web_x = left_channel_x - CHANNEL_WIDTH / 2.0 + CHANNEL_WALL / 2.0
    right_web_x = right_channel_x + CHANNEL_WIDTH / 2.0 - CHANNEL_WALL / 2.0

    mast.visual(
        Box((CHANNEL_WALL, CHANNEL_DEPTH, MAST_HEIGHT)),
        origin=Origin(xyz=(left_web_x, 0.0, MAST_HEIGHT / 2.0)),
        material=mast_material,
        name="left_web",
    )
    mast.visual(
        Box((CHANNEL_WIDTH, CHANNEL_WALL, MAST_HEIGHT)),
        origin=Origin(xyz=(left_channel_x, flange_y, MAST_HEIGHT / 2.0)),
        material=mast_material,
        name="left_front_flange",
    )
    mast.visual(
        Box((CHANNEL_WIDTH, CHANNEL_WALL, MAST_HEIGHT)),
        origin=Origin(xyz=(left_channel_x, -flange_y, MAST_HEIGHT / 2.0)),
        material=mast_material,
        name="left_rear_flange",
    )
    mast.visual(
        Box((CHANNEL_WALL, CHANNEL_DEPTH, MAST_HEIGHT)),
        origin=Origin(xyz=(right_web_x, 0.0, MAST_HEIGHT / 2.0)),
        material=mast_material,
        name="right_web",
    )
    mast.visual(
        Box((CHANNEL_WIDTH, CHANNEL_WALL, MAST_HEIGHT)),
        origin=Origin(xyz=(right_channel_x, flange_y, MAST_HEIGHT / 2.0)),
        material=mast_material,
        name="right_front_flange",
    )
    mast.visual(
        Box((CHANNEL_WIDTH, CHANNEL_WALL, MAST_HEIGHT)),
        origin=Origin(xyz=(right_channel_x, -flange_y, MAST_HEIGHT / 2.0)),
        material=mast_material,
        name="right_rear_flange",
    )
    mast.visual(
        Box((MAST_OVERALL_WIDTH, 0.065, CROSSHEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, CROSS_MEMBER_Y, MAST_HEIGHT - CROSSHEAD_HEIGHT / 2.0)),
        material=mast_material,
        name="top_crosshead",
    )
    mast.visual(
        Box((BOTTOM_TIE_WIDTH, 0.065, BOTTOM_TIE_HEIGHT)),
        origin=Origin(xyz=(0.0, CROSS_MEMBER_Y, BOTTOM_TIE_HEIGHT / 2.0)),
        material=mast_material,
        name="bottom_tie",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_SIDE_WIDTH, CARRIAGE_SIDE_DEPTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(-CARRIAGE_SIDE_X, 0.0, CARRIAGE_HEIGHT / 2.0)),
        material=carriage_material,
        name="left_side",
    )
    carriage.visual(
        Box((CARRIAGE_SIDE_WIDTH, CARRIAGE_SIDE_DEPTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(CARRIAGE_SIDE_X, 0.0, CARRIAGE_HEIGHT / 2.0)),
        material=carriage_material,
        name="right_side",
    )
    carriage.visual(
        Box((CARRIAGE_BAR_WIDTH, 0.03, CARRIAGE_TOP_BAR_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_TOP_BAR_BOTTOM + CARRIAGE_TOP_BAR_HEIGHT / 2.0)
        ),
        material=carriage_material,
        name="top_bar",
    )
    carriage.visual(
        Box((CARRIAGE_BAR_WIDTH, 0.022, CARRIAGE_MID_BAR_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_MID_BAR_BOTTOM + CARRIAGE_MID_BAR_HEIGHT / 2.0)
        ),
        material=carriage_material,
        name="mid_bar",
    )
    carriage.visual(
        Box((CARRIAGE_BAR_WIDTH, 0.04, CARRIAGE_BOTTOM_BAR_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_BOTTOM_BAR_BOTTOM + CARRIAGE_BOTTOM_BAR_HEIGHT / 2.0)
        ),
        material=carriage_material,
        name="bottom_bar",
    )
    carriage.visual(
        Box((GUIDE_BLOCK_THICKNESS, GUIDE_BLOCK_DEPTH, GUIDE_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(-GUIDE_BLOCK_X, 0.0, GUIDE_LOWER_Z + GUIDE_BLOCK_HEIGHT / 2.0)
        ),
        material=carriage_material,
        name="left_lower_guide",
    )
    carriage.visual(
        Box((GUIDE_BLOCK_THICKNESS, GUIDE_BLOCK_DEPTH, GUIDE_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(-GUIDE_BLOCK_X, 0.0, GUIDE_UPPER_Z + GUIDE_BLOCK_HEIGHT / 2.0)
        ),
        material=carriage_material,
        name="left_upper_guide",
    )
    carriage.visual(
        Box((GUIDE_BLOCK_THICKNESS, GUIDE_BLOCK_DEPTH, GUIDE_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(GUIDE_BLOCK_X, 0.0, GUIDE_LOWER_Z + GUIDE_BLOCK_HEIGHT / 2.0)
        ),
        material=carriage_material,
        name="right_lower_guide",
    )
    carriage.visual(
        Box((GUIDE_BLOCK_THICKNESS, GUIDE_BLOCK_DEPTH, GUIDE_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(GUIDE_BLOCK_X, 0.0, GUIDE_UPPER_Z + GUIDE_BLOCK_HEIGHT / 2.0)
        ),
        material=carriage_material,
        name="right_upper_guide",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.45, BACKREST_MOUNT_DEPTH, BACKREST_BOTTOM_BAR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BACKREST_BOTTOM_BAR_HEIGHT / 2.0)),
        material=carriage_material,
        name="bottom_mount",
    )
    backrest.visual(
        Box((0.02, BACKREST_DEPTH, BACKREST_HEIGHT)),
        origin=Origin(xyz=(-0.205, 0.0, BACKREST_HEIGHT / 2.0)),
        material=carriage_material,
        name="left_upright",
    )
    backrest.visual(
        Box((0.02, BACKREST_DEPTH, BACKREST_HEIGHT)),
        origin=Origin(xyz=(0.205, 0.0, BACKREST_HEIGHT / 2.0)),
        material=carriage_material,
        name="right_upright",
    )
    backrest.visual(
        Box((0.45, BACKREST_DEPTH, BACKREST_TOP_BAR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BACKREST_HEIGHT - BACKREST_TOP_BAR_HEIGHT / 2.0)),
        material=carriage_material,
        name="top_rail",
    )
    for index, x_pos in enumerate((-0.11, 0.0, 0.11), start=1):
        backrest.visual(
            Box((0.014, BACKREST_DEPTH, 0.34)),
            origin=Origin(xyz=(x_pos, 0.0, 0.025 + 0.34 / 2.0)),
            material=carriage_material,
            name=f"slat_{index}",
        )

    left_fork = model.part("left_fork")
    left_fork.visual(
        Box((FORK_WIDTH, FORK_LENGTH, FORK_TINE_HEIGHT)),
        origin=Origin(xyz=(0.0, FORK_LENGTH / 2.0, FORK_TINE_HEIGHT / 2.0)),
        material=fork_material,
        name="tine",
    )
    left_fork.visual(
        Box((FORK_WIDTH, FORK_SHANK_DEPTH, FORK_SHANK_HEIGHT)),
        origin=Origin(xyz=(0.0, FORK_SHANK_DEPTH / 2.0, FORK_SHANK_HEIGHT / 2.0)),
        material=fork_material,
        name="shank",
    )

    right_fork = model.part("right_fork")
    right_fork.visual(
        Box((FORK_WIDTH, FORK_LENGTH, FORK_TINE_HEIGHT)),
        origin=Origin(xyz=(0.0, FORK_LENGTH / 2.0, FORK_TINE_HEIGHT / 2.0)),
        material=fork_material,
        name="tine",
    )
    right_fork.visual(
        Box((FORK_WIDTH, FORK_SHANK_DEPTH, FORK_SHANK_HEIGHT)),
        origin=Origin(xyz=(0.0, FORK_SHANK_DEPTH / 2.0, FORK_SHANK_HEIGHT / 2.0)),
        material=fork_material,
        name="shank",
    )

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.40,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_backrest",
        ArticulationType.FIXED,
        parent=carriage,
        child=backrest,
        origin=Origin(
            xyz=(0.0, BACKREST_OFFSET_Y, CARRIAGE_TOP_BAR_BOTTOM + CARRIAGE_TOP_BAR_HEIGHT)
        ),
    )
    model.articulation(
        "carriage_to_left_fork",
        ArticulationType.FIXED,
        parent=carriage,
        child=left_fork,
        origin=Origin(xyz=(-FORK_SPACING_HALF, FORK_MOUNT_Y, FORK_MOUNT_Z)),
    )
    model.articulation(
        "carriage_to_right_fork",
        ArticulationType.FIXED,
        parent=carriage,
        child=right_fork,
        origin=Origin(xyz=(FORK_SPACING_HALF, FORK_MOUNT_Y, FORK_MOUNT_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    backrest = object_model.get_part("backrest")
    left_fork = object_model.get_part("left_fork")
    right_fork = object_model.get_part("right_fork")
    lift = object_model.get_articulation("mast_lift")

    left_web = mast.get_visual("left_web")
    right_web = mast.get_visual("right_web")
    top_crosshead = mast.get_visual("top_crosshead")
    bottom_tie = mast.get_visual("bottom_tie")
    top_bar = carriage.get_visual("top_bar")
    bottom_bar = carriage.get_visual("bottom_bar")
    left_lower_guide = carriage.get_visual("left_lower_guide")
    right_lower_guide = carriage.get_visual("right_lower_guide")
    left_upper_guide = carriage.get_visual("left_upper_guide")
    right_upper_guide = carriage.get_visual("right_upper_guide")
    backrest_mount = backrest.get_visual("bottom_mount")
    left_fork_shank = left_fork.get_visual("shank")
    right_fork_shank = right_fork.get_visual("shank")

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
        "all_main_parts_present",
        all(part is not None for part in (mast, carriage, backrest, left_fork, right_fork)),
        "Expected mast, carriage, backrest, and both forks.",
    )
    ctx.check(
        "lift_axis_vertical",
        tuple(round(value, 6) for value in lift.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical prismatic axis, got {lift.axis!r}.",
    )

    ctx.expect_contact(
        carriage,
        mast,
        elem_a=left_lower_guide,
        elem_b=left_web,
        contact_tol=0.001,
        name="left_lower_guide_contacts_left_rail",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=right_lower_guide,
        elem_b=right_web,
        contact_tol=0.001,
        name="right_lower_guide_contacts_right_rail",
    )
    ctx.expect_contact(
        backrest,
        carriage,
        elem_a=backrest_mount,
        elem_b=top_bar,
        contact_tol=0.001,
        name="backrest_mounts_to_carriage_top_bar",
    )
    ctx.expect_contact(
        left_fork,
        carriage,
        elem_a=left_fork_shank,
        elem_b=bottom_bar,
        contact_tol=0.001,
        name="left_fork_mounts_to_carriage",
    )
    ctx.expect_contact(
        right_fork,
        carriage,
        elem_a=right_fork_shank,
        elem_b=bottom_bar,
        contact_tol=0.001,
        name="right_fork_mounts_to_carriage",
    )
    ctx.expect_within(
        carriage,
        mast,
        axes="x",
        margin=0.02,
        name="carriage_stays_within_mast_width",
    )
    ctx.expect_origin_gap(
        carriage,
        mast,
        axis="z",
        min_gap=LIFT_BASE_Z - 0.001,
        max_gap=LIFT_BASE_Z + 0.001,
        name="carriage_rest_height",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="z",
        positive_elem=bottom_bar,
        negative_elem=bottom_tie,
        min_gap=0.12,
        name="carriage_clears_bottom_tie_at_rest",
    )
    ctx.expect_origin_gap(
        backrest,
        carriage,
        axis="y",
        min_gap=BACKREST_OFFSET_Y - 0.001,
        max_gap=BACKREST_OFFSET_Y + 0.001,
        name="backrest_is_forward_of_carriage",
    )
    ctx.expect_origin_gap(
        backrest,
        carriage,
        axis="z",
        min_gap=CARRIAGE_TOP_BAR_BOTTOM + CARRIAGE_TOP_BAR_HEIGHT - 0.001,
        max_gap=CARRIAGE_TOP_BAR_BOTTOM + CARRIAGE_TOP_BAR_HEIGHT + 0.001,
        name="backrest_sits_above_carriage",
    )
    ctx.expect_origin_distance(
        left_fork,
        right_fork,
        axes="x",
        min_dist=2.0 * FORK_SPACING_HALF - 0.001,
        max_dist=2.0 * FORK_SPACING_HALF + 0.001,
        name="fork_pair_spacing",
    )

    limits = lift.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({lift: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="mast_lift_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="mast_lift_lower_no_floating")
            ctx.expect_contact(
                carriage,
                mast,
                elem_a=left_lower_guide,
                elem_b=left_web,
                contact_tol=0.001,
                name="left_lower_guide_contact_at_lower_pose",
            )
            ctx.expect_contact(
                carriage,
                mast,
                elem_a=right_lower_guide,
                elem_b=right_web,
                contact_tol=0.001,
                name="right_lower_guide_contact_at_lower_pose",
            )
        with ctx.pose({lift: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="mast_lift_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="mast_lift_upper_no_floating")
            ctx.expect_contact(
                carriage,
                mast,
                elem_a=left_upper_guide,
                elem_b=left_web,
                contact_tol=0.001,
                name="left_upper_guide_contact_at_upper_pose",
            )
            ctx.expect_contact(
                carriage,
                mast,
                elem_a=right_upper_guide,
                elem_b=right_web,
                contact_tol=0.001,
                name="right_upper_guide_contact_at_upper_pose",
            )
            ctx.expect_origin_gap(
                carriage,
                mast,
                axis="z",
                min_gap=LIFT_BASE_Z + LIFT_TRAVEL - 0.001,
                max_gap=LIFT_BASE_Z + LIFT_TRAVEL + 0.001,
                name="carriage_reaches_upper_pose",
            )
            ctx.expect_gap(
                mast,
                carriage,
                axis="z",
                positive_elem=top_crosshead,
                negative_elem=top_bar,
                min_gap=0.18,
                name="carriage_clears_top_crosshead_at_upper_pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
