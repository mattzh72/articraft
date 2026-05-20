from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LEN = 0.72
OUTER_WIDTH = 0.12
OUTER_BASE_T = 0.012
OUTER_RAIL_W = 0.018
OUTER_RAIL_H = 0.035
OUTER_RAIL_Y = 0.042
OUTER_TOP_Z = OUTER_BASE_T + OUTER_RAIL_H

CARRIAGE_LEN = 0.22
CARRIAGE_SHOE_W = 0.020
CARRIAGE_SHOE_H = 0.016
CARRIAGE_BRIDGE_W = 0.084
CARRIAGE_BRIDGE_T = 0.012
CARRIAGE_HEIGHT = CARRIAGE_SHOE_H + CARRIAGE_BRIDGE_T

SECOND_GUIDE_LEN = 0.30
SECOND_GUIDE_BASE_W = 0.080
SECOND_GUIDE_BASE_T = 0.010
SECOND_GUIDE_RAIL_W = 0.012
SECOND_GUIDE_RAIL_H = 0.022
SECOND_GUIDE_RAIL_Y = 0.026
SECOND_GUIDE_HEIGHT = SECOND_GUIDE_BASE_T + SECOND_GUIDE_RAIL_H

END_SHOE_LEN = 0.09
END_SHOE_W = 0.016
END_SHOE_H = 0.014
END_BRIDGE_W = 0.062
END_BRIDGE_T = 0.010
END_PLATE_T = 0.012
END_PLATE_W = 0.11
END_PLATE_H = 0.12
END_HEIGHT = END_SHOE_H + END_BRIDGE_T + END_PLATE_H

STAGE1_START_X = -0.15
STAGE1_TRAVEL = 0.30
SECOND_GUIDE_MOUNT_X = 0.025
STAGE2_START_X = -0.05
STAGE2_TRAVEL = 0.10


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_linear_slide")

    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    rail_silver = model.material("rail_silver", rgba=(0.66, 0.69, 0.73, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.91, 0.42, 0.11, 1.0))

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        Box((OUTER_LEN, OUTER_WIDTH, OUTER_BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_BASE_T / 2.0)),
        material=dark_steel,
        name="base_plate",
    )
    outer_guide.visual(
        Box((OUTER_LEN, OUTER_RAIL_W, OUTER_RAIL_H)),
        origin=Origin(
            xyz=(0.0, OUTER_RAIL_Y, OUTER_BASE_T + (OUTER_RAIL_H / 2.0))
        ),
        material=rail_silver,
        name="left_rail",
    )
    outer_guide.visual(
        Box((OUTER_LEN, OUTER_RAIL_W, OUTER_RAIL_H)),
        origin=Origin(
            xyz=(0.0, -OUTER_RAIL_Y, OUTER_BASE_T + (OUTER_RAIL_H / 2.0))
        ),
        material=rail_silver,
        name="right_rail",
    )
    outer_guide.inertial = Inertial.from_geometry(
        Box((OUTER_LEN, OUTER_WIDTH, OUTER_TOP_Z)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TOP_Z / 2.0)),
    )

    first_carriage = model.part("first_carriage")
    first_carriage.visual(
        Box((CARRIAGE_LEN, CARRIAGE_SHOE_W, CARRIAGE_SHOE_H)),
        origin=Origin(xyz=(0.0, OUTER_RAIL_Y, CARRIAGE_SHOE_H / 2.0)),
        material=aluminum,
        name="left_shoe",
    )
    first_carriage.visual(
        Box((CARRIAGE_LEN, CARRIAGE_SHOE_W, CARRIAGE_SHOE_H)),
        origin=Origin(xyz=(0.0, -OUTER_RAIL_Y, CARRIAGE_SHOE_H / 2.0)),
        material=aluminum,
        name="right_shoe",
    )
    first_carriage.visual(
        Box((CARRIAGE_LEN, CARRIAGE_BRIDGE_W, CARRIAGE_BRIDGE_T)),
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_SHOE_H + (CARRIAGE_BRIDGE_T / 2.0))
        ),
        material=aluminum,
        name="bridge",
    )
    first_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LEN, 0.104, CARRIAGE_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
    )

    second_guide = model.part("second_guide")
    second_guide.visual(
        Box((SECOND_GUIDE_LEN, SECOND_GUIDE_BASE_W, SECOND_GUIDE_BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, SECOND_GUIDE_BASE_T / 2.0)),
        material=rail_silver,
        name="guide_base",
    )
    second_guide.visual(
        Box((SECOND_GUIDE_LEN, SECOND_GUIDE_RAIL_W, SECOND_GUIDE_RAIL_H)),
        origin=Origin(
            xyz=(
                0.0,
                SECOND_GUIDE_RAIL_Y,
                SECOND_GUIDE_BASE_T + (SECOND_GUIDE_RAIL_H / 2.0),
            )
        ),
        material=dark_steel,
        name="left_track",
    )
    second_guide.visual(
        Box((SECOND_GUIDE_LEN, SECOND_GUIDE_RAIL_W, SECOND_GUIDE_RAIL_H)),
        origin=Origin(
            xyz=(
                0.0,
                -SECOND_GUIDE_RAIL_Y,
                SECOND_GUIDE_BASE_T + (SECOND_GUIDE_RAIL_H / 2.0),
            )
        ),
        material=dark_steel,
        name="right_track",
    )
    second_guide.inertial = Inertial.from_geometry(
        Box((SECOND_GUIDE_LEN, SECOND_GUIDE_BASE_W, SECOND_GUIDE_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, SECOND_GUIDE_HEIGHT / 2.0)),
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Box((END_SHOE_LEN, END_SHOE_W, END_SHOE_H)),
        origin=Origin(xyz=(0.0, SECOND_GUIDE_RAIL_Y, END_SHOE_H / 2.0)),
        material=aluminum,
        name="left_shoe",
    )
    end_plate.visual(
        Box((END_SHOE_LEN, END_SHOE_W, END_SHOE_H)),
        origin=Origin(xyz=(0.0, -SECOND_GUIDE_RAIL_Y, END_SHOE_H / 2.0)),
        material=aluminum,
        name="right_shoe",
    )
    end_plate.visual(
        Box((END_SHOE_LEN, END_BRIDGE_W, END_BRIDGE_T)),
        origin=Origin(xyz=(0.0, 0.0, END_SHOE_H + (END_BRIDGE_T / 2.0))),
        material=aluminum,
        name="bridge",
    )
    end_plate.visual(
        Box((END_PLATE_T, END_PLATE_W, END_PLATE_H)),
        origin=Origin(
            xyz=(
                (END_SHOE_LEN / 2.0) - (END_PLATE_T / 2.0),
                0.0,
                END_SHOE_H + END_BRIDGE_T + (END_PLATE_H / 2.0),
            )
        ),
        material=accent_orange,
        name="plate_panel",
    )
    end_plate.inertial = Inertial.from_geometry(
        Box((END_SHOE_LEN, END_PLATE_W, END_HEIGHT)),
        mass=1.0,
        origin=Origin(xyz=(0.015, 0.0, END_HEIGHT / 2.0)),
    )

    model.articulation(
        "outer_to_carriage",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=first_carriage,
        origin=Origin(xyz=(STAGE1_START_X, 0.0, OUTER_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.50,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_second_guide",
        ArticulationType.FIXED,
        parent=first_carriage,
        child=second_guide,
        origin=Origin(xyz=(SECOND_GUIDE_MOUNT_X, 0.0, CARRIAGE_HEIGHT)),
    )
    model.articulation(
        "second_guide_to_end_plate",
        ArticulationType.PRISMATIC,
        parent=second_guide,
        child=end_plate,
        origin=Origin(xyz=(STAGE2_START_X, 0.0, SECOND_GUIDE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.40,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_guide = object_model.get_part("outer_guide")
    first_carriage = object_model.get_part("first_carriage")
    second_guide = object_model.get_part("second_guide")
    end_plate = object_model.get_part("end_plate")

    stage1 = object_model.get_articulation("outer_to_carriage")
    guide_mount = object_model.get_articulation("carriage_to_second_guide")
    stage2 = object_model.get_articulation("second_guide_to_end_plate")

    outer_left_rail = outer_guide.get_visual("left_rail")
    outer_right_rail = outer_guide.get_visual("right_rail")
    carriage_left_shoe = first_carriage.get_visual("left_shoe")
    carriage_right_shoe = first_carriage.get_visual("right_shoe")
    carriage_bridge = first_carriage.get_visual("bridge")
    guide_base = second_guide.get_visual("guide_base")
    guide_left_track = second_guide.get_visual("left_track")
    guide_right_track = second_guide.get_visual("right_track")
    end_left_shoe = end_plate.get_visual("left_shoe")
    end_right_shoe = end_plate.get_visual("right_shoe")

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
        "stage1_prismatic_axis",
        stage1.articulation_type == ArticulationType.PRISMATIC
        and tuple(stage1.axis) == (1.0, 0.0, 0.0),
        f"Expected stage1 prismatic +X axis, got {stage1.articulation_type=} {stage1.axis=}",
    )
    ctx.check(
        "guide_mount_fixed",
        guide_mount.articulation_type == ArticulationType.FIXED,
        f"Expected fixed carriage mount, got {guide_mount.articulation_type=}",
    )
    ctx.check(
        "stage2_prismatic_axis",
        stage2.articulation_type == ArticulationType.PRISMATIC
        and tuple(stage2.axis) == (1.0, 0.0, 0.0),
        f"Expected stage2 prismatic +X axis, got {stage2.articulation_type=} {stage2.axis=}",
    )

    stage1_limits = stage1.motion_limits
    stage2_limits = stage2.motion_limits
    ctx.check(
        "stage1_limits_present",
        stage1_limits is not None
        and stage1_limits.lower == 0.0
        and stage1_limits.upper == STAGE1_TRAVEL,
        f"Unexpected stage1 limits: {stage1_limits=}",
    )
    ctx.check(
        "stage2_limits_present",
        stage2_limits is not None
        and stage2_limits.lower == 0.0
        and stage2_limits.upper == STAGE2_TRAVEL,
        f"Unexpected stage2 limits: {stage2_limits=}",
    )

    with ctx.pose({stage1: 0.0, stage2: 0.0}):
        ctx.expect_contact(
            first_carriage,
            outer_guide,
            elem_a=carriage_left_shoe,
            elem_b=outer_left_rail,
            name="stage1_left_contact_retracted",
        )
        ctx.expect_contact(
            first_carriage,
            outer_guide,
            elem_a=carriage_right_shoe,
            elem_b=outer_right_rail,
            name="stage1_right_contact_retracted",
        )
        ctx.expect_gap(
            first_carriage,
            outer_guide,
            axis="z",
            min_gap=0.0,
            max_gap=0.0,
            positive_elem=carriage_left_shoe,
            negative_elem=outer_left_rail,
            name="stage1_left_flush_retracted",
        )
        ctx.expect_gap(
            second_guide,
            first_carriage,
            axis="z",
            max_gap=1e-6,
            max_penetration=1e-6,
            positive_elem=guide_base,
            negative_elem=carriage_bridge,
            name="guide_mount_flush_retracted",
        )
        ctx.expect_contact(
            end_plate,
            second_guide,
            elem_a=end_left_shoe,
            elem_b=guide_left_track,
            name="stage2_left_contact_retracted",
        )
        ctx.expect_contact(
            end_plate,
            second_guide,
            elem_a=end_right_shoe,
            elem_b=guide_right_track,
            name="stage2_right_contact_retracted",
        )
        ctx.expect_overlap(
            first_carriage,
            outer_guide,
            axes="x",
            min_overlap=0.20,
            name="stage1_supported_length_retracted",
        )
        ctx.expect_within(
            first_carriage,
            outer_guide,
            axes="y",
            margin=0.0,
            name="carriage_within_outer_width",
        )
        ctx.expect_within(
            second_guide,
            first_carriage,
            axes="y",
            margin=0.0,
            name="second_guide_within_carriage_width",
        )

    with ctx.pose({stage1: 0.0, stage2: 0.0}):
        stage1_rest = ctx.part_world_position(first_carriage)
        stage2_rest = ctx.part_world_position(end_plate)
    with ctx.pose({stage1: STAGE1_TRAVEL, stage2: 0.0}):
        stage1_extended = ctx.part_world_position(first_carriage)
    with ctx.pose({stage1: 0.0, stage2: STAGE2_TRAVEL}):
        stage2_extended = ctx.part_world_position(end_plate)

    if stage1_rest is not None and stage1_extended is not None:
        dx = stage1_extended[0] - stage1_rest[0]
        dy = stage1_extended[1] - stage1_rest[1]
        dz = stage1_extended[2] - stage1_rest[2]
        ctx.check(
            "stage1_moves_along_x_only",
            abs(dx - STAGE1_TRAVEL) < 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9,
            f"Stage1 displacement was {(dx, dy, dz)}",
        )
    else:
        ctx.fail("stage1_position_probe", "Could not resolve first carriage world positions")

    if stage2_rest is not None and stage2_extended is not None:
        dx = stage2_extended[0] - stage2_rest[0]
        dy = stage2_extended[1] - stage2_rest[1]
        dz = stage2_extended[2] - stage2_rest[2]
        ctx.check(
            "stage2_moves_along_x_only",
            abs(dx - STAGE2_TRAVEL) < 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9,
            f"Stage2 displacement was {(dx, dy, dz)}",
        )
    else:
        ctx.fail("stage2_position_probe", "Could not resolve end plate world positions")

    for label, pos in (("lower", 0.0), ("upper", STAGE1_TRAVEL)):
        with ctx.pose({stage1: pos, stage2: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"stage1_{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"stage1_{label}_no_floating")
            ctx.expect_contact(
                first_carriage,
                outer_guide,
                elem_a=carriage_left_shoe,
                elem_b=outer_left_rail,
                name=f"stage1_left_contact_{label}",
            )
            ctx.expect_contact(
                first_carriage,
                outer_guide,
                elem_a=carriage_right_shoe,
                elem_b=outer_right_rail,
                name=f"stage1_right_contact_{label}",
            )
            ctx.expect_overlap(
                first_carriage,
                outer_guide,
                axes="x",
                min_overlap=0.20,
                name=f"stage1_supported_length_{label}",
            )

    for label, pos in (("lower", 0.0), ("upper", STAGE2_TRAVEL)):
        with ctx.pose({stage1: STAGE1_TRAVEL, stage2: pos}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"stage2_{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"stage2_{label}_no_floating")
            ctx.expect_contact(
                end_plate,
                second_guide,
                elem_a=end_left_shoe,
                elem_b=guide_left_track,
                name=f"stage2_left_contact_{label}",
            )
            ctx.expect_contact(
                end_plate,
                second_guide,
                elem_a=end_right_shoe,
                elem_b=guide_right_track,
                name=f"stage2_right_contact_{label}",
            )
            ctx.expect_overlap(
                end_plate,
                second_guide,
                axes="x",
                min_overlap=0.08,
                elem_a=end_left_shoe,
                elem_b=guide_left_track,
                name=f"stage2_supported_length_{label}",
            )

    with ctx.pose({stage1: STAGE1_TRAVEL, stage2: STAGE2_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="fully_extended_no_overlap")
        ctx.fail_if_isolated_parts(name="fully_extended_no_floating")
        ctx.expect_contact(
            second_guide,
            first_carriage,
            elem_a=guide_base,
            elem_b=carriage_bridge,
            name="guide_mount_contact_fully_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
