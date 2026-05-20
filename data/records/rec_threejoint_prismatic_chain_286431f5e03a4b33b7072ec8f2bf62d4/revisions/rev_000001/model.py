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


WALL_T = 0.0035

SUPPORT_LENGTH = 0.68
STAGE1_LENGTH = 0.56
STAGE2_LENGTH = 0.44
STAGE3_LENGTH = 0.32

SUPPORT_WIDTH = 0.072
STAGE1_WIDTH = 0.062
STAGE2_WIDTH = 0.052
STAGE3_WIDTH = 0.042

SUPPORT_DEPTH = 0.042
STAGE1_DEPTH = 0.034
STAGE2_DEPTH = 0.027
STAGE3_DEPTH = 0.020

SUPPORT_FLANGE_WIDTH = 0.092
SUPPORT_FLANGE_THICKNESS = 0.005

TRAVEL_1 = 0.24
TRAVEL_2 = 0.18
TRAVEL_3 = 0.14


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_three_stage_slide")

    model.material("support_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("slider_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    support = model.part("support")
    support.visual(
        Box((SUPPORT_LENGTH, SUPPORT_FLANGE_WIDTH, WALL_T + SUPPORT_FLANGE_THICKNESS)),
        origin=Origin(xyz=(SUPPORT_LENGTH / 2.0, 0.0, (-WALL_T + SUPPORT_FLANGE_THICKNESS) / 2.0)),
        material="support_dark",
        name="support_cap",
    )
    support.visual(
        Box((SUPPORT_LENGTH, WALL_T, SUPPORT_DEPTH)),
        origin=Origin(
            xyz=(
                SUPPORT_LENGTH / 2.0,
                (SUPPORT_WIDTH / 2.0) - (WALL_T / 2.0),
                -SUPPORT_DEPTH / 2.0,
            )
        ),
        material="support_dark",
        name="support_left_wall",
    )
    support.visual(
        Box((SUPPORT_LENGTH, WALL_T, SUPPORT_DEPTH)),
        origin=Origin(
            xyz=(
                SUPPORT_LENGTH / 2.0,
                -(SUPPORT_WIDTH / 2.0) + (WALL_T / 2.0),
                -SUPPORT_DEPTH / 2.0,
            )
        ),
        material="support_dark",
        name="support_right_wall",
    )
    support.inertial = Inertial.from_geometry(
        Box((SUPPORT_LENGTH, SUPPORT_FLANGE_WIDTH, SUPPORT_DEPTH + SUPPORT_FLANGE_THICKNESS)),
        mass=2.8,
        origin=Origin(
            xyz=(
                SUPPORT_LENGTH / 2.0,
                0.0,
                -(SUPPORT_DEPTH / 2.0) + (SUPPORT_FLANGE_THICKNESS / 2.0),
            )
        ),
    )

    stage1 = model.part("stage1")
    stage1.visual(
        Box((STAGE1_LENGTH, STAGE1_WIDTH, WALL_T)),
        origin=Origin(xyz=(STAGE1_LENGTH / 2.0, 0.0, -WALL_T / 2.0)),
        material="slider_steel",
        name="stage1_top",
    )
    stage1.visual(
        Box((STAGE1_LENGTH, WALL_T, STAGE1_DEPTH)),
        origin=Origin(
            xyz=(
                STAGE1_LENGTH / 2.0,
                (STAGE1_WIDTH / 2.0) - (WALL_T / 2.0),
                -STAGE1_DEPTH / 2.0,
            )
        ),
        material="slider_steel",
        name="stage1_left_wall",
    )
    stage1.visual(
        Box((STAGE1_LENGTH, WALL_T, STAGE1_DEPTH)),
        origin=Origin(
            xyz=(
                STAGE1_LENGTH / 2.0,
                -(STAGE1_WIDTH / 2.0) + (WALL_T / 2.0),
                -STAGE1_DEPTH / 2.0,
            )
        ),
        material="slider_steel",
        name="stage1_right_wall",
    )
    stage1.inertial = Inertial.from_geometry(
        Box((STAGE1_LENGTH, STAGE1_WIDTH, STAGE1_DEPTH)),
        mass=1.9,
        origin=Origin(xyz=(STAGE1_LENGTH / 2.0, 0.0, -STAGE1_DEPTH / 2.0)),
    )

    stage2 = model.part("stage2")
    stage2.visual(
        Box((STAGE2_LENGTH, STAGE2_WIDTH, WALL_T)),
        origin=Origin(xyz=(STAGE2_LENGTH / 2.0, 0.0, -WALL_T / 2.0)),
        material="slider_steel",
        name="stage2_top",
    )
    stage2.visual(
        Box((STAGE2_LENGTH, WALL_T, STAGE2_DEPTH)),
        origin=Origin(
            xyz=(
                STAGE2_LENGTH / 2.0,
                (STAGE2_WIDTH / 2.0) - (WALL_T / 2.0),
                -STAGE2_DEPTH / 2.0,
            )
        ),
        material="slider_steel",
        name="stage2_left_wall",
    )
    stage2.visual(
        Box((STAGE2_LENGTH, WALL_T, STAGE2_DEPTH)),
        origin=Origin(
            xyz=(
                STAGE2_LENGTH / 2.0,
                -(STAGE2_WIDTH / 2.0) + (WALL_T / 2.0),
                -STAGE2_DEPTH / 2.0,
            )
        ),
        material="slider_steel",
        name="stage2_right_wall",
    )
    stage2.inertial = Inertial.from_geometry(
        Box((STAGE2_LENGTH, STAGE2_WIDTH, STAGE2_DEPTH)),
        mass=1.4,
        origin=Origin(xyz=(STAGE2_LENGTH / 2.0, 0.0, -STAGE2_DEPTH / 2.0)),
    )

    stage3 = model.part("stage3")
    stage3.visual(
        Box((STAGE3_LENGTH, STAGE3_WIDTH, WALL_T)),
        origin=Origin(xyz=(STAGE3_LENGTH / 2.0, 0.0, -WALL_T / 2.0)),
        material="slider_steel",
        name="stage3_top",
    )
    stage3.visual(
        Box((STAGE3_LENGTH, WALL_T, STAGE3_DEPTH)),
        origin=Origin(
            xyz=(
                STAGE3_LENGTH / 2.0,
                (STAGE3_WIDTH / 2.0) - (WALL_T / 2.0),
                -STAGE3_DEPTH / 2.0,
            )
        ),
        material="slider_steel",
        name="stage3_left_wall",
    )
    stage3.visual(
        Box((STAGE3_LENGTH, WALL_T, STAGE3_DEPTH)),
        origin=Origin(
            xyz=(
                STAGE3_LENGTH / 2.0,
                -(STAGE3_WIDTH / 2.0) + (WALL_T / 2.0),
                -STAGE3_DEPTH / 2.0,
            )
        ),
        material="slider_steel",
        name="stage3_right_wall",
    )
    stage3.inertial = Inertial.from_geometry(
        Box((STAGE3_LENGTH, STAGE3_WIDTH, STAGE3_DEPTH)),
        mass=1.0,
        origin=Origin(xyz=(STAGE3_LENGTH / 2.0, 0.0, -STAGE3_DEPTH / 2.0)),
    )

    model.articulation(
        "support_to_stage1",
        ArticulationType.PRISMATIC,
        parent=support,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, -WALL_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAVEL_1, effort=250.0, velocity=0.35),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.0, 0.0, -WALL_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAVEL_2, effort=200.0, velocity=0.35),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=(0.0, 0.0, -WALL_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAVEL_3, effort=150.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    stage3 = object_model.get_part("stage3")

    slide_1 = object_model.get_articulation("support_to_stage1")
    slide_2 = object_model.get_articulation("stage1_to_stage2")
    slide_3 = object_model.get_articulation("stage2_to_stage3")

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

    joints = (slide_1, slide_2, slide_3)
    ctx.check(
        "three_serial_prismatic_joints",
        all(joint.articulation_type == ArticulationType.PRISMATIC for joint in joints),
        "Expected a pure support -> stage1 -> stage2 -> stage3 prismatic chain.",
    )
    ctx.check(
        "all_slides_extend_forward",
        all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in joints),
        "All three slide joints should translate along +X from root to tip.",
    )

    with ctx.pose({slide_1: 0.0, slide_2: 0.0, slide_3: 0.0}):
        ctx.expect_contact(stage1, support, name="stage1_hangs_from_support")
        ctx.expect_contact(stage2, stage1, name="stage2_hangs_from_stage1")
        ctx.expect_contact(stage3, stage2, name="stage3_hangs_from_stage2")

        ctx.expect_within(stage1, support, axes="yz", margin=0.0, name="stage1_nested_under_support")
        ctx.expect_within(stage2, stage1, axes="yz", margin=0.0, name="stage2_nested_under_stage1")
        ctx.expect_within(stage3, stage2, axes="yz", margin=0.0, name="stage3_nested_under_stage2")

        ctx.expect_origin_gap(
            support,
            stage1,
            axis="z",
            min_gap=WALL_T - 0.0002,
            max_gap=WALL_T + 0.0002,
            name="stage1_is_hung_below_support",
        )
        ctx.expect_origin_gap(
            stage1,
            stage2,
            axis="z",
            min_gap=WALL_T - 0.0002,
            max_gap=WALL_T + 0.0002,
            name="stage2_is_hung_below_stage1",
        )
        ctx.expect_origin_gap(
            stage2,
            stage3,
            axis="z",
            min_gap=WALL_T - 0.0002,
            max_gap=WALL_T + 0.0002,
            name="stage3_is_hung_below_stage2",
        )

    with ctx.pose({slide_1: TRAVEL_1, slide_2: TRAVEL_2, slide_3: TRAVEL_3}):
        ctx.expect_contact(stage1, support, name="stage1_remains_captured_when_extended")
        ctx.expect_contact(stage2, stage1, name="stage2_remains_captured_when_extended")
        ctx.expect_contact(stage3, stage2, name="stage3_remains_captured_when_extended")

        ctx.expect_origin_gap(
            stage1,
            support,
            axis="x",
            min_gap=TRAVEL_1 - 0.001,
            max_gap=TRAVEL_1 + 0.001,
            name="stage1_advances_by_first_travel",
        )
        ctx.expect_origin_gap(
            stage2,
            stage1,
            axis="x",
            min_gap=TRAVEL_2 - 0.001,
            max_gap=TRAVEL_2 + 0.001,
            name="stage2_advances_by_second_travel",
        )
        ctx.expect_origin_gap(
            stage3,
            stage2,
            axis="x",
            min_gap=TRAVEL_3 - 0.001,
            max_gap=TRAVEL_3 + 0.001,
            name="stage3_advances_by_third_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
