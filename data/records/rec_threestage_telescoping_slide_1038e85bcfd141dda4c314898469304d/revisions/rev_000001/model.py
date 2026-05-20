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


OUTER_LENGTH = 0.82
OUTER_WIDTH = 0.065
OUTER_BASE_THICKNESS = 0.004
OUTER_WALL_THICKNESS = 0.004
OUTER_WALL_HEIGHT = 0.024

MIDDLE_LENGTH = 0.64
MIDDLE_BODY_WIDTH = 0.034
MIDDLE_BODY_THICKNESS = 0.010
MIDDLE_RUNNER_LENGTH = 0.54
MIDDLE_RUNNER_WIDTH = 0.008
MIDDLE_RUNNER_THICKNESS = 0.004
MIDDLE_HEIGHT = MIDDLE_RUNNER_THICKNESS + MIDDLE_BODY_THICKNESS

INNER_LENGTH = 0.48
INNER_BODY_WIDTH = 0.026
INNER_BODY_THICKNESS = 0.0095
INNER_RUNNER_LENGTH = 0.40
INNER_RUNNER_WIDTH = 0.006
INNER_RUNNER_THICKNESS = 0.0035
INNER_HEIGHT = INNER_RUNNER_THICKNESS + INNER_BODY_THICKNESS

CARRIAGE_LENGTH = 0.085
CARRIAGE_WIDTH = 0.050
CARRIAGE_BLOCK_THICKNESS = 0.013
CARRIAGE_FOOT_LENGTH = 0.055
CARRIAGE_FOOT_WIDTH = 0.024
CARRIAGE_FOOT_THICKNESS = 0.004

OUTER_TO_MIDDLE_X = 0.10
MIDDLE_TO_INNER_X = 0.09
CARRIAGE_ON_INNER_X = INNER_LENGTH - 0.060

MIDDLE_TRAVEL = 0.38
INNER_TRAVEL = 0.30
STAGE_CLEARANCE = 0.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_extension_rail")

    outer_mat = model.material("outer_steel", rgba=(0.46, 0.48, 0.51, 1.0))
    middle_mat = model.material("middle_zinc", rgba=(0.72, 0.74, 0.77, 1.0))
    inner_mat = model.material("inner_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    carriage_mat = model.material("carriage_black", rgba=(0.14, 0.15, 0.16, 1.0))

    outer = model.part("outer_member")
    outer.visual(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_BASE_THICKNESS)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_BASE_THICKNESS / 2.0)),
        material=outer_mat,
        name="outer_base",
    )
    outer.visual(
        Box((OUTER_LENGTH, OUTER_WALL_THICKNESS, OUTER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                -(OUTER_WIDTH / 2.0 - OUTER_WALL_THICKNESS / 2.0),
                OUTER_WALL_HEIGHT / 2.0,
            )
        ),
        material=outer_mat,
        name="outer_left_wall",
    )
    outer.visual(
        Box((OUTER_LENGTH, OUTER_WALL_THICKNESS, OUTER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                OUTER_WIDTH / 2.0 - OUTER_WALL_THICKNESS / 2.0,
                OUTER_WALL_HEIGHT / 2.0,
            )
        ),
        material=outer_mat,
        name="outer_right_wall",
    )

    middle = model.part("middle_member")
    middle.visual(
        Box((MIDDLE_LENGTH, MIDDLE_BODY_WIDTH, MIDDLE_BODY_THICKNESS)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                0.0,
                MIDDLE_RUNNER_THICKNESS + MIDDLE_BODY_THICKNESS / 2.0,
            )
        ),
        material=middle_mat,
        name="middle_body",
    )
    middle.visual(
        Box((MIDDLE_RUNNER_LENGTH, MIDDLE_RUNNER_WIDTH, MIDDLE_RUNNER_THICKNESS)),
        origin=Origin(
            xyz=(MIDDLE_LENGTH / 2.0, -0.0105, MIDDLE_RUNNER_THICKNESS / 2.0)
        ),
        material=middle_mat,
        name="middle_left_runner",
    )
    middle.visual(
        Box((MIDDLE_RUNNER_LENGTH, MIDDLE_RUNNER_WIDTH, MIDDLE_RUNNER_THICKNESS)),
        origin=Origin(
            xyz=(MIDDLE_LENGTH / 2.0, 0.0105, MIDDLE_RUNNER_THICKNESS / 2.0)
        ),
        material=middle_mat,
        name="middle_right_runner",
    )

    inner = model.part("inner_member")
    inner.visual(
        Box((INNER_LENGTH, INNER_BODY_WIDTH, INNER_BODY_THICKNESS)),
        origin=Origin(
            xyz=(
                INNER_LENGTH / 2.0,
                0.0,
                INNER_RUNNER_THICKNESS + INNER_BODY_THICKNESS / 2.0,
            )
        ),
        material=inner_mat,
        name="inner_body",
    )
    inner.visual(
        Box((INNER_RUNNER_LENGTH, INNER_RUNNER_WIDTH, INNER_RUNNER_THICKNESS)),
        origin=Origin(
            xyz=(INNER_LENGTH / 2.0, -0.0085, INNER_RUNNER_THICKNESS / 2.0)
        ),
        material=inner_mat,
        name="inner_left_runner",
    )
    inner.visual(
        Box((INNER_RUNNER_LENGTH, INNER_RUNNER_WIDTH, INNER_RUNNER_THICKNESS)),
        origin=Origin(
            xyz=(INNER_LENGTH / 2.0, 0.0085, INNER_RUNNER_THICKNESS / 2.0)
        ),
        material=inner_mat,
        name="inner_right_runner",
    )
    inner.visual(
        Box((0.095, INNER_BODY_WIDTH, 0.004)),
        origin=Origin(xyz=(INNER_LENGTH - 0.070, 0.0, INNER_HEIGHT - 0.002)),
        material=inner_mat,
        name="inner_top_pad",
    )

    carriage = model.part("top_carriage")
    carriage.visual(
        Box((CARRIAGE_FOOT_LENGTH, CARRIAGE_FOOT_WIDTH, CARRIAGE_FOOT_THICKNESS)),
        origin=Origin(
            xyz=(
                CARRIAGE_LENGTH / 2.0,
                0.0,
                CARRIAGE_FOOT_THICKNESS / 2.0,
            )
        ),
        material=carriage_mat,
        name="carriage_foot",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BLOCK_THICKNESS)),
        origin=Origin(
            xyz=(
                CARRIAGE_LENGTH / 2.0,
                0.0,
                CARRIAGE_FOOT_THICKNESS + CARRIAGE_BLOCK_THICKNESS / 2.0,
            )
        ),
        material=carriage_mat,
        name="carriage_block",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_X, 0.0, OUTER_BASE_THICKNESS + STAGE_CLEARANCE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.60,
            lower=0.0,
            upper=MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_X, 0.0, MIDDLE_HEIGHT + STAGE_CLEARANCE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.60,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_carriage",
        ArticulationType.FIXED,
        parent=inner,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_ON_INNER_X, 0.0, INNER_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    outer = object_model.get_part("outer_member")
    middle = object_model.get_part("middle_member")
    inner = object_model.get_part("inner_member")
    carriage = object_model.get_part("top_carriage")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "outer_to_middle is prismatic on +x",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in outer_to_middle.axis) == (1.0, 0.0, 0.0),
        f"type={outer_to_middle.articulation_type} axis={outer_to_middle.axis}",
    )
    ctx.check(
        "middle_to_inner is prismatic on +x",
        middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in middle_to_inner.axis) == (1.0, 0.0, 0.0),
        f"type={middle_to_inner.articulation_type} axis={middle_to_inner.axis}",
    )

    ctx.expect_contact(
        outer,
        middle,
        elem_a="outer_base",
        elem_b="middle_left_runner",
        contact_tol=1e-6,
        name="middle left runner is seated on outer base",
    )
    ctx.expect_contact(
        outer,
        middle,
        elem_a="outer_base",
        elem_b="middle_right_runner",
        contact_tol=1e-6,
        name="middle right runner is seated on outer base",
    )
    ctx.expect_contact(
        middle,
        inner,
        elem_a="middle_body",
        elem_b="inner_left_runner",
        contact_tol=1e-6,
        name="inner left runner is seated on middle body",
    )
    ctx.expect_contact(
        middle,
        inner,
        elem_a="middle_body",
        elem_b="inner_right_runner",
        contact_tol=1e-6,
        name="inner right runner is seated on middle body",
    )
    ctx.expect_contact(
        inner,
        carriage,
        elem_a="inner_top_pad",
        elem_b="carriage_foot",
        contact_tol=1e-6,
        name="carriage foot contacts inner top pad",
    )

    ctx.expect_within(
        middle,
        outer,
        axes="y",
        margin=0.0005,
        name="middle remains laterally within outer member",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="y",
        margin=0.0005,
        name="inner remains laterally within middle member",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.54,
        name="closed outer and middle have long overlap",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.39,
        name="closed middle and inner have long overlap",
    )

    with ctx.pose({outer_to_middle: 0.36, middle_to_inner: 0.28}):
        ctx.expect_within(
            middle,
            outer,
            axes="y",
            margin=0.0005,
            name="extended middle stays laterally guided",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="y",
            margin=0.0005,
            name="extended inner stays laterally guided",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.18,
            name="outer and middle keep stage overlap when extended",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.11,
            name="middle and inner keep stage overlap when extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
