from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GUIDE_LENGTH = 0.36
GUIDE_OUTER_WIDTH = 0.060
GUIDE_OUTER_HEIGHT = 0.050
GUIDE_WALL_THICKNESS = 0.006
GUIDE_BASE_THICKNESS = 0.006
GUIDE_INNER_WIDTH = GUIDE_OUTER_WIDTH - 2.0 * GUIDE_WALL_THICKNESS
GUIDE_INNER_HEIGHT = GUIDE_OUTER_HEIGHT - GUIDE_BASE_THICKNESS

RUNNER_BODY_LENGTH = 0.160
RUNNER_BODY_WIDTH = 0.040
RUNNER_BODY_HEIGHT = 0.022
RUNNER_BODY_BASE_Z = GUIDE_BASE_THICKNESS

RUNNER_STEM_LENGTH = 0.010
RUNNER_STEM_WIDTH = 0.020
RUNNER_STEM_HEIGHT = 0.014
RUNNER_STEM_BASE_Z = 0.010

OUTPUT_PLATE_THICKNESS = 0.006
OUTPUT_PLATE_WIDTH = 0.052
OUTPUT_PLATE_HEIGHT = 0.068
OUTPUT_PLATE_BASE_Z = 0.002
OUTPUT_PLATE_CENTER_X = 0.011

SLIDE_TRAVEL = 0.110
def _build_runner_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            RUNNER_BODY_LENGTH,
            RUNNER_BODY_WIDTH,
            RUNNER_BODY_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-RUNNER_BODY_LENGTH / 2.0, 0.0, RUNNER_BODY_BASE_Z))
    )
    stem = (
        cq.Workplane("XY")
        .box(
            RUNNER_STEM_LENGTH,
            RUNNER_STEM_WIDTH,
            RUNNER_STEM_HEIGHT,
            centered=(True, True, False),
        )
        .translate((RUNNER_STEM_LENGTH / 2.0, 0.0, RUNNER_STEM_BASE_Z))
    )
    return body.union(stem)


def _build_output_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            OUTPUT_PLATE_THICKNESS,
            OUTPUT_PLATE_WIDTH,
            OUTPUT_PLATE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((OUTPUT_PLATE_CENTER_X, 0.0, OUTPUT_PLATE_BASE_Z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="channel_runner_slide_module")

    model.material("guide_aluminum", rgba=(0.56, 0.60, 0.64, 1.0))
    model.material("runner_black", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("plate_steel", rgba=(0.79, 0.81, 0.84, 1.0))

    guide = model.part("channel_guide")
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_OUTER_WIDTH, GUIDE_BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_BASE_THICKNESS / 2.0)),
        material="guide_aluminum",
        name="guide_base",
    )
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_WALL_THICKNESS, GUIDE_INNER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                GUIDE_OUTER_WIDTH / 2.0 - GUIDE_WALL_THICKNESS / 2.0,
                GUIDE_BASE_THICKNESS + GUIDE_INNER_HEIGHT / 2.0,
            )
        ),
        material="guide_aluminum",
        name="guide_left_wall",
    )
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_WALL_THICKNESS, GUIDE_INNER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -GUIDE_OUTER_WIDTH / 2.0 + GUIDE_WALL_THICKNESS / 2.0,
                GUIDE_BASE_THICKNESS + GUIDE_INNER_HEIGHT / 2.0,
            )
        ),
        material="guide_aluminum",
        name="guide_right_wall",
    )
    guide.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, GUIDE_OUTER_WIDTH, GUIDE_OUTER_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_OUTER_HEIGHT / 2.0)),
    )

    runner = model.part("runner")
    runner.visual(
        mesh_from_cadquery(_build_runner_body_shape(), "runner_body"),
        material="runner_black",
        name="runner_body",
    )
    runner.visual(
        mesh_from_cadquery(_build_output_plate_shape(), "runner_output_plate"),
        material="plate_steel",
        name="output_plate",
    )
    runner.inertial = Inertial.from_geometry(
        Box((0.174, OUTPUT_PLATE_WIDTH, OUTPUT_PLATE_HEIGHT)),
        mass=0.42,
        origin=Origin(xyz=(-0.073, 0.0, OUTPUT_PLATE_BASE_Z + OUTPUT_PLATE_HEIGHT / 2.0)),
    )

    model.articulation(
        "guide_to_runner",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=runner,
        origin=Origin(xyz=(GUIDE_LENGTH / 2.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=180.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    guide = object_model.get_part("channel_guide")
    runner = object_model.get_part("runner")
    slide = object_model.get_articulation("guide_to_runner")
    runner_body = runner.get_visual("runner_body")
    output_plate = runner.get_visual("output_plate")

    ctx.expect_within(
        runner,
        guide,
        axes="yz",
        inner_elem=runner_body,
        margin=0.0,
        name="runner body stays centered within guide envelope",
    )
    ctx.expect_overlap(
        runner,
        guide,
        axes="x",
        elem_a=runner_body,
        min_overlap=0.150,
        name="collapsed runner remains deeply inserted in the guide",
    )
    ctx.expect_gap(
        runner,
        guide,
        axis="x",
        positive_elem=output_plate,
        min_gap=0.006,
        name="output plate starts beyond the guide end",
    )

    rest_position = ctx.part_world_position(runner)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            runner,
            guide,
            axes="yz",
            inner_elem=runner_body,
            margin=0.0,
            name="extended runner body stays laterally aligned with the guide",
        )
        ctx.expect_overlap(
            runner,
            guide,
            axes="x",
            elem_a=runner_body,
            min_overlap=0.045,
            name="extended runner still retains insertion in the guide",
        )
        ctx.expect_gap(
            runner,
            guide,
            axis="x",
            positive_elem=output_plate,
            min_gap=0.110,
            name="output plate advances outward when extended",
        )
        extended_position = ctx.part_world_position(runner)

    ctx.check(
        "runner extends along +X",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.10,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
