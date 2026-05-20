from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.60
OUTER_WIDTH = 0.072
OUTER_WALL = 0.0045
OUTER_FLOOR = 0.006
OUTER_SIDE_HEIGHT = 0.041
OUTER_BEARING_HEIGHT = 0.003
OUTER_BEARING_WIDTH = 0.008
OUTER_BEARING_Y = 0.018

MIDDLE_LENGTH = 0.46
MIDDLE_WIDTH = 0.054
MIDDLE_WALL = 0.0035
MIDDLE_FLOOR = 0.005
MIDDLE_SIDE_HEIGHT = 0.026
MIDDLE_BEARING_HEIGHT = 0.0025
MIDDLE_BEARING_WIDTH = 0.006
MIDDLE_BEARING_Y = 0.011

INNER_LENGTH = 0.34
INNER_WIDTH = 0.036
INNER_HEIGHT = 0.015

BASE_PLATE_LENGTH = 0.58
BASE_PLATE_WIDTH = 0.084
BASE_PLATE_THICKNESS = 0.008

GUIDE_TO_MIDDLE_TRAVEL = 0.22
MIDDLE_TO_INNER_TRAVEL = 0.18


def _outer_guide_shape():
    guide_floor_bottom = -(OUTER_FLOOR + OUTER_BEARING_HEIGHT)
    guide = (
        cq.Workplane("XY")
        .box(OUTER_LENGTH, OUTER_WIDTH, OUTER_FLOOR, centered=(False, True, False))
        .translate((0.0, 0.0, guide_floor_bottom))
    )

    base_plate = (
        cq.Workplane("XY")
        .box(
            BASE_PLATE_LENGTH,
            BASE_PLATE_WIDTH,
            BASE_PLATE_THICKNESS,
            centered=(False, True, False),
        )
        .translate((0.01, 0.0, guide_floor_bottom - BASE_PLATE_THICKNESS))
    )

    left_wall = (
        cq.Workplane("XY")
        .box(OUTER_LENGTH, OUTER_WALL, OUTER_SIDE_HEIGHT, centered=(False, True, False))
        .translate((0.0, OUTER_WIDTH / 2.0 - OUTER_WALL / 2.0, guide_floor_bottom))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(OUTER_LENGTH, OUTER_WALL, OUTER_SIDE_HEIGHT, centered=(False, True, False))
        .translate((0.0, -(OUTER_WIDTH / 2.0 - OUTER_WALL / 2.0), guide_floor_bottom))
    )

    left_bearing = (
        cq.Workplane("XY")
        .box(0.54, OUTER_BEARING_WIDTH, OUTER_BEARING_HEIGHT, centered=(False, True, False))
        .translate((0.03, OUTER_BEARING_Y, -OUTER_BEARING_HEIGHT))
    )
    right_bearing = (
        cq.Workplane("XY")
        .box(0.54, OUTER_BEARING_WIDTH, OUTER_BEARING_HEIGHT, centered=(False, True, False))
        .translate((0.03, -OUTER_BEARING_Y, -OUTER_BEARING_HEIGHT))
    )

    rear_saddle = (
        cq.Workplane("XY")
        .box(0.060, 0.040, 0.012, centered=(False, True, False))
        .translate((0.03, 0.0, guide_floor_bottom - 0.012))
    )
    front_saddle = (
        cq.Workplane("XY")
        .box(0.060, 0.040, 0.012, centered=(False, True, False))
        .translate((0.49, 0.0, guide_floor_bottom - 0.012))
    )

    return (
        guide.union(base_plate)
        .union(left_wall)
        .union(right_wall)
        .union(left_bearing)
        .union(right_bearing)
        .union(rear_saddle)
        .union(front_saddle)
    )


def _middle_runner_shape():
    runner = cq.Workplane("XY").box(
        MIDDLE_LENGTH,
        MIDDLE_WIDTH,
        MIDDLE_FLOOR,
        centered=(False, True, False),
    )
    left_wall = (
        cq.Workplane("XY")
        .box(MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_SIDE_HEIGHT, centered=(False, True, False))
        .translate((0.0, MIDDLE_WIDTH / 2.0 - MIDDLE_WALL / 2.0, 0.0))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_SIDE_HEIGHT, centered=(False, True, False))
        .translate((0.0, -(MIDDLE_WIDTH / 2.0 - MIDDLE_WALL / 2.0), 0.0))
    )
    left_bearing = (
        cq.Workplane("XY")
        .box(0.40, MIDDLE_BEARING_WIDTH, MIDDLE_BEARING_HEIGHT, centered=(False, True, False))
        .translate((0.03, MIDDLE_BEARING_Y, MIDDLE_FLOOR))
    )
    right_bearing = (
        cq.Workplane("XY")
        .box(0.40, MIDDLE_BEARING_WIDTH, MIDDLE_BEARING_HEIGHT, centered=(False, True, False))
        .translate((0.03, -MIDDLE_BEARING_Y, MIDDLE_FLOOR))
    )

    nose_band = (
        cq.Workplane("XY")
        .box(0.022, MIDDLE_WIDTH, 0.010, centered=(False, True, False))
        .translate((MIDDLE_LENGTH - 0.022, 0.0, 0.0))
    )

    return runner.union(left_wall).union(right_wall).union(left_bearing).union(right_bearing).union(
        nose_band
    )


def _inner_runner_shape():
    bar = cq.Workplane("XY").box(
        INNER_LENGTH,
        INNER_WIDTH,
        INNER_HEIGHT,
        centered=(False, True, False),
    )
    carriage_pad = (
        cq.Workplane("XY")
        .box(0.100, INNER_WIDTH * 0.78, 0.003, centered=(False, True, False))
        .translate((INNER_LENGTH - 0.125, 0.0, INNER_HEIGHT))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.025, INNER_WIDTH * 0.90, INNER_HEIGHT * 0.90, centered=(False, True, False))
        .translate((INNER_LENGTH - 0.025, 0.0, 0.0))
    )

    return bar.union(carriage_pad).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_runner_two_stage_slide_chain")

    model.material("graphite_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("runner_gray", rgba=(0.56, 0.59, 0.63, 1.0))
    model.material("machined_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    guide_section = model.part("guide_section")
    guide_section.visual(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_FLOOR)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, -(OUTER_BEARING_HEIGHT + OUTER_FLOOR / 2.0))),
        material="graphite_steel",
        name="guide_floor",
    )
    guide_section.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                OUTER_WIDTH / 2.0 - OUTER_WALL / 2.0,
                -(OUTER_BEARING_HEIGHT + OUTER_FLOOR) + OUTER_SIDE_HEIGHT / 2.0,
            )
        ),
        material="graphite_steel",
        name="guide_left_wall",
    )
    guide_section.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                -(OUTER_WIDTH / 2.0 - OUTER_WALL / 2.0),
                -(OUTER_BEARING_HEIGHT + OUTER_FLOOR) + OUTER_SIDE_HEIGHT / 2.0,
            )
        ),
        material="graphite_steel",
        name="guide_right_wall",
    )
    guide_section.visual(
        Box((0.54, OUTER_BEARING_WIDTH, OUTER_BEARING_HEIGHT)),
        origin=Origin(xyz=(0.30, OUTER_BEARING_Y, -OUTER_BEARING_HEIGHT / 2.0)),
        material="graphite_steel",
        name="guide_left_bearing",
    )
    guide_section.visual(
        Box((0.54, OUTER_BEARING_WIDTH, OUTER_BEARING_HEIGHT)),
        origin=Origin(xyz=(0.30, -OUTER_BEARING_Y, -OUTER_BEARING_HEIGHT / 2.0)),
        material="graphite_steel",
        name="guide_right_bearing",
    )
    guide_section.visual(
        Box((BASE_PLATE_LENGTH, BASE_PLATE_WIDTH, BASE_PLATE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.01 + BASE_PLATE_LENGTH / 2.0,
                0.0,
                -(OUTER_BEARING_HEIGHT + OUTER_FLOOR + BASE_PLATE_THICKNESS / 2.0),
            )
        ),
        material="graphite_steel",
        name="base_plate",
    )
    guide_section.visual(
        Box((0.060, 0.040, 0.012)),
        origin=Origin(
            xyz=(0.06, 0.0, -(OUTER_BEARING_HEIGHT + OUTER_FLOOR + 0.012 / 2.0))
        ),
        material="graphite_steel",
        name="rear_saddle",
    )
    guide_section.visual(
        Box((0.060, 0.040, 0.012)),
        origin=Origin(
            xyz=(0.52, 0.0, -(OUTER_BEARING_HEIGHT + OUTER_FLOOR + 0.012 / 2.0))
        ),
        material="graphite_steel",
        name="front_saddle",
    )

    middle_runner = model.part("middle_runner")
    middle_runner.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_FLOOR)),
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_FLOOR / 2.0)),
        material="runner_gray",
        name="middle_floor",
    )
    middle_runner.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                MIDDLE_WIDTH / 2.0 - MIDDLE_WALL / 2.0,
                MIDDLE_SIDE_HEIGHT / 2.0,
            )
        ),
        material="runner_gray",
        name="middle_left_wall",
    )
    middle_runner.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                -(MIDDLE_WIDTH / 2.0 - MIDDLE_WALL / 2.0),
                MIDDLE_SIDE_HEIGHT / 2.0,
            )
        ),
        material="runner_gray",
        name="middle_right_wall",
    )
    middle_runner.visual(
        Box((0.40, MIDDLE_BEARING_WIDTH, MIDDLE_BEARING_HEIGHT)),
        origin=Origin(
            xyz=(0.23, MIDDLE_BEARING_Y, MIDDLE_FLOOR + MIDDLE_BEARING_HEIGHT / 2.0)
        ),
        material="runner_gray",
        name="middle_left_bearing",
    )
    middle_runner.visual(
        Box((0.40, MIDDLE_BEARING_WIDTH, MIDDLE_BEARING_HEIGHT)),
        origin=Origin(
            xyz=(0.23, -MIDDLE_BEARING_Y, MIDDLE_FLOOR + MIDDLE_BEARING_HEIGHT / 2.0)
        ),
        material="runner_gray",
        name="middle_right_bearing",
    )
    middle_runner.visual(
        Box((0.022, MIDDLE_WIDTH, 0.010)),
        origin=Origin(xyz=(MIDDLE_LENGTH - 0.011, 0.0, 0.005)),
        material="runner_gray",
        name="middle_front_bridge",
    )

    inner_runner = model.part("inner_runner")
    inner_runner.visual(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0)),
        material="machined_steel",
        name="inner_bar",
    )
    inner_runner.visual(
        Box((0.100, INNER_WIDTH * 0.78, 0.003)),
        origin=Origin(xyz=(INNER_LENGTH - 0.075, 0.0, INNER_HEIGHT + 0.0015)),
        material="machined_steel",
        name="inner_pad",
    )
    inner_runner.visual(
        Box((0.025, INNER_WIDTH * 0.90, INNER_HEIGHT * 0.90)),
        origin=Origin(
            xyz=(INNER_LENGTH - 0.0125, 0.0, INNER_HEIGHT * 0.90 / 2.0)
        ),
        material="machined_steel",
        name="inner_nose",
    )

    model.articulation(
        "guide_to_middle",
        ArticulationType.PRISMATIC,
        parent=guide_section,
        child=middle_runner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=GUIDE_TO_MIDDLE_TRAVEL,
            effort=120.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_runner,
        child=inner_runner,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_FLOOR + MIDDLE_BEARING_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
            effort=90.0,
            velocity=0.35,
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

    guide_section = object_model.get_part("guide_section")
    middle_runner = object_model.get_part("middle_runner")
    inner_runner = object_model.get_part("inner_runner")
    guide_to_middle = object_model.get_articulation("guide_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "serial stages use shared +X slide axis",
        tuple(guide_to_middle.axis) == (1.0, 0.0, 0.0)
        and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        details=f"guide_to_middle={guide_to_middle.axis}, middle_to_inner={middle_to_inner.axis}",
    )

    ctx.expect_within(
        middle_runner,
        guide_section,
        axes="yz",
        margin=0.001,
        name="middle runner stays centered inside guide section at rest",
    )
    ctx.expect_overlap(
        middle_runner,
        guide_section,
        axes="x",
        min_overlap=0.40,
        name="middle runner has substantial retained insertion at rest",
    )
    ctx.expect_within(
        inner_runner,
        middle_runner,
        axes="yz",
        margin=0.0015,
        name="inner runner stays centered inside middle runner at rest",
    )
    ctx.expect_overlap(
        inner_runner,
        middle_runner,
        axes="x",
        min_overlap=0.30,
        name="inner runner has substantial retained insertion at rest",
    )

    middle_rest = ctx.part_world_position(middle_runner)
    inner_rest = ctx.part_world_position(inner_runner)

    with ctx.pose({guide_to_middle: GUIDE_TO_MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle_runner,
            guide_section,
            axes="yz",
            margin=0.001,
            name="extended middle runner remains guided by outer section",
        )
        ctx.expect_overlap(
            middle_runner,
            guide_section,
            axes="x",
            min_overlap=0.36,
            name="extended middle runner still remains inserted in guide section",
        )
        middle_extended = ctx.part_world_position(middle_runner)

    with ctx.pose(
        {
            guide_to_middle: GUIDE_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_within(
            inner_runner,
            middle_runner,
            axes="yz",
            margin=0.0015,
            name="extended inner runner remains guided by middle runner",
        )
        ctx.expect_within(
            inner_runner,
            guide_section,
            axes="yz",
            margin=0.004,
            name="entire slide chain stays on one shared guide axis",
        )
        ctx.expect_overlap(
            inner_runner,
            middle_runner,
            axes="x",
            min_overlap=0.27,
            name="extended inner runner still remains inserted in middle runner",
        )
        inner_extended = ctx.part_world_position(inner_runner)

    ctx.check(
        "middle runner extends in the positive slide direction",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.10,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "inner runner extends farther when both stages deploy",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.30,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
