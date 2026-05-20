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
    mesh_from_cadquery,
)


OUTER_LENGTH = 0.46
OUTER_WIDTH = 0.11
OUTER_HEIGHT = 0.06
OUTER_SIDE_T = 0.004
OUTER_BASE_T = 0.006
OUTER_FOOT_T = 0.008
OUTER_FOOT_WIDTH = OUTER_WIDTH
OUTER_FOOT_LENGTH = OUTER_LENGTH

MIDDLE_LENGTH = 0.34
MIDDLE_WIDTH = 0.064
MIDDLE_HEIGHT = 0.034
MIDDLE_SIDE_T = 0.0035
MIDDLE_BASE_T = 0.004
MIDDLE_RUNNER_WIDTH = 0.010
MIDDLE_RUNNER_CENTER_Y = 0.038
MIDDLE_REAR_BRIDGE_WIDTH = 0.066

OUTPUT_TOTAL_LENGTH = 0.18
OUTPUT_RAIL_LENGTH = 0.172
OUTPUT_BAR_WIDTH = 0.028
OUTPUT_BAR_HEIGHT = 0.020
OUTPUT_BASE_WIDTH = 0.046
OUTPUT_BASE_HEIGHT = 0.006
OUTPUT_SHOE_LENGTH = 0.152
OUTPUT_SHOE_WIDTH = 0.010
OUTPUT_SHOE_CENTER_Y = 0.028
END_PLATE_T = 0.008
END_PLATE_WIDTH = 0.078
END_PLATE_HEIGHT = 0.056
END_PLATE_OVERLAP = 0.0

OUTER_TO_MIDDLE_HOME = OUTER_LENGTH - MIDDLE_LENGTH
MIDDLE_TO_OUTPUT_HOME = 0.17
OUTER_TRAVEL = 0.18
OUTPUT_TRAVEL = 0.08

OUTER_TO_MIDDLE_Z = 0.014
MIDDLE_TO_OUTPUT_Z = 0.014

_EPS = 0.001


def _channel_shape(
    *,
    length: float,
    width: float,
    height: float,
    side_t: float,
    base_t: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(length, width, height, centered=(False, True, False))
    cavity = (
        cq.Workplane("XY")
        .box(
            length + 2.0 * _EPS,
            width - 2.0 * side_t,
            height - base_t + _EPS,
            centered=(False, True, False),
        )
        .translate((-_EPS, 0.0, base_t))
    )
    return outer.cut(cavity)


def _outer_guide_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(
            OUTER_FOOT_LENGTH,
            OUTER_FOOT_WIDTH,
            OUTER_FOOT_T,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, -OUTER_FOOT_T))
    )
    left_rail = (
        cq.Workplane("XY")
        .box(
            OUTER_LENGTH,
            0.012,
            OUTER_HEIGHT,
            centered=(False, True, False),
        )
        .translate((0.0, -0.049, 0.0))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(
            OUTER_LENGTH,
            0.012,
            OUTER_HEIGHT,
            centered=(False, True, False),
        )
        .translate((0.0, 0.049, 0.0))
    )
    left_ledge = (
        cq.Workplane("XY")
        .box(OUTER_LENGTH, 0.008, 0.006, centered=(False, True, False))
        .translate((0.0, -0.039, 0.008))
    )
    right_ledge = (
        cq.Workplane("XY")
        .box(OUTER_LENGTH, 0.008, 0.006, centered=(False, True, False))
        .translate((0.0, 0.039, 0.008))
    )
    rear_stop = (
        cq.Workplane("XY")
        .box(0.02, 0.086, 0.022, centered=(False, True, False))
        .translate((0.0, 0.0, 0.014))
    )
    mount_holes = (
        cq.Workplane("XY")
        .pushPoints([(0.095, 0.0), (OUTER_LENGTH - 0.095, 0.0)])
        .circle(0.0055)
        .extrude(OUTER_HEIGHT + OUTER_FOOT_T + 0.004)
        .translate((0.0, 0.0, -OUTER_FOOT_T - 0.002))
    )
    return (
        base.union(left_rail)
        .union(right_rail)
        .union(left_ledge)
        .union(right_ledge)
        .union(rear_stop)
        .cut(mount_holes)
    )


def _middle_stage_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_BASE_T, centered=(False, True, False))
    )
    left_rail = (
        cq.Workplane("XY")
        .box(MIDDLE_LENGTH, 0.010, MIDDLE_HEIGHT - 0.006, centered=(False, True, False))
        .translate((0.0, -0.033, 0.006))
    )
    right_rail = (
        cq.Workplane("XY")
        .box(MIDDLE_LENGTH, 0.010, MIDDLE_HEIGHT - 0.006, centered=(False, True, False))
        .translate((0.0, 0.033, 0.006))
    )
    left_ledge = (
        cq.Workplane("XY")
        .box(MIDDLE_LENGTH, 0.007, 0.006, centered=(False, True, False))
        .translate((0.0, -0.024, 0.006))
    )
    right_ledge = (
        cq.Workplane("XY")
        .box(MIDDLE_LENGTH, 0.007, 0.006, centered=(False, True, False))
        .translate((0.0, 0.024, 0.006))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.018, MIDDLE_WIDTH, 0.022, centered=(False, True, False))
        .translate((0.0, 0.0, MIDDLE_BASE_T))
    )
    return base.union(left_rail).union(right_rail).union(left_ledge).union(right_ledge).union(rear_bridge)


def _output_rail_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(
            OUTPUT_RAIL_LENGTH,
            OUTPUT_BASE_WIDTH,
            OUTPUT_BASE_HEIGHT,
            centered=(False, True, False),
        )
    )
    bar = (
        cq.Workplane("XY")
        .box(
            OUTPUT_RAIL_LENGTH,
            OUTPUT_BAR_WIDTH,
            OUTPUT_BAR_HEIGHT,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, OUTPUT_BASE_HEIGHT))
    )
    return base.union(bar)


def _end_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(END_PLATE_T, END_PLATE_WIDTH, END_PLATE_HEIGHT, centered=(False, True, False))
        .translate((OUTPUT_RAIL_LENGTH - END_PLATE_OVERLAP, 0.0, 0.0))
    )
    holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.022, 0.018),
                (0.022, 0.018),
                (-0.022, 0.042),
                (0.022, 0.042),
            ]
        )
        .circle(0.0042)
        .extrude(END_PLATE_T + 2.0 * _EPS)
        .translate((OUTPUT_RAIL_LENGTH - END_PLATE_OVERLAP - _EPS, 0.0, 0.0))
    )
    return plate.cut(holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_slide_pair")

    model.material("outer_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("middle_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("output_alloy", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("plate_finish", rgba=(0.16, 0.18, 0.20, 1.0))

    outer = model.part("outer_guide")
    outer.visual(
        Box((OUTER_FOOT_LENGTH, OUTER_FOOT_WIDTH, OUTER_FOOT_T)),
        origin=Origin(xyz=(OUTER_FOOT_LENGTH / 2.0, 0.0, -OUTER_FOOT_T / 2.0)),
        material="outer_steel",
        name="elem_outer_guide",
    )
    outer.visual(
        Box((OUTER_LENGTH, 0.012, 0.056)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, -0.049, 0.028)),
        material="outer_steel",
        name="elem_outer_left_rail",
    )
    outer.visual(
        Box((OUTER_LENGTH, 0.012, 0.056)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.049, 0.028)),
        material="outer_steel",
        name="elem_outer_right_rail",
    )
    outer.visual(
        Box((0.022, 0.086, 0.024)),
        origin=Origin(xyz=(0.011, 0.0, 0.012)),
        material="outer_steel",
        name="elem_outer_rear_bridge",
    )

    middle = model.part("middle_stage")
    middle.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_BASE_T)),
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_BASE_T / 2.0)),
        material="middle_steel",
        name="elem_middle_stage",
    )
    middle.visual(
        Box((MIDDLE_LENGTH - 0.018, MIDDLE_RUNNER_WIDTH, MIDDLE_HEIGHT)),
        origin=Origin(xyz=(0.179, -MIDDLE_RUNNER_CENTER_Y, MIDDLE_HEIGHT / 2.0)),
        material="middle_steel",
        name="elem_middle_left_runner",
    )
    middle.visual(
        Box((MIDDLE_LENGTH - 0.018, MIDDLE_RUNNER_WIDTH, MIDDLE_HEIGHT)),
        origin=Origin(xyz=(0.179, MIDDLE_RUNNER_CENTER_Y, MIDDLE_HEIGHT / 2.0)),
        material="middle_steel",
        name="elem_middle_right_runner",
    )
    middle.visual(
        Box((0.018, MIDDLE_REAR_BRIDGE_WIDTH, 0.022)),
        origin=Origin(xyz=(0.009, 0.0, 0.011)),
        material="middle_steel",
        name="elem_middle_rear_bridge",
    )

    output = model.part("output_stage")
    output.visual(
        Box((OUTPUT_RAIL_LENGTH, OUTPUT_BASE_WIDTH, OUTPUT_BAR_HEIGHT)),
        origin=Origin(xyz=(OUTPUT_RAIL_LENGTH / 2.0, 0.0, OUTPUT_BAR_HEIGHT / 2.0)),
        material="output_alloy",
        name="elem_output_rail",
    )
    output.visual(
        Box((OUTPUT_SHOE_LENGTH, OUTPUT_SHOE_WIDTH, OUTPUT_BAR_HEIGHT)),
        origin=Origin(
            xyz=(OUTPUT_SHOE_LENGTH / 2.0, -OUTPUT_SHOE_CENTER_Y, OUTPUT_BAR_HEIGHT / 2.0)
        ),
        material="output_alloy",
        name="elem_output_left_shoe",
    )
    output.visual(
        Box((OUTPUT_SHOE_LENGTH, OUTPUT_SHOE_WIDTH, OUTPUT_BAR_HEIGHT)),
        origin=Origin(
            xyz=(OUTPUT_SHOE_LENGTH / 2.0, OUTPUT_SHOE_CENTER_Y, OUTPUT_BAR_HEIGHT / 2.0)
        ),
        material="output_alloy",
        name="elem_output_right_shoe",
    )
    output.visual(
        Box((END_PLATE_T, END_PLATE_WIDTH, END_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTPUT_RAIL_LENGTH + END_PLATE_T / 2.0 - END_PLATE_OVERLAP,
                0.0,
                END_PLATE_HEIGHT / 2.0,
            )
        ),
        material="plate_finish",
        name="elem_end_plate",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_HOME, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_output",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=output,
        origin=Origin(xyz=(MIDDLE_TO_OUTPUT_HOME, 0.0, MIDDLE_TO_OUTPUT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.40,
            lower=0.0,
            upper=OUTPUT_TRAVEL,
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

    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_stage")
    output = object_model.get_part("output_stage")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_output = object_model.get_articulation("middle_to_output")
    outer_left_rail = outer.get_visual("elem_outer_left_rail")
    outer_right_rail = outer.get_visual("elem_outer_right_rail")
    middle_left_runner = middle.get_visual("elem_middle_left_runner")
    middle_right_runner = middle.get_visual("elem_middle_right_runner")
    output_rail = output.get_visual("elem_output_rail")
    output_left_shoe = output.get_visual("elem_output_left_shoe")
    output_right_shoe = output.get_visual("elem_output_right_shoe")

    ctx.expect_contact(
        middle,
        outer,
        elem_a=middle_left_runner,
        elem_b=outer_left_rail,
        name="middle left runner bears on outer left rail",
    )
    ctx.expect_contact(
        middle,
        outer,
        elem_a=middle_right_runner,
        elem_b=outer_right_rail,
        name="middle right runner bears on outer right rail",
    )

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0005,
        name="middle stage stays centered in outer guide",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.30,
        name="middle stage remains deeply inserted when retracted",
    )
    ctx.expect_within(
        output,
        middle,
        axes="yz",
        inner_elem=output_rail,
        margin=0.0005,
        name="output rail stays centered in middle stage",
    )
    ctx.expect_contact(
        output,
        middle,
        elem_a=output_left_shoe,
        elem_b=middle_left_runner,
        name="output left shoe bears on middle left runner",
    )
    ctx.expect_contact(
        output,
        middle,
        elem_a=output_right_shoe,
        elem_b=middle_right_runner,
        name="output right shoe bears on middle right runner",
    )
    ctx.expect_overlap(
        output,
        middle,
        axes="x",
        elem_a=output_rail,
        min_overlap=0.16,
        name="output rail remains inserted when retracted",
    )

    middle_rest = ctx.part_world_position(middle)
    with ctx.pose({outer_to_middle: OUTER_TRAVEL}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0005,
            name="extended middle stage stays aligned in outer guide",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.15,
            name="middle stage retains insertion at full travel",
        )
        middle_extended = ctx.part_world_position(middle)

    ctx.check(
        "middle stage extends along +X",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.12,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )

    output_rest = ctx.part_world_position(output)
    with ctx.pose({middle_to_output: OUTPUT_TRAVEL}):
        ctx.expect_within(
            output,
            middle,
            axes="yz",
            inner_elem=output_rail,
            margin=0.0005,
            name="extended output rail stays aligned in middle stage",
        )
        ctx.expect_overlap(
            output,
            middle,
            axes="x",
            elem_a=output_rail,
            min_overlap=0.089,
            name="output rail retains insertion at full travel",
        )
        ctx.expect_contact(
            output,
            middle,
            elem_a=output_left_shoe,
            elem_b=middle_left_runner,
            name="extended output left shoe stays on middle runner",
        )
        ctx.expect_contact(
            output,
            middle,
            elem_a=output_right_shoe,
            elem_b=middle_right_runner,
            name="extended output right shoe stays on middle runner",
        )
        output_extended = ctx.part_world_position(output)

    ctx.check(
        "output stage extends along +X",
        output_rest is not None
        and output_extended is not None
        and output_extended[0] > output_rest[0] + 0.05,
        details=f"rest={output_rest}, extended={output_extended}",
    )

    with ctx.pose({outer_to_middle: OUTER_TRAVEL, middle_to_output: OUTPUT_TRAVEL}):
        outer_aabb = ctx.part_world_aabb(outer)
        output_aabb = ctx.part_world_aabb(output)
        ctx.check(
            "serial slides project the end plate beyond the grounded guide",
            outer_aabb is not None
            and output_aabb is not None
            and output_aabb[1][0] > outer_aabb[1][0] + 0.10,
            details=f"outer_aabb={outer_aabb}, output_aabb={output_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
