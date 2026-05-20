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


OUTER_LENGTH = 0.50
OUTER_WIDTH = 0.10
OUTER_HEIGHT = 0.08
OUTER_WALL = 0.004
OUTER_FRONT_MOUTH_X = OUTER_LENGTH / 2.0

BASE_PAD_LENGTH = 0.24
BASE_PAD_WIDTH = 0.14
BASE_PAD_THICKNESS = 0.012
BASE_PAD_CENTER_X = -0.10
BASE_PAD_CENTER_Z = -(OUTER_HEIGHT / 2.0 + BASE_PAD_THICKNESS / 2.0) + 0.001

INTERMEDIATE_LENGTH = 0.42
INTERMEDIATE_WIDTH = 0.080
INTERMEDIATE_HEIGHT = 0.060
INTERMEDIATE_WALL = 0.003
INTERMEDIATE_REST_INSERTION = 0.31
INTERMEDIATE_FRONT_PROJECTION = INTERMEDIATE_LENGTH - INTERMEDIATE_REST_INSERTION
INTERMEDIATE_TRAVEL = 0.18
INTERMEDIATE_PAD_LENGTH = 0.036
INTERMEDIATE_PAD_WIDTH = 0.020
INTERMEDIATE_PAD_HEIGHT = OUTER_HEIGHT / 2.0 - OUTER_WALL - INTERMEDIATE_HEIGHT / 2.0
INTERMEDIATE_PAD_X = -0.255

OUTPUT_BODY_REST_INSERTION = 0.24
OUTPUT_BODY_FRONT_PROJECTION = 0.11
OUTPUT_BODY_WIDTH = 0.058
OUTPUT_BODY_HEIGHT = 0.042
OUTPUT_TRAVEL = 0.15
OUTPUT_END_PLATE_THICKNESS = 0.008
OUTPUT_END_PLATE_WIDTH = 0.074
OUTPUT_END_PLATE_HEIGHT = 0.056
OUTPUT_END_PLATE_OVERLAP = 0.001
OUTPUT_PAD_LENGTH = 0.032
OUTPUT_PAD_WIDTH = 0.016
OUTPUT_PAD_HEIGHT = (
    INTERMEDIATE_HEIGHT / 2.0 - INTERMEDIATE_WALL - OUTPUT_BODY_HEIGHT / 2.0
)
OUTPUT_PAD_X = -0.185


def _span_box(
    part,
    *,
    name: str,
    x_min: float,
    x_max: float,
    width: float,
    height: float,
    material: str,
    y: float = 0.0,
    z: float = 0.0,
) -> None:
    length = x_max - x_min
    part.visual(
        Box((length, width, height)),
        origin=Origin(xyz=((x_min + x_max) / 2.0, y, z)),
        material=material,
        name=name,
    )


def _add_u_channel(
    part,
    *,
    prefix: str,
    x_min: float,
    x_max: float,
    width: float,
    height: float,
    wall: float,
    material: str,
) -> None:
    _span_box(
        part,
        name=f"{prefix}_top_wall",
        x_min=x_min,
        x_max=x_max,
        width=width,
        height=wall,
        z=height / 2.0 - wall / 2.0,
        material=material,
    )
    side_height = height - wall
    side_center_z = -wall / 2.0
    side_center_y = width / 2.0 - wall / 2.0
    _span_box(
        part,
        name=f"{prefix}_left_wall",
        x_min=x_min,
        x_max=x_max,
        width=wall,
        height=side_height,
        y=side_center_y,
        z=side_center_z,
        material=material,
    )
    _span_box(
        part,
        name=f"{prefix}_right_wall",
        x_min=x_min,
        x_max=x_max,
        width=wall,
        height=side_height,
        y=-side_center_y,
        z=side_center_z,
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide")

    model.material("outer_finish", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("intermediate_finish", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("output_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("plate_finish", rgba=(0.84, 0.85, 0.87, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    _add_u_channel(
        outer_sleeve,
        prefix="outer",
        x_min=-OUTER_LENGTH / 2.0,
        x_max=OUTER_LENGTH / 2.0,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        material="outer_finish",
    )
    _span_box(
        outer_sleeve,
        name="outer_base_pad",
        x_min=BASE_PAD_CENTER_X - BASE_PAD_LENGTH / 2.0,
        x_max=BASE_PAD_CENTER_X + BASE_PAD_LENGTH / 2.0,
        width=BASE_PAD_WIDTH,
        height=BASE_PAD_THICKNESS,
        z=BASE_PAD_CENTER_Z,
        material="outer_finish",
    )

    intermediate_stage = model.part("intermediate_stage")
    _add_u_channel(
        intermediate_stage,
        prefix="intermediate",
        x_min=-INTERMEDIATE_REST_INSERTION,
        x_max=INTERMEDIATE_FRONT_PROJECTION,
        width=INTERMEDIATE_WIDTH,
        height=INTERMEDIATE_HEIGHT,
        wall=INTERMEDIATE_WALL,
        material="intermediate_finish",
    )
    _span_box(
        intermediate_stage,
        name="intermediate_bearing_pad",
        x_min=INTERMEDIATE_PAD_X - INTERMEDIATE_PAD_LENGTH / 2.0,
        x_max=INTERMEDIATE_PAD_X + INTERMEDIATE_PAD_LENGTH / 2.0,
        width=INTERMEDIATE_PAD_WIDTH,
        height=INTERMEDIATE_PAD_HEIGHT,
        z=INTERMEDIATE_HEIGHT / 2.0 + INTERMEDIATE_PAD_HEIGHT / 2.0,
        material="intermediate_finish",
    )

    output_stage = model.part("output_stage")
    _span_box(
        output_stage,
        name="output_member_body",
        x_min=-OUTPUT_BODY_REST_INSERTION,
        x_max=OUTPUT_BODY_FRONT_PROJECTION,
        width=OUTPUT_BODY_WIDTH,
        height=OUTPUT_BODY_HEIGHT,
        material="output_finish",
    )
    _span_box(
        output_stage,
        name="output_bearing_pad",
        x_min=OUTPUT_PAD_X - OUTPUT_PAD_LENGTH / 2.0,
        x_max=OUTPUT_PAD_X + OUTPUT_PAD_LENGTH / 2.0,
        width=OUTPUT_PAD_WIDTH,
        height=OUTPUT_PAD_HEIGHT,
        z=OUTPUT_BODY_HEIGHT / 2.0 + OUTPUT_PAD_HEIGHT / 2.0,
        material="output_finish",
    )
    _span_box(
        output_stage,
        name="output_end_plate",
        x_min=OUTPUT_BODY_FRONT_PROJECTION - OUTPUT_END_PLATE_OVERLAP,
        x_max=OUTPUT_BODY_FRONT_PROJECTION + OUTPUT_END_PLATE_THICKNESS,
        width=OUTPUT_END_PLATE_WIDTH,
        height=OUTPUT_END_PLATE_HEIGHT,
        material="plate_finish",
    )

    model.articulation(
        "outer_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=intermediate_stage,
        origin=Origin(xyz=(OUTER_FRONT_MOUTH_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INTERMEDIATE_TRAVEL,
            effort=120.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "intermediate_to_output",
        ArticulationType.PRISMATIC,
        parent=intermediate_stage,
        child=output_stage,
        origin=Origin(xyz=(INTERMEDIATE_FRONT_PROJECTION, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTPUT_TRAVEL,
            effort=90.0,
            velocity=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_sleeve = object_model.get_part("outer_sleeve")
    intermediate_stage = object_model.get_part("intermediate_stage")
    output_stage = object_model.get_part("output_stage")
    outer_to_intermediate = object_model.get_articulation("outer_to_intermediate")
    intermediate_to_output = object_model.get_articulation("intermediate_to_output")

    ctx.check(
        "serial prismatic joints share the slide axis",
        outer_to_intermediate.axis == (1.0, 0.0, 0.0)
        and intermediate_to_output.axis == (1.0, 0.0, 0.0),
        details=(
            f"outer axis={outer_to_intermediate.axis}, "
            f"inner axis={intermediate_to_output.axis}"
        ),
    )

    ctx.expect_contact(
        intermediate_stage,
        outer_sleeve,
        elem_a="intermediate_bearing_pad",
        elem_b="outer_top_wall",
        name="intermediate stage is carried by a bearing pad under the outer sleeve",
    )
    ctx.expect_within(
        intermediate_stage,
        outer_sleeve,
        axes="yz",
        margin=0.0,
        name="intermediate stage stays centered inside the outer sleeve",
    )
    ctx.expect_overlap(
        intermediate_stage,
        outer_sleeve,
        axes="x",
        min_overlap=0.12,
        name="collapsed intermediate stage retains insertion in the outer sleeve",
    )

    ctx.expect_contact(
        output_stage,
        intermediate_stage,
        elem_a="output_bearing_pad",
        elem_b="intermediate_top_wall",
        name="output member is carried by a bearing pad under the intermediate sleeve",
    )
    ctx.expect_within(
        output_stage,
        intermediate_stage,
        axes="yz",
        inner_elem="output_member_body",
        margin=0.0,
        name="output member stays centered inside the intermediate sleeve",
    )
    ctx.expect_overlap(
        output_stage,
        intermediate_stage,
        axes="x",
        elem_a="output_member_body",
        min_overlap=0.18,
        name="collapsed output member retains insertion in the intermediate sleeve",
    )
    ctx.expect_gap(
        output_stage,
        intermediate_stage,
        axis="x",
        positive_elem="output_end_plate",
        min_gap=0.09,
        name="end plate sits ahead of the intermediate sleeve",
    )

    intermediate_rest_pos = ctx.part_world_position(intermediate_stage)
    with ctx.pose({outer_to_intermediate: INTERMEDIATE_TRAVEL}):
        ctx.expect_contact(
            intermediate_stage,
            outer_sleeve,
            elem_a="intermediate_bearing_pad",
            elem_b="outer_top_wall",
            name="intermediate bearing pad stays engaged at full first-stage extension",
        )
        ctx.expect_within(
            intermediate_stage,
            outer_sleeve,
            axes="yz",
            margin=0.0,
            name="extended intermediate stage stays centered in the outer sleeve",
        )
        ctx.expect_overlap(
            intermediate_stage,
            outer_sleeve,
            axes="x",
            min_overlap=0.12,
            name="extended intermediate stage still retains insertion in the outer sleeve",
        )
        intermediate_extended_pos = ctx.part_world_position(intermediate_stage)

    output_rest_pos = ctx.part_world_position(output_stage)
    with ctx.pose({intermediate_to_output: OUTPUT_TRAVEL}):
        ctx.expect_contact(
            output_stage,
            intermediate_stage,
            elem_a="output_bearing_pad",
            elem_b="intermediate_top_wall",
            name="output bearing pad stays engaged at full second-stage extension",
        )
        ctx.expect_within(
            output_stage,
            intermediate_stage,
            axes="yz",
            inner_elem="output_member_body",
            margin=0.0,
            name="extended output member stays centered in the intermediate sleeve",
        )
        ctx.expect_overlap(
            output_stage,
            intermediate_stage,
            axes="x",
            elem_a="output_member_body",
            min_overlap=0.07,
            name="extended output member still retains insertion in the intermediate sleeve",
        )
        output_extended_pos = ctx.part_world_position(output_stage)

    ctx.check(
        "intermediate stage extends along +X",
        intermediate_rest_pos is not None
        and intermediate_extended_pos is not None
        and intermediate_extended_pos[0] > intermediate_rest_pos[0] + 0.05,
        details=(
            f"rest={intermediate_rest_pos}, "
            f"extended={intermediate_extended_pos}"
        ),
    )
    ctx.check(
        "output stage extends along +X",
        output_rest_pos is not None
        and output_extended_pos is not None
        and output_extended_pos[0] > output_rest_pos[0] + 0.05,
        details=f"rest={output_rest_pos}, extended={output_extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
