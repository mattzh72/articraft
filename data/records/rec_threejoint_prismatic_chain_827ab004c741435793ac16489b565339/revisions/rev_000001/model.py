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


OUTER_LENGTH = 0.72
OUTER_WIDTH = 0.18
OUTER_FLOOR_THICK = 0.012
OUTER_WALL_THICK = 0.010
OUTER_SIDE_HEIGHT = 0.060
FOOT_LENGTH = 0.11
FOOT_WIDTH = 0.24
FOOT_THICK = 0.016
PEDESTAL_HEIGHT = 0.008
OUTER_JOINT_X = 0.028
OUTER_JOINT_Z = FOOT_THICK + PEDESTAL_HEIGHT + OUTER_FLOOR_THICK

FIRST_SHOE_LENGTH = 0.40
FIRST_SHOE_WIDTH = 0.150
FIRST_SHOE_HEIGHT = 0.020
FIRST_UPPER_LENGTH = 0.34
FIRST_UPPER_WIDTH = 0.120
FIRST_UPPER_FLOOR = 0.010
FIRST_UPPER_WALL = 0.008
FIRST_UPPER_HEIGHT = 0.040
FIRST_UPPER_Z = 0.060
FIRST_TRAVEL = 0.22
FIRST_TO_SECOND_X = 0.050
FIRST_TO_SECOND_Z = FIRST_UPPER_Z + FIRST_UPPER_FLOOR

SECOND_SHOE_LENGTH = 0.26
SECOND_SHOE_WIDTH = 0.098
SECOND_SHOE_HEIGHT = 0.018
SECOND_UPPER_LENGTH = 0.16
SECOND_UPPER_WIDTH = 0.072
SECOND_UPPER_FLOOR = 0.008
SECOND_UPPER_WALL = 0.006
SECOND_UPPER_HEIGHT = 0.024
SECOND_UPPER_Z = 0.056
SECOND_TRAVEL = 0.12
SECOND_TO_OUTPUT_X = 0.045
SECOND_TO_OUTPUT_Z = SECOND_UPPER_Z + SECOND_UPPER_FLOOR

OUTPUT_SHOE_LENGTH = 0.10
OUTPUT_SHOE_WIDTH = 0.058
OUTPUT_SHOE_HEIGHT = 0.014
OUTPUT_PLATE_LENGTH = 0.13
OUTPUT_PLATE_WIDTH = 0.090
OUTPUT_PLATE_THICK = 0.010
OUTPUT_STAGE_Z = 0.034
OUTPUT_TRAVEL = 0.05


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _populate_outer_channel(part) -> None:
    z0 = FOOT_THICK + PEDESTAL_HEIGHT
    rear_foot_x = 0.10
    front_foot_x = OUTER_LENGTH - 0.10
    _add_box_visual(
        part,
        size=(OUTER_LENGTH, OUTER_WIDTH, OUTER_FLOOR_THICK),
        center=(OUTER_LENGTH / 2.0, 0.0, z0 + OUTER_FLOOR_THICK / 2.0),
        material="grounded_steel",
        name="outer_floor",
    )
    _add_box_visual(
        part,
        size=(OUTER_LENGTH, OUTER_WALL_THICK, OUTER_SIDE_HEIGHT),
        center=(
            OUTER_LENGTH / 2.0,
            OUTER_WIDTH / 2.0 - OUTER_WALL_THICK / 2.0,
            z0 + OUTER_FLOOR_THICK + OUTER_SIDE_HEIGHT / 2.0,
        ),
        material="grounded_steel",
        name="outer_left_wall",
    )
    _add_box_visual(
        part,
        size=(OUTER_LENGTH, OUTER_WALL_THICK, OUTER_SIDE_HEIGHT),
        center=(
            OUTER_LENGTH / 2.0,
            -OUTER_WIDTH / 2.0 + OUTER_WALL_THICK / 2.0,
            z0 + OUTER_FLOOR_THICK + OUTER_SIDE_HEIGHT / 2.0,
        ),
        material="grounded_steel",
        name="outer_right_wall",
    )
    _add_box_visual(
        part,
        size=(0.020, OUTER_WIDTH - 2.0 * OUTER_WALL_THICK, 0.028),
        center=(0.010, 0.0, z0 + OUTER_FLOOR_THICK + 0.014),
        material="grounded_steel",
        name="outer_rear_bridge",
    )
    _add_box_visual(
        part,
        size=(FOOT_LENGTH, FOOT_WIDTH, FOOT_THICK),
        center=(rear_foot_x, 0.0, FOOT_THICK / 2.0),
        material="grounded_steel",
        name="rear_foot",
    )
    _add_box_visual(
        part,
        size=(FOOT_LENGTH, FOOT_WIDTH, FOOT_THICK),
        center=(front_foot_x, 0.0, FOOT_THICK / 2.0),
        material="grounded_steel",
        name="front_foot",
    )
    _add_box_visual(
        part,
        size=(FOOT_LENGTH * 0.78, OUTER_WIDTH - 0.020, PEDESTAL_HEIGHT),
        center=(rear_foot_x, 0.0, FOOT_THICK + PEDESTAL_HEIGHT / 2.0),
        material="grounded_steel",
        name="rear_pedestal",
    )
    _add_box_visual(
        part,
        size=(FOOT_LENGTH * 0.78, OUTER_WIDTH - 0.020, PEDESTAL_HEIGHT),
        center=(front_foot_x, 0.0, FOOT_THICK + PEDESTAL_HEIGHT / 2.0),
        material="grounded_steel",
        name="front_pedestal",
    )


def _populate_first_runner(part) -> None:
    _add_box_visual(
        part,
        size=(FIRST_SHOE_LENGTH, FIRST_SHOE_WIDTH, FIRST_SHOE_HEIGHT),
        center=(FIRST_SHOE_LENGTH / 2.0, 0.0, FIRST_SHOE_HEIGHT / 2.0),
        material="runner_gray",
        name="first_shoe",
    )
    _add_box_visual(
        part,
        size=(FIRST_UPPER_LENGTH, FIRST_UPPER_WIDTH, FIRST_UPPER_FLOOR),
        center=(0.045 + FIRST_UPPER_LENGTH / 2.0, 0.0, FIRST_UPPER_Z + FIRST_UPPER_FLOOR / 2.0),
        material="runner_gray",
        name="first_upper_floor",
    )
    _add_box_visual(
        part,
        size=(FIRST_UPPER_LENGTH, FIRST_UPPER_WALL, FIRST_UPPER_HEIGHT),
        center=(
            0.045 + FIRST_UPPER_LENGTH / 2.0,
            FIRST_UPPER_WIDTH / 2.0 - FIRST_UPPER_WALL / 2.0,
            FIRST_UPPER_Z + FIRST_UPPER_FLOOR + FIRST_UPPER_HEIGHT / 2.0,
        ),
        material="runner_gray",
        name="first_left_wall",
    )
    _add_box_visual(
        part,
        size=(FIRST_UPPER_LENGTH, FIRST_UPPER_WALL, FIRST_UPPER_HEIGHT),
        center=(
            0.045 + FIRST_UPPER_LENGTH / 2.0,
            -FIRST_UPPER_WIDTH / 2.0 + FIRST_UPPER_WALL / 2.0,
            FIRST_UPPER_Z + FIRST_UPPER_FLOOR + FIRST_UPPER_HEIGHT / 2.0,
        ),
        material="runner_gray",
        name="first_right_wall",
    )
    web_length = 0.22
    web_thickness = 0.012
    web_height = FIRST_UPPER_Z - FIRST_SHOE_HEIGHT
    _add_box_visual(
        part,
        size=(web_length, web_thickness, web_height),
        center=(
            0.17,
            FIRST_UPPER_WIDTH / 2.0 - web_thickness / 2.0,
            FIRST_SHOE_HEIGHT + web_height / 2.0,
        ),
        material="runner_gray",
        name="first_left_web",
    )
    _add_box_visual(
        part,
        size=(web_length, web_thickness, web_height),
        center=(
            0.17,
            -FIRST_UPPER_WIDTH / 2.0 + web_thickness / 2.0,
            FIRST_SHOE_HEIGHT + web_height / 2.0,
        ),
        material="runner_gray",
        name="first_right_web",
    )
    _add_box_visual(
        part,
        size=(0.030, FIRST_UPPER_WIDTH - 0.016, 0.018),
        center=(FIRST_SHOE_LENGTH - 0.025, 0.0, FIRST_SHOE_HEIGHT + 0.009),
        material="runner_gray",
        name="first_nose_block",
    )


def _populate_second_runner(part) -> None:
    rail_width = 0.014
    rail_height = 0.008
    rail_offset_y = 0.022
    bridge_height = 0.008
    bridge_top = rail_height + bridge_height
    riser_height = SECOND_UPPER_Z - bridge_top
    _add_box_visual(
        part,
        size=(0.24, rail_width, rail_height),
        center=(0.12, rail_offset_y, rail_height / 2.0),
        material="runner_light",
        name="second_left_rail",
    )
    _add_box_visual(
        part,
        size=(0.24, rail_width, rail_height),
        center=(0.12, -rail_offset_y, rail_height / 2.0),
        material="runner_light",
        name="second_right_rail",
    )
    _add_box_visual(
        part,
        size=(0.18, 0.050, bridge_height),
        center=(0.11, 0.0, rail_height + bridge_height / 2.0),
        material="runner_light",
        name="second_bridge",
    )
    _add_box_visual(
        part,
        size=(0.12, 0.008, riser_height),
        center=(0.115, 0.026, bridge_top + riser_height / 2.0),
        material="runner_light",
        name="second_left_riser",
    )
    _add_box_visual(
        part,
        size=(0.12, 0.008, riser_height),
        center=(0.115, -0.026, bridge_top + riser_height / 2.0),
        material="runner_light",
        name="second_right_riser",
    )
    _add_box_visual(
        part,
        size=(0.020, 0.048, riser_height),
        center=(0.185, 0.0, bridge_top + riser_height / 2.0),
        material="runner_light",
        name="second_front_mast",
    )
    _add_box_visual(
        part,
        size=(SECOND_UPPER_LENGTH, SECOND_UPPER_WIDTH, SECOND_UPPER_FLOOR),
        center=(0.040 + SECOND_UPPER_LENGTH / 2.0, 0.0, SECOND_UPPER_Z + SECOND_UPPER_FLOOR / 2.0),
        material="runner_light",
        name="second_upper_floor",
    )
    _add_box_visual(
        part,
        size=(SECOND_UPPER_LENGTH, SECOND_UPPER_WALL, SECOND_UPPER_HEIGHT),
        center=(
            0.040 + SECOND_UPPER_LENGTH / 2.0,
            SECOND_UPPER_WIDTH / 2.0 - SECOND_UPPER_WALL / 2.0,
            SECOND_UPPER_Z + SECOND_UPPER_FLOOR + SECOND_UPPER_HEIGHT / 2.0,
        ),
        material="runner_light",
        name="second_left_wall",
    )
    _add_box_visual(
        part,
        size=(SECOND_UPPER_LENGTH, SECOND_UPPER_WALL, SECOND_UPPER_HEIGHT),
        center=(
            0.040 + SECOND_UPPER_LENGTH / 2.0,
            -SECOND_UPPER_WIDTH / 2.0 + SECOND_UPPER_WALL / 2.0,
            SECOND_UPPER_Z + SECOND_UPPER_FLOOR + SECOND_UPPER_HEIGHT / 2.0,
        ),
        material="runner_light",
        name="second_right_wall",
    )


def _populate_output_stage(part) -> None:
    _add_box_visual(
        part,
        size=(OUTPUT_SHOE_LENGTH, OUTPUT_SHOE_WIDTH, OUTPUT_SHOE_HEIGHT),
        center=(OUTPUT_SHOE_LENGTH / 2.0, 0.0, OUTPUT_SHOE_HEIGHT / 2.0),
        material="stage_yellow",
        name="output_shoe",
    )
    _add_box_visual(
        part,
        size=(0.050, 0.040, OUTPUT_STAGE_Z - OUTPUT_SHOE_HEIGHT),
        center=(
            0.055,
            0.0,
            OUTPUT_SHOE_HEIGHT + (OUTPUT_STAGE_Z - OUTPUT_SHOE_HEIGHT) / 2.0,
        ),
        material="stage_yellow",
        name="output_pedestal",
    )
    _add_box_visual(
        part,
        size=(OUTPUT_PLATE_LENGTH, OUTPUT_PLATE_WIDTH, OUTPUT_PLATE_THICK),
        center=(0.070, 0.0, OUTPUT_STAGE_Z + OUTPUT_PLATE_THICK / 2.0),
        material="stage_yellow",
        name="output_plate",
    )
    _add_box_visual(
        part,
        size=(0.012, OUTPUT_PLATE_WIDTH * 0.72, 0.020),
        center=(0.125, 0.0, OUTPUT_STAGE_Z + OUTPUT_PLATE_THICK + 0.010),
        material="stage_yellow",
        name="output_front_lip",
    )
    _add_box_visual(
        part,
        size=(0.022, 0.050, 0.012),
        center=(0.018, 0.0, OUTPUT_STAGE_Z + OUTPUT_PLATE_THICK + 0.006),
        material="stage_yellow",
        name="output_rear_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_runner_transfer_stack")

    model.material("grounded_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("runner_gray", rgba=(0.60, 0.63, 0.66, 1.0))
    model.material("runner_light", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("stage_yellow", rgba=(0.87, 0.73, 0.16, 1.0))

    outer_channel = model.part("outer_channel")
    _populate_outer_channel(outer_channel)
    outer_channel.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, FOOT_WIDTH, FOOT_THICK + PEDESTAL_HEIGHT + OUTER_FLOOR_THICK + OUTER_SIDE_HEIGHT)),
        mass=9.5,
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                0.0,
                (FOOT_THICK + PEDESTAL_HEIGHT + OUTER_FLOOR_THICK + OUTER_SIDE_HEIGHT) / 2.0,
            )
        ),
    )

    first_runner = model.part("first_runner")
    _populate_first_runner(first_runner)
    first_runner.inertial = Inertial.from_geometry(
        Box((FIRST_SHOE_LENGTH, FIRST_SHOE_WIDTH, FIRST_UPPER_Z + FIRST_UPPER_FLOOR + FIRST_UPPER_HEIGHT)),
        mass=3.1,
        origin=Origin(
            xyz=(
                FIRST_SHOE_LENGTH / 2.0,
                0.0,
                (FIRST_UPPER_Z + FIRST_UPPER_FLOOR + FIRST_UPPER_HEIGHT) / 2.0,
            )
        ),
    )

    second_runner = model.part("second_runner")
    _populate_second_runner(second_runner)
    second_runner.inertial = Inertial.from_geometry(
        Box((SECOND_SHOE_LENGTH, SECOND_SHOE_WIDTH, SECOND_UPPER_Z + SECOND_UPPER_FLOOR + SECOND_UPPER_HEIGHT)),
        mass=1.7,
        origin=Origin(
            xyz=(
                SECOND_SHOE_LENGTH / 2.0,
                0.0,
                (SECOND_UPPER_Z + SECOND_UPPER_FLOOR + SECOND_UPPER_HEIGHT) / 2.0,
            )
        ),
    )

    output_stage = model.part("output_stage")
    _populate_output_stage(output_stage)
    output_stage.inertial = Inertial.from_geometry(
        Box((OUTPUT_PLATE_LENGTH, OUTPUT_PLATE_WIDTH, OUTPUT_STAGE_Z + OUTPUT_PLATE_THICK + 0.020)),
        mass=0.8,
        origin=Origin(
            xyz=(
                OUTPUT_PLATE_LENGTH / 2.0,
                0.0,
                (OUTPUT_STAGE_Z + OUTPUT_PLATE_THICK + 0.020) / 2.0,
            )
        ),
    )

    model.articulation(
        "outer_to_first",
        ArticulationType.PRISMATIC,
        parent=outer_channel,
        child=first_runner,
        origin=Origin(xyz=(OUTER_JOINT_X, 0.0, OUTER_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FIRST_TRAVEL,
            effort=240.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.PRISMATIC,
        parent=first_runner,
        child=second_runner,
        origin=Origin(xyz=(FIRST_TO_SECOND_X, 0.0, FIRST_TO_SECOND_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SECOND_TRAVEL,
            effort=180.0,
            velocity=0.50,
        ),
    )
    model.articulation(
        "second_to_output",
        ArticulationType.PRISMATIC,
        parent=second_runner,
        child=output_stage,
        origin=Origin(xyz=(SECOND_TO_OUTPUT_X, 0.0, SECOND_TO_OUTPUT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTPUT_TRAVEL,
            effort=90.0,
            velocity=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_channel = object_model.get_part("outer_channel")
    first_runner = object_model.get_part("first_runner")
    second_runner = object_model.get_part("second_runner")
    output_stage = object_model.get_part("output_stage")

    outer_to_first = object_model.get_articulation("outer_to_first")
    first_to_second = object_model.get_articulation("first_to_second")
    second_to_output = object_model.get_articulation("second_to_output")

    ctx.expect_overlap(
        first_runner,
        outer_channel,
        axes="x",
        min_overlap=0.35,
        name="first runner is substantially inserted in the grounded channel at rest",
    )
    ctx.expect_overlap(
        second_runner,
        first_runner,
        axes="x",
        min_overlap=0.20,
        name="second runner is substantially inserted in the first runner at rest",
    )
    ctx.expect_overlap(
        output_stage,
        second_runner,
        axes="x",
        min_overlap=0.10,
        name="output stage is substantially inserted in the second runner at rest",
    )

    first_rest = ctx.part_world_position(first_runner)
    with ctx.pose({outer_to_first: FIRST_TRAVEL}):
        ctx.expect_overlap(
            first_runner,
            outer_channel,
            axes="x",
            min_overlap=0.14,
            name="first runner retains insertion at full outer travel",
        )
        first_extended = ctx.part_world_position(first_runner)
    ctx.check(
        "first runner extends along +X",
        first_rest is not None and first_extended is not None and first_extended[0] > first_rest[0] + 0.18,
        details=f"rest={first_rest}, extended={first_extended}",
    )

    second_rest = ctx.part_world_position(second_runner)
    with ctx.pose({first_to_second: SECOND_TRAVEL}):
        ctx.expect_overlap(
            second_runner,
            first_runner,
            axes="x",
            min_overlap=0.12,
            name="second runner retains insertion at full intermediate travel",
        )
        second_extended = ctx.part_world_position(second_runner)
    ctx.check(
        "second runner extends along +X",
        second_rest is not None and second_extended is not None and second_extended[0] > second_rest[0] + 0.08,
        details=f"rest={second_rest}, extended={second_extended}",
    )

    output_rest = ctx.part_world_position(output_stage)
    with ctx.pose({second_to_output: OUTPUT_TRAVEL}):
        ctx.expect_overlap(
            output_stage,
            second_runner,
            axes="x",
            min_overlap=0.06,
            name="output stage retains insertion at full output travel",
        )
        output_extended = ctx.part_world_position(output_stage)
    ctx.check(
        "output stage extends along +X",
        output_rest is not None and output_extended is not None and output_extended[0] > output_rest[0] + 0.04,
        details=f"rest={output_rest}, extended={output_extended}",
    )

    with ctx.pose(
        {
            outer_to_first: FIRST_TRAVEL,
            first_to_second: SECOND_TRAVEL,
            second_to_output: OUTPUT_TRAVEL,
        }
    ):
        full_stack_position = ctx.part_world_position(output_stage)
    ctx.check(
        "full transfer stack reaches forward",
        output_rest is not None and full_stack_position is not None and full_stack_position[0] > output_rest[0] + 0.34,
        details=f"rest={output_rest}, fully_extended={full_stack_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
