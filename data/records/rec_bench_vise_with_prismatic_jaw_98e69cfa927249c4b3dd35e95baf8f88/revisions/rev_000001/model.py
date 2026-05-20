from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

BASE_LENGTH = 0.168
BASE_WIDTH = 0.062
BASE_THICK = 0.012

FIXED_JAW_LENGTH = 0.020
FIXED_JAW_WIDTH = 0.060
FIXED_JAW_HEIGHT = 0.035
FIXED_JAW_CENTER_X = -0.074
FIXED_JAW_FACE_X = FIXED_JAW_CENTER_X + FIXED_JAW_LENGTH * 0.5

WAY_LENGTH = 0.104
WAY_WIDTH = 0.014
WAY_HEIGHT = 0.010
WAY_CENTER_X = 0.004
WAY_CENTER_Y = 0.024

FRONT_BLOCK_LENGTH = 0.024
FRONT_BLOCK_WIDTH = 0.044
FRONT_BLOCK_HEIGHT = 0.028
FRONT_BLOCK_CENTER_X = 0.072

FRONT_LOWER_HEIGHT = 0.006
FRONT_UPPER_HEIGHT = 0.016
FRONT_CHEEK_WIDTH = 0.005
STOP_SLOT_HEIGHT = 0.006
STOP_SLOT_CLEAR_WIDTH = FRONT_BLOCK_WIDTH - 2.0 * FRONT_CHEEK_WIDTH
STOP_SLOT_Z = BASE_THICK + FRONT_LOWER_HEIGHT + STOP_SLOT_HEIGHT * 0.5

SCREW_AXIS_Z = 0.038
SCREW_RADIUS = 0.004
SCREW_BORE_RADIUS = 0.0052

JAW_CLOSED_X = -0.052
JAW_TRAVEL = 0.050
JAW_BLOCK_LENGTH = 0.020
JAW_BLOCK_WIDTH = 0.054
JAW_BLOCK_HEIGHT = 0.035
JAW_CARRIAGE_START_X = 0.008
JAW_CARRIAGE_LENGTH = 0.052
JAW_CARRIAGE_WIDTH = 0.030
JAW_CARRIAGE_HEIGHT = 0.010
JAW_BLOCK_BASE_Z = JAW_CARRIAGE_HEIGHT
JAW_FRONT_PAD_LENGTH = 0.020
JAW_FRONT_PAD_CENTER_X = 0.048
JAW_FRONT_PAD_WIDTH = 0.040
JAW_FRONT_PAD_HEIGHT = 0.015

INSERT_THICK = 0.003
INSERT_WIDTH = 0.052
INSERT_HEIGHT = 0.022

STOP_TRAVEL = 0.011
STOP_TONGUE_LENGTH = 0.020
STOP_TONGUE_WIDTH = 0.010
STOP_HEAD_WIDTH = 0.020
STOP_HEAD_HEIGHT = 0.018
STOP_CAP_WIDTH = 0.026
STOP_CAP_HEIGHT = 0.008
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_drill_vise")

    cast_iron = model.material("cast_iron", rgba=(0.36, 0.38, 0.40, 1.0))
    jaw_steel = model.material("jaw_steel", rgba=(0.68, 0.70, 0.73, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK * 0.5)),
        material=cast_iron,
        name="plate",
    )
    base.visual(
        Box((WAY_LENGTH, WAY_WIDTH, WAY_HEIGHT)),
        origin=Origin(xyz=(WAY_CENTER_X, WAY_CENTER_Y, BASE_THICK + WAY_HEIGHT * 0.5)),
        material=cast_iron,
        name="way_0",
    )
    base.visual(
        Box((WAY_LENGTH, WAY_WIDTH, WAY_HEIGHT)),
        origin=Origin(xyz=(WAY_CENTER_X, -WAY_CENTER_Y, BASE_THICK + WAY_HEIGHT * 0.5)),
        material=cast_iron,
        name="way_1",
    )
    base.visual(
        Box((FIXED_JAW_LENGTH, FIXED_JAW_WIDTH, FIXED_JAW_HEIGHT)),
        origin=Origin(
            xyz=(
                FIXED_JAW_CENTER_X,
                0.0,
                BASE_THICK + FIXED_JAW_HEIGHT * 0.5,
            )
        ),
        material=cast_iron,
        name="fixed_jaw",
    )
    base.visual(
        Box((FRONT_BLOCK_LENGTH, FRONT_BLOCK_WIDTH, FRONT_LOWER_HEIGHT)),
        origin=Origin(
            xyz=(
                FRONT_BLOCK_CENTER_X,
                0.0,
                BASE_THICK + FRONT_LOWER_HEIGHT * 0.5,
            )
        ),
        material=cast_iron,
        name="front_lower",
    )
    base.visual(
        Box((FRONT_BLOCK_LENGTH, FRONT_BLOCK_WIDTH, FRONT_UPPER_HEIGHT)),
        origin=Origin(
            xyz=(
                FRONT_BLOCK_CENTER_X,
                0.0,
                BASE_THICK + FRONT_LOWER_HEIGHT + STOP_SLOT_HEIGHT + FRONT_UPPER_HEIGHT * 0.5,
            )
        ),
        material=cast_iron,
        name="front_upper",
    )
    base.visual(
        Box((FRONT_BLOCK_LENGTH, FRONT_CHEEK_WIDTH, STOP_SLOT_HEIGHT)),
        origin=Origin(
            xyz=(
                FRONT_BLOCK_CENTER_X,
                (FRONT_BLOCK_WIDTH - FRONT_CHEEK_WIDTH) * 0.5,
                STOP_SLOT_Z,
            )
        ),
        material=cast_iron,
        name="slot_cheek_0",
    )
    base.visual(
        Box((FRONT_BLOCK_LENGTH, FRONT_CHEEK_WIDTH, STOP_SLOT_HEIGHT)),
        origin=Origin(
            xyz=(
                FRONT_BLOCK_CENTER_X,
                -(FRONT_BLOCK_WIDTH - FRONT_CHEEK_WIDTH) * 0.5,
                STOP_SLOT_Z,
            )
        ),
        material=cast_iron,
        name="slot_cheek_1",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(
            xyz=(FRONT_BLOCK_CENTER_X + FRONT_BLOCK_LENGTH * 0.5 - 0.004, 0.0, SCREW_AXIS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=cast_iron,
        name="bearing_boss",
    )
    base.visual(
        Box((INSERT_THICK, INSERT_WIDTH, INSERT_HEIGHT)),
        origin=Origin(
            xyz=(
                FIXED_JAW_FACE_X + INSERT_THICK * 0.5,
                0.0,
                BASE_THICK + INSERT_HEIGHT * 0.5,
            )
        ),
        material=jaw_steel,
        name="fixed_insert",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICK + FIXED_JAW_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICK + FIXED_JAW_HEIGHT) * 0.5)),
    )

    jaw = model.part("jaw")
    jaw.visual(
        Box((JAW_BLOCK_LENGTH, JAW_BLOCK_WIDTH, JAW_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(JAW_BLOCK_LENGTH * 0.5, 0.0, JAW_BLOCK_BASE_Z + JAW_BLOCK_HEIGHT * 0.5)
        ),
        material=cast_iron,
        name="jaw_block",
    )
    jaw.visual(
        Box((JAW_CARRIAGE_LENGTH, JAW_CARRIAGE_WIDTH, JAW_CARRIAGE_HEIGHT)),
        origin=Origin(
            xyz=(
                JAW_CARRIAGE_START_X + JAW_CARRIAGE_LENGTH * 0.5,
                0.0,
                JAW_CARRIAGE_HEIGHT * 0.5,
            )
        ),
        material=cast_iron,
        name="carriage",
    )
    jaw.visual(
        Box((JAW_FRONT_PAD_LENGTH, JAW_FRONT_PAD_WIDTH, JAW_FRONT_PAD_HEIGHT)),
        origin=Origin(xyz=(JAW_FRONT_PAD_CENTER_X, 0.0, JAW_FRONT_PAD_HEIGHT * 0.5)),
        material=cast_iron,
        name="front_pad",
    )
    jaw.visual(
        Box((INSERT_THICK, INSERT_WIDTH, INSERT_HEIGHT)),
        origin=Origin(xyz=(-INSERT_THICK * 0.5, 0.0, JAW_BLOCK_BASE_Z + INSERT_HEIGHT * 0.5)),
        material=jaw_steel,
        name="sliding_insert",
    )
    jaw.inertial = Inertial.from_geometry(
        Box((0.060, JAW_BLOCK_WIDTH, JAW_BLOCK_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(0.030, 0.0, JAW_BLOCK_HEIGHT * 0.5)),
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=jaw_steel,
        name="hub",
    )
    screw_handle.visual(
        Cylinder(radius=SCREW_RADIUS, length=0.008),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=jaw_steel,
        name="pilot",
    )
    screw_handle.visual(
        Cylinder(radius=0.0035, length=0.078),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=jaw_steel,
        name="bar",
    )
    screw_handle.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.033, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=jaw_steel,
        name="grip_0",
    )
    screw_handle.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, -0.033, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=jaw_steel,
        name="grip_1",
    )
    screw_handle.inertial = Inertial.from_geometry(
        Box((0.020, 0.090, 0.020)),
        mass=0.25,
        origin=Origin(),
    )

    stop = model.part("stop")
    stop.visual(
        Box((STOP_TONGUE_LENGTH, STOP_TONGUE_WIDTH, STOP_SLOT_HEIGHT)),
        origin=Origin(xyz=(-STOP_TONGUE_LENGTH * 0.5, 0.0, 0.0)),
        material=jaw_steel,
        name="tongue",
    )
    stop.visual(
        Box((0.008, 0.018, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.001)),
        material=jaw_steel,
        name="head",
    )
    stop.visual(
        Box((0.010, 0.024, 0.002)),
        origin=Origin(xyz=(0.005, 0.0, 0.006)),
        material=jaw_steel,
        name="cap",
    )
    stop.inertial = Inertial.from_geometry(
        Box((0.032, STOP_CAP_WIDTH, STOP_HEAD_HEIGHT + STOP_CAP_HEIGHT)),
        mass=0.12,
        origin=Origin(xyz=(-0.004, 0.0, 0.013)),
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=jaw,
        origin=Origin(xyz=(JAW_CLOSED_X, 0.0, BASE_THICK)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.05,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "screw_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=screw_handle,
        origin=Origin(
            xyz=(FRONT_BLOCK_CENTER_X + FRONT_BLOCK_LENGTH * 0.5 + 0.010, 0.0, SCREW_AXIS_Z)
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=8.0),
    )
    model.articulation(
        "stop_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stop,
        origin=Origin(
            xyz=(FRONT_BLOCK_CENTER_X + FRONT_BLOCK_LENGTH * 0.5, 0.0, STOP_SLOT_Z)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.04,
            lower=-STOP_TRAVEL,
            upper=STOP_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    jaw = object_model.get_part("jaw")
    base = object_model.get_part("base")
    jaw_slide = object_model.get_articulation("jaw_slide")
    screw_handle = object_model.get_part("screw_handle")
    stop = object_model.get_part("stop")
    screw_rotation = object_model.get_articulation("screw_rotation")
    stop_slide = object_model.get_articulation("stop_slide")

    ctx.expect_gap(
        jaw,
        base,
        axis="x",
        positive_elem="sliding_insert",
        negative_elem="fixed_insert",
        min_gap=0.005,
        max_gap=0.007,
        name="jaw starts nearly closed",
    )
    ctx.expect_within(
        jaw,
        base,
        axes="y",
        margin=0.0,
        name="jaw stays centered between the base edges",
    )

    rest_position = ctx.part_world_position(jaw)
    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        ctx.expect_gap(
            jaw,
            base,
            axis="x",
            positive_elem="sliding_insert",
            negative_elem="fixed_insert",
            min_gap=0.055,
            max_gap=0.057,
            name="jaw opens to a useful drilling span",
        )
        open_position = ctx.part_world_position(jaw)

    ctx.check(
        "jaw slides forward",
        rest_position is not None
        and open_position is not None
        and open_position[0] > rest_position[0] + 0.045,
        details=f"rest={rest_position}, open={open_position}",
    )

    stop_rest = ctx.part_world_position(stop)
    with ctx.pose({stop_slide: STOP_TRAVEL}):
        stop_open = ctx.part_world_position(stop)
    ctx.check(
        "stop slides across the front slot",
        stop_rest is not None and stop_open is not None and stop_open[1] > stop_rest[1] + 0.010,
        details=f"rest={stop_rest}, shifted={stop_open}",
    )

    bar_rest = ctx.part_element_world_aabb(screw_handle, elem="bar")
    with ctx.pose({screw_rotation: math.pi * 0.5}):
        bar_quarter_turn = ctx.part_element_world_aabb(screw_handle, elem="bar")

    def axis_span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    rest_y = axis_span(bar_rest, 1)
    rest_z = axis_span(bar_rest, 2)
    turned_y = axis_span(bar_quarter_turn, 1)
    turned_z = axis_span(bar_quarter_turn, 2)
    ctx.check(
        "handle rotates about the lead screw axis",
        rest_y is not None
        and rest_z is not None
        and turned_y is not None
        and turned_z is not None
        and rest_y > rest_z + 0.050
        and turned_z > turned_y + 0.050,
        details=(
            f"rest_y={rest_y}, rest_z={rest_z}, "
            f"turned_y={turned_y}, turned_z={turned_z}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
