from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WIDTH = 0.96
DEPTH = 0.24
HEIGHT = 0.58
PLINTH_HEIGHT = 0.04
WALL_THICKNESS = 0.016
TOP_THICKNESS = 0.016

OUTLET_WIDTH = 0.74
OUTLET_DEPTH = 0.055
OUTLET_FRONT_Y = 0.015
OUTLET_REAR_Y = OUTLET_FRONT_Y + OUTLET_DEPTH
OUTLET_CENTER_Y = (OUTLET_FRONT_Y + OUTLET_REAR_Y) / 2.0
OUTLET_MARGIN_X = (WIDTH - OUTLET_WIDTH) / 2.0

FLAP_WIDTH = OUTLET_WIDTH - 0.020
FLAP_DEPTH = OUTLET_DEPTH - 0.006
FLAP_THICKNESS = 0.009

CONTROL_PANEL_WIDTH = 0.19
CONTROL_PANEL_HEIGHT = 0.18
CONTROL_PANEL_THICKNESS = 0.004
CONTROL_PANEL_X = WIDTH / 2.0 - 0.12
CONTROL_PANEL_Z = 0.405

KNOB_RADIUS = 0.014
KNOB_SKIRT_RADIUS = 0.018
KNOB_BODY_LENGTH = 0.018
KNOB_SKIRT_LENGTH = 0.004
KNOB_POINTER_LENGTH = 0.002


def _add_knob_visuals(part, material) -> None:
    part.visual(
        Cylinder(radius=KNOB_SKIRT_RADIUS, length=KNOB_SKIRT_LENGTH),
        origin=Origin(
            xyz=(0.0, -KNOB_SKIRT_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name="skirt",
    )
    part.visual(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_BODY_LENGTH),
        origin=Origin(
            xyz=(0.0, -KNOB_BODY_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name="body",
    )
    part.visual(
        Box((0.003, KNOB_POINTER_LENGTH, 0.010)),
        origin=Origin(
            xyz=(0.0, -KNOB_BODY_LENGTH - KNOB_POINTER_LENGTH / 2.0, KNOB_RADIUS * 0.55),
        ),
        material=material,
        name="pointer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_console_air_conditioner")

    housing = model.material("housing", rgba=(0.90, 0.91, 0.88, 1.0))
    trim = model.material("trim", rgba=(0.78, 0.80, 0.78, 1.0))
    vent_dark = model.material("vent_dark", rgba=(0.25, 0.29, 0.31, 1.0))
    panel = model.material("panel", rgba=(0.72, 0.75, 0.74, 1.0))
    knob = model.material("knob", rgba=(0.80, 0.81, 0.78, 1.0))
    pointer = model.material("pointer", rgba=(0.20, 0.22, 0.24, 1.0))

    body = model.part("body")

    side_height = HEIGHT - PLINTH_HEIGHT - TOP_THICKNESS
    front_top_depth = OUTLET_FRONT_Y + DEPTH / 2.0
    rear_top_depth = DEPTH / 2.0 - OUTLET_REAR_Y
    intake_slat_width = WIDTH - 0.18
    intake_slats = 6
    intake_zone_height = 0.20
    intake_bottom_z = 0.12
    intake_spacing = intake_zone_height / (intake_slats + 1)

    body.visual(
        Box((WIDTH, DEPTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT / 2.0)),
        material=trim,
        name="plinth",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL_THICKNESS, DEPTH - 2.0 * WALL_THICKNESS, 0.010)),
        origin=Origin(
            xyz=(0.0, 0.0, PLINTH_HEIGHT + 0.005),
        ),
        material=vent_dark,
        name="base_shadow",
    )
    body.visual(
        Box((WALL_THICKNESS, DEPTH, side_height)),
        origin=Origin(
            xyz=(
                -WIDTH / 2.0 + WALL_THICKNESS / 2.0,
                0.0,
                PLINTH_HEIGHT + side_height / 2.0,
            )
        ),
        material=housing,
        name="side_left",
    )
    body.visual(
        Box((WALL_THICKNESS, DEPTH, side_height)),
        origin=Origin(
            xyz=(
                WIDTH / 2.0 - WALL_THICKNESS / 2.0,
                0.0,
                PLINTH_HEIGHT + side_height / 2.0,
            )
        ),
        material=housing,
        name="side_right",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, side_height)),
        origin=Origin(
            xyz=(
                0.0,
                DEPTH / 2.0 - WALL_THICKNESS / 2.0,
                PLINTH_HEIGHT + side_height / 2.0,
            )
        ),
        material=housing,
        name="rear_shell",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, side_height)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + WALL_THICKNESS / 2.0,
                PLINTH_HEIGHT + side_height / 2.0,
            )
        ),
        material=housing,
        name="front_shell",
    )
    body.visual(
        Box((WIDTH, front_top_depth, TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -DEPTH / 2.0 + front_top_depth / 2.0,
                HEIGHT - TOP_THICKNESS / 2.0,
            )
        ),
        material=housing,
        name="top_front",
    )
    body.visual(
        Box((WIDTH, rear_top_depth, TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                OUTLET_REAR_Y + rear_top_depth / 2.0,
                HEIGHT - TOP_THICKNESS / 2.0,
            )
        ),
        material=housing,
        name="top_rear",
    )
    for index, sign in enumerate((-1.0, 1.0)):
        body.visual(
            Box((OUTLET_MARGIN_X, OUTLET_DEPTH, TOP_THICKNESS)),
            origin=Origin(
                xyz=(
                    sign * (OUTLET_WIDTH / 2.0 + OUTLET_MARGIN_X / 2.0),
                    OUTLET_CENTER_Y,
                    HEIGHT - TOP_THICKNESS / 2.0,
                )
            ),
            material=housing,
            name=f"outlet_cap_{index}",
        )
    body.visual(
        Box((OUTLET_WIDTH, OUTLET_DEPTH, 0.040)),
        origin=Origin(
            xyz=(0.0, OUTLET_CENTER_Y, HEIGHT - TOP_THICKNESS - 0.020),
        ),
        material=vent_dark,
        name="outlet_well",
    )
    body.visual(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(CONTROL_PANEL_X, -DEPTH / 2.0 + CONTROL_PANEL_THICKNESS / 2.0, CONTROL_PANEL_Z),
        ),
        material=panel,
        name="control_panel",
    )
    body.visual(
        Box((WIDTH - 0.12, 0.004, 0.028)),
        origin=Origin(
            xyz=(0.0, -DEPTH / 2.0 + 0.002, 0.525),
        ),
        material=trim,
        name="front_upper_band",
    )
    for index in range(intake_slats):
        body.visual(
            Box((intake_slat_width, 0.004, 0.010)),
            origin=Origin(
                xyz=(
                    -0.03,
                    -DEPTH / 2.0 + 0.002,
                    intake_bottom_z + (index + 1) * intake_spacing,
                )
            ),
            material=trim,
            name=f"intake_slat_{index}",
        )

    flap = model.part("flap")
    flap.visual(
        Box((FLAP_WIDTH, FLAP_DEPTH, FLAP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, FLAP_DEPTH / 2.0, FLAP_THICKNESS / 2.0),
        ),
        material=housing,
        name="panel",
    )
    flap.visual(
        Cylinder(radius=0.006, length=FLAP_WIDTH - 0.020),
        origin=Origin(
            xyz=(0.0, 0.004, 0.006),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim,
        name="barrel",
    )

    knob_0 = model.part("knob_0")
    _add_knob_visuals(knob_0, knob)
    knob_0.visual(
        Box((0.002, KNOB_POINTER_LENGTH, 0.006)),
        origin=Origin(
            xyz=(0.0, -KNOB_BODY_LENGTH - KNOB_POINTER_LENGTH / 2.0, KNOB_RADIUS * 0.55),
        ),
        material=pointer,
        name="mark",
    )

    knob_1 = model.part("knob_1")
    _add_knob_visuals(knob_1, knob)
    knob_1.visual(
        Box((0.002, KNOB_POINTER_LENGTH, 0.006)),
        origin=Origin(
            xyz=(0.0, -KNOB_BODY_LENGTH - KNOB_POINTER_LENGTH / 2.0, KNOB_RADIUS * 0.55),
        ),
        material=pointer,
        name="mark",
    )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, OUTLET_FRONT_Y, HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "body_to_knob_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob_0,
        origin=Origin(xyz=(CONTROL_PANEL_X + 0.015, -DEPTH / 2.0, 0.435)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )
    model.articulation(
        "body_to_knob_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob_1,
        origin=Origin(xyz=(CONTROL_PANEL_X + 0.015, -DEPTH / 2.0, 0.345)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    flap = object_model.get_part("flap")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")

    flap_joint = object_model.get_articulation("body_to_flap")
    knob_0_joint = object_model.get_articulation("body_to_knob_0")
    knob_1_joint = object_model.get_articulation("body_to_knob_1")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Expected the room-unit body to compile.")
    if body_aabb is None:
        return ctx.report()

    body_mins, body_maxs = body_aabb
    body_size = tuple(float(body_maxs[i] - body_mins[i]) for i in range(3))
    ctx.check(
        "body_floor_console_scale",
        0.90 <= body_size[0] <= 1.02 and 0.21 <= body_size[1] <= 0.27 and 0.55 <= body_size[2] <= 0.60,
        details=f"size={body_size!r}",
    )
    ctx.check(
        "knob_joints_are_continuous",
        knob_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_1_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"knob_0_type={knob_0_joint.articulation_type!r}, "
            f"knob_1_type={knob_1_joint.articulation_type!r}"
        ),
    )
    ctx.check(
        "control_axes_face_forward",
        abs(knob_0_joint.axis[1]) > 0.99 and abs(knob_1_joint.axis[1]) > 0.99,
        details=f"axes={knob_0_joint.axis!r}, {knob_1_joint.axis!r}",
    )

    knob_0_pos = ctx.part_world_position(knob_0)
    knob_1_pos = ctx.part_world_position(knob_1)
    ctx.check(
        "controls_stack_on_one_front_side",
        knob_0_pos is not None
        and knob_1_pos is not None
        and knob_0_pos[0] > 0.20
        and knob_1_pos[0] > 0.20
        and abs(knob_0_pos[0] - knob_1_pos[0]) < 0.01
        and knob_0_pos[2] > knob_1_pos[2] + 0.07,
        details=f"knob_0={knob_0_pos!r}, knob_1={knob_1_pos!r}",
    )

    flap_upper = flap_joint.motion_limits.upper if flap_joint.motion_limits is not None else 1.05

    with ctx.pose({flap_joint: 0.0}):
        ctx.expect_gap(
            flap,
            body,
            axis="z",
            max_gap=0.010,
            max_penetration=0.0,
            name="flap_closes_flush_to_top",
        )
        ctx.expect_overlap(
            flap,
            body,
            axes="x",
            min_overlap=0.65,
            name="flap_spans_wide_discharge_slot",
        )
        flap_rest_aabb = ctx.part_element_world_aabb(flap, elem="panel")

    with ctx.pose({flap_joint: flap_upper}):
        flap_open_aabb = ctx.part_element_world_aabb(flap, elem="panel")

    ctx.check(
        "flap_opens_upward",
        flap_rest_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][2] > flap_rest_aabb[1][2] + 0.03,
        details=f"rest={flap_rest_aabb!r}, open={flap_open_aabb!r}",
    )

    knob_0_rest = ctx.part_world_position(knob_0)
    knob_1_rest = ctx.part_world_position(knob_1)
    with ctx.pose({knob_0_joint: math.pi / 2.0, knob_1_joint: -math.pi / 2.0}):
        knob_0_rot = ctx.part_world_position(knob_0)
        knob_1_rot = ctx.part_world_position(knob_1)

    ctx.check(
        "controls_rotate_in_place",
        knob_0_rest is not None
        and knob_1_rest is not None
        and knob_0_rot is not None
        and knob_1_rot is not None
        and max(abs(knob_0_rot[i] - knob_0_rest[i]) for i in range(3)) < 1e-6
        and max(abs(knob_1_rot[i] - knob_1_rest[i]) for i in range(3)) < 1e-6,
        details=(
            f"knob_0_rest={knob_0_rest!r}, knob_0_rot={knob_0_rot!r}, "
            f"knob_1_rest={knob_1_rest!r}, knob_1_rot={knob_1_rot!r}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
