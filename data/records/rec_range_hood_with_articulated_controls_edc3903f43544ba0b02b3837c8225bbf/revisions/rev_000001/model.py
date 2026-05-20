from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_TOP_DEPTH = 0.32
CANOPY_HEIGHT = 0.20
VALANCE_HEIGHT = 0.075
SHELL_THICKNESS = 0.014

CHIMNEY_WIDTH = 0.30
CHIMNEY_DEPTH = 0.24
CHIMNEY_HEIGHT = 0.72

OPENING_WIDTH = 0.78
OPENING_DEPTH = 0.36

CONTROL_Z = 0.041
BUTTON_XS = (-0.21, -0.158, -0.106)
BUTTON_PANEL_Y = 0.4985
BUTTON_CAP_SIZE = 0.028
BUTTON_CAP_DEPTH = 0.006
BUTTON_STEM_SIZE = 0.022
BUTTON_STEM_DEPTH = 0.022
BUTTON_TRAVEL = 0.003
BUTTON_POCKET_SIZE = BUTTON_CAP_SIZE
BUTTON_POCKET_DEPTH = 0.008
BUTTON_HOLE_SIZE = 0.023

KNOB_X = 0.305
KNOB_DIAMETER = 0.046
KNOB_HEIGHT = 0.024
KNOB_SHAFT_RADIUS = 0.006
KNOB_SHAFT_DEPTH = 0.016
KNOB_HOLE_DEPTH = 0.050
KNOB_SLOT_SIZE = 0.020

def make_canopy_shell():
    outer_profile = [
        (0.0, 0.0),
        (CANOPY_DEPTH, 0.0),
        (CANOPY_DEPTH, VALANCE_HEIGHT),
        (CANOPY_TOP_DEPTH, CANOPY_HEIGHT),
        (0.0, CANOPY_HEIGHT),
    ]
    inner_profile = [
        (SHELL_THICKNESS, SHELL_THICKNESS),
        (CANOPY_DEPTH - SHELL_THICKNESS, SHELL_THICKNESS),
        (CANOPY_DEPTH - SHELL_THICKNESS, VALANCE_HEIGHT - SHELL_THICKNESS),
        (CANOPY_TOP_DEPTH - 0.012, CANOPY_HEIGHT - SHELL_THICKNESS),
        (SHELL_THICKNESS, CANOPY_HEIGHT - SHELL_THICKNESS),
    ]

    outer = cq.Workplane("YZ").polyline(outer_profile).close().extrude(CANOPY_WIDTH / 2.0, both=True)
    inner = cq.Workplane("YZ").polyline(inner_profile).close().extrude(
        (CANOPY_WIDTH - 2.0 * SHELL_THICKNESS) / 2.0,
        both=True,
    )
    shell = outer.val().cut(inner.val())
    bottom_opening = (
        cq.Workplane("XY")
        .box(OPENING_WIDTH, OPENING_DEPTH, SHELL_THICKNESS * 3.0)
        .translate((0.0, 0.23, SHELL_THICKNESS * 0.5))
        .val()
    )
    shell = shell.cut(bottom_opening)

    for button_x in BUTTON_XS:
        button_pocket = (
            cq.Workplane("XY")
            .box(BUTTON_POCKET_SIZE, BUTTON_POCKET_DEPTH, BUTTON_POCKET_SIZE)
            .translate((button_x, CANOPY_DEPTH - BUTTON_POCKET_DEPTH / 2.0, CONTROL_Z))
            .val()
        )
        button_slot = (
            cq.Workplane("XY")
            .box(BUTTON_HOLE_SIZE, KNOB_HOLE_DEPTH, BUTTON_HOLE_SIZE)
            .translate((button_x, CANOPY_DEPTH - KNOB_HOLE_DEPTH / 2.0, CONTROL_Z))
            .val()
        )
        shell = shell.cut(button_pocket).cut(button_slot)

    knob_slot = (
        cq.Workplane("XY")
        .box(KNOB_SLOT_SIZE, KNOB_HOLE_DEPTH, KNOB_SLOT_SIZE)
        .translate((KNOB_X, CANOPY_DEPTH - KNOB_HOLE_DEPTH / 2.0, CONTROL_Z))
        .val()
    )
    return shell.cut(knob_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    steel = model.material("steel", rgba=(0.79, 0.80, 0.82, 1.0))
    control_plastic = model.material("control_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    hood_body = model.part("hood_body")
    hood_body.visual(
        mesh_from_cadquery(make_canopy_shell(), "canopy_shell"),
        material=steel,
        name="canopy_shell",
    )
    hood_body.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(0.0, CHIMNEY_DEPTH / 2.0, CANOPY_HEIGHT + CHIMNEY_HEIGHT / 2.0 - 0.01)),
        material=steel,
        name="chimney",
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_CAP_SIZE, BUTTON_CAP_DEPTH, BUTTON_CAP_SIZE)),
            origin=Origin(xyz=(0.0, BUTTON_CAP_DEPTH / 2.0, 0.0)),
            material=control_plastic,
            name="cap",
        )
        button.visual(
            Box((BUTTON_STEM_SIZE, BUTTON_STEM_DEPTH, BUTTON_STEM_SIZE)),
            origin=Origin(xyz=(0.0, -BUTTON_STEM_DEPTH / 2.0 + 0.0015, 0.0)),
            material=control_plastic,
            name="stem",
        )
        model.articulation(
            f"hood_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_PANEL_Y, CONTROL_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                KNOB_DIAMETER,
                KNOB_HEIGHT,
                body_style="skirted",
                top_diameter=0.038,
                skirt=KnobSkirt(0.056, 0.006, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=control_plastic,
        name="knob",
    )
    speed_knob.visual(
        Cylinder(radius=KNOB_SHAFT_RADIUS, length=KNOB_SHAFT_DEPTH),
        origin=Origin(xyz=(0.0, -KNOB_SHAFT_DEPTH / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=control_plastic,
        name="shaft",
    )
    model.articulation(
        "hood_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=hood_body,
        child=speed_knob,
        origin=Origin(xyz=(KNOB_X, CANOPY_DEPTH, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood_body = object_model.get_part("hood_body")
    speed_knob = object_model.get_part("speed_knob")
    knob_joint = object_model.get_articulation("hood_to_speed_knob")
    aabb = ctx.part_world_aabb(hood_body)

    ctx.check(
        "hood has residential chimney scale",
        aabb is not None
        and (aabb[1][0] - aabb[0][0]) >= 0.85
        and (aabb[1][1] - aabb[0][1]) >= 0.45
        and (aabb[1][2] - aabb[0][2]) >= 0.85,
        details=f"aabb={aabb}",
    )
    ctx.check(
        "speed knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"type={knob_joint.articulation_type}, limits={knob_joint.motion_limits}",
    )

    button_rest_positions = []
    for index in range(len(BUTTON_XS)):
        button = object_model.get_part(f"button_{index}")
        pos = ctx.part_world_position(button)
        button_rest_positions.append(pos)
        ctx.check(
            f"button_{index} sits on the front valance",
            pos is not None and pos[1] > 0.48 and 0.02 < pos[2] < 0.06,
            details=f"position={pos}",
        )

    knob_pos = ctx.part_world_position(speed_knob)
    ctx.check(
        "speed knob sits at the far right of the control row",
        knob_pos is not None
        and button_rest_positions[-1] is not None
        and knob_pos[0] > button_rest_positions[-1][0] + 0.30,
        details=f"knob={knob_pos}, buttons={button_rest_positions}",
    )
    with ctx.pose(hood_to_speed_knob=1.2):
        knob_rotated_pos = ctx.part_world_position(speed_knob)
    ctx.check(
        "speed knob rotates in place",
        knob_pos is not None
        and knob_rotated_pos is not None
        and abs(knob_rotated_pos[0] - knob_pos[0]) < 1e-6
        and abs(knob_rotated_pos[1] - knob_pos[1]) < 1e-6
        and abs(knob_rotated_pos[2] - knob_pos[2]) < 1e-6,
        details=f"rest={knob_pos}, rotated={knob_rotated_pos}",
    )

    rest_button_0 = button_rest_positions[0]
    rest_button_1 = button_rest_positions[1]
    rest_button_2 = button_rest_positions[2]
    with ctx.pose(hood_to_button_0=BUTTON_TRAVEL):
        pressed_button_0 = ctx.part_world_position("button_0")
        unchanged_button_1 = ctx.part_world_position("button_1")
        unchanged_button_2 = ctx.part_world_position("button_2")

    ctx.check(
        "push buttons articulate independently",
        rest_button_0 is not None
        and rest_button_1 is not None
        and rest_button_2 is not None
        and pressed_button_0 is not None
        and unchanged_button_1 is not None
        and unchanged_button_2 is not None
        and pressed_button_0[1] < rest_button_0[1] - 0.002
        and abs(unchanged_button_1[1] - rest_button_1[1]) < 1e-6
        and abs(unchanged_button_2[1] - rest_button_2[1]) < 1e-6,
        details=(
            f"rest0={rest_button_0}, pressed0={pressed_button_0}, "
            f"rest1={rest_button_1}, posed1={unchanged_button_1}, "
            f"rest2={rest_button_2}, posed2={unchanged_button_2}"
        ),
    )

    button_cap_aabb = ctx.part_element_world_aabb("button_0", elem="cap")
    knob_aabb = ctx.part_element_world_aabb(speed_knob, elem="knob")
    ctx.check(
        "speed knob reads larger than a push button",
        button_cap_aabb is not None
        and knob_aabb is not None
        and (knob_aabb[1][0] - knob_aabb[0][0]) > (button_cap_aabb[1][0] - button_cap_aabb[0][0]) * 1.5,
        details=f"button_cap={button_cap_aabb}, knob={knob_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
