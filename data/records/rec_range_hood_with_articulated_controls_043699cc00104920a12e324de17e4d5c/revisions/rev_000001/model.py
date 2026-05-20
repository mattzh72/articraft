from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CANOPY_WIDTH = 1.20
CANOPY_DEPTH = 0.70
CANOPY_HEIGHT = 0.36
SHELL_THICKNESS = 0.022
TOP_THICKNESS = 0.025
CONTROL_PANEL_WIDTH = 0.24
CONTROL_PANEL_HEIGHT = 0.20
CONTROL_PANEL_THICKNESS = 0.008
CONTROL_PANEL_Z = 0.205
CONTROL_FACE_Y = CANOPY_DEPTH / 2.0 + CONTROL_PANEL_THICKNESS / 2.0
CONTROL_OUTER_Y = CONTROL_FACE_Y + CONTROL_PANEL_THICKNESS / 2.0
CONTROL_RPY = (-math.pi / 2.0, 0.0, 0.0)

KNOB_X = 0.065
KNOB_Z = 0.255
SWITCH_Z = 0.155
SWITCH_WIDTH = 0.044
SWITCH_HEIGHT = 0.024
SWITCH_THICKNESS = 0.008
SWITCH_CENTER_Y = CONTROL_OUTER_Y + SWITCH_THICKNESS / 2.0


def control_origin(x: float, z: float, y: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=CONTROL_RPY)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="island_box_range_hood")

    stainless = model.material("stainless", rgba=(0.77, 0.78, 0.80, 1.0))
    trim = model.material("trim", rgba=(0.26, 0.27, 0.29, 1.0))
    control_black = model.material("control_black", rgba=(0.09, 0.09, 0.10, 1.0))
    switch_black = model.material("switch_black", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")

    wall_height = 0.338
    body.visual(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT - TOP_THICKNESS / 2.0)),
        material=stainless,
        name="canopy_top",
    )
    body.visual(
        Box((CANOPY_WIDTH, SHELL_THICKNESS, wall_height)),
        origin=Origin(xyz=(0.0, CANOPY_DEPTH / 2.0 - SHELL_THICKNESS / 2.0, wall_height / 2.0)),
        material=stainless,
        name="front_wall",
    )
    body.visual(
        Box((CANOPY_WIDTH, SHELL_THICKNESS, wall_height)),
        origin=Origin(xyz=(0.0, -CANOPY_DEPTH / 2.0 + SHELL_THICKNESS / 2.0, wall_height / 2.0)),
        material=stainless,
        name="rear_wall",
    )
    body.visual(
        Box((SHELL_THICKNESS, 0.666, wall_height)),
        origin=Origin(xyz=(CANOPY_WIDTH / 2.0 - SHELL_THICKNESS / 2.0, 0.0, wall_height / 2.0)),
        material=stainless,
        name="side_wall_0",
    )
    body.visual(
        Box((SHELL_THICKNESS, 0.666, wall_height)),
        origin=Origin(xyz=(-CANOPY_WIDTH / 2.0 + SHELL_THICKNESS / 2.0, 0.0, wall_height / 2.0)),
        material=stainless,
        name="side_wall_1",
    )
    body.visual(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, CONTROL_FACE_Y, CONTROL_PANEL_Z)),
        material=trim,
        name="control_panel",
    )

    collar_width = 0.34
    collar_depth = 0.24
    collar_height = 0.18
    collar_thickness = 0.018
    collar_center_z = CANOPY_HEIGHT + collar_height / 2.0 - 0.002
    body.visual(
        Box((collar_width, collar_thickness, collar_height)),
        origin=Origin(
            xyz=(0.0, collar_depth / 2.0 - collar_thickness / 2.0, collar_center_z),
        ),
        material=stainless,
        name="collar_front",
    )
    body.visual(
        Box((collar_width, collar_thickness, collar_height)),
        origin=Origin(
            xyz=(0.0, -collar_depth / 2.0 + collar_thickness / 2.0, collar_center_z),
        ),
        material=stainless,
        name="collar_rear",
    )
    body.visual(
        Box((collar_thickness, 0.212, collar_height)),
        origin=Origin(
            xyz=(collar_width / 2.0 - collar_thickness / 2.0, 0.0, collar_center_z),
        ),
        material=stainless,
        name="collar_side_0",
    )
    body.visual(
        Box((collar_thickness, 0.212, collar_height)),
        origin=Origin(
            xyz=(-collar_width / 2.0 + collar_thickness / 2.0, 0.0, collar_center_z),
        ),
        material=stainless,
        name="collar_side_1",
    )

    knob_geometry = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.026,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.056, 0.006, flare=0.05),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=0.0),
            center=False,
        ),
        "range_hood_knob",
    )

    for index, x_pos in enumerate((-KNOB_X, KNOB_X)):
        knob = model.part(f"knob_{index}")
        knob.visual(knob_geometry, material=control_black, name="knob_shell")
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=control_origin(x=x_pos, y=CONTROL_OUTER_Y, z=KNOB_Z),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=6.0),
        )

    for index, x_pos in enumerate((-KNOB_X, KNOB_X)):
        switch = model.part(f"switch_{index}")
        switch.visual(
            Box((SWITCH_WIDTH, SWITCH_HEIGHT, SWITCH_THICKNESS)),
            material=switch_black,
            name="switch_cap",
        )
        model.articulation(
            f"body_to_switch_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=switch,
            origin=control_origin(x=x_pos, y=SWITCH_CENTER_Y, z=SWITCH_Z),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=4.0,
                lower=-0.22,
                upper=0.22,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_knob = object_model.get_part("knob_0")
    right_knob = object_model.get_part("knob_1")
    left_switch = object_model.get_part("switch_0")
    right_switch = object_model.get_part("switch_1")
    left_knob_joint = object_model.get_articulation("body_to_knob_0")
    right_knob_joint = object_model.get_articulation("body_to_knob_1")
    left_switch_joint = object_model.get_articulation("body_to_switch_0")
    right_switch_joint = object_model.get_articulation("body_to_switch_1")

    ctx.expect_gap(
        left_knob,
        body,
        axis="y",
        positive_elem="knob_shell",
        negative_elem="control_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="left knob seats on the control panel",
    )
    ctx.expect_gap(
        right_knob,
        body,
        axis="y",
        positive_elem="knob_shell",
        negative_elem="control_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="right knob seats on the control panel",
    )
    ctx.expect_gap(
        left_switch,
        body,
        axis="y",
        positive_elem="switch_cap",
        negative_elem="control_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="left rocker sits on the control panel",
    )
    ctx.expect_gap(
        right_switch,
        body,
        axis="y",
        positive_elem="switch_cap",
        negative_elem="control_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="right rocker sits on the control panel",
    )
    ctx.expect_origin_gap(
        left_knob,
        left_switch,
        axis="z",
        min_gap=0.08,
        max_gap=0.12,
        name="left knob is above left rocker",
    )
    ctx.expect_origin_gap(
        right_knob,
        right_switch,
        axis="z",
        min_gap=0.08,
        max_gap=0.12,
        name="right knob is above right rocker",
    )

    ctx.check(
        "both knobs use continuous rotation",
        left_knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and left_knob_joint.motion_limits is not None
        and right_knob_joint.motion_limits is not None
        and left_knob_joint.motion_limits.lower is None
        and left_knob_joint.motion_limits.upper is None
        and right_knob_joint.motion_limits.lower is None
        and right_knob_joint.motion_limits.upper is None,
        details=(
            f"left={left_knob_joint.articulation_type}, limits={left_knob_joint.motion_limits}; "
            f"right={right_knob_joint.articulation_type}, limits={right_knob_joint.motion_limits}"
        ),
    )
    ctx.check(
        "both rocker switches have realistic travel limits",
        left_switch_joint.motion_limits is not None
        and right_switch_joint.motion_limits is not None
        and left_switch_joint.motion_limits.lower is not None
        and left_switch_joint.motion_limits.upper is not None
        and right_switch_joint.motion_limits.lower is not None
        and right_switch_joint.motion_limits.upper is not None
        and -0.30 <= left_switch_joint.motion_limits.lower < 0.0 < left_switch_joint.motion_limits.upper <= 0.30
        and -0.30 <= right_switch_joint.motion_limits.lower < 0.0 < right_switch_joint.motion_limits.upper <= 0.30,
        details=(
            f"left={left_switch_joint.motion_limits}, right={right_switch_joint.motion_limits}"
        ),
    )

    body_top_aabb = ctx.part_element_world_aabb(body, elem="canopy_top")
    collar_aabb = ctx.part_element_world_aabb(body, elem="collar_front")
    ctx.check(
        "chimney collar rises above the canopy",
        body_top_aabb is not None
        and collar_aabb is not None
        and collar_aabb[1][2] > body_top_aabb[1][2] + 0.12,
        details=f"canopy_top={body_top_aabb}, collar_front={collar_aabb}",
    )

    rest_aabb = ctx.part_world_aabb(left_switch)
    with ctx.pose({left_switch_joint: left_switch_joint.motion_limits.upper}):
        actuated_aabb = ctx.part_world_aabb(left_switch)
    ctx.check(
        "rocker switch tilts outward when actuated",
        rest_aabb is not None
        and actuated_aabb is not None
        and actuated_aabb[1][1] > rest_aabb[1][1] + 0.002,
        details=f"rest={rest_aabb}, actuated={actuated_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
