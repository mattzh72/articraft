from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BlowerWheelGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_RADIUS = 0.170
BASE_THICKNESS = 0.030
PLINTH_RADIUS = 0.082
PLINTH_THICKNESS = 0.012
POST_RADIUS = 0.055
POST_HEIGHT = 0.024
OSCILLATION_Z = BASE_THICKNESS + PLINTH_THICKNESS + POST_HEIGHT

BODY_WIDTH = 0.148
BODY_DEPTH = 0.118
BODY_HEIGHT = 0.860
BODY_BOTTOM_Z = 0.020
BODY_CORNER_RADIUS = 0.018
SHELL_WALL = 0.005
SHELL_BOTTOM = 0.010
SHELL_TOP = 0.020
SHELL_CENTER_Z = BODY_BOTTOM_Z + BODY_HEIGHT * 0.5

FRONT_SLOT_WIDTH = 0.050
FRONT_SLOT_HEIGHT = 0.720
FRONT_SLOT_CENTER_Z = BODY_BOTTOM_Z + 0.100 + FRONT_SLOT_HEIGHT * 0.5
FRONT_BEZEL_WIDTH = 0.066
FRONT_BEZEL_HEIGHT = 0.736
FRONT_BEZEL_THICKNESS = 0.004
FRONT_BEZEL_FRAME = 0.006

REAR_GRILLE_WIDTH = 0.108
REAR_GRILLE_HEIGHT = 0.750
REAR_GRILLE_CENTER_Z = BODY_BOTTOM_Z + 0.085 + REAR_GRILLE_HEIGHT * 0.5
REAR_GRILLE_FRAME = 0.008
REAR_GRILLE_THICKNESS = 0.006
REAR_GRILLE_SLAT_WIDTH = 0.003
REAR_GRILLE_PITCH = 0.008

DECK_WIDTH = 0.112
DECK_DEPTH = 0.076
DECK_THICKNESS = 0.014
DECK_CORNER_RADIUS = 0.010
DECK_CENTER_Z = BODY_BOTTOM_Z + BODY_HEIGHT + DECK_THICKNESS * 0.5 - 0.002
DECK_TOP_Z = DECK_CENTER_Z + DECK_THICKNESS * 0.5

BLOWER_RADIUS = 0.034
BLOWER_HUB_RADIUS = 0.011
BLOWER_HEIGHT = 0.744
BLOWER_CENTER_Z = 0.445
BLOWER_OFFSET_Y = 0.012
SHAFT_RADIUS = 0.012
SHAFT_LENGTH = 0.762
BEARING_BEAM_WIDTH = 0.026
BEARING_BEAM_DEPTH = 0.054
BEARING_BEAM_HEIGHT = 0.010
BEARING_BEAM_CENTER_Y = -0.027
UPPER_BEARING_Z = BLOWER_CENTER_Z + SHAFT_LENGTH * 0.5 - BEARING_BEAM_HEIGHT * 0.5
LOWER_BEARING_Z = BLOWER_CENTER_Z - SHAFT_LENGTH * 0.5 + BEARING_BEAM_HEIGHT * 0.5

KNOB_RADIUS = 0.017
KNOB_HEIGHT = 0.018
KNOB_SHAFT_RADIUS = 0.005
KNOB_SHAFT_LENGTH = 0.010
KNOB_BUSHING_RADIUS = 0.013
KNOB_BUSHING_HEIGHT = 0.006
KNOB_Y = 0.004
KNOB_X = 0.028
KNOB_JOINT_Z = DECK_TOP_Z + KNOB_BUSHING_HEIGHT


def _body_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS)
        .translate((0.0, 0.0, SHELL_CENTER_Z))
    )

    inner_height = BODY_HEIGHT - SHELL_BOTTOM - SHELL_TOP
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_WIDTH - 2.0 * SHELL_WALL,
            BODY_DEPTH - 2.0 * SHELL_WALL,
            inner_height,
        )
        .edges("|Z")
        .fillet(max(BODY_CORNER_RADIUS - SHELL_WALL, 0.006))
        .translate((0.0, 0.0, BODY_BOTTOM_Z + SHELL_BOTTOM + inner_height * 0.5))
    )

    front_slot = cq.Workplane("XY").box(
        FRONT_SLOT_WIDTH,
        SHELL_WALL + 0.030,
        FRONT_SLOT_HEIGHT,
    ).translate((0.0, BODY_DEPTH * 0.5 - SHELL_WALL * 0.5, FRONT_SLOT_CENTER_Z))

    rear_grille_opening = cq.Workplane("XY").box(
        REAR_GRILLE_WIDTH,
        SHELL_WALL + 0.030,
        REAR_GRILLE_HEIGHT,
    ).translate((0.0, -BODY_DEPTH * 0.5 + SHELL_WALL * 0.5, REAR_GRILLE_CENTER_Z))

    return outer.cut(inner).cut(front_slot).cut(rear_grille_opening)


def _rear_grille_shape() -> cq.Workplane:
    panel_width = REAR_GRILLE_WIDTH + 2.0 * REAR_GRILLE_FRAME
    panel_height = REAR_GRILLE_HEIGHT + 2.0 * REAR_GRILLE_FRAME
    y_center = -BODY_DEPTH * 0.5 + REAR_GRILLE_THICKNESS * 0.5 + 0.0005

    shape = cq.Workplane("XY").box(
        REAR_GRILLE_FRAME,
        REAR_GRILLE_THICKNESS,
        panel_height,
    ).translate(
        (
            -panel_width * 0.5 + REAR_GRILLE_FRAME * 0.5,
            y_center,
            REAR_GRILLE_CENTER_Z,
        )
    )
    shape = shape.union(
        cq.Workplane("XY").box(
            REAR_GRILLE_FRAME,
            REAR_GRILLE_THICKNESS,
            panel_height,
        ).translate(
            (
                panel_width * 0.5 - REAR_GRILLE_FRAME * 0.5,
                y_center,
                REAR_GRILLE_CENTER_Z,
            )
        )
    )
    shape = shape.union(
        cq.Workplane("XY").box(
            panel_width,
            REAR_GRILLE_THICKNESS,
            REAR_GRILLE_FRAME,
        ).translate(
            (
                0.0,
                y_center,
                REAR_GRILLE_CENTER_Z + panel_height * 0.5 - REAR_GRILLE_FRAME * 0.5,
            )
        )
    )
    shape = shape.union(
        cq.Workplane("XY").box(
            panel_width,
            REAR_GRILLE_THICKNESS,
            REAR_GRILLE_FRAME,
        ).translate(
            (
                0.0,
                y_center,
                REAR_GRILLE_CENTER_Z - panel_height * 0.5 + REAR_GRILLE_FRAME * 0.5,
            )
        )
    )

    x = -REAR_GRILLE_WIDTH * 0.5 + REAR_GRILLE_PITCH * 0.5
    x_limit = REAR_GRILLE_WIDTH * 0.5 - REAR_GRILLE_PITCH * 0.5
    while x <= x_limit + 1e-9:
        shape = shape.union(
            cq.Workplane("XY").box(
                REAR_GRILLE_SLAT_WIDTH,
                REAR_GRILLE_THICKNESS,
                REAR_GRILLE_HEIGHT,
            ).translate((x, y_center, REAR_GRILLE_CENTER_Z))
        )
        x += REAR_GRILLE_PITCH

    return shape


def _front_bezel_shape() -> cq.Workplane:
    y_center = BODY_DEPTH * 0.5 - FRONT_BEZEL_THICKNESS * 0.5 + 0.0008
    outer = cq.Workplane("XY").box(
        FRONT_BEZEL_WIDTH,
        FRONT_BEZEL_THICKNESS,
        FRONT_BEZEL_HEIGHT,
    ).translate((0.0, y_center, FRONT_SLOT_CENTER_Z))
    inner = cq.Workplane("XY").box(
        FRONT_BEZEL_WIDTH - 2.0 * FRONT_BEZEL_FRAME,
        FRONT_BEZEL_THICKNESS + 0.004,
        FRONT_BEZEL_HEIGHT - 2.0 * FRONT_BEZEL_FRAME,
    ).translate((0.0, y_center, FRONT_SLOT_CENTER_Z))
    return outer.cut(inner)


def _control_deck_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(DECK_WIDTH, DECK_DEPTH, DECK_THICKNESS)
        .edges("|Z")
        .fillet(DECK_CORNER_RADIUS)
        .translate((0.0, 0.0, DECK_CENTER_Z))
    )


def _knob_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .circle(KNOB_SHAFT_RADIUS)
        .extrude(KNOB_SHAFT_LENGTH)
        .union(
            cq.Workplane("XY")
            .circle(KNOB_RADIUS)
            .extrude(KNOB_HEIGHT)
            .translate((0.0, 0.0, KNOB_SHAFT_LENGTH))
        )
    )

    ridge_radius = KNOB_RADIUS - 0.0015
    for ridge_index in range(18):
        angle_deg = ridge_index * 20.0
        angle_rad = math.radians(angle_deg)
        ridge = (
            cq.Workplane("XY")
            .box(0.003, 0.006, KNOB_HEIGHT * 0.82)
            .translate(
                (
                    ridge_radius * math.cos(angle_rad),
                    ridge_radius * math.sin(angle_rad),
                    KNOB_SHAFT_LENGTH + KNOB_HEIGHT * 0.5,
                )
            )
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        body = body.union(ridge)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    model.material("body_silver", rgba=(0.82, 0.84, 0.85, 1.0))
    model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("trim_black", rgba=(0.07, 0.08, 0.09, 1.0))
    model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("indicator_silver", rgba=(0.88, 0.89, 0.90, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material="charcoal",
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=PLINTH_RADIUS, length=PLINTH_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PLINTH_THICKNESS * 0.5)),
        material="trim_black",
        name="turntable_plinth",
    )
    pedestal.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + PLINTH_THICKNESS + POST_HEIGHT * 0.5)
        ),
        material="charcoal",
        name="oscillation_post",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material="body_silver",
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_rear_grille_shape(), "rear_grille"),
        material="trim_black",
        name="rear_grille",
    )
    body.visual(
        mesh_from_cadquery(_front_bezel_shape(), "front_bezel"),
        material="trim_black",
        name="front_bezel",
    )
    body.visual(
        mesh_from_cadquery(_control_deck_shape(), "control_deck"),
        material="trim_black",
        name="control_deck",
    )
    body.visual(
        Cylinder(radius=0.066, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material="trim_black",
        name="turntable_collar",
    )
    body.visual(
        Cylinder(radius=KNOB_BUSHING_RADIUS, length=KNOB_BUSHING_HEIGHT),
        origin=Origin(xyz=(-KNOB_X, KNOB_Y, DECK_TOP_Z + KNOB_BUSHING_HEIGHT * 0.5)),
        material="trim_black",
        name="knob_0_bushing",
    )
    body.visual(
        Cylinder(radius=KNOB_BUSHING_RADIUS, length=KNOB_BUSHING_HEIGHT),
        origin=Origin(xyz=(KNOB_X, KNOB_Y, DECK_TOP_Z + KNOB_BUSHING_HEIGHT * 0.5)),
        material="trim_black",
        name="knob_1_bushing",
    )
    body.visual(
        Box((BEARING_BEAM_WIDTH, BEARING_BEAM_DEPTH, BEARING_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, BEARING_BEAM_CENTER_Y, UPPER_BEARING_Z)),
        material="trim_black",
        name="upper_bearing",
    )
    body.visual(
        Box((BEARING_BEAM_WIDTH, BEARING_BEAM_DEPTH, BEARING_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, BEARING_BEAM_CENTER_Y, LOWER_BEARING_Z)),
        material="trim_black",
        name="lower_bearing",
    )

    blower = model.part("blower")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                BLOWER_RADIUS,
                BLOWER_HUB_RADIUS,
                BLOWER_HEIGHT,
                28,
                blade_thickness=0.0026,
                blade_sweep_deg=32.0,
                backplate=True,
                shroud=True,
                center=True,
            ),
            "blower_wheel",
        ),
        material="charcoal",
        name="blower_drum",
    )
    blower.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        material="trim_black",
        name="shaft",
    )

    knob_0 = model.part("knob_0")
    knob_0.visual(
        mesh_from_cadquery(_knob_shape(), "knob_0"),
        material="knob_black",
        name="knob_body",
    )
    knob_0.visual(
        Box((0.010, 0.0022, 0.0016)),
        origin=Origin(
            xyz=(0.0, KNOB_RADIUS * 0.55, KNOB_SHAFT_LENGTH + KNOB_HEIGHT - 0.0008)
        ),
        material="indicator_silver",
        name="indicator",
    )

    knob_1 = model.part("knob_1")
    knob_1.visual(
        mesh_from_cadquery(_knob_shape(), "knob_1"),
        material="knob_black",
        name="knob_body",
    )
    knob_1.visual(
        Box((0.010, 0.0022, 0.0016)),
        origin=Origin(
            xyz=(0.0, KNOB_RADIUS * 0.55, KNOB_SHAFT_LENGTH + KNOB_HEIGHT - 0.0008)
        ),
        material="indicator_silver",
        name="indicator",
    )

    model.articulation(
        "pedestal_to_body",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, OSCILLATION_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=-math.radians(70.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, BLOWER_OFFSET_Y, BLOWER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
    )
    model.articulation(
        "body_to_knob_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob_0,
        origin=Origin(xyz=(-KNOB_X, KNOB_Y, KNOB_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=4.0,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "body_to_knob_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob_1,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, KNOB_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=4.0,
            lower=-2.2,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    oscillation = object_model.get_articulation("pedestal_to_body")
    blower_spin = object_model.get_articulation("body_to_blower")

    ctx.expect_gap(
        body,
        pedestal,
        axis="z",
        min_gap=0.015,
        max_gap=0.040,
        positive_elem="shell",
        name="tower body clears the pedestal turntable",
    )
    ctx.expect_overlap(
        body,
        pedestal,
        axes="xy",
        min_overlap=0.080,
        name="tower body stays centered over the pedestal footprint",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xyz",
        inner_elem="blower_drum",
        outer_elem="shell",
        margin=0.0,
        name="blower wheel stays inside the tower shell",
    )
    ctx.expect_contact(
        knob_0,
        body,
        elem_a="knob_body",
        elem_b="knob_0_bushing",
        name="first knob is seated on its bushing",
    )
    ctx.expect_contact(
        knob_1,
        body,
        elem_a="knob_body",
        elem_b="knob_1_bushing",
        name="second knob is seated on its bushing",
    )
    ctx.expect_contact(
        blower,
        body,
        elem_a="shaft",
        elem_b="upper_bearing",
        name="blower shaft is supported by the upper bearing beam",
    )
    ctx.expect_contact(
        blower,
        body,
        elem_a="shaft",
        elem_b="lower_bearing",
        name="blower shaft is supported by the lower bearing beam",
    )

    rest_pos = ctx.part_world_position(knob_1)
    upper = oscillation.motion_limits.upper if oscillation.motion_limits else None
    upper_pos = None
    if upper is not None:
        with ctx.pose({oscillation: upper}):
            upper_pos = ctx.part_world_position(knob_1)
    ctx.check(
        "oscillation sweeps the tower to one side",
        rest_pos is not None
        and upper_pos is not None
        and upper_pos[1] > rest_pos[1] + 0.020,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )
    ctx.check(
        "blower uses continuous rotation",
        blower_spin.articulation_type == ArticulationType.CONTINUOUS
        and blower_spin.motion_limits is not None
        and blower_spin.motion_limits.lower is None
        and blower_spin.motion_limits.upper is None,
        details=f"type={blower_spin.articulation_type}, limits={blower_spin.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
