from __future__ import annotations

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


BODY_W = 0.132
BODY_H = 0.094
BODY_D = 0.024
BODY_EDGE_R = 0.004

SCREEN_W = 0.074
SCREEN_H = 0.025
SCREEN_Y = 0.024
SCREEN_RECESS = 0.0015

BUTTON_CAP_W = 0.018
BUTTON_CAP_H = 0.010
BUTTON_CAP_T = 0.0028
BUTTON_STEM_W = 0.0145
BUTTON_STEM_H = 0.0065
BUTTON_STEM_L = 0.0050
BUTTON_PROUD = 0.0007
BUTTON_TRAVEL = 0.0016
BUTTON_CLEARANCE = 0.0007
BUTTON_POCKET_D = 0.0105

BUTTON_POSITIONS = [
    (-0.026, -0.002),
    (0.0, -0.002),
    (0.026, -0.002),
    (-0.026, -0.016),
    (0.0, -0.016),
    (0.026, -0.016),
]

SLIDER_Y = -0.032
SLIDER_LEFT_X = -0.008
SLIDER_TRAVEL = 0.016
SLIDER_PROUD = 0.0006
SLIDER_THUMB_W = 0.011
SLIDER_THUMB_H = 0.007
SLIDER_THUMB_T = 0.0030
SLIDER_CARRIAGE_W = 0.017
SLIDER_CARRIAGE_H = 0.0045
SLIDER_CARRIAGE_L = 0.0053
SLIDER_SLOT_W = 0.035
SLIDER_SLOT_H = 0.0092
SLIDER_POCKET_D = 0.0096


def build_body_shape():
    body = cq.Workplane("XY").box(BODY_W, BODY_H, BODY_D).edges("|Z").fillet(BODY_EDGE_R)

    body = (
        body.faces(">Z")
        .workplane()
        .center(0.0, SCREEN_Y)
        .rect(SCREEN_W, SCREEN_H)
        .cutBlind(-SCREEN_RECESS)
    )

    body = (
        body.faces(">Z")
        .workplane()
        .pushPoints(BUTTON_POSITIONS)
        .rect(BUTTON_CAP_W + BUTTON_CLEARANCE, BUTTON_CAP_H + BUTTON_CLEARANCE)
        .cutBlind(-BUTTON_POCKET_D)
    )

    body = (
        body.faces(">Z")
        .workplane()
        .center(0.0, SLIDER_Y)
        .rect(SLIDER_SLOT_W, SLIDER_SLOT_H)
        .cutBlind(-SLIDER_POCKET_D)
    )

    return body


def add_button(
    model: ArticulatedObject,
    *,
    name: str,
    joint_name: str,
    x: float,
    y: float,
    cap_material,
    stem_material,
    body,
):
    button = model.part(name)
    button.visual(
        Box((BUTTON_CAP_W, BUTTON_CAP_H, BUTTON_CAP_T)),
        origin=Origin(xyz=(0.0, 0.0, -BUTTON_CAP_T / 2.0)),
        material=cap_material,
        name="cap",
    )
    button.visual(
        Box((BUTTON_STEM_W, BUTTON_STEM_H, BUTTON_STEM_L)),
        origin=Origin(xyz=(0.0, 0.0, -BUTTON_CAP_T - BUTTON_STEM_L / 2.0)),
        material=stem_material,
        name="stem",
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(x, y, BODY_D / 2.0 + BUTTON_PROUD)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="programmable_thermostat")

    shell_material = model.material("shell", rgba=(0.93, 0.92, 0.88, 1.0))
    button_material = model.material("button", rgba=(0.86, 0.85, 0.82, 1.0))
    stem_material = model.material("stem", rgba=(0.28, 0.30, 0.33, 1.0))
    slider_material = model.material("slider", rgba=(0.40, 0.42, 0.46, 1.0))
    slider_carriage_material = model.material("slider_carriage", rgba=(0.22, 0.24, 0.27, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shape(), "thermostat_body"),
        material=shell_material,
        name="shell",
    )

    for index, (x, y) in enumerate(BUTTON_POSITIONS):
        add_button(
            model,
            name=f"button_{index}",
            joint_name=f"body_to_button_{index}",
            x=x,
            y=y,
            cap_material=button_material,
            stem_material=stem_material,
            body=body,
        )

    slider = model.part("fan_slider")
    slider.visual(
        Box((SLIDER_THUMB_W, SLIDER_THUMB_H, SLIDER_THUMB_T)),
        origin=Origin(xyz=(0.0, 0.0, -SLIDER_THUMB_T / 2.0)),
        material=slider_material,
        name="thumb",
    )
    slider.visual(
        Box((SLIDER_CARRIAGE_W, SLIDER_CARRIAGE_H, SLIDER_CARRIAGE_L)),
        origin=Origin(xyz=(0.0, 0.0, -SLIDER_THUMB_T - SLIDER_CARRIAGE_L / 2.0)),
        material=slider_carriage_material,
        name="carriage",
    )

    model.articulation(
        "body_to_fan_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(SLIDER_LEFT_X, SLIDER_Y, BODY_D / 2.0 + SLIDER_PROUD)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=SLIDER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    slider = object_model.get_part("fan_slider")
    slider_joint = object_model.get_articulation("body_to_fan_slider")

    button_parts = [object_model.get_part(f"button_{index}") for index in range(len(BUTTON_POSITIONS))]
    button_joints = [
        object_model.get_articulation(f"body_to_button_{index}")
        for index in range(len(BUTTON_POSITIONS))
    ]

    for joint in button_joints + [slider_joint]:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC and limits is not None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    for index, button in enumerate(button_parts):
        rest_pos = ctx.part_world_position(button)
        other_button = button_parts[(index + 1) % len(button_parts)]
        other_rest_pos = ctx.part_world_position(other_button)

        with ctx.pose({button_joints[index]: BUTTON_TRAVEL}):
            depressed_pos = ctx.part_world_position(button)
            other_pos = ctx.part_world_position(other_button)

        ctx.check(
            f"{button.name} depresses inward",
            rest_pos is not None
            and depressed_pos is not None
            and depressed_pos[2] < rest_pos[2] - 0.0014,
            details=f"rest={rest_pos}, depressed={depressed_pos}",
        )
        ctx.check(
            f"{button.name} moves independently",
            other_rest_pos is not None
            and other_pos is not None
            and abs(other_pos[2] - other_rest_pos[2]) < 1e-6,
            details=f"other_rest={other_rest_pos}, other={other_pos}",
        )

    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        moved_slider_pos = ctx.part_world_position(slider)

    ctx.check(
        "fan slider translates rightward",
        rest_slider_pos is not None
        and moved_slider_pos is not None
        and moved_slider_pos[0] > rest_slider_pos[0] + 0.014,
        details=f"rest={rest_slider_pos}, moved={moved_slider_pos}",
    )
    ctx.check(
        "fan slider stays on lower front band",
        rest_slider_pos is not None
        and abs(rest_slider_pos[1] - SLIDER_Y) < 1e-6
        and rest_slider_pos[2] > BODY_D / 2.0 - 0.003,
        details=f"slider_rest={rest_slider_pos}",
    )

    return ctx.report()


object_model = build_object_model()
