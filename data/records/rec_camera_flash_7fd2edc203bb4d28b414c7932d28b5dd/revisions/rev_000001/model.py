from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.074
BODY_DEPTH = 0.046
BODY_HEIGHT = 0.102
BODY_BOTTOM_Z = 0.030
COLLAR_TOP_Z = 0.148

YOKE_TRUNNION_Z = 0.034

HEAD_WIDTH = 0.068
HEAD_DEPTH = 0.050
HEAD_HEIGHT = 0.040


def _body_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_BOTTOM_Z))
    )
    shell = shell.edges("|Z").fillet(0.006)

    neck = (
        cq.Workplane("XY")
        .box(0.056, 0.036, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT))
    )
    neck = neck.edges("|Z").fillet(0.004)

    collar = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(COLLAR_TOP_Z - (BODY_BOTTOM_Z + BODY_HEIGHT + 0.012))
        .translate((0.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT + 0.012))
    )

    foot_plate = (
        cq.Workplane("XY")
        .box(0.054, 0.042, 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0))
    )
    foot_rail = (
        cq.Workplane("XY")
        .box(0.022, 0.032, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, 0.006))
    )
    foot_stem = (
        cq.Workplane("XY")
        .box(0.032, 0.028, BODY_BOTTOM_Z - 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, 0.018))
    )

    body = shell.union(neck).union(collar).union(foot_plate).union(foot_rail).union(foot_stem)

    control_pocket = (
        cq.Workplane("XY")
        .box(0.056, 0.008, 0.074, centered=(True, True, False))
        .translate((0.0, -(BODY_DEPTH * 0.5) + 0.0035, 0.042))
    )
    return body.cut(control_pocket)


def _head_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(HEAD_WIDTH, HEAD_DEPTH, HEAD_HEIGHT, centered=(True, True, True))
        .translate((0.0, 0.018, 0.0))
    )
    shell = shell.edges("|Z").fillet(0.004)
    shell = shell.edges("|X").fillet(0.003)

    shell = (
        shell.faces(">Y")
        .workplane()
        .rect(0.056, 0.026)
        .cutBlind(0.002)
    )

    trunnions = cq.Workplane("YZ").circle(0.006).extrude(0.039, both=True)
    return shell.union(trunnions)


def _yoke_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(0.016).extrude(0.008)
    pedestal = (
        cq.Workplane("XY")
        .box(0.040, 0.024, 0.008, centered=(True, True, False))
        .translate((0.0, 0.0, 0.008))
    )
    spine = (
        cq.Workplane("XY")
        .box(0.018, 0.006, 0.022, centered=(True, True, False))
        .translate((0.0, -0.014, 0.016))
    )
    crossbar = (
        cq.Workplane("XY")
        .box(0.088, 0.006, 0.010, centered=(True, True, False))
        .translate((0.0, -0.014, 0.030))
    )
    return base.union(pedestal).union(spine).union(crossbar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_flash")

    model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("trim_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("diffuser", rgba=(0.92, 0.93, 0.95, 0.78))
    model.material("screen_glass", rgba=(0.26, 0.40, 0.46, 0.55))
    model.material("button_dark", rgba=(0.24, 0.24, 0.25, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "flash_body"),
        material="body_black",
        name="shell",
    )
    body.visual(
        Box((0.032, 0.0040, 0.022)),
        origin=Origin(xyz=(0.0, -0.0175, 0.090)),
        material="trim_dark",
        name="screen_frame",
    )
    body.visual(
        Box((0.028, 0.0018, 0.018)),
        origin=Origin(xyz=(0.0, -0.0196, 0.090)),
        material="screen_glass",
        name="screen",
    )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_cadquery(_yoke_shape(), "flash_yoke"),
        material="trim_dark",
        name="frame",
    )

    for index, arm_x in enumerate((-0.0435, 0.0435)):
        arm = model.part(f"arm_{index}")
        arm.visual(
            Box((0.007, 0.022, 0.036)),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material="trim_dark",
            name="arm",
        )
        arm.visual(
            Cylinder(radius=0.0065, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, YOKE_TRUNNION_Z), rpy=(0.0, math.pi * 0.5, 0.0)),
            material="trim_dark",
            name="pivot_cap",
        )
        model.articulation(
            f"yoke_to_arm_{index}",
            ArticulationType.FIXED,
            parent=yoke,
            child=arm,
            origin=Origin(xyz=(arm_x, 0.0, 0.008)),
        )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shape(), "flash_head"),
        material="body_black",
        name="shell",
    )
    head.visual(
        Box((0.054, 0.0030, 0.026)),
        origin=Origin(xyz=(0.0, 0.0410, 0.0)),
        material="diffuser",
        name="diffuser",
    )

    model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-math.radians(120.0),
            upper=math.radians(120.0),
        ),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, YOKE_TRUNNION_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-math.radians(15.0),
            upper=math.radians(110.0),
        ),
    )

    button_specs = (
        ("button_0", (-0.021, 0.100), (0.010, 0.0035, 0.008)),
        ("button_1", (-0.021, 0.082), (0.010, 0.0035, 0.008)),
        ("button_2", (0.021, 0.100), (0.010, 0.0035, 0.008)),
        ("button_3", (0.021, 0.082), (0.010, 0.0035, 0.008)),
        ("button_4", (-0.010, 0.064), (0.012, 0.0038, 0.006)),
        ("button_5", (0.010, 0.064), (0.012, 0.0038, 0.006)),
        ("button_6", (0.000, 0.049), (0.014, 0.0038, 0.008)),
    )
    for button_name, (button_x, button_z), button_size in button_specs:
        button = model.part(button_name)
        button.visual(
            Box(button_size),
            origin=Origin(xyz=(0.0, -(button_size[1] * 0.5), 0.0)),
            material="button_dark",
            name="cap",
        )
        button.visual(
            Box((min(button_size[0] * 0.5, 0.006), 0.0030, min(button_size[2] * 0.7, 0.005))),
            origin=Origin(xyz=(0.0, 0.0015, 0.0)),
            material="button_dark",
            name="stem",
        )
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, -0.0185, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=0.04,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_yoke")
    pitch = object_model.get_articulation("yoke_to_head")
    button_0 = object_model.get_part("button_0")
    button_6 = object_model.get_part("button_6")
    button_0_joint = object_model.get_articulation("body_to_button_0")

    ctx.allow_overlap(
        head,
        yoke,
        elem_a="shell",
        elem_b="frame",
        reason="The flash head trunnion assembly is intentionally represented as nesting into the swivel yoke support.",
    )

    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.010,
        name="head clears body at rest",
    )
    ctx.expect_overlap(
        yoke,
        body,
        axes="xy",
        min_overlap=0.020,
        name="yoke stays centered above body",
    )

    pitch_limits = pitch.motion_limits
    swivel_limits = swivel.motion_limits
    if pitch_limits is not None and pitch_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(head, elem="diffuser")
        with ctx.pose({pitch: pitch_limits.upper}):
            raised_aabb = ctx.part_element_world_aabb(head, elem="diffuser")
        ctx.check(
            "head pitches upward",
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.020,
            details=f"rest={rest_aabb}, raised={raised_aabb}",
        )

    if swivel_limits is not None and swivel_limits.upper is not None:
        rest_pos = ctx.part_element_world_aabb(head, elem="diffuser")
        with ctx.pose({swivel: swivel_limits.upper}):
            turned_pos = ctx.part_element_world_aabb(head, elem="diffuser")
        ctx.check(
            "head swivels away from centerline",
            rest_pos is not None
            and turned_pos is not None
            and abs((turned_pos[0][0] + turned_pos[1][0]) * 0.5)
            > abs((rest_pos[0][0] + rest_pos[1][0]) * 0.5) + 0.020,
            details=f"rest={rest_pos}, turned={turned_pos}",
        )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_6 = ctx.part_world_position(button_6)
    if button_0_joint.motion_limits is not None and button_0_joint.motion_limits.upper is not None:
        with ctx.pose({button_0_joint: button_0_joint.motion_limits.upper}):
            pressed_button_0 = ctx.part_world_position(button_0)
        ctx.check(
            "rear button presses inward",
            rest_button_0 is not None
            and pressed_button_0 is not None
            and pressed_button_0[1] > rest_button_0[1] + 0.001,
            details=f"rest={rest_button_0}, pressed={pressed_button_0}",
        )

    ctx.check(
        "button bank sits on rear panel",
        rest_button_0 is not None
        and rest_button_6 is not None
        and rest_button_0[1] < -0.015
        and rest_button_6[1] < -0.015
        and rest_button_0[2] > 0.075
        and rest_button_6[2] > 0.040,
        details=f"button_0={rest_button_0}, button_6={rest_button_6}",
    )

    return ctx.report()


object_model = build_object_model()
