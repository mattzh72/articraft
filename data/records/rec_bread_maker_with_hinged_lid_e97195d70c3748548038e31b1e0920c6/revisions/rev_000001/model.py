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


BODY_W = 0.330
BODY_D = 0.285
BODY_H = 0.345

OPEN_Y = 0.020
DEEP_CAVITY_W = 0.222
DEEP_CAVITY_D = 0.162
DEEP_CAVITY_Z = 0.090
MOUTH_W = 0.206
MOUTH_D = 0.146
MOUTH_H = 0.028

CHAMBER_W = 0.198
CHAMBER_D = 0.138
CHAMBER_H = 0.220
CHAMBER_FLANGE_W = 0.214
CHAMBER_FLANGE_D = 0.154
CHAMBER_FLANGE_T = 0.004
CHAMBER_TOTAL_H = CHAMBER_H + CHAMBER_FLANGE_T
CHAMBER_Z = BODY_H - MOUTH_H - CHAMBER_H

HINGE_Y = 0.130
HINGE_Z = BODY_H
LID_W = 0.300
LID_D = 0.218
LID_H = 0.030


def _body_shell_shape():
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )

    deep_cavity = (
        cq.Workplane("XY")
        .box(
            DEEP_CAVITY_W,
            DEEP_CAVITY_D,
            BODY_H - MOUTH_H - DEEP_CAVITY_Z,
            centered=(True, True, False),
        )
        .translate((0.0, OPEN_Y, DEEP_CAVITY_Z))
    )
    mouth = (
        cq.Workplane("XY")
        .box(MOUTH_W, MOUTH_D, MOUTH_H + 0.004, centered=(True, True, False))
        .translate((0.0, OPEN_Y, BODY_H - MOUTH_H))
    )

    return shell.cut(deep_cavity).cut(mouth)


def _chamber_shape():
    outer = (
        cq.Workplane("XY")
        .box(CHAMBER_W, CHAMBER_D, CHAMBER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
    )
    flange = (
        cq.Workplane("XY")
        .box(
            CHAMBER_FLANGE_W,
            CHAMBER_FLANGE_D,
            CHAMBER_FLANGE_T,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CHAMBER_H))
    )
    inner = (
        cq.Workplane("XY")
        .box(CHAMBER_W - 0.012, CHAMBER_D - 0.012, CHAMBER_H, centered=(True, True, False))
        .translate((0.0, 0.0, 0.006))
    )
    return outer.union(flange).cut(inner)


def _lid_shape():
    frame = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_H, centered=(True, False, False))
        .translate((0.0, -LID_D, 0.0))
        .edges("|Z")
        .fillet(0.010)
    )
    window = (
        cq.Workplane("XY")
        .box(0.184, 0.108, LID_H + 0.006, centered=(True, True, False))
        .translate((0.0, -0.122, -0.003))
    )
    barrel = (
        cq.Workplane("YZ")
        .center(-0.004, 0.010)
        .circle(0.008)
        .extrude(0.240, both=True)
    )
    return frame.cut(window).union(barrel)


def _paddle_shape():
    blade = (
        cq.Workplane("XY")
        .box(0.064, 0.018, 0.008, centered=(True, True, False))
        .translate((0.004, 0.0, 0.000))
        .edges("|Z")
        .fillet(0.003)
    )
    raised_wing = (
        cq.Workplane("XY")
        .box(0.030, 0.018, 0.005, centered=(True, True, False))
        .translate((0.015, 0.0, 0.008))
        .edges("|Z")
        .fillet(0.002)
    )
    hub = cq.Workplane("XY").circle(0.011).extrude(0.010)
    return blade.union(raised_wing).union(hub)


def _dial_shape():
    skirt = cq.Workplane("XY").circle(0.028).extrude(0.004)
    body = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(0.024)
        .workplane(offset=0.014)
        .circle(0.020)
        .loft(combine=True)
    )
    return skirt.union(body)


def _front_button_shape(width: float, depth: float, height: float):
    cap = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, False, True))
        .translate((0.0, -depth, 0.0))
        .edges("|Z")
        .fillet(min(width, height) * 0.14)
    )
    face = (
        cq.Workplane("XY")
        .box(width * 0.86, depth * 0.35, height * 0.78, centered=(True, False, True))
        .translate((0.0, -depth * 1.12, 0.0))
        .edges("|Z")
        .fillet(min(width, height) * 0.10)
    )
    return cap.union(face)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bread_maker")

    body_finish = model.material("body_finish", rgba=(0.92, 0.92, 0.88, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    chamber_finish = model.material("chamber_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.28, 0.34, 0.38, 0.40))
    metal_dark = model.material("metal_dark", rgba=(0.33, 0.33, 0.35, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "bread_maker_body"),
        material=body_finish,
        name="shell",
    )
    body.visual(
        Box((0.250, 0.018, 0.076)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + 0.007, 0.265)),
        material=panel_finish,
        name="control_strip",
    )
    body.visual(
        Box((0.245, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.004, BODY_H - 0.005)),
        material=panel_finish,
        name="hinge_bridge",
    )
    for index, x_pos in enumerate((-0.145, 0.145)):
        body.visual(
            Box((0.014, 0.010, 0.014)),
            origin=Origin(xyz=(x_pos, BODY_D / 2.0 - 0.005, BODY_H + 0.004)),
            material=panel_finish,
            name=f"hinge_bracket_{index}",
        )
    for index, xy in enumerate(
        (
            (-0.118, -0.094),
            (0.118, -0.094),
            (-0.118, 0.094),
            (0.118, 0.094),
        )
    ):
        body.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(xy[0], xy[1], 0.004)),
            material=panel_finish,
            name=f"foot_{index}",
        )

    body.visual(
        mesh_from_cadquery(_chamber_shape(), "bread_maker_chamber"),
        origin=Origin(xyz=(0.0, OPEN_Y, CHAMBER_Z)),
        material=chamber_finish,
        name="chamber_liner",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal_dark,
        name="base_collar",
    )
    spindle.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=metal_dark,
        name="shaft",
    )
    model.articulation(
        "body_to_spindle",
        ArticulationType.FIXED,
        parent=body,
        child=spindle,
        origin=Origin(xyz=(0.0, OPEN_Y, CHAMBER_Z + 0.006)),
    )

    paddle = model.part("paddle")
    paddle.visual(
        mesh_from_cadquery(_paddle_shape(), "bread_maker_paddle"),
        material=metal_dark,
        name="blade",
    )
    model.articulation(
        "spindle_to_paddle",
        ArticulationType.CONTINUOUS,
        parent=spindle,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "bread_maker_lid"),
        material=body_finish,
        name="lid_shell",
    )
    lid.visual(
        Box((0.188, 0.112, 0.003)),
        origin=Origin(xyz=(0.0, -0.122, 0.014)),
        material=glass_finish,
        name="window",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_cadquery(_dial_shape(), "bread_maker_timer_dial"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_finish,
        name="dial_body",
    )
    timer_dial.visual(
        Box((0.003, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.020, 0.014)),
        material=glass_finish,
        name="pointer",
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(-0.074, -BODY_D / 2.0 - 0.002, 0.265)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    for index, x_pos in enumerate((0.020, 0.060, 0.100)):
        button = model.part(f"program_button_{index}")
        button.visual(
            mesh_from_cadquery(
                _front_button_shape(0.026, 0.011, 0.017),
                f"bread_maker_program_button_{index}",
            ),
            material=body_finish,
            name="cap",
        )
        model.articulation(
            f"body_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, -BODY_D / 2.0 - 0.002, 0.269)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.003,
            ),
        )

    latch_button = model.part("latch_button")
    latch_button.visual(
        mesh_from_cadquery(
            _front_button_shape(0.052, 0.013, 0.018),
            "bread_maker_latch_button",
        ),
        material=panel_finish,
        name="cap",
    )
    model.articulation(
        "body_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch_button,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, 0.316)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.05,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    spindle = object_model.get_part("spindle")
    paddle = object_model.get_part("paddle")
    lid = object_model.get_part("lid")
    timer_dial = object_model.get_part("timer_dial")
    latch_button = object_model.get_part("latch_button")
    program_buttons = [object_model.get_part(f"program_button_{index}") for index in range(3)]
    lid_hinge = object_model.get_articulation("body_to_lid")
    paddle_joint = object_model.get_articulation("spindle_to_paddle")
    dial_joint = object_model.get_articulation("body_to_timer_dial")
    latch_joint = object_model.get_articulation("body_to_latch_button")
    button_joints = [
        object_model.get_articulation(f"body_to_program_button_{index}")
        for index in range(3)
    ]

    ctx.expect_contact(
        spindle,
        body,
        elem_a="base_collar",
        elem_b="chamber_liner",
        name="spindle mounts on the chamber floor",
    )
    ctx.expect_within(
        paddle,
        body,
        axes="xy",
        outer_elem="chamber_liner",
        margin=0.01,
        name="paddle stays centered inside the loaf chamber",
    )
    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.004,
            negative_elem="shell",
            name="closed lid sits down onto the top deck",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.18,
            name="closed lid covers the top opening area",
        )

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward",
            closed_aabb is not None
            and open_aabb is not None
            and float(open_aabb[1][2]) > float(closed_aabb[1][2]) + 0.08,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    dial_pointer_rest = ctx.part_element_world_aabb(timer_dial, elem="pointer")
    with ctx.pose({dial_joint: 1.1}):
        dial_pointer_turned = ctx.part_element_world_aabb(timer_dial, elem="pointer")
    ctx.check(
        "timer dial rotates continuously",
        dial_pointer_rest is not None
        and dial_pointer_turned is not None
        and abs(float(dial_pointer_turned[0][0]) - float(dial_pointer_rest[0][0])) > 0.004,
        details=f"rest={dial_pointer_rest}, turned={dial_pointer_turned}",
    )

    paddle_rest = ctx.part_world_aabb(paddle)
    with ctx.pose({paddle_joint: 0.9}):
        paddle_turned = ctx.part_world_aabb(paddle)
    ctx.check(
        "kneading paddle rotates on the spindle",
        paddle_rest is not None
        and paddle_turned is not None
        and abs(float(paddle_turned[0][1]) - float(paddle_rest[0][1])) > 0.01,
        details=f"rest={paddle_rest}, turned={paddle_turned}",
    )

    for index, (button, joint) in enumerate(zip(program_buttons, button_joints)):
        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        moved_pos = None
        if upper is not None:
            with ctx.pose({joint: upper}):
                moved_pos = ctx.part_world_position(button)
        ctx.check(
            f"program_button_{index} presses inward",
            rest_pos is not None
            and moved_pos is not None
            and float(moved_pos[1]) > float(rest_pos[1]) + 0.002,
            details=f"rest={rest_pos}, moved={moved_pos}",
        )

    latch_rest = ctx.part_world_position(latch_button)
    latch_upper = latch_joint.motion_limits.upper if latch_joint.motion_limits is not None else None
    latch_pressed = None
    if latch_upper is not None:
        with ctx.pose({latch_joint: latch_upper}):
            latch_pressed = ctx.part_world_position(latch_button)
    ctx.check(
        "front latch button presses into the shell",
        latch_rest is not None
        and latch_pressed is not None
        and float(latch_pressed[1]) > float(latch_rest[1]) + 0.0025,
        details=f"rest={latch_rest}, pressed={latch_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
