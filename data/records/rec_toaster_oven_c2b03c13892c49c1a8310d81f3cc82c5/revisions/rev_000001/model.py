from __future__ import annotations

import math

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

BODY_W = 0.46
BODY_D = 0.36
BODY_H = 0.29
FRONT_Y = BODY_D * 0.5

OPEN_W = 0.308
OPEN_H = 0.206
OPEN_CX = -0.056
OPEN_BOTTOM = 0.048
CAVITY_D = 0.300

CONTROL_W = 0.104
CONTROL_H = 0.232
CONTROL_T = 0.004
CONTROL_X = 0.147
CONTROL_FRONT = FRONT_Y + CONTROL_T - 0.0005

DOOR_W = OPEN_W + 0.016
DOOR_H = OPEN_H + 0.016
DOOR_T = 0.020

KNOB_RADIUS = 0.026
KNOB_Z = (0.205, 0.128)
KNOB_X = CONTROL_X

SELECTOR_X = CONTROL_X + CONTROL_W * 0.5 + 0.024
SELECTOR_Z = 0.112


def _body_shell() -> object:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).translate((0.0, 0.0, BODY_H * 0.5))
    outer = outer.edges("|Z").fillet(0.016)

    cavity = (
        cq.Workplane("XY")
        .box(OPEN_W - 0.010, CAVITY_D, OPEN_H)
        .translate((OPEN_CX, FRONT_Y - CAVITY_D * 0.5, OPEN_BOTTOM + OPEN_H * 0.5))
    )
    return outer.cut(cavity)


def _door_frame() -> object:
    frame = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((0.0, DOOR_T * 0.5, DOOR_H * 0.5))
    )
    window = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.090, DOOR_T * 1.8, DOOR_H - 0.082)
        .translate((0.0, DOOR_T * 0.5, DOOR_H * 0.56))
    )
    return frame.cut(window)


def _knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.027,
            body_style="skirted",
            top_diameter=0.035,
            skirt=KnobSkirt(0.052, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0011),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_toaster_oven")

    shell_finish = model.material("shell_finish", rgba=(0.74, 0.75, 0.76, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.20, 0.28, 0.32, 0.38))
    handle_finish = model.material("handle_finish", rgba=(0.62, 0.64, 0.66, 1.0))
    foot_finish = model.material("foot_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    lever_finish = model.material("lever_finish", rgba=(0.13, 0.13, 0.14, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shell(), "toaster_body_shell"), material=shell_finish, name="shell")
    body.visual(
        Box((CONTROL_W, CONTROL_T, CONTROL_H)),
        origin=Origin(xyz=(CONTROL_X, FRONT_Y + CONTROL_T * 0.5 - 0.0005, BODY_H * 0.5 + 0.010)),
        material=trim_finish,
        name="panel",
    )
    body.visual(
        Box((0.034, CONTROL_T, 0.086)),
        origin=Origin(xyz=(SELECTOR_X, FRONT_Y + CONTROL_T * 0.5 - 0.0005, SELECTOR_Z)),
        material=trim_finish,
        name="selector_mount",
    )
    for index, z in enumerate(KNOB_Z):
        body.visual(
            Cylinder(radius=KNOB_RADIUS, length=0.0035),
            origin=Origin(
                xyz=(KNOB_X, FRONT_Y + 0.0012, z),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=trim_finish,
            name=f"bezel_{index}",
        )
    for index, (x, y) in enumerate(((-0.176, -0.132), (0.176, -0.132), (-0.176, 0.132), (0.176, 0.132))):
        body.visual(
            Box((0.034, 0.026, 0.014)),
            origin=Origin(xyz=(x, y, 0.005)),
            material=foot_finish,
            name=f"foot_{index}",
        )

    door = model.part("door")
    door.visual(mesh_from_cadquery(_door_frame(), "toaster_door_frame"), material=trim_finish, name="frame")
    door.visual(
        Box((DOOR_W - 0.040, 0.012, DOOR_H - 0.035)),
        origin=Origin(xyz=(0.0, 0.008, DOOR_H * 0.56)),
        material=glass_finish,
        name="glass",
    )
    for index, x in enumerate((-0.090, 0.090)):
        door.visual(
            Cylinder(radius=0.008, length=0.018),
            origin=Origin(
                xyz=((-0.122 if index == 0 else 0.122), DOOR_T + 0.009, DOOR_H * 0.70),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=handle_finish,
            name=f"standoff_{index}",
        )
    door.visual(
        Box((0.252, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, DOOR_T + 0.018, DOOR_H * 0.70)),
        material=handle_finish,
        name="handle",
    )

    knob_cap_mesh = _knob_mesh("toaster_knob")
    for index, z in enumerate(KNOB_Z):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.0065, length=0.016),
            origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=trim_finish,
            name="shaft",
        )
        knob.visual(
            knob_cap_mesh,
            origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_finish,
            name="cap",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(KNOB_X, CONTROL_FRONT, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=6.0),
        )

    selector = model.part("selector_lever")
    selector.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=lever_finish,
        name="pivot",
    )
    selector.visual(
        Box((0.026, 0.008, 0.014)),
        origin=Origin(xyz=(-0.013, 0.010, -0.010)),
        material=lever_finish,
        name="neck",
    )
    selector.visual(
        Box((0.034, 0.008, 0.017)),
        origin=Origin(xyz=(-0.028, 0.012, -0.021)),
        material=lever_finish,
        name="paddle",
    )
    selector.visual(
        Cylinder(radius=0.007, length=0.009),
        origin=Origin(xyz=(-0.046, 0.012, -0.021), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=lever_finish,
        name="tip",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(OPEN_CX, FRONT_Y + 0.001, OPEN_BOTTOM)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector,
        origin=Origin(xyz=(SELECTOR_X, CONTROL_FRONT, SELECTOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=2.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    selector = object_model.get_part("selector_lever")

    door_hinge = object_model.get_articulation("body_to_door")
    knob_0_joint = object_model.get_articulation("body_to_knob_0")
    knob_1_joint = object_model.get_articulation("body_to_knob_1")
    selector_joint = object_model.get_articulation("body_to_selector")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="frame",
            negative_elem="shell",
            min_gap=0.0003,
            max_gap=0.004,
            name="door sits just ahead of the front shell",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            elem_a="frame",
            elem_b="shell",
            min_overlap=0.28,
            name="door spans most of the oven opening width",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="z",
            elem_a="frame",
            elem_b="shell",
            min_overlap=0.18,
            name="door covers the oven opening height",
        )
        ctx.expect_gap(
            knob_0,
            body,
            axis="y",
            positive_elem="shaft",
            negative_elem="panel",
            min_gap=0.0,
            max_gap=0.0005,
            name="upper knob shaft seats on the control panel",
        )
        ctx.expect_gap(
            knob_1,
            body,
            axis="y",
            positive_elem="shaft",
            negative_elem="panel",
            min_gap=0.0,
            max_gap=0.0005,
            name="lower knob shaft seats on the control panel",
        )
        ctx.expect_gap(
            selector,
            body,
            axis="y",
            positive_elem="pivot",
            negative_elem="selector_mount",
            min_gap=0.0,
            max_gap=0.0005,
            name="selector pivot seats on its side mount",
        )

    ctx.check(
        "knob_0 uses a continuous rotary joint",
        knob_0_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_0_joint.motion_limits is not None
        and knob_0_joint.motion_limits.lower is None
        and knob_0_joint.motion_limits.upper is None,
        details=f"type={knob_0_joint.articulation_type}, limits={knob_0_joint.motion_limits}",
    )
    ctx.check(
        "knob_1 uses a continuous rotary joint",
        knob_1_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_1_joint.motion_limits is not None
        and knob_1_joint.motion_limits.lower is None
        and knob_1_joint.motion_limits.upper is None,
        details=f"type={knob_1_joint.articulation_type}, limits={knob_1_joint.motion_limits}",
    )

    closed_handle = None
    open_handle = None
    with ctx.pose({door_hinge: 0.0}):
        closed_handle = ctx.part_element_world_aabb(door, elem="handle")
    with ctx.pose({door_hinge: 1.35}):
        open_handle = ctx.part_element_world_aabb(door, elem="handle")
    ctx.check(
        "door opens downward and outward",
        closed_handle is not None
        and open_handle is not None
        and open_handle[1][1] > closed_handle[1][1] + 0.10
        and open_handle[1][2] < closed_handle[1][2] - 0.09,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    lower_tip = None
    upper_tip = None
    selector_lower = -0.55
    selector_upper = 0.55
    with ctx.pose({selector_joint: selector_lower}):
        lower_tip = ctx.part_element_world_aabb(selector, elem="tip")
    with ctx.pose({selector_joint: selector_upper}):
        upper_tip = ctx.part_element_world_aabb(selector, elem="tip")
    ctx.check(
        "selector lever sweeps through a visible arc",
        lower_tip is not None
        and upper_tip is not None
        and upper_tip[1][2] > lower_tip[1][2] + 0.020,
        details=f"lower_tip={lower_tip}, upper_tip={upper_tip}",
    )

    return ctx.report()


object_model = build_object_model()
