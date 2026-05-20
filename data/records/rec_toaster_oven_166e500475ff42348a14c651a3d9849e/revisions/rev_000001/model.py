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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.48
BODY_D = 0.36
BODY_H = 0.29

CAVITY_W = 0.335
CAVITY_D = 0.325
CAVITY_H = 0.175
CAVITY_X = -0.025
CAVITY_Y = -BODY_D / 2.0 + CAVITY_D / 2.0
CAVITY_Z = 0.005

DOOR_W = 0.345
DOOR_H = 0.205
DOOR_T = 0.022
DOOR_HINGE_Z = CAVITY_Z - CAVITY_H / 2.0 - 0.008
DOOR_Y = -BODY_D / 2.0 - DOOR_T / 2.0 - 0.003
DOOR_OPEN = 1.45

RACK_W = 0.302
RACK_D = 0.285
RACK_R = 0.0032
RACK_Z = -0.018
RACK_Y = -BODY_D / 2.0 + 0.028
RACK_TRAVEL = 0.145

KNOB_X = 0.172
TOP_KNOB_Z = 0.052
BOTTOM_KNOB_Z = -0.030
KNOB_SHAFT_LEN = 0.012
KNOB_AXIS_Y = -BODY_D / 2.0


def _body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    shell = shell.edges("|Z").fillet(0.010)
    cavity = (
        cq.Workplane("XY")
        .box(CAVITY_W, CAVITY_D, CAVITY_H)
        .translate((CAVITY_X, CAVITY_Y, CAVITY_Z))
    )
    return shell.cut(cavity)


def _door_frame() -> cq.Workplane:
    frame = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((0.0, 0.0, DOOR_H / 2.0))
    )
    window = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.090, DOOR_T + 0.010, DOOR_H - 0.090)
        .translate((0.0, 0.0, DOOR_H * 0.56))
    )
    return frame.cut(window)


def _rack_bar(length: float, *, axis: str) -> Cylinder:
    if axis == "x":
        return Cylinder(radius=RACK_R, length=length)
    if axis == "y":
        return Cylinder(radius=RACK_R, length=length)
    if axis == "z":
        return Cylinder(radius=RACK_R * 0.90, length=length)
    raise ValueError(axis)


def _knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.040,
            0.023,
            body_style="skirted",
            top_diameter=0.030,
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(
                style="line",
                mode="engraved",
                depth=0.0007,
                angle_deg=0.0,
            ),
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toaster_oven")

    shell_finish = model.material("shell_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.72, 0.74, 0.76, 1.0))
    rack_finish = model.material("rack_finish", rgba=(0.74, 0.76, 0.79, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.18, 0.21, 0.24, 0.75))
    rubber_finish = model.material("rubber_finish", rgba=(0.06, 0.06, 0.07, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "toaster_oven_body"),
        material=shell_finish,
        name="shell",
    )

    for index, x in enumerate((-0.175, 0.175)):
        body.visual(
            Cylinder(radius=0.032, length=0.004),
            origin=Origin(
                xyz=(KNOB_X, -BODY_D / 2.0 - 0.001, TOP_KNOB_Z if index == 0 else BOTTOM_KNOB_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim_finish,
            name=f"knob_bezel_{index}",
        )

    guide_y = RACK_Y + (RACK_D / 2.0)
    guide_z = RACK_Z + 0.005
    guide_positions = (
        CAVITY_X - (RACK_W / 2.0 + 0.005),
        CAVITY_X + (RACK_W / 2.0 + 0.005),
    )
    for index, x in enumerate(guide_positions):
        body.visual(
            Box((0.024, RACK_D, 0.004)),
            origin=Origin(xyz=(x, guide_y, guide_z)),
            material=trim_finish,
            name=f"rack_guide_{index}",
        )

    for index, (x, y) in enumerate(
        (
            (-0.175, -0.125),
            (0.175, -0.125),
            (-0.175, 0.125),
            (0.175, 0.125),
        )
    ):
        body.visual(
            Box((0.028, 0.020, 0.010)),
            origin=Origin(xyz=(x, y, -BODY_H / 2.0 - 0.004)),
            material=rubber_finish,
            name=f"foot_{index}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_frame(), "toaster_oven_door"),
        material=trim_finish,
        name="frame",
    )
    door.visual(
        Box((DOOR_W - 0.070, 0.006, DOOR_H - 0.070)),
        origin=Origin(xyz=(0.0, 0.003, DOOR_H * 0.56)),
        material=glass_finish,
        name="glass",
    )
    door.visual(
        Cylinder(radius=0.006, length=DOOR_W * 0.86),
        origin=Origin(
            xyz=(0.0, -0.002, 0.006),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_finish,
        name="hinge_barrel",
    )
    handle_z = DOOR_H * 0.74
    for index, x in enumerate((-0.132, 0.132)):
        door.visual(
            Box((0.016, 0.030, 0.020)),
            origin=Origin(xyz=(x, -0.016, handle_z)),
            material=trim_finish,
            name=f"handle_post_{index}",
        )
    door.visual(
        Cylinder(radius=0.009, length=0.280),
        origin=Origin(
            xyz=(0.0, -0.029, handle_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_finish,
        name="pull_handle",
    )

    rack = model.part("rack")
    for index, x in enumerate((-RACK_W / 2.0, RACK_W / 2.0)):
        rack.visual(
            _rack_bar(RACK_D, axis="y"),
            origin=Origin(
                xyz=(x, RACK_D / 2.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rack_finish,
            name=f"side_rail_{index}",
        )
    rack.visual(
        _rack_bar(RACK_W, axis="x"),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rack_finish,
        name="front_rail",
    )
    rack.visual(
        _rack_bar(RACK_W, axis="x"),
        origin=Origin(
            xyz=(0.0, RACK_D, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rack_finish,
        name="rear_rail",
    )
    for index, y in enumerate((0.045, 0.095, 0.145, 0.195, 0.245)):
        rack.visual(
            _rack_bar(RACK_W, axis="x"),
            origin=Origin(
                xyz=(0.0, y, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rack_finish,
            name=f"wire_{index}",
        )
    for index, x in enumerate((-0.105, 0.105)):
        rack.visual(
            _rack_bar(0.014, axis="z"),
            origin=Origin(xyz=(x, -0.0035, 0.009)),
            material=rack_finish,
            name=f"handle_strut_{index}",
        )
    rack.visual(
        Cylinder(radius=0.0042, length=0.230),
        origin=Origin(
            xyz=(0.0, -0.0035, 0.018),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rack_finish,
        name="pull_bar",
    )

    top_knob = model.part("top_knob")
    top_knob.visual(
        Cylinder(radius=0.006, length=KNOB_SHAFT_LEN),
        origin=Origin(
            xyz=(0.0, -KNOB_SHAFT_LEN / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_finish,
        name="shaft",
    )
    top_knob.visual(
        _knob_mesh("toaster_oven_top_knob"),
        origin=Origin(
            xyz=(0.0, -KNOB_SHAFT_LEN, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_finish,
        name="cap",
    )

    bottom_knob = model.part("bottom_knob")
    bottom_knob.visual(
        Cylinder(radius=0.006, length=KNOB_SHAFT_LEN),
        origin=Origin(
            xyz=(0.0, -KNOB_SHAFT_LEN / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_finish,
        name="shaft",
    )
    bottom_knob.visual(
        _knob_mesh("toaster_oven_bottom_knob"),
        origin=Origin(
            xyz=(0.0, -KNOB_SHAFT_LEN, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_finish,
        name="cap",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(CAVITY_X, DOOR_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=DOOR_OPEN,
        ),
    )
    model.articulation(
        "body_to_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(CAVITY_X, RACK_Y, RACK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.30,
            lower=0.0,
            upper=RACK_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_top_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=top_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_AXIS_Y, TOP_KNOB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )
    model.articulation(
        "body_to_bottom_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=bottom_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_AXIS_Y, BOTTOM_KNOB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    rack = object_model.get_part("rack")
    top_knob = object_model.get_part("top_knob")
    bottom_knob = object_model.get_part("bottom_knob")

    door_hinge = object_model.get_articulation("body_to_door")
    rack_slide = object_model.get_articulation("body_to_rack")
    top_knob_joint = object_model.get_articulation("body_to_top_knob")
    bottom_knob_joint = object_model.get_articulation("body_to_bottom_knob")

    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.16,
        name="door covers the front opening footprint",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.012,
        name="closed door sits just proud of the oven face",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: DOOR_OPEN}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens downward from the lower hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.06
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.08,
        details=f"closed={closed_door_aabb!r}, open={open_door_aabb!r}",
    )

    ctx.expect_within(
        rack,
        body,
        axes="xz",
        margin=0.035,
        name="rack stays centered inside the cavity span",
    )
    ctx.expect_overlap(
        rack,
        body,
        axes="y",
        min_overlap=0.14,
        name="collapsed rack remains inserted in the oven body",
    )
    rest_rack_pos = ctx.part_world_position(rack)
    with ctx.pose({rack_slide: RACK_TRAVEL}):
        ctx.expect_within(
            rack,
            body,
            axes="xz",
            margin=0.035,
            name="extended rack stays centered on the side guides",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            min_overlap=0.10,
            name="extended rack still retains insertion in the cavity",
        )
        extended_rack_pos = ctx.part_world_position(rack)
    ctx.check(
        "rack pulls out toward the front",
        rest_rack_pos is not None
        and extended_rack_pos is not None
        and extended_rack_pos[1] < rest_rack_pos[1] - 0.08,
        details=f"rest={rest_rack_pos!r}, extended={extended_rack_pos!r}",
    )

    ctx.expect_contact(top_knob, body, name="top knob mounts on the control panel")
    ctx.expect_contact(bottom_knob, body, name="bottom knob mounts on the control panel")
    ctx.check(
        "top knob uses a continuous rotary joint",
        top_knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={top_knob_joint.articulation_type!r}",
    )
    ctx.check(
        "bottom knob uses a continuous rotary joint",
        bottom_knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={bottom_knob_joint.articulation_type!r}",
    )

    return ctx.report()


object_model = build_object_model()
