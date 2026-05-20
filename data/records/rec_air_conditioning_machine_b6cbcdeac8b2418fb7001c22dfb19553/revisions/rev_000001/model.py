from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.36
BODY_D = 0.29
BODY_H = 0.70
BODY_WALL = 0.004
BODY_CORNER_R = 0.032
TOP_EDGE_R = 0.018

OUTLET_W = 0.23
OUTLET_H = 0.11
OUTLET_R = 0.012
OUTLET_BOTTOM = 0.455
OUTLET_TOP = OUTLET_BOTTOM + OUTLET_H

DOOR_W = 0.218
DOOR_H = 0.102
DOOR_T = 0.007
DOOR_R = 0.010

KNOB_Z = 0.625

WHEEL_RADIUS = 0.036
WHEEL_WIDTH = 0.028
WHEEL_CENTER_X = BODY_W * 0.5 - WHEEL_WIDTH * 0.5 - 0.002
WHEEL_CENTER_Y = -(BODY_D * 0.5 + 0.037)
WHEEL_CENTER_Z = 0.040


def _body_shell_shape():
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_R)
        .edges(">Z")
        .fillet(TOP_EDGE_R)
        .faces("<Z")
        .shell(-BODY_WALL)
    )

    outlet_cutter = (
        cq.Workplane("XY")
        .box(OUTLET_W, BODY_D * 0.42, OUTLET_H, centered=(True, False, False))
        .edges("|Y")
        .fillet(OUTLET_R)
        .translate((0.0, BODY_D * 0.5 - BODY_D * 0.42, OUTLET_BOTTOM))
    )
    return shell.cut(outlet_cutter)


def _door_shape():
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H, centered=(True, False, False))
        .edges("|Y")
        .fillet(DOOR_R)
        .translate((0.0, 0.0, -DOOR_H))
    )
    lip = (
        cq.Workplane("XY")
        .box(DOOR_W * 0.82, DOOR_T * 1.3, 0.010, centered=(True, False, False))
        .translate((0.0, 0.0, -DOOR_H + 0.004))
    )
    return panel.union(lip)


def _add_transport_wheel(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    mesh_prefix: str,
    side_sign: float,
    wheel_finish,
    tire_finish,
) -> None:
    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_finish,
        name="wheel",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=WHEEL_WIDTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_finish,
        name="hub",
    )
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                WHEEL_RADIUS,
                WHEEL_WIDTH,
                inner_radius=0.026,
                sidewall=TireSidewall(style="rounded", bulge=0.05),
            ),
            f"{mesh_prefix}_tire",
        ),
        material=tire_finish,
        name="tire",
    )
    wheel.inertial = Inertial.from_geometry(
        Box((WHEEL_WIDTH, WHEEL_RADIUS * 2.0, WHEEL_RADIUS * 2.0)),
        mass=0.16,
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(
            xyz=(
                side_sign * WHEEL_CENTER_X,
                WHEEL_CENTER_Y,
                WHEEL_CENTER_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_air_conditioner")

    shell_finish = model.material("shell_finish", rgba=(0.92, 0.93, 0.91, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.25, 0.28, 0.30, 1.0))
    cavity_finish = model.material("cavity_finish", rgba=(0.13, 0.14, 0.15, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.46, 0.49, 0.52, 1.0))
    tire_finish = model.material("tire_finish", rgba=(0.07, 0.07, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material=shell_finish,
        name="shell",
    )
    body.visual(
        Box((BODY_W - 0.020, BODY_D - 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=trim_finish,
        name="base_pan",
    )
    body.visual(
        Box((OUTLET_W + 0.030, 0.040, OUTLET_H + 0.028)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D * 0.5 - 0.020,
                OUTLET_BOTTOM + OUTLET_H * 0.5,
            )
        ),
        material=cavity_finish,
        name="outlet_duct",
    )

    for index, side_sign in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.010, 0.037, 0.068)),
            origin=Origin(
                xyz=(
                    side_sign * (WHEEL_CENTER_X - WHEEL_WIDTH * 0.5 - 0.005),
                    WHEEL_CENTER_Y + 0.0185,
                    0.044,
                )
            ),
            material=trim_finish,
            name=f"wheel_bracket_{index}",
        )

    for index, side_sign in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.026, 0.032, 0.020)),
            origin=Origin(
                xyz=(
                    side_sign * 0.108,
                    BODY_D * 0.5 - 0.016,
                    0.010,
                )
            ),
            material=trim_finish,
            name=f"foot_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_shape(), "outlet_door"),
        material=trim_finish,
        name="door_panel",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T * 1.4, DOOR_H)),
        mass=0.35,
        origin=Origin(xyz=(0.0, DOOR_T * 0.5, -DOOR_H * 0.5)),
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.015,
                body_style="domed",
                edge_radius=0.0015,
                grip=KnobGrip(style="ribbed", count=12, depth=0.0007),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005, angle_deg=0.0),
                center=False,
            ),
            "control_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="knob_shell",
    )
    knob.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="knob_shaft",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.018),
        mass=0.08,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, BODY_D * 0.5, OUTLET_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,
            upper=1.10,
        ),
    )

    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, BODY_D * 0.5, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )

    _add_transport_wheel(
        model,
        body,
        name="wheel_0",
        mesh_prefix="rear_wheel_0",
        side_sign=-1.0,
        wheel_finish=wheel_finish,
        tire_finish=tire_finish,
    )
    _add_transport_wheel(
        model,
        body,
        name="wheel_1",
        mesh_prefix="rear_wheel_1",
        side_sign=1.0,
        wheel_finish=wheel_finish,
        tire_finish=tire_finish,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    knob = object_model.get_part("knob")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    door_joint = object_model.get_articulation("body_to_door")
    knob_joint = object_model.get_articulation("body_to_knob")
    wheel_joint_0 = object_model.get_articulation("body_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("body_to_wheel_1")

    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="shell",
        max_gap=0.004,
        max_penetration=0.0,
        name="outlet door rests at the front body face",
    )
    ctx.expect_origin_gap(
        knob,
        door,
        axis="z",
        min_gap=0.045,
        max_gap=0.080,
        name="control knob sits above the outlet door",
    )

    body_aabb = ctx.part_world_aabb(body)
    wheel_0_pos = ctx.part_world_position(wheel_0)
    wheel_1_pos = ctx.part_world_position(wheel_1)
    ctx.check(
        "rear wheels sit low and aft",
        body_aabb is not None
        and wheel_0_pos is not None
        and wheel_1_pos is not None
        and body_aabb[1][1] - wheel_0_pos[1] > 0.30
        and body_aabb[1][1] - wheel_1_pos[1] > 0.30
        and 0.02 <= wheel_0_pos[2] <= 0.06
        and 0.02 <= wheel_1_pos[2] <= 0.06,
        details=f"body_aabb={body_aabb}, wheel_0={wheel_0_pos}, wheel_1={wheel_1_pos}",
    )

    ctx.check(
        "continuous control joints remain unbounded",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None
        and wheel_joint_0.motion_limits is not None
        and wheel_joint_0.motion_limits.lower is None
        and wheel_joint_0.motion_limits.upper is None
        and wheel_joint_1.motion_limits is not None
        and wheel_joint_1.motion_limits.lower is None
        and wheel_joint_1.motion_limits.upper is None,
        details=(
            f"knob_limits={knob_joint.motion_limits}, "
            f"wheel_0_limits={wheel_joint_0.motion_limits}, "
            f"wheel_1_limits={wheel_joint_1.motion_limits}"
        ),
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.0}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door opens outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.05,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.check(
        "door lower edge lifts when opened",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] > closed_aabb[0][2] + 0.025,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
