from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_WIDTH = 0.158
BODY_DEPTH = 0.108
BODY_HEIGHT = 0.124

DRAWER_WIDTH = 0.088
DRAWER_DEPTH = 0.072
DRAWER_HEIGHT = 0.034
DRAWER_WALL = 0.003
DRAWER_FLOOR = 0.0028
DRAWER_FRONT_THICKNESS = 0.0055
DRAWER_TRAVEL = 0.040
DRAWER_BOTTOM_Z = 0.012

PORT_Z = 0.080
PORT_CENTER_Y = -BODY_DEPTH * 0.5
PORT_INNER_RADIUS = 0.0058
PORT_COUNTERBORE_RADIUS = 0.0110
PORT_COUNTERBORE_DEPTH = 0.008

CRANK_Y = -0.012
CRANK_Z = PORT_Z

DIAL_OUTER_RADIUS = 0.020
DIAL_INNER_RADIUS = 0.0064
DIAL_COLLAR_RADIUS = 0.0102
DIAL_FRONT_THICKNESS = 0.0058
DIAL_REAR_THICKNESS = 0.0060


def _build_body_shape():
    body = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.007)
    )

    drawer_cavity = (
        cq.Workplane("XY")
        .box(0.092, 0.073, 0.039, centered=(True, False, False))
        .translate((0.0, -BODY_DEPTH * 0.5 - 0.001, 0.0105))
    )

    port_counterbore = (
        cq.Workplane("XZ")
        .circle(PORT_COUNTERBORE_RADIUS)
        .extrude(PORT_COUNTERBORE_DEPTH)
        .translate((0.0, PORT_CENTER_Y, PORT_Z))
    )
    port_channel = (
        cq.Workplane("XZ")
        .circle(PORT_INNER_RADIUS)
        .extrude(0.044)
        .translate((0.0, PORT_CENTER_Y + PORT_COUNTERBORE_DEPTH, PORT_Z))
    )

    cutter_cavity = (
        cq.Workplane("YZ")
        .circle(0.0185)
        .extrude(0.062)
        .translate((-0.024, CRANK_Y, PORT_Z))
    )
    side_tunnel = (
        cq.Workplane("YZ")
        .circle(0.0086)
        .extrude(0.060)
        .translate((0.021, CRANK_Y, PORT_Z))
    )
    shaving_chute = (
        cq.Workplane("XY")
        .box(0.024, 0.028, 0.033, centered=(True, True, False))
        .translate((0.0, -0.012, 0.046))
    )

    return (
        body.cut(drawer_cavity)
        .cut(port_counterbore)
        .cut(port_channel)
        .cut(cutter_cavity)
        .cut(side_tunnel)
        .cut(shaving_chute)
    )


def _build_drawer_shape():
    outer = cq.Workplane("XY").box(
        DRAWER_WIDTH,
        DRAWER_DEPTH,
        DRAWER_HEIGHT,
        centered=(True, False, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            DRAWER_WIDTH - 2.0 * DRAWER_WALL,
            DRAWER_DEPTH - (DRAWER_FRONT_THICKNESS + DRAWER_WALL),
            DRAWER_HEIGHT - DRAWER_FLOOR,
            centered=(True, False, False),
        )
        .translate((0.0, DRAWER_FRONT_THICKNESS, DRAWER_FLOOR))
    )
    finger_pull = (
        cq.Workplane("YZ")
        .circle(0.012)
        .extrude(DRAWER_WIDTH * 1.2)
        .translate((-DRAWER_WIDTH * 0.6, DRAWER_FRONT_THICKNESS * 0.52, DRAWER_HEIGHT * 0.82))
    )
    return outer.cut(inner).cut(finger_pull)


def _build_dial_shape():
    return (
        cq.Workplane("XZ")
        .circle(DIAL_OUTER_RADIUS)
        .circle(DIAL_INNER_RADIUS)
        .extrude(-DIAL_FRONT_THICKNESS)
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_pencil_sharpener")

    body_finish = model.material("body_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.65, 1.0))
    smoked = model.material("smoked", rgba=(0.24, 0.26, 0.29, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "sharpener_body"),
        material=body_finish,
        name="shell",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Cylinder(radius=0.0085, length=0.004),
                origin=Origin(
                    xyz=(
                        x_sign * (BODY_WIDTH * 0.5 - 0.022),
                        y_sign * (BODY_DEPTH * 0.5 - 0.020),
                        0.002,
                    )
                ),
                material=rubber,
                name=f"foot_{int((x_sign + 1) * 0.5)}_{int((y_sign + 1) * 0.5)}",
            )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer_shape(), "sharpener_drawer"),
        material=smoked,
        name="drawer_shell",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_WIDTH, DRAWER_DEPTH, DRAWER_HEIGHT)),
        mass=0.16,
        origin=Origin(xyz=(0.0, DRAWER_DEPTH * 0.5, DRAWER_HEIGHT * 0.5)),
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0072, length=0.086),
        origin=Origin(xyz=(-0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="drive_shaft",
    )
    crank.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(-0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="cutter_drum",
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="hub_cap",
    )
    crank.visual(
        Box((0.007, 0.014, 0.052)),
        origin=Origin(xyz=(0.009, 0.0, 0.028)),
        material=aluminum,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(xyz=(0.009, 0.011, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_post",
    )
    crank.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.009, 0.022, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="handle_grip",
    )
    crank.inertial = Inertial.from_geometry(
        Box((0.104, 0.050, 0.110)),
        mass=0.25,
        origin=Origin(xyz=(-0.036, 0.006, 0.028)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_dial_shape(), "sharpener_dial"),
        origin=Origin(xyz=(0.0, -0.0058, 0.0)),
        material=aluminum,
        name="dial_ring",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.050, DIAL_FRONT_THICKNESS, 0.050)),
        mass=0.05,
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5, DRAWER_BOTTOM_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.20,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(BODY_WIDTH * 0.5, CRANK_Y, CRANK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, PORT_CENTER_Y, PORT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    dial = object_model.get_part("dial")

    drawer_slide = object_model.get_articulation("body_to_drawer")
    crank_spin = object_model.get_articulation("body_to_crank")
    dial_spin = object_model.get_articulation("body_to_dial")

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="xz",
            min_overlap=0.030,
            name="closed drawer stays aligned with the body opening",
        )
        body_aabb = ctx.part_world_aabb(body)
        drawer_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_shell")
        if body_aabb is not None and drawer_aabb is not None:
            body_front = float(body_aabb[0][1])
            drawer_front = float(drawer_aabb[0][1])
            ctx.check(
                "drawer front sits flush when closed",
                abs(drawer_front - body_front) <= 0.0015,
                details=f"body_front={body_front:.5f}, drawer_front={drawer_front:.5f}",
            )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="xz",
            min_overlap=0.030,
            name="extended drawer stays centered in the opening",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            min_overlap=0.030,
            name="extended drawer retains insertion in the body",
        )
        drawer_extended = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends forward",
        drawer_rest is not None
        and drawer_extended is not None
        and float(drawer_extended[1]) < float(drawer_rest[1]) - 0.030,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    body_aabb = ctx.part_world_aabb(body)
    dial_aabb = ctx.part_world_aabb(dial)
    if body_aabb is not None and dial_aabb is not None:
        ctx.check(
            "dial stands proud of the front shell",
            float(dial_aabb[0][1]) < float(body_aabb[0][1]) - 0.003,
            details=f"body_front={float(body_aabb[0][1]):.5f}, dial_front={float(dial_aabb[0][1]):.5f}",
        )

    crank_grip_rest = _aabb_center(ctx.part_element_world_aabb(crank, elem="handle_grip"))
    with ctx.pose({crank_spin: math.pi / 2.0}):
        crank_grip_turned = _aabb_center(ctx.part_element_world_aabb(crank, elem="handle_grip"))
    ctx.check(
        "crank handle sweeps around the hub",
        crank_grip_rest is not None
        and crank_grip_turned is not None
        and abs(float(crank_grip_turned[1]) - float(crank_grip_rest[1])) > 0.020
        and abs(float(crank_grip_turned[2]) - float(crank_grip_rest[2])) > 0.020,
        details=f"rest={crank_grip_rest}, turned={crank_grip_turned}",
    )

    ctx.check(
        "dial uses the pencil-entry axis as a continuous rotary control",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(float(value) for value in dial_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={dial_spin.articulation_type}, axis={dial_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
