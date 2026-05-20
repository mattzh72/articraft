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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_DEPTH = 0.112
BASE_WIDTH = 0.090
BASE_THICK = 0.008

BODY_DEPTH = 0.096
BODY_WIDTH = 0.078
BODY_HEIGHT = 0.120

DRAWER_CAVITY_DEPTH = 0.064
DRAWER_CAVITY_WIDTH = 0.058
DRAWER_CAVITY_HEIGHT = 0.038
DRAWER_BOTTOM_Z = 0.0105
DRAWER_TRAVEL = 0.040

AXLE_X = 0.004
AXLE_Z = 0.073
AXLE_BOSS_RADIUS = 0.010
AXLE_BOSS_LENGTH = 0.006

PORT_RADIUS = 0.0048
PORT_COUNTERBORE_RADIUS = 0.0078
PORT_COUNTERBORE_LENGTH = 0.009
PORT_LENGTH = 0.056
PORT_Z = 0.084


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _build_body_mesh():
    base = (
        cq.Workplane("XY")
        .box(BASE_DEPTH, BASE_WIDTH, BASE_THICK, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0055)
    )

    housing = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, BASE_THICK - 0.0005))
    )

    axle_boss = (
        cq.Workplane("XZ")
        .circle(AXLE_BOSS_RADIUS)
        .extrude(AXLE_BOSS_LENGTH)
        .translate((AXLE_X, BODY_WIDTH * 0.5 - 0.001, AXLE_Z))
    )

    drawer_cavity = (
        cq.Workplane("XY")
        .box(
            DRAWER_CAVITY_DEPTH,
            DRAWER_CAVITY_WIDTH,
            DRAWER_CAVITY_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                BODY_DEPTH * 0.5 - DRAWER_CAVITY_DEPTH * 0.5,
                0.0,
                DRAWER_BOTTOM_Z,
            )
        )
    )

    port_start_x = BODY_DEPTH * 0.5 - PORT_LENGTH
    pencil_port = (
        cq.Workplane("YZ")
        .circle(PORT_RADIUS)
        .extrude(PORT_LENGTH)
        .translate((port_start_x, 0.0, PORT_Z))
    )
    port_counterbore = (
        cq.Workplane("YZ")
        .circle(PORT_COUNTERBORE_RADIUS)
        .extrude(PORT_COUNTERBORE_LENGTH)
        .translate((BODY_DEPTH * 0.5 - PORT_COUNTERBORE_LENGTH, 0.0, PORT_Z))
    )
    mechanism_chamber = (
        cq.Workplane("XY")
        .box(0.030, 0.024, 0.024, centered=(True, True, True))
        .translate((0.002, 0.0, PORT_Z))
    )

    front_slot = (
        cq.Workplane("YZ")
        .rect(0.014, 0.024)
        .extrude(0.018)
        .translate((BODY_DEPTH * 0.5 - 0.018, 0.0, PORT_Z - 0.001))
    )

    body = (
        base.union(housing)
        .union(axle_boss)
        .cut(drawer_cavity)
        .cut(pencil_port)
        .cut(port_counterbore)
        .cut(mechanism_chamber)
        .cut(front_slot)
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_desktop_pencil_sharpener")

    body_paint = model.material("body_paint", rgba=(0.50, 0.09, 0.08, 1.0))
    drawer_metal = model.material("drawer_metal", rgba=(0.40, 0.16, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.71, 0.73, 1.0))
    dark_knob = model.material("dark_knob", rgba=(0.10, 0.09, 0.08, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_mesh(), "pencil_sharpener_body"),
        material=body_paint,
        name="housing",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.007),
        origin=Origin(
            xyz=(AXLE_X, BODY_WIDTH * 0.5 + 0.0025, AXLE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="axle_boss",
    )

    drawer = model.part("drawer")
    drawer_front_thickness = 0.004
    drawer_front_width = 0.062
    drawer_front_height = 0.041
    tray_depth = 0.060
    tray_width = 0.054
    tray_height = 0.030
    wall_thickness = 0.002
    floor_thickness = 0.0025
    drawer.visual(
        Box((drawer_front_thickness, drawer_front_width, drawer_front_height)),
        origin=Origin(
            xyz=(drawer_front_thickness * 0.5 - 0.0005, 0.0, drawer_front_height * 0.5)
        ),
        material=drawer_metal,
        name="drawer_front",
    )
    drawer.visual(
        Box((tray_depth + 0.001, tray_width - 2.0 * wall_thickness, floor_thickness)),
        origin=Origin(
            xyz=(-tray_depth * 0.5 + 0.0005, 0.0, floor_thickness * 0.5),
        ),
        material=drawer_metal,
        name="tray_floor",
    )
    drawer.visual(
        Box((tray_depth + 0.001, wall_thickness, tray_height)),
        origin=Origin(
            xyz=(-tray_depth * 0.5 + 0.0005, (tray_width - wall_thickness) * 0.5, tray_height * 0.5),
        ),
        material=drawer_metal,
        name="tray_side_0",
    )
    drawer.visual(
        Box((tray_depth + 0.001, wall_thickness, tray_height)),
        origin=Origin(
            xyz=(-tray_depth * 0.5 + 0.0005, -(tray_width - wall_thickness) * 0.5, tray_height * 0.5),
        ),
        material=drawer_metal,
        name="tray_side_1",
    )
    drawer.visual(
        Box((wall_thickness, tray_width, tray_height)),
        origin=Origin(
            xyz=(-tray_depth + wall_thickness * 0.5, 0.0, tray_height * 0.5),
        ),
        material=drawer_metal,
        name="tray_back",
    )
    drawer.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(xyz=(0.008, 0.0, 0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_knob,
        name="pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )

    arm_a = (0.0, 0.009, 0.0)
    arm_b = (-0.016, 0.011, 0.014)
    arm_c = (-0.041, 0.018, 0.032)
    for visual_name, start, end in (
        ("arm_0", arm_a, arm_b),
        ("arm_1", arm_b, arm_c),
    ):
        crank.visual(
            Cylinder(radius=0.0042, length=_distance(start, end)),
            origin=Origin(xyz=_midpoint(start, end), rpy=_rpy_for_cylinder(start, end)),
            material=graphite,
            name=visual_name,
        )

    crank.visual(
        Sphere(radius=0.0055),
        origin=Origin(xyz=arm_b),
        material=graphite,
        name="elbow",
    )
    crank.visual(
        Cylinder(radius=0.0034, length=0.016),
        origin=Origin(
            xyz=(arm_c[0], 0.018, arm_c[2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="spindle",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="collar",
    )
    knob.visual(
        Cylinder(radius=0.0105, length=0.018),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_knob,
        name="grip",
    )
    knob.visual(
        Sphere(radius=0.0105),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=dark_knob,
        name="endcap",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_DEPTH * 0.5 - 0.001, 0.0, DRAWER_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.12,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(AXLE_X, BODY_WIDTH * 0.5 + AXLE_BOSS_LENGTH, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=10.0),
    )
    model.articulation(
        "crank_to_knob",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=knob,
        origin=Origin(xyz=(arm_c[0], 0.026, arm_c[2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    knob = object_model.get_part("knob")

    drawer_slide = object_model.get_articulation("body_to_drawer")
    crank_spin = object_model.get_articulation("body_to_crank")
    knob_spin = object_model.get_articulation("crank_to_knob")

    ctx.check(
        "drawer uses prismatic travel",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={drawer_slide.articulation_type}",
    )
    ctx.check(
        "crank uses continuous rotation",
        crank_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={crank_spin.articulation_type}",
    )
    ctx.check(
        "knob uses continuous rotation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )

    ctx.expect_contact(knob, crank, name="knob is mounted on the crank spindle")
    ctx.expect_origin_gap(
        knob,
        crank,
        axis="y",
        min_gap=0.020,
        name="knob sits outboard of the crank arm",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    drawer_limits = drawer_slide.motion_limits
    if (
        drawer_limits is not None
        and drawer_limits.upper is not None
        and closed_drawer_pos is not None
    ):
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            open_drawer_pos = ctx.part_world_position(drawer)
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                margin=0.008,
                name="drawer stays centered in the front opening",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.020,
                name="drawer remains retained at full extension",
            )
        ctx.check(
            "drawer slides forward from the housing",
            open_drawer_pos is not None and open_drawer_pos[0] > closed_drawer_pos[0] + 0.030,
            details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
        )

    rest_knob_pos = ctx.part_world_position(knob)
    with ctx.pose({crank_spin: math.pi}):
        half_turn_knob_pos = ctx.part_world_position(knob)
    ctx.check(
        "crank carries the knob through a circular path",
        rest_knob_pos is not None
        and half_turn_knob_pos is not None
        and abs(half_turn_knob_pos[0] - rest_knob_pos[0]) > 0.050
        and abs(half_turn_knob_pos[2] - rest_knob_pos[2]) > 0.040,
        details=f"rest={rest_knob_pos}, half_turn={half_turn_knob_pos}",
    )

    return ctx.report()


object_model = build_object_model()
