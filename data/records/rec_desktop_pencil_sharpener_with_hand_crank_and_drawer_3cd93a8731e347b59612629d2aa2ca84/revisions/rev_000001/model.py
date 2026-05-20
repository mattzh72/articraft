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


BODY_W = 0.105
BODY_D = 0.092
BODY_FRONT_Y = -BODY_D / 2.0
BODY_BACK_Y = BODY_D / 2.0
BODY_SIDE_X = BODY_W / 2.0

ENTRY_Z = 0.071
DIAL_Z = 0.093
CRANK_Y = 0.006
CRANK_Z = 0.063

DRAWER_W = 0.078
DRAWER_D = 0.073
DRAWER_H = 0.031
DRAWER_BOTTOM_Z = 0.011
DRAWER_TRAVEL = 0.045


def cylinder_x(radius: float, length: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("YZ", origin=(x, y, z)).circle(radius).extrude(length)


def cylinder_y(radius: float, length: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("XZ", origin=(x, y, z)).circle(radius).extrude(length)


def cylinder_z(radius: float, length: float, *, x: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("XY", origin=(x, y, z)).circle(radius).extrude(length)


def make_body_shape() -> cq.Workplane:
    outer_profile = [
        (BODY_FRONT_Y, 0.0),
        (BODY_BACK_Y, 0.0),
        (BODY_BACK_Y, 0.089),
        (0.020, 0.112),
        (-0.006, 0.110),
        (BODY_FRONT_Y, 0.102),
    ]
    housing = (
        cq.Workplane("YZ")
        .polyline(outer_profile)
        .close()
        .extrude(BODY_W / 2.0, both=True)
        .edges("|X")
        .fillet(0.0035)
    )

    upper_cavity_profile = [
        (BODY_FRONT_Y + 0.006, 0.054),
        (BODY_BACK_Y - 0.006, 0.046),
        (BODY_BACK_Y - 0.006, 0.083),
        (0.014, 0.101),
        (-0.002, 0.098),
        (BODY_FRONT_Y + 0.006, 0.085),
    ]
    upper_cavity = (
        cq.Workplane("YZ")
        .polyline(upper_cavity_profile)
        .close()
        .extrude((BODY_W - 0.018) / 2.0, both=True)
    )
    drawer_bay = cq.Workplane("XY").box(0.082, 0.080, 0.036).translate((0.0, -0.007, 0.028))
    chute = cq.Workplane("XY").box(0.028, 0.030, 0.026).translate((0.0, -0.010, 0.054))
    chamber = cq.Workplane("XY").sphere(0.016).translate((0.0, -0.010, 0.073))
    pencil_bore = cylinder_y(0.0048, 0.030, x=0.0, y=BODY_FRONT_Y - 0.002, z=ENTRY_Z)
    dial_socket = cylinder_y(0.0036, 0.005, x=0.0, y=BODY_FRONT_Y - 0.001, z=DIAL_Z)
    shaft_clearance = cylinder_x(0.0062, 0.012, x=BODY_SIDE_X - 0.009, y=CRANK_Y, z=CRANK_Z)

    housing = (
        housing.cut(upper_cavity)
        .cut(drawer_bay)
        .cut(chute)
        .cut(chamber)
        .cut(pencil_bore)
        .cut(dial_socket)
        .cut(shaft_clearance)
    )
    return housing


def make_drawer_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(DRAWER_W, DRAWER_D, DRAWER_H).translate((0.0, DRAWER_D / 2.0, DRAWER_H / 2.0))
    pocket = cq.Workplane("XY").box(DRAWER_W - 0.006, DRAWER_D - 0.012, DRAWER_H - 0.004).translate(
        (0.0, 0.0345, 0.0165)
    )
    finger_pull = cq.Workplane("XY").box(0.036, 0.003, 0.010).translate((0.0, 0.0005, 0.017))
    return shell.cut(pocket).union(finger_pull)


def make_crank_shape() -> cq.Workplane:
    hub = cylinder_x(0.0085, 0.012, x=0.0, y=0.0, z=0.0)
    shaft_stub = cylinder_x(0.0045, 0.013, x=0.0105, y=0.0, z=0.0)
    rise = cylinder_z(0.0045, 0.029, x=0.0215, y=0.0, z=-0.001)
    reach = cylinder_y(0.0042, 0.022, x=0.0215, y=-0.001, z=0.0275)
    spinner_mount = cylinder_x(0.0052, 0.007, x=0.0175, y=0.019, z=0.0275)
    lower_bend = cq.Workplane("XY").sphere(0.0052).translate((0.0215, 0.0, 0.001))
    upper_bend = cq.Workplane("XY").sphere(0.0050).translate((0.0215, 0.019, 0.0275))
    return hub.union(shaft_stub).union(rise).union(reach).union(spinner_mount).union(lower_bend).union(upper_bend)


def make_spinner_shape() -> cq.Workplane:
    core = cylinder_x(0.0066, 0.021, x=0.0, y=0.0, z=0.0)
    bulge = cylinder_x(0.0071, 0.010, x=0.0055, y=0.0, z=0.0)
    return core.union(bulge)


def make_dial_shape() -> cq.Workplane:
    dial = cylinder_y(0.0108, 0.0048, x=0.0, y=-0.0048, z=0.0)
    skirt = cylinder_y(0.0122, 0.0016, x=0.0, y=-0.0016, z=0.0)
    stem = cylinder_y(0.0032, 0.0042, x=0.0, y=-0.0004, z=0.0)
    pointer = cq.Workplane("XY").box(0.0022, 0.0014, 0.0076).translate((0.0, -0.0038, 0.0065))
    return dial.union(skirt).union(stem).union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_pencil_sharpener")

    cast_metal = model.material("cast_metal", rgba=(0.58, 0.60, 0.64, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.79, 0.81, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.11, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(make_body_shape(), "sharpener_body"), material=cast_metal, name="housing")

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.084, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, 0.002, 0.017)),
        material=dark_plastic,
        name="front",
    )
    drawer.visual(
        Box((0.078, 0.070, 0.003)),
        origin=Origin(xyz=(0.0, 0.038, 0.0015)),
        material=dark_plastic,
        name="bottom",
    )
    drawer.visual(
        Box((0.003, 0.069, 0.028)),
        origin=Origin(xyz=(-0.0375, 0.0375, 0.017)),
        material=dark_plastic,
        name="side_0",
    )
    drawer.visual(
        Box((0.003, 0.069, 0.028)),
        origin=Origin(xyz=(0.0375, 0.0375, 0.017)),
        material=dark_plastic,
        name="side_1",
    )
    drawer.visual(
        Box((0.078, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, 0.071, 0.016)),
        material=dark_plastic,
        name="back",
    )
    drawer.visual(
        Box((0.038, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, 0.0005, 0.017)),
        material=dark_plastic,
        name="pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0085, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    crank.visual(
        Box((0.020, 0.006, 0.006)),
        origin=Origin(xyz=(0.015, 0.0, 0.003)),
        material=steel,
        name="web",
    )
    crank.visual(
        Box((0.008, 0.006, 0.030)),
        origin=Origin(xyz=(0.021, 0.0, 0.015)),
        material=steel,
        name="riser",
    )
    crank.visual(
        Box((0.008, 0.022, 0.006)),
        origin=Origin(xyz=(0.021, 0.010, 0.028)),
        material=steel,
        name="reach",
    )
    crank.visual(
        Cylinder(radius=0.0052, length=0.006),
        origin=Origin(xyz=(0.021, 0.020, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="mount",
    )

    spinner = model.part("spinner")
    spinner.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="grip",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.011, length=0.0044),
        origin=Origin(xyz=(0.0, -0.0022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="selector",
    )

    crank_joint = model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(BODY_SIDE_X, CRANK_Y, CRANK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "crank_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=spinner,
        origin=Origin(xyz=(0.024, 0.020, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=18.0),
    )
    drawer_joint = model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, DRAWER_BOTTOM_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.18, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, DIAL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    body.meta["entry_center"] = (0.0, BODY_FRONT_Y, ENTRY_Z)
    body.meta["dial_center"] = (0.0, BODY_FRONT_Y, DIAL_Z)
    crank.meta["joint"] = crank_joint.name
    drawer.meta["joint"] = drawer_joint.name

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    spinner = object_model.get_part("spinner")

    crank_joint = object_model.get_articulation("body_to_crank")
    spinner_joint = object_model.get_articulation("crank_to_spinner")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.check(
        "crank rotates continuously",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={crank_joint.articulation_type}",
    )
    ctx.check(
        "spinner rotates continuously",
        spinner_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spinner_joint.articulation_type}",
    )
    ctx.check(
        "drawer slides prismatically",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={drawer_joint.articulation_type}",
    )
    ctx.check(
        "selector dial rotates continuously",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )

    with ctx.pose({drawer_joint: 0.0}):
        ctx.expect_overlap(drawer, body, axes="xz", min_overlap=0.028, name="closed drawer stays aligned to the housing")
        body_aabb = ctx.part_world_aabb(body)
        drawer_aabb = ctx.part_world_aabb(drawer)
        proud = None
        if body_aabb is not None and drawer_aabb is not None:
            proud = body_aabb[0][1] - drawer_aabb[0][1]
        ctx.check(
            "drawer front sits slightly proud",
            proud is not None and 0.001 <= proud <= 0.004,
            details=f"drawer_proud={proud}",
        )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(drawer, body, axes="xz", min_overlap=0.028, name="extended drawer remains aligned to the housing")

    ctx.check(
        "drawer extends forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.035,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    rest_spinner_pos = ctx.part_world_position(spinner)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        turned_spinner_pos = ctx.part_world_position(spinner)
    ctx.check(
        "crank carries the spinner around its orbit",
        rest_spinner_pos is not None
        and turned_spinner_pos is not None
        and abs(turned_spinner_pos[1] - rest_spinner_pos[1]) > 0.03,
        details=f"rest={rest_spinner_pos}, turned={turned_spinner_pos}",
    )

    return ctx.report()


object_model = build_object_model()
