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


def _housing_shape() -> cq.Workplane:
    plinth = (
        cq.Workplane("XY")
        .box(0.126, 0.094, 0.018, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )
    shell = (
        cq.Workplane("XY")
        .box(0.116, 0.084, 0.130, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.006)
        .translate((0.0, 0.0, 0.018))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.096, 0.074, 0.016, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .translate((-0.006, 0.0, 0.148))
    )
    crank_boss = (
        cq.Workplane("XY")
        .circle(0.012)
        .extrude(0.010)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((-0.006, 0.042, 0.100))
    )
    access_surround = (
        cq.Workplane("XY")
        .box(0.046, 0.004, 0.044, centered=(True, True, False))
        .translate((-0.004, -0.044, 0.018))
    )

    outer = plinth.union(shell).union(top_cap).union(crank_boss).union(access_surround)

    cavity = (
        cq.Workplane("XY")
        .box(0.092, 0.060, 0.136, centered=(True, True, False))
        .translate((-0.004, 0.0, 0.014))
    )
    drawer_opening = (
        cq.Workplane("XY")
        .box(0.036, 0.060, 0.032, centered=(True, True, False))
        .translate((0.060, 0.0, 0.016))
    )
    cleanout_opening = (
        cq.Workplane("XY")
        .box(0.034, 0.014, 0.032, centered=(True, True, False))
        .translate((-0.004, -0.046, 0.019))
    )
    pencil_port = (
        cq.Workplane("XY")
        .circle(0.0055)
        .extrude(0.040)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.030, 0.0, 0.112))
    )
    axle_recess = (
        cq.Workplane("XY")
        .circle(0.007)
        .extrude(0.020)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((-0.006, 0.040, 0.100))
    )

    return outer.cut(cavity).cut(drawer_opening).cut(cleanout_opening).cut(pencil_port).cut(axle_recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_desktop_pencil_sharpener")

    body_finish = model.material("body_finish", rgba=(0.45, 0.07, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    bakelite = model.material("bakelite", rgba=(0.10, 0.09, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_housing_shape(), "sharpener_housing"),
        material=body_finish,
        name="housing",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.088, 0.054, 0.002)),
        origin=Origin(xyz=(-0.044, 0.0, 0.001)),
        material=dark_metal,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.088, 0.002, 0.022)),
        origin=Origin(xyz=(-0.044, 0.026, 0.012)),
        material=dark_metal,
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.088, 0.002, 0.022)),
        origin=Origin(xyz=(-0.044, -0.026, 0.012)),
        material=dark_metal,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.002, 0.054, 0.022)),
        origin=Origin(xyz=(-0.087, 0.0, 0.012)),
        material=dark_metal,
        name="tray_back",
    )
    drawer.visual(
        Box((0.006, 0.066, 0.034)),
        origin=Origin(xyz=(0.003, 0.0, 0.017)),
        material=body_finish,
        name="drawer_front",
    )
    drawer.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(xyz=(0.013, 0.0, 0.017), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bakelite,
        name="drawer_knob",
    )

    cleanout_door = model.part("cleanout_door")
    cleanout_door.visual(
        Box((0.034, 0.003, 0.032)),
        origin=Origin(xyz=(0.017, -0.0015, 0.0)),
        material=body_finish,
        name="door_panel",
    )
    cleanout_door.visual(
        Box((0.028, 0.0025, 0.026)),
        origin=Origin(xyz=(0.018, 0.00125, 0.0)),
        material=dark_metal,
        name="door_flange",
    )
    cleanout_door.visual(
        Cylinder(radius=0.0025, length=0.008),
        origin=Origin(xyz=(-0.0024, -0.0012, -0.011)),
        material=steel,
        name="door_hinge_0",
    )
    cleanout_door.visual(
        Cylinder(radius=0.0025, length=0.008),
        origin=Origin(xyz=(-0.0024, -0.0012, 0.0)),
        material=steel,
        name="door_hinge_1",
    )
    cleanout_door.visual(
        Cylinder(radius=0.0025, length=0.008),
        origin=Origin(xyz=(-0.0024, -0.0012, 0.011)),
        material=steel,
        name="door_hinge_2",
    )
    cleanout_door.visual(
        Cylinder(radius=0.0022, length=0.006),
        origin=Origin(xyz=(0.034, -0.0015, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="door_tab",
    )

    crank = model.part("crank")
    arm_start_x = 0.004
    arm_start_z = -0.004
    arm_end_x = -0.032
    arm_end_z = 0.036
    arm_dx = arm_end_x - arm_start_x
    arm_dz = arm_end_z - arm_start_z
    arm_len = math.hypot(arm_dx, arm_dz)
    arm_pitch = math.atan2(arm_dz, arm_dx)
    crank.visual(
        Cylinder(radius=0.0065, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_hub",
    )
    crank.visual(
        Box((0.020, 0.012, 0.018)),
        origin=Origin(xyz=(-0.002, 0.014, 0.003)),
        material=steel,
        name="crank_shoulder",
    )
    crank.visual(
        Box((arm_len, 0.010, 0.010)),
        origin=Origin(
            xyz=((arm_start_x + arm_end_x) / 2.0, 0.014, (arm_start_z + arm_end_z) / 2.0),
            rpy=(0.0, arm_pitch, 0.0),
        ),
        material=steel,
        name="crank_arm",
    )
    crank.visual(
        Box((0.024, 0.014, 0.022)),
        origin=Origin(xyz=(-0.028, 0.017, 0.028)),
        material=steel,
        name="crank_elbow",
    )
    crank.visual(
        Cylinder(radius=0.0032, length=0.032),
        origin=Origin(xyz=(arm_end_x, 0.028, arm_end_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_pin",
    )
    crank.visual(
        Cylinder(radius=0.0075, length=0.020),
        origin=Origin(xyz=(arm_end_x, 0.051, arm_end_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bakelite,
        name="crank_grip",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.058, 0.0, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.12, lower=0.0, upper=0.045),
    )
    model.articulation(
        "body_to_cleanout_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cleanout_door,
        origin=Origin(xyz=(-0.021, -0.043, 0.035)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(-0.006, 0.042, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    cleanout_door = object_model.get_part("cleanout_door")
    crank = object_model.get_part("crank")

    drawer_slide = object_model.get_articulation("body_to_drawer")
    door_hinge = object_model.get_articulation("body_to_cleanout_door")
    crank_spin = object_model.get_articulation("body_to_crank")

    ctx.expect_contact(
        crank,
        body,
        elem_a="crank_hub",
        elem_b="housing",
        contact_tol=1e-4,
        name="crank hub seats on the side boss",
    )

    drawer_limits = drawer_slide.motion_limits
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_slide: 0.0}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="yz",
                min_overlap=0.028,
                name="closed drawer stays centered in the front opening",
            )
            closed_drawer_aabb = ctx.part_world_aabb(drawer)

        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="yz",
                min_overlap=0.028,
                name="extended drawer stays guided by the housing",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.030,
                name="extended drawer remains retained in the housing",
            )
            extended_drawer_aabb = ctx.part_world_aabb(drawer)

        ctx.check(
            "drawer extends forward",
            closed_drawer_aabb is not None
            and extended_drawer_aabb is not None
            and extended_drawer_aabb[0][0] > closed_drawer_aabb[0][0] + 0.035,
            details=f"closed={closed_drawer_aabb}, extended={extended_drawer_aabb}",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: 0.0}):
            ctx.expect_overlap(
                cleanout_door,
                body,
                axes="xz",
                min_overlap=0.024,
                name="closed cleanout door covers the side access opening",
            )
            closed_door_aabb = ctx.part_world_aabb(cleanout_door)

        with ctx.pose({door_hinge: door_limits.upper}):
            open_door_aabb = ctx.part_world_aabb(cleanout_door)

        ctx.check(
            "cleanout door swings outward",
            closed_door_aabb is not None
            and open_door_aabb is not None
            and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.012,
            details=f"closed={closed_door_aabb}, open={open_door_aabb}",
        )

    with ctx.pose({crank_spin: 0.0}):
        crank_rest_aabb = ctx.part_world_aabb(crank)
    with ctx.pose({crank_spin: math.pi / 2.0}):
        crank_quarter_aabb = ctx.part_world_aabb(crank)

    ctx.check(
        "crank rotation changes the silhouette",
        crank_rest_aabb is not None
        and crank_quarter_aabb is not None
        and (
            abs(crank_rest_aabb[0][0] - crank_quarter_aabb[0][0]) > 0.010
            or abs(crank_rest_aabb[1][0] - crank_quarter_aabb[1][0]) > 0.010
            or abs(crank_rest_aabb[0][2] - crank_quarter_aabb[0][2]) > 0.010
            or abs(crank_rest_aabb[1][2] - crank_quarter_aabb[1][2]) > 0.010
        ),
        details=f"rest={crank_rest_aabb}, quarter_turn={crank_quarter_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
