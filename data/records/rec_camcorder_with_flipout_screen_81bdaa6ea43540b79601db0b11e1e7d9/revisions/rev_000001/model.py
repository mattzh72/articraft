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


BODY_LEN = 0.132
BODY_WIDTH = 0.058
BODY_HEIGHT = 0.074

LENS_BARREL_RADIUS = 0.0248
LENS_BARREL_LENGTH = 0.014
LENS_RING_RADIUS = 0.031
LENS_RING_INNER_RADIUS = 0.0266
LENS_RING_LENGTH = 0.012

MONITOR_LEN = 0.072
MONITOR_THICKNESS = 0.008
MONITOR_HEIGHT = 0.054
MONITOR_HINGE_RADIUS = 0.004
MONITOR_HINGE_X = -0.030
MONITOR_HINGE_Y = BODY_WIDTH * 0.5 + MONITOR_HINGE_RADIUS
MONITOR_HINGE_Z = -0.021

PORT_COVER_LEN = 0.034
PORT_COVER_THICKNESS = 0.0045
PORT_COVER_HEIGHT = 0.018
PORT_COVER_HINGE_RADIUS = 0.0025
PORT_COVER_HINGE_X = -0.010
PORT_COVER_HINGE_Y = BODY_WIDTH * 0.5 + PORT_COVER_HINGE_RADIUS
PORT_COVER_HINGE_Z = -0.031


def _body_shell_mesh():
    main_shell = (
        cq.Workplane("XY")
        .box(BODY_LEN, BODY_WIDTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.010)
    )
    battery_pack = (
        cq.Workplane("XY")
        .box(0.030, 0.046, 0.048)
        .edges("|Z")
        .fillet(0.006)
        .translate((-0.073, 0.0, 0.004))
    )
    handgrip_bulge = (
        cq.Workplane("XY")
        .box(0.086, 0.018, 0.056)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.006, -0.038, -0.002))
    )
    top_ridge = (
        cq.Workplane("XY")
        .box(0.050, 0.018, 0.010)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.016, -0.004, 0.037))
    )
    return mesh_from_cadquery(
        main_shell.union(battery_pack).union(handgrip_bulge).union(top_ridge),
        "camcorder_body_shell",
    )


def _lens_ring_mesh():
    base_ring = (
        cq.Workplane("YZ")
        .circle(LENS_RING_RADIUS)
        .circle(LENS_RING_INNER_RADIUS)
        .extrude(LENS_RING_LENGTH)
    )
    ridge_a = (
        cq.Workplane("YZ")
        .circle(LENS_RING_RADIUS + 0.0012)
        .circle(LENS_RING_INNER_RADIUS)
        .extrude(0.0015)
        .translate((0.0020, 0.0, 0.0))
    )
    ridge_b = (
        cq.Workplane("YZ")
        .circle(LENS_RING_RADIUS + 0.0012)
        .circle(LENS_RING_INNER_RADIUS)
        .extrude(0.0015)
        .translate((0.0085, 0.0, 0.0))
    )
    return mesh_from_cadquery(
        base_ring.union(ridge_a).union(ridge_b).translate((-LENS_RING_LENGTH * 0.5, 0.0, 0.0)),
        "camcorder_lens_ring",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_camcorder")

    body_finish = model.material("body_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.28, 0.29, 0.31, 1.0))
    grip_finish = model.material("grip_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    lens_finish = model.material("lens_finish", rgba=(0.11, 0.11, 0.12, 1.0))
    ring_finish = model.material("ring_finish", rgba=(0.40, 0.41, 0.43, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.10, 0.18, 0.22, 0.55))
    accent_finish = model.material("accent_finish", rgba=(0.52, 0.53, 0.56, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_finish, name="housing_shell")
    body.visual(
        Cylinder(radius=LENS_BARREL_RADIUS, length=LENS_BARREL_LENGTH),
        origin=Origin(
            xyz=(BODY_LEN * 0.5 + LENS_BARREL_LENGTH * 0.5 - 0.0005, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=lens_finish,
        name="lens_barrel",
    )
    body.visual(
        Cylinder(radius=0.0285, length=0.002),
        origin=Origin(
            xyz=(0.069, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=ring_finish,
        name="lens_shoulder",
    )
    body.visual(
        Cylinder(radius=0.0205, length=0.002),
        origin=Origin(
            xyz=(BODY_LEN * 0.5 + LENS_BARREL_LENGTH - 0.001, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=glass_finish,
        name="lens_glass",
    )
    body.visual(
        Box((0.090, 0.004, 0.046)),
        origin=Origin(xyz=(0.006, -0.044, -0.002)),
        material=grip_finish,
        name="handgrip_pad",
    )
    body.visual(
        Box((0.014, 0.004, MONITOR_HEIGHT - 0.004)),
        origin=Origin(xyz=(MONITOR_HINGE_X, 0.031, MONITOR_HINGE_Z + MONITOR_HEIGHT * 0.5)),
        material=accent_finish,
        name="monitor_mount",
    )
    body.visual(
        Box((PORT_COVER_LEN + 0.004, 0.0025, 0.008)),
        origin=Origin(xyz=(PORT_COVER_HINGE_X, 0.03025, PORT_COVER_HINGE_Z)),
        material=accent_finish,
        name="port_mount",
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(_lens_ring_mesh(), material=ring_finish, name="ring_shell")

    side_monitor = model.part("side_monitor")
    side_monitor.visual(
        Box((MONITOR_LEN, MONITOR_THICKNESS, MONITOR_HEIGHT)),
        origin=Origin(
            xyz=(0.0035 + MONITOR_LEN * 0.5, MONITOR_THICKNESS * 0.5, MONITOR_HEIGHT * 0.5)
        ),
        material=trim_finish,
        name="screen_panel",
    )
    side_monitor.visual(
        Box((MONITOR_LEN - 0.018, 0.0014, MONITOR_HEIGHT - 0.016)),
        origin=Origin(
            xyz=(0.0035 + MONITOR_LEN * 0.5, 0.0007, MONITOR_HEIGHT * 0.5),
        ),
        material=glass_finish,
        name="screen_glass",
    )
    side_monitor.visual(
        Cylinder(radius=MONITOR_HINGE_RADIUS, length=MONITOR_HEIGHT - 0.004),
        origin=Origin(xyz=(0.0, MONITOR_HINGE_RADIUS, MONITOR_HEIGHT * 0.5)),
        material=trim_finish,
        name="monitor_hinge",
    )

    port_cover = model.part("port_cover")
    port_cover.visual(
        Box((PORT_COVER_LEN, PORT_COVER_THICKNESS, PORT_COVER_HEIGHT)),
        origin=Origin(xyz=(0.0, PORT_COVER_THICKNESS * 0.5, PORT_COVER_HEIGHT * 0.5)),
        material=trim_finish,
        name="cover_panel",
    )
    port_cover.visual(
        Cylinder(radius=PORT_COVER_HINGE_RADIUS, length=PORT_COVER_LEN),
        origin=Origin(
            xyz=(0.0, PORT_COVER_HINGE_RADIUS, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=body_finish,
        name="cover_hinge",
    )

    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(BODY_LEN * 0.5 + LENS_BARREL_LENGTH - 0.004, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "body_to_side_monitor",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_monitor,
        origin=Origin(xyz=(MONITOR_HINGE_X, MONITOR_HINGE_Y, MONITOR_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "body_to_port_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=port_cover,
        origin=Origin(xyz=(PORT_COVER_HINGE_X, PORT_COVER_HINGE_Y, PORT_COVER_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lens_ring = object_model.get_part("lens_ring")
    side_monitor = object_model.get_part("side_monitor")
    port_cover = object_model.get_part("port_cover")

    lens_joint = object_model.get_articulation("body_to_lens_ring")
    monitor_joint = object_model.get_articulation("body_to_side_monitor")
    cover_joint = object_model.get_articulation("body_to_port_cover")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    with ctx.pose({monitor_joint: 0.0, cover_joint: 0.0, lens_joint: 0.0}):
        ctx.expect_gap(
            side_monitor,
            body,
            axis="y",
            positive_elem="screen_panel",
            negative_elem="housing_shell",
            min_gap=0.0,
            max_gap=0.005,
            name="monitor closes flush to the camcorder side",
        )
        ctx.expect_overlap(
            side_monitor,
            body,
            axes="xz",
            elem_a="screen_panel",
            elem_b="housing_shell",
            min_overlap=0.030,
            name="closed monitor covers the side wall footprint",
        )
        ctx.expect_gap(
            port_cover,
            body,
            axis="y",
            positive_elem="cover_panel",
            negative_elem="housing_shell",
            min_gap=0.0,
            max_gap=0.004,
            name="port cover sits flush under the monitor mount",
        )
        ctx.expect_overlap(
            port_cover,
            body,
            axes="xz",
            elem_a="cover_panel",
            elem_b="housing_shell",
            min_overlap=0.014,
            name="port cover remains seated in the side wall opening area",
        )
        ctx.expect_overlap(
            lens_ring,
            body,
            axes="yz",
            elem_a="ring_shell",
            elem_b="lens_barrel",
            min_overlap=0.045,
            name="lens ring stays concentric with the fixed barrel",
        )

    closed_monitor_center = None
    open_monitor_center = None
    with ctx.pose({monitor_joint: 0.0}):
        closed_monitor_center = _aabb_center(
            ctx.part_element_world_aabb(side_monitor, elem="screen_panel")
        )
    with ctx.pose({monitor_joint: 1.75}):
        open_monitor_center = _aabb_center(
            ctx.part_element_world_aabb(side_monitor, elem="screen_panel")
        )
    ctx.check(
        "monitor swings outward from the side hinge",
        closed_monitor_center is not None
        and open_monitor_center is not None
        and open_monitor_center[1] > closed_monitor_center[1] + 0.030,
        details=f"closed={closed_monitor_center}, open={open_monitor_center}",
    )

    closed_cover_center = None
    open_cover_center = None
    with ctx.pose({cover_joint: 0.0}):
        closed_cover_center = _aabb_center(
            ctx.part_element_world_aabb(port_cover, elem="cover_panel")
        )
    with ctx.pose({cover_joint: 1.15}):
        open_cover_center = _aabb_center(
            ctx.part_element_world_aabb(port_cover, elem="cover_panel")
        )
    ctx.check(
        "port cover drops outward from its bottom hinge",
        closed_cover_center is not None
        and open_cover_center is not None
        and open_cover_center[1] > closed_cover_center[1] + 0.006
        and open_cover_center[2] < closed_cover_center[2] - 0.002,
        details=f"closed={closed_cover_center}, open={open_cover_center}",
    )

    limits = lens_joint.motion_limits
    ctx.check(
        "lens ring uses continuous unbounded rotation",
        lens_joint.articulation_type == ArticulationType.CONTINUOUS
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"joint_type={lens_joint.articulation_type}, limits={limits}",
    )

    return ctx.report()


object_model = build_object_model()
