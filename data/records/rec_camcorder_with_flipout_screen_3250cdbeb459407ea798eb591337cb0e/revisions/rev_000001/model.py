from __future__ import annotations

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


BODY_LEN = 0.175
BODY_W = 0.078
BODY_H = 0.092

SCREEN_LEN = 0.082
SCREEN_H = 0.058
SCREEN_THICK = 0.0065
SCREEN_CENTER_X = -0.008
SCREEN_CENTER_Z = 0.004
SCREEN_RECESS_DEPTH = 0.0035

BUTTON_SIZE_X = 0.011
BUTTON_SIZE_Y = 0.0038
BUTTON_SIZE_Z = 0.0065
BUTTON_PROUD = 0.0012
BUTTON_TRAVEL = 0.0014
BUTTON_Z = -0.031
BUTTON_XS = (-0.006, 0.010, 0.026)

RING_LEN = 0.010
RING_CENTER_X = BODY_LEN / 2.0 + 0.125


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_LEN, BODY_W, BODY_H).edges("|Z").fillet(0.008)

    rear_pack = (
        cq.Workplane("XY")
        .box(0.060, 0.052, 0.028)
        .edges("|Z").fillet(0.004)
        .translate((-0.048, 0.0, BODY_H / 2.0 + 0.010))
    )
    body = body.union(rear_pack)

    handgrip = (
        cq.Workplane("XY")
        .box(0.082, 0.030, 0.078)
        .edges("|X").fillet(0.006)
        .translate((0.018, BODY_W / 2.0 + 0.011, -0.004))
    )
    body = body.union(handgrip)

    grip_nose = (
        cq.Workplane("XY")
        .box(0.030, 0.018, 0.062)
        .edges("|X").fillet(0.004)
        .translate((0.062, BODY_W / 2.0 + 0.010, 0.002))
    )
    body = body.union(grip_nose)

    barrel = (
        cq.Workplane("YZ")
        .circle(0.034)
        .extrude(0.110)
        .translate((BODY_LEN / 2.0 - 0.010, 0.0, 0.0))
    )
    hood = (
        cq.Workplane("YZ")
        .circle(0.031)
        .extrude(0.020)
        .translate((BODY_LEN / 2.0 + 0.100, 0.0, 0.0))
    )
    body = body.union(barrel).union(hood)

    eyepiece = (
        cq.Workplane("YZ")
        .circle(0.011)
        .extrude(0.024)
        .translate((-BODY_LEN / 2.0 - 0.020, -0.010, 0.014))
    )
    body = body.union(eyepiece)

    screen_recess = (
        cq.Workplane("XY")
        .box(SCREEN_LEN - 0.004, SCREEN_RECESS_DEPTH + 0.001, SCREEN_H - 0.004)
        .translate(
            (
                SCREEN_CENTER_X,
                -BODY_W / 2.0 + SCREEN_RECESS_DEPTH / 2.0 - 0.0005,
                SCREEN_CENTER_Z,
            )
        )
    )
    body = body.cut(screen_recess)

    for button_x in BUTTON_XS:
        button_pocket = (
            cq.Workplane("XY")
            .box(BUTTON_SIZE_X, 0.0050, BUTTON_SIZE_Z)
            .translate((button_x, -BODY_W / 2.0 + 0.0020, BUTTON_Z))
        )
        body = body.cut(button_pocket)

    return body


def _screen_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(SCREEN_LEN, SCREEN_THICK, SCREEN_H)
        .edges("|Z").fillet(0.0025)
        .translate((SCREEN_LEN / 2.0, -SCREEN_THICK / 2.0, 0.0))
    )
    display_recess = (
        cq.Workplane("XY")
        .box(SCREEN_LEN - 0.014, 0.0028, SCREEN_H - 0.014)
        .translate((SCREEN_LEN / 2.0 + 0.001, -0.0012, 0.0))
    )
    hinge_spine = (
        cq.Workplane("XY")
        .circle(0.0036)
        .extrude(SCREEN_H)
        .translate((0.0, -0.0036, -SCREEN_H / 2.0))
    )
    return panel.cut(display_recess).union(hinge_spine)


def _ring_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("YZ")
        .circle(0.0385)
        .extrude(RING_LEN)
        .translate((-RING_LEN / 2.0, 0.0, 0.0))
    )
    ring = ring.cut(
        cq.Workplane("YZ")
        .circle(0.0265)
        .extrude(RING_LEN + 0.004)
        .translate((-RING_LEN / 2.0 - 0.002, 0.0, 0.0))
    )
    for groove_x in (-0.003, 0.0, 0.003):
        ring = ring.cut(
            cq.Workplane("YZ")
            .circle(0.0390)
            .circle(0.0368)
            .extrude(0.0012)
            .translate((groove_x - 0.0006, 0.0, 0.0))
        )
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prosumer_camcorder")

    body_mat = model.material("body_mat", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber_mat = model.material("rubber_mat", rgba=(0.08, 0.08, 0.09, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.25, 0.26, 0.28, 1.0))
    glass_mat = model.material("glass_mat", rgba=(0.04, 0.06, 0.08, 1.0))
    button_mat = model.material("button_mat", rgba=(0.12, 0.13, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "camcorder_body"),
        material=body_mat,
        name="shell",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.002),
        origin=Origin(
            xyz=(BODY_LEN / 2.0 + 0.101, 0.0, 0.0),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=glass_mat,
        name="front_glass",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_ring_shape(), "focus_ring"),
        material=rubber_mat,
        name="ring",
    )

    monitor = model.part("monitor")
    monitor.visual(
        mesh_from_cadquery(_screen_shape(), "monitor_panel"),
        material=trim_mat,
        name="panel",
    )
    monitor.visual(
        Box((SCREEN_LEN - 0.020, 0.0024, SCREEN_H - 0.018)),
        origin=Origin(xyz=(SCREEN_LEN / 2.0 + 0.001, -0.0018, 0.0)),
        material=glass_mat,
        name="display",
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_SIZE_X, BUTTON_SIZE_Y, BUTTON_SIZE_Z)),
            material=button_mat,
            name="cap",
        )
        model.articulation(
            f"button_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(
                xyz=(
                    button_x,
                    -BODY_W / 2.0 - BUTTON_PROUD + BUTTON_SIZE_Y / 2.0,
                    BUTTON_Z,
                )
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.03,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    model.articulation(
        "focus_ring_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(RING_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )
    model.articulation(
        "monitor_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=monitor,
        origin=Origin(
            xyz=(
                SCREEN_CENTER_X - SCREEN_LEN / 2.0,
                -BODY_W / 2.0,
                SCREEN_CENTER_Z,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    monitor = object_model.get_part("monitor")
    focus_ring = object_model.get_part("focus_ring")
    monitor_hinge = object_model.get_articulation("monitor_hinge")
    ring_joint = object_model.get_articulation("focus_ring_spin")
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"button_{index}_slide") for index in range(3)]

    ctx.expect_gap(
        body,
        monitor,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        name="monitor closes nearly flush to the body side",
    )
    ctx.expect_overlap(
        monitor,
        body,
        axes="xz",
        min_overlap=0.040,
        name="monitor covers the left-side recess",
    )
    ctx.expect_origin_distance(
        body,
        focus_ring,
        axes="yz",
        max_dist=0.001,
        name="focus ring stays on the lens axis",
    )
    ctx.expect_origin_gap(
        focus_ring,
        body,
        axis="x",
        min_gap=0.20,
        max_gap=0.23,
        name="focus ring sits forward of the camera body",
    )

    rest_monitor_aabb = ctx.part_world_aabb(monitor)
    opened_monitor_aabb = None
    limits = monitor_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({monitor_hinge: limits.upper}):
            opened_monitor_aabb = ctx.part_world_aabb(monitor)
            ctx.expect_overlap(
                monitor,
                body,
                axes="z",
                min_overlap=0.040,
                name="opened monitor keeps its vertical hinge alignment",
            )
    ctx.check(
        "monitor swings outward from the body",
        rest_monitor_aabb is not None
        and opened_monitor_aabb is not None
        and opened_monitor_aabb[0][1] < rest_monitor_aabb[0][1] - 0.030,
        details=f"rest={rest_monitor_aabb}, opened={opened_monitor_aabb}",
    )

    rest_ring_pos = ctx.part_world_position(focus_ring)
    with ctx.pose({ring_joint: 1.2}):
        rotated_ring_pos = ctx.part_world_position(focus_ring)
    ctx.check(
        "focus ring rotates without translating off axis",
        rest_ring_pos is not None
        and rotated_ring_pos is not None
        and abs(rotated_ring_pos[0] - rest_ring_pos[0]) < 1e-9
        and abs(rotated_ring_pos[1] - rest_ring_pos[1]) < 1e-9
        and abs(rotated_ring_pos[2] - rest_ring_pos[2]) < 1e-9,
        details=f"rest={rest_ring_pos}, rotated={rotated_ring_pos}",
    )

    rest_button_aabbs = [ctx.part_world_aabb(button) for button in buttons]
    for index, (button, joint) in enumerate(zip(buttons, button_joints)):
        limits = joint.motion_limits
        pressed_aabbs = None
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                pressed_aabbs = [ctx.part_world_aabb(item) for item in buttons]
        moved_ok = (
            rest_button_aabbs[index] is not None
            and pressed_aabbs is not None
            and pressed_aabbs[index] is not None
            and pressed_aabbs[index][0][1] > rest_button_aabbs[index][0][1] + 0.0010
        )
        other_static = (
            pressed_aabbs is not None
            and all(
                rest_button_aabbs[other] is not None
                and pressed_aabbs[other] is not None
                and abs(pressed_aabbs[other][0][1] - rest_button_aabbs[other][0][1]) < 1e-6
                for other in range(3)
                if other != index
            )
        )
        ctx.check(
            f"button_{index} depresses independently",
            moved_ok and other_static,
            details=f"rest={rest_button_aabbs}, pressed={pressed_aabbs}",
        )

    return ctx.report()


object_model = build_object_model()
