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


BODY_LENGTH = 0.112
BODY_WIDTH = 0.052
BODY_HEIGHT = 0.070
MONITOR_LENGTH = 0.064
MONITOR_HEIGHT = 0.046
MONITOR_THICKNESS = 0.005
BUTTON_WIDTH = 0.010
BUTTON_HEIGHT = 0.0065
BUTTON_DEPTH = 0.004
BUTTON_TRAVEL = 0.0018
BUTTON_SHAFT_WIDTH = 0.0074
BUTTON_SHAFT_HEIGHT = 0.0044
BUTTON_SHAFT_DEPTH = 0.0022
BUTTON_POCKET_DEPTH = BUTTON_SHAFT_DEPTH + BUTTON_TRAVEL + 0.0002
BUTTON_X_POSITIONS = (-0.010, 0.004, 0.018)
BUTTON_Z = -0.022


def _make_body_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.009)
        .edges(">Z")
        .fillet(0.004)
    )

    grip = (
        cq.Workplane("YZ")
        .circle(0.015)
        .extrude(0.074)
        .translate((-0.018, -0.034, -0.010))
    )

    screen_cut = (
        cq.Workplane("XY")
        .box(0.054, 0.004, MONITOR_HEIGHT + 0.004)
        .translate((-0.012, BODY_WIDTH * 0.5 - 0.002, 0.004))
    )

    button_cuts = None
    for button_x in BUTTON_X_POSITIONS:
        cut = (
            cq.Workplane("XY")
            .box(BUTTON_SHAFT_WIDTH, BUTTON_POCKET_DEPTH, BUTTON_SHAFT_HEIGHT)
            .translate((button_x, BODY_WIDTH * 0.5 - BUTTON_POCKET_DEPTH * 0.5, BUTTON_Z))
        )
        button_cuts = cut if button_cuts is None else button_cuts.union(cut)

    body = shell.union(grip).cut(screen_cut)
    if button_cuts is not None:
        body = body.cut(button_cuts)
    return body


def _make_lens_barrel_shape() -> cq.Workplane:
    barrel = cq.Workplane("YZ").circle(0.0152).extrude(0.032)
    rear_collar = cq.Workplane("YZ").circle(0.0172).extrude(0.006)
    front_bezel = cq.Workplane("YZ").circle(0.0160).extrude(0.004).translate((0.028, 0.0, 0.0))
    front_recess = cq.Workplane("YZ").circle(0.0105).extrude(0.004).translate((0.028, 0.0, 0.0))
    return barrel.union(rear_collar).union(front_bezel).cut(front_recess)


def _make_lens_ring_shape() -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(0.0186).extrude(0.008)
    inner = cq.Workplane("YZ").circle(0.0156).extrude(0.008)
    return outer.cut(inner)


def _make_button_cap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)
        .edges("|Y")
        .fillet(0.0012)
        .translate((0.0, BUTTON_DEPTH * 0.5, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="palm_camcorder")

    shell_mat = model.material("shell_mat", rgba=(0.16, 0.17, 0.19, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.23, 0.24, 0.26, 1.0))
    lens_mat = model.material("lens_mat", rgba=(0.08, 0.08, 0.09, 1.0))
    ring_mat = model.material("ring_mat", rgba=(0.20, 0.21, 0.22, 1.0))
    screen_mat = model.material("screen_mat", rgba=(0.10, 0.16, 0.20, 0.70))
    button_mat = model.material("button_mat", rgba=(0.52, 0.54, 0.57, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "camcorder_body"),
        material=shell_mat,
        name="shell",
    )

    lens_barrel = model.part("lens_barrel")
    lens_barrel.visual(
        mesh_from_cadquery(_make_lens_barrel_shape(), "camcorder_lens_barrel"),
        material=lens_mat,
        name="barrel",
    )
    model.articulation(
        "body_to_lens_barrel",
        ArticulationType.FIXED,
        parent=body,
        child=lens_barrel,
        origin=Origin(xyz=(BODY_LENGTH * 0.5, -0.004, 0.004)),
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        mesh_from_cadquery(_make_lens_ring_shape(), "camcorder_lens_ring"),
        material=ring_mat,
        name="ring",
    )
    model.articulation(
        "lens_barrel_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=lens_barrel,
        child=lens_ring,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    monitor_hinge = model.part("monitor_hinge")
    monitor_hinge.visual(
        Box((0.010, 0.006, MONITOR_HEIGHT)),
        origin=Origin(xyz=(0.005, 0.0, MONITOR_HEIGHT * 0.5)),
        material=trim_mat,
        name="block",
    )
    monitor_hinge.visual(
        Cylinder(radius=0.0032, length=0.014),
        origin=Origin(xyz=(0.0015, 0.0, 0.011)),
        material=trim_mat,
        name="barrel_lower",
    )
    monitor_hinge.visual(
        Cylinder(radius=0.0032, length=0.014),
        origin=Origin(xyz=(0.0015, 0.0, MONITOR_HEIGHT - 0.011)),
        material=trim_mat,
        name="barrel_upper",
    )
    model.articulation(
        "body_to_monitor_hinge",
        ArticulationType.FIXED,
        parent=body,
        child=monitor_hinge,
        origin=Origin(xyz=(-0.048, BODY_WIDTH * 0.5 + 0.003, -0.018)),
    )

    monitor = model.part("monitor")
    monitor.visual(
        Box((MONITOR_LENGTH, MONITOR_THICKNESS, MONITOR_HEIGHT)),
        origin=Origin(
            xyz=(
                MONITOR_LENGTH * 0.5,
                MONITOR_THICKNESS * 0.5,
                MONITOR_HEIGHT * 0.5,
            )
        ),
        material=trim_mat,
        name="shell",
    )
    monitor.visual(
        Box((MONITOR_LENGTH - 0.012, 0.0012, MONITOR_HEIGHT - 0.010)),
        origin=Origin(
            xyz=(
                MONITOR_LENGTH * 0.5 + 0.001,
                MONITOR_THICKNESS - 0.0006,
                MONITOR_HEIGHT * 0.5,
            )
        ),
        material=screen_mat,
        name="screen",
    )
    model.articulation(
        "monitor_hinge_to_monitor",
        ArticulationType.REVOLUTE,
        parent=monitor_hinge,
        child=monitor,
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=0.0, upper=2.2),
    )

    button_mesh = mesh_from_cadquery(_make_button_cap_shape(), "camcorder_button_cap")
    for index, button_x in enumerate(BUTTON_X_POSITIONS):
        button = model.part(f"button_{index}")
        button.visual(button_mesh, material=button_mat, name="cap")
        button.visual(
            Box((BUTTON_SHAFT_WIDTH, BUTTON_SHAFT_DEPTH, BUTTON_SHAFT_HEIGHT)),
            origin=Origin(xyz=(0.0, -BUTTON_SHAFT_DEPTH * 0.5, 0.0)),
            material=button_mat,
            name="shaft",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, BODY_WIDTH * 0.5, BUTTON_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.03,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens_barrel = object_model.get_part("lens_barrel")
    lens_ring = object_model.get_part("lens_ring")
    monitor = object_model.get_part("monitor")
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    ring_joint = object_model.get_articulation("lens_barrel_to_lens_ring")
    monitor_joint = object_model.get_articulation("monitor_hinge_to_monitor")
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]

    with ctx.pose({monitor_joint: 0.0}):
        ctx.expect_overlap(
            monitor,
            body,
            axes="xz",
            elem_a="shell",
            elem_b="shell",
            min_overlap=0.040,
            name="closed monitor covers the side recess",
        )
        ctx.expect_gap(
            monitor,
            body,
            axis="y",
            positive_elem="shell",
            negative_elem="shell",
            min_gap=0.004,
            max_gap=0.012,
            name="closed monitor sits just off the camcorder side",
        )

    closed_monitor_max_y = None
    open_monitor_max_y = None
    with ctx.pose({monitor_joint: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(monitor, elem="shell")
        if closed_aabb is not None:
            closed_monitor_max_y = closed_aabb[1][1]
    with ctx.pose({monitor_joint: 1.3}):
        open_aabb = ctx.part_element_world_aabb(monitor, elem="shell")
        if open_aabb is not None:
            open_monitor_max_y = open_aabb[1][1]
    ctx.check(
        "monitor swings outward from the body",
        closed_monitor_max_y is not None
        and open_monitor_max_y is not None
        and open_monitor_max_y > closed_monitor_max_y + 0.020,
        details=f"closed_max_y={closed_monitor_max_y}, open_max_y={open_monitor_max_y}",
    )

    ctx.expect_overlap(
        lens_ring,
        lens_barrel,
        axes="yz",
        elem_a="ring",
        elem_b="barrel",
        min_overlap=0.028,
        name="lens ring stays concentric on the barrel",
    )
    with ctx.pose({ring_joint: 1.7}):
        ctx.expect_overlap(
            lens_ring,
            lens_barrel,
            axes="yz",
            elem_a="ring",
            elem_b="barrel",
            min_overlap=0.028,
            name="lens ring remains coaxial while rotated",
        )

    for index, button in enumerate(buttons):
        ctx.expect_gap(
            button,
            body,
            axis="y",
            positive_elem="cap",
            negative_elem="shell",
            min_gap=0.0,
            max_gap=0.0045,
            name=f"button_{index} sits proud of the body wall",
        )

    for index, (button, joint) in enumerate(zip(buttons, button_joints)):
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
            ctx.expect_gap(
                button,
                body,
                axis="y",
                positive_elem="cap",
                negative_elem="shell",
                max_gap=0.003,
                max_penetration=BUTTON_TRAVEL + 0.0002,
                name=f"button_{index} can depress into its pocket",
            )
        ctx.check(
            f"button_{index} moves inward when pressed",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
