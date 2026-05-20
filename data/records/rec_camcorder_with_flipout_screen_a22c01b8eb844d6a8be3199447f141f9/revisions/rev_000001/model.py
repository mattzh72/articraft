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


BODY_LEN = 0.096
BODY_W = 0.036
BODY_H = 0.056
BODY_LEFT_Y = BODY_W / 2.0

GRIP_LEN = 0.056
GRIP_Y_RADIUS = 0.015
GRIP_Z_RADIUS = 0.028
GRIP_X = 0.006
GRIP_Y = -0.024
GRIP_Z = -0.002

MONITOR_W = 0.068
MONITOR_H = 0.046
MONITOR_T = 0.006
MONITOR_HINGE_X = -0.015
MONITOR_HINGE_Y = BODY_LEFT_Y + 0.0006
MONITOR_Z = 0.004
MONITOR_PANEL_OFFSET_Y = 0.0006 + MONITOR_T / 2.0

DISPLAY_W = 0.056
DISPLAY_H = 0.036
DISPLAY_T = 0.0008
DISPLAY_X = 0.034
DISPLAY_Y = 0.00135

SCREEN_RECESS_DEPTH = 0.0045
SCREEN_RECESS_X = 0.010
SCREEN_RECESS_Z = MONITOR_Z

BUTTON_LEN = 0.008
BUTTON_DEPTH = 0.0035
BUTTON_HEIGHT = 0.007
BUTTON_TRAVEL = 0.002
BUTTON_Z = -0.017
BUTTON_XS = (0.006, 0.017, 0.028)
BUTTON_POCKET_DEPTH = 0.008
BUTTON_PANEL_DEPTH = 0.004
BUTTON_PANEL_LEN = 0.034
BUTTON_PANEL_HEIGHT = 0.010

LENS_Z = 0.008
LENS_AXIS_X = BODY_LEN / 2.0 + 0.012
LENS_HOUSING_R = 0.0184
LENS_HOUSING_LEN = 0.024
LENS_RING_OUTER_R = 0.022
LENS_RING_INNER_R = 0.0184
LENS_RING_LEN = 0.012


def make_body_shell() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_LEN, BODY_W, BODY_H)
    body = body.edges("|Z").fillet(0.005)

    grip = (
        cq.Workplane("YZ")
        .ellipse(GRIP_Y_RADIUS, GRIP_Z_RADIUS)
        .extrude(GRIP_LEN / 2.0, both=True)
        .translate((GRIP_X, GRIP_Y, GRIP_Z))
    )
    body = body.union(grip)

    screen_recess = cq.Workplane("XY").box(
        0.068,
        SCREEN_RECESS_DEPTH + 0.001,
        0.042,
    ).translate(
        (
            SCREEN_RECESS_X,
            BODY_LEFT_Y - SCREEN_RECESS_DEPTH / 2.0 + 0.0005,
            SCREEN_RECESS_Z,
        )
    )
    body = body.cut(screen_recess)

    for x in BUTTON_XS:
        button_pocket = cq.Workplane("XY").box(
            BUTTON_LEN,
            BUTTON_POCKET_DEPTH + 0.001,
            BUTTON_HEIGHT,
        ).translate(
            (
                x,
                BODY_LEFT_Y - BUTTON_POCKET_DEPTH / 2.0 + 0.0005,
                BUTTON_Z,
            )
        )
        body = body.cut(button_pocket)

    hinge_spine = (
        cq.Workplane("XY")
        .circle(0.0032)
        .extrude(0.042, both=True)
        .translate((MONITOR_HINGE_X, BODY_LEFT_Y + 0.0010, MONITOR_Z))
    )
    body = body.union(hinge_spine)

    top_bump = cq.Workplane("XY").box(0.028, 0.018, 0.010).translate((-0.005, -0.003, 0.023))
    body = body.union(top_bump)

    return body


def make_monitor_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(MONITOR_W, MONITOR_T, MONITOR_H).translate(
        (MONITOR_W / 2.0, MONITOR_PANEL_OFFSET_Y, 0.0)
    )
    shell = shell.edges("|Z").fillet(0.002)

    display_recess = cq.Workplane("XY").box(DISPLAY_W, 0.0019, DISPLAY_H).translate(
        (DISPLAY_X, 0.0010, 0.0)
    )
    return shell.cut(display_recess)


def make_button_panel() -> cq.Workplane:
    panel = cq.Workplane("XY").box(BUTTON_PANEL_LEN, BUTTON_PANEL_DEPTH, BUTTON_PANEL_HEIGHT).translate(
        (
            sum(BUTTON_XS) / len(BUTTON_XS),
            BODY_LEFT_Y + BUTTON_PANEL_DEPTH / 2.0,
            BUTTON_Z,
        )
    )
    for x in BUTTON_XS:
        opening = cq.Workplane("XY").box(
            BUTTON_LEN,
            BUTTON_PANEL_DEPTH + 0.001,
            BUTTON_HEIGHT,
        ).translate((x, BODY_LEFT_Y + BUTTON_PANEL_DEPTH / 2.0, BUTTON_Z))
        panel = panel.cut(opening)
    return panel


def make_lens_ring() -> cq.Workplane:
    ring = (
        cq.Workplane("YZ")
        .circle(LENS_RING_OUTER_R)
        .circle(LENS_RING_INNER_R)
        .extrude(LENS_RING_LEN / 2.0, both=True)
    )
    front_lip = (
        cq.Workplane("YZ")
        .circle(LENS_RING_OUTER_R + 0.0014)
        .circle(LENS_RING_INNER_R + 0.0006)
        .extrude(0.0022)
        .translate((LENS_RING_LEN / 2.0 - 0.0022, 0.0, 0.0))
    )
    return ring.union(front_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_hd_camcorder")

    body_mat = model.material("body_mat", rgba=(0.12, 0.12, 0.13, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.19, 0.19, 0.20, 1.0))
    glass_mat = model.material("glass_mat", rgba=(0.16, 0.22, 0.28, 1.0))
    button_mat = model.material("button_mat", rgba=(0.28, 0.29, 0.30, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shell(), "body_shell"),
        material=body_mat,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=LENS_HOUSING_R, length=LENS_HOUSING_LEN),
        origin=Origin(
            xyz=(LENS_AXIS_X, 0.0, LENS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_mat,
        name="lens_housing",
    )
    body.visual(
        Cylinder(radius=0.0145, length=0.0015),
        origin=Origin(
            xyz=(LENS_AXIS_X + LENS_HOUSING_LEN / 2.0 - 0.0010, 0.0, LENS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=glass_mat,
        name="lens_glass",
    )
    body.visual(
        mesh_from_cadquery(make_button_panel(), "button_panel"),
        material=trim_mat,
        name="button_panel",
    )

    monitor = model.part("monitor")
    monitor.visual(
        mesh_from_cadquery(make_monitor_shell(), "monitor_shell"),
        material=body_mat,
        name="monitor_shell",
    )
    monitor.visual(
        Box((DISPLAY_W, DISPLAY_T, DISPLAY_H)),
        origin=Origin(xyz=(DISPLAY_X, DISPLAY_Y, 0.0)),
        material=glass_mat,
        name="display",
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        mesh_from_cadquery(make_lens_ring(), "lens_ring"),
        material=trim_mat,
        name="ring",
    )

    for index, x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_LEN, BUTTON_DEPTH, BUTTON_HEIGHT)),
            origin=Origin(xyz=(0.0, BUTTON_DEPTH / 2.0, 0.0)),
            material=button_mat,
            name="button",
        )
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, BODY_LEFT_Y + BUTTON_TRAVEL, BUTTON_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    model.articulation(
        "monitor_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=monitor,
        origin=Origin(xyz=(MONITOR_HINGE_X, MONITOR_HINGE_Y, MONITOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=2.1),
    )
    model.articulation(
        "lens_ring_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(LENS_AXIS_X, 0.0, LENS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    monitor = object_model.get_part("monitor")
    lens_ring = object_model.get_part("lens_ring")
    monitor_hinge = object_model.get_articulation("monitor_hinge")
    lens_ring_hinge = object_model.get_articulation("lens_ring_spin")

    ctx.allow_overlap(
        body,
        lens_ring,
        elem_a="lens_housing",
        elem_b="ring",
        reason="The focus ring intentionally nests concentrically around the front lens housing.",
    )

    ctx.expect_overlap(
        monitor,
        body,
        axes="xz",
        elem_a="monitor_shell",
        elem_b="body_shell",
        min_overlap=0.030,
        name="monitor covers the side screen opening when closed",
    )

    closed_center = _aabb_center(ctx.part_world_aabb(monitor))
    limits = monitor_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({monitor_hinge: limits.upper}):
            open_center = _aabb_center(ctx.part_world_aabb(monitor))
        ctx.check(
            "monitor swings outward from the body",
            closed_center is not None
            and open_center is not None
            and open_center[1] > closed_center[1] + 0.010,
            details=f"closed_center={closed_center}, open_center={open_center}",
        )

    lens_limits = lens_ring_hinge.motion_limits
    ctx.check(
        "lens ring uses continuous rotation limits",
        lens_limits is not None and lens_limits.lower is None and lens_limits.upper is None,
        details=f"limits={lens_limits}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"button_{index}_press")
        rest_pos = ctx.part_world_position(button)
        press_limits = joint.motion_limits
        if press_limits is not None and press_limits.upper is not None:
            with ctx.pose({joint: press_limits.upper}):
                pressed_pos = ctx.part_world_position(button)
            ctx.check(
                f"button_{index} depresses inward",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[1] < rest_pos[1] - 0.0015,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_0_joint = object_model.get_articulation("button_0_press")
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_0_joint: BUTTON_TRAVEL}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_same = ctx.part_world_position(button_1)
    button_0_rest = ctx.part_world_position(button_0)
    ctx.check(
        "front buttons articulate independently",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_same is not None
        and button_0_pressed[1] < button_0_rest[1] - 0.0015
        and abs(button_1_same[1] - button_1_rest[1]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_same={button_1_same}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
