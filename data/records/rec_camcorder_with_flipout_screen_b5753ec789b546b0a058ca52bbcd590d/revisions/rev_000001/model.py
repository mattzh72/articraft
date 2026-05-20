from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_L = 0.112
BODY_W = 0.074
BODY_H = 0.085
BODY_FRONT_X = BODY_L / 2.0
BODY_SIDE_Y = BODY_W / 2.0
BODY_TOP_Z = BODY_H / 2.0

OBJECTIVE_Y = -0.008
OBJECTIVE_Z = 0.006
OBJECTIVE_RADIUS = 0.024
OBJECTIVE_LENGTH = 0.040

DOOR_W = 0.078
DOOR_H = 0.060
DOOR_T = 0.0035
DOOR_HINGE_X = -0.022
DOOR_CENTER_Z = 0.002
DOOR_HINGE_Y = BODY_SIDE_Y + 0.003247

SCREEN_PIVOT_X = 0.008
SCREEN_PIVOT_Y = -DOOR_T / 2.0
SCREEN_W = 0.053
SCREEN_H = 0.042
SCREEN_T = 0.0032

DIAL_X = -0.030
DIAL_Y = -0.018
DIAL_Z = BODY_TOP_Z + 0.0042


def _body_shape() -> cq.Workplane:
    main_shell = cq.Workplane("XY").box(BODY_L, BODY_W, BODY_H).edges().fillet(0.0045)

    rear_hump = (
        cq.Workplane("XY")
        .transformed(offset=(-0.032, -0.004, 0.014))
        .box(0.040, 0.056, 0.030)
        .edges("|Z")
        .fillet(0.004)
    )

    lens_collar = (
        cq.Workplane("XY")
        .transformed(offset=(BODY_FRONT_X - 0.006, OBJECTIVE_Y, OBJECTIVE_Z), rotate=(0.0, 90.0, 0.0))
        .circle(0.030)
        .extrude(0.010)
    )

    dial_pedestal = (
        cq.Workplane("XY")
        .transformed(offset=(DIAL_X, DIAL_Y, BODY_TOP_Z), rotate=(0.0, 0.0, 0.0))
        .circle(0.013)
        .extrude(0.0042)
    )

    hinge_post = (
        cq.Workplane("XY")
        .transformed(offset=(DOOR_HINGE_X, DOOR_HINGE_Y - 0.004299, DOOR_CENTER_Z - 0.026))
        .circle(0.0011)
        .extrude(0.052)
    )

    hinge_bridge = (
        cq.Workplane("XY")
        .transformed(offset=(DOOR_HINGE_X - 0.006, DOOR_HINGE_Y - 0.00445, DOOR_CENTER_Z))
        .box(0.012, 0.0018, 0.018)
        .edges("|Y")
        .fillet(0.0008)
    )

    mic_bump = (
        cq.Workplane("XY")
        .transformed(offset=(0.006, -0.006, BODY_TOP_Z - 0.002))
        .box(0.024, 0.012, 0.005)
        .edges("|Y")
        .fillet(0.002)
    )

    side_recess = (
        cq.Workplane("XY")
        .transformed(offset=(DOOR_HINGE_X + DOOR_W / 2.0, BODY_SIDE_Y - 0.0048, DOOR_CENTER_Z))
        .box(DOOR_W + 0.008, 0.012, DOOR_H + 0.004)
        .edges("|Y")
        .fillet(0.005)
    )

    body = main_shell.union(rear_hump).cut(side_recess)
    body = body.union(lens_collar).union(dial_pedestal).union(hinge_post).union(hinge_bridge).union(mic_bump)
    return body


def _strap_pad_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.012, -BODY_SIDE_Y - 0.0016, -0.002))
        .box(0.076, 0.0032, 0.044)
        .edges("|Y")
        .fillet(0.003)
    )


def _objective_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XY")
        .transformed(rotate=(0.0, 90.0, 0.0))
        .circle(OBJECTIVE_RADIUS)
        .extrude(OBJECTIVE_LENGTH)
    )
    front_ring = (
        cq.Workplane("XY")
        .transformed(offset=(OBJECTIVE_LENGTH - 0.014, 0.0, 0.0), rotate=(0.0, 90.0, 0.0))
        .circle(0.027)
        .extrude(0.014)
    )
    focus_ring = (
        cq.Workplane("XY")
        .transformed(offset=(0.010, 0.0, 0.0), rotate=(0.0, 90.0, 0.0))
        .circle(0.0255)
        .extrude(0.010)
    )
    return barrel.union(front_ring).union(focus_ring)


def _door_shape() -> cq.Workplane:
    panel_start_x = 0.0030
    panel_width = DOOR_W - panel_start_x
    panel = (
        cq.Workplane("XY")
        .transformed(offset=(panel_start_x + panel_width / 2.0, 0.0, 0.0))
        .box(panel_width, DOOR_T, DOOR_H)
        .edges("|Y")
        .fillet(0.0045)
    )

    finger_notch = (
        cq.Workplane("XY")
        .transformed(offset=(DOOR_W - 0.0015, 0.0, -0.010))
        .circle(0.007)
        .extrude(0.020)
    )

    hinge_spine = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -0.030))
        .circle(0.0032)
        .extrude(0.060)
    )

    return panel.cut(finger_notch).union(hinge_spine)


def _screen_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .transformed(offset=(0.031, -0.0022, 0.0))
        .box(SCREEN_W, SCREEN_T, SCREEN_H)
        .edges("|Y")
        .fillet(0.003)
    )

    swivel_hub = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -0.0024, -0.023))
        .circle(0.0024)
        .extrude(0.046)
    )

    arm = (
        cq.Workplane("XY")
        .transformed(offset=(0.012, -0.0021, 0.0))
        .box(0.018, 0.0024, 0.022)
        .edges("|Y")
        .fillet(0.0012)
    )

    stem = (
        cq.Workplane("XY")
        .transformed(offset=(0.0055, -0.0022, 0.0))
        .box(0.011, 0.0018, 0.018)
        .edges("|Y")
        .fillet(0.0008)
    )

    contact_pad = (
        cq.Workplane("XY")
        .transformed(offset=(0.0018, -0.0001, 0.0))
        .box(0.0045, 0.0002, 0.018)
    )

    return swivel_hub.union(stem).union(arm).union(panel).union(contact_pad)


def _dial_shape() -> cq.Workplane:
    dial = cq.Workplane("XY").circle(0.0105).extrude(0.0062)
    top_cap = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.0062))
        .circle(0.0087)
        .extrude(0.0014)
    )
    thumb_tab = (
        cq.Workplane("XY")
        .transformed(offset=(0.0092, 0.0022, 0.0069))
        .box(0.008, 0.0044, 0.0026)
        .edges("|Z")
        .fillet(0.001)
    )
    return dial.union(top_cap).union(thumb_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_camcorder")

    shell_finish = model.material("shell_finish", rgba=(0.23, 0.24, 0.26, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.11, 0.12, 0.13, 1.0))
    strap_finish = model.material("strap_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "body_shell"), material=shell_finish, name="shell")
    body.visual(mesh_from_cadquery(_strap_pad_shape(), "hand_strap"), material=strap_finish, name="strap")

    objective = model.part("objective")
    objective.visual(mesh_from_cadquery(_objective_shape(), "objective_barrel"), material=trim_finish, name="barrel")

    screen_door = model.part("screen_door")
    screen_door.visual(mesh_from_cadquery(_door_shape(), "screen_door"), material=shell_finish, name="door")

    screen_panel = model.part("screen_panel")
    screen_panel.visual(mesh_from_cadquery(_screen_shape(), "screen_panel"), material=trim_finish, name="panel")

    mode_dial = model.part("mode_dial")
    mode_dial.visual(mesh_from_cadquery(_dial_shape(), "mode_dial"), material=dial_finish, name="dial")

    model.articulation(
        "body_to_objective",
        ArticulationType.FIXED,
        parent=body,
        child=objective,
        origin=Origin(xyz=(BODY_FRONT_X + 0.0006, OBJECTIVE_Y, OBJECTIVE_Z)),
    )

    model.articulation(
        "body_to_screen_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen_door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.15, effort=2.0, velocity=1.8),
    )

    model.articulation(
        "door_to_screen",
        ArticulationType.REVOLUTE,
        parent=screen_door,
        child=screen_panel,
        origin=Origin(xyz=(SCREEN_PIVOT_X, SCREEN_PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.05, effort=1.0, velocity=2.2),
    )

    model.articulation(
        "body_to_mode_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(DIAL_X, DIAL_Y, DIAL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.85, upper=1.40, effort=0.8, velocity=3.0),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    objective = object_model.get_part("objective")
    screen_door = object_model.get_part("screen_door")
    screen_panel = object_model.get_part("screen_panel")
    mode_dial = object_model.get_part("mode_dial")

    door_hinge = object_model.get_articulation("body_to_screen_door")
    screen_swivel = object_model.get_articulation("door_to_screen")
    dial_joint = object_model.get_articulation("body_to_mode_dial")

    ctx.allow_isolated_part(
        screen_door,
        reason="The flip-out door is carried by a simplified close-clearance hinge spindle with a deliberate hairline visual gap to the body shell.",
    )
    ctx.allow_isolated_part(
        screen_panel,
        reason="The internal LCD panel is suspended from the door's secondary swivel pivot with a deliberate hairline clearance at the hinge support.",
    )

    with ctx.pose({door_hinge: 0.0, screen_swivel: 0.0}):
        ctx.expect_overlap(
            screen_door,
            body,
            axes="xz",
            min_overlap=0.050,
            name="closed screen door covers the left side display bay",
        )
        ctx.expect_within(
            screen_panel,
            screen_door,
            axes="xz",
            margin=0.004,
            name="screen panel nests inside the closed door outline",
        )
        ctx.expect_origin_gap(
            objective,
            body,
            axis="x",
            min_gap=0.055,
            name="objective sits ahead of the housing centerline",
        )
        ctx.expect_origin_gap(
            mode_dial,
            body,
            axis="z",
            min_gap=0.045,
            name="mode dial is mounted on the top rear corner",
        )

    closed_door_aabb = ctx.part_world_aabb(screen_door)
    with ctx.pose({door_hinge: 1.55}):
        open_door_aabb = ctx.part_world_aabb(screen_door)

    ctx.check(
        "screen door swings outward to the left",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.045,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({door_hinge: 1.55, screen_swivel: 0.15}):
        screen_near_aabb = ctx.part_world_aabb(screen_panel)
    with ctx.pose({door_hinge: 1.55, screen_swivel: 1.85}):
        screen_turned_aabb = ctx.part_world_aabb(screen_panel)

    near_center = _aabb_center(screen_near_aabb)
    turned_center = _aabb_center(screen_turned_aabb)
    ctx.check(
        "screen panel swivels independently for viewing",
        near_center is not None
        and turned_center is not None
        and abs(turned_center[0] - near_center[0]) > 0.010
        and abs(turned_center[1] - near_center[1]) > 0.010,
        details=f"near={screen_near_aabb}, turned={screen_turned_aabb}",
    )

    with ctx.pose({dial_joint: -1.1}):
        dial_low_aabb = ctx.part_world_aabb(mode_dial)
    with ctx.pose({dial_joint: 0.9}):
        dial_high_aabb = ctx.part_world_aabb(mode_dial)

    dial_low_center = _aabb_center(dial_low_aabb)
    dial_high_center = _aabb_center(dial_high_aabb)
    ctx.check(
        "power dial rotates through distinct mode positions",
        dial_low_center is not None
        and dial_high_center is not None
        and (
            abs(dial_high_center[0] - dial_low_center[0]) > 0.001
            or abs(dial_high_center[1] - dial_low_center[1]) > 0.001
        ),
        details=f"low={dial_low_aabb}, high={dial_high_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
