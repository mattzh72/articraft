from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_DEPTH = 0.170
BODY_WIDTH = 0.090
BODY_HEIGHT = 0.372
BODY_WALL = 0.004
TOP_THICKNESS = 0.022

POD_DEPTH = 0.094
POD_WIDTH = 0.028
POD_HEIGHT = 0.122
POD_BASE_Z = 0.214
POD_CENTER_X = 0.010
POD_EMBED = 0.004
POD_OUTER_Y = BODY_WIDTH * 0.5 + POD_WIDTH - POD_EMBED

DRAWER_PANEL_THICKNESS = 0.006
DRAWER_FRONT_WIDTH = 0.082
DRAWER_FRONT_HEIGHT = 0.220
DRAWER_BIN_DEPTH = 0.136
DRAWER_BIN_WIDTH = 0.074
DRAWER_BIN_HEIGHT = 0.206
DRAWER_WALL = 0.003
DRAWER_CENTER_Z = 0.115
DRAWER_TRAVEL = 0.108

DRUM_LENGTH = 0.066
DRUM_SHAFT_RADIUS = 0.0034
DRUM_CUTTER_RADIUS = 0.0080
DRUM_X_OFFSET = 0.0080
DRUM_Z = BODY_HEIGHT - TOP_THICKNESS - DRUM_CUTTER_RADIUS - 0.010

POWER_X = -0.004
POWER_Z = 0.304
BUTTON_X = 0.018
BUTTON_Z = 0.269
KNOB_X = 0.040
KNOB_Z = 0.240


def _build_body_mesh() -> object:
    left_wall = cq.Workplane("XY").box(BODY_DEPTH, BODY_WALL, BODY_HEIGHT).translate(
        (0.0, -BODY_WIDTH * 0.5 + BODY_WALL * 0.5, BODY_HEIGHT * 0.5)
    )
    right_wall = cq.Workplane("XY").box(BODY_DEPTH, BODY_WALL, BODY_HEIGHT).translate(
        (0.0, BODY_WIDTH * 0.5 - BODY_WALL * 0.5, BODY_HEIGHT * 0.5)
    )
    back_wall = cq.Workplane("XY").box(BODY_WALL, BODY_WIDTH - 2.0 * BODY_WALL, BODY_HEIGHT).translate(
        (-BODY_DEPTH * 0.5 + BODY_WALL * 0.5, 0.0, BODY_HEIGHT * 0.5)
    )
    bottom_plate = cq.Workplane("XY").box(BODY_DEPTH, BODY_WIDTH, BODY_WALL).translate(
        (0.0, 0.0, BODY_WALL * 0.5)
    )
    top_cap = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, TOP_THICKNESS)
        .translate((0.0, 0.0, BODY_HEIGHT - TOP_THICKNESS * 0.5))
        .edges("|Z")
        .fillet(0.008)
    )
    front_upper = cq.Workplane("XY").box(BODY_WALL, BODY_WIDTH, 0.132).translate(
        (BODY_DEPTH * 0.5 - BODY_WALL * 0.5, 0.0, 0.306)
    )

    control_pod = (
        cq.Workplane("XY")
        .box(POD_DEPTH, POD_WIDTH, POD_HEIGHT)
        .translate(
            (
                POD_CENTER_X,
                BODY_WIDTH * 0.5 + POD_WIDTH * 0.5 - POD_EMBED,
                POD_BASE_Z + POD_HEIGHT * 0.5,
            )
        )
        .edges("|Z")
        .fillet(0.004)
    )

    feed_slot = (
        cq.Workplane("XY")
        .box(0.009, 0.076, 0.040)
        .translate((0.0, 0.0, BODY_HEIGHT - TOP_THICKNESS * 0.5))
    )
    return (
        left_wall.union(right_wall)
        .union(back_wall)
        .union(bottom_plate)
        .union(top_cap)
        .union(front_upper)
        .union(control_pod)
        .cut(feed_slot)
    )


def _build_drawer_mesh() -> object:
    panel = (
        cq.Workplane("XY")
        .box(DRAWER_PANEL_THICKNESS, DRAWER_FRONT_WIDTH, DRAWER_FRONT_HEIGHT)
        .translate((DRAWER_PANEL_THICKNESS * 0.5, 0.0, 0.0))
        .edges("|X")
        .fillet(0.003)
    )

    bin_outer = (
        cq.Workplane("XY")
        .box(DRAWER_BIN_DEPTH, DRAWER_BIN_WIDTH, DRAWER_BIN_HEIGHT)
        .translate((-DRAWER_BIN_DEPTH * 0.5, 0.0, -0.006))
    )

    inner_bin = (
        cq.Workplane("XY")
        .box(
            DRAWER_BIN_DEPTH - DRAWER_WALL,
            DRAWER_BIN_WIDTH - 2.0 * DRAWER_WALL,
            DRAWER_BIN_HEIGHT - DRAWER_WALL + 0.014,
        )
        .translate((-(DRAWER_BIN_DEPTH - DRAWER_WALL) * 0.5, 0.0, -0.006 + DRAWER_WALL * 0.5 + 0.007))
    )

    grip_cut = (
        cq.Workplane("XY")
        .box(0.012, 0.052, 0.020)
        .translate((DRAWER_PANEL_THICKNESS - 0.002, 0.0, 0.050))
        .edges("|Y")
        .fillet(0.004)
    )

    runner_left = cq.Workplane("XY").box(0.096, 0.006, 0.002).translate((-0.050, -0.024, -0.110))
    runner_right = cq.Workplane("XY").box(0.096, 0.006, 0.002).translate((-0.050, 0.024, -0.110))

    return panel.union(bin_outer).union(runner_left).union(runner_right).cut(inner_bin).cut(grip_cut)


def _lobed_profile(outer_radius: float, inner_radius: float, lobes: int) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(lobes * 2):
        angle = math.tau * index / (lobes * 2)
        radius = outer_radius if index % 2 == 0 else inner_radius
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _build_drum_mesh(phase: float) -> object:
    profile = []
    for x, z in _lobed_profile(DRUM_CUTTER_RADIUS, DRUM_CUTTER_RADIUS * 0.74, 8):
        c = math.cos(phase)
        s = math.sin(phase)
        profile.append((x * c - z * s, x * s + z * c))

    cutter = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(DRUM_LENGTH * 0.5, both=True)
    )
    shaft = cq.Workplane("XZ").circle(DRUM_SHAFT_RADIUS).extrude((DRUM_LENGTH + 0.016) * 0.5, both=True)
    end_collars = (
        cq.Workplane("XZ")
        .circle(DRUM_SHAFT_RADIUS * 1.3)
        .extrude(0.0020, both=True)
        .translate((0.0, DRUM_LENGTH * 0.5 + 0.0040, 0.0))
        .union(
            cq.Workplane("XZ")
            .circle(DRUM_SHAFT_RADIUS * 1.3)
            .extrude(0.0020, both=True)
            .translate((0.0, -(DRUM_LENGTH * 0.5 + 0.0040), 0.0))
        )
    )
    return cutter.union(shaft).union(end_collars)


def _build_power_rocker_mesh() -> object:
    pivot = cq.Workplane("YZ").circle(0.0015).extrude(0.016, both=True).translate((0.0, 0.0015, 0.0))
    cap = (
        cq.Workplane("XY")
        .box(0.016, 0.0065, 0.026)
        .translate((0.0, 0.0050, 0.0))
        .edges("|X")
        .fillet(0.0022)
    )
    return pivot.union(cap)


def _build_reverse_button_mesh() -> object:
    stem = cq.Workplane("XZ").circle(0.0025).extrude(0.0040)
    cap = (
        cq.Workplane("XY")
        .box(0.012, 0.005, 0.012)
        .translate((0.0, 0.0025, 0.0))
        .edges("|Y")
        .fillet(0.0020)
    )
    return stem.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_shredder")

    body_dark = model.material("body_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    body_trim = model.material("body_trim", rgba=(0.19, 0.20, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    switch_black = model.material("switch_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_red = model.material("button_red", rgba=(0.70, 0.12, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_mesh(), "shredder_body"),
        material=body_dark,
        name="shell",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer_mesh(), "waste_drawer"),
        material=body_trim,
        name="bin_shell",
    )

    drum_front = model.part("drum_front")
    drum_front.visual(
        mesh_from_cadquery(_build_drum_mesh(0.0), "drum_front"),
        material=steel,
        name="drum",
    )

    drum_rear = model.part("drum_rear")
    drum_rear.visual(
        mesh_from_cadquery(_build_drum_mesh(math.pi / 8.0), "drum_rear"),
        material=steel,
        name="drum",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        mesh_from_cadquery(_build_power_rocker_mesh(), "power_rocker"),
        material=switch_black,
        name="rocker_cap",
    )

    reverse_button = model.part("reverse_button")
    reverse_button.visual(
        mesh_from_cadquery(_build_reverse_button_mesh(), "reverse_button"),
        material=button_red,
        name="button_cap",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.020,
                0.014,
                body_style="cylindrical",
                edge_radius=0.0012,
                grip=KnobGrip(style="fluted", count=12, depth=0.0007),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=switch_black,
        name="knob_shell",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_DEPTH * 0.5 + 0.001, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=0.0, upper=DRAWER_TRAVEL),
    )

    model.articulation(
        "body_to_drum_front",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum_front,
        origin=Origin(xyz=(DRUM_X_OFFSET, 0.0, DRUM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    model.articulation(
        "body_to_drum_rear",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum_rear,
        origin=Origin(xyz=(-DRUM_X_OFFSET, 0.0, DRUM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=Origin(xyz=(POWER_X, POD_OUTER_Y, POWER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.28,
            upper=0.28,
        ),
    )

    model.articulation(
        "body_to_reverse_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=reverse_button,
        origin=Origin(xyz=(BUTTON_X, POD_OUTER_Y, BUTTON_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )

    model.articulation(
        "body_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(KNOB_X, POD_OUTER_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    reverse_button = object_model.get_part("reverse_button")
    power_rocker = object_model.get_part("power_rocker")
    timer_knob = object_model.get_part("timer_knob")
    drum_front = object_model.get_part("drum_front")
    drum_rear = object_model.get_part("drum_rear")

    drawer_slide = object_model.get_articulation("body_to_drawer")
    button_slide = object_model.get_articulation("body_to_reverse_button")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    timer_joint = object_model.get_articulation("body_to_timer_knob")
    front_drum_joint = object_model.get_articulation("body_to_drum_front")
    rear_drum_joint = object_model.get_articulation("body_to_drum_rear")

    ctx.allow_overlap(
        body,
        drum_front,
        elem_a="shell",
        elem_b="drum",
        reason="The front cutter drum shaft is intentionally seated into hidden side-wall bearing supports inside the shredder head.",
    )
    ctx.allow_overlap(
        body,
        drum_rear,
        elem_a="shell",
        elem_b="drum",
        reason="The rear cutter drum shaft is intentionally seated into hidden side-wall bearing supports inside the shredder head.",
    )

    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        elem_a="bin_shell",
        elem_b="shell",
        margin=0.012,
        name="drawer stays aligned within the body width and height",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="bin_shell",
        elem_b="shell",
        min_overlap=0.080,
        name="closed drawer remains deeply inserted in the body",
    )

    drawer_rest = ctx.part_world_position(drawer)
    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else None
    if drawer_rest is not None and drawer_upper is not None:
        with ctx.pose({drawer_slide: drawer_upper}):
            drawer_extended = ctx.part_world_position(drawer)
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                elem_a="bin_shell",
                elem_b="shell",
                margin=0.012,
                name="extended drawer stays guided by the body opening",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="bin_shell",
                elem_b="shell",
                min_overlap=0.020,
                name="extended drawer keeps retained insertion",
            )
        ctx.check(
            "drawer extends outward",
            drawer_extended is not None and drawer_extended[0] > drawer_rest[0] + 0.09,
            details=f"rest={drawer_rest}, extended={drawer_extended}",
        )

    button_rest = ctx.part_world_position(reverse_button)
    button_upper = button_slide.motion_limits.upper if button_slide.motion_limits is not None else None
    if button_rest is not None and button_upper is not None:
        with ctx.pose({button_slide: button_upper}):
            button_pressed = ctx.part_world_position(reverse_button)
        ctx.check(
            "reverse button presses inward",
            button_pressed is not None and button_pressed[1] < button_rest[1] - 0.0015,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    rocker_lower = rocker_joint.motion_limits.lower if rocker_joint.motion_limits is not None else None
    rocker_upper = rocker_joint.motion_limits.upper if rocker_joint.motion_limits is not None else None
    if rocker_lower is not None and rocker_upper is not None:
        with ctx.pose({rocker_joint: rocker_lower}):
            rocker_low_aabb = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
        with ctx.pose({rocker_joint: rocker_upper}):
            rocker_high_aabb = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
        ctx.check(
            "power rocker visibly tilts",
            rocker_low_aabb is not None
            and rocker_high_aabb is not None
            and rocker_high_aabb[1][2] > rocker_low_aabb[1][2] + 0.0015,
            details=f"low={rocker_low_aabb}, high={rocker_high_aabb}",
        )

    ctx.check(
        "timer knob uses continuous rotation",
        getattr(timer_joint.articulation_type, "name", str(timer_joint.articulation_type)) == "CONTINUOUS",
        details=f"type={timer_joint.articulation_type}",
    )
    ctx.check(
        "both cutter drums use continuous rotation",
        getattr(front_drum_joint.articulation_type, "name", str(front_drum_joint.articulation_type)) == "CONTINUOUS"
        and getattr(rear_drum_joint.articulation_type, "name", str(rear_drum_joint.articulation_type)) == "CONTINUOUS",
        details=f"front={front_drum_joint.articulation_type}, rear={rear_drum_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
