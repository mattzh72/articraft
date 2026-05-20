from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_LENGTH = 0.205
BASE_WIDTH = 0.168
BASE_HEIGHT = 0.070
SEAT_HEIGHT = 0.012
SEAT_RADIUS = 0.072
SEAT_INNER_RADIUS = 0.049

BOWL_SEAT_Z = BASE_HEIGHT + SEAT_HEIGHT
BOWL_RADIUS = 0.079
LID_Z = 0.092
LID_THICKNESS = 0.004

TUBE_X = -0.004
TUBE_Y = 0.024
TUBE_OUTER_RADIUS = 0.027
TUBE_INNER_RADIUS = 0.023
TUBE_HEIGHT = 0.054
TUBE_TOP_Z = LID_Z + TUBE_HEIGHT

PUSHER_TRAVEL = 0.038
PUSHER_SHAFT_RADIUS = 0.022
PUSHER_SHAFT_LENGTH = 0.050

SPINDLE_RADIUS = 0.0045
SPINDLE_LENGTH = 0.044
SPINDLE_CENTER_Z = BOWL_SEAT_Z + SPINDLE_LENGTH / 2.0 - 0.001
SPINDLE_TOP_Z = BOWL_SEAT_Z + SPINDLE_LENGTH

BLADE_AXIS_Z = 0.114


def _make_base_shell() -> cq.Workplane:
    main_body = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
        .edges(">Z")
        .fillet(0.010)
    )

    front_fascia = (
        cq.Workplane("XY")
        .box(0.120, 0.090, 0.024, centered=(True, True, False))
        .translate((0.030, 0.0, 0.046))
        .edges("|Z")
        .fillet(0.008)
        .edges(">Z")
        .fillet(0.006)
    )

    seat_ring = (
        cq.Workplane("XY")
        .circle(SEAT_RADIUS)
        .extrude(SEAT_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
        .cut(
            cq.Workplane("XY")
            .circle(SEAT_INNER_RADIUS)
            .extrude(SEAT_HEIGHT + 0.002)
            .translate((0.0, 0.0, BASE_HEIGHT - 0.001))
        )
    )

    dial_boss = (
        cq.Workplane("XY")
        .circle(0.016)
        .extrude(0.004)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((BASE_LENGTH / 2.0, 0.028, 0.031))
    )

    paddle_boss = (
        cq.Workplane("XY")
        .circle(0.0065)
        .extrude(0.024)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((BASE_LENGTH / 2.0, -0.018, 0.034))
    )

    return main_body.union(front_fascia).union(seat_ring).union(dial_boss).union(paddle_boss)


def _make_bowl_body() -> cq.Workplane:
    foot_ring = cq.Workplane("XY").circle(0.076).extrude(0.010)
    outer_wall = cq.Workplane("XY").circle(0.073).extrude(0.090)
    top_lip = cq.Workplane("XY").circle(0.079).extrude(0.006).translate((0.0, 0.0, 0.088))
    bowl_shell = foot_ring.union(outer_wall).union(top_lip).cut(
        cq.Workplane("XY").circle(0.069).extrude(0.080).translate((0.0, 0.0, 0.014))
    )

    lid_plate = (
        cq.Workplane("XY")
        .circle(BOWL_RADIUS)
        .extrude(LID_THICKNESS)
        .translate((0.0, 0.0, LID_Z))
        .cut(
            cq.Workplane("XY")
            .circle(TUBE_INNER_RADIUS + 0.001)
            .extrude(LID_THICKNESS + 0.004)
            .translate((TUBE_X, TUBE_Y, LID_Z - 0.002))
        )
    )

    center_hole = (
        cq.Workplane("XY")
        .circle(0.0068)
        .extrude(0.018)
    )

    center_sleeve = (
        cq.Workplane("XY")
        .circle(0.014)
        .circle(0.0068)
        .extrude(0.016)
    )

    bowl_shell = bowl_shell.cut(center_hole).union(center_sleeve).union(lid_plate)

    lug = (
        cq.Workplane("XY")
        .box(0.018, 0.008, 0.006, centered=(True, True, False))
        .translate((0.068, 0.0, 0.002))
    )
    for angle_deg in (20.0, 140.0, 260.0):
        bowl_shell = bowl_shell.union(
            lug.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )

    return bowl_shell


def _make_feed_tube() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(TUBE_OUTER_RADIUS)
        .extrude(TUBE_HEIGHT + 0.002)
        .translate((TUBE_X, TUBE_Y, LID_Z))
        .cut(
            cq.Workplane("XY")
            .circle(TUBE_INNER_RADIUS)
            .extrude(TUBE_HEIGHT + 0.006)
            .translate((TUBE_X, TUBE_Y, LID_Z - 0.002))
        )
    )


def _make_bowl_handle() -> cq.Workplane:
    lower_bridge = (
        cq.Workplane("XY")
        .box(0.012, 0.024, 0.010, centered=(True, True, False))
        .translate((0.0, -0.080, 0.026))
    )
    upper_bridge = (
        cq.Workplane("XY")
        .box(0.012, 0.024, 0.010, centered=(True, True, False))
        .translate((0.0, -0.080, 0.070))
    )
    grip = (
        cq.Workplane("XY")
        .box(0.012, 0.018, 0.046, centered=(True, True, False))
        .translate((0.0, -0.096, 0.026))
    )
    return lower_bridge.union(upper_bridge).union(grip)


def _make_blade() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(0.011)
        .circle(0.0056)
        .extrude(0.014)
        .translate((0.0, 0.0, -0.007))
    )

    lower_blade = (
        cq.Workplane("XY")
        .box(0.094, 0.012, 0.003, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 10.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 20.0)
        .translate((0.0, 0.0, -0.003))
    )

    upper_blade = (
        cq.Workplane("XY")
        .box(0.085, 0.010, 0.003, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -8.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 110.0)
        .translate((0.0, 0.0, 0.004))
    )

    bore = (
        cq.Workplane("XY")
        .circle(0.0054)
        .extrude(0.032)
        .translate((0.0, 0.0, -0.016))
    )

    return hub.union(lower_blade).union(upper_blade).cut(bore)


def _make_pulse_paddle() -> cq.Workplane:
    pivot_barrel = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(0.022)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )
    paddle_plate = (
        cq.Workplane("XY")
        .box(0.006, 0.026, 0.030, centered=(True, True, False))
        .translate((0.003, 0.0, -0.030))
    )
    finger_pad = (
        cq.Workplane("XY")
        .box(0.010, 0.022, 0.010, centered=(True, True, False))
        .translate((0.005, 0.0, -0.040))
    )
    return pivot_barrel.union(paddle_plate).union(finger_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_food_processor")

    body_white = model.material("body_white", rgba=(0.91, 0.92, 0.90, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.65, 0.67, 0.68, 1.0))
    clear_bowl = model.material("clear_bowl", rgba=(0.80, 0.90, 1.00, 0.35))
    clear_pusher = model.material("clear_pusher", rgba=(0.92, 0.95, 0.98, 0.80))
    control_black = model.material("control_black", rgba=(0.12, 0.12, 0.13, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.83, 0.84, 0.86, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shell(), "food_processor_base_shell"),
        material=body_white,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.0064, length=0.041),
        origin=Origin(xyz=(0.0, 0.0, 0.0855)),
        material=trim_gray,
        name="drive_hub",
    )

    base.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_CENTER_Z)),
        material=trim_gray,
        name="spindle_shaft",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=trim_gray,
        name="spindle_cap",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=(0.107498, 0.028, 0.031), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=trim_gray,
        name="dial_pedestal",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_make_bowl_body(), "food_processor_bowl_body"),
        material=clear_bowl,
        name="bowl_body",
    )
    bowl.visual(
        mesh_from_cadquery(_make_feed_tube(), "food_processor_feed_tube"),
        material=clear_bowl,
        name="feed_tube",
    )
    bowl.visual(
        mesh_from_cadquery(_make_bowl_handle(), "food_processor_bowl_handle"),
        material=clear_bowl,
        name="bowl_handle",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=PUSHER_SHAFT_RADIUS, length=PUSHER_SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -PUSHER_SHAFT_LENGTH / 2.0)),
        material=clear_pusher,
        name="pusher_shaft",
    )
    pusher.visual(
        Cylinder(radius=0.0285, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=clear_pusher,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.0105, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=clear_pusher,
        name="pusher_knob",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_make_blade(), "food_processor_blade"),
        material=blade_steel,
        name="blade_assembly",
    )

    pulse_paddle = model.part("pulse_paddle")
    pulse_paddle.visual(
        mesh_from_cadquery(_make_pulse_paddle(), "food_processor_pulse_paddle"),
        material=control_black,
        name="paddle_body",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.024,
                0.012,
                body_style="skirted",
                top_diameter=0.018,
                skirt=KnobSkirt(0.028, 0.003, flare=0.05),
                grip=KnobGrip(style="fluted", count=12, depth=0.0006),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
                center=False,
            ),
            "food_processor_speed_dial",
        ),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=control_black,
        name="dial_knob",
    )
    speed_dial.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=trim_gray,
        name="dial_shaft",
    )
    speed_dial.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=Origin(xyz=(0.0015, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=trim_gray,
        name="dial_mount",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, BOWL_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5),
    )
    model.articulation(
        "bowl_to_pusher",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=pusher,
        origin=Origin(xyz=(TUBE_X, TUBE_Y, TUBE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=PUSHER_TRAVEL,
        ),
    )
    model.articulation(
        "base_to_blade",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, BLADE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=30.0),
    )
    model.articulation(
        "base_to_pulse_paddle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pulse_paddle,
        origin=Origin(xyz=(0.113998, -0.018, 0.034)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=0.35),
    )
    model.articulation(
        "base_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(0.108998, 0.028, 0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    pusher = object_model.get_part("pusher")
    blade = object_model.get_part("blade")
    pulse_paddle = object_model.get_part("pulse_paddle")
    speed_dial = object_model.get_part("speed_dial")
    bowl_joint = object_model.get_articulation("base_to_bowl")
    pusher_joint = object_model.get_articulation("bowl_to_pusher")
    blade_joint = object_model.get_articulation("base_to_blade")
    pulse_joint = object_model.get_articulation("base_to_pulse_paddle")
    dial_joint = object_model.get_articulation("base_to_speed_dial")

    ctx.check(
        "bowl joint is continuous",
        bowl_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={bowl_joint.articulation_type!r}",
    )
    ctx.check(
        "pusher joint is prismatic",
        pusher_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={pusher_joint.articulation_type!r}",
    )
    ctx.check(
        "blade joint is continuous",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type!r}",
    )
    ctx.check(
        "pulse paddle joint is revolute",
        pulse_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pulse_joint.articulation_type!r}",
    )
    ctx.check(
        "speed dial joint is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type!r}",
    )

    with ctx.pose({bowl_joint: 0.0}):
        ctx.expect_gap(
            bowl,
            base,
            axis="z",
            positive_elem="bowl_body",
            negative_elem="base_shell",
            max_gap=0.002,
            max_penetration=0.0,
            name="bowl seats on the locking collar",
        )
        ctx.expect_overlap(
            bowl,
            base,
            axes="xy",
            elem_a="bowl_body",
            elem_b="base_shell",
            min_overlap=0.12,
            name="bowl footprint stays over the base",
        )

    with ctx.pose({bowl_joint: 0.30}):
        ctx.expect_gap(
            bowl,
            base,
            axis="z",
            positive_elem="bowl_body",
            negative_elem="base_shell",
            max_gap=0.002,
            max_penetration=0.0,
            name="twisted bowl stays seated on the base",
        )

    ctx.expect_origin_distance(
        blade,
        bowl,
        axes="xy",
        max_dist=0.002,
        name="blade stays centered in the bowl",
    )
    ctx.expect_contact(
        blade,
        base,
        elem_a="blade_assembly",
        elem_b="spindle_cap",
        contact_tol=1e-6,
        name="blade hub seats on the spindle cap",
    )

    pusher_limits = pusher_joint.motion_limits
    if pusher_limits is not None and pusher_limits.upper is not None:
        with ctx.pose({pusher_joint: 0.0}):
            ctx.expect_within(
                pusher,
                bowl,
                axes="xy",
                inner_elem="pusher_shaft",
                outer_elem="feed_tube",
                margin=0.0012,
                name="pusher shaft fits the feed tube at rest",
            )
            ctx.expect_overlap(
                pusher,
                bowl,
                axes="z",
                elem_a="pusher_shaft",
                elem_b="feed_tube",
                min_overlap=0.045,
                name="collapsed pusher remains inserted in the feed tube",
            )
            pusher_rest = ctx.part_world_position(pusher)

        with ctx.pose({pusher_joint: pusher_limits.upper}):
            ctx.expect_within(
                pusher,
                bowl,
                axes="xy",
                inner_elem="pusher_shaft",
                outer_elem="feed_tube",
                margin=0.0012,
                name="extended pusher stays centered in the feed tube",
            )
            ctx.expect_overlap(
                pusher,
                bowl,
                axes="z",
                elem_a="pusher_shaft",
                elem_b="feed_tube",
                min_overlap=0.012,
                name="extended pusher retains insertion in the feed tube",
            )
            pusher_extended = ctx.part_world_position(pusher)

        ctx.check(
            "pusher extends upward",
            pusher_rest is not None
            and pusher_extended is not None
            and pusher_extended[2] > pusher_rest[2] + 0.02,
            details=f"rest={pusher_rest}, extended={pusher_extended}",
        )

    ctx.expect_origin_gap(
        speed_dial,
        pulse_paddle,
        axis="y",
        min_gap=0.03,
        max_gap=0.06,
        name="speed dial sits beside the pulse paddle",
    )

    ctx.expect_gap(
        speed_dial,
        base,
        axis="x",
        positive_elem="dial_knob",
        negative_elem="base_shell",
        max_gap=0.020,
        max_penetration=0.0,
        name="speed dial mounts proud of the front fascia",
    )

    return ctx.report()


object_model = build_object_model()
