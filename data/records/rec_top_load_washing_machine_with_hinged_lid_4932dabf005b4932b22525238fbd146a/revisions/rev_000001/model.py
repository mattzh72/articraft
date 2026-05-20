from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


CABINET_WIDTH = 0.68
CABINET_DEPTH = 0.71
BODY_HEIGHT = 0.92
WALL_THICKNESS = 0.018
DECK_THICKNESS = 0.026
TOP_SURFACE_Z = BODY_HEIGHT + DECK_THICKNESS

OPENING_WIDTH = 0.50
OPENING_DEPTH = 0.47
OPENING_CENTER_Y = -0.03
OPENING_CORNER = 0.060

CONSOLE_WIDTH = 0.62
CONSOLE_DEPTH = 0.09
CONSOLE_HEIGHT = 0.15
CONSOLE_FRONT_Y = (CABINET_DEPTH * 0.5) - CONSOLE_DEPTH

LID_WIDTH = 0.56
LID_DEPTH = 0.56
LID_THICKNESS = 0.032
LID_BOTTOM_FROM_HINGE = -0.012
HINGE_AXIS_Z = TOP_SURFACE_Z - LID_BOTTOM_FROM_HINGE
HINGE_AXIS_Y = OPENING_CENTER_Y + (OPENING_DEPTH * 0.5) + 0.020

BASKET_OUTER_RADIUS = 0.215
BASKET_WALL = 0.016
BASKET_DEPTH = 0.44
BASKET_BOTTOM = 0.035
BASKET_TOP_Z = BODY_HEIGHT - 0.006

CYCLE_KNOB_X = -0.070
CONTROL_Z = TOP_SURFACE_Z + 0.078
START_BUTTON_X = 0.085
CANCEL_BUTTON_X = 0.175
BUTTON_TRAVEL = 0.009
BLEACH_CAP_X = 0.175
BLEACH_CAP_Y = OPENING_CENTER_Y + (OPENING_DEPTH * 0.5) + 0.005


def _rounded_prism(width: float, depth: float, height: float, corner: float) -> cq.Workplane:
    return cq.Workplane("XY").box(width, depth, height).edges("|Z").fillet(corner)


def _build_cabinet_shape() -> cq.Workplane:
    left_wall = cq.Workplane("XY").box(
        WALL_THICKNESS,
        CABINET_DEPTH,
        BODY_HEIGHT,
    ).translate((-(CABINET_WIDTH - WALL_THICKNESS) * 0.5, 0.0, BODY_HEIGHT * 0.5))
    right_wall = cq.Workplane("XY").box(
        WALL_THICKNESS,
        CABINET_DEPTH,
        BODY_HEIGHT,
    ).translate(((CABINET_WIDTH - WALL_THICKNESS) * 0.5, 0.0, BODY_HEIGHT * 0.5))
    front_wall = cq.Workplane("XY").box(
        CABINET_WIDTH - (2.0 * WALL_THICKNESS),
        WALL_THICKNESS,
        BODY_HEIGHT,
    ).translate((0.0, -((CABINET_DEPTH - WALL_THICKNESS) * 0.5), BODY_HEIGHT * 0.5))
    rear_wall = cq.Workplane("XY").box(
        CABINET_WIDTH - (2.0 * WALL_THICKNESS),
        WALL_THICKNESS,
        BODY_HEIGHT,
    ).translate((0.0, ((CABINET_DEPTH - WALL_THICKNESS) * 0.5), BODY_HEIGHT * 0.5))
    bottom = cq.Workplane("XY").box(
        CABINET_WIDTH - (2.0 * WALL_THICKNESS),
        CABINET_DEPTH - (2.0 * WALL_THICKNESS),
        WALL_THICKNESS,
    ).translate((0.0, 0.0, WALL_THICKNESS * 0.5))

    toe_strip = cq.Workplane("XY").box(
        CABINET_WIDTH - 0.010,
        0.040,
        0.080,
    ).translate((0.0, -(CABINET_DEPTH * 0.5) + 0.020, 0.040))

    basket_spindle = (
        cq.Workplane("XY")
        .circle(0.045)
        .extrude((BASKET_TOP_Z - BASKET_DEPTH) - WALL_THICKNESS)
        .translate((0.0, OPENING_CENTER_Y, WALL_THICKNESS))
    )

    body = (
        left_wall.union(right_wall)
        .union(front_wall)
        .union(rear_wall)
        .union(bottom)
        .union(toe_strip)
        .union(basket_spindle)
    )

    opening_cut = _rounded_prism(
        OPENING_WIDTH,
        OPENING_DEPTH,
        DECK_THICKNESS + 0.120,
        OPENING_CORNER,
    ).translate((0.0, OPENING_CENTER_Y, BODY_HEIGHT + (DECK_THICKNESS * 0.5)))

    deck = (
        cq.Workplane("XY")
        .box(CABINET_WIDTH, CABINET_DEPTH, DECK_THICKNESS)
        .translate((0.0, 0.0, BODY_HEIGHT + (DECK_THICKNESS * 0.5)))
        .cut(opening_cut)
    )

    collar_outer = _rounded_prism(
        OPENING_WIDTH + 0.048,
        OPENING_DEPTH + 0.050,
        0.080,
        OPENING_CORNER + 0.012,
    ).translate((0.0, OPENING_CENTER_Y, BODY_HEIGHT - 0.018))
    collar_inner = _rounded_prism(
        OPENING_WIDTH - 0.018,
        OPENING_DEPTH - 0.018,
        0.110,
        max(OPENING_CORNER - 0.008, 0.018),
    ).translate((0.0, OPENING_CENTER_Y, BODY_HEIGHT - 0.018))
    collar = collar_outer.cut(collar_inner)

    console = (
        cq.Workplane("XY")
        .box(CONSOLE_WIDTH, CONSOLE_DEPTH, CONSOLE_HEIGHT)
        .edges("|Z")
        .fillet(0.012)
        .translate(
            (
                0.0,
                (CABINET_DEPTH * 0.5) - (CONSOLE_DEPTH * 0.5),
                TOP_SURFACE_Z + (CONSOLE_HEIGHT * 0.5),
            )
        )
    )

    console_crown = (
        cq.Workplane("XY")
        .box(CONSOLE_WIDTH - 0.030, CONSOLE_DEPTH - 0.018, 0.020)
        .edges("|Z")
        .fillet(0.010)
        .translate(
            (
                0.0,
                (CABINET_DEPTH * 0.5) - (CONSOLE_DEPTH * 0.5) + 0.004,
                TOP_SURFACE_Z + CONSOLE_HEIGHT + 0.010,
            )
        )
    )

    return body.union(deck).union(collar).union(console).union(console_crown)


def _build_lid_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(LID_WIDTH, LID_DEPTH, LID_THICKNESS)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, -LID_DEPTH * 0.5, 0.004))
    )

    underside_pocket = cq.Workplane("XY").box(
        LID_WIDTH - 0.090,
        LID_DEPTH - 0.080,
        LID_THICKNESS - 0.010,
    ).translate((0.0, -(LID_DEPTH * 0.5) + 0.008, -0.001))

    hinge_barrel = (
        cq.Workplane("YZ")
        .circle(0.012)
        .extrude(LID_WIDTH - 0.070, both=True)
        .translate((0.0, 0.004, 0.0))
    )

    front_grip = (
        cq.Workplane("XY")
        .box(0.180, 0.018, 0.010)
        .translate((0.0, -LID_DEPTH + 0.010, -0.007))
    )

    return panel.cut(underside_pocket).union(hinge_barrel).union(front_grip)


def _build_basket_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .circle(BASKET_OUTER_RADIUS)
        .extrude(-BASKET_DEPTH)
        .cut(
            cq.Workplane("XY")
            .circle(BASKET_OUTER_RADIUS - BASKET_WALL)
            .extrude(-(BASKET_DEPTH - BASKET_BOTTOM))
        )
    )

    rim = (
        cq.Workplane("XY")
        .circle(BASKET_OUTER_RADIUS + 0.008)
        .circle(BASKET_OUTER_RADIUS - 0.006)
        .extrude(-0.020)
    )

    agitator_column = (
        cq.Workplane("XY")
        .circle(0.046)
        .extrude(-0.380)
        .union(cq.Workplane("XY").circle(0.072).extrude(-0.055))
        .union(
            cq.Workplane("XY")
            .circle(0.088)
            .extrude(-0.095)
            .translate((0.0, 0.0, -0.315))
        )
    )

    agitator_fins = cq.Workplane("XY")
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        fin = (
            cq.Workplane("XY")
            .box(0.018, 0.110, 0.280)
            .translate((0.0, 0.0, -0.150))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        agitator_fins = agitator_fins.union(fin)

    return shell.union(rim).union(agitator_column).union(agitator_fins)


def _build_button_shape(width: float, depth: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.0, -(depth * 0.5), 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_top_load_washer")

    enamel = model.material("enamel_white", rgba=(0.93, 0.94, 0.95, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    steel = model.material("basket_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    agitator_grey = model.material("agitator_grey", rgba=(0.80, 0.82, 0.85, 1.0))
    bleach_blue = model.material("bleach_blue", rgba=(0.71, 0.83, 0.96, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_shape(), "washer_cabinet"),
        material=enamel,
        name="cabinet_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "washer_lid"),
        material=enamel,
        name="lid_shell",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_build_basket_shape(), "washer_basket"),
        material=steel,
        name="basket_shell",
    )

    cycle_knob = model.part("cycle_knob")
    cycle_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.033,
                body_style="skirted",
                top_diameter=0.046,
                skirt=KnobSkirt(0.072, 0.008, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0016),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "washer_cycle_knob",
        ),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_trim,
        name="cycle_knob_shell",
    )

    button_mesh = mesh_from_cadquery(_build_button_shape(0.060, 0.018, 0.028), "washer_button")

    start_button = model.part("start_button")
    start_button.visual(
        button_mesh,
        material=dark_trim,
        name="start_button_cap",
    )

    cancel_button = model.part("cancel_button")
    cancel_button.visual(
        button_mesh,
        material=dark_trim,
        name="cancel_button_cap",
    )

    bleach_cap = model.part("bleach_cap")
    bleach_cap.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.018,
                body_style="domed",
                top_diameter=0.038,
                grip=KnobGrip(style="fluted", count=12, depth=0.0012),
                center=False,
            ),
            "washer_bleach_cap",
        ),
        material=bleach_blue,
        name="bleach_cap_shell",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )

    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, BASKET_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=8.0),
    )

    model.articulation(
        "cycle_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=cycle_knob,
        origin=Origin(xyz=(CYCLE_KNOB_X, CONSOLE_FRONT_Y, CONTROL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=6.0),
    )

    model.articulation(
        "start_button_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_button,
        origin=Origin(xyz=(START_BUTTON_X, CONSOLE_FRONT_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    model.articulation(
        "cancel_button_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=cancel_button,
        origin=Origin(xyz=(CANCEL_BUTTON_X, CONSOLE_FRONT_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    model.articulation(
        "bleach_cap_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=bleach_cap,
        origin=Origin(xyz=(BLEACH_CAP_X, BLEACH_CAP_Y, TOP_SURFACE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    cycle_knob = object_model.get_part("cycle_knob")
    start_button = object_model.get_part("start_button")
    cancel_button = object_model.get_part("cancel_button")
    bleach_cap = object_model.get_part("bleach_cap")
    lid_hinge = object_model.get_articulation("lid_hinge")
    basket_spin = object_model.get_articulation("basket_spin")
    cycle_knob_spin = object_model.get_articulation("cycle_knob_spin")
    start_button_slide = object_model.get_articulation("start_button_slide")
    cancel_button_slide = object_model.get_articulation("cancel_button_slide")
    bleach_cap_spin = object_model.get_articulation("bleach_cap_spin")

    ctx.allow_overlap(
        "cabinet",
        "cycle_knob",
        elem_a="cabinet_shell",
        elem_b="cycle_knob_shell",
        reason="The cycle knob is intentionally modeled as a flush console-mounted control seated against the front panel.",
    )
    ctx.allow_overlap(
        "cabinet",
        "start_button",
        elem_a="cabinet_shell",
        elem_b="start_button_cap",
        reason="The start button cap is intentionally flush-mounted to the console face.",
    )
    ctx.allow_overlap(
        "cabinet",
        "cancel_button",
        elem_a="cabinet_shell",
        elem_b="cancel_button_cap",
        reason="The cancel button cap is intentionally flush-mounted to the console face.",
    )
    ctx.allow_overlap(
        "lid",
        "bleach_cap",
        elem_a="lid_shell",
        elem_b="bleach_cap_shell",
        reason="The bleach cap intentionally nests beneath the closed lid along the rear of the tub opening.",
    )

    ctx.check(
        "lid_joint_is_revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lid_hinge.articulation_type!r}",
    )
    ctx.check(
        "basket_joint_is_continuous",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={basket_spin.articulation_type!r}",
    )
    ctx.check(
        "basket_spin_axis_is_vertical",
        tuple(round(value, 3) for value in basket_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={basket_spin.axis!r}",
    )
    ctx.check(
        "cycle_knob_joint_is_continuous",
        cycle_knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={cycle_knob_spin.articulation_type!r}",
    )
    ctx.check(
        "cycle_knob_axis_faces_forward",
        tuple(round(value, 3) for value in cycle_knob_spin.axis) == (0.0, -1.0, 0.0),
        details=f"axis={cycle_knob_spin.axis!r}",
    )
    ctx.check(
        "bleach_cap_joint_is_continuous",
        bleach_cap_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={bleach_cap_spin.articulation_type!r}",
    )
    ctx.check(
        "start_button_joint_is_prismatic",
        start_button_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={start_button_slide.articulation_type!r}",
    )
    ctx.check(
        "cancel_button_joint_is_prismatic",
        cancel_button_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={cancel_button_slide.articulation_type!r}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    basket_aabb = ctx.part_world_aabb(basket)
    if closed_lid_aabb is not None and basket_aabb is not None:
        ctx.check(
            "basket_sits_below_closed_lid",
            basket_aabb[1][2] < closed_lid_aabb[0][2] - 0.015,
            details=f"basket_max_z={basket_aabb[1][2]:.4f}, lid_min_z={closed_lid_aabb[0][2]:.4f}",
        )

    lid_limits = lid_hinge.motion_limits
    if closed_lid_aabb is not None and lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        if open_lid_aabb is not None:
            ctx.check(
                "lid_opens_upward",
                open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.250,
                details=f"closed={closed_lid_aabb!r}, open={open_lid_aabb!r}",
            )

    start_rest = ctx.part_world_position(start_button)
    cancel_rest = ctx.part_world_position(cancel_button)
    knob_pos = ctx.part_world_position(cycle_knob)
    bleach_pos = ctx.part_world_position(bleach_cap)
    if start_rest is not None and cancel_rest is not None:
        ctx.check(
            "front_buttons_are_separate",
            abs(cancel_rest[0] - start_rest[0]) > 0.070,
            details=f"start={start_rest!r}, cancel={cancel_rest!r}",
        )
    if knob_pos is not None and bleach_pos is not None:
        ctx.check(
            "bleach_cap_sits_behind_knob",
            bleach_pos[1] > knob_pos[1] - 0.08,
            details=f"knob={knob_pos!r}, bleach={bleach_pos!r}",
        )

    with ctx.pose({start_button_slide: BUTTON_TRAVEL, cancel_button_slide: BUTTON_TRAVEL}):
        start_pressed = ctx.part_world_position(start_button)
        cancel_pressed = ctx.part_world_position(cancel_button)
    if start_rest is not None and start_pressed is not None:
        ctx.check(
            "start_button_presses_inward",
            start_pressed[1] > start_rest[1] + 0.006,
            details=f"rest={start_rest!r}, pressed={start_pressed!r}",
        )
    if cancel_rest is not None and cancel_pressed is not None:
        ctx.check(
            "cancel_button_presses_inward",
            cancel_pressed[1] > cancel_rest[1] + 0.006,
            details=f"rest={cancel_rest!r}, pressed={cancel_pressed!r}",
        )

    return ctx.report()


object_model = build_object_model()
