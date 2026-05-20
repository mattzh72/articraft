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


BASE_WIDTH = 0.340
BASE_DEPTH = 0.220
BASE_HEIGHT = 0.044
PEDESTAL_RADIUS = 0.040
PEDESTAL_HEIGHT = 0.018

TOWER_WIDTH = 0.180
TOWER_DEPTH = 0.122
TOWER_HEIGHT = 0.860
TOWER_WALL = 0.004
TOWER_FLOOR = 0.012
TOWER_CORNER = 0.028
FRONT_SLOT_WIDTH = 0.086
FRONT_SLOT_HEIGHT = 0.710
FRONT_SLOT_BOTTOM = 0.080

DECK_WIDTH = 0.144
DECK_DEPTH = 0.086
DECK_PLUG_WIDTH = 0.158
DECK_PLUG_DEPTH = 0.102
DECK_PLUG_HEIGHT = 0.004
DECK_TOP_THICKNESS = 0.006

TIMER_X = -0.036
SPEED_X = 0.006
BUTTON_X = 0.048
TIMER_BOSS_HEIGHT = 0.0018
SPEED_BOSS_HEIGHT = 0.0015
BUTTON_BOSS_HEIGHT = 0.0025

BLOWER_RADIUS = 0.044
BLOWER_WIDTH = 0.730
BLOWER_CENTER_Z = 0.430
BLOWER_SPINDLE_LENGTH = 0.820


def _build_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").ellipse(BASE_WIDTH * 0.5, BASE_DEPTH * 0.5).extrude(BASE_HEIGHT)
    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    return base.union(pedestal)


def _build_tower_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(TOWER_WIDTH, TOWER_DEPTH, TOWER_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(TOWER_CORNER)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            TOWER_WIDTH - 2.0 * TOWER_WALL,
            TOWER_DEPTH - 2.0 * TOWER_WALL,
            TOWER_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(TOWER_CORNER - TOWER_WALL, 0.006))
        .translate((0.0, 0.0, TOWER_FLOOR))
    )
    slot = cq.Workplane("XY").box(
        FRONT_SLOT_WIDTH,
        TOWER_DEPTH,
        FRONT_SLOT_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, FRONT_SLOT_BOTTOM))
    return outer.cut(inner).cut(slot)


def _build_control_deck_shape() -> cq.Workplane:
    deck_plug = cq.Workplane("XY").box(
        DECK_PLUG_WIDTH,
        DECK_PLUG_DEPTH,
        DECK_PLUG_HEIGHT,
        centered=(True, True, False),
    )
    deck_plate = (
        cq.Workplane("XY")
        .box(
            DECK_WIDTH,
            DECK_DEPTH,
            DECK_TOP_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, DECK_PLUG_HEIGHT))
        .edges("|Z")
        .fillet(0.012)
    )
    deck = deck_plug.union(deck_plate)
    for x_pos, boss_radius, boss_height in (
        (TIMER_X, 0.022, TIMER_BOSS_HEIGHT),
        (SPEED_X, 0.018, SPEED_BOSS_HEIGHT),
        (BUTTON_X, 0.012, BUTTON_BOSS_HEIGHT),
    ):
        deck = deck.union(
            cq.Workplane("XY")
            .circle(boss_radius)
            .extrude(boss_height)
            .translate((x_pos, 0.0, DECK_PLUG_HEIGHT + DECK_TOP_THICKNESS))
        )
    for x_pos, hole_radius in (
        (TIMER_X, 0.0045),
        (SPEED_X, 0.0040),
        (BUTTON_X, 0.0060),
    ):
        deck = deck.cut(
            cq.Workplane("XY")
            .circle(hole_radius)
            .extrude(DECK_PLUG_HEIGHT + DECK_TOP_THICKNESS + 0.010)
            .translate((x_pos, 0.0, -0.001))
        )
    return deck


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedroom_tower_fan")

    base_plastic = model.material("base_plastic", rgba=(0.86, 0.87, 0.88, 1.0))
    tower_plastic = model.material("tower_plastic", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.69, 0.71, 0.74, 1.0))
    deck_graphite = model.material("deck_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.12, 0.13, 0.14, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))
    control_grey = model.material("control_grey", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "tower_fan_base"),
        material=base_plastic,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_RADIUS * 1.08, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PEDESTAL_HEIGHT - 0.0015)),
        material=trim_grey,
        name="turntable_trim",
    )

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_build_tower_shell_shape(), "tower_fan_shell"),
        material=tower_plastic,
        name="shell",
    )
    tower.visual(
        Box((0.090, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.044, FRONT_SLOT_BOTTOM + FRONT_SLOT_HEIGHT + 0.010)),
        material=trim_grey,
        name="front_badge",
    )

    control_deck = model.part("control_deck")
    control_deck.visual(
        mesh_from_cadquery(_build_control_deck_shape(), "tower_fan_control_deck"),
        material=deck_graphite,
        name="deck_shell",
    )
    control_deck.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=trim_grey,
        name="motor_pod",
    )
    control_deck.visual(
        Cylinder(radius=0.010, length=0.0025),
        origin=Origin(xyz=(BUTTON_X, 0.0, DECK_PLUG_HEIGHT + DECK_TOP_THICKNESS + 0.00125)),
        material=control_grey,
        name="button_bezel",
    )

    blower = model.part("blower")
    blower.visual(
        Cylinder(radius=BLOWER_RADIUS, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, BLOWER_WIDTH * 0.5 - 0.008)),
        material=rotor_black,
        name="upper_ring",
    )
    blower.visual(
        Cylinder(radius=BLOWER_RADIUS, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -BLOWER_WIDTH * 0.5 + 0.008)),
        material=rotor_black,
        name="lower_ring",
    )
    blower.visual(
        Cylinder(radius=0.0038, length=BLOWER_SPINDLE_LENGTH),
        material=trim_grey,
        name="spindle",
    )
    blower.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, BLOWER_WIDTH * 0.5 + 0.008)),
        material=trim_grey,
        name="upper_hub",
    )
    blower.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -BLOWER_WIDTH * 0.5 - 0.008)),
        material=trim_grey,
        name="lower_hub",
    )
    for index in range(18):
        angle = (index * 360.0) / 18.0
        blower.visual(
            Box((0.016, 0.004, BLOWER_WIDTH - 0.032)),
            origin=Origin(
                xyz=(
                    (BLOWER_RADIUS - 0.008) * math.cos(math.radians(angle)),
                    (BLOWER_RADIUS - 0.008) * math.sin(math.radians(angle)),
                    0.0,
                ),
                rpy=(0.0, 0.0, math.radians(angle)),
            ),
            material=rotor_black,
            name=f"blade_{index:02d}",
        )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.017, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=control_black,
        name="timer_cap",
    )
    timer_dial.visual(
        Cylinder(radius=0.0145, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=control_black,
        name="timer_crown",
    )
    timer_dial.visual(
        Cylinder(radius=0.011, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0003)),
        material=control_black,
        name="timer_skirt",
    )
    timer_dial.visual(
        Box((0.003, 0.015, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0186)),
        material=control_grey,
        name="timer_indicator",
    )
    timer_dial.visual(
        Cylinder(radius=0.0032, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, -0.0055)),
        material=trim_grey,
        name="timer_shaft",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.014, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=control_black,
        name="speed_cap",
    )
    speed_dial.visual(
        Cylinder(radius=0.0115, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=control_black,
        name="speed_crown",
    )
    speed_dial.visual(
        Cylinder(radius=0.009, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0003)),
        material=control_black,
        name="speed_skirt",
    )
    speed_dial.visual(
        Box((0.0025, 0.011, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=control_grey,
        name="speed_indicator",
    )
    speed_dial.visual(
        Cylinder(radius=0.0030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=trim_grey,
        name="speed_shaft",
    )

    oscillation_button = model.part("oscillation_button")
    oscillation_button.visual(
        Cylinder(radius=0.0070, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=control_grey,
        name="button_cap",
    )
    oscillation_button.visual(
        Cylinder(radius=0.0052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=control_grey,
        name="button_stem",
    )

    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.90,
            upper=0.90,
        ),
    )
    model.articulation(
        "tower_to_control_deck",
        ArticulationType.FIXED,
        parent=tower,
        child=control_deck,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT - DECK_PLUG_HEIGHT)),
    )
    model.articulation(
        "tower_to_blower",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, BLOWER_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=24.0,
        ),
    )
    model.articulation(
        "control_deck_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=control_deck,
        child=timer_dial,
        origin=Origin(
            xyz=(
                TIMER_X,
                0.0,
                DECK_PLUG_HEIGHT + DECK_TOP_THICKNESS + TIMER_BOSS_HEIGHT + 0.0012,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=8.0,
        ),
    )
    model.articulation(
        "control_deck_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=control_deck,
        child=speed_dial,
        origin=Origin(
            xyz=(
                SPEED_X,
                0.0,
                DECK_PLUG_HEIGHT + DECK_TOP_THICKNESS + SPEED_BOSS_HEIGHT + 0.0012,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=8.0,
        ),
    )
    model.articulation(
        "control_deck_to_oscillation_button",
        ArticulationType.PRISMATIC,
        parent=control_deck,
        child=oscillation_button,
        origin=Origin(
            xyz=(
                BUTTON_X,
                0.0,
                DECK_PLUG_HEIGHT + DECK_TOP_THICKNESS + BUTTON_BOSS_HEIGHT,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.06,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    control_deck = object_model.get_part("control_deck")
    blower = object_model.get_part("blower")
    timer_dial = object_model.get_part("timer_dial")
    speed_dial = object_model.get_part("speed_dial")
    oscillation_button = object_model.get_part("oscillation_button")
    oscillation = object_model.get_articulation("base_to_tower")
    blower_spin = object_model.get_articulation("tower_to_blower")
    timer_spin = object_model.get_articulation("control_deck_to_timer_dial")
    speed_spin = object_model.get_articulation("control_deck_to_speed_dial")
    button_slide = object_model.get_articulation("control_deck_to_oscillation_button")

    ctx.expect_gap(
        tower,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="tower seats on the base pedestal",
    )
    ctx.expect_overlap(
        tower,
        base,
        axes="xy",
        min_overlap=0.070,
        name="tower stands over the oval base footprint",
    )
    ctx.expect_within(
        control_deck,
        tower,
        axes="xy",
        margin=0.020,
        name="control deck stays within the tower shell footprint",
    )
    ctx.expect_within(
        blower,
        tower,
        axes="xy",
        margin=0.010,
        name="blower wheel stays inside the tower body",
    )
    ctx.expect_overlap(
        blower,
        tower,
        axes="z",
        min_overlap=0.700,
        name="blower spans the tower opening height",
    )
    ctx.expect_gap(
        timer_dial,
        control_deck,
        axis="z",
        positive_elem="timer_skirt",
        negative_elem="deck_shell",
        max_gap=0.001,
        max_penetration=0.0012,
        name="timer dial skirt seats on the control deck boss",
    )
    ctx.expect_gap(
        speed_dial,
        control_deck,
        axis="z",
        positive_elem="speed_skirt",
        negative_elem="deck_shell",
        max_gap=0.001,
        max_penetration=0.0012,
        name="speed dial skirt seats on the control deck boss",
    )
    ctx.expect_gap(
        oscillation_button,
        control_deck,
        axis="z",
        positive_elem="button_cap",
        negative_elem="button_bezel",
        min_gap=0.002,
        max_gap=0.004,
        name="oscillation button sits proud above the bezel at rest",
    )

    ctx.check(
        "continuous articulations configured for blower and dials",
        all(
            joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in (blower_spin, timer_spin, speed_spin)
        ),
        details=(
            f"blower={blower_spin.motion_limits}, "
            f"timer={timer_spin.motion_limits}, "
            f"speed={speed_spin.motion_limits}"
        ),
    )

    limits = oscillation.motion_limits
    if limits is not None and limits.upper is not None:
        rest_timer_pos = ctx.part_world_position(timer_dial)
        with ctx.pose({oscillation: limits.upper}):
            ctx.expect_overlap(
                tower,
                base,
                axes="xy",
                min_overlap=0.070,
                name="tower remains carried by the base at full oscillation",
            )
            swung_timer_pos = ctx.part_world_position(timer_dial)
        ctx.check(
            "oscillation rotates the tower controls off center",
            rest_timer_pos is not None
            and swung_timer_pos is not None
            and abs(swung_timer_pos[1] - rest_timer_pos[1]) > 0.020,
            details=f"rest={rest_timer_pos}, swung={swung_timer_pos}",
        )

    button_limits = button_slide.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        rest_button_pos = ctx.part_world_position(oscillation_button)
        with ctx.pose({button_slide: button_limits.upper}):
            ctx.expect_gap(
                oscillation_button,
                control_deck,
                axis="z",
                positive_elem="button_cap",
                negative_elem="button_bezel",
                max_gap=0.0015,
                max_penetration=0.0,
                name="oscillation button can press down nearly flush",
            )
            pressed_button_pos = ctx.part_world_position(oscillation_button)
        ctx.check(
            "oscillation button moves downward when pressed",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[2] < rest_button_pos[2] - 0.002,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    return ctx.report()


object_model = build_object_model()
