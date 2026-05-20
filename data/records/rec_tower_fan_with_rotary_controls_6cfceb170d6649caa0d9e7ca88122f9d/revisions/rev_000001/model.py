from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
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


BODY_WIDTH = 0.122
BODY_DEPTH = 0.106
BODY_HEIGHT = 0.918
BODY_CORNER_RADIUS = 0.021
BODY_WALL = 0.0045
BODY_BASE_Z = 0.066
VENT_WIDTH = 0.070
VENT_HEIGHT = 0.735
VENT_BOTTOM_Z = 0.086


def _tower_shell_shape():
    outer = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS)
        .edges(">Z")
        .fillet(0.008)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_WIDTH - 2.0 * BODY_WALL,
            BODY_DEPTH - 2.0 * BODY_WALL,
            BODY_HEIGHT - 0.020 - 0.030,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.020))
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS - BODY_WALL)
    )
    front_slot = (
        cq.Workplane("XY")
        .box(VENT_WIDTH, 0.030, VENT_HEIGHT, centered=(True, True, False))
        .translate((0.0, BODY_DEPTH * 0.5 - 0.014, VENT_BOTTOM_Z))
    )
    return outer.cut(inner).cut(front_slot)


def _button_bezel_shape():
    return (
        cq.Workplane("XY")
        .circle(0.0105)
        .circle(0.0086)
        .extrude(0.003)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    base_dark = model.material("base_dark", rgba=(0.13, 0.14, 0.16, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    rotor_dark = model.material("rotor_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    deck_black = model.material("deck_black", rgba=(0.06, 0.07, 0.08, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.160, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=base_dark,
        name="pedestal_disk",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=trim_dark,
        name="pedestal_cap",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=body_dark,
        name="pivot_collar",
    )

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.028, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=body_dark,
        name="pivot_post",
    )
    tower.visual(
        Box((0.058, 0.050, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=body_dark,
        name="lower_mount",
    )
    tower.visual(
        mesh_from_cadquery(_tower_shell_shape(), "tower_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_Z)),
        material=body_dark,
        name="tower_shell",
    )
    tower.visual(
        Box((0.086, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.987)),
        material=deck_black,
        name="control_deck",
    )
    tower.visual(
        mesh_from_cadquery(_button_bezel_shape(), "button_bezel"),
        origin=Origin(xyz=(0.036, 0.0, 0.990)),
        material=deck_black,
        name="button_bezel",
    )
    vent_rib_z = BODY_BASE_Z + VENT_BOTTOM_Z - 0.004 + (VENT_HEIGHT + 0.008) * 0.5
    for index, rib_x in enumerate((-0.024, -0.012, 0.0, 0.012, 0.024)):
        tower.visual(
            Box((0.004, 0.008, VENT_HEIGHT + 0.008)),
            origin=Origin(xyz=(rib_x, BODY_DEPTH * 0.5 - 0.008, vent_rib_z)),
            material=trim_dark,
            name=f"vent_rib_{index}",
        )

    blower = model.part("blower")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.041,
                0.016,
                0.760,
                34,
                blade_thickness=0.0028,
                blade_sweep_deg=18.0,
                center=False,
            ),
            "tower_fan_blower",
        ),
        material=rotor_dark,
        name="blower_wheel",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.022,
            0.015,
            body_style="skirted",
            top_diameter=0.018,
            skirt=KnobSkirt(0.028, 0.004, flare=0.04),
            grip=KnobGrip(style="fluted", count=16, depth=0.0010),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            center=False,
        ),
        "tower_fan_knob",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        knob_mesh,
        material=knob_silver,
        name="speed_knob",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        knob_mesh,
        material=knob_silver,
        name="timer_knob",
    )

    osc_button = model.part("osc_button")
    osc_button.visual(
        Cylinder(radius=0.0098, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, -0.0005)),
        material=trim_dark,
        name="button_stop",
    )
    osc_button.visual(
        Cylinder(radius=0.0064, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.00225)),
        material=trim_dark,
        name="button_stem",
    )
    osc_button.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=trim_dark,
        name="button_cap",
    )

    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=-1.05,
            upper=1.05,
        ),
    )
    model.articulation(
        "tower_to_blower",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=24.0),
    )
    model.articulation(
        "tower_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=speed_knob,
        origin=Origin(xyz=(-0.021, 0.0, 0.990)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.12, velocity=8.0),
    )
    model.articulation(
        "tower_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=timer_knob,
        origin=Origin(xyz=(0.007, 0.0, 0.990)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.12, velocity=8.0),
    )
    model.articulation(
        "tower_to_osc_button",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=osc_button,
        origin=Origin(xyz=(0.036, 0.0, 0.990)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0025,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    blower = object_model.get_part("blower")
    speed_knob = object_model.get_part("speed_knob")
    timer_knob = object_model.get_part("timer_knob")
    osc_button = object_model.get_part("osc_button")
    oscillation = object_model.get_articulation("base_to_tower")
    button_slide = object_model.get_articulation("tower_to_osc_button")

    ctx.allow_isolated_part(
        blower,
        reason="The internal blower wheel is carried on hidden end bearings inside the tower housing.",
    )

    ctx.expect_gap(
        tower,
        base,
        axis="z",
        positive_elem="tower_shell",
        negative_elem="pivot_collar",
        min_gap=0.020,
        max_gap=0.080,
        name="tower housing stays visibly above the oscillation collar",
    )
    ctx.expect_overlap(
        tower,
        base,
        axes="xy",
        elem_a="pivot_post",
        elem_b="pivot_collar",
        min_overlap=0.040,
        name="pivot post remains centered over the collar",
    )
    ctx.expect_within(
        blower,
        tower,
        axes="xy",
        inner_elem="blower_wheel",
        outer_elem="tower_shell",
        margin=0.010,
        name="blower wheel stays within the tower body footprint",
    )
    ctx.expect_gap(
        speed_knob,
        tower,
        axis="z",
        positive_elem="speed_knob",
        negative_elem="control_deck",
        max_gap=0.001,
        max_penetration=0.0,
        name="speed knob is seated on the control deck",
    )
    ctx.expect_gap(
        timer_knob,
        tower,
        axis="z",
        positive_elem="timer_knob",
        negative_elem="control_deck",
        max_gap=0.001,
        max_penetration=0.0,
        name="timer knob is seated on the control deck",
    )
    ctx.expect_within(
        osc_button,
        tower,
        axes="xy",
        inner_elem="button_cap",
        outer_elem="button_bezel",
        margin=0.001,
        name="oscillation button stays centered in its bezel",
    )
    ctx.expect_contact(
        osc_button,
        tower,
        elem_a="button_stop",
        elem_b="button_bezel",
        name="oscillation button is retained by the bezel sleeve",
    )

    rest_knob_pos = ctx.part_world_position(speed_knob)
    with ctx.pose({oscillation: 0.85}):
        swung_knob_pos = ctx.part_world_position(speed_knob)

    ctx.check(
        "tower oscillation swings the control deck around the pedestal axis",
        rest_knob_pos is not None
        and swung_knob_pos is not None
        and (
            abs(swung_knob_pos[0] - rest_knob_pos[0]) > 0.010
            or abs(swung_knob_pos[1] - rest_knob_pos[1]) > 0.010
        ),
        details=f"rest_knob_pos={rest_knob_pos}, swung_knob_pos={swung_knob_pos}",
    )

    rest_button_pos = ctx.part_world_position(osc_button)
    with ctx.pose({button_slide: 0.0025}):
        pressed_button_pos = ctx.part_world_position(osc_button)
    ctx.check(
        "oscillation button depresses downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.0015,
        details=f"rest_button_pos={rest_button_pos}, pressed_button_pos={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
