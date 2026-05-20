from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BASE_RADIUS = 0.160
BASE_THICKNESS = 0.018
PEDESTAL_RADIUS = 0.030
PEDESTAL_HEIGHT = 0.050
JOINT_Z = BASE_THICKNESS + PEDESTAL_HEIGHT

BODY_DEPTH = 0.112
BODY_WIDTH = 0.138
BODY_BOTTOM = -0.050
BODY_TOTAL_HEIGHT = 0.930
BODY_TOP = BODY_BOTTOM + BODY_TOTAL_HEIGHT
BODY_WALL = 0.003
BODY_TOP_THICKNESS = 0.006
OUTER_CORNER = 0.022
INNER_CORNER = 0.019

TOP_PAD_DEPTH = 0.086
TOP_PAD_WIDTH = 0.112
TOP_PAD_HEIGHT = 0.014
TOP_PAD_CORNER = 0.016

OUTLET_WIDTH = 0.086
OUTLET_HEIGHT = 0.650
OUTLET_BOTTOM = 0.110
OUTLET_CUT_DEPTH = 0.030
OUTLET_CORNER = 0.012

CONTROL_PLANE_Z = BODY_TOP
POWER_KNOB_X = 0.004
POWER_KNOB_Y = -0.028
TIMER_KNOB_X = 0.004
TIMER_KNOB_Y = 0.028
BUTTON_X = -0.023
BUTTON_Y = 0.000
KNOB_HOLE_RADIUS = 0.0055
BUTTON_HOLE_RADIUS = 0.0045
HOLE_DEPTH = 0.020

IMPELLER_BOTTOM = 0.100
IMPELLER_WIDTH = 0.700
IMPELLER_TOP = IMPELLER_BOTTOM + IMPELLER_WIDTH
UPPER_JOURNAL_TOP = BODY_TOP - BODY_TOP_THICKNESS
UPPER_JOURNAL_LENGTH = UPPER_JOURNAL_TOP - IMPELLER_TOP


def rounded_prism(
    x_size: float,
    y_size: float,
    z_size: float,
    corner_radius: float,
    *,
    z0: float = 0.0,
):
    solid = cq.Workplane("XY").box(
        x_size,
        y_size,
        z_size,
        centered=(True, True, False),
    )
    if corner_radius > 0.0:
        solid = solid.edges("|Z").fillet(corner_radius)
    return solid.translate((0.0, 0.0, z0))


def build_base_shape():
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    base = base.faces(">Z").circle(PEDESTAL_RADIUS).extrude(PEDESTAL_HEIGHT)
    return base


def build_body_shape():
    outer_shell = rounded_prism(
        BODY_DEPTH,
        BODY_WIDTH,
        BODY_TOTAL_HEIGHT,
        OUTER_CORNER,
        z0=BODY_BOTTOM,
    )
    top_pad = rounded_prism(
        TOP_PAD_DEPTH,
        TOP_PAD_WIDTH,
        TOP_PAD_HEIGHT,
        TOP_PAD_CORNER,
        z0=BODY_TOP - TOP_PAD_HEIGHT,
    )
    inner_cavity = rounded_prism(
        BODY_DEPTH - 2.0 * BODY_WALL,
        BODY_WIDTH - 2.0 * BODY_WALL,
        BODY_TOTAL_HEIGHT - BODY_TOP_THICKNESS,
        INNER_CORNER,
        z0=BODY_BOTTOM,
    )
    outlet = (
        cq.Workplane("XY")
        .box(
            OUTLET_CUT_DEPTH,
            OUTLET_WIDTH,
            OUTLET_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|X")
        .fillet(OUTLET_CORNER)
        .translate(
            (
                BODY_DEPTH / 2.0 - OUTLET_CUT_DEPTH / 2.0 + 0.001,
                0.0,
                OUTLET_BOTTOM,
            )
        )
    )

    knob_holes = (
        cq.Workplane("XY")
        .circle(KNOB_HOLE_RADIUS)
        .extrude(HOLE_DEPTH)
        .translate((POWER_KNOB_X, POWER_KNOB_Y, CONTROL_PLANE_Z - HOLE_DEPTH))
        .union(
            cq.Workplane("XY")
            .circle(KNOB_HOLE_RADIUS)
            .extrude(HOLE_DEPTH)
            .translate((TIMER_KNOB_X, TIMER_KNOB_Y, CONTROL_PLANE_Z - HOLE_DEPTH))
        )
        .union(
            cq.Workplane("XY")
            .circle(BUTTON_HOLE_RADIUS)
            .extrude(HOLE_DEPTH)
            .translate((BUTTON_X, BUTTON_Y, CONTROL_PLANE_Z - HOLE_DEPTH))
        )
    )

    return outer_shell.union(top_pad).cut(inner_cavity).cut(outlet).cut(knob_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_tower_fan")

    base_finish = model.material("base_finish", rgba=(0.23, 0.24, 0.26, 1.0))
    shell_finish = model.material("shell_finish", rgba=(0.90, 0.91, 0.92, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.74, 0.76, 0.78, 1.0))
    control_finish = model.material("control_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    impeller_finish = model.material("impeller_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    button_finish = model.material("button_finish", rgba=(0.70, 0.72, 0.74, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(build_base_shape(), "tower_fan_base"),
        material=base_finish,
        name="base_shell",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shape(), "tower_fan_body"),
        material=shell_finish,
        name="tower_shell",
    )

    grille = model.part("grille")
    grille.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (OUTLET_HEIGHT, OUTLET_WIDTH),
                0.004,
                slot_size=(0.608, 0.0048),
                pitch=(1.000, 0.0082),
                frame=0.008,
                corner_radius=0.010,
                center=False,
            ),
            "tower_fan_grille",
        ),
        origin=Origin(
            xyz=(
                BODY_DEPTH / 2.0 - BODY_WALL - 0.004,
                0.0,
                OUTLET_BOTTOM + OUTLET_HEIGHT / 2.0,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=grille_finish,
        name="outlet_grille",
    )

    impeller = model.part("impeller")
    impeller.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.038,
                0.021,
                IMPELLER_WIDTH,
                22,
                blade_thickness=0.0032,
                blade_sweep_deg=27.0,
                center=False,
            ),
            "tower_fan_impeller",
        ),
        origin=Origin(xyz=(0.0, 0.0, IMPELLER_BOTTOM)),
        material=impeller_finish,
        name="impeller_wheel",
    )
    impeller.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, IMPELLER_TOP - 0.005)),
        material=impeller_finish,
        name="top_hub",
    )
    impeller.visual(
        Cylinder(radius=0.0035, length=UPPER_JOURNAL_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, IMPELLER_TOP + UPPER_JOURNAL_LENGTH / 2.0),
        ),
        material=impeller_finish,
        name="upper_journal",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.030,
            0.018,
            body_style="skirted",
            top_diameter=0.026,
            skirt=KnobSkirt(0.038, 0.004, flare=0.05),
            grip=KnobGrip(style="fluted", count=16, depth=0.0009),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            center=False,
        ),
        "tower_fan_control_knob",
    )

    power_knob = model.part("power_knob")
    power_knob.visual(
        knob_mesh,
        material=control_finish,
        name="knob_shell",
    )
    power_knob.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=control_finish,
        name="knob_shaft",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        knob_mesh,
        material=control_finish,
        name="knob_shell",
    )
    timer_knob.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=control_finish,
        name="knob_shaft",
    )

    oscillation_button = model.part("oscillation_button")
    oscillation_button.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=button_finish,
        name="button_cap",
    )
    oscillation_button.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=button_finish,
        name="button_stem",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.8,
            lower=-math.radians(38.0),
            upper=math.radians(38.0),
        ),
    )

    model.articulation(
        "body_to_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
        origin=Origin(),
    )

    model.articulation(
        "body_to_impeller",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=impeller,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=22.0),
    )

    model.articulation(
        "body_to_power_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=power_knob,
        origin=Origin(xyz=(POWER_KNOB_X, POWER_KNOB_Y, CONTROL_PLANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    model.articulation(
        "body_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(TIMER_KNOB_X, TIMER_KNOB_Y, CONTROL_PLANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    model.articulation(
        "body_to_oscillation_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=oscillation_button,
        origin=Origin(xyz=(BUTTON_X, BUTTON_Y, CONTROL_PLANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.040,
            lower=-0.0025,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    grille = object_model.get_part("grille")
    impeller = object_model.get_part("impeller")
    power_knob = object_model.get_part("power_knob")
    timer_knob = object_model.get_part("timer_knob")
    oscillation_button = object_model.get_part("oscillation_button")

    oscillation_joint = object_model.get_articulation("base_to_body")
    impeller_joint = object_model.get_articulation("body_to_impeller")
    power_joint = object_model.get_articulation("body_to_power_knob")
    timer_joint = object_model.get_articulation("body_to_timer_knob")
    button_joint = object_model.get_articulation("body_to_oscillation_button")

    ctx.expect_overlap(
        body,
        base,
        axes="xy",
        min_overlap=0.10,
        name="tower body stays centered over the round base",
    )
    ctx.expect_contact(
        grille,
        body,
        name="front outlet grille is mounted to the tower shell",
    )
    ctx.allow_overlap(
        body,
        impeller,
        elem_a="tower_shell",
        elem_b="upper_journal",
        reason="The hidden upper impeller journal is intentionally simplified as running inside the tower's concealed top bearing pocket.",
    )
    ctx.expect_origin_distance(
        power_knob,
        timer_knob,
        axes="y",
        min_dist=0.045,
        name="power and timer knobs remain distinct side-by-side controls",
    )
    ctx.expect_origin_distance(
        oscillation_button,
        power_knob,
        axes="xy",
        min_dist=0.020,
        name="oscillation button stays separate from the power knob",
    )
    ctx.expect_origin_distance(
        oscillation_button,
        timer_knob,
        axes="xy",
        min_dist=0.020,
        name="oscillation button stays separate from the timer knob",
    )

    ctx.check(
        "joint stack matches tower fan prompt",
        oscillation_joint.articulation_type == ArticulationType.REVOLUTE
        and impeller_joint.articulation_type == ArticulationType.CONTINUOUS
        and power_joint.articulation_type == ArticulationType.CONTINUOUS
        and timer_joint.articulation_type == ArticulationType.CONTINUOUS
        and button_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"oscillation={oscillation_joint.articulation_type}, "
            f"impeller={impeller_joint.articulation_type}, "
            f"power={power_joint.articulation_type}, "
            f"timer={timer_joint.articulation_type}, "
            f"button={button_joint.articulation_type}"
        ),
    )

    rest_button_pos = ctx.part_world_position(oscillation_button)
    pressed_button_pos = None
    rest_knob_pos = ctx.part_world_position(power_knob)
    swung_knob_pos = None

    button_limits = button_joint.motion_limits
    if button_limits is not None and button_limits.lower is not None:
        with ctx.pose({button_joint: button_limits.lower}):
            pressed_button_pos = ctx.part_world_position(oscillation_button)
    ctx.check(
        "oscillation button depresses downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.0015,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    swing_limits = oscillation_joint.motion_limits
    if swing_limits is not None and swing_limits.upper is not None:
        with ctx.pose({oscillation_joint: swing_limits.upper}):
            swung_knob_pos = ctx.part_world_position(power_knob)
    ctx.check(
        "tower body oscillation sweeps the top controls sideways",
        rest_knob_pos is not None
        and swung_knob_pos is not None
        and math.hypot(
            swung_knob_pos[0] - rest_knob_pos[0],
            swung_knob_pos[1] - rest_knob_pos[1],
        )
        > 0.015,
        details=f"rest={rest_knob_pos}, swung={swung_knob_pos}",
    )

    return ctx.report()


object_model = build_object_model()
