from __future__ import annotations

import math

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
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


OSCILLATION_TOP_Z = 0.075
BODY_TOP_Z = 1.114
SPEED_DIAL_X = -0.060
TIMER_DIAL_X = 0.025
OSC_BUTTON_X = 0.088
CONTROL_Y = -0.020


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _base_geometry():
    disk = cq.Workplane("XY").cylinder(0.070, 0.205).translate((0.0, 0.0, 0.035))
    turntable = cq.Workplane("XY").cylinder(0.020, 0.098).translate((0.0, 0.0, 0.065))
    low_plinth = cq.Workplane("XY").cylinder(0.006, 0.165).translate((0.0, 0.0, 0.072))
    return disk.union(turntable).union(low_plinth)


def _housing_geometry():
    left_rail = _cq_box((0.038, 0.178, 1.080), (-0.106, -0.005, 0.550))
    right_rail = _cq_box((0.038, 0.178, 1.080), (0.106, -0.005, 0.550))
    rear_spine = _cq_box((0.230, 0.032, 0.980), (0.0, 0.073, 0.540))
    bottom_cap = _cq_box((0.250, 0.185, 0.110), (0.0, -0.002, 0.055))
    top_cap = _cq_box((0.250, 0.185, 0.110), (0.0, -0.002, 1.055))
    return (
        left_rail.union(right_rail)
        .union(rear_spine)
        .union(bottom_cap)
        .union(top_cap)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    warm_white = model.material("warm_white_plastic", rgba=(0.88, 0.86, 0.80, 1.0))
    satin_black = model.material("satin_black_plastic", rgba=(0.025, 0.026, 0.028, 1.0))
    dark_grey = model.material("weighted_dark_base", rgba=(0.11, 0.115, 0.12, 1.0))
    grille_grey = model.material("shadow_grille_grey", rgba=(0.18, 0.19, 0.20, 1.0))
    indicator_white = model.material("white_control_marks", rgba=(0.95, 0.94, 0.88, 1.0))
    soft_button = model.material("soft_grey_button", rgba=(0.55, 0.57, 0.58, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_geometry(), "weighted_base", tolerance=0.0015),
        material=dark_grey,
        name="weighted_base",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_housing_geometry(), "tower_housing", tolerance=0.0015),
        material=warm_white,
        name="housing_frame",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.178, 0.880),
                frame=0.014,
                face_thickness=0.004,
                duct_depth=0.012,
                slat_pitch=0.018,
                slat_width=0.007,
                slat_angle_deg=16.0,
                corner_radius=0.009,
                slats=VentGrilleSlats(
                    profile="airfoil",
                    direction="down",
                    inset=0.0015,
                    divider_count=2,
                    divider_width=0.004,
                ),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.0012),
                sleeve=VentGrilleSleeve(style="short", depth=0.010, wall=0.002),
            ),
            "front_vent_grille",
        ),
        origin=Origin(xyz=(0.0, -0.090, 0.550), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_grey,
        name="front_grille",
    )
    body.visual(
        Box((0.215, 0.105, 0.004)),
        origin=Origin(xyz=(0.0, CONTROL_Y, 1.112)),
        material=satin_black,
        name="control_panel",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.003),
        origin=Origin(xyz=(OSC_BUTTON_X, CONTROL_Y, BODY_TOP_Z + 0.0015)),
        material=satin_black,
        name="button_socket",
    )
    body.visual(
        Box((0.196, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=satin_black,
        name="lower_bearing_arm",
    )
    body.visual(
        Box((0.196, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
        material=satin_black,
        name="upper_bearing_arm",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=satin_black,
        name="lower_bearing",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
        material=satin_black,
        name="upper_bearing",
    )

    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.055,
                0.025,
                0.798,
                26,
                blade_thickness=0.0025,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "blower_wheel",
        ),
        material=satin_black,
        name="squirrel_cage",
    )
    blower_wheel.visual(
        Cylinder(radius=0.026, length=0.798),
        material=satin_black,
        name="center_core",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.024,
                body_style="skirted",
                top_diameter=0.042,
                skirt=KnobSkirt(0.058, 0.005, flare=0.05, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
                center=False,
            ),
            "speed_dial",
        ),
        material=satin_black,
        name="speed_knob",
    )
    speed_dial.visual(
        Box((0.004, 0.030, 0.002)),
        origin=Origin(xyz=(0.0, 0.010, 0.025)),
        material=indicator_white,
        name="speed_pointer",
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.023,
                body_style="skirted",
                top_diameter=0.038,
                skirt=KnobSkirt(0.054, 0.005, flare=0.04, chamfer=0.001),
                grip=KnobGrip(style="ribbed", count=14, depth=0.001),
                indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_dial",
        ),
        material=satin_black,
        name="timer_knob",
    )
    timer_dial.visual(
        Box((0.004, 0.027, 0.002)),
        origin=Origin(xyz=(0.0, 0.009, 0.024)),
        material=indicator_white,
        name="timer_pointer",
    )

    oscillation_button = model.part("oscillation_button")
    oscillation_button.visual(
        Cylinder(radius=0.017, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=soft_button,
        name="button_cap",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, OSCILLATION_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.8, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "body_to_blower_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, -0.006, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=60.0),
    )
    model.articulation(
        "body_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=speed_dial,
        origin=Origin(xyz=(SPEED_DIAL_X, CONTROL_Y, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(TIMER_DIAL_X, CONTROL_Y, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )
    model.articulation(
        "body_to_oscillation_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=oscillation_button,
        origin=Origin(xyz=(OSC_BUTTON_X, CONTROL_Y, BODY_TOP_Z + 0.003)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.010),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower_wheel")
    speed = object_model.get_part("speed_dial")
    timer = object_model.get_part("timer_dial")
    button = object_model.get_part("oscillation_button")
    oscillation = object_model.get_articulation("base_to_body")
    button_slide = object_model.get_articulation("body_to_oscillation_button")

    ctx.check(
        "primary articulated parts present",
        all(part is not None for part in (base, body, blower, speed, timer, button)),
        "Expected base, body, blower wheel, two dials, and oscillation button parts.",
    )
    ctx.check(
        "control and rotor joints present",
        all(
            object_model.get_articulation(name) is not None
            for name in (
                "base_to_body",
                "body_to_blower_wheel",
                "body_to_speed_dial",
                "body_to_timer_dial",
                "body_to_oscillation_button",
            )
        ),
        "Expected oscillation, blower, dial, and push-button joints.",
    )

    ctx.expect_gap(
        body,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="oscillating body rests on weighted base",
    )
    ctx.expect_within(
        speed,
        body,
        axes="xy",
        inner_elem="speed_knob",
        outer_elem="control_panel",
        margin=0.002,
        name="speed dial is contained by the top panel",
    )
    ctx.expect_within(
        timer,
        body,
        axes="xy",
        inner_elem="timer_knob",
        outer_elem="control_panel",
        margin=0.002,
        name="timer dial is contained by the top panel",
    )
    ctx.expect_gap(
        speed,
        body,
        axis="z",
        positive_elem="speed_knob",
        negative_elem="control_panel",
        max_gap=0.0015,
        max_penetration=0.0,
        name="speed dial sits on the deck",
    )
    ctx.expect_gap(
        timer,
        body,
        axis="z",
        positive_elem="timer_knob",
        negative_elem="control_panel",
        max_gap=0.0015,
        max_penetration=0.0,
        name="timer dial sits on the deck",
    )
    ctx.expect_within(
        button,
        body,
        axes="xy",
        inner_elem="button_cap",
        outer_elem="button_socket",
        margin=0.001,
        name="oscillation button is supported by its socket",
    )
    ctx.expect_gap(
        button,
        body,
        axis="z",
        positive_elem="button_cap",
        negative_elem="button_socket",
        max_gap=0.001,
        max_penetration=0.000001,
        name="oscillation button rests on its socket",
    )

    if oscillation is not None and oscillation.motion_limits is not None:
        ctx.check(
            "body oscillation has limited yaw travel",
            oscillation.motion_limits.lower is not None
            and oscillation.motion_limits.upper is not None
            and oscillation.motion_limits.lower < -0.5
            and oscillation.motion_limits.upper > 0.5,
            details=f"limits={oscillation.motion_limits!r}",
        )

    rest_button_pos = ctx.part_world_position(button)
    if button_slide is not None:
        with ctx.pose({button_slide: 0.010}):
            pressed_button_pos = ctx.part_world_position(button)
        ctx.check(
            "oscillation button depresses downward",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[2] < rest_button_pos[2] - 0.008,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    return ctx.report()


object_model = build_object_model()
