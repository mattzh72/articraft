from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


PLASTIC = Material("warm_white_plastic", rgba=(0.88, 0.86, 0.78, 1.0))
BRACKET = Material("satin_black_metal", rgba=(0.03, 0.035, 0.04, 1.0))
WIRE = Material("zinc_wire", rgba=(0.62, 0.64, 0.62, 1.0))
HUB_DARK = Material("dark_motor_hub", rgba=(0.08, 0.085, 0.09, 1.0))
BLADE = Material("translucent_smoke_blade", rgba=(0.18, 0.27, 0.34, 0.72))
KNOB = Material("charcoal_knob", rgba=(0.015, 0.016, 0.018, 1.0))
MARK = Material("white_indicator_mark", rgba=(0.96, 0.95, 0.88, 1.0))


def _front_grille_mesh():
    """Concentric wire guard in the fan's front XZ plane at local +Y."""

    grille = None
    y = 0.264
    wire_radius = 0.0036

    # Several closely spaced rings make it read as a real wire grille rather
    # than a single decorative hoop.
    for radius in (0.060, 0.120, 0.178, 0.236, 0.300):
        points = [
            (
                radius * math.cos(theta),
                y,
                radius * math.sin(theta),
            )
            for theta in [2.0 * math.pi * i / 72 for i in range(72)]
        ]
        ring = tube_from_spline_points(
            points,
            radius=wire_radius,
            samples_per_segment=2,
            closed_spline=True,
            radial_segments=10,
            cap_ends=False,
        )
        grille = ring if grille is None else grille.merge(ring)

    # Radial wires tie every ring back to the hub ring and into the square
    # frame.  They deliberately cross the rings so the grille is one mounted
    # guard visually, not floating circles.
    for index in range(16):
        theta = 2.0 * math.pi * index / 16
        start_r = 0.045
        end_r = 0.334
        spoke = tube_from_spline_points(
            [
                (start_r * math.cos(theta), y, start_r * math.sin(theta)),
                (end_r * math.cos(theta), y, end_r * math.sin(theta)),
            ],
            radius=wire_radius * 0.82,
            samples_per_segment=3,
            closed_spline=False,
            radial_segments=10,
            cap_ends=True,
        )
        grille.merge(spoke)

    return grille


def _pivot_sleeve_mesh():
    """A hollow vertical plastic sleeve that clears the bracket's pan pin."""

    return LatheGeometry.from_shell_profiles(
        [(0.046, -0.118), (0.046, 0.118)],
        [(0.031, -0.104), (0.031, 0.104)],
        segments=48,
        start_cap="round",
        end_cap="round",
        lip_samples=6,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_oscillating_box_fan")

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.190, 0.030, 0.560)),
        origin=Origin(xyz=(0.0, -0.240, 0.0)),
        material=BRACKET,
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.074, 0.200, 0.046)),
        origin=Origin(xyz=(0.0, -0.125, -0.160)),
        material=BRACKET,
        name="support_arm",
    )
    wall_bracket.visual(
        Cylinder(radius=0.031, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=BRACKET,
        name="pivot_pin",
    )
    wall_bracket.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.131)),
        material=BRACKET,
        name="upper_pin_cap",
    )
    wall_bracket.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.131)),
        material=BRACKET,
        name="lower_pin_cap",
    )
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            wall_bracket.visual(
                Cylinder(radius=0.017, length=0.010),
                origin=Origin(
                    xyz=(sx * 0.052, -0.220, sz * 0.205),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=HUB_DARK,
                name=f"screw_head_{int(sx > 0)}_{int(sz > 0)}",
            )

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_pivot_sleeve_mesh(), "pivot_sleeve"),
        material=PLASTIC,
        name="pivot_sleeve",
    )
    housing.visual(
        Box((0.084, 0.130, 0.060)),
        origin=Origin(xyz=(0.0, 0.095, 0.0)),
        material=PLASTIC,
        name="rear_neck",
    )

    # Square box-fan housing: four deep frame bars, with a real open center.
    housing.visual(
        Box((0.066, 0.124, 0.682)),
        origin=Origin(xyz=(-0.307, 0.202, 0.0)),
        material=PLASTIC,
        name="side_frame_0",
    )
    housing.visual(
        Box((0.066, 0.124, 0.682)),
        origin=Origin(xyz=(0.307, 0.202, 0.0)),
        material=PLASTIC,
        name="side_frame_1",
    )
    housing.visual(
        Box((0.682, 0.124, 0.066)),
        origin=Origin(xyz=(0.0, 0.202, 0.307)),
        material=PLASTIC,
        name="top_frame",
    )
    housing.visual(
        Box((0.682, 0.124, 0.066)),
        origin=Origin(xyz=(0.0, 0.202, -0.307)),
        material=PLASTIC,
        name="bottom_frame",
    )
    housing.visual(
        Cylinder(radius=0.095, length=0.062),
        origin=Origin(xyz=(0.0, 0.126, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=HUB_DARK,
        name="motor_pod",
    )
    housing.visual(
        Box((0.570, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.132, 0.0)),
        material=HUB_DARK,
        name="rear_spoke_x",
    )
    housing.visual(
        Box((0.026, 0.030, 0.570)),
        origin=Origin(xyz=(0.0, 0.132, 0.0)),
        material=HUB_DARK,
        name="rear_spoke_z",
    )
    housing.visual(
        mesh_from_geometry(
            TorusGeometry(0.322, 0.011, radial_segments=18, tubular_segments=96),
            "front_bezel",
        ),
        origin=Origin(xyz=(0.0, 0.264, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=PLASTIC,
        name="front_bezel",
    )
    housing.visual(
        mesh_from_geometry(_front_grille_mesh(), "front_grille"),
        material=WIRE,
        name="front_grille",
    )
    housing.visual(
        Cylinder(radius=0.058, length=0.007),
        origin=Origin(xyz=(0.222, 0.268, -0.236), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=PLASTIC,
        name="dial_plate",
    )
    for angle, tick_name in ((-0.75, "tick_low"), (0.0, "tick_mid"), (0.75, "tick_high")):
        housing.visual(
            Box((0.006, 0.006, 0.023)),
            origin=Origin(
                xyz=(
                    0.222 + 0.052 * math.sin(angle),
                    0.272,
                    -0.236 + 0.052 * math.cos(angle),
                ),
                rpy=(0.0, angle, 0.0),
            ),
            material=MARK,
            name=tick_name,
        )

    propeller = model.part("propeller")
    rotor = FanRotorGeometry(
        0.236,
        0.054,
        4,
        thickness=0.036,
        blade_pitch_deg=31.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="broad", tip_pitch_deg=16.0, camber=0.12),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.012, bore_diameter=0.011),
    )
    propeller.visual(
        mesh_from_geometry(rotor, "four_blade_propeller"),
        material=BLADE,
        name="rotor",
    )
    propeller.visual(
        Cylinder(radius=0.011, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=HUB_DARK,
        name="axle_stub",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.028,
                body_style="skirted",
                top_diameter=0.039,
                edge_radius=0.0012,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0011, width=0.0018),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_knob",
        ),
        material=KNOB,
        name="knob_cap",
    )
    speed_knob.visual(
        Box((0.006, 0.028, 0.0016)),
        origin=Origin(xyz=(0.0, 0.010, 0.0285)),
        material=MARK,
        name="pointer_mark",
    )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=housing,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.8, lower=-0.78, upper=0.78),
        motion_properties=MotionProperties(damping=0.15, friction=0.04),
    )
    model.articulation(
        "propeller_axle",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=propeller,
        origin=Origin(xyz=(0.0, 0.202, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.4, velocity=95.0),
        motion_properties=MotionProperties(damping=0.01, friction=0.002),
    )
    model.articulation(
        "speed_selector",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(0.222, 0.272, -0.236), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=2.0, lower=-0.85, upper=0.85),
        motion_properties=MotionProperties(damping=0.04, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("wall_bracket")
    housing = object_model.get_part("housing")
    propeller = object_model.get_part("propeller")
    speed_knob = object_model.get_part("speed_knob")
    pan = object_model.get_articulation("pan_joint")
    axle = object_model.get_articulation("propeller_axle")
    selector = object_model.get_articulation("speed_selector")

    ctx.allow_overlap(
        housing,
        bracket,
        elem_a="pivot_sleeve",
        elem_b="pivot_pin",
        reason="The vertical pan pin is intentionally captured inside the simplified bearing sleeve at the wall bracket.",
    )
    ctx.check(
        "primary mechanisms are articulated",
        pan.articulation_type == ArticulationType.REVOLUTE
        and axle.articulation_type == ArticulationType.CONTINUOUS
        and selector.articulation_type == ArticulationType.REVOLUTE,
        details=f"pan={pan.articulation_type}, axle={axle.articulation_type}, selector={selector.articulation_type}",
    )
    ctx.expect_overlap(
        housing,
        bracket,
        axes="z",
        elem_a="pivot_sleeve",
        elem_b="pivot_pin",
        min_overlap=0.18,
        name="vertical pan sleeve surrounds bracket pin height",
    )
    ctx.expect_within(
        bracket,
        housing,
        axes="xy",
        inner_elem="pivot_pin",
        outer_elem="pivot_sleeve",
        margin=0.0,
        name="pan pin is centered inside sleeve footprint",
    )
    ctx.expect_gap(
        housing,
        propeller,
        axis="y",
        positive_elem="front_grille",
        negative_elem="rotor",
        min_gap=0.030,
        max_gap=0.070,
        name="propeller sits behind the wire grille",
    )
    ctx.expect_within(
        propeller,
        housing,
        axes="xz",
        inner_elem="rotor",
        outer_elem="front_grille",
        margin=0.010,
        name="four blade propeller fits inside grille circle",
    )
    ctx.expect_gap(
        speed_knob,
        housing,
        axis="y",
        positive_elem="knob_cap",
        negative_elem="dial_plate",
        min_gap=0.0,
        max_gap=0.010,
        name="speed selector knob is seated on front dial",
    )

    closed_aabb = ctx.part_element_world_aabb(housing, elem="front_grille")
    with ctx.pose({pan: 0.55}):
        panned_aabb = ctx.part_element_world_aabb(housing, elem="front_grille")
    closed_center_x = None if closed_aabb is None else (closed_aabb[0][0] + closed_aabb[1][0]) * 0.5
    panned_center_x = None if panned_aabb is None else (panned_aabb[0][0] + panned_aabb[1][0]) * 0.5
    ctx.check(
        "pan joint swings the box housing side to side",
        closed_center_x is not None
        and panned_center_x is not None
        and abs(panned_center_x - closed_center_x) > 0.055,
        details=f"closed_x={closed_center_x}, panned_x={panned_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
