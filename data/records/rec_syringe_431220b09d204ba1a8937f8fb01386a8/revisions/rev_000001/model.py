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


def _x_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Origin for a primitive cylinder whose length should run along syringe X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _y_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Origin for a primitive cylinder whose length should run along syringe Y."""
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _tube(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    """Open annular tube extruded along +X in local coordinates."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_syringe")

    barrel_clear = model.material("clear_polycarbonate", rgba=(0.55, 0.85, 1.0, 0.34))
    etched_black = model.material("etched_black", rgba=(0.02, 0.025, 0.025, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    dark_steel = model.material("dark_hardened_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.04, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.85, 0.05, 0.03, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    frame = model.part("barrel_frame")

    frame.visual(
        mesh_from_cadquery(_tube(0.255, 0.035, 0.030), "barrel_tube"),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=barrel_clear,
        name="barrel_tube",
    )
    frame.visual(
        mesh_from_cadquery(_tube(0.030, 0.047, 0.0305), "rear_pressure_ring"),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=brushed_steel,
        name="rear_pressure_ring",
    )
    frame.visual(
        mesh_from_cadquery(_tube(0.030, 0.047, 0.0305), "front_pressure_ring"),
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
        material=brushed_steel,
        name="front_pressure_ring",
    )

    # Reinforced nozzle stack: short stout boss, smaller nose, and dark outlet.
    frame.visual(Cylinder(radius=0.034, length=0.030), origin=_x_origin(0.307), material=brushed_steel, name="nozzle_boss")
    frame.visual(Cylinder(radius=0.013, length=0.030), origin=_x_origin(0.333), material=brushed_steel, name="nozzle_reducer")
    frame.visual(Cylinder(radius=0.008, length=0.040), origin=_x_origin(0.365), material=brushed_steel, name="nozzle_tip")
    frame.visual(Cylinder(radius=0.0055, length=0.002), origin=_x_origin(0.386), material=etched_black, name="outlet_bore")

    # External guard cage and load path from collars to the rear guide.
    frame.visual(Box((0.285, 0.014, 0.014)), origin=Origin(xyz=(0.157, 0.020, 0.063)), material=dark_steel, name="upper_guard_rail")
    frame.visual(Box((0.285, 0.014, 0.014)), origin=Origin(xyz=(0.157, 0.0, -0.063)), material=dark_steel, name="lower_guard_rail")
    for x, label in ((0.035, "rear"), (0.280, "front")):
        frame.visual(Box((0.020, 0.014, 0.044)), origin=Origin(xyz=(x, 0.020, 0.049)), material=dark_steel, name=f"{label}_upper_post")
        frame.visual(Box((0.020, 0.014, 0.044)), origin=Origin(xyz=(x, 0.0, -0.049)), material=dark_steel, name=f"{label}_lower_post")

    for y, label in ((0.060, "side_0"), (-0.060, "side_1")):
        frame.visual(Box((0.285, 0.012, 0.018)), origin=Origin(xyz=(0.157, y, 0.0)), material=dark_steel, name=f"{label}_guard_rail")
        frame.visual(Box((0.020, 0.044, 0.018)), origin=Origin(xyz=(0.035, y * 0.80, 0.0)), material=dark_steel, name=f"{label}_rear_post")
        frame.visual(Box((0.020, 0.044, 0.018)), origin=Origin(xyz=(0.280, y * 0.80, 0.0)), material=dark_steel, name=f"{label}_front_post")

    # Rear guide yoke with a clear central opening for the coaxial rod.
    frame.visual(Box((0.075, 0.014, 0.012)), origin=Origin(xyz=(-0.006, 0.0, 0.036)), material=dark_steel, name="guide_top_bridge")
    frame.visual(Box((0.075, 0.014, 0.012)), origin=Origin(xyz=(-0.006, 0.0, -0.036)), material=dark_steel, name="guide_bottom_bridge")
    frame.visual(Box((0.075, 0.012, 0.014)), origin=Origin(xyz=(-0.006, 0.036, 0.0)), material=dark_steel, name="guide_side_bridge_0")
    frame.visual(Box((0.075, 0.012, 0.014)), origin=Origin(xyz=(-0.006, -0.036, 0.0)), material=dark_steel, name="guide_side_bridge_1")
    frame.visual(Box((0.035, 0.092, 0.012)), origin=Origin(xyz=(-0.038, 0.0, 0.036)), material=brushed_steel, name="rear_yoke_top")
    frame.visual(Box((0.035, 0.092, 0.012)), origin=Origin(xyz=(-0.038, 0.0, -0.036)), material=brushed_steel, name="rear_yoke_bottom")
    frame.visual(Box((0.035, 0.012, 0.084)), origin=Origin(xyz=(-0.038, 0.046, 0.0)), material=brushed_steel, name="rear_yoke_side_0")
    frame.visual(Box((0.035, 0.012, 0.084)), origin=Origin(xyz=(-0.038, -0.046, 0.0)), material=brushed_steel, name="rear_yoke_side_1")

    # Rear hand guard: a rigid hoop behind the thumb plate, braced back to the guide.
    frame.visual(Box((0.018, 0.144, 0.012)), origin=Origin(xyz=(-0.132, 0.0, 0.058)), material=safety_yellow, name="hand_guard_top")
    frame.visual(Box((0.018, 0.144, 0.012)), origin=Origin(xyz=(-0.132, 0.0, -0.058)), material=safety_yellow, name="hand_guard_bottom")
    frame.visual(Box((0.018, 0.012, 0.128)), origin=Origin(xyz=(-0.132, 0.072, 0.0)), material=safety_yellow, name="hand_guard_side_0")
    frame.visual(Box((0.018, 0.012, 0.128)), origin=Origin(xyz=(-0.132, -0.072, 0.0)), material=safety_yellow, name="hand_guard_side_1")
    for y, label in ((0.058, "hand_strut_0"), (-0.058, "hand_strut_1")):
        frame.visual(Box((0.100, 0.016, 0.022)), origin=Origin(xyz=(-0.084, y, 0.051)), material=safety_yellow, name=f"{label}_upper")
        frame.visual(Box((0.100, 0.016, 0.022)), origin=Origin(xyz=(-0.084, y, -0.051)), material=safety_yellow, name=f"{label}_lower")

    # Positive-stop yoke: pads stop the plunger collar at the upper travel limit.
    frame.visual(Box((0.015, 0.052, 0.018)), origin=Origin(xyz=(0.0925, 0.0, 0.026)), material=safety_yellow, name="front_stop_upper")
    frame.visual(Box((0.015, 0.052, 0.018)), origin=Origin(xyz=(0.0925, 0.0, -0.026)), material=safety_yellow, name="front_stop_lower")

    # Lockout rack teeth are structural, not decorative: they bolt to the upper barrel rail.
    frame.visual(Box((0.122, 0.014, 0.006)), origin=Origin(xyz=(0.123, 0.0, 0.0375)), material=safety_yellow, name="lockout_rack")
    for i, x in enumerate((0.090, 0.110, 0.130, 0.150, 0.170)):
        frame.visual(Box((0.006, 0.018, 0.010)), origin=Origin(xyz=(x, 0.0, 0.043)), material=safety_yellow, name=f"rack_tooth_{i}")

    # Graduated scale marks printed/etched into the top of the transparent barrel.
    for i in range(10):
        x = 0.065 + 0.020 * i
        major = i % 5 == 0
        frame.visual(
            Box((0.0025, 0.030 if major else 0.018, 0.0012)),
            origin=Origin(xyz=(x, -0.012 if major else -0.018, 0.0318)),
            material=etched_black,
            name=f"graduation_{i}",
        )

    # Visible fastener logic on pressure rings and lockout mount.
    bolt_positions = ((0.028, 0.028), (0.028, -0.028), (-0.028, 0.028), (-0.028, -0.028))
    for i, (y, z) in enumerate(bolt_positions):
        frame.visual(Cylinder(radius=0.0042, length=0.003), origin=_x_origin(0.014, y, z), material=dark_steel, name=f"rear_bolt_{i}")
        frame.visual(Cylinder(radius=0.0042, length=0.003), origin=_x_origin(0.301, y, z), material=dark_steel, name=f"front_bolt_{i}")

    # Latch hinge bracket.  The pin is intentionally captured by the latch knuckle.
    frame.visual(Box((0.018, 0.014, 0.052)), origin=Origin(xyz=(0.006, 0.0, 0.061)), material=brushed_steel, name="latch_stanchion")
    frame.visual(Box((0.032, 0.064, 0.010)), origin=Origin(xyz=(0.006, 0.0, 0.084)), material=brushed_steel, name="latch_base")
    frame.visual(Box((0.018, 0.010, 0.024)), origin=Origin(xyz=(0.006, 0.027, 0.099)), material=brushed_steel, name="latch_ear_0")
    frame.visual(Box((0.018, 0.010, 0.024)), origin=Origin(xyz=(0.006, -0.027, 0.099)), material=brushed_steel, name="latch_ear_1")
    frame.visual(Cylinder(radius=0.004, length=0.064), origin=_y_origin(0.006, 0.0, 0.099), material=dark_steel, name="latch_pin")

    plunger = model.part("plunger")
    plunger.visual(Cylinder(radius=0.006, length=0.250), origin=_x_origin(-0.005), material=brushed_steel, name="plunger_rod")
    plunger.visual(Cylinder(radius=0.0304, length=0.025), origin=_x_origin(0.100), material=rubber, name="piston_seal")
    plunger.visual(Cylinder(radius=0.018, length=0.015), origin=_x_origin(-0.0525), material=safety_yellow, name="travel_collar")
    plunger.visual(Box((0.018, 0.098, 0.062)), origin=Origin(xyz=(-0.134, 0.0, 0.0)), material=dark_steel, name="thumb_plate")
    plunger.visual(Box((0.006, 0.080, 0.046)), origin=Origin(xyz=(-0.1445, 0.0, 0.0)), material=rubber, name="thumb_pad")

    safety_latch = model.part("safety_latch")
    safety_latch.visual(Cylinder(radius=0.007, length=0.022), origin=_y_origin(0.0, 0.0, 0.0), material=lockout_red, name="hinge_barrel")
    safety_latch.visual(Box((0.105, 0.018, 0.012)), origin=Origin(xyz=(0.053, 0.0, 0.004)), material=lockout_red, name="latch_handle")
    safety_latch.visual(Box((0.014, 0.014, 0.050)), origin=Origin(xyz=(0.075, 0.0, -0.022)), material=lockout_red, name="latch_pawl")
    safety_latch.visual(Box((0.030, 0.026, 0.006)), origin=Origin(xyz=(0.102, 0.0, 0.010)), material=etched_black, name="grip_pad")

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=plunger,
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.30, lower=0.0, upper=0.170),
    )
    model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=safety_latch,
        origin=Origin(xyz=(0.006, 0.0, 0.099)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("barrel_frame")
    plunger = object_model.get_part("plunger")
    latch = object_model.get_part("safety_latch")
    slide = object_model.get_articulation("plunger_slide")
    hinge = object_model.get_articulation("latch_hinge")

    ctx.allow_overlap(
        frame,
        latch,
        elem_a="latch_pin",
        elem_b="hinge_barrel",
        reason="The safety latch knuckle is intentionally captured around the fixed hinge pin.",
    )
    ctx.allow_overlap(
        frame,
        plunger,
        elem_a="barrel_tube",
        elem_b="piston_seal",
        reason="The rubber piston seal is intentionally modeled as a light compressed fit against the barrel bore.",
    )
    ctx.expect_within(
        frame,
        latch,
        axes="xz",
        inner_elem="latch_pin",
        outer_elem="hinge_barrel",
        margin=0.002,
        name="hinge pin sits inside latch barrel",
    )
    ctx.expect_overlap(
        frame,
        latch,
        axes="y",
        elem_a="latch_pin",
        elem_b="hinge_barrel",
        min_overlap=0.018,
        name="latch barrel retains captured pin length",
    )

    ctx.expect_within(
        plunger,
        frame,
        axes="yz",
        inner_elem="piston_seal",
        outer_elem="barrel_tube",
        margin=0.002,
        name="piston is coaxial inside barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        frame,
        axes="x",
        elem_a="piston_seal",
        elem_b="barrel_tube",
        min_overlap=0.020,
        name="piston remains inserted at rear stroke",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.170}):
        ctx.expect_within(
            plunger,
            frame,
            axes="yz",
            inner_elem="piston_seal",
            outer_elem="barrel_tube",
            margin=0.002,
            name="piston remains coaxial at full stroke",
        )
        ctx.expect_overlap(
            plunger,
            frame,
            axes="x",
            elem_a="piston_seal",
            elem_b="barrel_tube",
            min_overlap=0.020,
            name="piston remains inserted at full stroke",
        )
        ctx.expect_gap(
            frame,
            plunger,
            axis="x",
            positive_elem="front_stop_upper",
            negative_elem="travel_collar",
            max_gap=0.001,
            max_penetration=0.0002,
            name="upper stop blocks over-travel",
        )
        extended_pos = ctx.part_world_position(plunger)

    with ctx.pose({hinge: 0.75}):
        ctx.expect_gap(
            latch,
            frame,
            axis="z",
            positive_elem="latch_pawl",
            negative_elem="lockout_rack",
            min_gap=0.010,
            name="lockout latch lifts clear of rack",
        )

    ctx.check(
        "plunger advances along syringe axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.16,
        details=f"rest={rest_pos}, full_stroke={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
