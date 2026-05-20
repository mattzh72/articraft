from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _hollow_ring(outer_radius: float, inner_radius: float, height: float, name: str):
    """CadQuery annular cylinder, authored from z=0 to z=height."""
    return mesh_from_cadquery(
        cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height),
        name,
        tolerance=0.0007,
        angular_tolerance=0.08,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nutribullet_sippy_blender")

    graphite = model.material("matte_graphite", rgba=(0.05, 0.052, 0.055, 1.0))
    black = model.material("soft_black_plastic", rgba=(0.012, 0.012, 0.014, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.74, 0.72, 0.68, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.01, 0.01, 0.011, 1.0))
    translucent = model.material("smoky_clear_copolyester", rgba=(0.78, 0.92, 1.0, 0.34))
    cup_gray = model.material("threaded_gray_plastic", rgba=(0.30, 0.31, 0.32, 1.0))
    stainless = model.material("sharpened_stainless", rgba=(0.88, 0.88, 0.84, 1.0))
    mark_white = model.material("etched_white_marks", rgba=(0.92, 0.96, 1.0, 0.78))

    # Root: a low, puck-like Nutribullet motor base with a raised drive socket.
    base = model.part("motor_base")
    base.visual(
        Cylinder(radius=0.115, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_rubber,
        name="rubber_foot_disc",
    )
    base.visual(
        Cylinder(radius=0.102, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=graphite,
        name="base_body",
    )
    base.visual(
        Cylinder(radius=0.107, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=chrome,
        name="chrome_shoulder",
    )
    base.visual(
        Cylinder(radius=0.087, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=black,
        name="top_plate",
    )
    base.visual(
        _hollow_ring(0.035, 0.012, 0.012, "drive_socket"),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=chrome,
        name="drive_socket",
    )
    # Three low bayonet lugs inside the threaded cup mouth; they are visibly
    # attached to the drive socket and do not collide with the cup collar.
    for i in range(3):
        yaw = i * (2.0 * math.pi / 3.0)
        r = 0.025
        base.visual(
            Box((0.015, 0.008, 0.006)),
            origin=Origin(
                xyz=(r * math.cos(yaw), r * math.sin(yaw), 0.109),
                rpy=(0.0, 0.0, yaw),
            ),
            material=chrome,
            name=f"drive_lug_{i}",
        )

    # The short, squat cup is shown in blending orientation: its threaded mouth
    # is down on the base, the clear hollow jar rises above it, and a drinking
    # flip-lid sits on the upper rim for use after blending.
    cup = model.part("cup_jar")
    cup.visual(
        _hollow_ring(0.078, 0.044, 0.022, "lower_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=cup_gray,
        name="lower_collar",
    )
    for i, z in enumerate((0.100, 0.107, 0.114)):
        cup.visual(
            _hollow_ring(0.081, 0.077, 0.003, f"thread_band_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=cup_gray,
            name=f"thread_band_{i}",
        )
    cup.visual(
        _hollow_ring(0.080, 0.073, 0.164, "clear_cup_wall"),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=translucent,
        name="clear_cup_wall",
    )
    cup.visual(
        _hollow_ring(0.086, 0.071, 0.016, "top_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.276)),
        material=black,
        name="top_rim",
    )
    # Measuring ticks are slightly embedded in the outer wall so they read as
    # printed markings rather than floating decals.
    for i, z in enumerate((0.145, 0.170, 0.195, 0.220, 0.245)):
        tick_len = 0.032 if i % 2 == 0 else 0.022
        cup.visual(
            Box((tick_len, 0.0025, 0.004)),
            origin=Origin(xyz=(-0.030 + tick_len / 2.0, -0.079, z)),
            material=mark_white,
            name=f"fill_mark_{i}",
        )
    # Fixed hinge knuckles molded into the cup rim.
    for i, y in enumerate((-0.050, 0.050)):
        cup.visual(
            Box((0.018, 0.018, 0.012)),
            origin=Origin(xyz=(0.079, y, 0.291)),
            material=black,
            name=f"hinge_boss_{i}",
        )
        cup.visual(
            Cylinder(radius=0.006, length=0.024),
            origin=Origin(xyz=(0.084, y, 0.297), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"rim_hinge_knuckle_{i}",
        )

    model.articulation(
        "base_to_cup",
        ArticulationType.FIXED,
        parent=base,
        child=cup,
        origin=Origin(),
    )

    # Cross blade rotor at the downward cup mouth.  It spins about the vertical
    # drive axis and is dimensioned to clear the hollow cup wall.
    blade = model.part("blade_cross")
    blade.visual(
        Cylinder(radius=0.009, length=0.023),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=chrome,
        name="drive_shaft",
    )
    blade.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=chrome,
        name="blade_hub",
    )
    blade.visual(
        Box((0.028, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=chrome,
        name="drive_tang",
    )
    for i, arm_name in enumerate(("blade_arm_0", "blade_arm_1", "blade_arm_2", "blade_arm_3")):
        yaw = i * (math.pi / 2.0) + math.pi / 4.0
        r = 0.033
        # Two opposing blades pitch upward, two pitch downward, like the
        # stamped cross blades in personal blender blade caps.
        pitch = 0.20 if i % 2 == 0 else -0.20
        blade.visual(
            Box((0.058, 0.014, 0.003)),
            origin=Origin(
                xyz=(r * math.cos(yaw), r * math.sin(yaw), 0.003),
                rpy=(pitch, 0.0, yaw),
            ),
            material=stainless,
            name=arm_name,
        )

    model.articulation(
        "cup_to_blade",
        ArticulationType.CONTINUOUS,
        parent=cup,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )

    # A hinged flip-top drinking cap at the upper rim.  The child frame is on
    # the hinge barrel; the cap disc extends inward in local -X when closed.
    flip_lid = model.part("flip_lid")
    flip_lid.visual(
        Cylinder(radius=0.081, length=0.010),
        origin=Origin(xyz=(-0.084, 0.0, 0.0005)),
        material=black,
        name="lid_disc",
    )
    flip_lid.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hinge_barrel",
    )
    flip_lid.visual(
        Box((0.045, 0.026, 0.018)),
        origin=Origin(xyz=(-0.130, 0.0, 0.010)),
        material=black,
        name="sip_spout",
    )
    flip_lid.visual(
        Box((0.030, 0.018, 0.002)),
        origin=Origin(xyz=(-0.130, 0.0, 0.020)),
        material=dark_rubber,
        name="spout_slot",
    )
    flip_lid.visual(
        Box((0.018, 0.040, 0.006)),
        origin=Origin(xyz=(-0.162, 0.0, 0.000)),
        material=cup_gray,
        name="thumb_tab",
    )

    model.articulation(
        "cup_to_flip_lid",
        ArticulationType.REVOLUTE,
        parent=cup,
        child=flip_lid,
        origin=Origin(xyz=(0.084, 0.0, 0.297)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=1.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("motor_base")
    cup = object_model.get_part("cup_jar")
    blade = object_model.get_part("blade_cross")
    flip_lid = object_model.get_part("flip_lid")
    blade_joint = object_model.get_articulation("cup_to_blade")
    lid_joint = object_model.get_articulation("cup_to_flip_lid")

    ctx.allow_overlap(
        base,
        blade,
        elem_a="drive_socket",
        elem_b="drive_tang",
        reason="The blade drive tang is intentionally seated in the motor coupling socket.",
    )
    ctx.expect_within(
        blade,
        base,
        axes="xy",
        inner_elem="drive_tang",
        outer_elem="drive_socket",
        margin=0.002,
        name="drive tang is centered in socket",
    )
    ctx.expect_overlap(
        blade,
        base,
        axes="z",
        elem_a="drive_tang",
        elem_b="drive_socket",
        min_overlap=0.006,
        name="drive tang remains inserted in coupling",
    )
    ctx.expect_gap(
        cup,
        base,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="top_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="threaded cup collar sits on motor base",
    )
    ctx.expect_within(
        blade,
        cup,
        axes="xy",
        inner_elem="blade_arm_0",
        outer_elem="clear_cup_wall",
        margin=0.010,
        name="cross blade clears inside cup wall",
    )
    ctx.expect_gap(
        flip_lid,
        cup,
        axis="z",
        positive_elem="lid_disc",
        negative_elem="top_rim",
        max_gap=0.0015,
        max_penetration=0.0,
        name="closed flip lid rests on top rim",
    )
    ctx.check(
        "blade uses vertical spin axis",
        tuple(round(v, 6) for v in blade_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={blade_joint.axis}",
    )

    rest_spout = ctx.part_element_world_aabb(flip_lid, elem="sip_spout")
    with ctx.pose({lid_joint: 1.20, blade_joint: math.pi / 2.0}):
        open_spout = ctx.part_element_world_aabb(flip_lid, elem="sip_spout")
        ctx.expect_within(
            blade,
            cup,
            axes="xy",
            inner_elem="blade_arm_0",
            outer_elem="clear_cup_wall",
            margin=0.010,
            name="rotated blade remains inside cup wall",
        )
    ctx.check(
        "flip lid opens upward from rim hinge",
        rest_spout is not None
        and open_spout is not None
        and open_spout[1][2] > rest_spout[1][2] + 0.045,
        details=f"rest={rest_spout}, open={open_spout}",
    )

    return ctx.report()


object_model = build_object_model()
