from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="derrick_mast_crane")

    steel = model.material("painted_steel_dark", rgba=(0.16, 0.18, 0.19, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.06, 1.0))
    black = model.material("bearing_black", rgba=(0.02, 0.025, 0.025, 1.0))
    concrete = model.material("matte_concrete", rgba=(0.46, 0.46, 0.43, 1.0))
    pin_metal = model.material("bright_pin_metal", rgba=(0.72, 0.72, 0.68, 1.0))

    # Stationary foundation and the lower race of the slew bearing.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.72, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=concrete,
        name="foundation_disc",
    )
    base.visual(
        Cylinder(radius=0.48, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=steel,
        name="fixed_pedestal",
    )
    base.visual(
        mesh_from_geometry(TorusGeometry(radius=0.42, tube=0.025, radial_segments=18, tubular_segments=72), "lower_slew_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=black,
        name="lower_slew_ring",
    )

    # Rotating assembly: upper bearing ring, round central mast, and the luff yoke.
    carriage = model.part("slewing_carriage")
    carriage.visual(
        Cylinder(radius=0.46, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=steel,
        name="turntable_disc",
    )
    carriage.visual(
        mesh_from_geometry(TorusGeometry(radius=0.40, tube=0.026, radial_segments=18, tubular_segments=72), "upper_slew_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=black,
        name="upper_slew_ring",
    )
    carriage.visual(
        Cylinder(radius=0.11, length=2.40),
        origin=Origin(xyz=(0.0, 0.0, 1.32)),
        material=steel,
        name="round_mast",
    )
    carriage.visual(
        Cylinder(radius=0.14, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 2.565)),
        material=steel,
        name="mast_collar",
    )
    carriage.visual(
        Box((0.22, 0.36, 0.08)),
        origin=Origin(xyz=(0.04, 0.0, 2.58)),
        material=steel,
        name="yoke_bridge",
    )
    carriage.visual(
        Box((0.18, 0.050, 0.28)),
        origin=Origin(xyz=(0.08, 0.135, 2.72)),
        material=steel,
        name="yoke_cheek_0",
    )
    carriage.visual(
        Box((0.18, 0.050, 0.28)),
        origin=Origin(xyz=(0.08, -0.135, 2.72)),
        material=steel,
        name="yoke_cheek_1",
    )

    # A single diagonal boom, authored in the child frame whose origin is the
    # luff pin center.  At q=0 the boom leans upward and outward from the mast.
    boom = model.part("boom")
    boom_rise = 0.95
    boom_reach = 2.35
    boom.visual(
        mesh_from_geometry(
            wire_from_points(
                [(0.045, 0.0, 0.0), (boom_reach, 0.0, boom_rise)],
                radius=0.055,
                radial_segments=24,
                cap_ends=True,
                corner_mode="miter",
            ),
            "diagonal_boom_tube",
        ),
        origin=Origin(),
        material=yellow,
        name="boom_tube",
    )
    boom.visual(
        Cylinder(radius=0.083, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="hinge_hub",
    )
    boom.visual(
        Cylinder(radius=0.072, length=0.16),
        origin=Origin(xyz=(boom_reach, 0.0, boom_rise), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="tip_sheave",
    )

    # The entire mast and boom slew continuously on the base ring.
    model.articulation(
        "slew_bearing",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.6),
    )

    # The boom luffs about a horizontal pin at the mast top.
    model.articulation(
        "luff_joint",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=boom,
        origin=Origin(xyz=(0.08, 0.0, 2.72)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=-0.45, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("slewing_carriage")
    boom = object_model.get_part("boom")
    slew = object_model.get_articulation("slew_bearing")
    luff = object_model.get_articulation("luff_joint")

    ctx.check(
        "slew bearing is continuous vertical rotation",
        slew.articulation_type == ArticulationType.CONTINUOUS and slew.axis == (0.0, 0.0, 1.0),
        details=f"type={slew.articulation_type}, axis={slew.axis}",
    )
    ctx.check(
        "luff joint is a horizontal revolute hinge",
        luff.articulation_type == ArticulationType.REVOLUTE and abs(luff.axis[2]) < 1e-6,
        details=f"type={luff.articulation_type}, axis={luff.axis}",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="turntable_disc",
        negative_elem="fixed_pedestal",
        name="turntable is seated on pedestal",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        min_overlap=0.35,
        elem_a="upper_slew_ring",
        elem_b="lower_slew_ring",
        name="slewing rings are coaxial",
    )
    ctx.expect_overlap(
        boom,
        carriage,
        axes="xz",
        min_overlap=0.04,
        elem_a="hinge_hub",
        elem_b="yoke_cheek_0",
        name="boom hinge hub lines up with luff yoke",
    )

    rest_aabb = ctx.part_element_world_aabb(boom, elem="boom_tube")
    with ctx.pose({luff: 0.65}):
        raised_aabb = ctx.part_element_world_aabb(boom, elem="boom_tube")
    ctx.check(
        "positive luff raises boom tip",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.30,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
