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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_live_axle_beam")

    painted_steel = model.material("painted_steel", color=(0.06, 0.065, 0.065, 1.0))
    forged_steel = model.material("forged_steel", color=(0.22, 0.23, 0.22, 1.0))
    machined_steel = model.material("machined_steel", color=(0.55, 0.56, 0.54, 1.0))
    dark_rubber = model.material("dark_seal", color=(0.015, 0.015, 0.014, 1.0))

    # Root frame: +X spans the axle beam, +Y is fore/aft vehicle depth, +Z is up.
    beam = model.part("beam")
    beam.visual(
        Box((1.40, 0.16, 0.14)),
        origin=Origin(),
        material=painted_steel,
        name="solid_beam",
    )

    kingpin_x = 0.78
    for idx, side in enumerate((-1.0, 1.0)):
        x = side * kingpin_x

        # Thick end webs tie the yoke bosses back into the rectangular beam.
        beam.visual(
            Box((0.08, 0.15, 0.32)),
            origin=Origin(xyz=(side * 0.66, 0.0, 0.0)),
            material=painted_steel,
            name=f"end_web_{idx}",
        )
        for z, suffix in ((0.1375, "upper"), (-0.1375, "lower")):
            beam.visual(
                Cylinder(radius=0.095, length=0.075),
                origin=Origin(xyz=(x, 0.0, z)),
                material=painted_steel,
                name=f"kingpin_{suffix}_{idx}",
            )

    for idx, side in enumerate((-1.0, 1.0)):
        # Rotate the left-side joint frame 180 degrees so every knuckle's local
        # +X points outward from the beam.  The steering axis remains vertical.
        yaw = math.pi if side < 0.0 else 0.0

        knuckle = model.part(f"knuckle_{idx}")
        knuckle.visual(
            Cylinder(radius=0.055, length=0.20),
            origin=Origin(),
            material=forged_steel,
            name="kingpin_barrel",
        )
        knuckle.visual(
            Box((0.22, 0.095, 0.082)),
            origin=Origin(xyz=(0.145, 0.0, 0.0)),
            material=forged_steel,
            name="spindle_bridge",
        )
        knuckle.visual(
            Cylinder(radius=0.076, length=0.09),
            origin=Origin(xyz=(0.255, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=forged_steel,
            name="bearing_boss",
        )
        knuckle.visual(
            Box((0.10, 0.15, 0.035)),
            origin=Origin(xyz=(0.01, -0.075, 0.075)),
            material=forged_steel,
            name="steering_arm",
        )

        model.articulation(
            f"kingpin_{idx}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=knuckle,
            origin=Origin(xyz=(side * kingpin_x, 0.0, 0.0), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2200.0, velocity=1.4, lower=-0.58, upper=0.58),
        )

        hub = model.part(f"hub_{idx}")
        hub.visual(
            Cylinder(radius=0.090, length=0.11),
            origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="hub_barrel",
        )
        hub.visual(
            Cylinder(radius=0.155, length=0.045),
            origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="wheel_flange",
        )
        hub.visual(
            Cylinder(radius=0.056, length=0.080),
            origin=Origin(xyz=(0.175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="grease_cap",
        )
        hub.visual(
            Cylinder(radius=0.034, length=0.018),
            origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_rubber,
            name="inner_seal",
        )
        for lug_idx in range(5):
            angle = 2.0 * math.pi * lug_idx / 5.0
            hub.visual(
                Cylinder(radius=0.011, length=0.056),
                origin=Origin(
                    xyz=(0.174, 0.095 * math.cos(angle), 0.095 * math.sin(angle)),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=machined_steel,
                name=f"lug_stud_{lug_idx}",
            )

        model.articulation(
            f"hub_spin_{idx}",
            ArticulationType.CONTINUOUS,
            parent=knuckle,
            child=hub,
            origin=Origin(xyz=(0.300, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=900.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    beam = object_model.get_part("beam")

    for idx in range(2):
        knuckle = object_model.get_part(f"knuckle_{idx}")
        hub = object_model.get_part(f"hub_{idx}")
        kingpin = object_model.get_articulation(f"kingpin_{idx}")
        hub_spin = object_model.get_articulation(f"hub_spin_{idx}")

        ctx.check(
            f"kingpin_{idx} is a limited vertical steering revolute",
            kingpin.articulation_type == ArticulationType.REVOLUTE
            and tuple(kingpin.axis) == (0.0, 0.0, 1.0)
            and kingpin.motion_limits is not None
            and kingpin.motion_limits.lower <= -0.5
            and kingpin.motion_limits.upper >= 0.5,
            details=f"type={kingpin.articulation_type}, axis={kingpin.axis}, limits={kingpin.motion_limits}",
        )
        ctx.check(
            f"hub_spin_{idx} is a continuous axle revolute",
            hub_spin.articulation_type == ArticulationType.CONTINUOUS
            and tuple(hub_spin.axis) == (1.0, 0.0, 0.0),
            details=f"type={hub_spin.articulation_type}, axis={hub_spin.axis}",
        )

        ctx.expect_overlap(
            knuckle,
            beam,
            axes="xy",
            elem_a="kingpin_barrel",
            elem_b=f"kingpin_upper_{idx}",
            min_overlap=0.08,
            name=f"kingpin_{idx} barrel sits under upper yoke boss",
        )
        ctx.expect_gap(
            beam,
            knuckle,
            axis="z",
            positive_elem=f"kingpin_upper_{idx}",
            negative_elem="kingpin_barrel",
            min_gap=0.0,
            max_gap=0.003,
            name=f"kingpin_{idx} upper boss has bearing clearance",
        )
        ctx.expect_gap(
            knuckle,
            beam,
            axis="z",
            positive_elem="kingpin_barrel",
            negative_elem=f"kingpin_lower_{idx}",
            min_gap=0.0,
            max_gap=0.003,
            name=f"kingpin_{idx} lower boss has bearing clearance",
        )

        if idx == 0:
            ctx.expect_gap(
                knuckle,
                hub,
                axis="x",
                positive_elem="bearing_boss",
                negative_elem="hub_barrel",
                min_gap=0.0,
                max_gap=0.001,
                name="left hub seats at knuckle face",
            )
        else:
            ctx.expect_gap(
                hub,
                knuckle,
                axis="x",
                positive_elem="hub_barrel",
                negative_elem="bearing_boss",
                min_gap=0.0,
                max_gap=0.001,
                name="right hub seats at knuckle face",
            )

        hub_rest = ctx.part_world_position(hub)
        with ctx.pose({kingpin: 0.45}):
            hub_steered = ctx.part_world_position(hub)
        ctx.check(
            f"kingpin_{idx} steers the hub around the vertical pivot",
            hub_rest is not None
            and hub_steered is not None
            and abs(hub_steered[1] - hub_rest[1]) > 0.10,
            details=f"rest={hub_rest}, steered={hub_steered}",
        )

        hub_origin = ctx.part_world_position(hub)
        with ctx.pose({hub_spin: 1.3}):
            spun_origin = ctx.part_world_position(hub)
        ctx.check(
            f"hub_spin_{idx} rotates in place on its axle",
            hub_origin is not None
            and spun_origin is not None
            and sum((hub_origin[i] - spun_origin[i]) ** 2 for i in range(3)) < 1.0e-10,
            details=f"rest={hub_origin}, spun={spun_origin}",
        )

    return ctx.report()


object_model = build_object_model()
