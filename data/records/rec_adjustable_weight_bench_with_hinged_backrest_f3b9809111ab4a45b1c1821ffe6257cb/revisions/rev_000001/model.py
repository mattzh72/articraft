from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_incline_utility_bench")

    steel = model.material("powder_coated_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    vinyl = model.material("black_vinyl_pad", rgba=(0.015, 0.014, 0.013, 1.0))
    seam = model.material("pad_seam", rgba=(0.10, 0.10, 0.095, 1.0))
    rubber = model.material("dense_foam_roller", rgba=(0.02, 0.02, 0.018, 1.0))
    pin = model.material("brushed_pin", rgba=(0.55, 0.56, 0.54, 1.0))
    ladder_paint = model.material("ladder_steel", rgba=(0.08, 0.085, 0.09, 1.0))

    frame = model.part("frame")

    # Garage-gym rectangular base frame: low powder-coated box tubing.
    frame.visual(
        Box((1.70, 0.050, 0.055)),
        origin=Origin(xyz=(0.10, -0.25, 0.080)),
        material=steel,
        name="side_rail_0",
    )
    frame.visual(
        Box((1.70, 0.050, 0.055)),
        origin=Origin(xyz=(0.10, 0.25, 0.080)),
        material=steel,
        name="side_rail_1",
    )
    for name, x in (("front_crossbar", -0.75), ("mid_crossbar", 0.10), ("rear_crossbar", 0.92)):
        frame.visual(
            Box((0.070, 0.58, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.082)),
            material=steel,
            name=name,
        )

    # Upright hinge towers for the separate seat-front and backrest hinges.
    for x, prefix in ((-0.38, "seat_hinge"), (0.10, "backrest_hinge")):
        for i, y in enumerate((-0.205, 0.205)):
            frame.visual(
                Box((0.042, 0.046, 0.325)),
                origin=Origin(xyz=(x, y, 0.262)),
                material=steel,
                name=f"{prefix}_post_{i}",
            )
            frame.visual(
                Box((0.075, 0.034, 0.090)),
                origin=Origin(xyz=(x, y, 0.420)),
                material=steel,
                name=f"{prefix}_tab_{i}",
            )
    frame.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(-0.38, 0.0, 0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="seat_hinge_pin_line",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(0.10, 0.0, 0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="backrest_hinge_pin_line",
    )

    # Rear lower pivot brackets for the visible support ladder, placed behind
    # the backrest bracket rather than hidden inside the main base rectangle.
    for i, y in enumerate((-0.255, 0.255)):
        frame.visual(
            Box((0.085, 0.040, 0.145)),
            origin=Origin(xyz=(0.88, y, 0.145)),
            material=steel,
            name=f"ladder_pivot_tab_{i}",
        )
    frame.visual(
        Cylinder(radius=0.014, length=0.55),
        origin=Origin(xyz=(0.88, 0.0, 0.140), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="ladder_pivot_pin",
    )

    # Rigid front yoke and shared axle for the two rotating foot rollers.
    frame.visual(
        Box((0.22, 0.090, 0.055)),
        origin=Origin(xyz=(-0.84, 0.0, 0.108)),
        material=steel,
        name="yoke_neck",
    )
    frame.visual(
        Box((0.055, 0.085, 0.285)),
        origin=Origin(xyz=(-0.92, 0.0, 0.238)),
        material=steel,
        name="yoke_stem",
    )
    frame.visual(
        Box((0.070, 0.220, 0.045)),
        origin=Origin(xyz=(-0.92, 0.0, 0.303)),
        material=steel,
        name="yoke_bridge",
    )
    for i, y in enumerate((-0.065, 0.065)):
        frame.visual(
            Box((0.075, 0.035, 0.170)),
            origin=Origin(xyz=(-0.92, y, 0.365)),
            material=steel,
            name=f"yoke_cheek_{i}",
        )
    frame.visual(
        Cylinder(radius=0.014, length=0.62),
        origin=Origin(xyz=(-0.92, 0.0, 0.365), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="roller_axle",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.024, length=0.320),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="seat_hinge_barrel",
    )
    seat.visual(
        Box((0.430, 0.070, 0.034)),
        origin=Origin(xyz=(0.235, 0.0, 0.018)),
        material=steel,
        name="seat_under_rail",
    )
    seat.visual(
        Box((0.480, 0.320, 0.070)),
        origin=Origin(xyz=(0.240, 0.0, 0.055)),
        material=vinyl,
        name="seat_pad",
    )
    seat.visual(
        Box((0.440, 0.012, 0.008)),
        origin=Origin(xyz=(0.245, 0.0, 0.094)),
        material=seam,
        name="seat_center_seam",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.024, length=0.320),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="backrest_hinge_barrel",
    )
    backrest.visual(
        Box((0.795, 0.070, 0.034)),
        origin=Origin(xyz=(0.405, 0.0, 0.018)),
        material=steel,
        name="backrest_bracket",
    )
    backrest.visual(
        Box((0.820, 0.320, 0.070)),
        origin=Origin(xyz=(0.410, 0.0, 0.055)),
        material=vinyl,
        name="backrest_pad",
    )
    for i, x in enumerate((0.25, 0.55, 0.74)):
        backrest.visual(
            Box((0.018, 0.300, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.094)),
            material=seam,
            name=f"backrest_seam_{i}",
        )
    backrest.visual(
        Box((0.140, 0.065, 0.052)),
        origin=Origin(xyz=(0.620, 0.0, -0.020)),
        material=steel,
        name="ladder_catch",
    )

    ladder = model.part("ladder")
    ladder.visual(
        Cylinder(radius=0.024, length=0.430),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin,
        name="ladder_pivot_barrel",
    )
    for side_name, arm_name, y in (
        ("ladder_side_0", "ladder_lower_arm_0", -0.220),
        ("ladder_side_1", "ladder_lower_arm_1", 0.220),
    ):
        ladder.visual(
            Box((0.110, 0.036, 0.038)),
            origin=Origin(xyz=(0.079, y, 0.016)),
            material=ladder_paint,
            name=arm_name,
        )
        ladder.visual(
            Box((0.040, 0.040, 0.650)),
            origin=Origin(xyz=(0.110, y, 0.338)),
            material=ladder_paint,
            name=side_name,
        )
    ladder.visual(
        Box((0.045, 0.475, 0.028)),
        origin=Origin(xyz=(0.110, 0.0, 0.145)),
        material=ladder_paint,
        name="ladder_rung_0",
    )
    ladder.visual(
        Box((0.045, 0.475, 0.028)),
        origin=Origin(xyz=(0.110, 0.0, 0.285)),
        material=ladder_paint,
        name="ladder_rung_1",
    )
    ladder.visual(
        Box((0.045, 0.475, 0.028)),
        origin=Origin(xyz=(0.110, 0.0, 0.425)),
        material=ladder_paint,
        name="ladder_rung_2",
    )
    ladder.visual(
        Box((0.045, 0.475, 0.028)),
        origin=Origin(xyz=(0.110, 0.0, 0.565)),
        material=ladder_paint,
        name="ladder_rung_3",
    )

    roller_parts = []
    for i, y in enumerate((-0.200, 0.200)):
        roller = model.part(f"roller_{i}")
        roller.visual(
            Cylinder(radius=0.072, length=0.190),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="foam_roller",
        )
        roller.visual(
            Cylinder(radius=0.030, length=0.012),
            origin=Origin(xyz=(0.0, -0.101, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=seam,
            name="roller_end_0",
        )
        roller.visual(
            Cylinder(radius=0.030, length=0.012),
            origin=Origin(xyz=(0.0, 0.101, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=seam,
            name="roller_end_1",
        )
        roller_parts.append((roller, y))

    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(-0.38, 0.0, 0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=0.0, upper=0.42),
        motion_properties=MotionProperties(damping=0.4, friction=0.1),
    )
    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.10, 0.0, 0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.9, lower=0.0, upper=1.18),
        motion_properties=MotionProperties(damping=0.5, friction=0.12),
    )
    model.articulation(
        "ladder_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=ladder,
        origin=Origin(xyz=(0.88, 0.0, 0.140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.1, lower=0.0, upper=0.95),
        motion_properties=MotionProperties(damping=0.25, friction=0.06),
    )
    for i, (roller, y) in enumerate(roller_parts):
        model.articulation(
            f"roller_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=roller,
            origin=Origin(xyz=(-0.92, y, 0.365)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=25.0),
            motion_properties=MotionProperties(damping=0.03, friction=0.01),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    ladder = object_model.get_part("ladder")
    roller_0 = object_model.get_part("roller_0")
    roller_1 = object_model.get_part("roller_1")
    seat_hinge = object_model.get_articulation("seat_hinge")
    backrest_hinge = object_model.get_articulation("backrest_hinge")
    ladder_pivot = object_model.get_articulation("ladder_pivot")

    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_hinge_pin_line",
        elem_b="seat_hinge_barrel",
        reason="The stationary hinge pin intentionally runs through the seat hinge barrel.",
    )
    ctx.expect_within(
        frame,
        seat,
        axes="xz",
        inner_elem="seat_hinge_pin_line",
        outer_elem="seat_hinge_barrel",
        margin=0.0,
        name="seat hinge pin is captured in barrel",
    )
    ctx.expect_overlap(
        frame,
        seat,
        axes="y",
        elem_a="seat_hinge_pin_line",
        elem_b="seat_hinge_barrel",
        min_overlap=0.30,
        name="seat hinge pin spans barrel width",
    )

    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="backrest_hinge_pin_line",
        elem_b="backrest_hinge_barrel",
        reason="The stationary hinge pin intentionally runs through the backrest hinge barrel.",
    )
    ctx.expect_within(
        frame,
        backrest,
        axes="xz",
        inner_elem="backrest_hinge_pin_line",
        outer_elem="backrest_hinge_barrel",
        margin=0.0,
        name="backrest hinge pin is captured in barrel",
    )
    ctx.expect_overlap(
        frame,
        backrest,
        axes="y",
        elem_a="backrest_hinge_pin_line",
        elem_b="backrest_hinge_barrel",
        min_overlap=0.30,
        name="backrest hinge pin spans barrel width",
    )

    ctx.allow_overlap(
        frame,
        ladder,
        elem_a="ladder_pivot_pin",
        elem_b="ladder_pivot_barrel",
        reason="The rear ladder pivots on a captured lower pin through its barrel.",
    )
    ctx.expect_within(
        frame,
        ladder,
        axes="xz",
        inner_elem="ladder_pivot_pin",
        outer_elem="ladder_pivot_barrel",
        margin=0.0,
        name="ladder pivot pin is captured in barrel",
    )
    ctx.expect_overlap(
        frame,
        ladder,
        axes="y",
        elem_a="ladder_pivot_pin",
        elem_b="ladder_pivot_barrel",
        min_overlap=0.40,
        name="ladder pivot pin spans barrel width",
    )

    for roller in (roller_0, roller_1):
        ctx.allow_overlap(
            frame,
            roller,
            elem_a="roller_axle",
            elem_b="foam_roller",
            reason="The fixed metal axle intentionally passes through the spinning foam roller core.",
        )
        ctx.expect_within(
            frame,
            roller,
            axes="xz",
            inner_elem="roller_axle",
            outer_elem="foam_roller",
            margin=0.0,
            name=f"{roller.name} axle lies inside roller core",
        )
        ctx.expect_overlap(
            frame,
            roller,
            axes="y",
            elem_a="roller_axle",
            elem_b="foam_roller",
            min_overlap=0.16,
            name=f"{roller.name} captured on shared axle",
        )
        for end_name in ("roller_end_0", "roller_end_1"):
            ctx.allow_overlap(
                frame,
                roller,
                elem_a="roller_axle",
                elem_b=end_name,
                reason="The shared axle intentionally passes through the roller end bushing.",
            )
            ctx.expect_within(
                frame,
                roller,
                axes="xz",
                inner_elem="roller_axle",
                outer_elem=end_name,
                margin=0.0,
                name=f"{roller.name} axle centered in {end_name}",
            )

    ctx.expect_gap(
        backrest,
        seat,
        axis="x",
        positive_elem="backrest_pad",
        negative_elem="seat_pad",
        max_gap=0.003,
        max_penetration=0.003,
        name="flat pads meet at seat junction",
    )
    ctx.expect_overlap(
        backrest,
        seat,
        axes="y",
        elem_a="backrest_pad",
        elem_b="seat_pad",
        min_overlap=0.28,
        name="seat and backrest pads share bench width",
    )
    ctx.expect_origin_distance(
        roller_0,
        roller_1,
        axes="xz",
        max_dist=0.001,
        name="rollers share the same axle line",
    )
    ctx.expect_origin_distance(
        roller_0,
        roller_1,
        axes="y",
        min_dist=0.35,
        max_dist=0.45,
        name="rollers are separated as a front pair",
    )

    flat_back = ctx.part_element_world_aabb(backrest, elem="backrest_pad")
    with ctx.pose({backrest_hinge: 1.0}):
        raised_back = ctx.part_element_world_aabb(backrest, elem="backrest_pad")
    ctx.check(
        "backrest rotates upward",
        flat_back is not None
        and raised_back is not None
        and raised_back[1][2] > flat_back[1][2] + 0.45,
        details=f"flat={flat_back}, raised={raised_back}",
    )

    flat_seat = ctx.part_element_world_aabb(seat, elem="seat_pad")
    with ctx.pose({seat_hinge: 0.36}):
        raised_seat = ctx.part_element_world_aabb(seat, elem="seat_pad")
    ctx.check(
        "seat pivots on front hinge",
        flat_seat is not None
        and raised_seat is not None
        and raised_seat[1][2] > flat_seat[1][2] + 0.12,
        details=f"flat={flat_seat}, raised={raised_seat}",
    )

    vertical_ladder = ctx.part_element_world_aabb(ladder, elem="ladder_side_0")
    with ctx.pose({ladder_pivot: 0.75}):
        rotated_ladder = ctx.part_element_world_aabb(ladder, elem="ladder_side_0")
    ctx.check(
        "rear ladder rotates on lower pivot",
        vertical_ladder is not None
        and rotated_ladder is not None
        and rotated_ladder[1][0] > vertical_ladder[1][0] + 0.25,
        details=f"vertical={vertical_ladder}, rotated={rotated_ladder}",
    )
    ctx.expect_gap(
        ladder,
        backrest,
        axis="x",
        positive_elem="ladder_rung_2",
        negative_elem="backrest_bracket",
        min_gap=0.05,
        name="support ladder sits behind backrest bracket",
    )

    return ctx.report()


object_model = build_object_model()
