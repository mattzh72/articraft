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
    model = ArticulatedObject(name="swing_arm_transfer_axis")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.23, 0.28, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    dark_casting = model.material("dark_casting", rgba=(0.08, 0.09, 0.10, 1.0))
    beam_blue = model.material("beam_blue", rgba=(0.05, 0.22, 0.48, 1.0))
    rail_black = model.material("rail_black", rgba=(0.015, 0.017, 0.018, 1.0))
    truck_yellow = model.material("truck_yellow", rgba=(0.95, 0.64, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.95, 0.80, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_casting,
        name="floor_plinth",
    )
    mast.visual(
        Cylinder(radius=0.16, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=painted_steel,
        name="central_column",
    )
    mast.visual(
        Cylinder(radius=0.225, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=machined_steel,
        name="lower_collar",
    )
    mast.visual(
        Cylinder(radius=0.235, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
        material=machined_steel,
        name="upper_collar",
    )
    mast.visual(
        Cylinder(radius=0.42, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=machined_steel,
        name="slew_stator",
    )
    for idx, (x, y, sx, sy) in enumerate(
        (
            (0.0, 0.205, 0.42, 0.045),
            (0.0, -0.205, 0.42, 0.045),
            (0.205, 0.0, 0.045, 0.42),
            (-0.205, 0.0, 0.045, 0.42),
        )
    ):
        mast.visual(
            Box((sx, sy, 0.32)),
            origin=Origin(xyz=(x, y, 0.245)),
            material=painted_steel,
            name=f"mast_gusset_{idx}",
        )
    for idx in range(8):
        angle = idx * math.tau / 8.0
        mast.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(0.40 * math.cos(angle), 0.40 * math.sin(angle), 1.019)),
            material=dark_casting,
            name=f"stator_bolt_{idx}",
        )

    beam = model.part("beam")
    beam.visual(
        Cylinder(radius=0.36, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_steel,
        name="slew_rotor",
    )
    beam.visual(
        Box((2.12, 0.42, 0.075)),
        origin=Origin(xyz=(1.25, 0.0, 0.055)),
        material=beam_blue,
        name="beam_floor",
    )
    for idx, y in enumerate((0.17, -0.17)):
        beam.visual(
            Box((2.00, 0.08, 0.13)),
            origin=Origin(xyz=(1.30, y, 0.13)),
            material=beam_blue,
            name=f"beam_wall_{idx}",
        )
        beam.visual(
            Box((1.72, 0.055, 0.16)),
            origin=Origin(xyz=(1.38, math.copysign(0.23, y), 0.03)),
            material=beam_blue,
            name=f"side_stiffener_{idx}",
        )
        beam.visual(
            Box((1.66, 0.065, 0.045)),
            origin=Origin(xyz=(1.285, math.copysign(0.16, y), 0.2175)),
            material=rail_black,
            name=f"rail_cover_{idx}",
        )
    for idx, x in enumerate((0.68, 1.13, 1.58, 2.03)):
        beam.visual(
            Box((0.07, 0.38, 0.08)),
            origin=Origin(xyz=(x, 0.0, -0.005)),
            material=beam_blue,
            name=f"cross_rib_{idx}",
        )
    beam.visual(
        Box((0.06, 0.32, 0.16)),
        origin=Origin(xyz=(0.40, 0.0, 0.20)),
        material=rail_black,
        name="inner_stop",
    )
    beam.visual(
        Box((0.06, 0.32, 0.16)),
        origin=Origin(xyz=(2.17, 0.0, 0.20)),
        material=rail_black,
        name="outer_stop",
    )
    beam.visual(
        Box((0.42, 0.30, 0.07)),
        origin=Origin(xyz=(0.18, 0.0, 0.115)),
        material=beam_blue,
        name="hub_bridge",
    )
    for idx in range(10):
        angle = idx * math.tau / 10.0
        beam.visual(
            Cylinder(radius=0.012, length=0.016),
            origin=Origin(xyz=(0.27 * math.cos(angle), 0.27 * math.sin(angle), 0.048)),
            material=dark_casting,
            name=f"rotor_bolt_{idx}",
        )

    truck = model.part("truck")
    truck.visual(
        Box((0.42, 0.34, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=truck_yellow,
        name="truck_body",
    )
    truck.visual(
        Box((0.36, 0.16, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=machined_steel,
        name="guide_tongue",
    )
    truck.visual(
        Box((0.30, 0.08, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, 0.2175)),
        material=machined_steel,
        name="tongue_web",
    )
    for idx, y in enumerate((0.155, -0.155)):
        truck.visual(
            Box((0.38, 0.055, 0.020)),
            origin=Origin(xyz=(0.0, y, 0.25)),
            material=machined_steel,
            name=f"pad_{idx}",
        )
        for roller_idx, x in enumerate((-0.12, 0.12)):
            truck.visual(
                Cylinder(radius=0.032, length=0.052),
                origin=Origin(
                    xyz=(x, math.copysign(0.196, y), 0.295),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=rubber,
                name=f"roller_{idx}_{roller_idx}",
            )
    truck.visual(
        Box((0.30, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.4175)),
        material=machined_steel,
        name="tool_plate",
    )
    truck.visual(
        Box((0.12, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=truck_yellow,
        name="cable_guard",
    )

    model.articulation(
        "mast_to_beam",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.75, lower=-2.25, upper=2.25),
    )
    model.articulation(
        "beam_to_truck",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=truck,
        origin=Origin(xyz=(0.65, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.45, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    beam = object_model.get_part("beam")
    truck = object_model.get_part("truck")
    swing = object_model.get_articulation("mast_to_beam")
    slide = object_model.get_articulation("beam_to_truck")

    ctx.expect_gap(
        beam,
        mast,
        axis="z",
        positive_elem="slew_rotor",
        negative_elem="slew_stator",
        max_gap=0.001,
        max_penetration=0.0,
        name="slew rotor seats on fixed stator",
    )
    for idx in (0, 1):
        ctx.expect_gap(
            truck,
            beam,
            axis="z",
            positive_elem=f"pad_{idx}",
            negative_elem=f"rail_cover_{idx}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"truck pad {idx} rides on rail cover",
        )
    ctx.expect_within(
        truck,
        beam,
        axes="y",
        inner_elem="guide_tongue",
        outer_elem="beam_floor",
        margin=0.0,
        name="guide tongue stays laterally inside beam channel",
    )
    ctx.expect_gap(
        truck,
        beam,
        axis="z",
        positive_elem="guide_tongue",
        negative_elem="beam_floor",
        min_gap=0.005,
        max_gap=0.012,
        name="guide tongue clears the channel floor",
    )
    ctx.expect_gap(
        truck,
        beam,
        axis="x",
        positive_elem="truck_body",
        negative_elem="inner_stop",
        min_gap=0.005,
        name="truck clears inner stop at home",
    )

    rest_pos = ctx.part_world_position(truck)
    with ctx.pose({slide: 1.05}):
        ctx.expect_within(
            truck,
            beam,
            axes="y",
            inner_elem="guide_tongue",
            outer_elem="beam_floor",
            margin=0.0,
            name="extended tongue remains in channel width",
        )
        ctx.expect_overlap(
            truck,
            beam,
            axes="x",
            elem_a="guide_tongue",
            elem_b="beam_floor",
            min_overlap=0.30,
            name="extended truck retains beam insertion",
        )
        ctx.expect_gap(
            beam,
            truck,
            axis="x",
            positive_elem="outer_stop",
            negative_elem="truck_body",
            min_gap=0.10,
            name="truck clears outer stop at full travel",
        )
        extended_pos = ctx.part_world_position(truck)
    ctx.check(
        "truck translates outward along beam",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 1.0,
        details=f"home={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({swing: 1.0}):
        swung_pos = ctx.part_world_position(truck)
    ctx.check(
        "beam swing carries truck around mast",
        rest_pos is not None and swung_pos is not None and swung_pos[1] > rest_pos[1] + 0.45,
        details=f"home={rest_pos}, swung={swung_pos}",
    )

    for angle in (-2.25, 2.25):
        with ctx.pose({swing: angle, slide: 1.05}):
            ctx.expect_gap(
                beam,
                mast,
                axis="z",
                positive_elem="beam_floor",
                negative_elem="slew_stator",
                min_gap=0.04,
                name=f"beam structure stays above stator at swing {angle}",
            )

    return ctx.report()


object_model = build_object_model()
