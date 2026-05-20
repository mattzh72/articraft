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
    model = ArticulatedObject(name="pedestal_indexer")

    dark_cast = model.material("dark_cast_iron", color=(0.08, 0.09, 0.10, 1.0))
    blue_paint = model.material("machine_blue", color=(0.05, 0.18, 0.34, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.68, 0.70, 0.69, 1.0))
    black_rubber = model.material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))
    safety_orange = model.material("carriage_orange", color=(0.95, 0.42, 0.08, 1.0))
    bolt_black = model.material("blackened_bolts", color=(0.025, 0.025, 0.025, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.43, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_cast,
        name="floor_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.25, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=dark_cast,
        name="pedestal_drum",
    )
    pedestal.visual(
        Cylinder(radius=0.31, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1925)),
        material=brushed_steel,
        name="bearing_race",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=bolt_black,
        name="center_spindle_socket",
    )
    for index, (x, y) in enumerate(
        ((0.30, 0.00), (-0.30, 0.00), (0.00, 0.30), (0.00, -0.30))
    ):
        pedestal.visual(
            Cylinder(radius=0.035, length=0.018),
            origin=Origin(xyz=(x, y, 0.089)),
            material=black_rubber,
            name=f"floor_pad_{index}",
        )

    turntable_mast = model.part("turntable_mast")
    turntable_mast.visual(
        Cylinder(radius=0.35, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=blue_paint,
        name="turntable_disk",
    )
    turntable_mast.visual(
        Cylinder(radius=0.27, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=brushed_steel,
        name="top_wear_ring",
    )
    for index in range(8):
        angle = index * math.tau / 8.0
        turntable_mast.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(0.245 * math.cos(angle), 0.245 * math.sin(angle), 0.095)),
            material=bolt_black,
            name=f"table_bolt_{index}",
        )
    turntable_mast.visual(
        Box((0.34, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.015, 0.0975)),
        material=blue_paint,
        name="mast_foot_plate",
    )
    turntable_mast.visual(
        Box((0.16, 0.10, 1.02)),
        origin=Origin(xyz=(0.0, 0.03, 0.58)),
        material=blue_paint,
        name="mast_box",
    )
    turntable_mast.visual(
        Box((0.23, 0.085, 0.045)),
        origin=Origin(xyz=(0.0, -0.065, 0.17)),
        material=blue_paint,
        name="lower_rail_bridge",
    )
    turntable_mast.visual(
        Box((0.23, 0.085, 0.045)),
        origin=Origin(xyz=(0.0, -0.065, 1.025)),
        material=blue_paint,
        name="upper_rail_bridge",
    )
    turntable_mast.visual(
        Box((0.19, 0.08, 0.032)),
        origin=Origin(xyz=(0.0, -0.06, 0.57)),
        material=blue_paint,
        name="mid_rail_tie",
    )
    for rail_name, x in (("guide_rail_0", -0.075), ("guide_rail_1", 0.075)):
        turntable_mast.visual(
            Cylinder(radius=0.014, length=0.93),
            origin=Origin(xyz=(x, -0.095, 0.58)),
            material=brushed_steel,
            name=rail_name,
        )
    for index, x in enumerate((-0.115, 0.115)):
        turntable_mast.visual(
            Box((0.035, 0.12, 0.33)),
            origin=Origin(xyz=(x, 0.095, 0.235), rpy=(0.0, 0.23 if x < 0.0 else -0.23, 0.0)),
            material=blue_paint,
            name=f"mast_gusset_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.34, 0.03, 0.24)),
        origin=Origin(xyz=(0.0, -0.07, 0.0)),
        material=safety_orange,
        name="front_plate",
    )
    carriage.visual(
        Box((0.18, 0.035, 0.12)),
        origin=Origin(xyz=(0.0, -0.101, 0.005)),
        material=dark_cast,
        name="tool_mount_pad",
    )
    carriage.visual(
        Box((0.11, 0.13, 0.16)),
        origin=Origin(xyz=(0.0, 0.01, 0.0)),
        material=dark_cast,
        name="slide_shoe",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(0.0, -0.124, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_black,
        name="tool_socket",
    )

    rail_centers = (-0.075, 0.075)
    bearing_z = (-0.072, 0.072)
    for rail_index, x in enumerate(rail_centers):
        for station_index, z in enumerate(bearing_z):
            carriage.visual(
                Box((0.012, 0.095, 0.067)),
                origin=Origin(xyz=(x - 0.020, -0.004, z)),
                material=safety_orange,
                name=f"bearing_cheek_{rail_index}_{station_index}_0",
            )
            carriage.visual(
                Box((0.012, 0.095, 0.067)),
                origin=Origin(xyz=(x + 0.020, -0.004, z)),
                material=safety_orange,
                name=f"bearing_cheek_{rail_index}_{station_index}_1",
            )
            carriage.visual(
                Box((0.068, 0.022, 0.067)),
                origin=Origin(xyz=(x, -0.052, z)),
                material=safety_orange,
                name=f"bearing_front_{rail_index}_{station_index}",
            )
    carriage.visual(
        Box((0.30, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.046, 0.126)),
        material=safety_orange,
        name="top_tie_bar",
    )
    carriage.visual(
        Box((0.30, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.046, -0.126)),
        material=safety_orange,
        name="bottom_tie_bar",
    )

    model.articulation(
        "pedestal_to_turntable",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turntable_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=turntable_mast,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.095, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.48),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turntable_mast = object_model.get_part("turntable_mast")
    carriage = object_model.get_part("carriage")
    rotate = object_model.get_articulation("pedestal_to_turntable")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.expect_gap(
        turntable_mast,
        pedestal,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="bearing_race",
        max_gap=0.001,
        max_penetration=0.000001,
        name="turntable sits on bearing race",
    )
    ctx.expect_overlap(
        turntable_mast,
        pedestal,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="bearing_race",
        min_overlap=0.25,
        name="round turntable is concentric with bearing",
    )
    ctx.expect_overlap(
        carriage,
        turntable_mast,
        axes="z",
        elem_a="front_plate",
        elem_b="guide_rail_0",
        min_overlap=0.20,
        name="carriage spans a vertical guide rail at rest",
    )
    ctx.expect_contact(
        carriage,
        turntable_mast,
        elem_a="slide_shoe",
        elem_b="mast_box",
        contact_tol=0.00001,
        name="carriage slide shoe rides mast face",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.48}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            turntable_mast,
            axes="z",
            elem_a="front_plate",
            elem_b="guide_rail_0",
            min_overlap=0.20,
            name="raised carriage remains captured on guide rail",
        )
        ctx.expect_contact(
            carriage,
            turntable_mast,
            elem_a="slide_shoe",
            elem_b="mast_box",
            contact_tol=0.00001,
            name="raised slide shoe remains on mast face",
        )
    ctx.check(
        "upper carriage moves upward on prismatic axis",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.45,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    before_rotate = ctx.part_world_position(carriage)
    with ctx.pose({rotate: math.pi / 2.0}):
        after_rotate = ctx.part_world_position(carriage)
    ctx.check(
        "turntable rotation carries mast and carriage",
        before_rotate is not None
        and after_rotate is not None
        and abs(after_rotate[0] - 0.095) < 0.01
        and abs(after_rotate[1]) < 0.01,
        details=f"before={before_rotate}, after={after_rotate}",
    )

    return ctx.report()


object_model = build_object_model()
