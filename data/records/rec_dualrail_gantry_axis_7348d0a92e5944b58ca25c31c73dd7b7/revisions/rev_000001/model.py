from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_gantry")

    painted_base = model.material("painted_base", color=(0.12, 0.13, 0.14, 1.0))
    rail_steel = model.material("rail_steel", color=(0.72, 0.74, 0.72, 1.0))
    safety_yellow = model.material("safety_yellow", color=(0.95, 0.66, 0.10, 1.0))
    carriage_blue = model.material("carriage_blue", color=(0.05, 0.16, 0.32, 1.0))
    rubber_black = model.material("rubber_black", color=(0.02, 0.02, 0.018, 1.0))
    sensor_glass = model.material("sensor_glass", color=(0.10, 0.55, 0.48, 1.0))

    base = model.part("base")
    base.visual(
        Box((2.10, 1.36, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=painted_base,
        name="bed_plate",
    )
    for y, name in ((-0.52, "rail_0"), (0.52, "rail_1")):
        base.visual(
            Box((1.90, 0.08, 0.06)),
            origin=Origin(xyz=(0.0, y, 0.08)),
            material=rail_steel,
            name=name,
        )
        base.visual(
            Box((1.90, 0.16, 0.02)),
            origin=Origin(xyz=(0.0, y, 0.06)),
            material=painted_base,
            name=f"rail_pad_{name[-1]}",
        )
    for x, name in ((-0.86, "end_tie_0"), (0.86, "end_tie_1")):
        base.visual(
            Box((0.08, 1.16, 0.04)),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material=painted_base,
            name=name,
        )

    crossbeam = model.part("crossbeam")
    # I-beam-like traveling crossbeam spanning across the two base guides.
    crossbeam.visual(
        Box((0.16, 1.42, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.918)),
        material=safety_yellow,
        name="top_flange",
    )
    crossbeam.visual(
        Box((0.06, 1.42, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.862)),
        material=safety_yellow,
        name="beam_web",
    )
    crossbeam.visual(
        Box((0.16, 1.42, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.806)),
        material=safety_yellow,
        name="bottom_flange",
    )
    for y, suffix in ((-0.60, "0"), (0.60, "1")):
        crossbeam.visual(
            Box((0.18, 0.11, 0.66)),
            origin=Origin(xyz=(0.0, y, 0.49)),
            material=safety_yellow,
            name=f"end_support_{suffix}",
        )
        crossbeam.visual(
            Box((0.26, 0.18, 0.05)),
            origin=Origin(xyz=(0.0, 0.52 if y > 0 else -0.52, 0.17)),
            material=carriage_blue,
            name=f"guide_shoe_{suffix}",
        )
        for x, roller_suffix in ((-0.075, "a"), (0.075, "b")):
            crossbeam.visual(
                Cylinder(radius=0.025, length=0.075),
                origin=Origin(
                    xyz=(x, 0.52 if y > 0 else -0.52, 0.135),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=rubber_black,
                name=f"guide_roller_{suffix}_{roller_suffix}",
            )
        crossbeam.visual(
            Box((0.035, 0.20, 0.34)),
            origin=Origin(xyz=(0.09, y, 0.64)),
            material=safety_yellow,
            name=f"side_gusset_{suffix}",
        )

    center_truck = model.part("center_truck")
    center_truck.visual(
        Box((0.32, 0.22, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.999)),
        material=carriage_blue,
        name="top_bridge",
    )
    for x, suffix in ((-0.115, "0"), (0.115, "1")):
        center_truck.visual(
            Box((0.045, 0.22, 0.22)),
            origin=Origin(xyz=(x, 0.0, 0.875)),
            material=carriage_blue,
            name=f"side_cheek_{suffix}",
        )
    center_truck.visual(
        Box((0.20, 0.18, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.755)),
        material=carriage_blue,
        name="lower_saddle",
    )
    for x, suffix in ((-0.035, "0"), (0.035, "1")):
        center_truck.visual(
            Cylinder(radius=0.020, length=0.18),
            origin=Origin(xyz=(x, 0.0, 0.9555), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber_black,
            name=f"top_roller_{suffix}",
        )
    center_truck.visual(
        Box((0.12, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
        material=painted_base,
        name="sensor_body",
    )
    center_truck.visual(
        Cylinder(radius=0.035, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.58275)),
        material=sensor_glass,
        name="sensor_lens",
    )

    model.articulation(
        "base_to_crossbeam",
        ArticulationType.PRISMATIC,
        parent=base,
        child=crossbeam,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.45, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "crossbeam_to_truck",
        ArticulationType.PRISMATIC,
        parent=crossbeam,
        child=center_truck,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=-0.42, upper=0.42),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    crossbeam = object_model.get_part("crossbeam")
    center_truck = object_model.get_part("center_truck")
    base_slide = object_model.get_articulation("base_to_crossbeam")
    truck_slide = object_model.get_articulation("crossbeam_to_truck")

    ctx.expect_gap(
        crossbeam,
        base,
        axis="z",
        positive_elem="guide_roller_0_a",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="roller rests on guide",
    )
    ctx.expect_overlap(
        crossbeam,
        base,
        axes="xy",
        elem_a="guide_roller_0_a",
        elem_b="rail_0",
        min_overlap=0.02,
        name="roller stays over guide",
    )
    ctx.expect_gap(
        center_truck,
        crossbeam,
        axis="z",
        positive_elem="top_roller_0",
        negative_elem="top_flange",
        max_gap=0.001,
        max_penetration=0.0,
        name="truck roller rides beam",
    )

    rest_crossbeam_pos = ctx.part_world_position(crossbeam)
    rest_truck_pos = ctx.part_world_position(center_truck)
    with ctx.pose({base_slide: 0.50}):
        moved_crossbeam_pos = ctx.part_world_position(crossbeam)
        moved_truck_pos = ctx.part_world_position(center_truck)
    ctx.check(
        "crossbeam slides along base",
        rest_crossbeam_pos is not None
        and moved_crossbeam_pos is not None
        and moved_crossbeam_pos[0] > rest_crossbeam_pos[0] + 0.45,
        details=f"rest={rest_crossbeam_pos}, moved={moved_crossbeam_pos}",
    )
    ctx.check(
        "truck follows crossbeam",
        rest_truck_pos is not None
        and moved_truck_pos is not None
        and moved_truck_pos[0] > rest_truck_pos[0] + 0.45,
        details=f"rest={rest_truck_pos}, moved={moved_truck_pos}",
    )

    with ctx.pose({truck_slide: 0.30}):
        shifted_truck_pos = ctx.part_world_position(center_truck)
    ctx.check(
        "truck slides across beam",
        rest_truck_pos is not None
        and shifted_truck_pos is not None
        and shifted_truck_pos[1] > rest_truck_pos[1] + 0.25,
        details=f"rest={rest_truck_pos}, shifted={shifted_truck_pos}",
    )

    return ctx.report()


object_model = build_object_model()
