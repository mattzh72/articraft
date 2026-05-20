from __future__ import annotations

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
    model = ArticulatedObject(name="raised_guideway_slide")

    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    hardened_steel = model.material("hardened_steel", rgba=(0.34, 0.36, 0.37, 1.0))
    carriage_blue = model.material("blue_carriage", rgba=(0.08, 0.22, 0.72, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.015, 0.015, 0.014, 1.0))

    guideway = model.part("guideway")
    guideway.visual(
        Box((0.42, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=cast_iron,
        name="foot_plate",
    )
    guideway.visual(
        Box((0.16, 0.14, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=cast_iron,
        name="pedestal",
    )
    guideway.visual(
        Box((0.24, 0.19, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=cast_iron,
        name="top_cap",
    )
    guideway.visual(
        Box((0.98, 0.13, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=brushed_aluminum,
        name="elevated_beam",
    )
    guideway.visual(
        Box((0.86, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=hardened_steel,
        name="guide_spine",
    )
    for y in (-0.063, 0.063):
        guideway.visual(
            Box((0.86, 0.020, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.4325)),
            material=hardened_steel,
            name=f"side_rail_{0 if y < 0 else 1}",
        )
    for x in (-0.455, 0.455):
        guideway.visual(
            Box((0.035, 0.18, 0.10)),
            origin=Origin(xyz=(x, 0.0, 0.47)),
            material=black_oxide,
            name=f"end_stop_{0 if x < 0 else 1}",
        )
    for x in (-0.14, 0.14):
        for y in (-0.10, 0.10):
            guideway.visual(
                Cylinder(radius=0.013, length=0.008),
                origin=Origin(xyz=(x, y, 0.044)),
                material=black_oxide,
                name=f"base_bolt_{'n' if y > 0 else 's'}_{'w' if x < 0 else 'e'}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.18, 0.074, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=hardened_steel,
        name="bearing_shoe",
    )
    carriage.visual(
        Box((0.22, 0.18, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0505)),
        material=carriage_blue,
        name="saddle_block",
    )
    for y in (-0.065, 0.065):
        carriage.visual(
            Box((0.18, 0.025, 0.041)),
            origin=Origin(xyz=(0.0, y, 0.0075)),
            material=carriage_blue,
            name=f"side_lip_{0 if y < 0 else 1}",
        )
    for x in (-0.065, 0.065):
        for y in (-0.050, 0.050):
            carriage.visual(
                Cylinder(radius=0.010, length=0.008),
                origin=Origin(xyz=(x, y, 0.077)),
                material=black_oxide,
                name=f"cap_screw_{'n' if y > 0 else 's'}_{'w' if x < 0 else 'e'}",
            )

    model.articulation(
        "guideway_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guideway,
        child=carriage,
        # The carriage frame is on the bearing contact plane at the left travel stop.
        origin=Origin(xyz=(-0.29, 0.0, 0.46)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.58),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guideway = object_model.get_part("guideway")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("guideway_to_carriage")

    ctx.check(
        "carriage is the single prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.upper == 0.58,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            carriage,
            guideway,
            axis="z",
            positive_elem="bearing_shoe",
            negative_elem="guide_spine",
            max_gap=0.001,
            max_penetration=0.0,
            name="carriage shoe sits on the elevated guide",
        )
        ctx.expect_overlap(
            carriage,
            guideway,
            axes="xy",
            elem_a="bearing_shoe",
            elem_b="guide_spine",
            min_overlap=0.050,
            name="carriage rides over the guide footprint at lower stop",
        )
        lower_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: 0.58}):
        ctx.expect_gap(
            carriage,
            guideway,
            axis="z",
            positive_elem="bearing_shoe",
            negative_elem="guide_spine",
            max_gap=0.001,
            max_penetration=0.0,
            name="carriage stays seated on the guide at upper stop",
        )
        ctx.expect_overlap(
            carriage,
            guideway,
            axes="xy",
            elem_a="bearing_shoe",
            elem_b="guide_spine",
            min_overlap=0.050,
            name="carriage remains on the elevated guide at upper stop",
        )
        upper_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along the straight guide axis",
        lower_pos is not None
        and upper_pos is not None
        and upper_pos[0] > lower_pos[0] + 0.55
        and abs(upper_pos[1] - lower_pos[1]) < 1e-6
        and abs(upper_pos[2] - lower_pos[2]) < 1e-6,
        details=f"lower={lower_pos}, upper={upper_pos}",
    )

    return ctx.report()


object_model = build_object_model()
