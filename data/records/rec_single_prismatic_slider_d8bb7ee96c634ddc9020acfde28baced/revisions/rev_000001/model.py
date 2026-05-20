from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="linear_slide_module")

    dark_aluminum = Material("dark_anodized_aluminum", rgba=(0.05, 0.055, 0.06, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    carriage_aluminum = Material("brushed_carriage_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    black_plastic = Material("black_plastic", rgba=(0.015, 0.015, 0.017, 1.0))

    guide_rail = model.part("guide_rail")
    guide_rail.visual(
        Box((0.62, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_aluminum,
        name="base_plate",
    )
    guide_rail.visual(
        Box((0.56, 0.055, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=satin_steel,
        name="center_rail",
    )
    guide_rail.visual(
        Box((0.56, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=satin_steel,
        name="rail_crown",
    )
    for y, name in ((0.033, "race_pos"), (-0.033, "race_neg")):
        guide_rail.visual(
            Cylinder(radius=0.006, length=0.56),
            origin=Origin(xyz=(0.0, y, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
            material=satin_steel,
            name=name,
        )
    for x, name in ((-0.295, "end_stop_0"), (0.295, "end_stop_1")):
        guide_rail.visual(
            Box((0.025, 0.14, 0.056)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=dark_aluminum,
            name=name,
        )
    for x in (-0.22, 0.22):
        for y in (-0.060, 0.060):
            guide_rail.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(x, y, 0.014)),
                material=black_plastic,
                name=f"base_screw_{x:+.2f}_{y:+.2f}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.14, 0.12, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_aluminum,
        name="top_bridge",
    )
    carriage.visual(
        Box((0.14, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, 0.051, -0.0425)),
        material=carriage_aluminum,
        name="side_cheek_pos",
    )
    carriage.visual(
        Box((0.14, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, -0.051, -0.0425)),
        material=carriage_aluminum,
        name="side_cheek_neg",
    )
    carriage.visual(
        Box((0.16, 0.095, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0235)),
        material=carriage_aluminum,
        name="mount_plate",
    )
    for x in (-0.055, 0.055):
        for y in (-0.032, 0.032):
            carriage.visual(
                Cylinder(radius=0.0075, length=0.004),
                origin=Origin(xyz=(x, y, 0.0315)),
                material=black_plastic,
                name=f"plate_screw_{x:+.3f}_{y:+.3f}",
            )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_rail,
        child=carriage,
        origin=Origin(xyz=(-0.18, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.32),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_rail = object_model.get_part("guide_rail")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("rail_to_carriage")

    ctx.expect_gap(
        carriage,
        guide_rail,
        axis="z",
        positive_elem="top_bridge",
        negative_elem="rail_crown",
        min_gap=0.010,
        max_gap=0.020,
        name="bridge clears rail crown",
    )
    ctx.expect_gap(
        carriage,
        guide_rail,
        axis="y",
        positive_elem="side_cheek_pos",
        negative_elem="race_pos",
        max_penetration=1e-6,
        max_gap=0.0005,
        name="positive cheek clears rail race",
    )
    ctx.expect_gap(
        guide_rail,
        carriage,
        axis="y",
        positive_elem="race_neg",
        negative_elem="side_cheek_neg",
        max_penetration=1e-6,
        max_gap=0.0005,
        name="negative cheek clears rail race",
    )
    ctx.expect_overlap(
        carriage,
        guide_rail,
        axes="x",
        elem_a="side_cheek_pos",
        elem_b="center_rail",
        min_overlap=0.12,
        name="carriage remains seated on rail at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.32}):
        ctx.expect_overlap(
            carriage,
            guide_rail,
            axes="x",
            elem_a="side_cheek_pos",
            elem_b="center_rail",
            min_overlap=0.12,
            name="carriage remains seated on rail at full travel",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic joint moves along rail",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.30
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
