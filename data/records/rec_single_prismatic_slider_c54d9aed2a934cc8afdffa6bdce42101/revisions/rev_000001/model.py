from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_slide_module")

    aluminum = model.material("anodized_aluminum", rgba=(0.55, 0.58, 0.60, 1.0))
    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.10, 0.11, 0.12, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.01, 0.01, 0.012, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    rail_body = model.part("rail_body")
    rail_body.visual(
        Box((0.760, 0.140, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_aluminum,
        name="base_plate",
    )
    rail_body.visual(
        Box((0.640, 0.045, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=ground_steel,
        name="guide_rail",
    )
    rail_body.visual(
        Box((0.640, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.0195, 0.055)),
        material=ground_steel,
        name="race_strip_pos",
    )
    rail_body.visual(
        Box((0.640, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, -0.0195, 0.055)),
        material=ground_steel,
        name="race_strip_neg",
    )
    rail_body.visual(
        Box((0.035, 0.124, 0.060)),
        origin=Origin(xyz=(0.345, 0.0, 0.055)),
        material=aluminum,
        name="end_stop_pos",
    )
    rail_body.visual(
        Box((0.035, 0.124, 0.060)),
        origin=Origin(xyz=(-0.345, 0.0, 0.055)),
        material=aluminum,
        name="end_stop_neg",
    )
    rail_body.visual(
        Box((0.010, 0.070, 0.030)),
        origin=Origin(xyz=(0.322, 0.0, 0.071)),
        material=rubber,
        name="stop_bumper_pos",
    )
    rail_body.visual(
        Box((0.010, 0.070, 0.030)),
        origin=Origin(xyz=(-0.322, 0.0, 0.071)),
        material=rubber,
        name="stop_bumper_neg",
    )
    for i, x in enumerate((-0.275, 0.275)):
        for j, y in enumerate((-0.045, 0.045)):
            rail_body.visual(
                Cylinder(radius=0.007, length=0.004),
                origin=Origin(xyz=(x, y, 0.027)),
                material=black_oxide,
                name=f"base_screw_{i}_{j}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.220, 0.105, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=aluminum,
        name="saddle_block",
    )
    carriage.visual(
        Box((0.220, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, 0.035, 0.0585)),
        material=aluminum,
        name="side_skirt_pos",
    )
    carriage.visual(
        Box((0.220, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, -0.035, 0.0585)),
        material=aluminum,
        name="side_skirt_neg",
    )
    carriage.visual(
        Box((0.180, 0.004, 0.032)),
        origin=Origin(xyz=(0.0, 0.025, 0.054)),
        material=black_oxide,
        name="bearing_liner_pos",
    )
    carriage.visual(
        Box((0.180, 0.004, 0.032)),
        origin=Origin(xyz=(0.0, -0.025, 0.054)),
        material=black_oxide,
        name="bearing_liner_neg",
    )
    carriage.visual(
        Box((0.200, 0.095, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=dark_aluminum,
        name="tooling_plate",
    )
    for i, x in enumerate((-0.070, 0.070)):
        for j, y in enumerate((-0.030, 0.030)):
            carriage.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(x, y, 0.125)),
                material=black_oxide,
                name=f"plate_screw_{i}_{j}",
            )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_body,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=-0.200, upper=0.200),
        motion_properties=MotionProperties(damping=2.0, friction=0.3),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_body = object_model.get_part("rail_body")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("rail_to_carriage")

    ctx.check(
        "single carriage joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint_type={slide.articulation_type}",
    )
    limits = slide.motion_limits
    ctx.check(
        "carriage travel is centered and limited",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < -0.15
        and limits.upper > 0.15,
        details=f"limits={limits}",
    )

    ctx.expect_gap(
        carriage,
        rail_body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="saddle_block",
        negative_elem="guide_rail",
        name="saddle rides on guide rail top",
    )
    ctx.expect_overlap(
        carriage,
        rail_body,
        axes="xz",
        min_overlap=0.025,
        elem_a="side_skirt_pos",
        elem_b="guide_rail",
        name="positive skirt overlaps guide length and height",
    )
    ctx.expect_overlap(
        carriage,
        rail_body,
        axes="xz",
        min_overlap=0.025,
        elem_a="side_skirt_neg",
        elem_b="guide_rail",
        name="negative skirt overlaps guide length and height",
    )
    ctx.expect_gap(
        carriage,
        rail_body,
        axis="y",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="bearing_liner_pos",
        negative_elem="guide_rail",
        name="positive bearing liner has side clearance",
    )
    ctx.expect_gap(
        rail_body,
        carriage,
        axis="y",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="guide_rail",
        negative_elem="bearing_liner_neg",
        name="negative bearing liner has side clearance",
    )
    ctx.expect_gap(
        carriage,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="tooling_plate",
        negative_elem="saddle_block",
        name="tooling plate is seated on carriage",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: limits.upper}):
        ctx.expect_overlap(
            carriage,
            rail_body,
            axes="x",
            min_overlap=0.080,
            elem_a="bearing_liner_pos",
            elem_b="guide_rail",
            name="extended carriage remains on guide",
        )
        ctx.expect_gap(
            rail_body,
            carriage,
            axis="x",
            min_gap=0.0,
            max_gap=0.020,
            positive_elem="stop_bumper_pos",
            negative_elem="saddle_block",
            name="positive end stop limits travel",
        )
        extended_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: limits.lower}):
        ctx.expect_overlap(
            carriage,
            rail_body,
            axes="x",
            min_overlap=0.080,
            elem_a="bearing_liner_neg",
            elem_b="guide_rail",
            name="retracted carriage remains on guide",
        )
        ctx.expect_gap(
            carriage,
            rail_body,
            axis="x",
            min_gap=0.0,
            max_gap=0.020,
            positive_elem="saddle_block",
            negative_elem="stop_bumper_neg",
            name="negative end stop limits travel",
        )
        retracted_pos = ctx.part_world_position(carriage)

    ctx.check(
        "positive slide motion follows rail axis",
        rest_pos is not None
        and extended_pos is not None
        and retracted_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.15
        and retracted_pos[0] < rest_pos[0] - 0.15,
        details=f"rest={rest_pos}, extended={extended_pos}, retracted={retracted_pos}",
    )

    return ctx.report()


object_model = build_object_model()
