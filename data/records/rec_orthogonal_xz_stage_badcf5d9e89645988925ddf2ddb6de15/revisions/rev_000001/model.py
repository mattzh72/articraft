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
    model = ArticulatedObject(name="slide_lift_xz_stage")

    dark_anodized = Material("dark_anodized_aluminum", color=(0.08, 0.09, 0.10, 1.0))
    blue_anodized = Material("blue_anodized_carriage", color=(0.08, 0.20, 0.42, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.72, 0.74, 0.72, 1.0))
    black_rubber = Material("black_rubber_pad", color=(0.015, 0.015, 0.014, 1.0))
    stop_red = Material("red_limit_markers", color=(0.75, 0.06, 0.03, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.74, 0.26, 0.035)),
        origin=Origin(xyz=(0.37, 0.0, 0.0175)),
        material=dark_anodized,
        name="base_plate",
    )
    base.visual(
        Box((0.64, 0.025, 0.025)),
        origin=Origin(xyz=(0.37, 0.08, 0.0475)),
        material=brushed_steel,
        name="rail_y_pos",
    )
    base.visual(
        Box((0.64, 0.025, 0.025)),
        origin=Origin(xyz=(0.37, -0.08, 0.0475)),
        material=brushed_steel,
        name="rail_y_neg",
    )
    for x, name in ((0.035, "end_stop_0"), (0.705, "end_stop_1")):
        base.visual(
            Box((0.045, 0.24, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.065)),
            material=dark_anodized,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.006, length=0.66),
        origin=Origin(xyz=(0.37, 0.0, 0.063), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="center_leadscrew",
    )
    for x, name in ((0.095, "travel_marker_0"), (0.645, "travel_marker_1")):
        base.visual(
            Box((0.012, 0.040, 0.002)),
            origin=Origin(xyz=(x, 0.0, 0.036)),
            material=stop_red,
            name=name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.18, 0.17, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=blue_anodized,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.14, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, 0.08, -0.021)),
        material=dark_anodized,
        name="bearing_y_pos",
    )
    carriage.visual(
        Box((0.14, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, -0.08, -0.021)),
        material=dark_anodized,
        name="bearing_y_neg",
    )
    carriage.visual(
        Box((0.13, 0.13, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=blue_anodized,
        name="z_base_block",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.202),
        origin=Origin(xyz=(-0.035, 0.045, 0.148)),
        material=brushed_steel,
        name="guide_rod_0",
    )
    carriage.visual(
        Cylinder(radius=0.010, length=0.202),
        origin=Origin(xyz=(-0.035, -0.045, 0.148)),
        material=brushed_steel,
        name="guide_rod_1",
    )
    for y, name in ((0.045, "guide_collar_0"), (-0.045, "guide_collar_1")):
        carriage.visual(
            Box((0.040, 0.030, 0.018)),
            origin=Origin(xyz=(-0.035, y, 0.250)),
            material=dark_anodized,
            name=name,
        )

    top_pad = model.part("top_pad")
    top_pad.visual(
        Box((0.030, 0.035, 0.210)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=brushed_steel,
        name="lift_mast",
    )
    top_pad.visual(
        Box((0.13, 0.13, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.223)),
        material=black_rubber,
        name="work_pad",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.18, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.38),
    )
    model.articulation(
        "z_lift",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=top_pad,
        origin=Origin(xyz=(0.035, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.16),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    top_pad = object_model.get_part("top_pad")
    x_slide = object_model.get_articulation("x_slide")
    z_lift = object_model.get_articulation("z_lift")

    ctx.check(
        "orthogonal prismatic stage joints",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and z_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(z_lift.axis) == (0.0, 0.0, 1.0),
        details=f"x={x_slide.articulation_type}/{x_slide.axis}, z={z_lift.articulation_type}/{z_lift.axis}",
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="bearing_y_pos",
        negative_elem="rail_y_pos",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage bearing rides on base rail",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="bearing_y_pos",
        elem_b="rail_y_pos",
        min_overlap=0.12,
        name="x carriage retained on rail at home",
    )
    ctx.expect_within(
        top_pad,
        carriage,
        axes="xy",
        inner_elem="lift_mast",
        outer_elem="z_base_block",
        margin=0.001,
        name="lift mast centered in compact guide block",
    )
    ctx.expect_overlap(
        top_pad,
        carriage,
        axes="z",
        elem_a="lift_mast",
        elem_b="guide_rod_0",
        min_overlap=0.18,
        name="lift mast overlaps guide height at home",
    )

    x_home = ctx.part_world_position(carriage)
    z_home = ctx.part_world_position(top_pad)
    with ctx.pose({x_slide: 0.38, z_lift: 0.16}):
        x_extended = ctx.part_world_position(carriage)
        z_extended = ctx.part_world_position(top_pad)
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="bearing_y_pos",
            elem_b="rail_y_pos",
            min_overlap=0.12,
            name="x carriage retained on rail at full travel",
        )
        ctx.expect_overlap(
            top_pad,
            carriage,
            axes="z",
            elem_a="lift_mast",
            elem_b="guide_rod_0",
            min_overlap=0.030,
            name="z lift remains engaged at full height",
        )

    ctx.check(
        "x slide advances along horizontal base",
        x_home is not None and x_extended is not None and x_extended[0] > x_home[0] + 0.35,
        details=f"home={x_home}, extended={x_extended}",
    )
    ctx.check(
        "z lift raises the top pad",
        z_home is not None and z_extended is not None and z_extended[2] > z_home[2] + 0.14,
        details=f"home={z_home}, extended={z_extended}",
    )

    return ctx.report()


object_model = build_object_model()
