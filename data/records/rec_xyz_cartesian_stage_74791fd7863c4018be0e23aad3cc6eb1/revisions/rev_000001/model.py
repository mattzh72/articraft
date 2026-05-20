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
    model = ArticulatedObject(name="benchtop_xyz_positioning_stage")

    satin_aluminum = Material("satin_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_anodized = Material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    ground_steel = Material("ground_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    black_oxide = Material("black_oxide", rgba=(0.015, 0.015, 0.018, 1.0))
    brass = Material("brass", rgba=(0.75, 0.56, 0.22, 1.0))

    # Fixed long X-axis bed: wide enough for the bearing blocks, with separate
    # rails, a central screw/cover strip, bearing supports, and hard stops.
    x_base = model.part("x_base")
    x_base.visual(
        Box((0.72, 0.18, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_anodized,
        name="x_bed_plate",
    )
    x_base.visual(
        Box((0.620, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.055, 0.052)),
        material=ground_steel,
        name="x_rail_0",
    )
    x_base.visual(
        Box((0.620, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.055, 0.052)),
        material=ground_steel,
        name="x_rail_1",
    )
    x_base.visual(
        Box((0.580, 0.025, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=satin_aluminum,
        name="x_center_cover",
    )
    x_base.visual(
        Cylinder(radius=0.006, length=0.550),
        origin=Origin(xyz=(0.0, 0.0, 0.078), rpy=(0.0, pi / 2.0, 0.0)),
        material=ground_steel,
        name="x_lead_screw",
    )
    for x, name in [(-0.286, "x_bearing_minus"), (0.286, "x_bearing_plus")]:
        x_base.visual(
            Box((0.034, 0.050, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=satin_aluminum,
            name=name,
        )
    x_base.visual(
        Box((0.030, 0.170, 0.055)),
        origin=Origin(xyz=(-0.335, 0.0, 0.0675)),
        material=black_oxide,
        name="x_stop_minus",
    )
    x_base.visual(
        Box((0.030, 0.170, 0.055)),
        origin=Origin(xyz=(0.335, 0.0, 0.0675)),
        material=black_oxide,
        name="x_stop_plus",
    )

    # X moving saddle.  Its lower bearing blocks sit on the fixed X rails; the
    # crosswise table and rails mounted above it form the fixed bed for Y.
    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        Box((0.055, 0.028, 0.026)),
        origin=Origin(xyz=(-0.065, -0.055, 0.013)),
        material=satin_aluminum,
        name="x_block_front_0",
    )
    x_carriage.visual(
        Box((0.055, 0.028, 0.026)),
        origin=Origin(xyz=(-0.065, 0.055, 0.013)),
        material=satin_aluminum,
        name="x_block_front_1",
    )
    x_carriage.visual(
        Box((0.055, 0.028, 0.026)),
        origin=Origin(xyz=(0.065, -0.055, 0.013)),
        material=satin_aluminum,
        name="x_block_rear_0",
    )
    x_carriage.visual(
        Box((0.055, 0.028, 0.026)),
        origin=Origin(xyz=(0.065, 0.055, 0.013)),
        material=satin_aluminum,
        name="x_block_rear_1",
    )
    x_carriage.visual(
        Box((0.230, 0.155, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=satin_aluminum,
        name="x_saddle",
    )
    x_carriage.visual(
        Box((0.190, 0.320, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_anodized,
        name="y_bed_plate",
    )
    x_carriage.visual(
        Box((0.020, 0.280, 0.018)),
        origin=Origin(xyz=(-0.055, 0.0, 0.069)),
        material=ground_steel,
        name="y_rail_0",
    )
    x_carriage.visual(
        Box((0.020, 0.280, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, 0.069)),
        material=ground_steel,
        name="y_rail_1",
    )
    x_carriage.visual(
        Box((0.030, 0.250, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        material=satin_aluminum,
        name="y_center_cover",
    )
    x_carriage.visual(
        Box((0.180, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, -0.151, 0.0775)),
        material=black_oxide,
        name="y_stop_minus",
    )
    x_carriage.visual(
        Box((0.180, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.151, 0.0775)),
        material=black_oxide,
        name="y_stop_plus",
    )

    # Y carriage: a compact saddle riding on the cross rails, carrying a rigid
    # upright with exposed vertical guide rails for the Z slide.
    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        Box((0.036, 0.040, 0.024)),
        origin=Origin(xyz=(-0.055, -0.032, 0.012)),
        material=satin_aluminum,
        name="y_block_0_minus",
    )
    y_carriage.visual(
        Box((0.036, 0.040, 0.024)),
        origin=Origin(xyz=(-0.055, 0.032, 0.012)),
        material=satin_aluminum,
        name="y_block_0_plus",
    )
    y_carriage.visual(
        Box((0.036, 0.040, 0.024)),
        origin=Origin(xyz=(0.055, -0.032, 0.012)),
        material=satin_aluminum,
        name="y_block_1_minus",
    )
    y_carriage.visual(
        Box((0.036, 0.040, 0.024)),
        origin=Origin(xyz=(0.055, 0.032, 0.012)),
        material=satin_aluminum,
        name="y_block_1_plus",
    )
    y_carriage.visual(
        Box((0.170, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=satin_aluminum,
        name="y_table",
    )
    y_carriage.visual(
        Box((0.160, 0.100, 0.025)),
        origin=Origin(xyz=(0.0, 0.025, 0.0545)),
        material=dark_anodized,
        name="z_column_foot",
    )
    y_carriage.visual(
        Box((0.130, 0.035, 0.420)),
        origin=Origin(xyz=(0.0, 0.045, 0.277)),
        material=dark_anodized,
        name="z_column_backbone",
    )
    y_carriage.visual(
        Box((0.018, 0.014, 0.360)),
        origin=Origin(xyz=(-0.042, 0.0205, 0.252)),
        material=ground_steel,
        name="z_rail_0",
    )
    y_carriage.visual(
        Box((0.018, 0.014, 0.360)),
        origin=Origin(xyz=(0.042, 0.0205, 0.252)),
        material=ground_steel,
        name="z_rail_1",
    )
    y_carriage.visual(
        Box((0.028, 0.006, 0.290)),
        origin=Origin(xyz=(0.0, 0.0145, 0.252)),
        material=satin_aluminum,
        name="z_center_cover",
    )
    y_carriage.visual(
        Box((0.120, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.0205, 0.080)),
        material=black_oxide,
        name="z_lower_stop",
    )
    y_carriage.visual(
        Box((0.120, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.0205, 0.440)),
        material=black_oxide,
        name="z_top_stop",
    )
    y_carriage.visual(
        Cylinder(radius=0.0045, length=0.330),
        origin=Origin(xyz=(0.0, 0.008, 0.252), rpy=(0.0, 0.0, 0.0)),
        material=ground_steel,
        name="z_lift_screw",
    )
    y_carriage.visual(
        Box((0.040, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, 0.012, 0.080)),
        material=brass,
        name="z_lower_nut_support",
    )

    # Z slide and square tool plate.  The bearing shoes touch the vertical rails
    # along Y, while the front carrier and square face plate remain clear.
    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((0.026, 0.014, 0.055)),
        origin=Origin(xyz=(-0.042, 0.0065, 0.045)),
        material=satin_aluminum,
        name="z_shoe_0_lower",
    )
    z_slide.visual(
        Box((0.026, 0.014, 0.055)),
        origin=Origin(xyz=(-0.042, 0.0065, 0.125)),
        material=satin_aluminum,
        name="z_shoe_0_upper",
    )
    z_slide.visual(
        Box((0.026, 0.014, 0.055)),
        origin=Origin(xyz=(0.042, 0.0065, 0.045)),
        material=satin_aluminum,
        name="z_shoe_1_lower",
    )
    z_slide.visual(
        Box((0.026, 0.014, 0.055)),
        origin=Origin(xyz=(0.042, 0.0065, 0.125)),
        material=satin_aluminum,
        name="z_shoe_1_upper",
    )
    z_slide.visual(
        Box((0.120, 0.012, 0.180)),
        origin=Origin(xyz=(0.0, -0.0065, 0.095)),
        material=dark_anodized,
        name="z_front_carrier",
    )
    z_slide.visual(
        Box((0.120, 0.012, 0.120)),
        origin=Origin(xyz=(0.0, -0.0185, 0.095)),
        material=satin_aluminum,
        name="tool_plate",
    )
    for x in (-0.040, 0.040):
        for z in (0.065, 0.125):
            z_slide.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, -0.0265, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=black_oxide,
                name=f"tool_bolt_{'neg' if x < 0 else 'pos'}_{'low' if z < 0.09 else 'high'}",
            )

    model.articulation(
        "x_travel",
        ArticulationType.PRISMATIC,
        parent=x_base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.18, lower=-0.170, upper=0.170),
    )
    model.articulation(
        "y_travel",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=0.16, lower=-0.075, upper=0.075),
    )
    model.articulation(
        "z_travel",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.10, lower=0.0, upper=0.120),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    x_base = object_model.get_part("x_base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    z_slide = object_model.get_part("z_slide")
    x_travel = object_model.get_articulation("x_travel")
    y_travel = object_model.get_articulation("y_travel")
    z_travel = object_model.get_articulation("z_travel")

    # Each stage visibly rides on a retained precision guide rather than
    # hovering above it.
    ctx.expect_gap(
        x_carriage,
        x_base,
        axis="z",
        positive_elem="x_block_front_0",
        negative_elem="x_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="x bearing block sits on x rail",
    )
    ctx.expect_overlap(
        x_carriage,
        x_base,
        axes="xy",
        elem_a="x_block_front_0",
        elem_b="x_rail_0",
        min_overlap=0.010,
        name="x bearing block overlaps x rail footprint",
    )
    ctx.expect_gap(
        y_carriage,
        x_carriage,
        axis="z",
        positive_elem="y_block_0_minus",
        negative_elem="y_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="y bearing block sits on y rail",
    )
    ctx.expect_overlap(
        y_carriage,
        x_carriage,
        axes="xy",
        elem_a="y_block_0_minus",
        elem_b="y_rail_0",
        min_overlap=0.010,
        name="y bearing block overlaps y rail footprint",
    )
    ctx.expect_gap(
        y_carriage,
        z_slide,
        axis="y",
        positive_elem="z_rail_0",
        negative_elem="z_shoe_0_lower",
        max_gap=0.001,
        max_penetration=0.0,
        name="z shoe bears against vertical rail",
    )
    ctx.expect_overlap(
        z_slide,
        y_carriage,
        axes="xz",
        elem_a="z_shoe_0_lower",
        elem_b="z_rail_0",
        min_overlap=0.010,
        name="z shoe remains on vertical rail",
    )

    rest_x = ctx.part_world_position(x_carriage)
    rest_y = ctx.part_world_position(y_carriage)
    rest_z = ctx.part_world_position(z_slide)

    with ctx.pose({x_travel: 0.170}):
        ctx.expect_gap(
            x_base,
            x_carriage,
            axis="x",
            positive_elem="x_stop_plus",
            negative_elem="x_saddle",
            min_gap=0.020,
            name="x positive stop has clearance at full travel",
        )
        x_extended = ctx.part_world_position(x_carriage)
    with ctx.pose({x_travel: -0.170}):
        ctx.expect_gap(
            x_carriage,
            x_base,
            axis="x",
            positive_elem="x_saddle",
            negative_elem="x_stop_minus",
            min_gap=0.020,
            name="x negative stop has clearance at full travel",
        )

    with ctx.pose({y_travel: 0.075}):
        ctx.expect_gap(
            x_carriage,
            y_carriage,
            axis="y",
            positive_elem="y_stop_plus",
            negative_elem="y_table",
            min_gap=0.005,
            name="y positive stop has clearance at full travel",
        )
        y_extended = ctx.part_world_position(y_carriage)
    with ctx.pose({y_travel: -0.075}):
        ctx.expect_gap(
            y_carriage,
            x_carriage,
            axis="y",
            positive_elem="y_table",
            negative_elem="y_stop_minus",
            min_gap=0.005,
            name="y negative stop has clearance at full travel",
        )

    with ctx.pose({z_travel: 0.120}):
        ctx.expect_gap(
            y_carriage,
            z_slide,
            axis="z",
            positive_elem="z_top_stop",
            negative_elem="tool_plate",
            min_gap=0.020,
            name="z top stop remains clear at full travel",
        )
        ctx.expect_overlap(
            z_slide,
            y_carriage,
            axes="xz",
            elem_a="z_shoe_0_upper",
            elem_b="z_rail_0",
            min_overlap=0.010,
            name="z upper shoe remains retained at full height",
        )
        z_extended = ctx.part_world_position(z_slide)

    ctx.check(
        "x joint moves along x",
        rest_x is not None and x_extended is not None and x_extended[0] > rest_x[0] + 0.15,
        details=f"rest={rest_x}, extended={x_extended}",
    )
    ctx.check(
        "y joint moves along y",
        rest_y is not None and y_extended is not None and y_extended[1] > rest_y[1] + 0.06,
        details=f"rest={rest_y}, extended={y_extended}",
    )
    ctx.check(
        "z joint moves upward",
        rest_z is not None and z_extended is not None and z_extended[2] > rest_z[2] + 0.10,
        details=f"rest={rest_z}, extended={z_extended}",
    )

    return ctx.report()


object_model = build_object_model()
