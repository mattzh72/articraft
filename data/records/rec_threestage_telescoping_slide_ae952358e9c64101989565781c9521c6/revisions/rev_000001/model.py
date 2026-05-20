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


def _box(part, name, size, center, material):
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _side_pocket(part, name, x, y, z, radius, material):
    part.visual(
        Cylinder(radius=radius, length=0.003),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_drawer_slide_module")

    steel = model.material("brushed_zinc_steel", rgba=(0.58, 0.62, 0.63, 1.0))
    dark = model.material("shadowed_pocket", rgba=(0.04, 0.045, 0.05, 1.0))
    stop_rubber = model.material("black_stop_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    plate_mat = model.material("load_plate_steel", rgba=(0.72, 0.70, 0.62, 1.0))

    outer = model.part("outer_channel")
    middle = model.part("middle_channel")
    inner = model.part("inner_channel")
    plate = model.part("mounting_plate")

    # Fixed, grounded outer boxed C-channel: 0.90 m long with folded lips and
    # visible wall thickness.  The broad footprint makes it unmistakably the
    # stationary housing.
    _box(outer, "outer_floor", (0.900, 0.160, 0.008), (0.450, 0.000, 0.004), steel)
    _box(outer, "outer_side_pos", (0.900, 0.007, 0.078), (0.450, 0.0765, 0.041), steel)
    _box(outer, "outer_side_neg", (0.900, 0.007, 0.078), (0.450, -0.0765, 0.041), steel)
    _box(outer, "outer_lip_pos", (0.900, 0.030, 0.007), (0.450, 0.059, 0.0765), steel)
    _box(outer, "outer_lip_neg", (0.900, 0.030, 0.007), (0.450, -0.059, 0.0765), steel)
    _box(outer, "outer_front_stop_pos", (0.030, 0.014, 0.014), (0.872, 0.067, 0.064), stop_rubber)
    _box(outer, "outer_front_stop_neg", (0.030, 0.014, 0.014), (0.872, -0.067, 0.064), stop_rubber)
    _box(outer, "outer_rear_stop_pos", (0.030, 0.014, 0.014), (0.035, 0.067, 0.064), stop_rubber)
    _box(outer, "outer_rear_stop_neg", (0.030, 0.014, 0.014), (0.035, -0.067, 0.064), stop_rubber)
    for i, x in enumerate((0.180, 0.450, 0.720)):
        _side_pocket(outer, f"outer_pocket_pos_{i}", x, 0.072, 0.034, 0.012, dark)
        _side_pocket(outer, f"outer_pocket_neg_{i}", x, -0.072, 0.034, 0.012, dark)

    # First moving carriage.  It is shorter and visibly nested inside the outer
    # channel with its own boxed profile and inward retaining lips.
    _box(middle, "middle_floor", (0.720, 0.120, 0.006), (0.360, 0.000, 0.015), steel)
    _box(middle, "middle_side_pos", (0.720, 0.006, 0.050), (0.360, 0.057, 0.038), steel)
    _box(middle, "middle_side_neg", (0.720, 0.006, 0.050), (0.360, -0.057, 0.038), steel)
    _box(middle, "middle_lip_pos", (0.720, 0.024, 0.006), (0.360, 0.043, 0.061), steel)
    _box(middle, "middle_lip_neg", (0.720, 0.024, 0.006), (0.360, -0.043, 0.061), steel)
    _box(middle, "middle_front_stop_pos", (0.025, 0.011, 0.011), (0.690, 0.049, 0.052), stop_rubber)
    _box(middle, "middle_front_stop_neg", (0.025, 0.011, 0.011), (0.690, -0.049, 0.052), stop_rubber)
    _box(middle, "middle_rear_stop_pos", (0.025, 0.011, 0.011), (0.035, 0.049, 0.052), stop_rubber)
    _box(middle, "middle_rear_stop_neg", (0.025, 0.011, 0.011), (0.035, -0.049, 0.052), stop_rubber)
    _box(middle, "outer_bearing_pos", (0.100, 0.014, 0.018), (0.195, 0.066, 0.040), steel)
    _box(middle, "outer_bearing_neg", (0.100, 0.014, 0.018), (0.195, -0.066, 0.040), steel)
    _box(middle, "outer_bearing_pos_1", (0.100, 0.014, 0.018), (0.560, 0.066, 0.040), steel)
    _box(middle, "outer_bearing_neg_1", (0.100, 0.014, 0.018), (0.560, -0.066, 0.040), steel)
    for i, x in enumerate((0.170, 0.380, 0.590)):
        _side_pocket(middle, f"middle_pocket_pos_{i}", x, 0.053, 0.035, 0.009, dark)
        _side_pocket(middle, f"middle_pocket_neg_{i}", x, -0.053, 0.035, 0.009, dark)

    # Innermost rail: the shortest, narrowest load-carrying channel.  Its top
    # lips leave a central saddle slot for the cantilevered plate bracket.
    _box(inner, "inner_floor", (0.520, 0.062, 0.005), (0.260, 0.000, 0.0265), steel)
    _box(inner, "inner_side_pos", (0.520, 0.005, 0.027), (0.260, 0.0285, 0.0395), steel)
    _box(inner, "inner_side_neg", (0.520, 0.005, 0.027), (0.260, -0.0285, 0.0395), steel)
    _box(inner, "inner_lip_pos", (0.520, 0.014, 0.005), (0.260, 0.019, 0.0505), steel)
    _box(inner, "inner_lip_neg", (0.520, 0.014, 0.005), (0.260, -0.019, 0.0505), steel)
    _box(inner, "inner_front_stop_pos", (0.020, 0.008, 0.008), (0.495, 0.021, 0.054), stop_rubber)
    _box(inner, "inner_front_stop_neg", (0.020, 0.008, 0.008), (0.495, -0.021, 0.054), stop_rubber)
    _box(inner, "middle_bearing_pos", (0.075, 0.024, 0.014), (0.145, 0.042, 0.040), steel)
    _box(inner, "middle_bearing_neg", (0.075, 0.024, 0.014), (0.145, -0.042, 0.040), steel)
    _box(inner, "middle_bearing_pos_1", (0.075, 0.024, 0.014), (0.405, 0.042, 0.040), steel)
    _box(inner, "middle_bearing_neg_1", (0.075, 0.024, 0.014), (0.405, -0.042, 0.040), steel)
    for i, x in enumerate((0.130, 0.285, 0.440)):
        _side_pocket(inner, f"inner_pocket_pos_{i}", x, 0.026, 0.040, 0.006, dark)
        _side_pocket(inner, f"inner_pocket_neg_{i}", x, -0.026, 0.040, 0.006, dark)

    # Cantilevered load plate bolted to the inner member.  The two saddle feet
    # sit on the inner lips, a bridge crosses the open slot, and vertical webs
    # carry the deck so the load path is visually unambiguous.
    _box(plate, "saddle_foot_pos", (0.060, 0.012, 0.004), (0.000, 0.019, 0.002), plate_mat)
    _box(plate, "saddle_foot_neg", (0.060, 0.012, 0.004), (0.000, -0.019, 0.002), plate_mat)
    _box(plate, "saddle_bridge", (0.070, 0.052, 0.004), (0.002, 0.000, 0.006), plate_mat)
    _box(plate, "center_riser", (0.045, 0.020, 0.036), (0.010, 0.000, 0.026), plate_mat)
    _box(plate, "gusset_pos", (0.135, 0.006, 0.036), (0.072, 0.024, 0.026), plate_mat)
    _box(plate, "gusset_neg", (0.135, 0.006, 0.036), (0.072, -0.024, 0.026), plate_mat)
    _box(plate, "plate_deck", (0.340, 0.220, 0.008), (0.170, 0.000, 0.048), plate_mat)
    for i, (x, y) in enumerate(((0.095, 0.070), (0.245, 0.070), (0.095, -0.070), (0.245, -0.070))):
        plate.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(x, y, 0.0535)),
            material=dark,
            name=f"bolt_head_{i}",
        )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.080, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.420),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.060, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.40, lower=0.0, upper=0.300),
    )
    model.articulation(
        "inner_to_plate",
        ArticulationType.FIXED,
        parent=inner,
        child=plate,
        origin=Origin(xyz=(0.490, 0.000, 0.052)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_channel")
    middle = object_model.get_part("middle_channel")
    inner = object_model.get_part("inner_channel")
    plate = object_model.get_part("mounting_plate")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0,
        name="middle cross-section is nested inside outer channel",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.0,
        name="inner cross-section is nested inside middle channel",
    )
    ctx.expect_gap(
        outer,
        middle,
        axis="y",
        positive_elem="outer_side_pos",
        negative_elem="middle_side_pos",
        min_gap=0.010,
        name="positive outer wall clears middle wall",
    )
    ctx.expect_gap(
        middle,
        outer,
        axis="y",
        positive_elem="middle_side_neg",
        negative_elem="outer_side_neg",
        min_gap=0.010,
        name="negative outer wall clears middle wall",
    )
    ctx.expect_gap(
        middle,
        inner,
        axis="y",
        positive_elem="middle_side_pos",
        negative_elem="inner_side_pos",
        min_gap=0.020,
        name="positive middle wall clears inner wall",
    )
    ctx.expect_gap(
        inner,
        middle,
        axis="y",
        positive_elem="inner_side_neg",
        negative_elem="middle_side_neg",
        min_gap=0.020,
        name="negative middle wall clears inner wall",
    )
    ctx.expect_gap(
        middle,
        outer,
        axis="z",
        positive_elem="middle_floor",
        negative_elem="outer_floor",
        min_gap=0.004,
        name="middle rides above outer floor",
    )
    ctx.expect_gap(
        outer,
        middle,
        axis="z",
        positive_elem="outer_lip_pos",
        negative_elem="middle_lip_pos",
        min_gap=0.008,
        name="middle top lip clears outer retaining lip",
    )
    ctx.expect_gap(
        plate,
        outer,
        axis="z",
        positive_elem="plate_deck",
        negative_elem="outer_lip_pos",
        min_gap=0.016,
        name="cantilevered deck stays above outer lips",
    )

    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.60,
        name="collapsed middle stage remains deeply inserted",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.45,
        name="collapsed inner stage remains deeply inserted",
    )

    rest_plate = ctx.part_world_position(plate)
    with ctx.pose({outer_slide: 0.420, inner_slide: 0.300}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.35,
            name="extended middle stage keeps retained insertion",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.30,
            name="extended inner stage keeps retained insertion",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="extended inner cross-section stays in middle envelope",
        )
        extended_plate = ctx.part_world_position(plate)

    ctx.check(
        "serial slides move plate outward",
        rest_plate is not None
        and extended_plate is not None
        and extended_plate[0] > rest_plate[0] + 0.65,
        details=f"rest={rest_plate}, extended={extended_plate}",
    )

    return ctx.report()


object_model = build_object_model()
