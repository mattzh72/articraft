from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="compact_stacker_mast")

    mast_paint = model.material("charcoal_powder_coat", rgba=(0.10, 0.12, 0.13, 1.0))
    carriage_paint = model.material("safety_yellow_paint", rgba=(0.95, 0.66, 0.10, 1.0))
    bright_steel = model.material("bright_wear_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_steel = model.material("black_oxide_fasteners", rgba=(0.02, 0.02, 0.018, 1.0))
    rubber = model.material("black_polyurethane_rollers", rgba=(0.015, 0.014, 0.013, 1.0))
    weld = model.material("burnished_weld_bead", rgba=(0.18, 0.19, 0.18, 1.0))

    mast = model.part("mast")

    # Fixed welded mast: two C-shaped side channels tied by lower and upper
    # crossmembers.  The inward-facing guide strips are intentionally proud of
    # the channels so the moving carriage has visible clearance to the mast.
    mast.visual(
        Box((0.92, 0.28, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=mast_paint,
        name="lower_crossmember",
    )
    mast.visual(
        Box((0.96, 0.30, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 2.25)),
        material=mast_paint,
        name="upper_crosshead",
    )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        mast.visual(
            Box((0.040, 0.230, 1.98)),
            origin=Origin(xyz=(sign * 0.390, 0.0, 1.185)),
            material=mast_paint,
            name=f"{side_name}_channel_web",
        )
        mast.visual(
            Box((0.140, 0.035, 1.98)),
            origin=Origin(xyz=(sign * 0.323, 0.095, 1.185)),
            material=mast_paint,
            name=f"{side_name}_front_flange",
        )
        mast.visual(
            Box((0.140, 0.035, 1.98)),
            origin=Origin(xyz=(sign * 0.323, -0.095, 1.185)),
            material=mast_paint,
            name=f"{side_name}_rear_flange",
        )
        mast.visual(
            Box((0.014, 0.126, 1.74)),
            origin=Origin(xyz=(sign * 0.252, 0.0, 1.19)),
            material=bright_steel,
            name=f"{side_name}_wear_strip",
        )
        mast.visual(
            Box((0.060, 0.180, 1.74)),
            origin=Origin(xyz=(sign * 0.286, 0.0, 1.19)),
            material=mast_paint,
            name=f"{side_name}_strip_backing",
        )
        mast.visual(
            Cylinder(radius=0.006, length=1.90),
            origin=Origin(xyz=(sign * 0.374, 0.075, 1.185)),
            material=weld,
            name=f"{side_name}_front_weld",
        )
        mast.visual(
            Cylinder(radius=0.006, length=1.90),
            origin=Origin(xyz=(sign * 0.374, -0.075, 1.185)),
            material=weld,
            name=f"{side_name}_rear_weld",
        )

        for z in (0.48, 0.82, 1.16, 1.50, 1.84):
            mast.visual(
                Cylinder(radius=0.013, length=0.012),
                origin=Origin(xyz=(sign * 0.242, 0.043, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_steel,
                name=f"{side_name}_front_strip_bolt_{int(z * 100)}",
            )
            mast.visual(
                Cylinder(radius=0.013, length=0.012),
                origin=Origin(xyz=(sign * 0.242, -0.043, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_steel,
                name=f"{side_name}_rear_strip_bolt_{int(z * 100)}",
            )

    for x in (-0.32, 0.32):
        mast.visual(
            Box((0.25, 0.36, 0.05)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=mast_paint,
            name=f"base_foot_{'left' if x < 0 else 'right'}",
        )
        for y in (-0.115, 0.115):
            mast.visual(
                Cylinder(radius=0.018, length=0.008),
                origin=Origin(xyz=(x, y, 0.052)),
                material=dark_steel,
                name=f"floor_bolt_{'left' if x < 0 else 'right'}_{'rear' if y < 0 else 'front'}",
            )

    carriage = model.part("carriage")

    # The child frame origin is on the carriage guide centerline at its lower
    # travel position.  All visuals are local to that moving stage.
    carriage.visual(
        Box((0.36, 0.090, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=carriage_paint,
        name="bottom_bar",
    )
    carriage.visual(
        Box((0.36, 0.090, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
        material=carriage_paint,
        name="top_bar",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        carriage.visual(
            Box((0.055, 0.082, 0.720)),
            origin=Origin(xyz=(sign * 0.150, 0.0, 0.400)),
            material=carriage_paint,
            name=f"{side_name}_side_post",
        )
        carriage.visual(
            Box((0.028, 0.028, 0.650)),
            origin=Origin(xyz=(sign * 0.070, 0.046, 0.400)),
            material=carriage_paint,
            name=f"{side_name}_front_rib",
        )
        carriage.visual(
            Box((0.028, 0.028, 0.650)),
            origin=Origin(xyz=(sign * 0.070, -0.046, 0.400)),
            material=carriage_paint,
            name=f"{side_name}_rear_rib",
        )

        for level_name, z in (("lower", 0.205), ("upper", 0.595)):
            for face_name, y in (("front", 0.056), ("rear", -0.056)):
                carriage.visual(
                    Box((0.065, 0.032, 0.105)),
                    origin=Origin(xyz=(sign * 0.188, y, z)),
                    material=carriage_paint,
                    name=f"{side_name}_{level_name}_{face_name}_bearing_block",
                )
                carriage.visual(
                    Box((0.018, 0.045, 0.130)),
                    origin=Origin(xyz=(sign * 0.226, y, z)),
                    material=bright_steel,
                    name=f"{side_name}_{level_name}_{face_name}_shoe",
                )
                carriage.visual(
                    Cylinder(radius=0.032, length=0.034),
                    origin=Origin(xyz=(sign * 0.213, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                    material=rubber,
                    name=f"{side_name}_{level_name}_{face_name}_roller",
                )
                carriage.visual(
                    Cylinder(radius=0.010, length=0.070),
                    origin=Origin(xyz=(sign * 0.190, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                    material=dark_steel,
                    name=f"{side_name}_{level_name}_{face_name}_axle",
                )

    # A shallow front cross plate and two gusset-like ribs make the carriage read
    # as a welded rectangular lifting frame rather than a loose collection of
    # pads.
    carriage.visual(
        Box((0.235, 0.024, 0.150)),
        origin=Origin(xyz=(0.0, 0.054, 0.400)),
        material=carriage_paint,
        name="front_cross_rib",
    )
    carriage.visual(
        Box((0.235, 0.024, 0.150)),
        origin=Origin(xyz=(0.0, -0.054, 0.400)),
        material=carriage_paint,
        name="rear_cross_rib",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.35, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("carriage_slide")

    ctx.expect_within(
        carriage,
        mast,
        axes="xy",
        margin=0.0,
        name="carriage remains centered between side channels",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="z",
        positive_elem="bottom_bar",
        negative_elem="lower_crossmember",
        min_gap=0.16,
        name="lower crossmember clears the carriage at bottom travel",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="left_lower_front_roller",
        negative_elem="left_wear_strip",
        min_gap=0.0,
        max_gap=0.003,
        name="left guide roller has working clearance",
    )
    ctx.expect_gap(
        mast,
        carriage,
        axis="x",
        positive_elem="right_wear_strip",
        negative_elem="right_lower_front_roller",
        min_gap=0.0,
        max_gap=0.003,
        name="right guide roller has working clearance",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a="left_upper_front_roller",
        elem_b="left_wear_strip",
        min_overlap=0.05,
        name="rollers stay engaged with vertical guide strips",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.75}):
        ctx.expect_within(
            carriage,
            mast,
            axes="xy",
            margin=0.0,
            name="raised carriage remains inside mast envelope",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem="upper_crosshead",
            negative_elem="top_bar",
            min_gap=0.05,
            name="upper crosshead clears raised carriage",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="x",
            positive_elem="left_upper_front_roller",
            negative_elem="left_wear_strip",
            min_gap=0.0,
            max_gap=0.003,
            name="left guide clearance at upper travel",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="x",
            positive_elem="right_wear_strip",
            negative_elem="right_upper_front_roller",
            min_gap=0.0,
            max_gap=0.003,
            name="right guide clearance at upper travel",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic stage travels upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
