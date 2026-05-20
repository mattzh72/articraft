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
    model = ArticulatedObject(name="narrow_fork_carriage")

    mast_paint = model.material("mast_paint", rgba=(0.12, 0.15, 0.17, 1.0))
    carriage_paint = model.material("carriage_paint", rgba=(0.95, 0.63, 0.08, 1.0))
    fork_steel = model.material("fork_steel", rgba=(0.10, 0.10, 0.10, 1.0))
    roller_rubber = model.material("roller_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    mast = model.part("mast")
    # A narrow fixed mast: two box-section guide rails tied together by rigid
    # crossmembers and floor shoes.  The rails remain visible through the
    # moving carriage window.
    for suffix, y, guide_name, shoe_name in (
        ("0", -0.28, "guide_rail_0", "floor_shoe_0"),
        ("1", 0.28, "guide_rail_1", "floor_shoe_1"),
    ):
        mast.visual(
            Box((0.08, 0.06, 2.25)),
            origin=Origin(xyz=(0.0, y, 1.18)),
            material=mast_paint,
            name=guide_name,
        )
        mast.visual(
            Box((0.50, 0.10, 0.08)),
            origin=Origin(xyz=(0.18, y, 0.04)),
            material=mast_paint,
            name=shoe_name,
        )

    mast.visual(
        Box((0.12, 0.72, 0.10)),
        origin=Origin(xyz=(-0.025, 0.0, 0.10)),
        material=mast_paint,
        name="bottom_sill",
    )
    mast.visual(
        Box((0.14, 0.76, 0.12)),
        origin=Origin(xyz=(-0.025, 0.0, 2.35)),
        material=mast_paint,
        name="top_header",
    )
    mast.visual(
        Box((0.08, 0.68, 0.08)),
        origin=Origin(xyz=(-0.055, 0.0, 1.25)),
        material=mast_paint,
        name="rear_tie",
    )

    carriage = model.part("carriage")
    # Boxy welded lift carriage.  Its part frame sits near the lower guide
    # shoe line; all geometry moves together on one vertical prismatic joint.
    carriage.visual(
        Box((0.10, 0.82, 0.08)),
        origin=Origin(xyz=(0.14, 0.0, 0.64)),
        material=carriage_paint,
        name="top_bar",
    )
    carriage.visual(
        Box((0.12, 0.82, 0.10)),
        origin=Origin(xyz=(0.15, 0.0, 0.05)),
        material=carriage_paint,
        name="bottom_bar",
    )
    for suffix, y in (("0", -0.39), ("1", 0.39)):
        carriage.visual(
            Box((0.10, 0.08, 0.68)),
            origin=Origin(xyz=(0.14, y, 0.34)),
            material=carriage_paint,
            name=f"side_upright_{suffix}",
        )
    carriage.visual(
        Box((0.07, 0.68, 0.06)),
        origin=Origin(xyz=(0.165, 0.0, 0.34)),
        material=carriage_paint,
        name="middle_bar",
    )

    # Four close-fitting guide channels wrap around the mast rails without
    # intersecting them.  Small black rollers nearly touch the rail faces.
    for rail_suffix, y, front_name, lower_roller_name, upper_roller_name in (
        ("0", -0.28, "front_shoe_0", "lower_roller_0", "upper_roller_0"),
        ("1", 0.28, "front_shoe_1", "lower_roller_1", "upper_roller_1"),
    ):
        carriage.visual(
            Box((0.05, 0.16, 0.58)),
            origin=Origin(xyz=(0.095, y, 0.34)),
            material=carriage_paint,
            name=front_name,
        )
        for cheek_suffix, cheek_y in (
            ("inner", y - 0.075 if y > 0 else y + 0.075),
            ("outer", y + 0.075 if y > 0 else y - 0.075),
        ):
            carriage.visual(
                Box((0.15, 0.025, 0.58)),
                origin=Origin(xyz=(0.040, cheek_y, 0.34)),
                material=carriage_paint,
                name=f"{cheek_suffix}_cheek_{rail_suffix}",
            )
        for roller_name, z in ((lower_roller_name, 0.18), (upper_roller_name, 0.52)):
            carriage.visual(
                Cylinder(radius=0.035, length=0.052),
                origin=Origin(xyz=(0.075, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=roller_rubber,
                name=roller_name,
            )

    # Two spaced forks are welded/latched to the lower carriage edge.  The
    # stepped front boxes make the tips read thinner than the shanks.
    for suffix, y in (("0", -0.21), ("1", 0.21)):
        carriage.visual(
            Box((0.12, 0.10, 0.42)),
            origin=Origin(xyz=(0.18, y, 0.11)),
            material=fork_steel,
            name=f"fork_shank_{suffix}",
        )
        carriage.visual(
            Box((0.78, 0.09, 0.07)),
            origin=Origin(xyz=(0.58, y, -0.10)),
            material=fork_steel,
            name=f"fork_tine_{suffix}",
        )
        carriage.visual(
            Box((0.22, 0.07, 0.045)),
            origin=Origin(xyz=(1.06, y, -0.112)),
            material=fork_steel,
            name=f"fork_tip_{suffix}",
        )
        carriage.visual(
            Box((0.07, 0.18, 0.12)),
            origin=Origin(xyz=(0.135, y, 0.23)),
            material=fork_steel,
            name=f"fork_hook_{suffix}",
        )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.45, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single vertical prismatic lift",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper > 1.0,
        details=f"type={lift.articulation_type}, axis={lift.axis}, limits={lift.motion_limits}",
    )

    # The guide rollers contact the fixed rail face, proving the
    # carriage wraps and rides the mast instead of floating forward of it.
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="lower_roller_0",
        negative_elem="guide_rail_0",
        max_gap=0.0005,
        max_penetration=0.000001,
        name="lower guide roller bears on rail",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="upper_roller_1",
        negative_elem="guide_rail_1",
        max_gap=0.0005,
        max_penetration=0.000001,
        name="upper guide roller bears on rail",
    )

    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a="front_shoe_0",
        elem_b="guide_rail_0",
        min_overlap=0.50,
        name="carriage shoe has long retained rail engagement",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({lift: 1.05}):
        raised_position = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a="front_shoe_0",
            elem_b="guide_rail_0",
            min_overlap=0.50,
            name="raised carriage remains on guide rail",
        )

    ctx.check(
        "fork carriage lifts upward",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 1.0,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
