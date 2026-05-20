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
    model = ArticulatedObject(name="service_slide_unit")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    shadow = model.material("dark_recess", rgba=(0.015, 0.016, 0.018, 1.0))
    carriage_finish = model.material("oiled_inner_rail", rgba=(0.28, 0.30, 0.31, 1.0))
    end_plate_finish = model.material("brushed_end_plate", rgba=(0.50, 0.52, 0.54, 1.0))
    low_friction = model.material("white_bearing_strip", rgba=(0.88, 0.86, 0.78, 1.0))

    channel = model.part("channel")
    # Long fixed C-channel: bottom web, side walls, and inward top lips leave
    # a genuine clearanced slot for the nested moving carriage.
    channel.visual(
        Box((0.720, 0.100, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=galvanized,
        name="bottom_web",
    )
    for y, name in ((0.044, "side_wall_0"), (-0.044, "side_wall_1")):
        channel.visual(
            Box((0.720, 0.012, 0.070)),
            origin=Origin(xyz=(0.0, y, 0.035)),
            material=galvanized,
            name=name,
        )
    for y, name in ((0.028, "top_lip_0"), (-0.028, "top_lip_1")):
        channel.visual(
            Box((0.720, 0.022, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.064)),
            material=galvanized,
            name=name,
        )
    channel.visual(
        Box((0.012, 0.100, 0.070)),
        origin=Origin(xyz=(0.354, 0.0, 0.035)),
        material=galvanized,
        name="rear_stop",
    )
    for x, name in ((-0.220, "mount_hole_0"), (0.060, "mount_hole_1"), (0.260, "mount_hole_2")):
        channel.visual(
            Cylinder(radius=0.011, length=0.002),
            origin=Origin(xyz=(x, 0.0, 0.013)),
            material=shadow,
            name=name,
        )

    carriage = model.part("carriage")
    # The child frame is near the front seating plane.  The rail extends back
    # into the fixed guide so it remains captured at full travel.
    carriage.visual(
        Box((0.422, 0.052, 0.034)),
        origin=Origin(xyz=(0.189, 0.0, 0.0)),
        material=carriage_finish,
        name="slide_bar",
    )
    for y, name in ((0.019, "bearing_strip_0"), (-0.019, "bearing_strip_1")):
        carriage.visual(
            Box((0.360, 0.009, 0.004)),
            origin=Origin(xyz=(0.180, y, 0.018)),
            material=low_friction,
            name=name,
        )
    carriage.visual(
        Box((0.022, 0.112, 0.112)),
        origin=Origin(xyz=(-0.031, 0.0, 0.0)),
        material=end_plate_finish,
        name="end_plate",
    )
    for y in (-0.035, 0.035):
        for z in (-0.035, 0.035):
            carriage.visual(
                Cylinder(radius=0.008, length=0.003),
                origin=Origin(xyz=(-0.043, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=shadow,
                name=f"plate_bolt_{0 if y < 0 else 1}_{0 if z < 0 else 1}",
            )

    model.articulation(
        "channel_to_carriage",
        ArticulationType.PRISMATIC,
        parent=channel,
        child=carriage,
        origin=Origin(xyz=(-0.340, 0.0, 0.036)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.280),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    channel = object_model.get_part("channel")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("channel_to_carriage")

    ctx.expect_within(
        carriage,
        channel,
        axes="yz",
        inner_elem="slide_bar",
        margin=0.0,
        name="carriage bar fits inside channel envelope",
    )
    ctx.expect_gap(
        carriage,
        channel,
        axis="z",
        positive_elem="slide_bar",
        negative_elem="bottom_web",
        min_gap=0.004,
        max_gap=0.010,
        name="slide bar clears bottom web",
    )
    ctx.expect_gap(
        channel,
        carriage,
        axis="x",
        positive_elem="bottom_web",
        negative_elem="end_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="end plate seats against front of guide",
    )
    ctx.expect_overlap(
        carriage,
        channel,
        axes="x",
        elem_a="slide_bar",
        elem_b="bottom_web",
        min_overlap=0.35,
        name="retracted carriage remains deeply nested",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.280}):
        ctx.expect_within(
            carriage,
            channel,
            axes="yz",
            inner_elem="slide_bar",
            margin=0.0,
            name="extended carriage stays centered in guide",
        )
        ctx.expect_overlap(
            carriage,
            channel,
            axes="x",
            elem_a="slide_bar",
            elem_b="bottom_web",
            min_overlap=0.12,
            name="extended carriage retains insertion",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "joint extends carriage outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
