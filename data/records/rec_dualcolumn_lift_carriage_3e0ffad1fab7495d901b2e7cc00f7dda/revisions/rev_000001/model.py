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
    model = ArticulatedObject(name="cleanroom_lift_axis")

    white = model.material("white_anodized", rgba=(0.92, 0.94, 0.93, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.72, 1.0))
    dark = model.material("black_bearing_liner", rgba=(0.06, 0.065, 0.07, 1.0))

    post_spacing = 0.220
    post_radius = 0.012
    post_height = 0.735
    base_height = 0.105
    post_center_z = base_height + post_height / 2.0 - 0.006

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.155, length=base_height),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=white,
        name="base_can",
    )
    frame.visual(
        Cylinder(radius=0.126, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, base_height + 0.002)),
        material=steel,
        name="base_cap",
    )

    for post_name, collar_name, x in (
        ("post_0", "post_collar_0", -post_spacing / 2.0),
        ("post_1", "post_collar_1", post_spacing / 2.0),
    ):
        frame.visual(
            Cylinder(radius=post_radius, length=post_height),
            origin=Origin(xyz=(x, 0.0, post_center_z)),
            material=steel,
            name=post_name,
        )
        frame.visual(
            Cylinder(radius=0.024, length=0.020),
            origin=Origin(xyz=(x, 0.0, base_height + 0.004)),
            material=steel,
            name=collar_name,
        )

    frame.visual(
        Box((0.335, 0.052, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=white,
        name="top_bridge",
    )
    frame.visual(
        Box((0.090, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.032, 0.845)),
        material=dark,
        name="bridge_label_strip",
    )

    carriage = model.part("carriage")

    carriage.visual(
        Box((0.170, 0.074, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="carriage_block",
    )
    for x, inner_name, outer_name, front_name, rear_name, bearing_front, bearing_rear in (
        (
            -post_spacing / 2.0,
            "guide_0_inner",
            "guide_0_outer",
            "guide_0_front",
            "guide_0_rear",
            "bearing_0_front",
            "bearing_0_rear",
        ),
        (
            post_spacing / 2.0,
            "guide_1_inner",
            "guide_1_outer",
            "guide_1_front",
            "guide_1_rear",
            "bearing_1_front",
            "bearing_1_rear",
        ),
    ):
        for guide_name, dx in (
            (inner_name, -1.0 if x > 0.0 else 1.0),
            (outer_name, 1.0 if x > 0.0 else -1.0),
        ):
            carriage.visual(
                Box((0.014, 0.078, 0.110)),
                origin=Origin(xyz=(x + dx * 0.029, 0.0, 0.0)),
                material=steel,
                name=guide_name,
            )
        carriage.visual(
            Box((0.070, 0.014, 0.110)),
            origin=Origin(xyz=(x, 0.029, 0.0)),
            material=steel,
            name=front_name,
        )
        carriage.visual(
            Box((0.070, 0.014, 0.110)),
            origin=Origin(xyz=(x, -0.029, 0.0)),
            material=steel,
            name=rear_name,
        )
        carriage.visual(
            Box((0.014, 0.011, 0.090)),
            origin=Origin(xyz=(x, post_radius + 0.011 / 2.0, 0.0)),
            material=dark,
            name=bearing_front,
        )
        carriage.visual(
            Box((0.014, 0.011, 0.090)),
            origin=Origin(xyz=(x, -(post_radius + 0.011 / 2.0), 0.0)),
            material=dark,
            name=bearing_rear,
        )
    carriage.visual(
        Box((0.122, 0.080, 0.110)),
        origin=Origin(xyz=(0.0, -0.070, 0.047)),
        material=white,
        name="table_neck",
    )
    carriage.visual(
        Box((0.220, 0.150, 0.020)),
        origin=Origin(xyz=(0.0, -0.142, 0.111)),
        material=steel,
        name="table_plate",
    )
    carriage.visual(
        Box((0.186, 0.116, 0.004)),
        origin=Origin(xyz=(0.0, -0.142, 0.1225)),
        material=dark,
        name="table_insert",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.420),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")
    limits = lift.motion_limits

    ctx.check(
        "single vertical prismatic lift",
        lift.articulation_type == ArticulationType.PRISMATIC
        and lift.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper > 0.40,
        details=f"type={lift.articulation_type}, axis={lift.axis}, limits={limits}",
    )

    ctx.expect_contact(
        frame,
        carriage,
        elem_a="post_0",
        elem_b="bearing_0_front",
        contact_tol=1e-6,
        name="first post bears against carriage",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a="post_1",
        elem_b="bearing_1_front",
        contact_tol=1e-6,
        name="second post bears against carriage",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="carriage_block",
        negative_elem="base_cap",
        min_gap=0.060,
        name="carriage starts above base can",
    )

    rest_pos = ctx.part_world_position(carriage)
    upper = limits.upper if limits is not None and limits.upper is not None else 0.0
    with ctx.pose({lift: upper}):
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="top_bridge",
            negative_elem="table_insert",
            min_gap=0.020,
            name="raised table clears top bridge",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage rises along posts",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.40
        and abs(raised_pos[0] - rest_pos[0]) < 1e-6
        and abs(raised_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
