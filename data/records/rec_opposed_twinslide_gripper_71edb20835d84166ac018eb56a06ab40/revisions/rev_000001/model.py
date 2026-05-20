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
    model = ArticulatedObject(name="cheek_guided_twin_slide_gripper")

    cast_aluminum = model.material("cast_aluminum", color=(0.62, 0.66, 0.67, 1.0))
    dark_steel = model.material("dark_steel", color=(0.08, 0.09, 0.10, 1.0))
    ground_steel = model.material("ground_steel", color=(0.78, 0.80, 0.78, 1.0))
    black_rubber = model.material("black_rubber", color=(0.01, 0.01, 0.012, 1.0))
    blue_anodized = model.material("blue_anodized", color=(0.05, 0.18, 0.42, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.32, 0.16, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=cast_aluminum,
        name="body",
    )
    housing.visual(
        Box((0.48, 0.018, 0.095)),
        origin=Origin(xyz=(0.0, 0.069, 0.1125)),
        material=cast_aluminum,
        name="guide_cheek_0",
    )
    housing.visual(
        Box((0.48, 0.018, 0.095)),
        origin=Origin(xyz=(0.0, -0.069, 0.1125)),
        material=cast_aluminum,
        name="guide_cheek_1",
    )
    housing.visual(
        Box((0.455, 0.014, 0.025)),
        origin=Origin(xyz=(0.0, 0.026, 0.0775)),
        material=ground_steel,
        name="lower_rail_0",
    )
    housing.visual(
        Box((0.455, 0.014, 0.025)),
        origin=Origin(xyz=(0.0, -0.026, 0.0775)),
        material=ground_steel,
        name="lower_rail_1",
    )
    housing.visual(
        Box((0.405, 0.0065, 0.032)),
        origin=Origin(xyz=(0.0, 0.05675, 0.106)),
        material=dark_steel,
        name="wear_strip_0",
    )
    housing.visual(
        Box((0.405, 0.0065, 0.032)),
        origin=Origin(xyz=(0.0, -0.05675, 0.106)),
        material=dark_steel,
        name="wear_strip_1",
    )
    housing.visual(
        Box((0.16, 0.118, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=dark_steel,
        name="actuator_cover",
    )

    screw_index = 0
    for y in (-0.069, 0.069):
        for x in (-0.18, -0.075, 0.075, 0.18):
            housing.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, 0.162)),
                material=dark_steel,
                name=f"cap_screw_{screw_index}",
            )
            screw_index += 1

    def add_carriage(name: str, finger_sign: float):
        carriage = model.part(name)
        carriage.visual(
            Box((0.170, 0.104, 0.036)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=ground_steel,
            name="slide_block",
        )
        carriage.visual(
            Box((0.118, 0.030, 0.012)),
            origin=Origin(xyz=(-0.020 * finger_sign, 0.0, 0.024)),
            material=blue_anodized,
            name="carriage_cap",
        )
        carriage.visual(
            Box((0.030, 0.092, 0.160)),
            origin=Origin(xyz=(0.098 * finger_sign, 0.0, 0.098)),
            material=ground_steel,
            name="finger",
        )
        carriage.visual(
            Box((0.006, 0.076, 0.112)),
            origin=Origin(xyz=(0.1155 * finger_sign, 0.0, 0.102)),
            material=black_rubber,
            name="grip_pad",
        )
        return carriage

    left_carriage = add_carriage("left_carriage", 1.0)
    right_carriage = add_carriage("right_carriage", -1.0)

    model.articulation(
        "housing_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_carriage,
        origin=Origin(xyz=(-0.150, 0.0, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.028),
    )
    model.articulation(
        "housing_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_carriage,
        origin=Origin(xyz=(0.150, 0.0, 0.108)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.028),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left = object_model.get_part("left_carriage")
    right = object_model.get_part("right_carriage")
    left_slide = object_model.get_articulation("housing_to_left_carriage")
    right_slide = object_model.get_articulation("housing_to_right_carriage")

    ctx.check(
        "two independent prismatic carriage joints",
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and right_slide.articulation_type == ArticulationType.PRISMATIC
        and left_slide.mimic is None
        and right_slide.mimic is None,
        details=f"left={left_slide.articulation_type}, right={right_slide.articulation_type}",
    )

    for carriage in (left, right):
        ctx.expect_within(
            carriage,
            housing,
            axes="y",
            inner_elem="slide_block",
            outer_elem="body",
            margin=0.0,
            name=f"{carriage.name} lies inside the cheek span",
        )
        ctx.expect_gap(
            carriage,
            housing,
            axis="z",
            positive_elem="slide_block",
            negative_elem="lower_rail_0",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{carriage.name} rests on the lower rail",
        )
        ctx.expect_gap(
            housing,
            carriage,
            axis="y",
            positive_elem="guide_cheek_0",
            negative_elem="slide_block",
            min_gap=0.001,
            max_gap=0.012,
            name=f"{carriage.name} clears the upper cheek",
        )
        ctx.expect_gap(
            carriage,
            housing,
            axis="y",
            positive_elem="slide_block",
            negative_elem="guide_cheek_1",
            min_gap=0.001,
            max_gap=0.012,
            name=f"{carriage.name} clears the lower cheek",
        )

    ctx.expect_gap(
        right,
        left,
        axis="x",
        positive_elem="grip_pad",
        negative_elem="grip_pad",
        min_gap=0.055,
        max_gap=0.070,
        name="open pads leave a workpiece gap",
    )

    left_rest = ctx.part_world_position(left)
    right_rest = ctx.part_world_position(right)
    with ctx.pose({left_slide: 0.028, right_slide: 0.028}):
        left_closed = ctx.part_world_position(left)
        right_closed = ctx.part_world_position(right)
        ctx.expect_gap(
            right,
            left,
            axis="x",
            positive_elem="grip_pad",
            negative_elem="grip_pad",
            min_gap=0.004,
            max_gap=0.012,
            name="closed pads approach without colliding",
        )
        ctx.expect_gap(
            right,
            left,
            axis="x",
            positive_elem="slide_block",
            negative_elem="slide_block",
            min_gap=0.060,
            name="carriage blocks remain separate at full stroke",
        )

    ctx.check(
        "left carriage closes inward",
        left_rest is not None and left_closed is not None and left_closed[0] > left_rest[0] + 0.020,
        details=f"rest={left_rest}, closed={left_closed}",
    )
    ctx.check(
        "right carriage closes inward",
        right_rest is not None and right_closed is not None and right_closed[0] < right_rest[0] - 0.020,
        details=f"rest={right_rest}, closed={right_closed}",
    )

    return ctx.report()


object_model = build_object_model()
