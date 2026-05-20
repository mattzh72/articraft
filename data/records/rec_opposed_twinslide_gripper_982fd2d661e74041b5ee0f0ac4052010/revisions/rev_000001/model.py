from __future__ import annotations

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
    model = ArticulatedObject(name="carriage_block_opposed_gripper")

    anodized = Material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_steel = Material("ground_steel_ways", rgba=(0.72, 0.74, 0.72, 1.0))
    carriage_metal = Material("brushed_carriage_aluminum", rgba=(0.42, 0.45, 0.47, 1.0))
    jaw_steel = Material("machined_finger_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = Material("black_grip_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    fastener = Material("blackened_fasteners", rgba=(0.02, 0.02, 0.022, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.260, 0.125, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=anodized,
        name="mounting_foot",
    )
    body.visual(
        Box((0.162, 0.075, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=anodized,
        name="center_housing",
    )
    body.visual(
        Box((0.252, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=rail_steel,
        name="guide_rail",
    )
    body.visual(
        Box((0.018, 0.071, 0.052)),
        origin=Origin(xyz=(-0.130, 0.0, 0.056)),
        material=anodized,
        name="end_stop_0",
    )
    body.visual(
        Box((0.018, 0.071, 0.052)),
        origin=Origin(xyz=(0.130, 0.0, 0.056)),
        material=anodized,
        name="end_stop_1",
    )
    for x in (-0.098, 0.098):
        for y in (-0.045, 0.045):
            body.visual(
                Cylinder(radius=0.006, length=0.007),
                origin=Origin(xyz=(x, y, 0.0315)),
                material=fastener,
                name=f"cap_screw_{x:+.3f}_{y:+.3f}",
            )

    left = model.part("left_carriage")
    left.visual(
        Box((0.055, 0.050, 0.026)),
        origin=Origin(),
        material=carriage_metal,
        name="slider_block",
    )
    left.visual(
        Box((0.050, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=carriage_metal,
        name="top_cap",
    )
    for y in (-0.025, 0.025):
        left.visual(
            Box((0.050, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, y, -0.004)),
            material=carriage_metal,
            name=f"bearing_cheek_{'front' if y < 0 else 'rear'}",
        )
    left.visual(
        Box((0.030, 0.034, 0.020)),
        origin=Origin(xyz=(0.023, -0.034, -0.004)),
        material=jaw_steel,
        name="jaw_lug",
    )
    left.visual(
        Box((0.050, 0.018, 0.020)),
        origin=Origin(xyz=(0.044, -0.052, -0.005)),
        material=jaw_steel,
        name="finger_shank",
    )
    left.visual(
        Box((0.014, 0.028, 0.045)),
        origin=Origin(xyz=(0.073, -0.066, -0.018)),
        material=rubber,
        name="grip_pad",
    )

    right = model.part("right_carriage")
    right.visual(
        Box((0.055, 0.050, 0.026)),
        origin=Origin(),
        material=carriage_metal,
        name="slider_block",
    )
    right.visual(
        Box((0.050, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=carriage_metal,
        name="top_cap",
    )
    for y in (-0.025, 0.025):
        right.visual(
            Box((0.050, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, y, -0.004)),
            material=carriage_metal,
            name=f"bearing_cheek_{'front' if y < 0 else 'rear'}",
        )
    right.visual(
        Box((0.030, 0.034, 0.020)),
        origin=Origin(xyz=(-0.023, -0.034, -0.004)),
        material=jaw_steel,
        name="jaw_lug",
    )
    right.visual(
        Box((0.050, 0.018, 0.020)),
        origin=Origin(xyz=(-0.044, -0.052, -0.005)),
        material=jaw_steel,
        name="finger_shank",
    )
    right.visual(
        Box((0.014, 0.028, 0.045)),
        origin=Origin(xyz=(-0.073, -0.066, -0.018)),
        material=rubber,
        name="grip_pad",
    )

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left,
        origin=Origin(xyz=(-0.095, 0.0, 0.112)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.012),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right,
        origin=Origin(xyz=(0.095, 0.0, 0.112)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left = object_model.get_part("left_carriage")
    right = object_model.get_part("right_carriage")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

    ctx.expect_contact(
        left,
        body,
        elem_a="slider_block",
        elem_b="guide_rail",
        contact_tol=1e-5,
        name="left carriage rides on guide rail",
    )
    ctx.expect_contact(
        right,
        body,
        elem_a="slider_block",
        elem_b="guide_rail",
        contact_tol=1e-5,
        name="right carriage rides on guide rail",
    )
    ctx.expect_gap(
        right,
        left,
        axis="x",
        positive_elem="grip_pad",
        negative_elem="grip_pad",
        min_gap=0.028,
        max_gap=0.032,
        name="open fingertips frame pickup zone",
    )

    left_rest = ctx.part_world_position(left)
    right_rest = ctx.part_world_position(right)
    with ctx.pose({left_slide: 0.012, right_slide: 0.012}):
        ctx.expect_gap(
            right,
            left,
            axis="x",
            positive_elem="grip_pad",
            negative_elem="grip_pad",
            min_gap=0.004,
            max_gap=0.008,
            name="closed fingertips keep a small realistic clearance",
        )
        left_closed = ctx.part_world_position(left)
        right_closed = ctx.part_world_position(right)

    ctx.check(
        "left carriage translates toward pickup zone",
        left_rest is not None and left_closed is not None and left_closed[0] > left_rest[0] + 0.010,
        details=f"rest={left_rest}, closed={left_closed}",
    )
    ctx.check(
        "right carriage translates toward pickup zone",
        right_rest is not None and right_closed is not None and right_closed[0] < right_rest[0] - 0.010,
        details=f"rest={right_rest}, closed={right_closed}",
    )

    return ctx.report()


object_model = build_object_model()
