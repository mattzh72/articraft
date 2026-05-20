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
    model = ArticulatedObject(name="compact_parallel_gripper")

    body_mat = model.material("black_anodized", rgba=(0.025, 0.028, 0.032, 1.0))
    edge_mat = model.material("dark_hardcoat", rgba=(0.010, 0.012, 0.014, 1.0))
    slide_mat = model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    finger_mat = model.material("tool_steel", rgba=(0.46, 0.48, 0.47, 1.0))
    rubber_mat = model.material("dark_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    screw_mat = model.material("socket_screw", rgba=(0.05, 0.05, 0.052, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.095, 0.132, 0.050)),
        origin=Origin(xyz=(-0.0325, 0.0, 0.034)),
        material=body_mat,
        name="rear_body",
    )
    housing.visual(
        Box((0.100, 0.140, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, 0.061)),
        material=edge_mat,
        name="top_guide",
    )
    housing.visual(
        Box((0.100, 0.140, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, 0.007)),
        material=edge_mat,
        name="bottom_guide",
    )
    for index, y in enumerate((-0.074, 0.074)):
        housing.visual(
            Box((0.100, 0.008, 0.046)),
            origin=Origin(xyz=(0.060, y, 0.034)),
            material=edge_mat,
            name=f"side_stop_{index}",
        )
    for index, (y, z) in enumerate(
        ((-0.046, 0.022), (0.046, 0.022), (-0.046, 0.046), (0.046, 0.046))
    ):
        housing.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.017, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=screw_mat,
            name=f"front_screw_{index}",
        )

    def add_jaw(part_name: str, side: float):
        jaw = model.part(part_name)
        inward = -side
        jaw.visual(
            Box((0.070, 0.026, 0.043)),
            origin=Origin(xyz=(0.060, 0.0, 0.034)),
            material=slide_mat,
            name="carriage",
        )
        jaw.visual(
            Box((0.070, 0.014, 0.020)),
            origin=Origin(xyz=(0.052, 0.0, 0.034)),
            material=finger_mat,
            name="guide_tongue",
        )
        jaw.visual(
            Box((0.028, 0.032, 0.040)),
            origin=Origin(xyz=(0.106, 0.0, 0.034)),
            material=finger_mat,
            name="finger_mount",
        )
        jaw.visual(
            Box((0.076, 0.014, 0.030)),
            origin=Origin(xyz=(0.149, inward * 0.006, 0.034)),
            material=finger_mat,
            name="finger_step",
        )
        jaw.visual(
            Box((0.044, 0.003, 0.020)),
            origin=Origin(xyz=(0.162, inward * 0.014, 0.034)),
            material=rubber_mat,
            name="grip_pad",
        )
        return jaw

    jaw_0 = add_jaw("jaw_0", 1.0)
    jaw_1 = add_jaw("jaw_1", -1.0)

    travel = 0.016
    model.articulation(
        "housing_to_jaw_0",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=jaw_0,
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=travel),
    )
    model.articulation(
        "housing_to_jaw_1",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=jaw_1,
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=travel),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    jaw_0 = object_model.get_part("jaw_0")
    jaw_1 = object_model.get_part("jaw_1")
    joint_0 = object_model.get_articulation("housing_to_jaw_0")
    joint_1 = object_model.get_articulation("housing_to_jaw_1")

    ctx.check(
        "two independent prismatic jaw slides",
        joint_0.articulation_type == ArticulationType.PRISMATIC
        and joint_1.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint types: {joint_0.articulation_type}, {joint_1.articulation_type}",
    )
    ctx.expect_overlap(
        jaw_0,
        housing,
        axes="xy",
        elem_a="carriage",
        elem_b="top_guide",
        min_overlap=0.020,
        name="jaw_0 carriage visibly remains under the top guide",
    )
    ctx.expect_overlap(
        jaw_1,
        housing,
        axes="xy",
        elem_a="carriage",
        elem_b="top_guide",
        min_overlap=0.020,
        name="jaw_1 carriage visibly remains under the top guide",
    )
    ctx.expect_gap(
        housing,
        jaw_0,
        axis="z",
        positive_elem="top_guide",
        negative_elem="carriage",
        min_gap=0.0,
        max_gap=0.001,
        name="jaw_0 has close top guide clearance",
    )
    ctx.expect_gap(
        housing,
        jaw_1,
        axis="z",
        positive_elem="top_guide",
        negative_elem="carriage",
        min_gap=0.0,
        max_gap=0.001,
        name="jaw_1 has close top guide clearance",
    )

    rest_0 = ctx.part_world_position(jaw_0)
    rest_1 = ctx.part_world_position(jaw_1)
    with ctx.pose({joint_0: 0.016, joint_1: 0.016}):
        ctx.expect_overlap(
            jaw_0,
            housing,
            axes="xy",
            elem_a="carriage",
            elem_b="top_guide",
            min_overlap=0.020,
            name="closed jaw_0 remains captured by the guide",
        )
        ctx.expect_overlap(
            jaw_1,
            housing,
            axes="xy",
            elem_a="carriage",
            elem_b="top_guide",
            min_overlap=0.020,
            name="closed jaw_1 remains captured by the guide",
        )
        ctx.expect_gap(
            jaw_0,
            jaw_1,
            axis="y",
            positive_elem="grip_pad",
            negative_elem="grip_pad",
            min_gap=0.004,
            max_gap=0.010,
            name="opposing stepped fingers close without colliding",
        )
        closed_0 = ctx.part_world_position(jaw_0)
        closed_1 = ctx.part_world_position(jaw_1)
    ctx.check(
        "jaw_0 closes toward the centerline",
        rest_0 is not None and closed_0 is not None and closed_0[1] < rest_0[1] - 0.010,
        details=f"rest={rest_0}, closed={closed_0}",
    )
    ctx.check(
        "jaw_1 closes toward the centerline",
        rest_1 is not None and closed_1 is not None and closed_1[1] > rest_1[1] + 0.010,
        details=f"rest={rest_1}, closed={closed_1}",
    )

    return ctx.report()


object_model = build_object_model()
