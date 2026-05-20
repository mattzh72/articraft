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
    model = ArticulatedObject(name="narrow_opposed_slide_gripper")

    body_mat = model.material("dark_anodized_aluminum", rgba=(0.16, 0.18, 0.20, 1.0))
    rail_mat = model.material("polished_linear_rail", rgba=(0.72, 0.74, 0.72, 1.0))
    carriage_mat = model.material("satin_jaw_carriage", rgba=(0.48, 0.50, 0.52, 1.0))
    rubber_mat = model.material("black_rubber_grip", rgba=(0.015, 0.014, 0.012, 1.0))
    screw_mat = model.material("black_socket_screws", rgba=(0.03, 0.03, 0.035, 1.0))

    body = model.part("center_body")
    body.visual(
        Box((0.240, 0.070, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=body_mat,
        name="mounting_plate",
    )
    body.visual(
        Box((0.180, 0.038, 0.033)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=body_mat,
        name="central_housing",
    )
    body.visual(
        Box((0.012, 0.058, 0.030)),
        origin=Origin(xyz=(-0.113, 0.0, 0.029)),
        material=body_mat,
        name="end_stop_0",
    )
    body.visual(
        Box((0.012, 0.058, 0.030)),
        origin=Origin(xyz=(0.113, 0.0, 0.029)),
        material=body_mat,
        name="end_stop_1",
    )

    body.visual(
        Box((0.085, 0.008, 0.008)),
        origin=Origin(xyz=(-0.060, -0.019, 0.050)),
        material=rail_mat,
        name="guide_0_front",
    )
    body.visual(
        Box((0.085, 0.008, 0.008)),
        origin=Origin(xyz=(-0.060, 0.019, 0.050)),
        material=rail_mat,
        name="guide_0_rear",
    )
    body.visual(
        Box((0.085, 0.008, 0.008)),
        origin=Origin(xyz=(0.060, -0.019, 0.050)),
        material=rail_mat,
        name="guide_1_front",
    )
    body.visual(
        Box((0.085, 0.008, 0.008)),
        origin=Origin(xyz=(0.060, 0.019, 0.050)),
        material=rail_mat,
        name="guide_1_rear",
    )

    for suffix, x in (("0", -0.095), ("1", 0.095)):
        for side, y in (("front", -0.027), ("rear", 0.027)):
            body.visual(
                Cylinder(radius=0.0055, length=0.0035),
                origin=Origin(xyz=(x, y, 0.0145)),
                material=screw_mat,
                name=f"screw_{suffix}_{side}",
            )

    def add_jaw_carriage(name: str, mirror: float) -> object:
        jaw = model.part(name)
        jaw.visual(
            Box((0.050, 0.064, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=carriage_mat,
            name="saddle",
        )
        jaw.visual(
            Box((0.013, 0.024, 0.070)),
            origin=Origin(xyz=(mirror * 0.0305, -0.043, -0.043)),
            material=carriage_mat,
            name="finger",
        )
        jaw.visual(
            Box((0.004, 0.020, 0.052)),
            origin=Origin(xyz=(mirror * 0.0385, -0.043, -0.045)),
            material=rubber_mat,
            name="pad",
        )
        return jaw

    jaw_0 = add_jaw_carriage("jaw_carriage_0", 1.0)
    jaw_1 = add_jaw_carriage("jaw_carriage_1", -1.0)

    travel = 0.016
    model.articulation(
        "guide_to_jaw_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw_0,
        origin=Origin(xyz=(-0.058, 0.0, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.18, lower=0.0, upper=travel),
    )
    model.articulation(
        "guide_to_jaw_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw_1,
        origin=Origin(xyz=(0.058, 0.0, 0.065)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.18, lower=0.0, upper=travel),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("center_body")
    jaw_0 = object_model.get_part("jaw_carriage_0")
    jaw_1 = object_model.get_part("jaw_carriage_1")
    joint_0 = object_model.get_articulation("guide_to_jaw_0")
    joint_1 = object_model.get_articulation("guide_to_jaw_1")

    ctx.expect_gap(
        jaw_1,
        jaw_0,
        axis="x",
        positive_elem="pad",
        negative_elem="pad",
        min_gap=0.030,
        max_gap=0.040,
        name="rest pose leaves a pickup gap between pads",
    )
    ctx.expect_gap(
        jaw_0,
        body,
        axis="z",
        positive_elem="saddle",
        negative_elem="guide_0_front",
        max_gap=0.001,
        max_penetration=0.0,
        name="jaw 0 saddle rides on its guide",
    )
    ctx.expect_gap(
        jaw_1,
        body,
        axis="z",
        positive_elem="saddle",
        negative_elem="guide_1_front",
        max_gap=0.001,
        max_penetration=0.0,
        name="jaw 1 saddle rides on its guide",
    )
    ctx.expect_overlap(
        jaw_0,
        body,
        axes="xy",
        elem_a="saddle",
        elem_b="guide_0_front",
        min_overlap=0.006,
        name="jaw 0 stays captured over guide footprint",
    )
    ctx.expect_overlap(
        jaw_1,
        body,
        axes="xy",
        elem_a="saddle",
        elem_b="guide_1_front",
        min_overlap=0.006,
        name="jaw 1 stays captured over guide footprint",
    )

    rest_0 = ctx.part_world_position(jaw_0)
    rest_1 = ctx.part_world_position(jaw_1)
    travel = 0.016
    with ctx.pose({joint_0: travel, joint_1: travel}):
        ctx.expect_gap(
            jaw_1,
            jaw_0,
            axis="x",
            positive_elem="pad",
            negative_elem="pad",
            min_gap=0.002,
            max_gap=0.006,
            name="closed pose narrows but preserves the pickup gap",
        )
        ctx.expect_overlap(
            jaw_0,
            body,
            axes="xy",
            elem_a="saddle",
            elem_b="guide_0_front",
            min_overlap=0.006,
            name="jaw 0 remains on guide at full travel",
        )
        ctx.expect_overlap(
            jaw_1,
            body,
            axes="xy",
            elem_a="saddle",
            elem_b="guide_1_front",
            min_overlap=0.006,
            name="jaw 1 remains on guide at full travel",
        )
        closed_0 = ctx.part_world_position(jaw_0)
        closed_1 = ctx.part_world_position(jaw_1)

    inward_ok = (
        rest_0 is not None
        and rest_1 is not None
        and closed_0 is not None
        and closed_1 is not None
        and closed_0[0] > rest_0[0] + 0.012
        and closed_1[0] < rest_1[0] - 0.012
        and abs((closed_0[0] - rest_0[0]) - (rest_1[0] - closed_1[0])) < 1e-6
    )
    ctx.check(
        "two independent prismatic jaws move inward symmetrically",
        inward_ok,
        details=f"rest_0={rest_0}, rest_1={rest_1}, closed_0={closed_0}, closed_1={closed_1}",
    )

    return ctx.report()


object_model = build_object_model()
