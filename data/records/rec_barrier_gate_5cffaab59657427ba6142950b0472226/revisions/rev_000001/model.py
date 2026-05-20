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
    model = ArticulatedObject(name="pivot_drop_arm_parking_barrier")

    concrete = model.material("weathered_concrete", rgba=(0.45, 0.45, 0.42, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.58, 0.57, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    white = model.material("white_powder_coat", rgba=(0.93, 0.92, 0.86, 1.0))
    red = model.material("red_reflective_panels", rgba=(0.85, 0.05, 0.03, 1.0))
    brass = model.material("brass_padlock", rgba=(0.86, 0.62, 0.20, 1.0))

    curb_frame = model.part("curb_frame")
    curb_frame.visual(
        Box((3.65, 0.46, 0.10)),
        origin=Origin(xyz=(1.55, 0.0, 0.05)),
        material=concrete,
        name="concrete_curb",
    )
    curb_frame.visual(
        Box((0.30, 0.30, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=galvanized,
        name="pivot_foot_plate",
    )
    curb_frame.visual(
        Box((0.16, 0.16, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        material=galvanized,
        name="pivot_post",
    )
    curb_frame.visual(
        Box((0.24, 0.24, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.22)),
        material=dark_steel,
        name="pivot_top_cap",
    )
    curb_frame.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.29)),
        material=dark_steel,
        name="vertical_spindle",
    )
    curb_frame.visual(
        Box((0.28, 0.28, 0.035)),
        origin=Origin(xyz=(3.20, 0.0, 0.1175)),
        material=galvanized,
        name="receiver_foot_plate",
    )
    curb_frame.visual(
        Box((0.16, 0.16, 1.25)),
        origin=Origin(xyz=(3.20, 0.0, 0.725)),
        material=galvanized,
        name="receiver_post",
    )
    curb_frame.visual(
        Box((0.030, 0.20, 0.24)),
        origin=Origin(xyz=(3.105, 0.0, 1.26)),
        material=dark_steel,
        name="socket_back_plate",
    )
    curb_frame.visual(
        Box((0.21, 0.030, 0.18)),
        origin=Origin(xyz=(3.005, -0.075, 1.26)),
        material=dark_steel,
        name="socket_side_0",
    )
    curb_frame.visual(
        Box((0.21, 0.030, 0.18)),
        origin=Origin(xyz=(3.005, 0.075, 1.26)),
        material=dark_steel,
        name="socket_side_1",
    )
    curb_frame.visual(
        Cylinder(radius=0.012, length=0.15),
        origin=Origin(xyz=(2.965, 0.0, 1.26), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="socket_pin",
    )

    arm = model.part("arm")
    arm.visual(
        Box((2.44, 0.11, 0.14)),
        origin=Origin(xyz=(1.38, 0.0, 0.0)),
        material=white,
        name="main_arm",
    )
    arm.visual(
        Box((0.07, 0.18, 0.17)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=white,
        name="pivot_end_plate",
    )
    arm.visual(
        Box((0.08, 0.13, 0.15)),
        origin=Origin(xyz=(2.62, 0.0, 0.0)),
        material=white,
        name="tip_block",
    )

    for index, x_pos in enumerate((0.48, 1.02, 1.56, 2.10)):
        arm.visual(
            Box((0.34, 0.008, 0.115)),
            origin=Origin(xyz=(x_pos, -0.055, 0.0)),
            material=red,
            name=f"front_reflector_{index}",
        )
        arm.visual(
            Box((0.34, 0.008, 0.115)),
            origin=Origin(xyz=(x_pos, 0.055, 0.0)),
            material=red,
            name=f"rear_reflector_{index}",
        )

    arm.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(2.68, -0.046, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle_0",
    )
    arm.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(2.68, 0.046, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle_1",
    )

    latch = model.part("hasp_latch")
    latch.visual(
        Cylinder(radius=0.018, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    latch.visual(
        Box((0.22, 0.034, 0.060)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=galvanized,
        name="hasp_plate",
    )
    latch.visual(
        Box((0.110, 0.034, 0.034)),
        origin=Origin(xyz=(0.280, 0.0, 0.041)),
        material=galvanized,
        name="hook_upper",
    )
    latch.visual(
        Box((0.110, 0.034, 0.034)),
        origin=Origin(xyz=(0.280, 0.0, -0.041)),
        material=galvanized,
        name="hook_lower",
    )
    latch.visual(
        Box((0.024, 0.034, 0.116)),
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        material=galvanized,
        name="hook_web",
    )
    latch.visual(
        Box((0.060, 0.026, 0.016)),
        origin=Origin(xyz=(0.240, -0.025, -0.045)),
        material=galvanized,
        name="padlock_eye",
    )
    latch.visual(
        Box((0.070, 0.032, 0.055)),
        origin=Origin(xyz=(0.240, -0.040, -0.125)),
        material=brass,
        name="lock_body",
    )
    latch.visual(
        Box((0.008, 0.010, 0.052)),
        origin=Origin(xyz=(0.215, -0.040, -0.076)),
        material=dark_steel,
        name="lock_shackle_0",
    )
    latch.visual(
        Box((0.008, 0.010, 0.052)),
        origin=Origin(xyz=(0.265, -0.040, -0.076)),
        material=dark_steel,
        name="lock_shackle_1",
    )
    latch.visual(
        Box((0.058, 0.010, 0.008)),
        origin=Origin(xyz=(0.240, -0.040, -0.055)),
        material=dark_steel,
        name="lock_shackle_top",
    )

    model.articulation(
        "arm_swing",
        ArticulationType.REVOLUTE,
        parent=curb_frame,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.26)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.6, lower=0.0, upper=pi / 2.0),
    )
    model.articulation(
        "hasp_hinge",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=latch,
        origin=Origin(xyz=(2.68, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    curb_frame = object_model.get_part("curb_frame")
    arm = object_model.get_part("arm")
    latch = object_model.get_part("hasp_latch")
    arm_swing = object_model.get_articulation("arm_swing")
    hasp_hinge = object_model.get_articulation("hasp_hinge")

    ctx.expect_gap(
        latch,
        curb_frame,
        axis="y",
        positive_elem="hook_web",
        negative_elem="socket_side_0",
        min_gap=0.025,
        max_gap=0.080,
        name="hasp hook clears lower socket cheek",
    )
    ctx.expect_gap(
        curb_frame,
        latch,
        axis="y",
        positive_elem="socket_side_1",
        negative_elem="hook_web",
        min_gap=0.025,
        max_gap=0.080,
        name="hasp hook clears upper socket cheek",
    )
    ctx.expect_overlap(
        latch,
        curb_frame,
        axes="xy",
        elem_a="hook_upper",
        elem_b="socket_pin",
        min_overlap=0.015,
        name="upper jaw surrounds fixed socket pin in projection",
    )
    ctx.expect_gap(
        latch,
        curb_frame,
        axis="z",
        positive_elem="hook_upper",
        negative_elem="socket_pin",
        min_gap=0.006,
        max_gap=0.020,
        name="upper hook jaw sits above socket pin",
    )
    ctx.expect_gap(
        curb_frame,
        latch,
        axis="z",
        positive_elem="socket_pin",
        negative_elem="hook_lower",
        min_gap=0.006,
        max_gap=0.020,
        name="lower hook jaw sits below socket pin",
    )

    rest_latch_origin = ctx.part_world_position(latch)
    with ctx.pose({arm_swing: pi / 2.0}):
        open_latch_origin = ctx.part_world_position(latch)
    ctx.check(
        "wide arm swings horizontally away from receiver",
        rest_latch_origin is not None
        and open_latch_origin is not None
        and open_latch_origin[1] > rest_latch_origin[1] + 2.4
        and open_latch_origin[0] < rest_latch_origin[0] - 2.4,
        details=f"rest={rest_latch_origin}, open={open_latch_origin}",
    )

    def _aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    rest_hook = ctx.part_element_world_aabb(latch, elem="hook_web")
    with ctx.pose({hasp_hinge: 0.90}):
        raised_hook = ctx.part_element_world_aabb(latch, elem="hook_web")
    rest_hook_z = _aabb_center_z(rest_hook)
    raised_hook_z = _aabb_center_z(raised_hook)
    ctx.check(
        "hasp latch flips upward on its hinge",
        rest_hook_z is not None and raised_hook_z is not None and raised_hook_z > rest_hook_z + 0.20,
        details=f"rest_hook_z={rest_hook_z}, raised_hook_z={raised_hook_z}",
    )

    return ctx.report()


object_model = build_object_model()
