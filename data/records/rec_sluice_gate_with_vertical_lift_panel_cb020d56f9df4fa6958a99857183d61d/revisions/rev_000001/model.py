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
    model = ArticulatedObject(name="compact_sluice_mechanism")

    frame_paint = model.material("frame_paint", rgba=(0.22, 0.25, 0.27, 1.0))
    barrier_paint = model.material("barrier_paint", rgba=(0.37, 0.43, 0.45, 1.0))
    wheel_paint = model.material("wheel_paint", rgba=(0.72, 0.16, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.69, 0.71, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.46, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=frame_paint,
        name="bottom_sill",
    )
    frame.visual(
        Box((0.08, 0.10, 0.60)),
        origin=Origin(xyz=(-0.19, 0.0, 0.38)),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((0.08, 0.10, 0.60)),
        origin=Origin(xyz=(0.19, 0.0, 0.38)),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((0.46, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        material=frame_paint,
        name="top_beam",
    )
    for name, x_pos, y_pos in (
        ("left_front_guide", -0.141, -0.039),
        ("left_rear_guide", -0.141, 0.039),
        ("right_front_guide", 0.141, -0.039),
        ("right_rear_guide", 0.141, 0.039),
    ):
        frame.visual(
            Box((0.018, 0.022, 0.50)),
            origin=Origin(xyz=(x_pos, y_pos, 0.33)),
            material=frame_paint,
            name=name,
        )

    frame.visual(
        Box((0.20, 0.012, 0.16)),
        origin=Origin(xyz=(0.0, -0.044, 0.80)),
        material=frame_paint,
        name="front_bracket",
    )
    frame.visual(
        Box((0.20, 0.012, 0.16)),
        origin=Origin(xyz=(0.0, 0.044, 0.80)),
        material=frame_paint,
        name="rear_bracket",
    )
    frame.visual(
        Box((0.20, 0.10, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.8675)),
        material=frame_paint,
        name="bracket_cap",
    )
    frame.visual(
        Box((0.020, 0.010, 0.030)),
        origin=Origin(xyz=(0.088, -0.040, 0.806)),
        material=frame_paint,
        name="pawl_ear",
    )

    barrier = model.part("barrier")
    barrier.visual(
        Box((0.254, 0.018, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=barrier_paint,
        name="plate",
    )
    barrier.visual(
        Box((0.210, 0.026, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=steel,
        name="stiffener",
    )
    barrier.visual(
        Box((0.060, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=steel,
        name="lift_lug",
    )
    for name, x_pos, y_pos in (
        ("left_front_shoe", -0.127, -0.023),
        ("left_rear_shoe", -0.127, 0.023),
        ("right_front_shoe", 0.127, -0.023),
        ("right_rear_shoe", 0.127, 0.023),
    ):
        barrier.visual(
            Box((0.010, 0.030, 0.34)),
            origin=Origin(xyz=(x_pos, y_pos, 0.21)),
            material=steel,
            name=name,
        )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.062, length=0.014),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_paint,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.010, length=0.076),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    wheel.visual(
        Box((0.090, 0.008, 0.008)),
        material=steel,
        name="spoke_x",
    )
    wheel.visual(
        Box((0.008, 0.008, 0.090)),
        material=steel,
        name="spoke_z",
    )
    wheel.visual(
        Box((0.030, 0.010, 0.010)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=steel,
        name="handle_arm",
    )
    wheel.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="grip",
    )

    pawl = model.part("pawl")
    pawl.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot",
    )
    pawl.visual(
        Box((0.076, 0.010, 0.016)),
        origin=Origin(xyz=(-0.032, 0.0, -0.020), rpy=(0.0, -0.45, 0.0)),
        material=steel,
        name="body",
    )
    pawl.visual(
        Box((0.016, 0.010, 0.024)),
        origin=Origin(xyz=(-0.060, 0.0, -0.033), rpy=(0.0, -0.45, 0.0)),
        material=steel,
        name="tooth",
    )

    model.articulation(
        "frame_to_barrier",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=barrier,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.12,
            lower=0.0,
            upper=0.135,
        ),
    )
    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )
    model.articulation(
        "frame_to_pawl",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=pawl,
        origin=Origin(xyz=(0.088, -0.030, 0.806)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.25,
            upper=0.55,
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    barrier = object_model.get_part("barrier")
    wheel = object_model.get_part("wheel")
    pawl = object_model.get_part("pawl")

    barrier_slide = object_model.get_articulation("frame_to_barrier")
    wheel_spin = object_model.get_articulation("frame_to_wheel")
    pawl_joint = object_model.get_articulation("frame_to_pawl")

    ctx.expect_overlap(
        barrier,
        frame,
        axes="xz",
        min_overlap=0.18,
        name="barrier stays within the portal span",
    )
    ctx.expect_gap(
        barrier,
        frame,
        axis="z",
        positive_elem="plate",
        negative_elem="bottom_sill",
        min_gap=0.003,
        max_gap=0.015,
        name="closed barrier hovers just above the sill",
    )
    ctx.expect_gap(
        frame,
        barrier,
        axis="z",
        positive_elem="top_beam",
        negative_elem="plate",
        min_gap=0.10,
        name="closed barrier clears the top beam",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem="rim",
        negative_elem="top_beam",
        min_gap=0.006,
        name="wheel clears the frame beam",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="bracket_cap",
        negative_elem="rim",
        min_gap=0.002,
        name="wheel clears the bracket cap",
    )

    rest_barrier_pos = ctx.part_world_position(barrier)
    with ctx.pose({barrier_slide: 0.135}):
        raised_barrier_pos = ctx.part_world_position(barrier)
        ctx.expect_gap(
            frame,
            barrier,
            axis="z",
            positive_elem="top_beam",
            negative_elem="plate",
            min_gap=0.015,
            name="raised barrier still clears the top beam",
        )
        ctx.expect_overlap(
            barrier,
            frame,
            axes="z",
            min_overlap=0.35,
            name="raised barrier remains retained in the frame",
        )

    ctx.check(
        "barrier lifts upward",
        rest_barrier_pos is not None
        and raised_barrier_pos is not None
        and raised_barrier_pos[2] > rest_barrier_pos[2] + 0.10,
        details=f"rest={rest_barrier_pos}, raised={raised_barrier_pos}",
    )

    rest_grip = _aabb_center(ctx.part_element_world_aabb(wheel, elem="grip"))
    with ctx.pose({wheel_spin: 1.2}):
        turned_grip = _aabb_center(ctx.part_element_world_aabb(wheel, elem="grip"))
    ctx.check(
        "wheel grip orbits around the axle",
        rest_grip is not None
        and turned_grip is not None
        and (
            abs(turned_grip[0] - rest_grip[0]) > 0.015
            or abs(turned_grip[2] - rest_grip[2]) > 0.015
        ),
        details=f"rest={rest_grip}, turned={turned_grip}",
    )

    rest_tooth = _aabb_center(ctx.part_element_world_aabb(pawl, elem="tooth"))
    with ctx.pose({pawl_joint: 0.45}):
        lifted_tooth = _aabb_center(ctx.part_element_world_aabb(pawl, elem="tooth"))
    ctx.check(
        "pawl tooth swings clear when lifted",
        rest_tooth is not None
        and lifted_tooth is not None
        and lifted_tooth[2] > rest_tooth[2] + 0.01
        and abs(lifted_tooth[0] - rest_tooth[0]) > 0.005,
        details=f"rest={rest_tooth}, lifted={lifted_tooth}",
    )

    return ctx.report()


object_model = build_object_model()
