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
    model = ArticulatedObject(name="gaming_desk")

    top_finish = model.material("top_finish", rgba=(0.11, 0.12, 0.13, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.19, 0.20, 0.22, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.15, 0.15, 0.16, 1.0))
    accent_finish = model.material("accent_finish", rgba=(0.66, 0.08, 0.09, 1.0))
    hardware_finish = model.material("hardware_finish", rgba=(0.50, 0.51, 0.53, 1.0))

    desk_width = 1.40
    desk_depth = 0.72
    top_thickness = 0.03
    top_center_z = 0.735

    desk = model.part("desk")
    desk.visual(
        Box((desk_width, desk_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_center_z)),
        material=top_finish,
        name="top",
    )

    desk.visual(
        Box((1.28, 0.05, 0.11)),
        origin=Origin(xyz=(0.0, 0.305, 0.666)),
        material=frame_finish,
        name="rear_apron",
    )
    desk.visual(
        Box((0.44, 0.05, 0.11)),
        origin=Origin(xyz=(-0.47, -0.305, 0.666)),
        material=frame_finish,
        name="front_apron_0",
    )
    desk.visual(
        Box((0.44, 0.05, 0.11)),
        origin=Origin(xyz=(0.47, -0.305, 0.666)),
        material=frame_finish,
        name="front_apron_1",
    )
    desk.visual(
        Box((0.05, 0.62, 0.10)),
        origin=Origin(xyz=(-0.635, 0.0, 0.67)),
        material=frame_finish,
        name="side_rail_0",
    )
    desk.visual(
        Box((0.05, 0.62, 0.10)),
        origin=Origin(xyz=(0.635, 0.0, 0.67)),
        material=frame_finish,
        name="side_rail_1",
    )

    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.62, -0.29),
            (0.62, -0.29),
            (-0.62, 0.29),
            (0.62, 0.29),
        )
    ):
        desk.visual(
            Box((0.06, 0.06, 0.70)),
            origin=Origin(xyz=(x_pos, y_pos, 0.35)),
            material=frame_finish,
            name=f"leg_{index}",
        )

    runner_positions = (-0.196, 0.196)
    for index, x_pos in enumerate(runner_positions):
        desk.visual(
            Box((0.022, 0.24, 0.028)),
            origin=Origin(xyz=(x_pos, -0.10, 0.706)),
            material=hardware_finish,
            name=f"runner_{index}",
        )

    desk.visual(
        Box((0.09, 0.05, 0.012)),
        origin=Origin(xyz=(0.611, 0.18, 0.614)),
        material=frame_finish,
        name="hook_plate",
    )
    for index, y_pos in enumerate((0.164, 0.196)):
        desk.visual(
            Cylinder(radius=0.007, length=0.016),
            origin=Origin(xyz=(0.582, y_pos, 0.602), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hardware_finish,
            name=f"hook_ear_{index}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.392, 0.290, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=drawer_finish,
        name="bottom",
    )
    drawer.visual(
        Box((0.008, 0.290, 0.046)),
        origin=Origin(xyz=(-0.196, 0.0, -0.005)),
        material=drawer_finish,
        name="wall_0",
    )
    drawer.visual(
        Box((0.008, 0.290, 0.046)),
        origin=Origin(xyz=(0.196, 0.0, -0.005)),
        material=drawer_finish,
        name="wall_1",
    )
    drawer.visual(
        Box((0.392, 0.010, 0.046)),
        origin=Origin(xyz=(0.0, 0.145, -0.005)),
        material=drawer_finish,
        name="back_wall",
    )
    drawer.visual(
        Box((0.392, 0.010, 0.046)),
        origin=Origin(xyz=(0.0, -0.145, -0.005)),
        material=drawer_finish,
        name="front_wall",
    )
    drawer.visual(
        Box((0.42, 0.016, 0.08)),
        origin=Origin(xyz=(0.0, -0.151, 0.0)),
        material=accent_finish,
        name="front",
    )
    for index, x_pos in enumerate((-0.196, 0.196)):
        drawer.visual(
            Box((0.016, 0.22, 0.012)),
            origin=Origin(xyz=(x_pos, 0.02, 0.020)),
            material=hardware_finish,
            name=f"slide_{index}",
        )

    hook = model.part("hook")
    hook.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_finish,
        name="knuckle",
    )
    hook.visual(
        Cylinder(radius=0.007, length=0.120),
        origin=Origin(xyz=(-0.060, 0.0, -0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_finish,
        name="arm",
    )
    hook.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(xyz=(-0.114, 0.0, -0.028)),
        material=hardware_finish,
        name="lip",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.14, 0.666)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.15,
        ),
    )
    model.articulation(
        "hook_hinge",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=hook,
        origin=Origin(xyz=(0.582, 0.18, 0.602)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk = object_model.get_part("desk")
    drawer = object_model.get_part("drawer")
    hook = object_model.get_part("hook")
    drawer_slide = object_model.get_articulation("drawer_slide")
    hook_hinge = object_model.get_articulation("hook_hinge")

    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits else None
    hook_upper = hook_hinge.motion_limits.upper if hook_hinge.motion_limits else None

    ctx.expect_contact(
        desk,
        drawer,
        elem_a="runner_0",
        elem_b="slide_0",
        name="left drawer slide bears on its runner at rest",
    )
    ctx.expect_contact(
        desk,
        drawer,
        elem_a="runner_1",
        elem_b="slide_1",
        name="right drawer slide bears on its runner at rest",
    )
    ctx.expect_within(
        drawer,
        desk,
        axes="x",
        inner_elem="slide_0",
        outer_elem="runner_0",
        margin=0.002,
        name="left slide stays centered under its runner",
    )
    ctx.expect_within(
        drawer,
        desk,
        axes="x",
        inner_elem="slide_1",
        outer_elem="runner_1",
        margin=0.002,
        name="right slide stays centered under its runner",
    )
    ctx.expect_overlap(
        drawer,
        desk,
        axes="y",
        elem_a="slide_0",
        elem_b="runner_0",
        min_overlap=0.20,
        name="left slide remains deeply inserted when closed",
    )
    ctx.expect_overlap(
        drawer,
        desk,
        axes="y",
        elem_a="slide_1",
        elem_b="runner_1",
        min_overlap=0.20,
        name="right slide remains deeply inserted when closed",
    )
    ctx.expect_gap(
        desk,
        hook,
        axis="z",
        positive_elem="side_rail_1",
        negative_elem="arm",
        min_gap=0.004,
        max_gap=0.03,
        name="hook stores just beneath the side rail",
    )
    ctx.expect_contact(
        desk,
        hook,
        elem_a="hook_ear_0",
        elem_b="knuckle",
        name="hook knuckle seats against the first hinge ear",
    )
    ctx.expect_contact(
        desk,
        hook,
        elem_a="hook_ear_1",
        elem_b="knuckle",
        name="hook knuckle seats against the second hinge ear",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: drawer_upper}):
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="slide_0",
            elem_b="runner_0",
            min_overlap=0.055,
            name="left slide retains insertion when extended",
        )
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="slide_1",
            elem_b="runner_1",
            min_overlap=0.055,
            name="right slide retains insertion when extended",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends toward the front of the desk",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.12,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    closed_arm_center = _aabb_center(ctx.part_element_world_aabb(hook, elem="arm"))
    with ctx.pose({hook_hinge: hook_upper}):
        open_arm_center = _aabb_center(ctx.part_element_world_aabb(hook, elem="arm"))

    ctx.check(
        "hook rotates downward when deployed",
        closed_arm_center is not None
        and open_arm_center is not None
        and open_arm_center[2] < closed_arm_center[2] - 0.05,
        details=f"closed={closed_arm_center}, open={open_arm_center}",
    )

    return ctx.report()


object_model = build_object_model()
