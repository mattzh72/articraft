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
    model = ArticulatedObject(name="drafting_desk")

    frame_steel = model.material("frame_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    wood_top = model.material("wood_top", rgba=(0.73, 0.61, 0.42, 1.0))
    wood_drawer = model.material("wood_drawer", rgba=(0.60, 0.47, 0.30, 1.0))
    drawer_interior = model.material("drawer_interior", rgba=(0.42, 0.32, 0.20, 1.0))
    pull_metal = model.material("pull_metal", rgba=(0.74, 0.76, 0.78, 1.0))
    glide_dark = model.material("glide_dark", rgba=(0.14, 0.15, 0.17, 1.0))

    frame = model.part("frame")
    leg_specs = (
        ("rear_leg_0", -0.53, -0.31, 0.38, 0.06, 0.07, 0.76),
        ("rear_leg_1", 0.53, -0.31, 0.38, 0.06, 0.07, 0.76),
        ("front_leg_0", -0.51, 0.31, 0.37, 0.055, 0.065, 0.74),
        ("front_leg_1", 0.51, 0.31, 0.37, 0.055, 0.065, 0.74),
    )
    for name, x_pos, y_pos, z_pos, size_x, size_y, size_z in leg_specs:
        frame.visual(
            Box((size_x, size_y, size_z)),
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            material=frame_steel,
            name=name,
        )

    frame.visual(
        Box((1.12, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.35, 0.74)),
        material=frame_steel,
        name="rear_beam",
    )
    frame.visual(
        Box((0.96, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.29, 0.75)),
        material=frame_steel,
        name="front_rest",
    )
    frame.visual(
        Box((0.74, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.35, 0.735)),
        material=frame_steel,
        name="front_apron",
    )
    frame.visual(
        Box((0.18, 0.03, 0.11)),
        origin=Origin(xyz=(-0.41, 0.35, 0.665)),
        material=frame_steel,
        name="apron_0",
    )
    frame.visual(
        Box((0.18, 0.03, 0.11)),
        origin=Origin(xyz=(0.41, 0.35, 0.665)),
        material=frame_steel,
        name="apron_1",
    )

    for index, x_pos in enumerate((-0.35, 0.35)):
        frame.visual(
            Box((0.05, 0.67, 0.07)),
            origin=Origin(xyz=(x_pos, -0.025, 0.705)),
            material=frame_steel,
            name=f"side_beam_{index}",
        )
        frame.visual(
            Box((0.016, 0.50, 0.018)),
            origin=Origin(xyz=(math.copysign(0.320, x_pos), 0.07, 0.634)),
            material=glide_dark,
            name=f"fixed_runner_{index}",
        )
        frame.visual(
            Box((0.05, 0.10, 0.09)),
            origin=Origin(xyz=(x_pos, 0.295, 0.668)),
            material=frame_steel,
            name=f"front_bracket_{index}",
        )

    frame.visual(
        Box((0.06, 0.64, 0.06)),
        origin=Origin(xyz=(-0.50, 0.0, 0.18)),
        material=frame_steel,
        name="side_stretcher_0",
    )
    frame.visual(
        Box((0.06, 0.64, 0.06)),
        origin=Origin(xyz=(0.50, 0.0, 0.18)),
        material=frame_steel,
        name="side_stretcher_1",
    )
    frame.visual(
        Box((0.96, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=frame_steel,
        name="center_stretcher",
    )
    frame.visual(
        Box((0.96, 0.04, 0.035)),
        origin=Origin(xyz=(0.0, 0.23, 0.19)),
        material=frame_steel,
        name="foot_rail",
    )
    frame.visual(
        Box((1.06, 0.04, 0.035)),
        origin=Origin(xyz=(0.0, -0.29, 0.19)),
        material=frame_steel,
        name="rear_lower_rail",
    )

    top = model.part("top")
    top.visual(
        Box((1.24, 0.76, 0.03)),
        origin=Origin(xyz=(0.0, 0.38, 0.015)),
        material=wood_top,
        name="slab",
    )
    top.visual(
        Box((1.06, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, 0.744, 0.039)),
        material=wood_top,
        name="paper_stop",
    )
    for index, x_pos in enumerate((-0.50, 0.50)):
        top.visual(
            Box((0.06, 0.44, 0.05)),
            origin=Origin(xyz=(math.copysign(0.43, x_pos), 0.37, -0.025)),
            material=frame_steel,
            name=f"side_cleat_{index}",
        )
    drawer = model.part("drawer")
    drawer.visual(
        Box((0.60, 0.022, 0.16)),
        origin=Origin(xyz=(0.0, 0.011, -0.08)),
        material=wood_drawer,
        name="front_panel",
    )
    drawer.visual(
        Box((0.016, 0.40, 0.09)),
        origin=Origin(xyz=(-0.290, -0.20, -0.045)),
        material=drawer_interior,
        name="side_wall_0",
    )
    drawer.visual(
        Box((0.016, 0.40, 0.09)),
        origin=Origin(xyz=(0.290, -0.20, -0.045)),
        material=drawer_interior,
        name="side_wall_1",
    )
    drawer.visual(
        Box((0.564, 0.016, 0.09)),
        origin=Origin(xyz=(0.0, -0.392, -0.045)),
        material=drawer_interior,
        name="back_wall",
    )
    drawer.visual(
        Box((0.564, 0.40, 0.012)),
        origin=Origin(xyz=(0.0, -0.20, -0.084)),
        material=drawer_interior,
        name="bottom_panel",
    )
    for index, x_pos in enumerate((-0.304, 0.304)):
        drawer.visual(
            Box((0.012, 0.44, 0.022)),
            origin=Origin(xyz=(math.copysign(0.312, x_pos), -0.22, -0.059)),
            material=glide_dark,
            name=f"moving_runner_{index}",
        )
        drawer.visual(
            Box((0.014, 0.10, 0.06)),
            origin=Origin(xyz=(math.copysign(0.302, x_pos), -0.09, -0.055)),
            material=drawer_interior,
            name=f"runner_cleat_{index}",
        )

    for index, x_pos in enumerate((-0.075, 0.075)):
        drawer.visual(
            Cylinder(radius=0.0055, length=0.024),
            origin=Origin(
                xyz=(x_pos, 0.021, -0.075),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pull_metal,
            name=f"pull_post_{index}",
        )
    drawer.visual(
        Cylinder(radius=0.007, length=0.18),
        origin=Origin(
            xyz=(0.0, 0.035, -0.075),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pull_metal,
        name="pull_bar",
    )

    model.articulation(
        "frame_to_top",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=top,
        origin=Origin(xyz=(0.0, -0.35, 0.77)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "frame_to_drawer",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.365, 0.703)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.30,
            lower=0.0,
            upper=0.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    top = object_model.get_part("top")
    drawer = object_model.get_part("drawer")
    top_hinge = object_model.get_articulation("frame_to_top")
    drawer_slide = object_model.get_articulation("frame_to_drawer")

    ctx.allow_overlap(
        drawer,
        frame,
        elem_a="moving_runner_0",
        elem_b="fixed_runner_0",
        reason="The fixed and moving drawer runners are simplified nested rail members that intentionally occupy the same guided slide path.",
    )
    ctx.allow_overlap(
        drawer,
        frame,
        elem_a="moving_runner_1",
        elem_b="fixed_runner_1",
        reason="The fixed and moving drawer runners are simplified nested rail members that intentionally occupy the same guided slide path.",
    )

    ctx.expect_gap(
        top,
        frame,
        axis="z",
        positive_elem="slab",
        negative_elem="front_rest",
        max_gap=0.002,
        max_penetration=0.0,
        name="top rests on the front support rail when closed",
    )
    ctx.expect_gap(
        drawer,
        frame,
        axis="y",
        positive_elem="front_panel",
        negative_elem="front_apron",
        max_gap=0.022,
        min_gap=0.0,
        name="drawer front sits flush ahead of the apron",
    )
    ctx.expect_within(
        drawer,
        frame,
        axes="xz",
        inner_elem="moving_runner_0",
        outer_elem="fixed_runner_0",
        margin=0.012,
        name="drawer runner stays aligned to the left fixed rail",
    )

    closed_stop_aabb = ctx.part_element_world_aabb(top, elem="paper_stop")
    closed_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({top_hinge: 0.95}):
        open_stop_aabb = ctx.part_element_world_aabb(top, elem="paper_stop")

    with ctx.pose({drawer_slide: 0.26}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_within(
            drawer,
            frame,
            axes="xz",
            inner_elem="moving_runner_1",
            outer_elem="fixed_runner_1",
            margin=0.012,
            name="drawer runner stays aligned to the right fixed rail when extended",
        )
        ctx.expect_overlap(
            drawer,
            frame,
            axes="y",
            elem_a="moving_runner_1",
            elem_b="fixed_runner_1",
            min_overlap=0.12,
            name="drawer keeps retained overlap on the runner at full extension",
        )

    closed_stop_z = None if closed_stop_aabb is None else closed_stop_aabb[1][2]
    open_stop_z = None if open_stop_aabb is None else open_stop_aabb[1][2]
    ctx.check(
        "top front edge rises in the open pose",
        closed_stop_z is not None and open_stop_z is not None and open_stop_z > closed_stop_z + 0.28,
        details=f"closed_stop_z={closed_stop_z}, open_stop_z={open_stop_z}",
    )
    ctx.check(
        "drawer extends forward from the desk",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > closed_drawer_pos[1] + 0.20,
        details=f"closed_drawer_pos={closed_drawer_pos}, extended_drawer_pos={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
