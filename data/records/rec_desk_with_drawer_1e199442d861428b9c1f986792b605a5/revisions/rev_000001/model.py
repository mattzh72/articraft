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
    model = ArticulatedObject(name="compact_vanity_desk")

    painted_wood = model.material("painted_wood", rgba=(0.90, 0.87, 0.82, 1.0))
    oak_trim = model.material("oak_trim", rgba=(0.73, 0.58, 0.40, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    graphite = model.material("graphite", rgba=(0.26, 0.27, 0.29, 1.0))
    mirror_glass = model.material("mirror_glass", rgba=(0.76, 0.84, 0.90, 0.72))

    desk = model.part("desk")

    top_width = 0.92
    top_depth = 0.46
    top_thickness = 0.03
    desk_height = 0.76
    top_center_z = desk_height - top_thickness / 2.0
    top_surface_z = desk_height
    leg_size = 0.045
    leg_height = desk_height - top_thickness
    leg_center_z = leg_height / 2.0
    leg_center_x = top_width / 2.0 - 0.055 - leg_size / 2.0
    leg_center_y = top_depth / 2.0 - 0.050 - leg_size / 2.0

    desk.visual(
        Box((top_width, top_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_center_z)),
        material=painted_wood,
        name="top_panel",
    )

    for x_pos in (-leg_center_x, leg_center_x):
        for y_pos in (-leg_center_y, leg_center_y):
            desk.visual(
                Box((leg_size, leg_size, leg_height)),
                origin=Origin(xyz=(x_pos, y_pos, leg_center_z)),
                material=painted_wood,
                name=f"leg_{'front' if y_pos > 0.0 else 'rear'}_{0 if x_pos < 0.0 else 1}",
            )

    apron_height = 0.106
    apron_z = top_surface_z - top_thickness - apron_height / 2.0
    side_apron_x = leg_center_x - leg_size / 2.0 - 0.009
    desk.visual(
        Box((0.018, 0.295, apron_height)),
        origin=Origin(xyz=(-side_apron_x, 0.0, apron_z)),
        material=painted_wood,
        name="side_apron_0",
    )
    desk.visual(
        Box((0.018, 0.295, apron_height)),
        origin=Origin(xyz=(side_apron_x, 0.0, apron_z)),
        material=painted_wood,
        name="side_apron_1",
    )
    desk.visual(
        Box((0.734, 0.018, 0.096)),
        origin=Origin(xyz=(0.0, -0.182, top_surface_z - top_thickness - 0.048)),
        material=painted_wood,
        name="rear_apron",
    )
    desk.visual(
        Box((0.760, 0.036, 0.022)),
        origin=Origin(xyz=(0.0, -0.082, top_surface_z - top_thickness - 0.030)),
        material=oak_trim,
        name="rear_mount_beam",
    )

    runner_x = 0.324
    runner_center_y = -0.010
    runner_z = 0.610
    for index, x_pos in enumerate((-runner_x, runner_x)):
        desk.visual(
            Box((0.018, 0.330, 0.016)),
            origin=Origin(xyz=(x_pos, runner_center_y, runner_z)),
            material=graphite,
            name=f"runner_{index}",
        )
        desk.visual(
            Box((0.016, 0.330, 0.112)),
            origin=Origin(
                xyz=(
                    x_pos + 0.001 if x_pos > 0.0 else x_pos - 0.001,
                    runner_center_y,
                    0.674,
                )
            ),
            material=painted_wood,
            name=f"bay_panel_{index}",
        )

    bracket_y = -0.232
    hinge_axis_z = 0.788
    for index, x_pos in enumerate((-0.300, 0.300)):
        desk.visual(
            Box((0.020, 0.024, 0.028)),
            origin=Origin(xyz=(-0.314 if x_pos < 0.0 else 0.314, bracket_y - 0.009, top_surface_z + 0.014)),
            material=brushed_metal,
            name=f"mirror_bracket_{index}",
        )
    desk.visual(
        Box((0.670, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, bracket_y - 0.014, top_surface_z + 0.005)),
        material=brushed_metal,
        name="mirror_base_rail",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.700, 0.018, 0.120)),
        origin=Origin(),
        material=painted_wood,
        name="front_face",
    )
    drawer.visual(
        Box((0.012, 0.319, 0.090)),
        origin=Origin(xyz=(-0.304, -0.1675, -0.010)),
        material=oak_trim,
        name="side_wall_0",
    )
    drawer.visual(
        Box((0.012, 0.319, 0.090)),
        origin=Origin(xyz=(0.304, -0.1675, -0.010)),
        material=oak_trim,
        name="side_wall_1",
    )
    drawer.visual(
        Box((0.604, 0.014, 0.090)),
        origin=Origin(xyz=(0.0, -0.321, -0.010)),
        material=oak_trim,
        name="back_wall",
    )
    drawer.visual(
        Box((0.606, 0.319, 0.014)),
        origin=Origin(xyz=(0.0, -0.1675, -0.049)),
        material=oak_trim,
        name="bottom_panel",
    )
    for index, x_pos in enumerate((-runner_x, runner_x)):
        drawer.visual(
            Box((0.014, 0.300, 0.012)),
            origin=Origin(
                xyz=(
                    x_pos + 0.008 if x_pos < 0.0 else x_pos - 0.008,
                    -0.170,
                    -0.010,
                )
            ),
            material=graphite,
            name=f"runner_cleat_{index}",
        )

    drawer.visual(
        Box((0.180, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.031, 0.0)),
        material=brushed_metal,
        name="pull_bar",
    )
    drawer.visual(
        Box((0.014, 0.024, 0.020)),
        origin=Origin(xyz=(-0.072, 0.020, 0.0)),
        material=brushed_metal,
        name="pull_post_0",
    )
    drawer.visual(
        Box((0.014, 0.024, 0.020)),
        origin=Origin(xyz=(0.072, 0.020, 0.0)),
        material=brushed_metal,
        name="pull_post_1",
    )

    mirror = model.part("mirror")
    mirror.visual(
        Cylinder(radius=0.008, length=0.560),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="pivot_rod",
    )
    mirror.visual(
        Box((0.028, 0.018, 0.032)),
        origin=Origin(xyz=(-0.285, 0.006, 0.010)),
        material=brushed_metal,
        name="side_cheek_0",
    )
    mirror.visual(
        Box((0.028, 0.018, 0.032)),
        origin=Origin(xyz=(0.285, 0.006, 0.010)),
        material=brushed_metal,
        name="side_cheek_1",
    )
    mirror.visual(
        Box((0.800, 0.420, 0.028)),
        origin=Origin(xyz=(0.0, 0.210, -0.014)),
        material=painted_wood,
        name="panel_leaf",
    )
    mirror.visual(
        Box((0.700, 0.320, 0.004)),
        origin=Origin(xyz=(0.0, 0.210, -0.002)),
        material=mirror_glass,
        name="mirror_glass",
    )
    mirror.visual(
        Box((0.200, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.415, -0.008)),
        material=oak_trim,
        name="finger_rail",
    )

    model.articulation(
        "desk_to_drawer",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.223, 0.620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=0.220,
        ),
    )
    model.articulation(
        "desk_to_mirror",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=mirror,
        origin=Origin(xyz=(0.0, bracket_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.6,
            lower=0.0,
            upper=1.47,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk = object_model.get_part("desk")
    drawer = object_model.get_part("drawer")
    mirror = object_model.get_part("mirror")
    drawer_slide = object_model.get_articulation("desk_to_drawer")
    mirror_hinge = object_model.get_articulation("desk_to_mirror")
    mirror_open_angle = mirror_hinge.motion_limits.upper if mirror_hinge.motion_limits is not None else 1.47

    ctx.allow_overlap(
        desk,
        drawer,
        elem_a="runner_0",
        elem_b="runner_cleat_0",
        reason="The fixed cabinet slide and drawer-side member are simplified as solid nested runner proxies.",
    )
    ctx.allow_overlap(
        desk,
        drawer,
        elem_a="runner_1",
        elem_b="runner_cleat_1",
        reason="The fixed cabinet slide and drawer-side member are simplified as solid nested runner proxies.",
    )

    with ctx.pose({drawer_slide: 0.0, mirror_hinge: 0.0}):
        ctx.expect_gap(
            positive_link=desk,
            negative_link=drawer,
            axis="z",
            positive_elem="top_panel",
            negative_elem="front_face",
            min_gap=0.045,
            max_gap=0.065,
            name="drawer front sits below the desktop",
        )
        ctx.expect_overlap(
            drawer,
            desk,
            axes="x",
            elem_a="front_face",
            elem_b="top_panel",
            min_overlap=0.68,
            name="drawer spans most of the vanity width",
        )
        ctx.expect_gap(
            positive_link=mirror,
            negative_link=desk,
            axis="z",
            positive_elem="panel_leaf",
            negative_elem="top_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed mirror leaf rests on the desktop",
        )
        ctx.expect_overlap(
            mirror,
            desk,
            axes="xy",
            elem_a="panel_leaf",
            elem_b="top_panel",
            min_overlap=0.20,
            name="mirror folds over the vanity top when closed",
        )
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="runner_cleat_0",
            elem_b="runner_0",
            min_overlap=0.20,
            name="drawer runner stays engaged when closed",
        )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.220}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a="runner_cleat_0",
            elem_b="runner_0",
            min_overlap=0.025,
            name="drawer runner retains insertion at full extension",
        )

    ctx.check(
        "drawer extends forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > rest_drawer_pos[1] + 0.18,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    closed_mirror_aabb = ctx.part_element_world_aabb(mirror, elem="panel_leaf")
    with ctx.pose({mirror_hinge: mirror_open_angle}):
        open_mirror_aabb = ctx.part_element_world_aabb(mirror, elem="panel_leaf")
        ctx.expect_gap(
            positive_link=mirror,
            negative_link=desk,
            axis="z",
            positive_elem="finger_rail",
            negative_elem="top_panel",
            min_gap=0.30,
            name="opened mirror top rises above the desktop",
        )

    ctx.check(
        "mirror opens upward",
        closed_mirror_aabb is not None
        and open_mirror_aabb is not None
        and open_mirror_aabb[1][2] > closed_mirror_aabb[1][2] + 0.28,
        details=f"closed={closed_mirror_aabb}, open={open_mirror_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
