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


def add_drawer(
    part,
    *,
    front_width: float,
    front_height: float,
    front_thickness: float,
    body_width: float,
    body_depth: float,
    side_height: float,
    wall_thickness: float,
    bottom_thickness: float,
    box_center_z: float,
    front_material,
    body_material,
    metal_material,
) -> None:
    part.visual(
        Box((front_width, front_thickness, front_height)),
        material=front_material,
        name="front",
    )

    side_center_y = front_thickness / 2.0 + body_depth / 2.0

    part.visual(
        Box((wall_thickness, body_depth, side_height)),
        origin=Origin(xyz=(-body_width / 2.0 + wall_thickness / 2.0, side_center_y, box_center_z)),
        material=body_material,
        name="side_0",
    )
    part.visual(
        Box((wall_thickness, body_depth, side_height)),
        origin=Origin(xyz=(body_width / 2.0 - wall_thickness / 2.0, side_center_y, box_center_z)),
        material=body_material,
        name="side_1",
    )
    part.visual(
        Box((body_width - 2.0 * wall_thickness, wall_thickness, side_height)),
        origin=Origin(xyz=(0.0, front_thickness / 2.0 + body_depth - wall_thickness / 2.0, box_center_z)),
        material=body_material,
        name="back",
    )
    part.visual(
        Box((body_width - 2.0 * wall_thickness, body_depth, bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                side_center_y,
                box_center_z - side_height / 2.0 + bottom_thickness / 2.0,
            )
        ),
        material=body_material,
        name="bottom",
    )
    for index, x_pos in enumerate(
        (-body_width / 2.0 - 0.002, body_width / 2.0 + 0.002)
    ):
        part.visual(
            Box((0.004, body_depth, 0.024)),
            origin=Origin(xyz=(x_pos, side_center_y, box_center_z)),
            material=metal_material,
            name=f"runner_{index}",
        )

    handle_post_x = 0.042
    handle_post_depth = 0.020
    handle_post_height = 0.020
    handle_bar_depth = 0.012
    handle_bar_height = 0.016

    for index, x_pos in enumerate((-handle_post_x, handle_post_x)):
        part.visual(
            Box((0.012, handle_post_depth, handle_post_height)),
            origin=Origin(xyz=(x_pos, -front_thickness / 2.0 - handle_post_depth / 2.0, 0.0)),
            material=metal_material,
            name=f"post_{index}",
        )

    part.visual(
        Cylinder(radius=handle_bar_height / 2.0, length=0.104),
        origin=Origin(
            xyz=(0.0, -front_thickness / 2.0 - handle_post_depth - handle_bar_depth / 2.0, 0.0),
            rpy=(math.pi / 2.0, math.pi / 2.0, 0.0),
        ),
        material=metal_material,
        name="pull",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sewing_workstation_desk")

    oak = model.material("oak", rgba=(0.78, 0.71, 0.58, 1.0))
    painted = model.material("painted", rgba=(0.94, 0.94, 0.95, 1.0))
    birch = model.material("birch", rgba=(0.82, 0.76, 0.64, 1.0))
    metal = model.material("metal", rgba=(0.54, 0.56, 0.60, 1.0))

    desk_height = 0.760
    main_width = 0.980
    desk_depth = 0.580
    leaf_width = 0.340
    top_thickness = 0.036
    panel_thickness = 0.024
    shelf_thickness = 0.018
    overhang = 0.012
    frame_outer_x = main_width / 2.0 - overhang
    drawer_bay_outer_width = 0.340
    drawer_opening_width = drawer_bay_outer_width - 2.0 * panel_thickness
    drawer_front_width = 0.286
    drawer_front_height = 0.122
    drawer_front_thickness = 0.020
    drawer_body_width = drawer_opening_width - 0.014
    drawer_body_depth = 0.440
    drawer_side_height = 0.090
    drawer_wall_thickness = 0.012
    drawer_bottom_thickness = 0.012
    drawer_box_center_z = 0.008

    left_panel_center_x = -frame_outer_x + panel_thickness / 2.0
    right_panel_center_x = frame_outer_x - panel_thickness / 2.0
    partition_outer_face_x = -frame_outer_x + drawer_bay_outer_width
    partition_center_x = partition_outer_face_x - panel_thickness / 2.0
    drawer_bay_center_x = (-frame_outer_x + partition_outer_face_x) / 2.0

    desk_frame = model.part("desk_frame")
    desk_frame.visual(
        Box((main_width, desk_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, desk_height - top_thickness / 2.0)),
        material=oak,
        name="main_top",
    )
    desk_frame.visual(
        Box((panel_thickness, desk_depth, desk_height - top_thickness)),
        origin=Origin(xyz=(left_panel_center_x, 0.0, (desk_height - top_thickness) / 2.0)),
        material=painted,
        name="left_side",
    )
    desk_frame.visual(
        Box((panel_thickness, desk_depth, desk_height - top_thickness)),
        origin=Origin(xyz=(right_panel_center_x, 0.0, (desk_height - top_thickness) / 2.0)),
        material=painted,
        name="right_side",
    )
    desk_frame.visual(
        Box((panel_thickness, 0.520, desk_height - top_thickness)),
        origin=Origin(xyz=(partition_center_x, 0.0, (desk_height - top_thickness) / 2.0)),
        material=painted,
        name="drawer_partition",
    )
    desk_frame.visual(
        Box((drawer_opening_width, 0.460, shelf_thickness)),
        origin=Origin(xyz=(drawer_bay_center_x, -0.020, 0.444)),
        material=painted,
        name="drawer_floor",
    )
    desk_frame.visual(
        Box((drawer_opening_width, 0.460, shelf_thickness)),
        origin=Origin(xyz=(drawer_bay_center_x, -0.020, 0.582)),
        material=painted,
        name="drawer_divider",
    )
    left_inner_face_x = left_panel_center_x + panel_thickness / 2.0
    partition_inner_face_x = partition_center_x - panel_thickness / 2.0
    runner_center_y = -desk_depth / 2.0 + drawer_front_thickness + drawer_body_depth / 2.0
    for prefix, z_pos in (("top", 0.648 + drawer_box_center_z), ("bottom", 0.516 + drawer_box_center_z)):
        desk_frame.visual(
            Box((0.003, drawer_body_depth, 0.024)),
            origin=Origin(xyz=(left_inner_face_x + 0.0015, runner_center_y, z_pos)),
            material=metal,
            name=f"{prefix}_runner_0",
        )
        desk_frame.visual(
            Box((0.003, drawer_body_depth, 0.024)),
            origin=Origin(xyz=(partition_inner_face_x - 0.0015, runner_center_y, z_pos)),
            material=metal,
            name=f"{prefix}_runner_1",
        )

    apron_width = right_panel_center_x - panel_thickness / 2.0 - partition_outer_face_x
    apron_center_x = (partition_outer_face_x + (right_panel_center_x - panel_thickness / 2.0)) / 2.0
    desk_frame.visual(
        Box((apron_width, 0.018, 0.064)),
        origin=Origin(xyz=(apron_center_x, -desk_depth / 2.0 + 0.009, desk_height - top_thickness - 0.032)),
        material=painted,
        name="front_apron",
    )
    desk_frame.visual(
        Box((apron_width, 0.018, 0.260)),
        origin=Origin(xyz=(apron_center_x, desk_depth / 2.0 - 0.009, 0.280)),
        material=painted,
        name="rear_panel",
    )
    for index, y_pos in enumerate((-0.165, 0.165)):
        desk_frame.visual(
            Cylinder(radius=0.010, length=0.150),
            origin=Origin(xyz=(main_width / 2.0 + 0.008, y_pos, desk_height - top_thickness + 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"hinge_knuckle_{index}",
        )

    top_drawer = model.part("drawer_top")
    add_drawer(
        top_drawer,
        front_width=drawer_front_width,
        front_height=drawer_front_height,
        front_thickness=drawer_front_thickness,
        body_width=drawer_body_width,
        body_depth=drawer_body_depth,
        side_height=drawer_side_height,
        wall_thickness=drawer_wall_thickness,
        bottom_thickness=drawer_bottom_thickness,
        box_center_z=drawer_box_center_z,
        front_material=painted,
        body_material=birch,
        metal_material=metal,
    )

    bottom_drawer = model.part("drawer_bottom")
    add_drawer(
        bottom_drawer,
        front_width=drawer_front_width,
        front_height=drawer_front_height,
        front_thickness=drawer_front_thickness,
        body_width=drawer_body_width,
        body_depth=drawer_body_depth,
        side_height=drawer_side_height,
        wall_thickness=drawer_wall_thickness,
        bottom_thickness=drawer_bottom_thickness,
        box_center_z=drawer_box_center_z,
        front_material=painted,
        body_material=birch,
        metal_material=metal,
    )

    side_leaf = model.part("side_leaf")
    side_leaf.visual(
        Box((top_thickness, desk_depth, leaf_width)),
        origin=Origin(xyz=(top_thickness / 2.0, 0.0, -leaf_width / 2.0)),
        material=oak,
        name="panel",
    )
    side_leaf.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(xyz=(0.002, 0.0, -0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )

    drawer_limits = MotionLimits(effort=80.0, velocity=0.350, lower=0.0, upper=0.280)

    model.articulation(
        "top_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=desk_frame,
        child=top_drawer,
        origin=Origin(xyz=(drawer_bay_center_x, -desk_depth / 2.0 + drawer_front_thickness / 2.0, 0.648)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=drawer_limits,
    )
    model.articulation(
        "bottom_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=desk_frame,
        child=bottom_drawer,
        origin=Origin(xyz=(drawer_bay_center_x, -desk_depth / 2.0 + drawer_front_thickness / 2.0, 0.516)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.350, lower=0.0, upper=0.280),
    )
    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=desk_frame,
        child=side_leaf,
        origin=Origin(xyz=(main_width / 2.0 + 0.006, 0.0, desk_height - top_thickness)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.6,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk_frame = object_model.get_part("desk_frame")
    top_drawer = object_model.get_part("drawer_top")
    bottom_drawer = object_model.get_part("drawer_bottom")
    side_leaf = object_model.get_part("side_leaf")

    top_drawer_slide = object_model.get_articulation("top_drawer_slide")
    bottom_drawer_slide = object_model.get_articulation("bottom_drawer_slide")
    leaf_hinge = object_model.get_articulation("leaf_hinge")

    top_drawer_rest = ctx.part_world_position(top_drawer)
    bottom_drawer_rest = ctx.part_world_position(bottom_drawer)

    with ctx.pose({top_drawer_slide: top_drawer_slide.motion_limits.upper}):
        top_drawer_extended = ctx.part_world_position(top_drawer)
    with ctx.pose({bottom_drawer_slide: bottom_drawer_slide.motion_limits.upper}):
        bottom_drawer_extended = ctx.part_world_position(bottom_drawer)

    ctx.check(
        "top drawer extends forward",
        top_drawer_rest is not None
        and top_drawer_extended is not None
        and top_drawer_extended[1] < top_drawer_rest[1] - 0.220,
        details=f"rest={top_drawer_rest}, extended={top_drawer_extended}",
    )
    ctx.check(
        "bottom drawer extends forward",
        bottom_drawer_rest is not None
        and bottom_drawer_extended is not None
        and bottom_drawer_extended[1] < bottom_drawer_rest[1] - 0.220,
        details=f"rest={bottom_drawer_rest}, extended={bottom_drawer_extended}",
    )

    with ctx.pose({leaf_hinge: leaf_hinge.motion_limits.upper}):
        ctx.expect_overlap(
            side_leaf,
            desk_frame,
            axes="y",
            elem_a="panel",
            elem_b="main_top",
            min_overlap=0.540,
            name="leaf matches top depth when raised",
        )
        ctx.expect_gap(
            side_leaf,
            desk_frame,
            axis="x",
            positive_elem="panel",
            negative_elem="main_top",
            min_gap=0.0,
            max_gap=0.012,
            name="raised leaf meets the desk edge with a hinge gap",
        )

        leaf_aabb = ctx.part_element_world_aabb(side_leaf, elem="panel")
        top_aabb = ctx.part_element_world_aabb(desk_frame, elem="main_top")
        ctx.check(
            "raised leaf sits flush with the work surface",
            leaf_aabb is not None
            and top_aabb is not None
            and abs(leaf_aabb[0][2] - top_aabb[0][2]) <= 0.001
            and abs(leaf_aabb[1][2] - top_aabb[1][2]) <= 0.001,
            details=f"leaf_aabb={leaf_aabb}, top_aabb={top_aabb}",
        )

    leaf_closed_aabb = ctx.part_element_world_aabb(side_leaf, elem="panel")
    top_aabb_rest = ctx.part_element_world_aabb(desk_frame, elem="main_top")
    ctx.check(
        "leaf stores below the main top",
        leaf_closed_aabb is not None
        and top_aabb_rest is not None
        and leaf_closed_aabb[1][2] <= top_aabb_rest[0][2] + 0.001,
        details=f"leaf_closed_aabb={leaf_closed_aabb}, top_aabb={top_aabb_rest}",
    )

    return ctx.report()


object_model = build_object_model()
