from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retail_jewelry_showcase")

    walnut = model.material("walnut_veneer", rgba=(0.42, 0.28, 0.18, 1.0))
    brass = model.material("brass_trim", rgba=(0.79, 0.68, 0.38, 1.0))
    plinth_black = model.material("plinth_black", rgba=(0.12, 0.11, 0.10, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.66, 0.67, 0.70, 1.0))
    showcase_glass = model.material("showcase_glass", rgba=(0.78, 0.86, 0.92, 0.28))
    velvet = model.material("velvet_burgundy", rgba=(0.45, 0.07, 0.16, 1.0))
    drawer_liner = model.material("drawer_velvet", rgba=(0.36, 0.04, 0.13, 1.0))

    cabinet_width = 1.40
    cabinet_depth = 0.65
    plinth_height = 0.08
    lower_carcass_height = 0.56
    display_height = 0.26
    top_glass_thickness = 0.012
    lower_top_z = plinth_height + lower_carcass_height
    total_height = lower_top_z + display_height + top_glass_thickness

    side_wall_t = 0.025
    back_panel_t = 0.020
    top_deck_t = 0.022
    front_frame_t = 0.030

    drawer_front_width = 0.368
    drawer_front_height = 0.160
    drawer_front_t = 0.024
    drawer_box_width = 0.348
    drawer_box_depth = 0.430
    drawer_box_height = 0.110
    drawer_slide_t = 0.008
    drawer_center_y = -0.098
    drawer_center_z = 0.260
    drawer_extension = 0.260
    drawer_centers_x = (-0.410, 0.000, 0.410)

    display_post_x = cabinet_width * 0.5 - 0.015
    display_post_y = cabinet_depth * 0.5 - 0.010
    display_post_w = 0.030
    display_post_d = 0.020
    display_frame_h = 0.030
    display_side_span = 0.610
    display_panel_height = 0.204
    display_panel_width = 1.344
    side_door_depth = 0.592
    side_door_height = 0.196
    side_door_thickness = 0.018
    side_door_joint_y = 0.301
    side_door_bottom_z = 0.672
    side_door_open_limit = 1.60

    cabinet = model.part("cabinet_body")
    cabinet.visual(
        Box((1.30, 0.55, plinth_height)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height * 0.5)),
        material=plinth_black,
        name="plinth_base",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_wall_t, cabinet_depth - 0.040, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, plinth_height + 0.011)),
        material=walnut,
        name="lower_floor",
    )
    cabinet.visual(
        Box((side_wall_t, cabinet_depth, lower_carcass_height)),
        origin=Origin(
            xyz=(-cabinet_width * 0.5 + side_wall_t * 0.5, 0.0, plinth_height + lower_carcass_height * 0.5)
        ),
        material=walnut,
        name="left_side_wall",
    )
    cabinet.visual(
        Box((side_wall_t, cabinet_depth, lower_carcass_height)),
        origin=Origin(
            xyz=(cabinet_width * 0.5 - side_wall_t * 0.5, 0.0, plinth_height + lower_carcass_height * 0.5)
        ),
        material=walnut,
        name="right_side_wall",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_wall_t, back_panel_t, lower_carcass_height)),
        origin=Origin(
            xyz=(0.0, cabinet_depth * 0.5 - back_panel_t * 0.5, plinth_height + lower_carcass_height * 0.5)
        ),
        material=walnut,
        name="rear_panel",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, top_deck_t)),
        origin=Origin(xyz=(0.0, 0.0, lower_top_z + top_deck_t * 0.5)),
        material=walnut,
        name="display_deck",
    )
    cabinet.visual(
        Box((1.20, front_frame_t, 0.100)),
        origin=Origin(xyz=(0.0, -cabinet_depth * 0.5 + front_frame_t * 0.5, 0.130)),
        material=walnut,
        name="drawer_bottom_rail",
    )
    cabinet.visual(
        Box((1.20, front_frame_t, 0.260)),
        origin=Origin(xyz=(0.0, -cabinet_depth * 0.5 + front_frame_t * 0.5, 0.510)),
        material=walnut,
        name="upper_front_apron",
    )
    cabinet.visual(
        Box((0.100, front_frame_t, 0.200)),
        origin=Origin(xyz=(-0.650, -cabinet_depth * 0.5 + front_frame_t * 0.5, 0.280)),
        material=walnut,
        name="left_front_stile",
    )
    cabinet.visual(
        Box((0.100, front_frame_t, 0.200)),
        origin=Origin(xyz=(0.650, -cabinet_depth * 0.5 + front_frame_t * 0.5, 0.280)),
        material=walnut,
        name="right_front_stile",
    )
    cabinet.visual(
        Box((0.030, 0.560, lower_carcass_height)),
        origin=Origin(xyz=(-0.205, 0.0, plinth_height + lower_carcass_height * 0.5)),
        material=walnut,
        name="left_drawer_divider",
    )
    cabinet.visual(
        Box((0.030, 0.560, lower_carcass_height)),
        origin=Origin(xyz=(0.205, 0.0, plinth_height + lower_carcass_height * 0.5)),
        material=walnut,
        name="right_drawer_divider",
    )

    for drawer_index, center_x in enumerate(drawer_centers_x):
        cabinet.visual(
            Box((drawer_slide_t, 0.340, 0.030)),
            origin=Origin(xyz=(center_x - 0.186, -0.060, 0.240)),
            material=rail_metal,
            name=f"drawer_bay_{drawer_index}_left_rail",
        )
        cabinet.visual(
            Box((drawer_slide_t, 0.340, 0.030)),
            origin=Origin(xyz=(center_x + 0.186, -0.060, 0.240)),
            material=rail_metal,
            name=f"drawer_bay_{drawer_index}_right_rail",
        )

    cabinet.visual(
        Box((0.080, 0.060, 0.030)),
        origin=Origin(xyz=(-0.638, -0.060, 0.240)),
        material=walnut,
        name="left_outer_rail_support",
    )
    cabinet.visual(
        Box((0.080, 0.060, 0.030)),
        origin=Origin(xyz=(0.638, -0.060, 0.240)),
        material=walnut,
        name="right_outer_rail_support",
    )

    cabinet.visual(
        Box((1.28, 0.50, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, lower_top_z + top_deck_t + 0.003)),
        material=velvet,
        name="display_pad",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            cabinet.visual(
                Box((display_post_w, display_post_d, display_height)),
                origin=Origin(
                    xyz=(x_sign * display_post_x, y_sign * display_post_y, lower_top_z + display_height * 0.5)
                ),
                material=brass,
                name=f"corner_post_{'l' if x_sign < 0 else 'r'}_{'f' if y_sign < 0 else 'b'}",
            )

    cabinet.visual(
        Box((1.340, display_post_d, display_frame_h)),
        origin=Origin(xyz=(0.0, -display_post_y, lower_top_z + display_frame_h * 0.5)),
        material=brass,
        name="front_lower_frame",
    )
    cabinet.visual(
        Box((1.340, display_post_d, display_frame_h)),
        origin=Origin(xyz=(0.0, display_post_y, lower_top_z + display_frame_h * 0.5)),
        material=brass,
        name="rear_lower_frame",
    )
    cabinet.visual(
        Box((1.340, display_post_d, display_frame_h)),
        origin=Origin(xyz=(0.0, -display_post_y, lower_top_z + display_height - display_frame_h * 0.5)),
        material=brass,
        name="front_upper_frame",
    )
    cabinet.visual(
        Box((1.340, display_post_d, display_frame_h)),
        origin=Origin(xyz=(0.0, display_post_y, lower_top_z + display_height - display_frame_h * 0.5)),
        material=brass,
        name="rear_upper_frame",
    )
    cabinet.visual(
        Box((display_post_w, display_side_span, display_frame_h)),
        origin=Origin(xyz=(-display_post_x, 0.0, lower_top_z + display_frame_h * 0.5)),
        material=brass,
        name="left_lower_side_frame",
    )
    cabinet.visual(
        Box((display_post_w, display_side_span, display_frame_h)),
        origin=Origin(xyz=(display_post_x, 0.0, lower_top_z + display_frame_h * 0.5)),
        material=brass,
        name="right_lower_side_frame",
    )
    cabinet.visual(
        Box((display_post_w, display_side_span, display_frame_h)),
        origin=Origin(xyz=(-display_post_x, 0.0, lower_top_z + display_height - display_frame_h * 0.5)),
        material=brass,
        name="left_upper_side_frame",
    )
    cabinet.visual(
        Box((display_post_w, display_side_span, display_frame_h)),
        origin=Origin(xyz=(display_post_x, 0.0, lower_top_z + display_height - display_frame_h * 0.5)),
        material=brass,
        name="right_upper_side_frame",
    )
    cabinet.visual(
        Box((display_post_w, 0.018, 0.196)),
        origin=Origin(xyz=(-display_post_x, 0.310, 0.770)),
        material=brass,
        name="left_hinge_receiver",
    )
    cabinet.visual(
        Box((display_post_w, 0.018, 0.196)),
        origin=Origin(xyz=(display_post_x, 0.310, 0.770)),
        material=brass,
        name="right_hinge_receiver",
    )
    cabinet.visual(
        Box((display_panel_width, 0.010, display_panel_height)),
        origin=Origin(xyz=(0.0, -display_post_y, lower_top_z + display_height * 0.5)),
        material=showcase_glass,
        name="front_glass_panel",
    )
    cabinet.visual(
        Box((display_panel_width, 0.010, display_panel_height)),
        origin=Origin(xyz=(0.0, display_post_y, lower_top_z + display_height * 0.5)),
        material=showcase_glass,
        name="rear_glass_panel",
    )
    cabinet.visual(
        Box((1.36, 0.61, top_glass_thickness)),
        origin=Origin(xyz=(0.0, 0.0, lower_top_z + display_height + top_glass_thickness * 0.5)),
        material=showcase_glass,
        name="top_glass",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, total_height)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, total_height * 0.5)),
    )

    def add_drawer(part_name: str, joint_name: str, center_x: float) -> None:
        drawer = model.part(part_name)

        front_panel_y = -0.215
        tray_center_y = 0.012
        back_panel_y = 0.221

        drawer.visual(
            Box((drawer_front_width, drawer_front_height, drawer_front_t)),
            origin=Origin(xyz=(0.0, front_panel_y, 0.0)),
            material=walnut,
            name="drawer_front",
        )
        drawer.visual(
            Box((drawer_box_width, drawer_box_depth, 0.012)),
            origin=Origin(xyz=(0.0, tray_center_y, -0.054)),
            material=walnut,
            name="drawer_base",
        )
        drawer.visual(
            Box((0.012, drawer_box_depth, drawer_box_height)),
            origin=Origin(xyz=(-0.168, tray_center_y, -0.005)),
            material=walnut,
            name="left_drawer_side",
        )
        drawer.visual(
            Box((0.012, drawer_box_depth, drawer_box_height)),
            origin=Origin(xyz=(0.168, tray_center_y, -0.005)),
            material=walnut,
            name="right_drawer_side",
        )
        drawer.visual(
            Box((drawer_box_width - 0.024, 0.012, drawer_box_height)),
            origin=Origin(xyz=(0.0, back_panel_y, -0.005)),
            material=walnut,
            name="drawer_back",
        )
        drawer.visual(
            Box((drawer_box_width - 0.028, drawer_box_depth - 0.032, 0.004)),
            origin=Origin(xyz=(0.0, tray_center_y, -0.047)),
            material=drawer_liner,
            name="velvet_floor",
        )
        drawer.visual(
            Box((0.008, drawer_box_depth - 0.032, 0.070)),
            origin=Origin(xyz=(-0.158, tray_center_y, -0.022)),
            material=drawer_liner,
            name="velvet_left_wall",
        )
        drawer.visual(
            Box((0.008, drawer_box_depth - 0.032, 0.070)),
            origin=Origin(xyz=(0.158, tray_center_y, -0.022)),
            material=drawer_liner,
            name="velvet_right_wall",
        )
        drawer.visual(
            Box((drawer_slide_t, 0.360, 0.028)),
            origin=Origin(xyz=(-0.178, 0.000, -0.020)),
            material=rail_metal,
            name="left_slide",
        )
        drawer.visual(
            Box((drawer_slide_t, 0.360, 0.028)),
            origin=Origin(xyz=(0.178, 0.000, -0.020)),
            material=rail_metal,
            name="right_slide",
        )
        drawer.visual(
            Box((0.012, 0.016, 0.016)),
            origin=Origin(xyz=(-0.050, -0.234, 0.000)),
            material=brass,
            name="handle_left_mount",
        )
        drawer.visual(
            Box((0.012, 0.016, 0.016)),
            origin=Origin(xyz=(0.050, -0.234, 0.000)),
            material=brass,
            name="handle_right_mount",
        )
        drawer.visual(
            Box((0.118, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, -0.244, 0.000)),
            material=brass,
            name="handle_bar",
        )
        drawer.inertial = Inertial.from_geometry(
            Box((drawer_front_width, 0.454, drawer_front_height)),
            mass=5.5,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(center_x, drawer_center_y, drawer_center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=90.0,
                velocity=0.30,
                lower=0.0,
                upper=drawer_extension,
            ),
        )

    def add_side_door(
        *,
        part_name: str,
        joint_name: str,
        hinge_x: float,
        axis_z: float,
        handle_x: float,
    ) -> None:
        door = model.part(part_name)

        door.visual(
            Box((side_door_thickness, 0.018, side_door_height)),
            origin=Origin(xyz=(0.009, -0.009, side_door_height * 0.5)),
            material=brass,
            name="rear_stile",
        )
        door.visual(
            Box((side_door_thickness, 0.018, side_door_height)),
            origin=Origin(xyz=(0.009, -(side_door_depth - 0.009), side_door_height * 0.5)),
            material=brass,
            name="front_stile",
        )
        door.visual(
            Box((side_door_thickness, side_door_depth - 0.034, 0.018)),
            origin=Origin(xyz=(0.009, -side_door_depth * 0.5, 0.009)),
            material=brass,
            name="bottom_rail",
        )
        door.visual(
            Box((side_door_thickness, side_door_depth - 0.034, 0.018)),
            origin=Origin(xyz=(0.009, -side_door_depth * 0.5, side_door_height - 0.009)),
            material=brass,
            name="top_rail",
        )
        door.visual(
            Box((0.010, side_door_depth - 0.038, side_door_height - 0.030)),
            origin=Origin(xyz=(0.009, -side_door_depth * 0.5, side_door_height * 0.5)),
            material=showcase_glass,
            name="glass_panel",
        )
        for barrel_index, barrel_z in enumerate((0.024, side_door_height * 0.5, side_door_height - 0.024)):
            door.visual(
                Cylinder(radius=0.006, length=0.022),
                origin=Origin(xyz=(0.006, -0.009, barrel_z)),
                material=brass,
                name=f"hinge_barrel_{barrel_index}",
            )
        door.visual(
            Box((0.026, 0.010, 0.080)),
            origin=Origin(xyz=(handle_x, -(side_door_depth - 0.050), side_door_height * 0.5)),
            material=brass,
            name="pull_handle",
        )
        door.inertial = Inertial.from_geometry(
            Box((0.040, side_door_depth, side_door_height)),
            mass=2.8,
            origin=Origin(xyz=(0.010, -side_door_depth * 0.5, side_door_height * 0.5)),
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=door,
            origin=Origin(xyz=(hinge_x, side_door_joint_y, side_door_bottom_z)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=1.2,
                lower=0.0,
                upper=side_door_open_limit,
            ),
        )

    add_drawer("left_drawer", "cabinet_to_left_drawer", drawer_centers_x[0])
    add_drawer("center_drawer", "cabinet_to_center_drawer", drawer_centers_x[1])
    add_drawer("right_drawer", "cabinet_to_right_drawer", drawer_centers_x[2])

    add_side_door(
        part_name="left_side_door",
        joint_name="cabinet_to_left_side_door",
        hinge_x=-cabinet_width * 0.5,
        axis_z=-1.0,
        handle_x=-0.006,
    )
    add_side_door(
        part_name="right_side_door",
        joint_name="cabinet_to_right_side_door",
        hinge_x=cabinet_width * 0.5,
        axis_z=1.0,
        handle_x=0.024,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet_body")
    drawers = [
        object_model.get_part("left_drawer"),
        object_model.get_part("center_drawer"),
        object_model.get_part("right_drawer"),
    ]
    drawer_joints = [
        object_model.get_articulation("cabinet_to_left_drawer"),
        object_model.get_articulation("cabinet_to_center_drawer"),
        object_model.get_articulation("cabinet_to_right_drawer"),
    ]
    left_door = object_model.get_part("left_side_door")
    right_door = object_model.get_part("right_side_door")
    left_hinge = object_model.get_articulation("cabinet_to_left_side_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_side_door")

    with ctx.pose(
        {
            drawer_joints[0]: 0.0,
            drawer_joints[1]: 0.0,
            drawer_joints[2]: 0.0,
            left_hinge: 0.0,
            right_hinge: 0.0,
        }
    ):
        for drawer in drawers:
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="xz",
                min_overlap=0.10,
                name=f"{drawer.name} stays aligned with the front cabinet opening",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                min_overlap=0.20,
                name=f"{drawer.name} remains inserted into the cabinet at rest",
            )

        ctx.expect_overlap(
            left_door,
            cabinet,
            axes="yz",
            min_overlap=0.15,
            name="left side door covers the side opening when closed",
        )
        ctx.expect_overlap(
            right_door,
            cabinet,
            axes="yz",
            min_overlap=0.15,
            name="right side door covers the side opening when closed",
        )

    for drawer, joint in zip(drawers, drawer_joints):
        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: joint.motion_limits.upper}):
            open_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="x",
                min_overlap=0.25,
                name=f"{drawer.name} stays laterally guided when extended",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="z",
                min_overlap=0.08,
                name=f"{drawer.name} stays vertically seated when extended",
            )
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                min_overlap=0.10,
                name=f"{drawer.name} keeps retained insertion on its slides",
            )
        ctx.check(
            f"{drawer.name} slides outward",
            closed_pos is not None
            and open_pos is not None
            and open_pos[1] < closed_pos[1] - 0.18,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    with ctx.pose({left_hinge: 0.0}):
        left_closed_aabb = ctx.part_world_aabb(left_door)
    with ctx.pose({left_hinge: left_hinge.motion_limits.upper}):
        left_open_aabb = ctx.part_world_aabb(left_door)
    ctx.check(
        "left side door swings outward from the cabinet",
        left_closed_aabb is not None
        and left_open_aabb is not None
        and left_open_aabb[0][0] < left_closed_aabb[0][0] - 0.18,
        details=f"closed={left_closed_aabb}, open={left_open_aabb}",
    )

    with ctx.pose({right_hinge: 0.0}):
        right_closed_aabb = ctx.part_world_aabb(right_door)
    with ctx.pose({right_hinge: right_hinge.motion_limits.upper}):
        right_open_aabb = ctx.part_world_aabb(right_door)
    ctx.check(
        "right side door swings outward from the cabinet",
        right_closed_aabb is not None
        and right_open_aabb is not None
        and right_open_aabb[1][0] > right_closed_aabb[1][0] + 0.18,
        details=f"closed={right_closed_aabb}, open={right_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
