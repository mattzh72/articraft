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
    model = ArticulatedObject(name="rolling_toolbox")

    case_dark = model.material("case_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    latch_black = model.material("latch_black", rgba=(0.07, 0.07, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    hub_dark = model.material("hub_dark", rgba=(0.19, 0.20, 0.21, 1.0))

    case_width = 0.46
    case_depth = 0.30
    case_height = 0.44
    wall = 0.014

    drawer_opening_depth = 0.19
    drawer_opening_height = 0.11
    drawer_center_z = 0.275
    drawer_center_y = 0.0

    lid_width = 0.488
    lid_depth = 0.328
    lid_height = 0.052
    lid_wall = 0.010

    guide_y = case_depth / 2.0 + 0.026
    guide_z = 0.36
    rail_offset_x = 0.150

    case = model.part("case")
    case.visual(
        Box((case_width - 2.0 * wall, case_depth - 2.0 * wall, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=case_dark,
        name="bottom",
    )
    case.visual(
        Box((case_width, wall, case_height)),
        origin=Origin(xyz=(0.0, -(case_depth - wall) / 2.0, case_height / 2.0)),
        material=case_dark,
        name="front_wall",
    )
    case.visual(
        Box((case_width, wall, case_height)),
        origin=Origin(xyz=(0.0, (case_depth - wall) / 2.0, case_height / 2.0)),
        material=case_dark,
        name="back_wall",
    )
    case.visual(
        Box((wall, case_depth - 2.0 * wall, case_height)),
        origin=Origin(xyz=(-(case_width - wall) / 2.0, 0.0, case_height / 2.0)),
        material=case_dark,
        name="left_wall",
    )

    right_side_x = (case_width - wall) / 2.0
    opening_bottom = drawer_center_z - drawer_opening_height / 2.0
    opening_top = drawer_center_z + drawer_opening_height / 2.0
    side_margin = ((case_depth - 2.0 * wall) - drawer_opening_depth) / 2.0

    case.visual(
        Box((wall, case_depth - 2.0 * wall, opening_bottom)),
        origin=Origin(xyz=(right_side_x, 0.0, opening_bottom / 2.0)),
        material=case_dark,
        name="right_lower",
    )
    case.visual(
        Box((wall, case_depth - 2.0 * wall, case_height - opening_top)),
        origin=Origin(
            xyz=(
                right_side_x,
                0.0,
                opening_top + (case_height - opening_top) / 2.0,
            )
        ),
        material=case_dark,
        name="right_upper",
    )
    case.visual(
        Box((wall, side_margin, drawer_opening_height)),
        origin=Origin(
            xyz=(
                right_side_x,
                -(drawer_opening_depth / 2.0 + side_margin / 2.0),
                drawer_center_z,
            )
        ),
        material=case_dark,
        name="right_front_stile",
    )
    case.visual(
        Box((wall, side_margin, drawer_opening_height)),
        origin=Origin(
            xyz=(
                right_side_x,
                drawer_opening_depth / 2.0 + side_margin / 2.0,
                drawer_center_z,
            )
        ),
        material=case_dark,
        name="right_rear_stile",
    )
    case.visual(
        Box((0.012, case_depth - 2.0 * wall, case_height - 0.03)),
        origin=Origin(xyz=(0.083, 0.0, (case_height - 0.03) / 2.0)),
        material=trim_dark,
        name="drawer_partition",
    )
    case.visual(
        Box((0.028, 0.052, 0.075)),
        origin=Origin(xyz=(-0.145, -0.108, 0.0375)),
        material=trim_dark,
        name="front_foot_0",
    )
    case.visual(
        Box((0.028, 0.052, 0.075)),
        origin=Origin(xyz=(0.145, -0.108, 0.0375)),
        material=trim_dark,
        name="front_foot_1",
    )
    case.visual(
        Box((case_width * 0.56, 0.030, 0.090)),
        origin=Origin(xyz=(0.0, 0.105, 0.045)),
        material=trim_dark,
        name="rear_bumper",
    )

    guide_width = 0.008
    guide_depth = 0.026
    guide_height = 0.112
    guide_gap = 0.016
    for rail_index, rail_x in enumerate((-rail_offset_x, rail_offset_x)):
        for cheek_index, cheek_x in enumerate((rail_x - guide_gap, rail_x + guide_gap)):
            case.visual(
                Box((guide_width, guide_depth, guide_height)),
                origin=Origin(xyz=(cheek_x, guide_y - guide_depth / 2.0, guide_z)),
                material=trim_dark,
                name=f"guide_{rail_index}_{cheek_index}",
            )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, lid_wall)),
        origin=Origin(
            xyz=(
                0.0,
                -lid_depth / 2.0,
                lid_height - lid_wall / 2.0,
            )
        ),
        material=case_dark,
        name="lid_top",
    )
    lid.visual(
        Box((lid_wall, lid_depth, lid_height - lid_wall)),
        origin=Origin(
            xyz=(
                -(lid_width - lid_wall) / 2.0,
                -lid_depth / 2.0,
                (lid_height - lid_wall) / 2.0,
            )
        ),
        material=case_dark,
        name="lid_side_0",
    )
    lid.visual(
        Box((lid_wall, lid_depth, lid_height - lid_wall)),
        origin=Origin(
            xyz=(
                (lid_width - lid_wall) / 2.0,
                -lid_depth / 2.0,
                (lid_height - lid_wall) / 2.0,
            )
        ),
        material=case_dark,
        name="lid_side_1",
    )
    lid.visual(
        Box((lid_width - 2.0 * lid_wall, lid_wall, lid_height - lid_wall)),
        origin=Origin(
            xyz=(
                0.0,
                -(lid_depth - lid_wall / 2.0),
                (lid_height - lid_wall) / 2.0,
            )
        ),
        material=case_dark,
        name="lid_front",
    )
    lid.visual(
        Box((lid_width - 2.0 * lid_wall, lid_wall, 0.020)),
        origin=Origin(xyz=(0.0, -lid_wall / 2.0, 0.010)),
        material=case_dark,
        name="lid_rear",
    )
    lid.visual(
        Box((0.040, 0.036, 0.018)),
        origin=Origin(xyz=(-0.060, -0.110, lid_height + 0.009)),
        material=trim_dark,
        name="grip_post_0",
    )
    lid.visual(
        Box((0.040, 0.036, 0.018)),
        origin=Origin(xyz=(0.060, -0.110, lid_height + 0.009)),
        material=trim_dark,
        name="grip_post_1",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.140),
        origin=Origin(
            xyz=(0.0, -0.110, lid_height + 0.027),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_dark,
        name="grip_bar",
    )

    handle_frame = model.part("handle_frame")
    handle_frame.visual(
        Cylinder(radius=0.009, length=0.56),
        origin=Origin(xyz=(-rail_offset_x, 0.0, 0.110)),
        material=steel,
        name="rail_0",
    )
    handle_frame.visual(
        Cylinder(radius=0.009, length=0.56),
        origin=Origin(xyz=(rail_offset_x, 0.0, 0.110)),
        material=steel,
        name="rail_1",
    )
    for rail_index, rail_x in enumerate((-rail_offset_x, rail_offset_x)):
        handle_frame.visual(
            Box((0.006, 0.014, 0.100)),
            origin=Origin(xyz=(rail_x - 0.009, -0.007, 0.060)),
            material=trim_dark,
            name=f"slider_{rail_index}_0",
        )
        handle_frame.visual(
            Box((0.006, 0.014, 0.100)),
            origin=Origin(xyz=(rail_x + 0.009, -0.007, 0.060)),
            material=trim_dark,
            name=f"slider_{rail_index}_1",
        )
    handle_frame.visual(
        Cylinder(radius=0.011, length=0.320),
        origin=Origin(
            xyz=(0.0, 0.0, 0.390),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_dark,
        name="top_bar",
    )
    handle_frame.visual(
        Box((0.292, 0.020, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.388)),
        material=latch_black,
        name="grip_cover",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.014, 0.190, 0.096)),
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
        material=case_dark,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.106, 0.168, 0.008)),
        origin=Origin(xyz=(-0.067, 0.0, -0.044)),
        material=trim_dark,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.102, 0.008, 0.074)),
        origin=Origin(xyz=(-0.065, -0.080, -0.011)),
        material=trim_dark,
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.102, 0.008, 0.074)),
        origin=Origin(xyz=(-0.065, 0.080, -0.011)),
        material=trim_dark,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.010, 0.168, 0.074)),
        origin=Origin(xyz=(-0.118, 0.0, -0.011)),
        material=trim_dark,
        name="tray_back",
    )
    drawer.visual(
        Box((0.020, 0.082, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, 0.002)),
        material=latch_black,
        name="drawer_pull",
    )

    wheel_radius = 0.055
    wheel_width = 0.035
    wheel_y = 0.092
    wheel_z = wheel_radius
    wheel_x = case_width / 2.0 + wheel_width / 2.0
    for index, sign in enumerate((-1.0, 1.0)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=wheel_radius, length=wheel_width),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.031, length=wheel_width + 0.004),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hub_dark,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(
                xyz=(-sign * 0.010, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name="axle_cap",
        )
        model.articulation(
            f"case_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=case,
            child=wheel,
            origin=Origin(xyz=(sign * wheel_x, wheel_y, wheel_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=12.0),
        )

    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(0.0, case_depth / 2.0, case_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "case_to_handle_frame",
        ArticulationType.PRISMATIC,
        parent=case,
        child=handle_frame,
        origin=Origin(xyz=(0.0, guide_y, 0.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "case_to_drawer",
        ArticulationType.PRISMATIC,
        parent=case,
        child=drawer,
        origin=Origin(xyz=(case_width / 2.0, drawer_center_y, drawer_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    handle_frame = object_model.get_part("handle_frame")
    drawer = object_model.get_part("drawer")

    lid_hinge = object_model.get_articulation("case_to_lid")
    handle_slide = object_model.get_articulation("case_to_handle_frame")
    drawer_slide = object_model.get_articulation("case_to_drawer")

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        max_gap=0.004,
        max_penetration=0.002,
        name="closed lid sits on the case rim",
    )
    ctx.expect_overlap(
        lid,
        case,
        axes="xy",
        min_overlap=0.28,
        name="closed lid covers the case opening",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.14,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    collapsed_handle_aabb = ctx.part_world_aabb(handle_frame)
    with ctx.pose({handle_slide: 0.30}):
        extended_handle_aabb = ctx.part_world_aabb(handle_frame)
    ctx.check(
        "handle frame telescopes upward",
        collapsed_handle_aabb is not None
        and extended_handle_aabb is not None
        and extended_handle_aabb[1][2] > collapsed_handle_aabb[1][2] + 0.24,
        details=f"collapsed={collapsed_handle_aabb}, extended={extended_handle_aabb}",
    )

    case_aabb = ctx.part_world_aabb(case)
    closed_drawer_aabb = ctx.part_world_aabb(drawer)
    with ctx.pose({drawer_slide: 0.10}):
        open_drawer_aabb = ctx.part_world_aabb(drawer)
    ctx.check(
        "drawer slides outward from the side",
        closed_drawer_aabb is not None
        and open_drawer_aabb is not None
        and open_drawer_aabb[1][0] > closed_drawer_aabb[1][0] + 0.08,
        details=f"closed={closed_drawer_aabb}, open={open_drawer_aabb}",
    )
    ctx.check(
        "drawer stays partially retained when extended",
        case_aabb is not None
        and open_drawer_aabb is not None
        and open_drawer_aabb[0][0] < case_aabb[1][0] - 0.01,
        details=f"case={case_aabb}, open_drawer={open_drawer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
