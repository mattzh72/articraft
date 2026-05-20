from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standing_desk")

    top_finish = model.material("top_finish", rgba=(0.63, 0.49, 0.33, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    stage_finish = model.material("stage_finish", rgba=(0.76, 0.77, 0.79, 1.0))
    handset_finish = model.material("handset_finish", rgba=(0.12, 0.12, 0.13, 1.0))
    button_finish = model.material("button_finish", rgba=(0.18, 0.18, 0.19, 1.0))

    desk_width = 1.60
    desk_depth = 0.80
    top_thickness = 0.03
    leg_spacing = 0.90

    foot_height = 0.03
    foot_width = 0.09
    foot_depth = 0.72

    outer_width = 0.09
    outer_depth = 0.07
    wall_thickness = 0.006
    outer_height = 0.63

    inner_width = 0.072
    inner_depth = 0.050
    inner_length = 0.60
    inner_bottom_below_joint = 0.582
    inner_top_above_joint = inner_length - inner_bottom_below_joint

    mount_plate_width = 0.14
    mount_plate_depth = 0.10
    mount_plate_thickness = 0.012
    mount_plate_center_z = 0.010
    mount_plate_top_z = mount_plate_center_z + mount_plate_thickness / 2.0

    lift_travel = 0.38
    column_center_z = foot_height + outer_height / 2.0
    sleeve_entry_z = foot_height + outer_height
    half_leg_spacing = leg_spacing / 2.0

    frame = model.part("frame")

    for side, x_pos in (("left", -half_leg_spacing), ("right", half_leg_spacing)):
        frame.visual(
            Box((foot_width, foot_depth, foot_height)),
            origin=Origin(xyz=(x_pos, 0.0, foot_height / 2.0)),
            material=frame_finish,
            name=f"{side}_foot",
        )

        frame.visual(
            Box((outer_width, wall_thickness, outer_height)),
            origin=Origin(
                xyz=(x_pos, (outer_depth - wall_thickness) / 2.0, column_center_z)
            ),
            material=frame_finish,
            name=f"{side}_outer_front",
        )
        frame.visual(
            Box((outer_width, wall_thickness, outer_height)),
            origin=Origin(
                xyz=(x_pos, -(outer_depth - wall_thickness) / 2.0, column_center_z)
            ),
            material=frame_finish,
            name=f"{side}_outer_rear",
        )
        frame.visual(
            Box((wall_thickness, outer_depth - 2.0 * wall_thickness, outer_height)),
            origin=Origin(
                xyz=(x_pos - (outer_width - wall_thickness) / 2.0, 0.0, column_center_z)
            ),
            material=frame_finish,
            name=f"{side}_outer_side",
        )
        frame.visual(
            Box((wall_thickness, outer_depth - 2.0 * wall_thickness, outer_height)),
            origin=Origin(
                xyz=(x_pos + (outer_width - wall_thickness) / 2.0, 0.0, column_center_z)
            ),
            material=frame_finish,
            name=f"{side}_outer_inner",
        )

    frame.visual(
        Box((leg_spacing - outer_width + wall_thickness, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=frame_finish,
        name="crossbeam",
    )
    frame.visual(
        Box((0.42, 0.12, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
        material=frame_finish,
        name="motor_cover",
    )

    left_inner = model.part("left_inner")
    left_inner.visual(
        Box((inner_width, inner_depth, inner_length)),
        origin=Origin(
            xyz=(0.0, 0.0, (inner_top_above_joint - inner_bottom_below_joint) / 2.0)
        ),
        material=stage_finish,
        name="stage_tube",
    )
    left_inner.visual(
        Box((mount_plate_width, mount_plate_depth, mount_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, mount_plate_center_z)),
        material=frame_finish,
        name="mount_plate",
    )
    left_inner.visual(
        Box((outer_width - 2.0 * wall_thickness, outer_depth - 2.0 * wall_thickness, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=stage_finish,
        name="guide_block",
    )

    right_inner = model.part("right_inner")
    right_inner.visual(
        Box((inner_width, inner_depth, inner_length)),
        origin=Origin(
            xyz=(0.0, 0.0, (inner_top_above_joint - inner_bottom_below_joint) / 2.0)
        ),
        material=stage_finish,
        name="stage_tube",
    )
    right_inner.visual(
        Box((mount_plate_width, mount_plate_depth, mount_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, mount_plate_center_z)),
        material=frame_finish,
        name="mount_plate",
    )
    right_inner.visual(
        Box((outer_width - 2.0 * wall_thickness, outer_depth - 2.0 * wall_thickness, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=stage_finish,
        name="guide_block",
    )

    desktop = model.part("desktop")
    desktop.visual(
        Box((desk_width, desk_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_thickness / 2.0)),
        material=top_finish,
        name="top_panel",
    )
    desktop.visual(
        Box((0.18, 0.12, 0.012)),
        origin=Origin(xyz=(-half_leg_spacing, 0.0, -0.006)),
        material=frame_finish,
        name="left_mount",
    )
    desktop.visual(
        Box((0.18, 0.12, 0.012)),
        origin=Origin(xyz=(half_leg_spacing, 0.0, -0.006)),
        material=frame_finish,
        name="right_mount",
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.125, 0.045, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=handset_finish,
        name="housing",
    )

    button_positions = (-0.042, -0.014, 0.014, 0.042)
    for index, x_pos in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.018, 0.012, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, -0.0015)),
            material=button_finish,
            name="cap",
        )

        model.articulation(
            f"handset_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=handset,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=0.003,
            ),
        )

    left_lift = model.articulation(
        "left_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_inner,
        origin=Origin(xyz=(-half_leg_spacing, 0.0, sleeve_entry_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.04,
            lower=0.0,
            upper=lift_travel,
        ),
    )

    model.articulation(
        "right_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_inner,
        origin=Origin(xyz=(half_leg_spacing, 0.0, sleeve_entry_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.04,
            lower=0.0,
            upper=lift_travel,
        ),
        mimic=Mimic(joint=left_lift.name, multiplier=1.0, offset=0.0),
    )

    model.articulation(
        "left_inner_to_desktop",
        ArticulationType.FIXED,
        parent=left_inner,
        child=desktop,
        origin=Origin(xyz=(half_leg_spacing, 0.0, mount_plate_top_z + 0.012)),
    )

    model.articulation(
        "desktop_to_handset",
        ArticulationType.FIXED,
        parent=desktop,
        child=handset,
        origin=Origin(xyz=(0.56, desk_depth / 2.0 - 0.060, -0.016)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desktop = object_model.get_part("desktop")
    left_inner = object_model.get_part("left_inner")
    right_inner = object_model.get_part("right_inner")
    handset = object_model.get_part("handset")

    left_lift = object_model.get_articulation("left_lift")
    lift_limits = left_lift.motion_limits

    ctx.expect_contact(
        desktop,
        left_inner,
        elem_a="left_mount",
        elem_b="mount_plate",
        name="left lift stage supports the desktop at rest",
    )
    ctx.expect_overlap(
        desktop,
        left_inner,
        axes="xy",
        elem_a="left_mount",
        elem_b="mount_plate",
        min_overlap=0.10,
        name="left support pad overlaps its mount plate",
    )
    ctx.expect_contact(
        desktop,
        right_inner,
        elem_a="right_mount",
        elem_b="mount_plate",
        name="right lift stage supports the desktop at rest",
    )
    ctx.expect_overlap(
        desktop,
        right_inner,
        axes="xy",
        elem_a="right_mount",
        elem_b="mount_plate",
        min_overlap=0.10,
        name="right support pad overlaps its mount plate",
    )
    ctx.expect_origin_gap(
        right_inner,
        left_inner,
        axis="z",
        min_gap=-0.0005,
        max_gap=0.0005,
        name="lift stages stay level at rest",
    )
    ctx.expect_gap(
        desktop,
        handset,
        axis="z",
        positive_elem="top_panel",
        negative_elem="housing",
        min_gap=0.0,
        max_gap=0.0,
        name="handset housing mounts flush to the desktop underside",
    )

    rest_desktop_pos = ctx.part_world_position(desktop)
    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({left_lift: lift_limits.upper}):
            ctx.expect_contact(
                desktop,
                left_inner,
                elem_a="left_mount",
                elem_b="mount_plate",
                name="left lift stage supports the desktop at full height",
            )
            ctx.expect_contact(
                desktop,
                right_inner,
                elem_a="right_mount",
                elem_b="mount_plate",
                name="right lift stage supports the desktop at full height",
            )
            ctx.expect_origin_gap(
                right_inner,
                left_inner,
                axis="z",
                min_gap=-0.0005,
                max_gap=0.0005,
                name="lift stages stay level at full height",
            )
            raised_desktop_pos = ctx.part_world_position(desktop)

        ctx.check(
            "desktop rises through the standing range",
            rest_desktop_pos is not None
            and raised_desktop_pos is not None
            and raised_desktop_pos[2] > rest_desktop_pos[2] + 0.30,
            details=f"rest={rest_desktop_pos}, raised={raised_desktop_pos}",
        )

    button_rest_positions: dict[int, tuple[float, float, float] | None] = {}
    for index in range(4):
        button_rest_positions[index] = ctx.part_world_position(f"button_{index}")

    for index in range(3):
        ctx.expect_gap(
            f"button_{index + 1}",
            f"button_{index}",
            axis="x",
            min_gap=0.008,
            name=f"button_{index + 1} stays discrete from button_{index}",
        )

    for index in range(4):
        button_joint = object_model.get_articulation(f"handset_to_button_{index}")
        joint_limits = button_joint.motion_limits
        if joint_limits is None or joint_limits.upper is None:
            continue

        with ctx.pose({button_joint: joint_limits.upper}):
            pressed_position = ctx.part_world_position(f"button_{index}")

        neighbor_index = 0 if index != 0 else 1
        with ctx.pose({button_joint: joint_limits.upper}):
            neighbor_pressed_position = ctx.part_world_position(f"button_{neighbor_index}")

        rest_position = button_rest_positions[index]
        neighbor_rest_position = button_rest_positions[neighbor_index]

        ctx.check(
            f"button_{index} presses inward independently",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[2] > rest_position[2] + 0.0015,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )
        ctx.check(
            f"button_{index} does not drag button_{neighbor_index}",
            neighbor_rest_position is not None
            and neighbor_pressed_position is not None
            and abs(neighbor_pressed_position[2] - neighbor_rest_position[2]) < 1e-6,
            details=(
                f"neighbor_rest={neighbor_rest_position}, "
                f"neighbor_pressed={neighbor_pressed_position}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
