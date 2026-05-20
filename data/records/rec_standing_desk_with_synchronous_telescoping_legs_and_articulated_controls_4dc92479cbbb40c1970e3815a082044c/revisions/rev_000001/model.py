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


def _add_rect_tube(
    part,
    *,
    outer_x: float,
    outer_y: float,
    height: float,
    wall: float,
    material,
    name_prefix: str,
) -> None:
    part.visual(
        Box((outer_x, wall, height)),
        origin=Origin(xyz=(0.0, (outer_y - wall) * 0.5, -height * 0.5)),
        material=material,
        name=f"{name_prefix}_front",
    )
    part.visual(
        Box((outer_x, wall, height)),
        origin=Origin(xyz=(0.0, -(outer_y - wall) * 0.5, -height * 0.5)),
        material=material,
        name=f"{name_prefix}_rear",
    )
    part.visual(
        Box((wall, outer_y - 2.0 * wall, height)),
        origin=Origin(xyz=((outer_x - wall) * 0.5, 0.0, -height * 0.5)),
        material=material,
        name=f"{name_prefix}_right",
    )
    part.visual(
        Box((wall, outer_y - 2.0 * wall, height)),
        origin=Origin(xyz=(-(outer_x - wall) * 0.5, 0.0, -height * 0.5)),
        material=material,
        name=f"{name_prefix}_left",
    )


def _add_button_part(part, *, material, cap_x: float, cap_y: float, cap_z: float) -> None:
    part.visual(
        Box((cap_x, cap_y, cap_z)),
        origin=Origin(xyz=(0.0, 0.0, -cap_z * 0.5)),
        material=material,
        name="cap",
    )
    part.visual(
        Box((0.010, 0.008, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=material,
        name="stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_standing_desk")

    wood = model.material("wood", rgba=(0.63, 0.50, 0.34, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.83, 0.84, 0.85, 1.0))
    column_steel = model.material("column_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    charcoal = model.material("charcoal", rgba=(0.23, 0.24, 0.26, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    button_black = model.material("button_black", rgba=(0.12, 0.12, 0.13, 1.0))
    button_grey = model.material("button_grey", rgba=(0.24, 0.25, 0.27, 1.0))

    top_x = 1.80
    top_y = 0.82
    top_z = 0.032
    rail_x = 1.54
    rail_y = 0.058
    rail_z = 0.050
    cross_x = 0.090
    cross_y = 0.590
    column_spacing = 0.72

    stage_x = 0.090
    stage_y = 0.055
    stage_z = 0.630

    sleeve_x = 0.118
    sleeve_y = 0.082
    sleeve_z = 0.560
    sleeve_wall = 0.007
    foot_x = 0.720
    foot_y = 0.100
    foot_z = 0.030
    pad_x = 0.078
    pad_y = 0.055
    pad_z = 0.008
    lift_travel = 0.380
    sleeve_top_z = -0.100

    desktop_frame = model.part("desktop_frame")
    desktop_frame.visual(
        Box((top_x, top_y, top_z)),
        origin=Origin(xyz=(0.0, 0.0, -top_z * 0.5)),
        material=wood,
        name="top",
    )
    for rail_name, rail_y_pos in (("front_rail", -0.275), ("rear_rail", 0.275)):
        desktop_frame.visual(
            Box((rail_x, rail_y, rail_z)),
            origin=Origin(xyz=(0.0, rail_y_pos, -(top_z + rail_z * 0.5))),
            material=frame_steel,
            name=rail_name,
        )
    for cross_name, cross_x_pos in (
        ("left_cross", -column_spacing),
        ("center_cross", 0.0),
        ("right_cross", column_spacing),
    ):
        desktop_frame.visual(
            Box((cross_x, cross_y, rail_z)),
            origin=Origin(xyz=(cross_x_pos, 0.0, -(top_z + rail_z * 0.5))),
            material=frame_steel,
            name=cross_name,
        )
        desktop_frame.visual(
            Box((0.160, 0.090, 0.010)),
            origin=Origin(xyz=(cross_x_pos, 0.0, -(top_z + 0.005))),
            material=frame_steel,
            name=f"{cross_name}_plate",
        )

    for stage_name, stage_x_pos in (
        ("left_stage", -column_spacing),
        ("center_stage", 0.0),
        ("right_stage", column_spacing),
    ):
        desktop_frame.visual(
            Box((stage_x, stage_y, stage_z)),
            origin=Origin(xyz=(stage_x_pos, 0.0, -(top_z + stage_z * 0.5))),
            material=charcoal,
            name=stage_name,
        )

    left_column = model.part("left_column")
    _add_rect_tube(
        left_column,
        outer_x=sleeve_x,
        outer_y=sleeve_y,
        height=sleeve_z,
        wall=sleeve_wall,
        material=column_steel,
        name_prefix="sleeve",
    )
    left_column.visual(
        Box((foot_x, foot_y, foot_z)),
        origin=Origin(xyz=(0.0, 0.0, -(sleeve_z + foot_z * 0.5))),
        material=column_steel,
        name="foot",
    )
    left_column.visual(
        Box((0.140, 0.082, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -(sleeve_z + 0.009))),
        material=column_steel,
        name="foot_spine",
    )
    for pad_name, pad_x_pos in (("front_pad", 0.280), ("rear_pad", -0.280)):
        left_column.visual(
            Box((pad_x, pad_y, pad_z)),
            origin=Origin(
                xyz=(pad_x_pos, 0.0, -(sleeve_z + foot_z + pad_z * 0.5)),
            ),
            material=black_plastic,
            name=pad_name,
        )

    center_column = model.part("center_column")
    _add_rect_tube(
        center_column,
        outer_x=sleeve_x,
        outer_y=sleeve_y,
        height=sleeve_z,
        wall=sleeve_wall,
        material=column_steel,
        name_prefix="sleeve",
    )
    center_column.visual(
        Box((foot_x, foot_y, foot_z)),
        origin=Origin(xyz=(0.0, 0.0, -(sleeve_z + foot_z * 0.5))),
        material=column_steel,
        name="foot",
    )
    center_column.visual(
        Box((0.140, 0.082, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -(sleeve_z + 0.009))),
        material=column_steel,
        name="foot_spine",
    )
    for pad_name, pad_x_pos in (("front_pad", 0.280), ("rear_pad", -0.280)):
        center_column.visual(
            Box((pad_x, pad_y, pad_z)),
            origin=Origin(
                xyz=(pad_x_pos, 0.0, -(sleeve_z + foot_z + pad_z * 0.5)),
            ),
            material=black_plastic,
            name=pad_name,
        )

    right_column = model.part("right_column")
    _add_rect_tube(
        right_column,
        outer_x=sleeve_x,
        outer_y=sleeve_y,
        height=sleeve_z,
        wall=sleeve_wall,
        material=column_steel,
        name_prefix="sleeve",
    )
    right_column.visual(
        Box((foot_x, foot_y, foot_z)),
        origin=Origin(xyz=(0.0, 0.0, -(sleeve_z + foot_z * 0.5))),
        material=column_steel,
        name="foot",
    )
    right_column.visual(
        Box((0.140, 0.082, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -(sleeve_z + 0.009))),
        material=column_steel,
        name="foot_spine",
    )
    for pad_name, pad_x_pos in (("front_pad", 0.280), ("rear_pad", -0.280)):
        right_column.visual(
            Box((pad_x, pad_y, pad_z)),
            origin=Origin(
                xyz=(pad_x_pos, 0.0, -(sleeve_z + foot_z + pad_z * 0.5)),
            ),
            material=black_plastic,
            name=pad_name,
        )

    controller = model.part("controller")
    controller.visual(
        Box((0.142, 0.084, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=black_plastic,
        name="top_plate",
    )
    _add_rect_tube(
        controller,
        outer_x=0.142,
        outer_y=0.084,
        height=0.022,
        wall=0.006,
        material=black_plastic,
        name_prefix="body",
    )
    controller.visual(
        Box((0.072, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=black_plastic,
        name="mount_pad",
    )

    paddle = model.part("paddle")
    paddle.visual(
        Box((0.096, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=button_grey,
        name="cap",
    )
    paddle.visual(
        Box((0.026, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=button_grey,
        name="stem",
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.164, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=black_plastic,
        name="top_plate",
    )
    _add_rect_tube(
        handset,
        outer_x=0.164,
        outer_y=0.070,
        height=0.022,
        wall=0.006,
        material=black_plastic,
        name_prefix="body",
    )
    handset.visual(
        Box((0.070, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=black_plastic,
        name="mount_pad",
    )

    button_0 = model.part("button_0")
    _add_button_part(button_0, material=button_black, cap_x=0.022, cap_y=0.016, cap_z=0.006)
    button_1 = model.part("button_1")
    _add_button_part(button_1, material=button_black, cap_x=0.022, cap_y=0.016, cap_z=0.006)
    button_2 = model.part("button_2")
    _add_button_part(button_2, material=button_black, cap_x=0.022, cap_y=0.016, cap_z=0.006)
    button_3 = model.part("button_3")
    _add_button_part(button_3, material=button_black, cap_x=0.022, cap_y=0.016, cap_z=0.006)

    lift_limits = MotionLimits(
        effort=900.0,
        velocity=0.060,
        lower=0.0,
        upper=lift_travel,
    )
    button_limits = MotionLimits(
        effort=2.0,
        velocity=0.050,
        lower=0.0,
        upper=0.004,
    )
    paddle_limits = MotionLimits(
        effort=4.0,
        velocity=0.050,
        lower=0.0,
        upper=0.006,
    )

    model.articulation(
        "center_lift",
        ArticulationType.PRISMATIC,
        parent=desktop_frame,
        child=center_column,
        origin=Origin(xyz=(0.0, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "left_lift",
        ArticulationType.PRISMATIC,
        parent=desktop_frame,
        child=left_column,
        origin=Origin(xyz=(-column_spacing, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="center_lift"),
    )
    model.articulation(
        "right_lift",
        ArticulationType.PRISMATIC,
        parent=desktop_frame,
        child=right_column,
        origin=Origin(xyz=(column_spacing, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="center_lift"),
    )

    model.articulation(
        "controller_mount",
        ArticulationType.FIXED,
        parent=desktop_frame,
        child=controller,
        origin=Origin(xyz=(0.0, -(top_y * 0.5 - 0.060), -top_z)),
    )
    model.articulation(
        "paddle_press",
        ArticulationType.PRISMATIC,
        parent=controller,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=paddle_limits,
    )

    model.articulation(
        "handset_mount",
        ArticulationType.FIXED,
        parent=desktop_frame,
        child=handset,
        origin=Origin(xyz=(0.560, -(top_y * 0.5 - 0.040), -top_z)),
    )
    button_positions = {
        "button_0": (-0.025, 0.014),
        "button_1": (0.025, 0.014),
        "button_2": (-0.025, -0.014),
        "button_3": (0.025, -0.014),
    }
    for button_name, (button_x_pos, button_y_pos) in button_positions.items():
        model.articulation(
            f"{button_name}_press",
            ArticulationType.PRISMATIC,
            parent=handset,
            child=button_name,
            origin=Origin(xyz=(button_x_pos, button_y_pos, -0.022)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=button_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desktop_frame = object_model.get_part("desktop_frame")
    left_column = object_model.get_part("left_column")
    center_column = object_model.get_part("center_column")
    right_column = object_model.get_part("right_column")
    handset = object_model.get_part("handset")
    controller = object_model.get_part("controller")

    center_lift = object_model.get_articulation("center_lift")
    paddle_press = object_model.get_articulation("paddle_press")
    button_joints = [
        object_model.get_articulation("button_0_press"),
        object_model.get_articulation("button_1_press"),
        object_model.get_articulation("button_2_press"),
        object_model.get_articulation("button_3_press"),
    ]
    button_parts = [
        object_model.get_part("button_0"),
        object_model.get_part("button_1"),
        object_model.get_part("button_2"),
        object_model.get_part("button_3"),
    ]

    columns = (
        ("left", left_column, "left_stage"),
        ("center", center_column, "center_stage"),
        ("right", right_column, "right_stage"),
    )

    lift_limits = center_lift.motion_limits
    if lift_limits is not None and lift_limits.lower is not None and lift_limits.upper is not None:
        with ctx.pose({center_lift: lift_limits.lower}):
            for label, column, stage_name in columns:
                ctx.expect_gap(
                    desktop_frame,
                    column,
                    axis="z",
                    positive_elem=stage_name,
                    min_gap=-10.0,
                    max_gap=-0.45,
                    name=f"{label} stage stays deeply nested at low height",
                )
                ctx.expect_within(
                    desktop_frame,
                    column,
                    axes="xy",
                    inner_elem=stage_name,
                    margin=0.0,
                    name=f"{label} stage stays centered in its column footprint at low height",
                )

        rest_positions = {
            label: ctx.part_world_position(column) for label, column, _ in columns
        }
        with ctx.pose({center_lift: lift_limits.upper}):
            for label, column, stage_name in columns:
                ctx.expect_gap(
                    desktop_frame,
                    column,
                    axis="z",
                    positive_elem=stage_name,
                    min_gap=-10.0,
                    max_gap=-0.16,
                    name=f"{label} stage retains insertion at max lift",
                )
                ctx.expect_within(
                    desktop_frame,
                    column,
                    axes="xy",
                    inner_elem=stage_name,
                    margin=0.0,
                    name=f"{label} stage stays centered in its column footprint at max lift",
                )

            extended_positions = {
                label: ctx.part_world_position(column) for label, column, _ in columns
            }
            travels = []
            for label in ("left", "center", "right"):
                rest_pos = rest_positions.get(label)
                extended_pos = extended_positions.get(label)
                moved_ok = (
                    rest_pos is not None
                    and extended_pos is not None
                    and extended_pos[2] < rest_pos[2] - 0.35
                )
                ctx.check(
                    f"{label} column moves downward as the desk rises",
                    moved_ok,
                    details=f"rest={rest_pos}, extended={extended_pos}",
                )
                if rest_pos is not None and extended_pos is not None:
                    travels.append(rest_pos[2] - extended_pos[2])

            ctx.check(
                "all three lifting columns stay synchronized",
                len(travels) == 3 and max(travels) - min(travels) < 1e-6,
                details=f"travels={travels}",
            )

    paddle_rest = ctx.part_world_position(object_model.get_part("paddle"))
    paddle_limits = paddle_press.motion_limits
    if paddle_limits is not None and paddle_limits.upper is not None:
        with ctx.pose({paddle_press: paddle_limits.upper}):
            paddle_pressed = ctx.part_world_position(object_model.get_part("paddle"))
        ctx.check(
            "front paddle press moves upward into the controller",
            paddle_rest is not None
            and paddle_pressed is not None
            and paddle_pressed[2] > paddle_rest[2] + 0.004,
            details=f"rest={paddle_rest}, pressed={paddle_pressed}",
        )

    button_rest_positions = {
        button_part.name: ctx.part_world_position(button_part) for button_part in button_parts
    }
    for active_joint, active_part in zip(button_joints, button_parts):
        limits = active_joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        with ctx.pose({active_joint: limits.upper}):
            pressed_positions = {
                button_part.name: ctx.part_world_position(button_part)
                for button_part in button_parts
            }
        active_rest = button_rest_positions.get(active_part.name)
        active_pressed = pressed_positions.get(active_part.name)
        ctx.check(
            f"{active_part.name} presses independently",
            active_rest is not None
            and active_pressed is not None
            and active_pressed[2] > active_rest[2] + 0.002
            and all(
                button_name == active_part.name
                or (
                    button_rest_positions.get(button_name) is not None
                    and pressed_positions.get(button_name) is not None
                    and abs(
                        pressed_positions[button_name][2]
                        - button_rest_positions[button_name][2]
                    )
                    < 1e-6
                )
                for button_name in button_rest_positions
            ),
            details=(
                f"active_rest={active_rest}, active_pressed={active_pressed}, "
                f"all_positions={pressed_positions}"
            ),
        )

    for button_name in ("button_0", "button_1", "button_2", "button_3"):
        ctx.expect_overlap(
            button_name,
            handset,
            axes="xy",
            min_overlap=0.010,
            name=f"{button_name} stays within the handset face",
        )

    ctx.expect_overlap(
        "paddle",
        controller,
        axes="xy",
        min_overlap=0.030,
        name="paddle stays centered under the controller housing",
    )

    return ctx.report()


object_model = build_object_model()
