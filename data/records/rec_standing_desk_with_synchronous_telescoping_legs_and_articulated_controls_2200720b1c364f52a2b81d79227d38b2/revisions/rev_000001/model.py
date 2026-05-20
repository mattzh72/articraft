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

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq
# from sdk import mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standing_desk")

    top = model.material("top_oak", rgba=(0.67, 0.56, 0.44, 1.0))
    frame_metal = model.material("frame_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    stage_metal = model.material("stage_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    button_dark = model.material("button_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    desk_width = 1.50
    desk_depth = 0.75
    top_thickness = 0.03
    foot_width = 0.09
    foot_depth = 0.70
    foot_height = 0.03
    outer_width = 0.09
    outer_depth = 0.06
    outer_height = 0.64
    inner_width = 0.074
    inner_depth = 0.055
    inner_height = 0.55
    wall = 0.0025
    leg_x = 0.52
    lift_travel = 0.34
    low_top_height = 0.765
    handset_mount_x = 0.42
    handset_mount_y = -desk_depth / 2.0 + 0.055

    frame = model.part("frame")
    frame.visual(
        Box((foot_width, foot_depth, foot_height)),
        origin=Origin(xyz=(-leg_x, 0.0, foot_height / 2.0)),
        material=frame_metal,
        name="left_foot",
    )
    frame.visual(
        Box((foot_width, foot_depth, foot_height)),
        origin=Origin(xyz=(leg_x, 0.0, foot_height / 2.0)),
        material=frame_metal,
        name="right_foot",
    )
    for prefix, x_sign in (("left", -1.0), ("right", 1.0)):
        x_pos = x_sign * leg_x
        frame.visual(
            Box((outer_width, wall, outer_height)),
            origin=Origin(
                xyz=(x_pos, outer_depth / 2.0 - wall / 2.0, foot_height + outer_height / 2.0)
            ),
            material=frame_metal,
            name=f"{prefix}_outer_front",
        )
        frame.visual(
            Box((outer_width, wall, outer_height)),
            origin=Origin(
                xyz=(x_pos, -outer_depth / 2.0 + wall / 2.0, foot_height + outer_height / 2.0)
            ),
            material=frame_metal,
            name=f"{prefix}_outer_rear",
        )
        frame.visual(
            Box((wall, outer_depth - 2.0 * wall, outer_height)),
            origin=Origin(
                xyz=(
                    x_pos - outer_width / 2.0 + wall / 2.0,
                    0.0,
                    foot_height + outer_height / 2.0,
                )
            ),
            material=frame_metal,
            name=f"{prefix}_outer_inner",
        )
        frame.visual(
            Box((wall, outer_depth - 2.0 * wall, outer_height)),
            origin=Origin(
                xyz=(
                    x_pos + outer_width / 2.0 - wall / 2.0,
                    0.0,
                    foot_height + outer_height / 2.0,
                )
            ),
            material=frame_metal,
            name=f"{prefix}_outer_outer",
        )
    frame.visual(
        Box((0.98, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.23, 0.04)),
        material=frame_metal,
        name="rear_stretcher",
    )

    left_stage = model.part("left_stage")
    left_stage.visual(
        Box((inner_width, inner_depth, inner_height)),
        origin=Origin(xyz=(0.0, 0.0, -0.21)),
        material=stage_metal,
        name="inner_column",
    )

    right_stage = model.part("right_stage")
    right_stage.visual(
        Box((inner_width, inner_depth, inner_height)),
        origin=Origin(xyz=(0.0, 0.0, -0.21)),
        material=stage_metal,
        name="inner_column",
    )

    desktop = model.part("desktop")
    desktop_center_z = low_top_height - top_thickness / 2.0
    desktop.visual(
        Box((desk_width, desk_depth, top_thickness)),
        material=top,
        name="top_panel",
    )
    desktop.visual(
        Box((0.94, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=frame_metal,
        name="crossbeam",
    )
    desktop.visual(
        Box((1.16, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, -0.24, -0.035)),
        material=frame_metal,
        name="front_rail",
    )
    desktop.visual(
        Box((1.16, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.24, -0.035)),
        material=frame_metal,
        name="rear_rail",
    )

    lift_limits = MotionLimits(
        effort=900.0,
        velocity=0.05,
        lower=0.0,
        upper=lift_travel,
    )
    model.articulation(
        "left_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_stage,
        origin=Origin(xyz=(-leg_x, 0.0, foot_height + outer_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "right_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_stage,
        origin=Origin(xyz=(leg_x, 0.0, foot_height + outer_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.05,
            lower=0.0,
            upper=lift_travel,
        ),
        mimic=Mimic("left_lift"),
    )
    model.articulation(
        "stage_to_desktop",
        ArticulationType.FIXED,
        parent=left_stage,
        child=desktop,
        origin=Origin(xyz=(leg_x, 0.0, 0.08)),
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.045, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=frame_metal,
        name="mount_pad",
    )
    handset.visual(
        Box((0.130, 0.060, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=frame_metal,
        name="body",
    )
    handset.visual(
        Box((0.112, 0.036, 0.010)),
        origin=Origin(xyz=(-0.004, 0.0, -0.035)),
        material=frame_metal,
        name="nose",
    )

    model.articulation(
        "desktop_to_handset",
        ArticulationType.FIXED,
        parent=desktop,
        child=handset,
        origin=Origin(xyz=(handset_mount_x, handset_mount_y, -top_thickness / 2.0)),
    )

    button_specs = (
        ("preset_1", -0.032, Box((0.018, 0.018, 0.004))),
        ("preset_2", 0.0, Box((0.018, 0.018, 0.004))),
        ("preset_3", 0.032, Box((0.018, 0.018, 0.004))),
        ("power_button", 0.054, Box((0.013, 0.013, 0.004))),
    )
    for button_name, x_pos, geometry in button_specs:
        button = model.part(button_name)
        button.visual(
            geometry,
            material=button_dark,
            name="cap",
        )
        model.articulation(
            f"{button_name}_press",
            ArticulationType.PRISMATIC,
            parent=handset,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, -0.032)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.03,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_stage = object_model.get_part("left_stage")
    right_stage = object_model.get_part("right_stage")
    desktop = object_model.get_part("desktop")
    handset = object_model.get_part("handset")
    preset_1 = object_model.get_part("preset_1")
    preset_2 = object_model.get_part("preset_2")
    preset_3 = object_model.get_part("preset_3")
    power_button = object_model.get_part("power_button")
    left_lift = object_model.get_articulation("left_lift")
    lift_limits = left_lift.motion_limits
    preset_1_press = object_model.get_articulation("preset_1_press")
    preset_2_press = object_model.get_articulation("preset_2_press")
    preset_3_press = object_model.get_articulation("preset_3_press")
    power_button_press = object_model.get_articulation("power_button_press")

    def check_stage_guides(stage, side: str, overlap_rest: float, overlap_raised: float) -> None:
        ctx.expect_gap(
            stage,
            frame,
            axis="x",
            positive_elem="inner_column",
            negative_elem=f"{side}_outer_inner",
            min_gap=0.005,
            max_gap=0.0065,
            name=f"{side} stage clears inner sleeve wall",
        )
        ctx.expect_gap(
            frame,
            stage,
            axis="x",
            positive_elem=f"{side}_outer_outer",
            negative_elem="inner_column",
            min_gap=0.005,
            max_gap=0.0065,
            name=f"{side} stage clears outer sleeve wall",
        )
        ctx.expect_gap(
            frame,
            stage,
            axis="y",
            positive_elem=f"{side}_outer_front",
            negative_elem="inner_column",
            max_gap=1e-5,
            max_penetration=1e-5,
            name=f"{side} stage clears front sleeve wall",
        )
        ctx.expect_gap(
            stage,
            frame,
            axis="y",
            positive_elem="inner_column",
            negative_elem=f"{side}_outer_rear",
            max_gap=1e-5,
            max_penetration=1e-5,
            name=f"{side} stage clears rear sleeve wall",
        )
        ctx.expect_overlap(
            stage,
            frame,
            axes="z",
            elem_a="inner_column",
            elem_b=f"{side}_outer_front",
            min_overlap=overlap_rest,
            name=f"{side} stage remains inserted in sleeve",
        )

    if lift_limits is not None and lift_limits.upper is not None:
        with ctx.pose({left_lift: 0.0}):
            check_stage_guides(left_stage, "left", overlap_rest=0.45, overlap_raised=0.12)
            check_stage_guides(right_stage, "right", overlap_rest=0.45, overlap_raised=0.12)
            ctx.expect_gap(
                desktop,
                left_stage,
                axis="z",
                positive_elem="top_panel",
                negative_elem="inner_column",
                max_gap=0.0,
                max_penetration=1e-5,
                name="desktop sits on left stage at rest",
            )
            ctx.expect_gap(
                desktop,
                right_stage,
                axis="z",
                positive_elem="top_panel",
                negative_elem="inner_column",
                max_gap=0.0,
                max_penetration=1e-5,
                name="desktop sits on right stage at rest",
            )

        desktop_rest = ctx.part_world_position(desktop)
        left_rest = ctx.part_world_position(left_stage)
        right_rest = ctx.part_world_position(right_stage)

        with ctx.pose({left_lift: lift_limits.upper}):
            check_stage_guides(left_stage, "left", overlap_rest=0.12, overlap_raised=0.12)
            check_stage_guides(right_stage, "right", overlap_rest=0.12, overlap_raised=0.12)
            ctx.expect_gap(
                desktop,
                left_stage,
                axis="z",
                positive_elem="top_panel",
                negative_elem="inner_column",
                max_gap=0.0,
                max_penetration=1e-5,
                name="desktop stays seated on left stage when raised",
            )
            ctx.expect_gap(
                desktop,
                right_stage,
                axis="z",
                positive_elem="top_panel",
                negative_elem="inner_column",
                max_gap=0.0,
                max_penetration=1e-5,
                name="desktop stays seated on right stage when raised",
            )
            desktop_raised = ctx.part_world_position(desktop)
            left_raised = ctx.part_world_position(left_stage)
            right_raised = ctx.part_world_position(right_stage)

        ctx.check(
            "desktop rises through usable standing range",
            desktop_rest is not None
            and desktop_raised is not None
            and desktop_raised[2] > desktop_rest[2] + 0.25,
            details=f"rest={desktop_rest}, raised={desktop_raised}",
        )
        ctx.check(
            "paired lift stages stay synchronized",
            left_rest is not None
            and right_rest is not None
            and left_raised is not None
            and right_raised is not None
            and abs(left_rest[2] - right_rest[2]) < 1e-6
            and abs(left_raised[2] - right_raised[2]) < 1e-6,
            details=(
                f"left_rest={left_rest}, right_rest={right_rest}, "
                f"left_raised={left_raised}, right_raised={right_raised}"
            ),
        )

    ctx.expect_gap(
        desktop,
        handset,
        axis="z",
        positive_elem="top_panel",
        negative_elem="mount_pad",
        max_gap=0.0,
        max_penetration=1e-5,
        name="handset mount pad sits against desktop underside",
    )
    ctx.expect_gap(
        desktop,
        handset,
        axis="z",
        positive_elem="top_panel",
        negative_elem="body",
        min_gap=0.005,
        name="handset body hangs below the desktop",
    )
    ctx.expect_origin_gap(
        desktop,
        handset,
        axis="y",
        min_gap=0.20,
        name="handset sits toward the front edge",
    )

    buttons = {
        "preset_1": (preset_1, preset_1_press),
        "preset_2": (preset_2, preset_2_press),
        "preset_3": (preset_3, preset_3_press),
        "power_button": (power_button, power_button_press),
    }
    button_rest = {name: ctx.part_world_position(part) for name, (part, _) in buttons.items()}

    for button_name, (button_part, button_joint) in buttons.items():
        limits = button_joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        with ctx.pose({button_joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button_part)
            peer_positions = {
                name: ctx.part_world_position(part)
                for name, (part, _) in buttons.items()
                if name != button_name
            }
        ctx.check(
            f"{button_name} depresses independently",
            button_rest[button_name] is not None
            and pressed_pos is not None
            and pressed_pos[2] > button_rest[button_name][2] + 0.002
            and all(
                button_rest[name] is not None
                and peer_positions[name] is not None
                and abs(peer_positions[name][2] - button_rest[name][2]) < 1e-6
                for name in peer_positions
            ),
            details=(
                f"rest={button_rest[button_name]}, pressed={pressed_pos}, "
                f"peers={peer_positions}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
