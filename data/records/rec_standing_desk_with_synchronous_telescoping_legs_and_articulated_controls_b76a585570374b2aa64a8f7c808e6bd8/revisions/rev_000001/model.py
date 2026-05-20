from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FOOT_SIZE = (0.09, 0.58, 0.028)
OUTER_COLUMN_SIZE = (0.082, 0.062, 0.58)
OUTER_WALL = 0.011
INNER_STAGE_SIZE = (0.060, 0.040, 0.46)
LEG_X = 0.33
OUTER_TOP_Z = FOOT_SIZE[2] + OUTER_COLUMN_SIZE[2]
STAGE_TRAVEL = 0.26


def _add_leg_base(
    model: ArticulatedObject,
    name: str,
    steel: str,
    trim: str,
):
    leg = model.part(name)
    leg.visual(
        Box(FOOT_SIZE),
        origin=Origin(xyz=(0.0, 0.0, FOOT_SIZE[2] / 2.0)),
        material=steel,
        name="foot",
    )

    column_z = FOOT_SIZE[2] + OUTER_COLUMN_SIZE[2] / 2.0
    outer_x, outer_y, outer_z = OUTER_COLUMN_SIZE
    inner_x = outer_x - 2.0 * OUTER_WALL
    inner_y = outer_y - 2.0 * OUTER_WALL
    side_x = inner_x / 2.0 + OUTER_WALL / 2.0
    side_y = inner_y / 2.0 + OUTER_WALL / 2.0

    leg.visual(
        Box((OUTER_WALL, outer_y, outer_z)),
        origin=Origin(xyz=(-side_x, 0.0, column_z)),
        material=steel,
        name="outer_left_wall",
    )
    leg.visual(
        Box((OUTER_WALL, outer_y, outer_z)),
        origin=Origin(xyz=(side_x, 0.0, column_z)),
        material=steel,
        name="outer_right_wall",
    )
    leg.visual(
        Box((inner_x, OUTER_WALL, outer_z)),
        origin=Origin(xyz=(0.0, -side_y, column_z)),
        material=steel,
        name="outer_front_wall",
    )
    leg.visual(
        Box((inner_x, OUTER_WALL, outer_z)),
        origin=Origin(xyz=(0.0, side_y, column_z)),
        material=steel,
        name="outer_rear_wall",
    )
    leg.visual(
        Box((0.095, 0.085, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_SIZE[2] + 0.009)),
        material=trim,
        name="base_shroud",
    )

    return leg


def _add_button(
    model: ArticulatedObject,
    *,
    name: str,
    parent: str,
    joint_name: str,
    origin: Origin,
    upper: float,
    material: str,
    power: bool = False,
):
    button = model.part(name)
    if power:
        button.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, -0.0015)),
            material=material,
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.004, length=0.009),
            origin=Origin(xyz=(0.0, 0.0, 0.0045)),
            material=material,
            name="stem",
        )
    else:
        button.visual(
            Box((0.016, 0.016, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=material,
            name="cap",
        )
        button.visual(
            Box((0.010, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=material,
            name="stem",
        )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=parent,
        child=button,
        origin=origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=upper,
        ),
    )

    return button


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    steel = model.material("steel", rgba=(0.23, 0.24, 0.26, 1.0))
    trim = model.material("trim", rgba=(0.16, 0.17, 0.19, 1.0))
    wood = model.material("wood", rgba=(0.67, 0.53, 0.39, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    button_black = model.material("button_black", rgba=(0.10, 0.11, 0.12, 1.0))
    button_gray = model.material("button_gray", rgba=(0.35, 0.36, 0.38, 1.0))

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.57, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.24, 0.038)),
        material=steel,
        name="rear_bridge",
    )

    leg_0_base = _add_leg_base(model, "leg_0_base", steel, trim)
    leg_1_base = _add_leg_base(model, "leg_1_base", steel, trim)

    model.articulation(
        "bridge_to_leg_0_base",
        ArticulationType.FIXED,
        parent=bridge,
        child=leg_0_base,
        origin=Origin(xyz=(-LEG_X, 0.0, 0.0)),
    )
    model.articulation(
        "bridge_to_leg_1_base",
        ArticulationType.FIXED,
        parent=bridge,
        child=leg_1_base,
        origin=Origin(xyz=(LEG_X, 0.0, 0.0)),
    )

    for index, x_pos in enumerate((-LEG_X, LEG_X)):
        stage = model.part(f"stage_{index}")
        stage.visual(
            Box(INNER_STAGE_SIZE),
            origin=Origin(xyz=(0.0, 0.0, -0.17)),
            material=graphite,
            name="inner_column",
        )
        stage.visual(
            Box((0.14, 0.16, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.067)),
            material=steel,
            name="top_plate",
        )

        model.articulation(
            f"leg_{index}_base_to_stage_{index}",
            ArticulationType.PRISMATIC,
            parent=f"leg_{index}_base",
            child=stage,
            origin=Origin(xyz=(0.0, 0.0, OUTER_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=240.0,
                velocity=0.06,
                lower=0.0,
                upper=STAGE_TRAVEL,
            ),
            mimic=Mimic(joint="leg_0_base_to_stage_0") if index == 1 else None,
        )

    top_frame = model.part("top_frame")
    top_frame.visual(
        Box((0.24, 0.40, 0.03)),
        origin=Origin(xyz=(-0.28, 0.0, 0.015)),
        material=steel,
        name="plate_0",
    )
    top_frame.visual(
        Box((0.24, 0.40, 0.03)),
        origin=Origin(xyz=(0.28, 0.0, 0.015)),
        material=steel,
        name="plate_1",
    )
    top_frame.visual(
        Box((0.36, 0.14, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=graphite,
        name="center_beam",
    )
    top_frame.visual(
        Box((0.84, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, -0.20, 0.009)),
        material=steel,
        name="front_rail",
    )
    top_frame.visual(
        Box((0.84, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, 0.20, 0.009)),
        material=steel,
        name="rear_rail",
    )

    model.articulation(
        "stage_0_to_top_frame",
        ArticulationType.FIXED,
        parent="stage_0",
        child=top_frame,
        origin=Origin(xyz=(LEG_X, 0.0, 0.074)),
    )

    top = model.part("top")
    top.visual(
        Box((0.98, 0.55, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=wood,
        name="top_panel",
    )
    top.visual(
        Box((0.90, 0.49, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=trim,
        name="underside_liner",
    )

    model.articulation(
        "top_frame_to_top",
        ArticulationType.FIXED,
        parent=top_frame,
        child=top,
        origin=Origin(),
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((0.09, 0.055, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=graphite,
        name="mount_plate",
    )
    control_pod.visual(
        Box((0.004, 0.055, 0.020)),
        origin=Origin(xyz=(-0.043, 0.0, -0.016)),
        material=graphite,
        name="left_wall",
    )
    control_pod.visual(
        Box((0.004, 0.055, 0.020)),
        origin=Origin(xyz=(0.043, 0.0, -0.016)),
        material=graphite,
        name="right_wall",
    )
    control_pod.visual(
        Box((0.082, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, 0.0255, -0.016)),
        material=graphite,
        name="rear_wall",
    )
    control_pod.visual(
        Box((0.082, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.0225, -0.025)),
        material=graphite,
        name="front_lip",
    )

    preset_x = (-0.024, 0.0, 0.024)
    for index, x_pos in enumerate(preset_x):
        control_pod.visual(
            Box((0.002, 0.014, 0.014)),
            origin=Origin(xyz=(x_pos - 0.006, 0.0, -0.013)),
            material=graphite,
            name=f"preset_{index}_guide_left",
        )
        control_pod.visual(
            Box((0.002, 0.014, 0.014)),
            origin=Origin(xyz=(x_pos + 0.006, 0.0, -0.013)),
            material=graphite,
            name=f"preset_{index}_guide_right",
        )

    power_x = 0.033
    for suffix, x_pos, y_pos in (
        ("left", power_x - 0.005, 0.0),
        ("right", power_x + 0.005, 0.0),
        ("front", power_x, -0.005),
        ("rear", power_x, 0.005),
    ):
        size = (0.002, 0.012, 0.014) if suffix in ("left", "right") else (0.012, 0.002, 0.014)
        control_pod.visual(
            Box(size),
            origin=Origin(xyz=(x_pos, y_pos, -0.013)),
            material=graphite,
            name=f"power_guide_{suffix}",
        )

    model.articulation(
        "top_frame_to_control_pod",
        ArticulationType.FIXED,
        parent=top_frame,
        child=control_pod,
        origin=Origin(xyz=(0.38, -0.248, 0.034)),
    )

    _add_button(
        model,
        name="preset_0",
        parent="control_pod",
        joint_name="control_pod_to_preset_0",
        origin=Origin(xyz=(-0.024, 0.0, -0.020)),
        upper=0.0035,
        material=button_gray,
    )
    _add_button(
        model,
        name="preset_1",
        parent="control_pod",
        joint_name="control_pod_to_preset_1",
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        upper=0.0035,
        material=button_gray,
    )
    _add_button(
        model,
        name="preset_2",
        parent="control_pod",
        joint_name="control_pod_to_preset_2",
        origin=Origin(xyz=(0.024, 0.0, -0.020)),
        upper=0.0035,
        material=button_gray,
    )
    _add_button(
        model,
        name="power",
        parent="control_pod",
        joint_name="control_pod_to_power",
        origin=Origin(xyz=(power_x, 0.0, -0.020)),
        upper=0.003,
        material=button_black,
        power=True,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stage_0 = object_model.get_part("stage_0")
    stage_1 = object_model.get_part("stage_1")
    leg_0_base = object_model.get_part("leg_0_base")
    leg_1_base = object_model.get_part("leg_1_base")
    top_frame = object_model.get_part("top_frame")
    top = object_model.get_part("top")
    control_pod = object_model.get_part("control_pod")
    preset_0 = object_model.get_part("preset_0")
    preset_1 = object_model.get_part("preset_1")
    preset_2 = object_model.get_part("preset_2")
    power = object_model.get_part("power")
    stage_joint = object_model.get_articulation("leg_0_base_to_stage_0")
    button_joints = {
        "preset_0": object_model.get_articulation("control_pod_to_preset_0"),
        "preset_1": object_model.get_articulation("control_pod_to_preset_1"),
        "preset_2": object_model.get_articulation("control_pod_to_preset_2"),
        "power": object_model.get_articulation("control_pod_to_power"),
    }
    limits = stage_joint.motion_limits

    ctx.expect_contact(
        stage_0,
        leg_0_base,
        elem_a="inner_column",
        elem_b="outer_left_wall",
        name="stage_0 bears on the left guide wall",
    )
    ctx.expect_contact(
        leg_0_base,
        stage_0,
        elem_a="outer_right_wall",
        elem_b="inner_column",
        name="stage_0 bears on the right guide wall",
    )
    ctx.expect_contact(
        stage_0,
        leg_0_base,
        elem_a="inner_column",
        elem_b="outer_front_wall",
        name="stage_0 bears on the front guide wall",
    )
    ctx.expect_contact(
        leg_0_base,
        stage_0,
        elem_a="outer_rear_wall",
        elem_b="inner_column",
        name="stage_0 bears on the rear guide wall",
    )
    ctx.expect_gap(
        top,
        top_frame,
        axis="z",
        positive_elem="top_panel",
        negative_elem="plate_0",
        min_gap=0.009,
        max_gap=0.012,
        name="top sits above the frame rather than fusing into it",
    )
    ctx.expect_contact(
        top,
        control_pod,
        elem_a="underside_liner",
        elem_b="mount_plate",
        name="control pod mounts directly under the desktop",
    )
    ctx.expect_contact(
        top_frame,
        stage_1,
        elem_a="plate_1",
        elem_b="top_plate",
        name="frame bears on the second lifting stage",
    )

    top_pos = ctx.part_world_position(top)
    pod_pos = ctx.part_world_position(control_pod)
    ctx.check(
        "control pod hangs below the front edge on one corner",
        top_pos is not None
        and pod_pos is not None
        and pod_pos[0] > top_pos[0] + 0.25
        and pod_pos[1] < top_pos[1] - 0.20,
        details=f"top_pos={top_pos}, pod_pos={pod_pos}",
    )

    if limits is not None and limits.upper is not None:
        rest_0 = ctx.part_world_position(stage_0)
        rest_1 = ctx.part_world_position(stage_1)
        with ctx.pose({stage_joint: limits.upper}):
            ctx.expect_overlap(
                stage_0,
                leg_0_base,
                axes="z",
                elem_a="inner_column",
                elem_b="outer_left_wall",
                min_overlap=0.09,
                name="stage_0 keeps retained insertion at full height",
            )
            ctx.expect_overlap(
                stage_1,
                leg_1_base,
                axes="z",
                elem_a="inner_column",
                elem_b="outer_left_wall",
                min_overlap=0.09,
                name="stage_1 keeps retained insertion at full height",
            )
            ctx.expect_contact(
                top_frame,
                stage_1,
                elem_a="plate_1",
                elem_b="top_plate",
                name="second stage stays aligned with the moving frame at full height",
            )
            high_0 = ctx.part_world_position(stage_0)
            high_1 = ctx.part_world_position(stage_1)

        moved_up = (
            rest_0 is not None
            and rest_1 is not None
            and high_0 is not None
            and high_1 is not None
            and high_0[2] > rest_0[2] + 0.20
            and high_1[2] > rest_1[2] + 0.20
        )
        ctx.check(
            "twin stages lift upward together",
            moved_up,
            details=(
                f"rest_0={rest_0}, high_0={high_0}, "
                f"rest_1={rest_1}, high_1={high_1}"
            ),
        )
        synchronized = (
            rest_0 is not None
            and rest_1 is not None
            and high_0 is not None
            and high_1 is not None
            and abs((high_0[2] - rest_0[2]) - (high_1[2] - rest_1[2])) <= 1e-6
        )
        ctx.check(
            "twin stages remain synchronized",
            synchronized,
            details=(
                f"rest_0={rest_0}, high_0={high_0}, "
                f"rest_1={rest_1}, high_1={high_1}"
            ),
        )

    buttons = {
        "preset_0": preset_0,
        "preset_1": preset_1,
        "preset_2": preset_2,
        "power": power,
    }
    rest_positions = {
        name: ctx.part_world_position(part)
        for name, part in buttons.items()
    }
    for name, joint in button_joints.items():
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if upper is None:
            continue
        with ctx.pose({joint: upper}):
            posed_positions = {
                part_name: ctx.part_world_position(part)
                for part_name, part in buttons.items()
            }
        target_moved = (
            rest_positions[name] is not None
            and posed_positions[name] is not None
            and posed_positions[name][2] > rest_positions[name][2] + 0.002
        )
        others_still = True
        for other_name in buttons:
            if other_name == name:
                continue
            rest = rest_positions[other_name]
            posed = posed_positions[other_name]
            if rest is None or posed is None or abs(posed[2] - rest[2]) > 5e-4:
                others_still = False
                break
        ctx.check(
            f"{name} depresses independently",
            target_moved and others_still,
            details=f"rest={rest_positions}, posed={posed_positions}",
        )

    return ctx.report()


object_model = build_object_model()
