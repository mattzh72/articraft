from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


TOP_SIZE = (2.10, 0.80, 0.04)
TOP_FRONT_Y = -TOP_SIZE[1] / 2.0
FOOT_XS = (-0.72, 0.0, 0.72)
BASE_TOP_Z = 0.03
OUTER_HEIGHT = 0.67
LIFT_TRAVEL = 0.42
OUTER_SIZE = (0.092, 0.122)
STAGE_SIZE = (0.074, 0.104)
STAGE_LENGTH = 0.66
STAGE_MOUNT_Z = OUTER_HEIGHT + 0.010


def _column_sleeve_mesh(name: str):
    wall = 0.004
    outer = cq.Workplane("XY").box(
        OUTER_SIZE[0],
        OUTER_SIZE[1],
        OUTER_HEIGHT,
        centered=(True, True, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            OUTER_SIZE[0] - 2.0 * wall,
            OUTER_SIZE[1] - 2.0 * wall,
            OUTER_HEIGHT + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -0.002))
    )
    return mesh_from_cadquery(outer.cut(inner), name)


def _add_outer_column(model: ArticulatedObject, name: str):
    outer = model.part(name)
    outer.visual(
        _column_sleeve_mesh(f"{name}_sleeve"),
        material="steel_dark",
        name="sleeve",
    )
    return outer


def _add_inner_stage(model: ArticulatedObject, name: str):
    stage = model.part(name)
    stage.visual(
        Box((STAGE_SIZE[0], STAGE_SIZE[1], STAGE_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, -STAGE_LENGTH / 2.0)),
        material="steel_light",
        name="stage",
    )
    stage.visual(
        Box((0.180, 0.140, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material="steel_light",
        name="top_plate",
    )
    return stage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_standing_desk")

    model.material("oak_top", rgba=(0.63, 0.49, 0.30, 1.0))
    model.material("steel_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("steel_light", rgba=(0.58, 0.60, 0.64, 1.0))
    model.material("plastic_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("button_gray", rgba=(0.56, 0.57, 0.60, 1.0))
    model.material("power_blue", rgba=(0.26, 0.47, 0.70, 1.0))

    base = model.part("base")
    for index, x_pos in enumerate(FOOT_XS):
        base.visual(
            Box((0.150, 0.700, BASE_TOP_Z)),
            origin=Origin(xyz=(x_pos, 0.0, BASE_TOP_Z / 2.0)),
            material="steel_dark",
            name=f"foot_{index}",
        )
    for index, y_pos in enumerate((-0.220, 0.220)):
        base.visual(
            Box((1.590, 0.050, 0.060)),
            origin=Origin(xyz=(0.0, y_pos, 0.045)),
            material="steel_dark",
            name=f"tie_{index}",
        )

    left_outer = _add_outer_column(model, "left_outer")
    center_outer = _add_outer_column(model, "center_outer")
    right_outer = _add_outer_column(model, "right_outer")

    for x_pos, outer in zip(FOOT_XS, (left_outer, center_outer, right_outer)):
        model.articulation(
            f"base_to_{outer.name}",
            ArticulationType.FIXED,
            parent=base,
            child=outer,
            origin=Origin(xyz=(x_pos, 0.0, BASE_TOP_Z)),
        )

    left_stage = _add_inner_stage(model, "left_stage")
    center_stage = _add_inner_stage(model, "center_stage")
    right_stage = _add_inner_stage(model, "right_stage")

    lift_limits = MotionLimits(
        effort=900.0,
        velocity=0.050,
        lower=0.0,
        upper=LIFT_TRAVEL,
    )

    model.articulation(
        "center_outer_to_center_stage",
        ArticulationType.PRISMATIC,
        parent=center_outer,
        child=center_stage,
        origin=Origin(xyz=(0.0, 0.0, STAGE_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "left_outer_to_left_stage",
        ArticulationType.PRISMATIC,
        parent=left_outer,
        child=left_stage,
        origin=Origin(xyz=(0.0, 0.0, STAGE_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="center_outer_to_center_stage"),
    )
    model.articulation(
        "right_outer_to_right_stage",
        ArticulationType.PRISMATIC,
        parent=right_outer,
        child=right_stage,
        origin=Origin(xyz=(0.0, 0.0, STAGE_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="center_outer_to_center_stage"),
    )

    desktop = model.part("desktop")
    desktop.visual(
        Box(TOP_SIZE),
        origin=Origin(xyz=(0.0, 0.0, TOP_SIZE[2] / 2.0)),
        material="oak_top",
        name="top",
    )
    desktop.visual(
        Box((1.560, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, -0.230, -0.030)),
        material="steel_dark",
        name="front_apron",
    )
    desktop.visual(
        Box((1.560, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, 0.230, -0.030)),
        material="steel_dark",
        name="rear_apron",
    )

    model.articulation(
        "center_stage_to_desktop",
        ArticulationType.FIXED,
        parent=center_stage,
        child=desktop,
        origin=Origin(),
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((0.240, 0.110, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material="plastic_dark",
        name="mount",
    )
    control_pod.visual(
        Box((0.210, 0.084, 0.036)),
        origin=Origin(xyz=(0.0, 0.010, -0.044)),
        material="plastic_dark",
        name="body",
    )
    control_pod.visual(
        Box((0.096, 0.028, 0.008)),
        origin=Origin(xyz=(0.056, -0.010, -0.031)),
        material="button_gray",
        name="bank_bezel",
    )
    control_pod.visual(
        Box((0.238, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, -0.046, -0.043)),
        material="plastic_dark",
        name="control_face",
    )
    control_pod.visual(
        Box((0.238, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -0.039, -0.030)),
        material="plastic_dark",
        name="face_bridge",
    )

    model.articulation(
        "desktop_to_control_pod",
        ArticulationType.FIXED,
        parent=desktop,
        child=control_pod,
        origin=Origin(xyz=(0.0, -0.340, 0.0)),
    )

    paddle = model.part("paddle")
    paddle.visual(
        Box((0.092, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
        material="button_gray",
        name="paddle_cap",
    )
    model.articulation(
        "pod_to_paddle",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=paddle,
        origin=Origin(xyz=(-0.040, -0.048, -0.042)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.050,
            lower=0.0,
            upper=0.006,
        ),
    )

    preset_xs = (0.022, 0.050, 0.078)
    for index, x_pos in enumerate(preset_xs):
        preset = model.part(f"preset_{index}")
        preset.visual(
            Box((0.018, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material="button_gray",
            name="button",
        )
        model.articulation(
            f"pod_to_preset_{index}",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=preset,
            origin=Origin(xyz=(x_pos, -0.048, -0.044)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.050,
                lower=0.0,
                upper=0.004,
            ),
        )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="power_blue",
        name="button",
    )
    model.articulation(
        "pod_to_power_button",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=power_button,
        origin=Origin(xyz=(0.108, -0.048, -0.044)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.050,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desktop = object_model.get_part("desktop")
    left_outer = object_model.get_part("left_outer")
    center_outer = object_model.get_part("center_outer")
    right_outer = object_model.get_part("right_outer")
    left_stage = object_model.get_part("left_stage")
    center_stage = object_model.get_part("center_stage")
    right_stage = object_model.get_part("right_stage")
    control_pod = object_model.get_part("control_pod")
    paddle = object_model.get_part("paddle")
    preset_0 = object_model.get_part("preset_0")
    preset_1 = object_model.get_part("preset_1")
    preset_2 = object_model.get_part("preset_2")
    power_button = object_model.get_part("power_button")
    center_lift = object_model.get_articulation("center_outer_to_center_stage")
    paddle_joint = object_model.get_articulation("pod_to_paddle")
    preset_0_joint = object_model.get_articulation("pod_to_preset_0")
    preset_1_joint = object_model.get_articulation("pod_to_preset_1")
    preset_2_joint = object_model.get_articulation("pod_to_preset_2")
    power_joint = object_model.get_articulation("pod_to_power_button")

    for stage, outer, label in (
        (left_stage, left_outer, "left"),
        (center_stage, center_outer, "center"),
        (right_stage, right_outer, "right"),
    ):
        ctx.expect_within(
            stage,
            outer,
            axes="xy",
            inner_elem="stage",
            outer_elem="sleeve",
            margin=0.004,
            name=f"{label} stage stays centered in its sleeve at rest",
        )
        ctx.expect_overlap(
            stage,
            outer,
            axes="z",
            elem_a="stage",
            elem_b="sleeve",
            min_overlap=0.62,
            name=f"{label} stage remains deeply inserted at rest",
        )
        ctx.expect_contact(
            stage,
            desktop,
            elem_a="top_plate",
            elem_b="top",
            name=f"{label} stage top plate supports the desktop",
        )

    rest_positions = {
        "desktop": ctx.part_world_position(desktop),
        "left_stage": ctx.part_world_position(left_stage),
        "center_stage": ctx.part_world_position(center_stage),
        "right_stage": ctx.part_world_position(right_stage),
    }

    with ctx.pose({center_lift: LIFT_TRAVEL}):
        for stage, outer, label in (
            (left_stage, left_outer, "left"),
            (center_stage, center_outer, "center"),
            (right_stage, right_outer, "right"),
        ):
            ctx.expect_within(
                stage,
                outer,
                axes="xy",
                inner_elem="stage",
                outer_elem="sleeve",
                margin=0.004,
                name=f"{label} stage stays centered in its sleeve when raised",
            )
            ctx.expect_overlap(
                stage,
                outer,
                axes="z",
                elem_a="stage",
                elem_b="sleeve",
                min_overlap=0.20,
                name=f"{label} stage keeps retained insertion when raised",
            )
            ctx.expect_contact(
                stage,
                desktop,
                elem_a="top_plate",
                elem_b="top",
                name=f"{label} stage still supports the desktop when raised",
            )

        raised_positions = {
            "desktop": ctx.part_world_position(desktop),
            "left_stage": ctx.part_world_position(left_stage),
            "center_stage": ctx.part_world_position(center_stage),
            "right_stage": ctx.part_world_position(right_stage),
        }

    for key in ("left_stage", "center_stage", "right_stage", "desktop"):
        ctx.check(
            f"{key} rises through standing-desk travel",
            rest_positions[key] is not None
            and raised_positions[key] is not None
            and raised_positions[key][2] - rest_positions[key][2] > 0.40,
            details=f"rest={rest_positions[key]}, raised={raised_positions[key]}",
        )

    deltas = []
    for key in ("left_stage", "center_stage", "right_stage", "desktop"):
        if rest_positions[key] is not None and raised_positions[key] is not None:
            deltas.append(raised_positions[key][2] - rest_positions[key][2])
    ctx.check(
        "all lift members move in sync",
        len(deltas) == 4 and max(deltas) - min(deltas) < 1e-6,
        details=f"deltas={deltas}",
    )

    control_pos = ctx.part_world_position(control_pod)
    ctx.expect_gap(
        desktop,
        control_pod,
        axis="z",
        positive_elem="top",
        negative_elem="body",
        min_gap=0.020,
        max_gap=0.040,
        name="control pod body hangs below the desktop",
    )
    ctx.check(
        "control pod sits beneath the front center edge",
        control_pos is not None
        and abs(control_pos[0]) < 1e-6
        and -0.390 < control_pos[1] < -0.280,
        details=f"control_pos={control_pos}",
    )

    ctx.allow_overlap(
        control_pod,
        paddle,
        elem_a="control_face",
        elem_b="paddle_cap",
        reason="The paddle retracts into a simplified controller face proxy when pressed.",
    )
    for preset in (preset_0, preset_1, preset_2, power_button):
        ctx.allow_overlap(
            control_pod,
            preset,
            elem_a="control_face",
            elem_b="button",
            reason="The button caps retract into a simplified controller face proxy when pressed.",
        )

    button_parts = {
        "preset_0": preset_0,
        "preset_1": preset_1,
        "preset_2": preset_2,
        "power_button": power_button,
    }
    rest_button_positions = {
        name: ctx.part_world_position(part)
        for name, part in button_parts.items()
    }

    for active_name, joint in (
        ("preset_0", preset_0_joint),
        ("preset_1", preset_1_joint),
        ("preset_2", preset_2_joint),
        ("power_button", power_joint),
    ):
        joint_upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: joint_upper}):
            posed_positions = {
                name: ctx.part_world_position(part)
                for name, part in button_parts.items()
            }
        active_ok = (
            joint_upper is not None
            and rest_button_positions[active_name] is not None
            and posed_positions[active_name] is not None
            and posed_positions[active_name][1] - rest_button_positions[active_name][1]
            > joint_upper * 0.75
        )
        neighbors_ok = all(
            rest_button_positions[name] is not None
            and posed_positions[name] is not None
            and abs(posed_positions[name][1] - rest_button_positions[name][1]) < 1e-9
            for name in button_parts
            if name != active_name
        )
        ctx.check(
            f"{active_name} depresses independently",
            active_ok and neighbors_ok,
            details=f"rest={rest_button_positions}, posed={posed_positions}",
        )

    paddle_rest = ctx.part_world_position(paddle)
    paddle_upper = paddle_joint.motion_limits.upper if paddle_joint.motion_limits is not None else None
    with ctx.pose({paddle_joint: paddle_upper}):
        paddle_pressed = ctx.part_world_position(paddle)
    ctx.check(
        "front paddle depresses rearward",
        paddle_rest is not None
        and paddle_pressed is not None
        and paddle_upper is not None
        and paddle_pressed[1] - paddle_rest[1] > paddle_upper * 0.75,
        details=f"rest={paddle_rest}, pressed={paddle_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
