from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.34
BASE_WIDTH = 0.22
BASE_HEIGHT = 0.035

LOWER_ARM_LENGTH = 0.54
UPPER_ARM_LENGTH = 0.46

HEAD_LENGTH = 0.20
HEAD_WIDTH = 0.082
HEAD_HEIGHT = 0.032

BUTTON_CAP_LENGTH = 0.024
BUTTON_CAP_HEIGHT = 0.016
BUTTON_CAP_PROUD = 0.003
BUTTON_STEM_LENGTH = 0.016
BUTTON_STEM_HEIGHT = 0.010
BUTTON_TRAVEL = 0.003


def _head_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(HEAD_LENGTH, HEAD_WIDTH, HEAD_HEIGHT)
    shell = shell.translate((HEAD_LENGTH / 2.0 + 0.010, 0.0, -0.010))
    shell = shell.edges("|X").fillet(0.003)

    cap_recess = (
        cq.Workplane("XY")
        .box(BUTTON_CAP_LENGTH, BUTTON_CAP_PROUD + 0.0006, BUTTON_CAP_HEIGHT)
        .translate((0.136, HEAD_WIDTH / 2.0 - (BUTTON_CAP_PROUD + 0.0006) / 2.0, -0.008))
    )
    stem_recess = (
        cq.Workplane("XY")
        .box(BUTTON_STEM_LENGTH, BUTTON_TRAVEL + 0.0100, BUTTON_STEM_HEIGHT)
        .translate((0.136, HEAD_WIDTH / 2.0 - (BUTTON_TRAVEL + 0.0100) / 2.0, -0.008))
    )

    return shell.cut(cap_recess).cut(stem_recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="task_floor_lamp")

    model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("satin_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("diffuser", rgba=(0.94, 0.95, 0.92, 0.95))
    model.material("button_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material="powder_black",
        name="base_plate",
    )
    base.visual(
        Box((0.065, 0.085, 0.110)),
        origin=Origin(xyz=(-0.115, 0.0, BASE_HEIGHT + 0.055)),
        material="charcoal",
        name="pedestal",
    )
    base.visual(
        Box((0.048, 0.070, 0.030)),
        origin=Origin(xyz=(-0.102, 0.0, 0.160)),
        material="charcoal",
        name="mount_block",
    )
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        base.visual(
            Box((0.028, 0.008, 0.050)),
            origin=Origin(xyz=(-0.064, side * 0.021, 0.170)),
            material="satin_steel",
            name=f"shoulder_cheek_{suffix}",
        )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.0155, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="satin_steel",
        name="shoulder_barrel",
    )
    lower_arm.visual(
        Box((0.050, 0.026, 0.026)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material="satin_steel",
        name="shoulder_block",
    )
    lower_arm.visual(
        Box((0.440, 0.022, 0.018)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material="satin_steel",
        name="lower_beam",
    )
    lower_arm.visual(
        Box((0.050, 0.026, 0.026)),
        origin=Origin(xyz=(LOWER_ARM_LENGTH - 0.050, 0.0, 0.0)),
        material="satin_steel",
        name="elbow_block",
    )
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        lower_arm.visual(
            Box((0.030, 0.006, 0.042)),
            origin=Origin(xyz=(LOWER_ARM_LENGTH - 0.015, side * 0.015, 0.0)),
            material="satin_steel",
            name=f"elbow_cheek_{suffix}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.0125, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="satin_steel",
        name="elbow_barrel",
    )
    upper_arm.visual(
        Box((0.044, 0.024, 0.022)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material="satin_steel",
        name="elbow_hub",
    )
    upper_arm.visual(
        Box((0.370, 0.020, 0.016)),
        origin=Origin(xyz=(0.211, 0.0, 0.0)),
        material="satin_steel",
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.048, 0.024, 0.020)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.040, 0.0, 0.0)),
        material="satin_steel",
        name="head_block",
    )
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        upper_arm.visual(
            Box((0.032, 0.005, 0.032)),
            origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.016, side * 0.014, 0.0)),
            material="satin_steel",
            name=f"head_cheek_{suffix}",
        )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell_shape(), "head_shell"),
        material="powder_black",
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.0105, length=0.023),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="satin_steel",
        name="tilt_barrel",
    )
    head.visual(
        Box((0.150, 0.060, 0.0025)),
        origin=Origin(xyz=(0.122, 0.0, -0.02475)),
        material="diffuser",
        name="lens",
    )

    button = model.part("side_button")
    button.visual(
        Box((BUTTON_STEM_LENGTH, 0.010, BUTTON_STEM_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material="button_black",
        name="button_stem",
    )
    button.visual(
        Box((BUTTON_CAP_LENGTH, BUTTON_CAP_PROUD, BUTTON_CAP_HEIGHT)),
        origin=Origin(xyz=(0.0, BUTTON_CAP_PROUD / 2.0, 0.0)),
        material="button_black",
        name="button_cap",
    )

    shoulder = model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(-0.060, 0.0, 0.170), rpy=(0.0, -1.12, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.55,
            upper=0.35,
            effort=28.0,
            velocity=0.9,
        ),
    )
    elbow = model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.42, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.95,
            upper=0.90,
            effort=18.0,
            velocity=1.1,
        ),
    )
    head_tilt = model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.45, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.55,
            upper=0.85,
            effort=6.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "button_slide",
        ArticulationType.PRISMATIC,
        parent=head,
        child=button,
        origin=Origin(xyz=(0.136, HEAD_WIDTH / 2.0, -0.008)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=BUTTON_TRAVEL,
            effort=1.0,
            velocity=0.04,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head = object_model.get_part("head")
    button = object_model.get_part("side_button")

    shoulder = object_model.get_articulation("shoulder_hinge")
    elbow = object_model.get_articulation("elbow_hinge")
    head_tilt = object_model.get_articulation("head_tilt")
    button_slide = object_model.get_articulation("button_slide")

    ctx.allow_overlap(
        head,
        button,
        elem_a="head_shell",
        elem_b="button_stem",
        reason="The visible button cap is articulated explicitly, while the hidden plunger stem is simplified as running inside the lamp head shell.",
    )

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base sits on the floor",
        base_aabb is not None and abs(base_aabb[0][2]) <= 1e-6,
        details=f"base_aabb={base_aabb}",
    )
    ctx.expect_gap(
        head,
        base,
        axis="z",
        min_gap=0.68,
        name="head is elevated well above the floor base",
    )
    ctx.expect_within(
        button,
        head,
        axes="xz",
        margin=0.010,
        name="button stays within the head side footprint",
    )

    rest_upper_pos = ctx.part_world_position(upper_arm)
    shoulder_upper = shoulder.motion_limits.upper if shoulder.motion_limits is not None else None
    with ctx.pose({shoulder: shoulder_upper}):
        raised_upper_pos = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder hinge raises the elbow",
        rest_upper_pos is not None
        and raised_upper_pos is not None
        and shoulder_upper is not None
        and raised_upper_pos[2] > rest_upper_pos[2] + 0.045,
        details=f"rest={rest_upper_pos}, raised={raised_upper_pos}, q={shoulder_upper}",
    )

    rest_head_pos = ctx.part_world_position(head)
    elbow_upper = elbow.motion_limits.upper if elbow.motion_limits is not None else None
    with ctx.pose({elbow: elbow_upper}):
        elbow_head_pos = ctx.part_world_position(head)
    ctx.check(
        "elbow hinge lifts the head",
        rest_head_pos is not None
        and elbow_head_pos is not None
        and elbow_upper is not None
        and elbow_head_pos[2] > rest_head_pos[2] + 0.08,
        details=f"rest={rest_head_pos}, raised={elbow_head_pos}, q={elbow_upper}",
    )

    rest_lens = ctx.part_element_world_aabb(head, elem="lens")
    head_tilt_upper = head_tilt.motion_limits.upper if head_tilt.motion_limits is not None else None
    with ctx.pose({head_tilt: head_tilt_upper}):
        tilted_lens = ctx.part_element_world_aabb(head, elem="lens")
    ctx.check(
        "head tilt points the light downward",
        rest_lens is not None
        and tilted_lens is not None
        and head_tilt_upper is not None
        and tilted_lens[0][2] < rest_lens[0][2] - 0.02,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}, q={head_tilt_upper}",
    )

    rest_button_pos = ctx.part_world_position(button)
    button_upper = button_slide.motion_limits.upper if button_slide.motion_limits is not None else None
    with ctx.pose({button_slide: button_upper}):
        pressed_button_pos = ctx.part_world_position(button)
    ctx.check(
        "side button depresses inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and button_upper is not None
        and pressed_button_pos[1] < rest_button_pos[1] - 0.002,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}, q={button_upper}",
    )

    return ctx.report()


object_model = build_object_model()
