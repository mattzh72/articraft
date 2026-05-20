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


BASE_RADIUS = 0.120
BASE_THICKNESS = 0.028
SHOULDER_HEIGHT = 0.109
LOWER_ARM_LENGTH = 0.298
UPPER_ARM_LENGTH = 0.320


def _x_cylinder(radius: float, length: float, *, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(
        xyz=xyz,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def _y_cylinder(radius: float, length: float, *, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(
        xyz=xyz,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_task_lamp")

    base_black = model.material("base_black", rgba=(0.12, 0.12, 0.13, 1.0))
    arm_aluminum = model.material("arm_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    head_charcoal = model.material("head_charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.92, 0.93, 0.94, 0.9))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_black,
        name="weight_disk",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.006)),
        material=base_black,
        name="cap_ring",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=base_black,
        name="pivot_post",
    )
    base.visual(
        Box((0.014, 0.084, 0.028)),
        origin=Origin(xyz=(-0.021, 0.0, 0.091)),
        material=base_black,
        name="rear_bridge",
    )
    for index, y_offset in enumerate((-0.037, 0.037)):
        base.visual(
            Box((0.020, 0.010, 0.050)),
            origin=Origin(xyz=(-0.014, y_offset, 0.098)),
            material=hinge_gray,
            name=f"cheek_{index}",
        )

    lower_arm = model.part("lower_arm")
    root_hub, root_hub_origin = _y_cylinder(0.011, 0.058, xyz=(0.015, 0.0, 0.0))
    lower_arm.visual(root_hub, origin=root_hub_origin, material=hinge_gray, name="root_hub")
    front_hub, front_hub_origin = _y_cylinder(0.010, 0.058, xyz=(0.283, 0.0, 0.0))
    lower_arm.visual(front_hub, origin=front_hub_origin, material=hinge_gray, name="front_hub")
    for index, y_offset in enumerate((-0.022, 0.022)):
        lower_arm.visual(
            Box((0.274, 0.008, 0.014)),
            origin=Origin(xyz=(0.152, y_offset, 0.0)),
            material=arm_aluminum,
            name=f"bar_{index}",
        )
    tension_strut, tension_strut_origin = _x_cylinder(0.0045, 0.060, xyz=(0.255, 0.0, 0.013))
    lower_arm.visual(
        tension_strut,
        origin=tension_strut_origin,
        material=hinge_gray,
        name="tension_strut",
    )
    tension_pod, tension_pod_origin = _y_cylinder(0.010, 0.038, xyz=(0.220, 0.0, 0.013))
    lower_arm.visual(
        tension_pod,
        origin=tension_pod_origin,
        material=hinge_gray,
        name="tension_pod",
    )
    lower_arm.visual(
        Box((0.008, 0.040, 0.018)),
        origin=Origin(xyz=(0.294, 0.0, 0.0)),
        material=hinge_gray,
        name="elbow_plate",
    )

    upper_arm = model.part("upper_arm")
    upper_root_hub, upper_root_origin = _y_cylinder(0.008, 0.052, xyz=(0.014, 0.0, 0.0))
    upper_arm.visual(
        upper_root_hub,
        origin=upper_root_origin,
        material=hinge_gray,
        name="root_hub",
    )
    upper_tip_hub, upper_tip_origin = _y_cylinder(0.010, 0.052, xyz=(0.306, 0.0, 0.0))
    upper_arm.visual(
        upper_tip_hub,
        origin=upper_tip_origin,
        material=hinge_gray,
        name="tip_hub",
    )
    for index, y_offset in enumerate((-0.021, 0.021)):
        upper_arm.visual(
            Box((0.288, 0.007, 0.012)),
            origin=Origin(xyz=(0.158, y_offset, 0.0)),
            material=arm_aluminum,
            name=f"bar_{index}",
        )
    upper_arm.visual(
        Box((0.026, 0.042, 0.020)),
        origin=Origin(xyz=(0.296, 0.0, 0.0)),
        material=hinge_gray,
        name="tip_block",
    )
    upper_arm.visual(
        Box((0.008, 0.038, 0.018)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=hinge_gray,
        name="root_plate",
    )
    upper_arm.visual(
        Box((0.008, 0.038, 0.018)),
        origin=Origin(xyz=(0.316, 0.0, 0.0)),
        material=hinge_gray,
        name="tip_plate",
    )

    head = model.part("head")
    head_root, head_root_origin = _y_cylinder(0.008, 0.044, xyz=(0.014, 0.0, 0.0))
    head.visual(head_root, origin=head_root_origin, material=hinge_gray, name="hinge_barrel")
    head.visual(
        Box((0.008, 0.032, 0.018)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=hinge_gray,
        name="hinge_plate",
    )
    head.visual(
        Box((0.030, 0.028, 0.020)),
        origin=Origin(xyz=(0.022, 0.0, -0.001)),
        material=hinge_gray,
        name="neck_block",
    )
    head.visual(
        Box((0.210, 0.084, 0.015)),
        origin=Origin(xyz=(0.122, 0.0, -0.006)),
        material=head_charcoal,
        name="head_panel",
    )
    head.visual(
        Box((0.118, 0.034, 0.006)),
        origin=Origin(xyz=(0.138, 0.0, 0.004)),
        material=hinge_gray,
        name="top_rib",
    )
    head.visual(
        Box((0.174, 0.058, 0.004)),
        origin=Origin(xyz=(0.132, 0.0, -0.012)),
        material=diffuser_white,
        name="diffuser",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_HEIGHT), rpy=(0.0, -0.72, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=-0.50,
            upper=0.88,
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.46, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.6,
            lower=-0.80,
            upper=0.90,
        ),
    )
    model.articulation(
        "head_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.26, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.65,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    head = object_model.get_part("head")
    base_hinge = object_model.get_articulation("base_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    head_hinge = object_model.get_articulation("head_hinge")

    ctx.allow_overlap(
        lower_arm,
        "upper_arm",
        elem_a="elbow_plate",
        elem_b="root_plate",
        reason="The upper link is intentionally nested into the elbow hinge clevis around the shared pivot.",
    )

    ctx.expect_gap(
        head,
        base,
        axis="z",
        min_gap=0.18,
        name="head clears the weighted base at rest",
    )
    ctx.expect_origin_gap(
        head,
        base,
        axis="x",
        min_gap=0.45,
        name="head projects forward from the base",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose(
        {
            base_hinge: 0.45,
            elbow_hinge: 0.35,
        }
    ):
        raised_head_pos = ctx.part_world_position(head)

    ctx.check(
        "arm chain raises the lamp head",
        rest_head_pos is not None
        and raised_head_pos is not None
        and raised_head_pos[2] > rest_head_pos[2] + 0.17,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    with ctx.pose({head_hinge: -0.45}):
        raised_panel = ctx.part_element_world_aabb(head, elem="head_panel")
    with ctx.pose({head_hinge: 0.45}):
        lowered_panel = ctx.part_element_world_aabb(head, elem="head_panel")

    ctx.check(
        "head hinge changes the panel pitch",
        raised_panel is not None
        and lowered_panel is not None
        and lowered_panel[0][2] < raised_panel[0][2] - 0.035,
        details=f"raised={raised_panel}, lowered={lowered_panel}",
    )

    return ctx.report()


object_model = build_object_model()
