from __future__ import annotations

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


BASE_RADIUS = 0.17
BASE_DISC_THICKNESS = 0.028
BASE_CAP_RADIUS = 0.092
BASE_CAP_THICKNESS = 0.014
BASE_TOTAL_HEIGHT = BASE_DISC_THICKNESS + BASE_CAP_THICKNESS

UPRIGHT_RADIUS = 0.014
UPRIGHT_HEIGHT = 0.40
SHOULDER_Z = BASE_TOTAL_HEIGHT + UPRIGHT_HEIGHT + 0.020

ARM_LENGTH = 0.78
ARM_WIDTH = 0.018
ARM_THICKNESS = 0.016
ARM_BARREL_RADIUS = 0.014
ARM_BARREL_LENGTH = 0.018

HEAD_LENGTH = 0.300
HEAD_WIDTH = 0.042
HEAD_THICKNESS = 0.016
HEAD_BARREL_RADIUS = 0.008
HEAD_BARREL_LENGTH = 0.014

ARM_LIMITS = (-0.35, 1.05)
HEAD_LIMITS = (-0.55, 0.95)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((a + b) * 0.5 for a, b in zip(lo, hi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_floor_lamp")

    model.material("graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("opal_diffuser", rgba=(0.95, 0.96, 0.94, 0.92))
    model.material("joint_metal", rgba=(0.45, 0.47, 0.50, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_DISC_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_THICKNESS * 0.5)),
        material="graphite",
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=BASE_CAP_RADIUS, length=BASE_CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_THICKNESS + BASE_CAP_THICKNESS * 0.5)),
        material="satin_black",
        name="base_cap",
    )
    base.visual(
        Cylinder(radius=UPRIGHT_RADIUS, length=UPRIGHT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT + UPRIGHT_HEIGHT * 0.5)),
        material="satin_black",
        name="upright",
    )
    base.visual(
        Box((0.028, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT + UPRIGHT_HEIGHT - 0.011)),
        material="satin_black",
        name="shoulder_block",
    )
    for y_pos, name in ((-0.012, "shoulder_ear_0"), (0.012, "shoulder_ear_1")):
        base.visual(
            Box((0.014, 0.006, 0.038)),
            origin=Origin(xyz=(0.0, y_pos, SHOULDER_Z)),
            material="joint_metal",
            name=name,
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=ARM_BARREL_RADIUS, length=ARM_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material="joint_metal",
        name="arm_barrel",
    )
    arm.visual(
        Box((ARM_LENGTH - 0.020, ARM_WIDTH, ARM_THICKNESS)),
        origin=Origin(xyz=(0.5 * (ARM_LENGTH - 0.020), 0.0, 0.0)),
        material="satin_black",
        name="arm_beam",
    )
    arm.visual(
        Box((0.024, 0.018, 0.018)),
        origin=Origin(xyz=(ARM_LENGTH - 0.020, 0.0, 0.0)),
        material="satin_black",
        name="head_mount",
    )
    for y_pos, name in ((-0.009, "head_fork_0"), (0.009, "head_fork_1")):
        arm.visual(
            Box((0.010, 0.004, 0.034)),
            origin=Origin(xyz=(ARM_LENGTH - 0.005, y_pos, 0.0)),
            material="joint_metal",
            name=name,
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=HEAD_BARREL_RADIUS, length=HEAD_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material="joint_metal",
        name="head_barrel",
    )
    head.visual(
        Box((0.024, 0.016, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material="satin_black",
        name="head_neck",
    )
    head.visual(
        Box((HEAD_LENGTH, HEAD_WIDTH, HEAD_THICKNESS)),
        origin=Origin(xyz=(0.024 + HEAD_LENGTH * 0.5, 0.0, 0.0)),
        material="satin_black",
        name="head_body",
    )
    head.visual(
        Box((HEAD_LENGTH - 0.028, HEAD_WIDTH - 0.012, 0.002)),
        origin=Origin(xyz=(0.024 + HEAD_LENGTH * 0.5, 0.0, -0.009)),
        material="opal_diffuser",
        name="head_diffuser",
    )

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z), rpy=(0.0, -0.45, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ARM_LIMITS[0],
            upper=ARM_LIMITS[1],
            effort=20.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "head_hinge",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.30, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=HEAD_LIMITS[0],
            upper=HEAD_LIMITS[1],
            effort=8.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    arm_hinge = object_model.get_articulation("arm_hinge")
    head_hinge = object_model.get_articulation("head_hinge")

    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="arm_beam",
        negative_elem="base_cap",
        min_gap=0.38,
        name="arm beam sits well above the heavy base",
    )

    ctx.expect_overlap(
        arm,
        head,
        axes="yz",
        elem_a="head_mount",
        elem_b="head_barrel",
        min_overlap=0.012,
        name="head hinge stays centered at the arm tip",
    )

    rest_arm_box = ctx.part_element_world_aabb(arm, elem="arm_beam")
    with ctx.pose({arm_hinge: ARM_LIMITS[1]}):
        raised_arm_box = ctx.part_element_world_aabb(arm, elem="arm_beam")

    rest_arm_center = _aabb_center(rest_arm_box)
    raised_arm_center = _aabb_center(raised_arm_box)
    ctx.check(
        "bar arm pitches upward from the upright hinge",
        rest_arm_center is not None
        and raised_arm_center is not None
        and raised_arm_center[2] > rest_arm_center[2] + 0.20
        and raised_arm_center[0] < rest_arm_center[0] - 0.08,
        details=f"rest={rest_arm_center}, raised={raised_arm_center}",
    )

    rest_head_box = ctx.part_element_world_aabb(head, elem="head_body")
    with ctx.pose({head_hinge: HEAD_LIMITS[1]}):
        tilted_head_box = ctx.part_element_world_aabb(head, elem="head_body")

    rest_head_center = _aabb_center(rest_head_box)
    tilted_head_center = _aabb_center(tilted_head_box)
    ctx.check(
        "led head tilts downward at the bar tip",
        rest_head_center is not None
        and tilted_head_center is not None
        and tilted_head_center[2] < rest_head_center[2] - 0.10,
        details=f"rest={rest_head_center}, tilted={tilted_head_center}",
    )

    return ctx.report()


object_model = build_object_model()
