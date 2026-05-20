from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_RADIUS = 0.145
BASE_THICKNESS = 0.035
POST_RADIUS = 0.014
POST_LENGTH = 1.48
ARM_COLLAR_Z = 0.920
ARM_PIVOT_X = 0.051
ARM_LENGTH = 0.417
TOP_SOCKET_Z = BASE_THICKNESS + POST_LENGTH + 0.030
TOP_STEM_Z = TOP_SOCKET_Z + 0.060
BOWL_BOTTOM_Z = TOP_STEM_Z + 0.015


def _shade_shell_mesh(name: str, outer_profile, inner_profile):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="torchiere_floor_lamp")

    model.material("bronze", rgba=(0.23, 0.19, 0.16, 1.0))
    model.material("gunmetal", rgba=(0.33, 0.34, 0.36, 1.0))
    model.material("opal_glass", rgba=(0.93, 0.91, 0.84, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="bronze",
        name="base_disk",
    )
    stand.visual(
        Cylinder(radius=0.052, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.016)),
        material="bronze",
        name="lower_hub",
    )
    stand.visual(
        Cylinder(radius=POST_RADIUS, length=POST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_LENGTH / 2.0)),
        material="bronze",
        name="post",
    )
    stand.visual(
        Cylinder(radius=0.021, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, TOP_SOCKET_Z)),
        material="bronze",
        name="top_socket",
    )
    stand.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, TOP_STEM_Z)),
        material="bronze",
        name="top_stem",
    )
    stand.visual(
        _shade_shell_mesh(
            "torchiere_bowl_shade",
            outer_profile=[
                (0.034, 0.0),
                (0.092, 0.028),
                (0.148, 0.074),
                (0.190, 0.118),
            ],
            inner_profile=[
                (0.0, 0.0),
                (0.072, 0.028),
                (0.128, 0.072),
                (0.176, 0.110),
            ],
        ),
        origin=Origin(xyz=(0.0, 0.0, BOWL_BOTTOM_Z)),
        material="opal_glass",
        name="bowl_shade",
    )
    stand.visual(
        Cylinder(radius=0.026, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, ARM_COLLAR_Z)),
        material="bronze",
        name="arm_collar",
    )
    stand.visual(
        Box((0.022, 0.030, 0.060)),
        origin=Origin(xyz=(0.025, 0.0, ARM_COLLAR_Z)),
        material="bronze",
        name="arm_mount",
    )
    stand.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(
            xyz=(-0.036, 0.0, ARM_COLLAR_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="gunmetal",
        name="clamp_stem",
    )
    stand.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(-0.052, 0.0, ARM_COLLAR_Z)),
        material="gunmetal",
        name="clamp_knob",
    )

    reading_arm = model.part("reading_arm")
    reading_arm.visual(
        Cylinder(radius=0.015, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="gunmetal",
        name="hinge_barrel",
    )
    reading_arm.visual(
        Cylinder(radius=0.010, length=0.380),
        origin=Origin(xyz=(0.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="bronze",
        name="arm_tube",
    )
    reading_arm.visual(
        Box((0.024, 0.034, 0.042)),
        origin=Origin(xyz=(0.392, 0.0, 0.0)),
        material="gunmetal",
        name="head_mount",
    )

    reading_head = model.part("reading_head")
    reading_head.visual(
        Cylinder(radius=0.013, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="gunmetal",
        name="tilt_barrel",
    )
    reading_head.visual(
        Cylinder(radius=0.007, length=0.052),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="bronze",
        name="shade_neck",
    )
    reading_head.visual(
        _shade_shell_mesh(
            "reading_lamp_shade",
            outer_profile=[
                (0.012, 0.0),
                (0.034, 0.018),
                (0.052, 0.046),
                (0.066, 0.072),
            ],
            inner_profile=[
                (0.0, 0.0),
                (0.024, 0.017),
                (0.042, 0.044),
                (0.058, 0.066),
            ],
        ),
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, 2.15, 0.0)),
        material="opal_glass",
        name="reading_shade",
    )

    model.articulation(
        "stand_to_reading_arm",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=reading_arm,
        origin=Origin(xyz=(ARM_PIVOT_X, 0.0, ARM_COLLAR_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.65,
            upper=1.05,
        ),
    )
    model.articulation(
        "reading_arm_to_reading_head",
        ArticulationType.REVOLUTE,
        parent=reading_arm,
        child=reading_head,
        origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=-0.85,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    reading_arm = object_model.get_part("reading_arm")
    reading_head = object_model.get_part("reading_head")
    arm_hinge = object_model.get_articulation("stand_to_reading_arm")
    head_hinge = object_model.get_articulation("reading_arm_to_reading_head")

    arm_limits = arm_hinge.motion_limits
    head_limits = head_hinge.motion_limits
    arm_upper = 1.05 if arm_limits is None or arm_limits.upper is None else arm_limits.upper
    head_lower = -0.85 if head_limits is None or head_limits.lower is None else head_limits.lower
    head_upper = 0.55 if head_limits is None or head_limits.upper is None else head_limits.upper

    with ctx.pose({arm_hinge: 0.0, head_hinge: 0.0}):
        ctx.expect_gap(
            reading_arm,
            stand,
            axis="x",
            positive_elem="hinge_barrel",
            negative_elem="arm_mount",
            max_gap=0.001,
            max_penetration=1e-5,
            name="reading arm hinge barrel seats against stand mount",
        )

    rest_head_pos = ctx.part_world_position(reading_head)
    with ctx.pose({arm_hinge: arm_upper * 0.8, head_hinge: 0.0}):
        raised_head_pos = ctx.part_world_position(reading_head)
        ctx.expect_gap(
            reading_head,
            stand,
            axis="x",
            min_gap=0.03,
            name="raised reading head stays outward of the lamp column",
        )
    ctx.check(
        "reading arm lifts the head when opened upward",
        rest_head_pos is not None
        and raised_head_pos is not None
        and raised_head_pos[2] > rest_head_pos[2] + 0.12,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    with ctx.pose({arm_hinge: 0.35, head_hinge: head_lower}):
        shade_low = _aabb_center(
            ctx.part_element_world_aabb(reading_head, elem="reading_shade")
        )
    with ctx.pose({arm_hinge: 0.35, head_hinge: head_upper}):
        shade_high = _aabb_center(
            ctx.part_element_world_aabb(reading_head, elem="reading_shade")
        )
    ctx.check(
        "reading shade tilts upward with positive hinge motion",
        shade_low is not None
        and shade_high is not None
        and shade_high[2] > shade_low[2] + 0.02,
        details=f"low={shade_low}, high={shade_high}",
    )

    return ctx.report()


object_model = build_object_model()
