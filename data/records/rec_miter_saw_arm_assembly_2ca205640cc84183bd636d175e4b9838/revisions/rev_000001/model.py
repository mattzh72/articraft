from __future__ import annotations

from math import atan2, pi, radians, sin, sqrt

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


def _rounded_base_shape() -> cq.Workplane:
    """Low, heavy benchtop casting with softened exposed corners."""
    return (
        cq.Workplane("XY")
        .box(0.82, 0.52, 0.06)
        .edges("|Z")
        .fillet(0.035)
        .edges(">Z")
        .fillet(0.010)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_miter_saw_arm")

    model.material("cast_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    model.material("brushed_aluminum", rgba=(0.68, 0.70, 0.68, 1.0))
    model.material("dark_insert", rgba=(0.05, 0.055, 0.06, 1.0))
    model.material("safety_yellow", rgba=(0.96, 0.72, 0.10, 1.0))
    model.material("pin_steel", rgba=(0.52, 0.54, 0.56, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_base_shape(), "base_slab", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material="cast_dark",
        name="base_slab",
    )
    # Stationary front miter scale tick blocks, kept outside the rotating table.
    for index, angle_deg in enumerate(range(-50, 51, 25)):
        a = radians(angle_deg)
        radius = 0.335
        x = radius * 0.58 * abs(a) / radians(50) + 0.115
        y = radius * sin(a)
        base.visual(
            Box((0.006, 0.040 if angle_deg % 50 == 0 else 0.026, 0.004)),
            origin=Origin(xyz=(x, y, 0.062), rpy=(0.0, 0.0, a)),
            material="black_rubber",
            name=f"scale_tick_{index}",
        )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.255, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="brushed_aluminum",
        name="turntable",
    )
    table.visual(
        Box((0.280, 0.038, 0.006)),
        origin=Origin(xyz=(0.120, 0.0, 0.038)),
        material="dark_insert",
        name="blade_slot",
    )
    table.visual(
        Box((0.035, 0.540, 0.095)),
        origin=Origin(xyz=(-0.095, 0.0, 0.0825)),
        material="cast_dark",
        name="rear_fence",
    )
    table.visual(
        Box((0.145, 0.245, 0.050)),
        origin=Origin(xyz=(-0.240, 0.0, 0.057)),
        material="cast_dark",
        name="yoke_plinth",
    )
    table.visual(
        Box((0.060, 0.035, 0.320)),
        origin=Origin(xyz=(-0.240, -0.135, 0.192)),
        material="cast_dark",
        name="yoke_cheek_0",
    )
    table.visual(
        Cylinder(radius=0.050, length=0.035),
        origin=Origin(xyz=(-0.240, -0.135, 0.350), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="bearing_boss_0",
    )
    table.visual(
        Box((0.060, 0.035, 0.320)),
        origin=Origin(xyz=(-0.240, 0.135, 0.192)),
        material="cast_dark",
        name="yoke_cheek_1",
    )
    table.visual(
        Cylinder(radius=0.050, length=0.035),
        origin=Origin(xyz=(-0.240, 0.135, 0.350), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="bearing_boss_1",
    )

    arm = model.part("swing_arm")
    # The child frame is the transverse rear hinge axis.
    arm.visual(
        Cylinder(radius=0.042, length=0.235),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="pivot_hub",
    )

    guard_center = (0.380, 0.0, 0.420)
    beam_start = (0.012, 0.0, 0.010)
    beam_vec = (guard_center[0] - beam_start[0], 0.0, guard_center[2] - beam_start[2])
    beam_len = sqrt(beam_vec[0] ** 2 + beam_vec[2] ** 2)
    beam_angle = atan2(beam_vec[2], beam_vec[0])
    arm.visual(
        Box((beam_len, 0.070, 0.060)),
        origin=Origin(
            xyz=((guard_center[0] + beam_start[0]) / 2.0, 0.0, (guard_center[2] + beam_start[2]) / 2.0),
            rpy=(0.0, -beam_angle, 0.0),
        ),
        material="safety_yellow",
        name="upper_arm",
    )
    arm.visual(
        Cylinder(radius=0.158, length=0.082),
        origin=Origin(xyz=guard_center, rpy=(pi / 2.0, 0.0, 0.0)),
        material="safety_yellow",
        name="blade_guard",
    )
    arm.visual(
        Cylinder(radius=0.122, length=0.006),
        origin=Origin(xyz=(guard_center[0], 0.044, guard_center[2]), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_insert",
        name="guard_recess",
    )
    arm.visual(
        Box((0.180, 0.050, 0.040)),
        origin=Origin(xyz=(0.315, 0.0, 0.642), rpy=(0.0, -0.18, 0.0)),
        material="black_rubber",
        name="front_grip",
    )
    arm.visual(
        Box((0.055, 0.060, 0.115)),
        origin=Origin(xyz=(0.340, 0.0, 0.570), rpy=(0.0, -0.18, 0.0)),
        material="safety_yellow",
        name="grip_stem",
    )

    model.articulation(
        "table_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=-radians(50.0), upper=radians(50.0)),
    )
    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=table,
        child=arm,
        origin=Origin(xyz=(-0.240, 0.0, 0.350)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=0.0, upper=radians(55.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    table = object_model.get_part("table")
    arm = object_model.get_part("swing_arm")
    yaw = object_model.get_articulation("table_yaw")
    hinge = object_model.get_articulation("arm_hinge")

    ctx.check(
        "table yaw limits are about plus/minus 50 degrees",
        yaw.motion_limits is not None
        and abs(yaw.motion_limits.lower + radians(50.0)) < 1e-6
        and abs(yaw.motion_limits.upper - radians(50.0)) < 1e-6,
    )
    ctx.check(
        "arm hinge lowers through about 55 degrees",
        hinge.motion_limits is not None
        and abs(hinge.motion_limits.lower) < 1e-6
        and abs(hinge.motion_limits.upper - radians(55.0)) < 1e-6,
    )
    ctx.expect_gap(
        table,
        base,
        axis="z",
        positive_elem="turntable",
        negative_elem="base_slab",
        max_gap=0.002,
        max_penetration=0.0,
        name="turntable is seated on the flat base",
    )
    ctx.expect_contact(
        arm,
        table,
        elem_a="pivot_hub",
        elem_b="yoke_cheek_0",
        contact_tol=0.001,
        name="pivot hub bears against one yoke cheek",
    )
    ctx.expect_contact(
        arm,
        table,
        elem_a="pivot_hub",
        elem_b="yoke_cheek_1",
        contact_tol=0.001,
        name="pivot hub bears against the opposite yoke cheek",
    )

    rest_guard = ctx.part_element_world_aabb(arm, elem="blade_guard")
    with ctx.pose({hinge: radians(55.0)}):
        lowered_guard = ctx.part_element_world_aabb(arm, elem="blade_guard")
    rest_center_z = (rest_guard[0][2] + rest_guard[1][2]) / 2.0 if rest_guard else None
    lowered_center_z = (lowered_guard[0][2] + lowered_guard[1][2]) / 2.0 if lowered_guard else None
    ctx.check(
        "front blade guard moves downward at hinge upper limit",
        rest_center_z is not None and lowered_center_z is not None and lowered_center_z < rest_center_z - 0.18,
        details=f"rest_z={rest_center_z}, lowered_z={lowered_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
