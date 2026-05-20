from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_floor_lamp")

    black_metal = Material("satin_black_metal", rgba=(0.02, 0.018, 0.016, 1.0))
    brass = Material("warm_brass", rgba=(0.82, 0.62, 0.32, 1.0))
    shade_fabric = Material("warm_ivory_shade", rgba=(0.94, 0.86, 0.66, 0.92))
    warm_glass = Material("warm_glowing_glass", rgba=(1.0, 0.82, 0.38, 0.70))

    shade_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.068, -0.075), (0.068, 0.075)],
            [(0.056, -0.075), (0.056, 0.075)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
        "small_cylindrical_shade_shell",
    )
    hinge_ring_mesh = mesh_from_geometry(
        TorusGeometry(0.024, 0.006, radial_segments=18, tubular_segments=48),
        "hinge_washer_ring",
    )
    arm_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.030, 0.0, 0.000),
                (0.20, 0.0, 0.070),
                (0.46, 0.0, 0.115),
                (0.67, 0.0, 0.080),
                (0.78, 0.0, 0.040),
            ],
            radius=0.010,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "gently_curved_arm_tube",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.235, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=black_metal,
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=0.185, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=brass,
        name="base_top_trim",
    )
    stand.visual(
        Cylinder(radius=0.024, length=1.50),
        origin=Origin(xyz=(0.0, 0.0, 0.805)),
        material=black_metal,
        name="central_column",
    )
    stand.visual(
        Cylinder(radius=0.033, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 1.57)),
        material=brass,
        name="top_cap",
    )

    joint_radius = 0.085
    column_radius = 0.024
    bracket_length = joint_radius - column_radius + 0.016
    bracket_center_radius = column_radius + bracket_length / 2.0 - 0.006
    joint_heights = (1.02, 1.20, 1.38)
    joint_yaws = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)

    for i, (height, yaw) in enumerate(zip(joint_heights, joint_yaws)):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        joint_xyz = (joint_radius * cos_yaw, joint_radius * sin_yaw, height)
        bracket_xyz = (
            bracket_center_radius * cos_yaw,
            bracket_center_radius * sin_yaw,
            height,
        )

        stand.visual(
            Cylinder(radius=0.038, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, height), rpy=(0.0, 0.0, yaw)),
            material=brass,
            name=f"collar_{i}",
        )
        for suffix, z_offset in (("lower", -0.024), ("upper", 0.024)):
            stand.visual(
                Box((bracket_length, 0.030, 0.006)),
                origin=Origin(
                    xyz=(bracket_xyz[0], bracket_xyz[1], height + z_offset),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=black_metal,
                name=f"fork_{i}_{suffix}",
            )
        stand.visual(
            Cylinder(radius=0.0185, length=0.086),
            origin=Origin(xyz=joint_xyz),
            material=brass,
            name=f"pin_{i}",
        )

        arm = model.part(f"arm_{i}")
        arm.visual(
            hinge_ring_mesh,
            origin=Origin(),
            material=brass,
            name="hinge_ring",
        )
        arm.visual(
            arm_tube_mesh,
            origin=Origin(),
            material=black_metal,
            name="curved_arm",
        )
        arm.visual(
            Cylinder(radius=0.018, length=0.070),
            origin=Origin(xyz=(0.785, 0.0, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="lamp_socket",
        )
        arm.visual(
            Cylinder(radius=0.010, length=0.075),
            origin=Origin(xyz=(0.850, 0.0, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="bulb_stem",
        )
        arm.visual(
            Box((0.012, 0.122, 0.006)),
            origin=Origin(xyz=(0.785, 0.0, 0.040)),
            material=brass,
            name="shade_spider_y",
        )
        arm.visual(
            Box((0.012, 0.006, 0.122)),
            origin=Origin(xyz=(0.785, 0.0, 0.040)),
            material=brass,
            name="shade_spider_z",
        )
        arm.visual(
            shade_mesh,
            origin=Origin(xyz=(0.860, 0.0, 0.040)),
            material=shade_fabric,
            name="shade_shell",
        )
        arm.visual(
            Sphere(radius=0.024),
            origin=Origin(xyz=(0.900, 0.0, 0.040)),
            material=warm_glass,
            name="warm_bulb",
        )

        model.articulation(
            f"stand_to_arm_{i}",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=arm,
            origin=Origin(xyz=joint_xyz, rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-1.05, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    arms = [object_model.get_part(f"arm_{i}") for i in range(3)]
    joints = [object_model.get_articulation(f"stand_to_arm_{i}") for i in range(3)]

    ctx.check(
        "three separate articulated lamp arms",
        len(arms) == 3 and len(joints) == 3,
        details=f"arms={len(arms)}, joints={len(joints)}",
    )

    for i, (arm, joint) in enumerate(zip(arms, joints)):
        ctx.allow_overlap(
            stand,
            arm,
            elem_a=f"pin_{i}",
            elem_b="hinge_ring",
            reason="The brass hinge pin is intentionally captured inside the washer-like arm hinge ring.",
        )
        ctx.expect_within(
            stand,
            arm,
            axes="xy",
            inner_elem=f"pin_{i}",
            outer_elem="hinge_ring",
            margin=0.006,
            name=f"pin_{i} passes through hinge ring",
        )
        ctx.expect_overlap(
            arm,
            stand,
            axes="z",
            elem_a="hinge_ring",
            elem_b=f"pin_{i}",
            min_overlap=0.010,
            name=f"pin_{i} and hinge ring share height",
        )

        rest_aabb = ctx.part_element_world_aabb(arm, elem="shade_shell")
        with ctx.pose({joint: 0.70}):
            moved_aabb = ctx.part_element_world_aabb(arm, elem="shade_shell")

        if rest_aabb is not None and moved_aabb is not None:
            rest_center = tuple((rest_aabb[0][axis] + rest_aabb[1][axis]) / 2.0 for axis in range(3))
            moved_center = tuple((moved_aabb[0][axis] + moved_aabb[1][axis]) / 2.0 for axis in range(3))
            lateral_motion = math.hypot(moved_center[0] - rest_center[0], moved_center[1] - rest_center[1])
            vertical_drift = abs(moved_center[2] - rest_center[2])
        else:
            lateral_motion = 0.0
            vertical_drift = 99.0
        ctx.check(
            f"arm_{i} shade swings on vertical revolute joint",
            lateral_motion > 0.25 and vertical_drift < 0.015,
            details=f"lateral_motion={lateral_motion:.3f}, vertical_drift={vertical_drift:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
