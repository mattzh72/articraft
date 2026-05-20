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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    painted_frame = Material("painted_frame", color=(0.08, 0.16, 0.24, 1.0))
    black_steel = Material("black_steel", color=(0.02, 0.022, 0.024, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.72, 0.72, 0.68, 1.0))
    dark_rubber = Material("dark_rubber", color=(0.015, 0.014, 0.012, 1.0))

    frame = model.part("fixed_frame")
    frame.visual(
        Cylinder(radius=1.26, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=black_steel,
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=0.080, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        material=painted_frame,
        name="lower_column",
    )
    frame.visual(
        Cylinder(radius=0.035, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 1.14)),
        material=brushed_steel,
        name="center_spindle",
    )

    guard_radius = 1.14
    guard_ring = mesh_from_geometry(
        TorusGeometry(radius=guard_radius, tube=0.026, radial_segments=24, tubular_segments=96),
        "guard_ring",
    )
    frame.visual(
        guard_ring,
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
        material=painted_frame,
        name="guard_ring",
    )
    frame.visual(
        Cylinder(radius=0.030, length=1.53),
        origin=Origin(xyz=(0.0, -guard_radius, 0.765)),
        material=painted_frame,
        name="rear_upright",
    )

    # Four outer posts carry the guard hoop and make the fixed frame read as a
    # welded safety cage rather than a floating ring.
    for i, angle in enumerate((math.radians(25), math.radians(155), math.radians(205), math.radians(335))):
        x = guard_radius * math.cos(angle)
        y = guard_radius * math.sin(angle)
        frame.visual(
            Cylinder(radius=0.035, length=1.54),
            origin=Origin(xyz=(x, y, 0.77)),
            material=painted_frame,
            name=f"guard_post_{i}",
        )
        # Small foot pads visibly anchor each post to the floor plate.
        frame.visual(
            Cylinder(radius=0.075, length=0.025),
            origin=Origin(xyz=(x, y, 0.0675)),
            material=black_steel,
            name=f"post_foot_{i}",
        )

    hub_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.118, -0.17), (0.122, -0.145), (0.122, 0.145), (0.118, 0.17)],
            [(0.064, -0.17), (0.064, 0.17)],
            segments=64,
            start_cap="round",
            end_cap="round",
            lip_samples=6,
        ),
        "hub_shell",
    )

    rotor = model.part("rotor_hub")
    rotor.visual(
        hub_shell,
        origin=Origin(),
        material=brushed_steel,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
        material=brushed_steel,
        name="lower_bearing_flange",
    )
    rotor.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=brushed_steel,
        name="upper_bearing_flange",
    )

    arm_length = 0.86
    arm_start = 0.095
    arm_center_radius = arm_start + arm_length / 2.0
    for i in range(3):
        angle = i * 2.0 * math.pi / 3.0
        x = arm_center_radius * math.cos(angle)
        y = arm_center_radius * math.sin(angle)
        end_x = (arm_start + arm_length) * math.cos(angle)
        end_y = (arm_start + arm_length) * math.sin(angle)
        rotor.visual(
            Cylinder(radius=0.033, length=arm_length),
            origin=Origin(xyz=(x, y, 0.0), rpy=(0.0, math.pi / 2.0, angle)),
            material=brushed_steel,
            name=f"arm_{i}",
        )
        rotor.visual(
            Sphere(radius=0.046),
            origin=Origin(xyz=(end_x, end_y, 0.0)),
            material=dark_rubber,
            name=f"end_cap_{i}",
        )

    model.articulation(
        "hub_axis",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("fixed_frame")
    rotor = object_model.get_part("rotor_hub")
    axis = object_model.get_articulation("hub_axis")

    for flange in ("lower_bearing_flange", "upper_bearing_flange"):
        ctx.allow_overlap(
            frame,
            rotor,
            elem_a="center_spindle",
            elem_b=flange,
            reason="The fixed center spindle is intentionally captured inside the rotating bearing flange.",
        )
        ctx.expect_within(
            frame,
            rotor,
            axes="xy",
            inner_elem="center_spindle",
            outer_elem=flange,
            margin=0.001,
            name=f"center spindle is coaxial within {flange}",
        )
        ctx.expect_overlap(
            frame,
            rotor,
            axes="z",
            elem_a="center_spindle",
            elem_b=flange,
            min_overlap=0.025,
            name=f"center spindle passes through {flange}",
        )

    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="arm_0",
        outer_elem="guard_ring",
        margin=0.005,
        name="rotating arm fits inside fixed guard ring",
    )
    ctx.expect_overlap(
        frame,
        rotor,
        axes="z",
        elem_a="center_spindle",
        elem_b="hub_shell",
        min_overlap=0.25,
        name="fixed spindle passes through rotating hub height",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="arm_0",
        negative_elem="base_plate",
        min_gap=0.95,
        name="arms are raised above floor plate",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="arm_0")
    with ctx.pose({axis: math.pi / 3.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            inner_elem="arm_0",
            outer_elem="guard_ring",
            margin=0.005,
            name="rotated arm remains inside fixed guard ring",
        )
        moved_aabb = ctx.part_element_world_aabb(rotor, elem="arm_0")

    ctx.check(
        "continuous hub pose rotates arm geometry",
        rest_aabb is not None
        and moved_aabb is not None
        and abs((rest_aabb[0][1] + rest_aabb[1][1]) - (moved_aabb[0][1] + moved_aabb[1][1])) > 0.25,
        details=f"rest_aabb={rest_aabb}, moved_aabb={moved_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
