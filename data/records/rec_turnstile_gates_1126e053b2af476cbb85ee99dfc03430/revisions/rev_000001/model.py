from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_turnstile_gate")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = model.material("graphite", rgba=(0.06, 0.065, 0.07, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_bearing = model.material("dark_bearing", rgba=(0.11, 0.12, 0.13, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.04, 1.0))
    soft_rubber = model.material("soft_rubber", rgba=(0.02, 0.022, 0.024, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.58, 0.58, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=matte_black,
        name="base_plate",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.260, 0.004, radial_segments=12, tubular_segments=72), "sweep_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=safety_yellow,
        name="sweep_ring",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=graphite,
        name="central_post",
    )
    frame.visual(
        Cylinder(radius=0.058, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=dark_bearing,
        name="lower_bearing",
    )
    frame.visual(
        Cylinder(radius=0.052, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.3835)),
        material=dark_bearing,
        name="upper_bearing",
    )
    for x in (-0.070, 0.070):
        frame.visual(
            Cylinder(radius=0.010, length=0.390),
            origin=Origin(xyz=(x, -0.285, 0.205)),
            material=graphite,
            name=f"rear_post_{0 if x < 0 else 1}",
        )
    frame.visual(
        Box((0.170, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.285, 0.392)),
        material=graphite,
        name="rear_crossbar",
    )
    frame.visual(
        Box((0.018, 0.285, 0.018)),
        origin=Origin(xyz=(0.0, -0.1425, 0.392)),
        material=graphite,
        name="upper_bridge",
    )
    for side, x in enumerate((-0.275, 0.275)):
        frame.visual(
            Cylinder(radius=0.006, length=0.340),
            origin=Origin(xyz=(x, 0.0, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=f"guide_rail_{side}",
        )
        for end, y in enumerate((-0.170, 0.170)):
            frame.visual(
                Cylinder(radius=0.006, length=0.160),
                origin=Origin(xyz=(x, y, 0.090)),
                material=graphite,
                name=f"guide_post_{side}_{end}",
            )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.018, length=0.1435),
        origin=Origin(xyz=(0.0, 0.0, 0.07175)),
        material=brushed_steel,
        name="spindle",
    )
    rotor.visual(
        Cylinder(radius=0.040, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=brushed_steel,
        name="bearing_core",
    )
    rotor.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_bearing,
        name="lower_hub_flange",
    )
    rotor.visual(
        Cylinder(radius=0.048, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=dark_bearing,
        name="upper_hub_flange",
    )

    hinge_radius = 0.069
    arm_z = 0.067
    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        c = math.cos(yaw)
        s = math.sin(yaw)
        rotor.visual(
            Box((0.024, 0.045, 0.018)),
            origin=Origin(xyz=(0.0465 * c, 0.0465 * s, arm_z), rpy=(0.0, 0.0, yaw)),
            material=brushed_steel,
            name=f"fold_lug_{index}",
        )
        for side, offset in enumerate((-0.0185, 0.0185)):
            rotor.visual(
                Box((0.024, 0.006, 0.030)),
                origin=Origin(
                    xyz=(hinge_radius * c - offset * s, hinge_radius * s + offset * c, arm_z),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=brushed_steel,
                name=f"fold_cheek_{index}_{side}",
            )
        rotor.visual(
            Cylinder(radius=0.0035, length=0.045),
            origin=Origin(
                xyz=(hinge_radius * c, hinge_radius * s, arm_z),
                rpy=(math.pi / 2.0, 0.0, yaw),
            ),
            material=dark_bearing,
            name=f"fold_pin_{index}",
        )

    model.articulation(
        "central_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.2225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5),
        motion_properties=MotionProperties(damping=0.15, friction=0.03),
    )

    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        arm = model.part(f"arm_{index}")
        arm.visual(
            Cylinder(radius=0.009, length=0.024),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_bearing,
            name="hinge_barrel",
        )
        arm.visual(
            Cylinder(radius=0.008, length=0.155),
            origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name="arm_tube",
        )
        arm.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.170, 0.0, 0.0)),
            material=safety_yellow,
            name="safe_tip",
        )
        arm.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.159, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=soft_rubber,
            name="rubber_sleeve",
        )
        model.articulation(
            f"arm_fold_{index}",
            ArticulationType.REVOLUTE,
            parent=rotor,
            child=arm,
            origin=Origin(
                xyz=(hinge_radius * math.cos(yaw), hinge_radius * math.sin(yaw), arm_z),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=1.2, lower=0.0, upper=1.35),
            motion_properties=MotionProperties(damping=0.08, friction=0.02),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    central_rotation = object_model.get_articulation("central_rotation")

    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="spindle",
        negative_elem="lower_bearing",
        min_gap=0.0,
        max_gap=0.003,
        name="spindle clears lower bearing",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="upper_bearing",
        negative_elem="spindle",
        min_gap=0.0,
        max_gap=0.003,
        name="spindle clears upper bearing",
    )

    for index in range(4):
        arm = object_model.get_part(f"arm_{index}")
        ctx.allow_overlap(
            rotor,
            arm,
            elem_a=f"fold_pin_{index}",
            elem_b="hinge_barrel",
            reason="The hinge pin is intentionally captured inside the fold barrel bore proxy.",
        )
        ctx.expect_overlap(
            rotor,
            arm,
            axes="xyz",
            elem_a=f"fold_pin_{index}",
            elem_b="hinge_barrel",
            min_overlap=0.003,
            name=f"arm {index} hinge pin is captured",
        )

    arm_0 = object_model.get_part("arm_0")
    rest_aabb = ctx.part_world_aabb(arm_0)
    ctx.check(
        "deployed arm stays inside base",
        rest_aabb is not None and rest_aabb[1][0] < 0.270,
        details=f"arm_0_aabb={rest_aabb}",
    )
    with ctx.pose({central_rotation: math.pi / 2.0}):
        rotated_aabb = ctx.part_world_aabb(arm_0)
    ctx.check(
        "central rotor sweeps arm",
        rest_aabb is not None
        and rotated_aabb is not None
        and rotated_aabb[1][1] > rest_aabb[1][1] + 0.12,
        details=f"rest={rest_aabb}, rotated={rotated_aabb}",
    )

    fold_0 = object_model.get_articulation("arm_fold_0")
    with ctx.pose({fold_0: 1.35}):
        folded_aabb = ctx.part_world_aabb(arm_0)
    ctx.check(
        "arm folds upward for stowage",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] > rest_aabb[1][2] + 0.12
        and folded_aabb[1][0] < rest_aabb[1][0] - 0.06,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
