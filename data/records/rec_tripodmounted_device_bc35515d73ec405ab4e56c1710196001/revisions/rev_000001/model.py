from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_with_pan_tilt_head")

    anodized_black = model.material("anodized_black", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.65, 0.68, 1.0))
    camera_black = model.material("camera_black", rgba=(0.11, 0.11, 0.12, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.065, length=0.06),
        material=dark_graphite,
        name="crown_core",
    )
    crown.visual(
        Box((0.18, 0.18, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=anodized_black,
        name="top_deck",
    )
    crown.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=dark_graphite,
        name="pan_riser",
    )

    for index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        crown.visual(
            Box((0.09, 0.075, 0.02)),
            origin=Origin(xyz=(0.095 * math.cos(yaw), 0.095 * math.sin(yaw), -0.005), rpy=(0.0, 0.0, yaw)),
            material=dark_graphite,
            name=f"leg_web_{index}",
        )
        crown.visual(
            Box((0.08, 0.064, 0.03)),
            origin=Origin(xyz=(0.135 * math.cos(yaw), 0.135 * math.sin(yaw), -0.015), rpy=(0.0, 0.0, yaw)),
            material=anodized_black,
            name=f"leg_mount_{index}",
        )

    crown.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.12)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.05, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_metal,
        name="pan_disk",
    )
    head.visual(
        Box((0.07, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_graphite,
        name="tilt_pedestal",
    )
    head.visual(
        Box((0.03, 0.12, 0.038)),
        origin=Origin(xyz=(-0.03, 0.0, 0.091)),
        material=dark_graphite,
        name="rear_bridge",
    )
    head.visual(
        Box((0.07, 0.016, 0.07)),
        origin=Origin(xyz=(0.005, 0.046, 0.095)),
        material=anodized_black,
        name="left_cheek",
    )
    head.visual(
        Box((0.07, 0.016, 0.07)),
        origin=Origin(xyz=(0.005, -0.046, 0.095)),
        material=anodized_black,
        name="right_cheek",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.14, 0.14, 0.14)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    device_assembly = model.part("device_assembly")
    device_assembly.visual(
        Cylinder(radius=0.01, length=0.072),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="tilt_barrel",
    )
    device_assembly.visual(
        Box((0.14, 0.075, 0.01)),
        origin=Origin(xyz=(0.068, 0.0, 0.002)),
        material=satin_metal,
        name="device_plate",
    )
    device_assembly.visual(
        Box((0.065, 0.082, 0.014)),
        origin=Origin(xyz=(0.045, 0.0, 0.014)),
        material=dark_graphite,
        name="plate_clamp",
    )
    device_assembly.visual(
        Box((0.11, 0.06, 0.055)),
        origin=Origin(xyz=(0.078, 0.0, 0.045)),
        material=camera_black,
        name="device_body",
    )
    device_assembly.visual(
        Box((0.04, 0.04, 0.02)),
        origin=Origin(xyz=(0.052, 0.0, 0.082)),
        material=camera_black,
        name="viewfinder_hump",
    )
    device_assembly.visual(
        Cylinder(radius=0.022, length=0.07),
        origin=Origin(xyz=(0.145, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="lens",
    )
    device_assembly.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.183, 0.0, 0.045)),
        material=anodized_black,
        name="lens_front",
    )
    device_assembly.inertial = Inertial.from_geometry(
        Box((0.22, 0.10, 0.10)),
        mass=1.2,
        origin=Origin(xyz=(0.09, 0.0, 0.04)),
    )

    model.articulation(
        "crown_to_head_pan",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )

    model.articulation(
        "head_to_device_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=device_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=math.radians(-35.0),
            upper=math.radians(75.0),
        ),
    )

    leg_length = 0.60
    leg_width = 0.055
    leg_thickness = 0.03
    leg_deploy_angle = math.radians(50.0)
    leg_start_x = 0.04
    leg_start_z = -0.035
    leg_axis_dx = math.cos(leg_deploy_angle)
    leg_axis_dz = -math.sin(leg_deploy_angle)
    beam_center = (
        leg_start_x + 0.5 * leg_length * leg_axis_dx,
        0.0,
        leg_start_z + 0.5 * leg_length * leg_axis_dz,
    )
    foot_center = (
        leg_start_x + leg_length * leg_axis_dx - 0.012 * leg_axis_dx,
        0.0,
        leg_start_z + leg_length * leg_axis_dz + 0.012 * (-leg_axis_dz) - 0.01,
    )

    for index, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Box((0.08, 0.056, 0.028)),
            origin=Origin(xyz=(0.04, 0.0, -0.029)),
            material=dark_graphite,
            name="upper_shoe",
        )
        leg.visual(
            Box((leg_length, leg_width, leg_thickness)),
            origin=Origin(xyz=beam_center, rpy=(0.0, leg_deploy_angle, 0.0)),
            material=anodized_black,
            name="leg_beam",
        )
        leg.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=foot_center),
            material=rubber_black,
            name="foot",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.66, 0.08, 0.08)),
            mass=0.7,
            origin=Origin(xyz=(0.22, 0.0, -0.24)),
        )
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(0.115 * math.cos(yaw), 0.115 * math.sin(yaw), -0.015), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=1.5,
                lower=0.0,
                upper=math.radians(52.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    head = object_model.get_part("head")
    device_assembly = object_model.get_part("device_assembly")
    leg_0 = object_model.get_part("leg_0")
    pan = object_model.get_articulation("crown_to_head_pan")
    tilt = object_model.get_articulation("head_to_device_tilt")
    leg_fold = object_model.get_articulation("crown_to_leg_0")

    ctx.expect_gap(
        head,
        crown,
        axis="z",
        positive_elem="pan_disk",
        negative_elem="pan_riser",
        min_gap=0.0,
        max_gap=0.001,
        name="pan head seats on crown riser",
    )
    ctx.expect_overlap(
        device_assembly,
        head,
        axes="y",
        min_overlap=0.06,
        elem_a="tilt_barrel",
        elem_b="rear_bridge",
        name="tilt barrel remains captured between the head cheeks",
    )

    lens_rest = ctx.part_element_world_aabb(device_assembly, elem="lens_front")
    with ctx.pose({pan: math.pi / 2.0}):
        lens_turned = ctx.part_element_world_aabb(device_assembly, elem="lens_front")

    if lens_rest is not None and lens_turned is not None:
        rest_center = (
            0.5 * (lens_rest[0][0] + lens_rest[1][0]),
            0.5 * (lens_rest[0][1] + lens_rest[1][1]),
            0.5 * (lens_rest[0][2] + lens_rest[1][2]),
        )
        turned_center = (
            0.5 * (lens_turned[0][0] + lens_turned[1][0]),
            0.5 * (lens_turned[0][1] + lens_turned[1][1]),
            0.5 * (lens_turned[0][2] + lens_turned[1][2]),
        )
        ctx.check(
            "pan rotates device around vertical axis",
            abs(turned_center[1]) > abs(rest_center[1]) + 0.15 and abs(turned_center[0]) < 0.05,
            details=f"rest={rest_center}, turned={turned_center}",
        )
    else:
        ctx.fail("pan rotates device around vertical axis", "lens_front AABB unavailable")

    with ctx.pose({tilt: 0.0}):
        lens_level = ctx.part_element_world_aabb(device_assembly, elem="lens_front")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        lens_tilted = ctx.part_element_world_aabb(device_assembly, elem="lens_front")

    if lens_level is not None and lens_tilted is not None:
        level_center_z = 0.5 * (lens_level[0][2] + lens_level[1][2])
        tilted_center_z = 0.5 * (lens_tilted[0][2] + lens_tilted[1][2])
        ctx.check(
            "device tilts upward about horizontal axis",
            tilted_center_z > level_center_z + 0.12,
            details=f"level_z={level_center_z}, tilted_z={tilted_center_z}",
        )
    else:
        ctx.fail("device tilts upward about horizontal axis", "lens_front AABB unavailable")

    foot_rest = ctx.part_element_world_aabb(leg_0, elem="foot")
    with ctx.pose({leg_fold: leg_fold.motion_limits.upper}):
        foot_folded = ctx.part_element_world_aabb(leg_0, elem="foot")

    if foot_rest is not None and foot_folded is not None:
        rest_center_z = 0.5 * (foot_rest[0][2] + foot_rest[1][2])
        folded_center_z = 0.5 * (foot_folded[0][2] + foot_folded[1][2])
        ctx.check(
            "tripod leg folds upward toward the crown",
            folded_center_z > rest_center_z + 0.32,
            details=f"rest_z={rest_center_z}, folded_z={folded_center_z}",
        )
    else:
        ctx.fail("tripod leg folds upward toward the crown", "foot AABB unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
