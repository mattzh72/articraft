from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

BASE_RADIUS = 0.210
BASE_THICKNESS = 0.055
BASE_HINGE_Z = 0.505
ARM_TUBE_RADIUS = 0.017
ARM_TIP_HINGE = (1.211, 0.0, 0.900)
ARM_LIMITS = (-0.35, 0.45)
SHADE_LIMITS = (-0.55, 0.75)


def _arm_tube():
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.035, 0.0, 0.024),
                (0.110, 0.0, 0.250),
                (0.285, 0.0, 0.720),
                (0.595, 0.0, 1.115),
                (0.930, 0.0, 1.280),
                (1.110, 0.0, 1.135),
                (1.176, 0.0, 0.900),
            ],
            radius=ARM_TUBE_RADIUS,
            samples_per_segment=24,
            radial_segments=22,
            cap_ends=True,
        ),
        "arm_tube",
    )


def _shade_neck():
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.0, 0.000),
                (0.008, 0.0, -0.006),
                (0.012, 0.0, -0.014),
                (0.010, 0.0, -0.024),
            ],
            radius=0.013,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
        "shade_neck",
    )


def _bowl_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.102, 0.000),
                (0.095, -0.030),
                (0.074, -0.078),
                (0.040, -0.121),
                (0.014, -0.143),
            ],
            [
                (0.094, -0.004),
                (0.086, -0.030),
                (0.066, -0.074),
                (0.034, -0.112),
                (0.010, -0.133),
            ],
            segments=72,
            start_cap="flat",
            end_cap="round",
            lip_samples=8,
        ),
        "bowl_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arc_floor_lamp")

    model.material("base_stone", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("brass", rgba=(0.71, 0.63, 0.39, 1.0))
    model.material("shade_cream", rgba=(0.93, 0.92, 0.87, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="base_stone",
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.138, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="base_stone",
        name="base_cap",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.440),
        origin=Origin(xyz=(-0.055, 0.0, 0.280)),
        material="brass",
        name="stem",
    )
    base.visual(
        Box((0.045, 0.006, 0.082)),
        origin=Origin(xyz=(-0.020, 0.018, 0.515)),
        material="brass",
        name="yoke_0",
    )
    base.visual(
        Box((0.045, 0.006, 0.082)),
        origin=Origin(xyz=(-0.020, -0.018, 0.515)),
        material="brass",
        name="yoke_1",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(rpy=(1.5707963267948966, 0.0, 0.0)),
        material="brass",
        name="arm_hub",
    )
    arm.visual(
        Box((0.055, 0.020, 0.028)),
        origin=Origin(xyz=(0.028, 0.0, 0.016)),
        material="brass",
        name="shoulder_link",
    )
    arm.visual(_arm_tube(), material="brass", name="arm_tube")
    arm.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(1.176, 0.0, 0.900), rpy=(0.0, 1.5707963267948966, 0.0)),
        material="brass",
        name="tip_collar",
    )
    arm.visual(
        Box((0.036, 0.006, 0.050)),
        origin=Origin(xyz=(1.193, 0.016, 0.900)),
        material="brass",
        name="tip_plate_0",
    )
    arm.visual(
        Box((0.036, 0.006, 0.050)),
        origin=Origin(xyz=(1.193, -0.016, 0.900)),
        material="brass",
        name="tip_plate_1",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(rpy=(1.5707963267948966, 0.0, 0.0)),
        material="brass",
        name="shade_hub",
    )
    shade.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(xyz=(0.026, 0.0, -0.024), rpy=(0.0, 1.5707963267948966, 0.0)),
        material="brass",
        name="socket",
    )
    shade.visual(_shade_neck(), material="brass", name="neck")
    shade.visual(
        _bowl_shell(),
        origin=Origin(xyz=(0.110, 0.0, -0.024)),
        material="shade_cream",
        name="bowl_shell",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ARM_LIMITS[0],
            upper=ARM_LIMITS[1],
            effort=40.0,
            velocity=1.0,
        ),
    )
    model.articulation(
        "arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=ARM_TIP_HINGE),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=SHADE_LIMITS[0],
            upper=SHADE_LIMITS[1],
            effort=12.0,
            velocity=1.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    shade = object_model.get_part("shade")
    arm_joint = object_model.get_articulation("base_to_arm")
    shade_joint = object_model.get_articulation("arm_to_shade")

    ctx.expect_origin_gap(
        arm,
        base,
        axis="z",
        min_gap=0.49,
        max_gap=0.52,
        name="arm hinge sits at floor lamp height",
    )
    ctx.expect_contact(
        arm,
        base,
        elem_a="arm_hub",
        elem_b="yoke_0",
        contact_tol=0.0015,
        name="arm hub nests at the base yoke",
    )
    ctx.expect_contact(
        shade,
        arm,
        elem_a="shade_hub",
        elem_b="tip_plate_0",
        contact_tol=0.0015,
        name="shade hub nests at the tip bracket",
    )
    ctx.expect_gap(
        shade,
        base,
        axis="z",
        min_gap=0.65,
        name="shade hangs well above the weighted base",
    )

    rest_tip = ctx.part_world_position(shade)
    with ctx.pose({arm_joint: ARM_LIMITS[1]}):
        raised_tip = ctx.part_world_position(shade)
    ctx.check(
        "arm rotates upward from the base hinge",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[2] > rest_tip[2] + 0.22
        and raised_tip[0] < rest_tip[0] - 0.10,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    rest_bowl = ctx.part_element_world_aabb(shade, elem="bowl_shell")
    with ctx.pose({shade_joint: SHADE_LIMITS[1]}):
        tilted_bowl = ctx.part_element_world_aabb(shade, elem="bowl_shell")
    ctx.check(
        "shade can tilt downward at the tip bracket",
        rest_bowl is not None
        and tilted_bowl is not None
        and tilted_bowl[0][2] < rest_bowl[0][2] - 0.02,
        details=f"rest_bowl={rest_bowl}, tilted_bowl={tilted_bowl}",
    )

    return ctx.report()


object_model = build_object_model()
