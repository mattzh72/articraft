from __future__ import annotations

from math import pi, sin

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
)


CANOPY_RADIUS = 0.110
CANOPY_THICKNESS = 0.025
STEM_LENGTH = 0.036
HUB_RADIUS = 0.031
HUB_LENGTH = 0.032
HUB_CENTER_Z = -(CANOPY_THICKNESS + STEM_LENGTH + HUB_LENGTH * 0.5)
ARM_PIVOT_X = 0.046
ARM_REACH = 0.258
TIP_DROP = 0.050
TIP_HINGE_OFFSET = 0.030
SHADE_AIM = 0.28


def _bowl_shade_mesh(name: str, *, aim_angle: float):
    outer_profile = [
        (0.016, 0.000),
        (0.026, 0.008),
        (0.052, 0.032),
        (0.084, 0.078),
        (0.104, 0.125),
    ]
    inner_profile = [
        (0.012, 0.000),
        (0.020, 0.007),
        (0.044, 0.029),
        (0.073, 0.074),
        (0.095, 0.125),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    shell.rotate_y(aim_angle)
    return mesh_from_geometry(shell, name)


def _add_arm_geometry(part, *, side_sign: float, metal, accent) -> None:
    part.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=accent,
        name="pivot_collar",
    )
    part.visual(
        Cylinder(radius=0.011, length=0.236),
        origin=Origin(xyz=(side_sign * 0.134, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="arm_bar",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(side_sign * ARM_REACH, 0.0, -0.028)),
        material=metal,
        name="drop_post",
    )
    for y_offset, link_name, knuckle_name in (
        (-0.013, "tip_link_0", "tip_knuckle_0"),
        (0.013, "tip_link_1", "tip_knuckle_1"),
    ):
        part.visual(
            Box((0.032, 0.007, 0.008)),
            origin=Origin(
                xyz=(side_sign * (ARM_REACH + TIP_HINGE_OFFSET * 0.5), y_offset, -TIP_DROP),
            ),
            material=metal,
            name=link_name,
        )
        part.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(
                xyz=(side_sign * (ARM_REACH + TIP_HINGE_OFFSET), y_offset, -TIP_DROP),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=accent,
            name=knuckle_name,
        )


def _add_shade_geometry(part, *, side_sign: float, glass, accent) -> None:
    aim_angle = side_sign * (pi / 2.0 + SHADE_AIM)
    axis_x = side_sign * sin(pi / 2.0 + SHADE_AIM)
    axis_z = -sin(SHADE_AIM)

    part.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="tilt_barrel",
    )
    part.visual(
        Cylinder(radius=0.0095, length=0.040),
        origin=Origin(
            xyz=(axis_x * 0.020, 0.0, axis_z * 0.020),
            rpy=(0.0, aim_angle, 0.0),
        ),
        material=accent,
        name="shade_neck",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(
            xyz=(axis_x * 0.040, 0.0, axis_z * 0.040),
            rpy=(0.0, aim_angle, 0.0),
        ),
        material=accent,
        name="shade_collar",
    )
    part.visual(
        _bowl_shade_mesh(f"{part.name}_shell", aim_angle=aim_angle),
        origin=Origin(xyz=(axis_x * 0.034, 0.0, axis_z * 0.034)),
        material=glass,
        name="shade_shell",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_dual_bowl_fixture")

    brass = model.material("brass", rgba=(0.72, 0.59, 0.34, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.19, 1.0))
    opal_glass = model.material("opal_glass", rgba=(0.95, 0.94, 0.91, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=CANOPY_RADIUS, length=CANOPY_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -CANOPY_THICKNESS * 0.5)),
        material=brass,
        name="canopy_disk",
    )
    canopy.visual(
        Cylinder(radius=0.016, length=STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -(CANOPY_THICKNESS + STEM_LENGTH * 0.5))),
        material=brass,
        name="stem",
    )
    canopy.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, HUB_CENTER_Z)),
        material=dark_metal,
        name="hub",
    )
    canopy.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, HUB_CENTER_Z - HUB_LENGTH * 0.5 - 0.006)),
        material=brass,
        name="hub_cap",
    )
    for side_sign, suffix in ((1.0, "0"), (-1.0, "1")):
        for z_offset, name in ((-0.0075, f"mount_lower_{suffix}"), (0.0075, f"mount_upper_{suffix}")):
            canopy.visual(
                Box((0.038, 0.018, 0.005)),
                origin=Origin(xyz=(side_sign * 0.044, 0.0, HUB_CENTER_Z + z_offset)),
                material=dark_metal,
                name=name,
            )
        for z_offset, name in ((-0.0075, f"pivot_knuckle_lower_{suffix}"), (0.0075, f"pivot_knuckle_upper_{suffix}")):
            canopy.visual(
                Cylinder(radius=0.017, length=0.006),
                origin=Origin(xyz=(side_sign * ARM_PIVOT_X, 0.0, HUB_CENTER_Z + z_offset)),
                material=brass,
                name=name,
            )

    arm_0 = model.part("arm_0")
    _add_arm_geometry(arm_0, side_sign=1.0, metal=brass, accent=dark_metal)

    arm_1 = model.part("arm_1")
    _add_arm_geometry(arm_1, side_sign=-1.0, metal=brass, accent=dark_metal)

    shade_0 = model.part("shade_0")
    _add_shade_geometry(shade_0, side_sign=1.0, glass=opal_glass, accent=dark_metal)

    shade_1 = model.part("shade_1")
    _add_shade_geometry(shade_1, side_sign=-1.0, glass=opal_glass, accent=dark_metal)

    model.articulation(
        "arm_0_swing",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=arm_0,
        origin=Origin(xyz=(ARM_PIVOT_X, 0.0, HUB_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.25, upper=1.25, effort=12.0, velocity=1.3),
    )
    model.articulation(
        "arm_1_swing",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=arm_1,
        origin=Origin(xyz=(-ARM_PIVOT_X, 0.0, HUB_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=-1.25, upper=1.25, effort=12.0, velocity=1.3),
    )
    model.articulation(
        "shade_0_tilt",
        ArticulationType.REVOLUTE,
        parent=arm_0,
        child=shade_0,
        origin=Origin(xyz=(ARM_REACH + TIP_HINGE_OFFSET, 0.0, -TIP_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.85, effort=8.0, velocity=1.4),
    )
    model.articulation(
        "shade_1_tilt",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=shade_1,
        origin=Origin(xyz=(-(ARM_REACH + TIP_HINGE_OFFSET), 0.0, -TIP_DROP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.85, effort=8.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    shade_0 = object_model.get_part("shade_0")
    shade_1 = object_model.get_part("shade_1")

    arm_0_swing = object_model.get_articulation("arm_0_swing")
    arm_1_swing = object_model.get_articulation("arm_1_swing")
    shade_0_tilt = object_model.get_articulation("shade_0_tilt")
    shade_1_tilt = object_model.get_articulation("shade_1_tilt")

    ctx.expect_gap(
        canopy,
        shade_0,
        axis="z",
        min_gap=0.045,
        positive_elem="canopy_disk",
        negative_elem="shade_shell",
        name="shade_0 hangs below the canopy",
    )
    ctx.expect_gap(
        canopy,
        shade_1,
        axis="z",
        min_gap=0.045,
        positive_elem="canopy_disk",
        negative_elem="shade_shell",
        name="shade_1 hangs below the canopy",
    )

    shade_0_rest = _aabb_center(ctx.part_element_world_aabb(shade_0, elem="shade_shell"))
    shade_1_rest = _aabb_center(ctx.part_element_world_aabb(shade_1, elem="shade_shell"))
    ctx.check(
        "bowl shades aim in opposite directions at rest",
        shade_0_rest is not None
        and shade_1_rest is not None
        and shade_0_rest[0] > 0.22
        and shade_1_rest[0] < -0.22
        and abs(shade_0_rest[1] - shade_1_rest[1]) < 0.08,
        details=f"shade_0_center={shade_0_rest}, shade_1_center={shade_1_rest}",
    )

    shade_0_swept = None
    shade_1_swept = None
    with ctx.pose(
        {
            arm_0_swing: arm_0_swing.motion_limits.upper * 0.55,
            arm_1_swing: arm_1_swing.motion_limits.upper * 0.55,
        }
    ):
        shade_0_swept = ctx.part_world_position(shade_0)
        shade_1_swept = ctx.part_world_position(shade_1)

    shade_0_origin_rest = ctx.part_world_position(shade_0)
    shade_1_origin_rest = ctx.part_world_position(shade_1)
    ctx.check(
        "swing arms rotate around the body",
        shade_0_origin_rest is not None
        and shade_1_origin_rest is not None
        and shade_0_swept is not None
        and shade_1_swept is not None
        and shade_0_swept[1] > shade_0_origin_rest[1] + 0.10
        and shade_1_swept[1] > shade_1_origin_rest[1] + 0.10,
        details=(
            f"shade_0_rest={shade_0_origin_rest}, shade_0_swept={shade_0_swept}, "
            f"shade_1_rest={shade_1_origin_rest}, shade_1_swept={shade_1_swept}"
        ),
    )

    shade_0_tipped = None
    with ctx.pose({shade_0_tilt: shade_0_tilt.motion_limits.upper}):
        shade_0_tipped = _aabb_center(ctx.part_element_world_aabb(shade_0, elem="shade_shell"))
    ctx.check(
        "shade_0 tilts downward on its tip hinge",
        shade_0_rest is not None and shade_0_tipped is not None and shade_0_tipped[2] < shade_0_rest[2] - 0.015,
        details=f"shade_0_rest={shade_0_rest}, shade_0_tipped={shade_0_tipped}",
    )

    shade_1_tipped = None
    with ctx.pose({shade_1_tilt: shade_1_tilt.motion_limits.upper}):
        shade_1_tipped = _aabb_center(ctx.part_element_world_aabb(shade_1, elem="shade_shell"))
    ctx.check(
        "shade_1 tilts downward on its tip hinge",
        shade_1_rest is not None and shade_1_tipped is not None and shade_1_tipped[2] < shade_1_rest[2] - 0.015,
        details=f"shade_1_rest={shade_1_rest}, shade_1_tipped={shade_1_tipped}",
    )

    return ctx.report()


object_model = build_object_model()
