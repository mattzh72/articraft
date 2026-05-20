from __future__ import annotations

from math import pi

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


BASE_RADIUS = 0.17
BASE_THICKNESS = 0.045
POST_RADIUS = 0.014
POST_HEIGHT = 1.52
COLLAR_RADIUS = 0.042
COLLAR_THICKNESS = 0.040
UPPER_COLLAR_Z = 1.43
LOWER_COLLAR_Z = 1.31
SHOULDER_X = 0.062
ARM_LENGTH = 0.34
ARM_TUBE_RADIUS = 0.011


def _add_collar(stand, *, sign: float, z: float, material, name_prefix: str) -> None:
    stand.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, z)),
        material=material,
        name=f"{name_prefix}_collar",
    )
    for index, y_pos in enumerate((-0.016, 0.016)):
        stand.visual(
            Box((0.026, 0.010, 0.026)),
            origin=Origin(xyz=(sign * 0.033, y_pos, z)),
            material=material,
            name=f"{name_prefix}_bridge_{index}",
        )
        stand.visual(
            Box((0.036, 0.010, 0.056)),
            origin=Origin(xyz=(sign * 0.060, y_pos, z)),
            material=material,
            name=f"{name_prefix}_ear_{index}",
        )


def _add_arm(part, *, sign: float, material) -> None:
    part.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name="shoulder_barrel",
    )
    part.visual(
        Box((0.034, 0.022, 0.024)),
        origin=Origin(xyz=(sign * 0.017, 0.0, 0.0)),
        material=material,
        name="shoulder_block",
    )
    part.visual(
        Cylinder(radius=ARM_TUBE_RADIUS, length=0.304),
        origin=Origin(xyz=(sign * 0.160, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name="tube",
    )
    for index, y_pos in enumerate((-0.016, 0.016)):
        part.visual(
            Box((0.022, 0.010, 0.022)),
            origin=Origin(xyz=(sign * 0.316, y_pos, 0.0)),
            material=material,
            name=f"tip_bridge_{index}",
        )
        part.visual(
            Box((0.030, 0.010, 0.040)),
            origin=Origin(xyz=(sign * (ARM_LENGTH - 0.002), y_pos, 0.0)),
            material=material,
            name=f"tip_ear_{index}",
        )


def _shade_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=0.034)
        .circle(0.034)
        .workplane(offset=0.170)
        .circle(0.088)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("YZ")
        .workplane(offset=0.048)
        .circle(0.028)
        .workplane(offset=0.156)
        .circle(0.080)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _add_shade(part, *, sign: float, shell_name: str, shell_material, metal_material) -> None:
    shell = _shade_shell_shape()
    if sign < 0.0:
        shell = shell.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 180.0)

    part.visual(
        mesh_from_cadquery(shell, shell_name),
        material=shell_material,
        name="shell",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="tilt_barrel",
    )
    part.visual(
        Box((0.016, 0.026, 0.022)),
        origin=Origin(xyz=(sign * 0.010, 0.0, 0.0)),
        material=metal_material,
        name="neck_block",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.046),
        origin=Origin(xyz=(sign * 0.023, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_material,
        name="neck",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_head_floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.63, 0.55, 0.34, 1.0))
    shade_cream = model.material("shade_cream", rgba=(0.92, 0.90, 0.84, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=matte_black,
        name="base_disk",
    )
    stand.visual(
        Cylinder(radius=0.050, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.009)),
        material=matte_black,
        name="base_hub",
    )
    stand.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_HEIGHT / 2.0)),
        material=matte_black,
        name="post",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_HEIGHT + 0.015)),
        material=matte_black,
        name="post_cap",
    )
    _add_collar(stand, sign=1.0, z=UPPER_COLLAR_Z, material=warm_brass, name_prefix="upper")
    _add_collar(stand, sign=-1.0, z=LOWER_COLLAR_Z, material=warm_brass, name_prefix="lower")

    upper_arm = model.part("upper_arm")
    _add_arm(upper_arm, sign=1.0, material=warm_brass)

    lower_arm = model.part("lower_arm")
    _add_arm(lower_arm, sign=-1.0, material=warm_brass)

    upper_shade = model.part("upper_shade")
    _add_shade(
        upper_shade,
        sign=1.0,
        shell_name="upper_shade_shell",
        shell_material=shade_cream,
        metal_material=warm_brass,
    )

    lower_shade = model.part("lower_shade")
    _add_shade(
        lower_shade,
        sign=-1.0,
        shell_name="lower_shade_shell",
        shell_material=shade_cream,
        metal_material=warm_brass,
    )

    model.articulation(
        "upper_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, UPPER_COLLAR_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.45, upper=0.85),
    )
    model.articulation(
        "lower_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lower_arm,
        origin=Origin(xyz=(-SHOULDER_X, 0.0, LOWER_COLLAR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.45, upper=0.85),
    )
    model.articulation(
        "upper_shade_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=upper_shade,
        origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.75, upper=0.55),
    )
    model.articulation(
        "lower_shade_tilt",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=lower_shade,
        origin=Origin(xyz=(-ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.75, upper=0.55),
    )

    return model


def _aabb_z_span(aabb):
    if aabb is None:
        return None
    return aabb[0][2], aabb[1][2]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    upper_shade = object_model.get_part("upper_shade")
    lower_shade = object_model.get_part("lower_shade")
    upper_arm_hinge = object_model.get_articulation("upper_arm_hinge")
    lower_arm_hinge = object_model.get_articulation("lower_arm_hinge")
    upper_shade_tilt = object_model.get_articulation("upper_shade_tilt")
    lower_shade_tilt = object_model.get_articulation("lower_shade_tilt")

    stand_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "lamp reaches shared-room height",
        stand_aabb is not None and stand_aabb[1][2] > 1.55,
        details=f"stand_aabb={stand_aabb}",
    )

    upper_rest_pos = ctx.part_world_position(upper_shade)
    lower_rest_pos = ctx.part_world_position(lower_shade)
    ctx.check(
        "shade heads sit on opposite sides of the post",
        upper_rest_pos is not None
        and lower_rest_pos is not None
        and upper_rest_pos[0] > 0.30
        and lower_rest_pos[0] < -0.30
        and upper_rest_pos[2] > 1.20
        and lower_rest_pos[2] > 1.10,
        details=f"upper={upper_rest_pos}, lower={lower_rest_pos}",
    )

    upper_raised_pos = None
    with ctx.pose({upper_arm_hinge: 0.75}):
        upper_raised_pos = ctx.part_world_position(upper_shade)
    ctx.check(
        "upper arm can lift its shade",
        upper_rest_pos is not None
        and upper_raised_pos is not None
        and upper_raised_pos[2] > upper_rest_pos[2] + 0.18,
        details=f"rest={upper_rest_pos}, raised={upper_raised_pos}",
    )

    lower_raised_pos = None
    with ctx.pose({lower_arm_hinge: 0.75}):
        lower_raised_pos = ctx.part_world_position(lower_shade)
    ctx.check(
        "lower arm can lift its shade",
        lower_rest_pos is not None
        and lower_raised_pos is not None
        and lower_raised_pos[2] > lower_rest_pos[2] + 0.18,
        details=f"rest={lower_rest_pos}, raised={lower_raised_pos}",
    )

    upper_shell_rest = _aabb_z_span(ctx.part_element_world_aabb(upper_shade, elem="shell"))
    upper_shell_tilted = None
    with ctx.pose({upper_shade_tilt: 0.50}):
        upper_shell_tilted = _aabb_z_span(ctx.part_element_world_aabb(upper_shade, elem="shell"))
    ctx.check(
        "upper shade tilts downward",
        upper_shell_rest is not None
        and upper_shell_tilted is not None
        and upper_shell_tilted[0] < upper_shell_rest[0] - 0.04,
        details=f"rest={upper_shell_rest}, tilted={upper_shell_tilted}",
    )

    lower_shell_rest = _aabb_z_span(ctx.part_element_world_aabb(lower_shade, elem="shell"))
    lower_shell_tilted = None
    with ctx.pose({lower_shade_tilt: 0.50}):
        lower_shell_tilted = _aabb_z_span(ctx.part_element_world_aabb(lower_shade, elem="shell"))
    ctx.check(
        "lower shade tilts downward",
        lower_shell_rest is not None
        and lower_shell_tilted is not None
        and lower_shell_tilted[0] < lower_shell_rest[0] - 0.04,
        details=f"rest={lower_shell_rest}, tilted={lower_shell_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
