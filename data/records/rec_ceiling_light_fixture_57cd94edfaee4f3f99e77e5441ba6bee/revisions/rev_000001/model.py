from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CANOPY_RADIUS = 0.086
CANOPY_HEIGHT = 0.014
CANOPY_LIP_RADIUS = 0.098
CANOPY_LIP_HEIGHT = 0.004

HUB_Z = -0.108
HUB_RADIUS = 0.037
HUB_COLLAR_RADIUS = 0.029
HUB_COLLAR_HEIGHT = 0.040
FINIAL_RADIUS = 0.015
FINIAL_HEIGHT = 0.026

ARM_HINGE_RADIUS = 0.058
ARM_BARREL_RADIUS = 0.0085
ARM_BARREL_LENGTH = 0.018
ARM_TUBE_RADIUS = 0.009
ARM_TUBE_LENGTH = 0.318
ARM_COLLAR_RADIUS = 0.012
ARM_COLLAR_LENGTH = 0.022
ARM_TIP_X = 0.354
ARM_TIP_Z = -0.062
ARM_DROP_X = 0.322
ARM_DROP_RADIUS = 0.006
ARM_DROP_HEIGHT = 0.050

SHADE_BARREL_RADIUS = 0.005
SHADE_BARREL_LENGTH = 0.020
SHADE_STEM_RADIUS = 0.0055
SHADE_STEM_HEIGHT = 0.016
SHADE_CAP_RADIUS = 0.0105
SHADE_CAP_HEIGHT = 0.022
SHADE_CONE_TOP_Z = -0.032
SHADE_CONE_HEIGHT = 0.068
SHADE_TOP_RADIUS = 0.018
SHADE_BOTTOM_RADIUS = 0.060
SHADE_WALL = 0.0025

CARDINAL_YAWS = tuple(index * (math.pi / 2.0) for index in range(4))


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -(length / 2.0)))
        .translate(center)
    )


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return _cylinder_z(radius, length, (0.0, 0.0, 0.0)).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0).translate(
        center
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return _cylinder_z(radius, length, (0.0, 0.0, 0.0)).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0).translate(
        center
    )


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_body_shape() -> cq.Workplane:
    stem_bottom_z = HUB_Z + HUB_RADIUS - 0.007
    stem_height = -CANOPY_HEIGHT - stem_bottom_z

    body = _cylinder_z(CANOPY_RADIUS, CANOPY_HEIGHT, (0.0, 0.0, -(CANOPY_HEIGHT / 2.0)))
    body = body.union(
        _cylinder_z(
            CANOPY_LIP_RADIUS,
            CANOPY_LIP_HEIGHT,
            (0.0, 0.0, -(CANOPY_HEIGHT + (CANOPY_LIP_HEIGHT / 2.0))),
        )
    )
    body = body.union(
        _cylinder_z(
            0.014,
            stem_height,
            (0.0, 0.0, stem_bottom_z + (stem_height / 2.0)),
        )
    )
    body = body.union(cq.Workplane("XY").sphere(HUB_RADIUS).translate((0.0, 0.0, HUB_Z)))
    body = body.union(
        _cylinder_z(HUB_COLLAR_RADIUS, HUB_COLLAR_HEIGHT, (0.0, 0.0, HUB_Z))
    )
    body = body.union(
        _cylinder_z(
            FINIAL_RADIUS,
            FINIAL_HEIGHT,
            (0.0, 0.0, HUB_Z - HUB_RADIUS - (FINIAL_HEIGHT / 2.0) + 0.006),
        )
    )

    bracket = _cylinder_x(0.010, 0.032, (0.028, 0.0, HUB_Z))
    bracket = bracket.union(_box((0.018, 0.004, 0.024), (0.043, 0.011, HUB_Z)))
    bracket = bracket.union(_box((0.018, 0.004, 0.024), (0.043, -0.011, HUB_Z)))
    bracket = bracket.union(_box((0.014, 0.024, 0.010), (0.026, 0.0, HUB_Z - 0.010)))
    slot = _box((0.090, 0.018, 0.024), (0.078, 0.0, HUB_Z))

    for yaw in CARDINAL_YAWS:
        body = body.union(bracket.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(yaw)))
        body = body.cut(slot.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(yaw)))

    return body


def _build_arm_shape() -> cq.Workplane:
    arm = _cylinder_y(ARM_BARREL_RADIUS, ARM_BARREL_LENGTH, (0.0, 0.0, 0.0))
    arm = arm.union(_cylinder_x(ARM_TUBE_RADIUS, ARM_TUBE_LENGTH, (ARM_TUBE_LENGTH / 2.0, 0.0, 0.0)))
    arm = arm.union(_cylinder_x(ARM_COLLAR_RADIUS, ARM_COLLAR_LENGTH, (0.015, 0.0, 0.0)))
    arm = arm.union(_cylinder_x(0.0105, 0.018, (0.301, 0.0, 0.0)))
    arm = arm.union(
        _cylinder_z(
            ARM_DROP_RADIUS,
            ARM_DROP_HEIGHT,
            (ARM_DROP_X, 0.0, ARM_TIP_Z + 0.031),
        )
    )
    arm = arm.union(_box((0.024, 0.026, 0.012), (ARM_TIP_X - 0.020, 0.0, -0.060)))
    arm = arm.union(_box((0.010, 0.004, 0.016), (ARM_TIP_X - 0.008, 0.012, ARM_TIP_Z)))
    arm = arm.union(_box((0.010, 0.004, 0.016), (ARM_TIP_X - 0.008, -0.012, ARM_TIP_Z)))
    return arm


def _build_shade_shape() -> cq.Workplane:
    outer_cone = (
        cq.Workplane("XY")
        .circle(SHADE_TOP_RADIUS)
        .workplane(offset=-SHADE_CONE_HEIGHT)
        .circle(SHADE_BOTTOM_RADIUS)
        .loft()
    )
    inner_cone = (
        cq.Workplane("XY")
        .circle(SHADE_TOP_RADIUS - SHADE_WALL)
        .workplane(offset=-SHADE_CONE_HEIGHT)
        .circle(SHADE_BOTTOM_RADIUS - SHADE_WALL)
        .loft()
    )
    cone_shell = outer_cone.cut(inner_cone).translate((0.0, 0.0, SHADE_CONE_TOP_Z))

    shade = _cylinder_y(SHADE_BARREL_RADIUS, SHADE_BARREL_LENGTH, (0.0, 0.0, 0.0))
    shade = shade.union(_cylinder_z(SHADE_STEM_RADIUS, SHADE_STEM_HEIGHT, (0.0, 0.0, -(SHADE_STEM_HEIGHT / 2.0))))
    shade = shade.union(
        _cylinder_z(
            SHADE_CAP_RADIUS,
            SHADE_CAP_HEIGHT,
            (0.0, 0.0, -(SHADE_STEM_HEIGHT + (SHADE_CAP_HEIGHT / 2.0)) + 0.0025),
        )
    )
    shade = shade.union(_cylinder_z(SHADE_TOP_RADIUS, 0.008, (0.0, 0.0, SHADE_CONE_TOP_Z + 0.002)))
    shade = shade.union(cone_shell)
    return shade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_chandelier")

    aged_brass = model.material("aged_brass", rgba=(0.67, 0.56, 0.34, 1.0))
    soft_cream = model.material("soft_cream", rgba=(0.93, 0.90, 0.82, 1.0))

    body_mesh = mesh_from_cadquery(_build_body_shape(), "body_shell")
    arm_mesh = mesh_from_cadquery(_build_arm_shape(), "arm_shell")
    shade_mesh = mesh_from_cadquery(_build_shade_shape(), "shade_shell")

    body = model.part("body")
    body.visual(body_mesh, material=aged_brass, name="body_shell")

    for index, yaw in enumerate(CARDINAL_YAWS):
        arm = model.part(f"arm_{index}")
        arm.visual(arm_mesh, material=aged_brass, name="arm_shell")

        shade = model.part(f"shade_{index}")
        shade.visual(shade_mesh, material=soft_cream, name="shade_shell")

        arm_joint = model.articulation(
            f"body_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(
                xyz=(
                    ARM_HINGE_RADIUS * math.cos(yaw),
                    ARM_HINGE_RADIUS * math.sin(yaw),
                    HUB_Z,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=1.4,
                lower=-0.55,
                upper=0.70,
            ),
        )
        arm_joint.meta["qc_samples"] = [0.0, 0.35, 0.70]

        shade_joint = model.articulation(
            f"arm_{index}_to_shade_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=shade,
            origin=Origin(xyz=(ARM_TIP_X, 0.0, ARM_TIP_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.0,
                lower=-0.55,
                upper=0.80,
            ),
        )
        shade_joint.meta["qc_samples"] = [0.0, 0.40, 0.80]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    arm_0 = object_model.get_part("arm_0")
    shade_0 = object_model.get_part("shade_0")
    arm_joint = object_model.get_articulation("body_to_arm_0")
    shade_joint = object_model.get_articulation("arm_0_to_shade_0")

    for index in range(4):
        ctx.allow_overlap(
            body,
            object_model.get_part(f"arm_{index}"),
            elem_a="body_shell",
            elem_b="arm_shell",
            reason="The shoulder hinge is represented with a simplified solid knuckle seated into the center-body clevis volume.",
        )
        ctx.allow_overlap(
            object_model.get_part(f"arm_{index}"),
            object_model.get_part(f"shade_{index}"),
            elem_a="arm_shell",
            elem_b="shade_shell",
            reason="The tip hinge uses a compact solid bracket and cap proxy, so the shade knuckle is intentionally nested into the bracket envelope.",
        )

    ctx.expect_contact(
        arm_0,
        body,
        contact_tol=0.001,
        name="arm hinge seats in the center body clevis",
    )
    ctx.expect_contact(
        shade_0,
        arm_0,
        contact_tol=0.001,
        name="shade hinge seats in the tip bracket",
    )
    ctx.expect_origin_gap(
        body,
        shade_0,
        axis="z",
        min_gap=0.14,
        name="shade hangs well below the ceiling canopy",
    )

    rest_shade_pos = ctx.part_world_position(shade_0)
    arm_upper = arm_joint.motion_limits.upper or 0.0
    with ctx.pose({arm_joint: arm_upper}):
        raised_shade_pos = ctx.part_world_position(shade_0)

    ctx.check(
        "arm rotation raises the lamp head toward the canopy",
        rest_shade_pos is not None
        and raised_shade_pos is not None
        and raised_shade_pos[2] > rest_shade_pos[2] + 0.07,
        details=f"rest={rest_shade_pos}, raised={raised_shade_pos}",
    )

    rest_shade_aabb = ctx.part_world_aabb(shade_0)
    shade_upper = shade_joint.motion_limits.upper or 0.0
    with ctx.pose({shade_joint: shade_upper}):
        tilted_shade_aabb = ctx.part_world_aabb(shade_0)

    ctx.check(
        "shade tilt pushes the cone outward from the arm tip",
        rest_shade_aabb is not None
        and tilted_shade_aabb is not None
        and tilted_shade_aabb[1][0] > rest_shade_aabb[1][0] + 0.015,
        details=f"rest={rest_shade_aabb}, tilted={tilted_shade_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
