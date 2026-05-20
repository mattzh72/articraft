from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.110
BASE_STEP_RADIUS = 0.092
BASE_TOP_RADIUS = 0.074
BASE_THICKNESS = 0.010
BASE_STEP_THICKNESS = 0.012
BASE_TOP_THICKNESS = 0.010
POST_RADIUS = 0.014
POST_HEIGHT = 0.088
SHOULDER_Z = 0.128
SHOULDER_X = 0.026

LOWER_ARM_LENGTH = 0.290
UPPER_ARM_LENGTH = 0.285
ROD_RADIUS = 0.0045
ROD_Y = 0.016
PIVOT_RADIUS = 0.0105
CENTER_BARREL_LEN = 0.024
FORK_EAR_THICKNESS = 0.008
FORK_BRIDGE_WIDTH = 2.0 * ROD_Y + FORK_EAR_THICKNESS


def _x_axis_cylinder(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_axis_cylinder(part, *, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_center_to_fork_arm(part, *, length: float, material) -> None:
    _y_axis_cylinder(
        part,
        radius=PIVOT_RADIUS,
        length=CENTER_BARREL_LEN,
        xyz=(0.0, 0.0, 0.0),
        material=material,
        name="pivot_barrel",
    )

    for index, y in enumerate((-ROD_Y, ROD_Y)):
        part.visual(
            Box((0.020, 0.018, 0.012)),
            origin=Origin(xyz=(0.019, math.copysign(0.012, y), 0.0)),
            material=material,
            name=f"root_bridge_{index}",
        )

    rod_length = length - 0.028
    rod_center_x = 0.022 + rod_length / 2.0
    for index, y in enumerate((-ROD_Y, ROD_Y)):
        _x_axis_cylinder(
            part,
            radius=ROD_RADIUS,
            length=rod_length,
            xyz=(rod_center_x, y, 0.0),
            material=material,
            name=f"rod_{index}",
        )
        part.visual(
            Box((0.022, FORK_EAR_THICKNESS, 0.030)),
            origin=Origin(xyz=(length - 0.007, y, 0.0)),
            material=material,
            name=f"fork_ear_{index}",
        )

    part.visual(
        Box((0.012, FORK_BRIDGE_WIDTH + 0.004, 0.010)),
        origin=Origin(xyz=(length - 0.022, 0.0, 0.012)),
        material=material,
        name="fork_bridge",
    )
    part.visual(
        Box((0.012, FORK_BRIDGE_WIDTH + 0.004, 0.008)),
        origin=Origin(xyz=(length * 0.56, 0.0, -0.006)),
        material=material,
        name="mid_brace",
    )


def _build_shade_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(0.044, 0.028)
        .spline([(0.060, 0.036), (0.085, 0.047), (0.110, 0.056), (0.126, 0.060)])
        .lineTo(0.132, 0.054)
        .spline([(0.114, 0.050), (0.088, 0.041), (0.064, 0.031), (0.048, 0.024)])
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_desk_lamp")

    black = model.material("black", rgba=(0.05, 0.05, 0.06, 1.0))
    graphite = model.material("graphite", rgba=(0.13, 0.13, 0.14, 1.0))
    warm_white = model.material("warm_white", rgba=(0.94, 0.92, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=black,
        name="foot",
    )
    base.visual(
        Cylinder(radius=BASE_STEP_RADIUS, length=BASE_STEP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_STEP_THICKNESS / 2.0)),
        material=black,
        name="step",
    )
    base.visual(
        Cylinder(radius=BASE_TOP_RADIUS, length=BASE_TOP_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_THICKNESS + BASE_STEP_THICKNESS + BASE_TOP_THICKNESS / 2.0,
            )
        ),
        material=black,
        name="top",
    )
    base.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_THICKNESS + BASE_STEP_THICKNESS + BASE_TOP_THICKNESS + POST_HEIGHT / 2.0,
            )
        ),
        material=graphite,
        name="post",
    )
    base.visual(
        Box((0.018, 0.042, 0.014)),
        origin=Origin(xyz=(0.006, 0.0, SHOULDER_Z - 0.019)),
        material=graphite,
        name="shoulder_block",
    )
    for index, y in enumerate((-ROD_Y, ROD_Y)):
        base.visual(
            Box((0.020, FORK_EAR_THICKNESS, 0.034)),
            origin=Origin(xyz=(0.016, y, SHOULDER_Z)),
            material=graphite,
            name=f"shoulder_ear_{index}",
        )

    lower_arm = model.part("lower_arm")
    _build_center_to_fork_arm(lower_arm, length=LOWER_ARM_LENGTH, material=black)

    upper_arm = model.part("upper_arm")
    _build_center_to_fork_arm(upper_arm, length=UPPER_ARM_LENGTH, material=black)

    shade = model.part("shade")
    shade.visual(
        mesh_from_cadquery(_build_shade_shape(), "desk_lamp_shade"),
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
        material=black,
        name="shell",
    )
    _y_axis_cylinder(
        shade,
        radius=PIVOT_RADIUS * 0.95,
        length=CENTER_BARREL_LEN,
        xyz=(0.0, 0.0, 0.0),
        material=black,
        name="pivot_barrel",
    )
    _x_axis_cylinder(
        shade,
        radius=0.008,
        length=0.030,
        xyz=(0.009, 0.0, 0.0),
        material=graphite,
        name="neck",
    )
    shade.visual(
        Cylinder(radius=0.024, length=0.054),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=warm_white,
        name="bulb",
    )

    shoulder = model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z), rpy=(0.0, -1.28, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.20, upper=0.75, effort=18.0, velocity=1.5),
    )
    elbow = model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.82, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.40, upper=1.20, effort=12.0, velocity=1.8),
    )
    head = model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, 0.95, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.85, upper=1.00, effort=6.0, velocity=2.2),
    )

    shoulder.meta["qc_samples"] = [0.0, 0.45]
    elbow.meta["qc_samples"] = [0.0, 0.55]
    head.meta["qc_samples"] = [0.0, 0.55]

    return model


def _aabb_center_z(aabb) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][2] + aabb[1][2]) / 2.0


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    head = object_model.get_articulation("upper_arm_to_shade")

    ctx.allow_overlap(
        base,
        object_model.get_part("lower_arm"),
        elem_a="shoulder_ear_0",
        elem_b="root_bridge_0",
        reason="The shoulder hinge uses simplified interleaved bracket leaves around the pivot axis.",
    )
    ctx.allow_overlap(
        base,
        object_model.get_part("lower_arm"),
        elem_a="shoulder_ear_1",
        elem_b="root_bridge_1",
        reason="The shoulder hinge uses simplified interleaved bracket leaves around the pivot axis.",
    )
    ctx.allow_overlap(
        object_model.get_part("lower_arm"),
        upper_arm,
        elem_a="fork_ear_0",
        elem_b="root_bridge_0",
        reason="The elbow hinge is simplified as nested bracket leaves sharing the pivot region.",
    )
    ctx.allow_overlap(
        object_model.get_part("lower_arm"),
        upper_arm,
        elem_a="fork_ear_1",
        elem_b="root_bridge_1",
        reason="The elbow hinge is simplified as nested bracket leaves sharing the pivot region.",
    )

    ctx.expect_gap(
        shade,
        base,
        axis="z",
        min_gap=0.060,
        name="shade clears the base at rest",
    )
    ctx.expect_origin_gap(
        shade,
        base,
        axis="x",
        min_gap=0.180,
        name="shade projects forward of the base",
    )

    rest_upper = ctx.part_world_aabb(upper_arm)
    with ctx.pose({shoulder: 0.45}):
        raised_upper = ctx.part_world_aabb(upper_arm)
    ctx.check(
        "shoulder articulation lifts the arm",
        rest_upper is not None
        and raised_upper is not None
        and raised_upper[1][2] > rest_upper[1][2] + 0.055,
        details=f"rest={rest_upper}, raised={raised_upper}",
    )

    rest_shade_center_z = _aabb_center_z(ctx.part_world_aabb(shade))
    with ctx.pose({elbow: 0.55}):
        elbow_shade_center_z = _aabb_center_z(ctx.part_world_aabb(shade))
    ctx.check(
        "elbow articulation raises the lamp head",
        rest_shade_center_z is not None
        and elbow_shade_center_z is not None
        and elbow_shade_center_z > rest_shade_center_z + 0.035,
        details=f"rest_z={rest_shade_center_z}, elbow_z={elbow_shade_center_z}",
    )

    with ctx.pose({head: 0.55}):
        tilted_shade = ctx.part_world_aabb(shade)
    ctx.check(
        "head articulation can tilt the shade upward",
        rest_shade_center_z is not None
        and tilted_shade is not None
        and _aabb_center_z(tilted_shade) is not None
        and _aabb_center_z(tilted_shade) > rest_shade_center_z + 0.010,
        details=f"rest_z={rest_shade_center_z}, tilted={tilted_shade}",
    )

    return ctx.report()


object_model = build_object_model()
