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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_RADIUS = 0.095
BASE_THICKNESS = 0.018
PEDESTAL_RADIUS = 0.022
PEDESTAL_HEIGHT = 0.036
SHOULDER_Z = 0.076
SHOULDER_CHEEK_DEPTH = 0.030
SHOULDER_CHEEK_THICKNESS = 0.007
SHOULDER_CHEEK_HEIGHT = 0.036
SHOULDER_GAP = 0.024

LOWER_ARM_LENGTH = 0.190
LOWER_ARM_PITCH = -0.88
UPPER_ARM_LENGTH = 0.165
UPPER_ARM_PITCH = -0.38
ARM_BAR_THICKNESS = 0.0045
ARM_BAR_HEIGHT = 0.010
ARM_BAR_OFFSET_Y = 0.014

LOWER_ROOT_BARREL_RADIUS = 0.0065
LOWER_ROOT_BARREL_LENGTH = 0.022
ELBOW_GAP = 0.022
ELBOW_CHEEK_THICKNESS = 0.005
ELBOW_CHEEK_DEPTH = 0.024
ELBOW_CHEEK_HEIGHT = 0.020

UPPER_ROOT_BARREL_RADIUS = 0.0060
UPPER_ROOT_BARREL_LENGTH = 0.020
TIP_GAP = 0.018
TIP_CHEEK_THICKNESS = 0.0045
TIP_CHEEK_DEPTH = 0.020
TIP_CHEEK_HEIGHT = 0.018

SHADE_BARREL_RADIUS = 0.0048
SHADE_BARREL_LENGTH = 0.016
SHADE_NECK_RADIUS = 0.010
SHADE_NECK_LENGTH = 0.028
SHADE_REST_PITCH = 0.58
SHADE_LENGTH = 0.112
SHADE_REAR_RADIUS = 0.024
SHADE_FRONT_RADIUS = 0.060
SHADE_WALL = 0.0018
SHADE_SOCKET_RADIUS = 0.012


def _local_x_vector(distance: float, pitch: float) -> tuple[float, float, float]:
    return (
        distance * math.cos(pitch),
        0.0,
        -distance * math.sin(pitch),
    )


def _vector_add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _add_parallel_arm(
    part,
    *,
    prefix: str,
    arm_length: float,
    arm_pitch: float,
    root_barrel_radius: float,
    root_barrel_length: float,
    clevis_gap: float,
    clevis_thickness: float,
    clevis_depth: float,
    clevis_height: float,
    arm_material,
    hardware_material,
) -> tuple[float, float, float]:
    bar_inner_y = ARM_BAR_OFFSET_Y - ARM_BAR_THICKNESS / 2.0

    part.visual(
        Cylinder(radius=root_barrel_radius, length=root_barrel_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name=f"{prefix}_root_barrel",
    )

    root_link_width = max(bar_inner_y - root_barrel_length / 2.0 + 0.002, 0.003)
    root_link_center = _local_x_vector(0.006, arm_pitch)
    root_link_y = (root_barrel_length / 2.0 + bar_inner_y) / 2.0
    for index, y_sign in enumerate((-1.0, 1.0)):
        part.visual(
            Box((0.012, root_link_width, 0.008)),
            origin=Origin(
                xyz=(root_link_center[0], y_sign * root_link_y, root_link_center[2]),
                rpy=(0.0, arm_pitch, 0.0),
            ),
            material=hardware_material,
            name=f"{prefix}_root_link_{index}",
        )

    arm_center = _local_x_vector(arm_length / 2.0, arm_pitch)
    for index, y_sign in enumerate((-1.0, 1.0)):
        part.visual(
            Box((arm_length, ARM_BAR_THICKNESS, ARM_BAR_HEIGHT)),
            origin=Origin(
                xyz=(arm_center[0], y_sign * ARM_BAR_OFFSET_Y, arm_center[2]),
                rpy=(0.0, arm_pitch, 0.0),
            ),
            material=arm_material,
            name=f"{prefix}_bar_{index}",
        )

    elbow_origin = _local_x_vector(arm_length, arm_pitch)
    clevis_center = _vector_add(elbow_origin, _local_x_vector(clevis_depth / 2.0, arm_pitch))
    clevis_y = clevis_gap / 2.0 + clevis_thickness / 2.0

    for index, y_sign in enumerate((-1.0, 1.0)):
        part.visual(
            Box((clevis_depth, clevis_thickness, clevis_height)),
            origin=Origin(
                xyz=(clevis_center[0], y_sign * clevis_y, clevis_center[2]),
                rpy=(0.0, arm_pitch, 0.0),
            ),
            material=hardware_material,
            name=f"{prefix}_clevis_{index}",
        )

    brace_center = _local_x_vector(arm_length * 0.38, arm_pitch)
    part.visual(
        Box((0.016, 2.0 * bar_inner_y + 0.002, 0.007)),
        origin=Origin(
            xyz=brace_center,
            rpy=(0.0, arm_pitch, 0.0),
        ),
        material=hardware_material,
        name=f"{prefix}_brace",
    )
    return elbow_origin


def _build_shade_shell():
    inner_rear_radius = max(SHADE_REAR_RADIUS - SHADE_WALL, 0.004)
    inner_front_radius = SHADE_FRONT_RADIUS - SHADE_WALL

    outer = (
        cq.Workplane("YZ")
        .circle(SHADE_REAR_RADIUS)
        .workplane(offset=SHADE_LENGTH)
        .circle(SHADE_FRONT_RADIUS)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("YZ")
        .workplane(offset=SHADE_WALL)
        .circle(inner_rear_radius)
        .workplane(offset=SHADE_LENGTH - SHADE_WALL)
        .circle(inner_front_radius)
        .loft(combine=True)
    )
    rear_socket = (
        cq.Workplane("YZ")
        .circle(SHADE_SOCKET_RADIUS)
        .extrude(SHADE_REAR_RADIUS * 2.2)
    )
    return outer.cut(inner).cut(rear_socket)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architect_desk_lamp")

    base_paint = model.material("base_paint", rgba=(0.13, 0.13, 0.14, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.68, 0.69, 0.72, 1.0))
    hardware = model.material("hardware", rgba=(0.34, 0.35, 0.37, 1.0))
    shade_paint = model.material("shade_paint", rgba=(0.16, 0.16, 0.17, 1.0))

    lower_arm_end = _local_x_vector(LOWER_ARM_LENGTH, LOWER_ARM_PITCH)
    upper_arm_end = _local_x_vector(UPPER_ARM_LENGTH, UPPER_ARM_PITCH)

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_paint,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)),
        material=base_paint,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.027, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=hardware,
        name="shoulder_collar",
    )
    shoulder_cheek_y = SHOULDER_GAP / 2.0 + SHOULDER_CHEEK_THICKNESS / 2.0
    for index, y_sign in enumerate((-1.0, 1.0)):
        base.visual(
            Box((SHOULDER_CHEEK_DEPTH, SHOULDER_CHEEK_THICKNESS, SHOULDER_CHEEK_HEIGHT)),
            origin=Origin(
                xyz=(0.010, y_sign * shoulder_cheek_y, 0.071),
            ),
            material=hardware,
            name=f"shoulder_cheek_{index}",
        )

    lower_arm = model.part("lower_arm")
    _add_parallel_arm(
        lower_arm,
        prefix="lower",
        arm_length=LOWER_ARM_LENGTH,
        arm_pitch=LOWER_ARM_PITCH,
        root_barrel_radius=LOWER_ROOT_BARREL_RADIUS,
        root_barrel_length=LOWER_ROOT_BARREL_LENGTH,
        clevis_gap=ELBOW_GAP,
        clevis_thickness=ELBOW_CHEEK_THICKNESS,
        clevis_depth=ELBOW_CHEEK_DEPTH,
        clevis_height=ELBOW_CHEEK_HEIGHT,
        arm_material=arm_finish,
        hardware_material=hardware,
    )

    upper_arm = model.part("upper_arm")
    _add_parallel_arm(
        upper_arm,
        prefix="upper",
        arm_length=UPPER_ARM_LENGTH,
        arm_pitch=UPPER_ARM_PITCH,
        root_barrel_radius=UPPER_ROOT_BARREL_RADIUS,
        root_barrel_length=UPPER_ROOT_BARREL_LENGTH,
        clevis_gap=TIP_GAP,
        clevis_thickness=TIP_CHEEK_THICKNESS,
        clevis_depth=TIP_CHEEK_DEPTH,
        clevis_height=TIP_CHEEK_HEIGHT,
        arm_material=arm_finish,
        hardware_material=hardware,
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=SHADE_BARREL_RADIUS, length=SHADE_BARREL_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="shade_barrel",
    )

    neck_center = _local_x_vector(SHADE_NECK_LENGTH / 2.0, SHADE_REST_PITCH)
    shade.visual(
        Cylinder(radius=SHADE_NECK_RADIUS, length=SHADE_NECK_LENGTH),
        origin=Origin(
            xyz=neck_center,
            rpy=(0.0, math.pi / 2.0 + SHADE_REST_PITCH, 0.0),
        ),
        material=hardware,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(
            xyz=_local_x_vector(0.018, SHADE_REST_PITCH),
            rpy=(0.0, math.pi / 2.0 + SHADE_REST_PITCH, 0.0),
        ),
        material=hardware,
        name="shade_socket",
    )

    shade.visual(
        mesh_from_cadquery(_build_shade_shell(), "architect_lamp_shade_shell"),
        origin=Origin(
            xyz=_local_x_vector(0.015, SHADE_REST_PITCH),
            rpy=(0.0, SHADE_REST_PITCH, 0.0),
        ),
        material=shade_paint,
        name="shade_shell",
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.70,
            upper=0.85,
            effort=20.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=lower_arm_end),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.05,
            upper=0.95,
            effort=14.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=upper_arm_end),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.45,
            upper=0.75,
            effort=8.0,
            velocity=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    tip = object_model.get_articulation("upper_arm_to_shade")

    ctx.expect_overlap(
        lower_arm,
        base,
        axes="xy",
        elem_a="lower_root_barrel",
        elem_b="shoulder_collar",
        min_overlap=0.010,
        name="lower arm barrel stays centered over shoulder collar",
    )
    ctx.expect_gap(
        shade,
        base,
        axis="z",
        min_gap=0.135,
        positive_elem="shade_shell",
        negative_elem="base_disk",
        name="shade clears the weighted base in the rest pose",
    )

    rest_upper_position = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder: 0.45}):
        raised_upper_position = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder hinge raises the elbow",
        rest_upper_position is not None
        and raised_upper_position is not None
        and raised_upper_position[2] > rest_upper_position[2] + 0.035,
        details=f"rest={rest_upper_position}, raised={raised_upper_position}",
    )

    rest_shade_position = ctx.part_world_position(shade)
    with ctx.pose({elbow: 0.45}):
        raised_shade_position = ctx.part_world_position(shade)
    ctx.check(
        "elbow hinge raises the shade mount",
        rest_shade_position is not None
        and raised_shade_position is not None
        and raised_shade_position[2] > rest_shade_position[2] + 0.025,
        details=f"rest={rest_shade_position}, raised={raised_shade_position}",
    )

    with ctx.pose({tip: -0.25}):
        low_aim_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    with ctx.pose({tip: 0.45}):
        high_aim_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    ctx.check(
        "shade hinge tilts the cone upward",
        low_aim_center is not None
        and high_aim_center is not None
        and high_aim_center[2] > low_aim_center[2] + 0.020,
        details=f"low={low_aim_center}, high={high_aim_center}",
    )

    return ctx.report()


object_model = build_object_model()
