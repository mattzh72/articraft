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


ARM_COUNT = 6
CANOPY_RADIUS = 0.155
CANOPY_THICKNESS = 0.028
TRIM_RADIUS = 0.132
TRIM_THICKNESS = 0.012
STEM_RADIUS = 0.016
STEM_LENGTH = 0.055
HUB_Z = -0.125
HUB_RADIUS = 0.060
SHOULDER_RADIUS = 0.095
SHOULDER_NECK_RADIUS = 0.015
SHOULDER_NECK_LENGTH = 0.040

ARM_DROP = 0.12
ARM_SWING_LIMIT = 0.85
ARM_YOKE_X = 0.166
ARM_YOKE_Z = -0.022

HEAD_REST_PITCH = 0.28
HEAD_TILT_LOWER = -0.45
HEAD_TILT_UPPER = 0.85


def _build_head_shell() -> cq.Workplane:
    outer_radius = 0.034
    shell_thickness = 0.003
    shell_length = 0.108
    back_wall = 0.012

    outer = cq.Workplane("XY").circle(outer_radius).extrude(shell_length)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=back_wall)
        .circle(outer_radius - shell_thickness)
        .extrude(shell_length - back_wall + 0.001)
    )
    shell = outer.cut(cavity)
    return shell.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0).translate((0.008, 0.0, 0.0))


def _pairwise_max_xy_span(points: list[tuple[float, float, float]]) -> float:
    max_span = 0.0
    for index, point_a in enumerate(points):
        for point_b in points[index + 1 :]:
            span = math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])
            max_span = max(max_span, span)
    return max_span


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_spotlight")

    brushed_brass = model.material("brushed_brass", rgba=(0.72, 0.62, 0.37, 1.0))
    soft_black = model.material("soft_black", rgba=(0.13, 0.13, 0.14, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.93, 0.91, 0.84, 0.88))

    head_shell_mesh = mesh_from_cadquery(_build_head_shell(), "spot_head_shell")

    fixture = model.part("fixture")
    fixture.visual(
        Cylinder(radius=CANOPY_RADIUS, length=CANOPY_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -CANOPY_THICKNESS * 0.5)),
        material=brushed_brass,
        name="canopy",
    )
    fixture.visual(
        Cylinder(radius=TRIM_RADIUS, length=TRIM_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -(CANOPY_THICKNESS + TRIM_THICKNESS * 0.5))),
        material=brushed_brass,
        name="canopy_trim",
    )
    fixture.visual(
        Cylinder(radius=STEM_RADIUS, length=STEM_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, -(CANOPY_THICKNESS + TRIM_THICKNESS + STEM_LENGTH * 0.5))
        ),
        material=brushed_brass,
        name="stem",
    )
    fixture.visual(
        Sphere(radius=HUB_RADIUS),
        origin=Origin(xyz=(0.0, 0.0, HUB_Z)),
        material=brushed_brass,
        name="hub",
    )
    fixture.visual(
        Cylinder(radius=0.046, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, HUB_Z + 0.010)),
        material=soft_black,
        name="hub_band",
    )

    for index in range(ARM_COUNT):
        angle = (2.0 * math.pi * index) / ARM_COUNT
        fixture.visual(
            Cylinder(radius=SHOULDER_NECK_RADIUS, length=SHOULDER_NECK_LENGTH),
            origin=Origin(
                xyz=(
                    (SHOULDER_RADIUS - SHOULDER_NECK_LENGTH * 0.5) * math.cos(angle),
                    (SHOULDER_RADIUS - SHOULDER_NECK_LENGTH * 0.5) * math.sin(angle),
                    HUB_Z,
                ),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=brushed_brass,
            name=f"shoulder_neck_{index}",
        )

        arm = model.part(f"arm_{index}")
        arm.visual(
            Box((0.022, 0.032, 0.026)),
            origin=Origin(xyz=(0.011, 0.0, 0.0)),
            material=brushed_brass,
            name="shoulder_block",
        )
        arm.visual(
            Cylinder(radius=0.0135, length=0.030),
            origin=Origin(xyz=(0.032, 0.0, -0.004), rpy=(0.0, math.pi / 2.0 + ARM_DROP, 0.0)),
            material=brushed_brass,
            name="arm_collar",
        )
        arm.visual(
            Cylinder(radius=0.0105, length=0.124),
            origin=Origin(xyz=(0.086, 0.0, -0.011), rpy=(0.0, math.pi / 2.0 + ARM_DROP, 0.0)),
            material=brushed_brass,
            name="arm_tube",
        )
        arm.visual(
            Cylinder(radius=0.0115, length=0.026),
            origin=Origin(xyz=(0.136, 0.0, -0.018), rpy=(0.0, math.pi / 2.0 + ARM_DROP, 0.0)),
            material=brushed_brass,
            name="yoke_stem",
        )
        arm.visual(
            Box((0.016, 0.100, 0.022)),
            origin=Origin(xyz=(0.146, 0.0, -0.020)),
            material=brushed_brass,
            name="yoke_bridge",
        )
        arm.visual(
            Box((0.022, 0.010, 0.086)),
            origin=Origin(xyz=(0.157, 0.045, -0.005)),
            material=brushed_brass,
            name="yoke_cheek_pos",
        )
        arm.visual(
            Box((0.022, 0.010, 0.086)),
            origin=Origin(xyz=(0.157, -0.045, -0.005)),
            material=brushed_brass,
            name="yoke_cheek_neg",
        )

        head = model.part(f"head_{index}")
        head.visual(
            head_shell_mesh,
            origin=Origin(rpy=(0.0, HEAD_REST_PITCH, 0.0)),
            material=soft_black,
            name="shell",
        )
        head.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.004, 0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=soft_black,
            name="side_pin_pos",
        )
        head.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.004, -0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=soft_black,
            name="side_pin_neg",
        )
        head.visual(
            Cylinder(radius=0.037, length=0.012),
            origin=Origin(xyz=(0.101, 0.0, 0.0), rpy=(0.0, math.pi / 2.0 + HEAD_REST_PITCH, 0.0)),
            material=brushed_brass,
            name="bezel",
        )
        head.visual(
            Cylinder(radius=0.026, length=0.004),
            origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0 + HEAD_REST_PITCH, 0.0)),
            material=frosted_glass,
            name="lens",
        )

        model.articulation(
            f"arm_hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=fixture,
            child=arm,
            origin=Origin(
                xyz=(SHOULDER_RADIUS * math.cos(angle), SHOULDER_RADIUS * math.sin(angle), HUB_Z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.5,
                lower=-ARM_SWING_LIMIT,
                upper=ARM_SWING_LIMIT,
            ),
        )
        model.articulation(
            f"head_hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=head,
            origin=Origin(xyz=(ARM_YOKE_X, 0.0, ARM_YOKE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.0,
                lower=HEAD_TILT_LOWER,
                upper=HEAD_TILT_UPPER,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    arm_joints = [object_model.get_articulation(f"arm_hinge_{index}") for index in range(ARM_COUNT)]
    head_joints = [object_model.get_articulation(f"head_hinge_{index}") for index in range(ARM_COUNT)]
    head_parts = [object_model.get_part(f"head_{index}") for index in range(ARM_COUNT)]

    ctx.check(
        "six arms and six heads are articulated",
        len(arm_joints) == ARM_COUNT and len(head_joints) == ARM_COUNT and len(head_parts) == ARM_COUNT,
        details=f"arms={len(arm_joints)}, heads={len(head_joints)}, head_parts={len(head_parts)}",
    )

    head_positions = [ctx.part_world_position(head) for head in head_parts]
    scale_ok = all(position is not None for position in head_positions)
    span = None
    if scale_ok:
        span = _pairwise_max_xy_span(head_positions)  # type: ignore[arg-type]
        z_values = [position[2] for position in head_positions]  # type: ignore[index]
        scale_ok = 0.52 <= span <= 0.75 and max(z_values) < -0.10 and min(z_values) > -0.20
    ctx.check(
        "fixture reads as living room ceiling scale",
        scale_ok,
        details=f"span={span}, head_positions={head_positions}",
    )

    head_0 = object_model.get_part("head_0")
    arm_hinge_0 = object_model.get_articulation("arm_hinge_0")
    head_hinge_0 = object_model.get_articulation("head_hinge_0")

    rest_head_origin = ctx.part_world_position(head_0)
    with ctx.pose({arm_hinge_0: 0.60}):
        swung_head_origin = ctx.part_world_position(head_0)
    ctx.check(
        "arm_0 swivels around the hub",
        rest_head_origin is not None
        and swung_head_origin is not None
        and swung_head_origin[1] > rest_head_origin[1] + 0.08,
        details=f"rest={rest_head_origin}, swung={swung_head_origin}",
    )

    rest_head_center = _aabb_center(ctx.part_world_aabb(head_0))
    with ctx.pose({head_hinge_0: 0.55}):
        tilted_head_center = _aabb_center(ctx.part_world_aabb(head_0))
    ctx.check(
        "head_0 tilts downward in its yoke",
        rest_head_center is not None
        and tilted_head_center is not None
        and tilted_head_center[2] < rest_head_center[2] - 0.018,
        details=f"rest_center={rest_head_center}, tilted_center={tilted_head_center}",
    )

    return ctx.report()


object_model = build_object_model()
