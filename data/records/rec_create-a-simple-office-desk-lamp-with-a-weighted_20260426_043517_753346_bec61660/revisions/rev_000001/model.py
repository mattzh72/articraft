from __future__ import annotations

import math

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


def _axis_point(distance: float, angle: float) -> tuple[float, float, float]:
    return (distance * math.cos(angle), 0.0, distance * math.sin(angle))


def _pitched_origin(distance: float, angle: float) -> Origin:
    return Origin(xyz=_axis_point(distance, angle), rpy=(0.0, -angle, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_desk_lamp")

    base_finish = model.material("base_finish", rgba=(0.16, 0.16, 0.18, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.70, 0.72, 0.75, 1.0))
    shade_finish = model.material("shade_finish", rgba=(0.87, 0.86, 0.80, 1.0))

    lower_angle = math.radians(60.0)
    upper_angle = math.radians(30.0)

    base_hinge_xyz = (-0.006, 0.0, 0.064)

    lower_beam_length = 0.26
    upper_beam_length = 0.22
    lower_beam_start = 0.004
    upper_beam_start = 0.004

    elbow_fork_xyz = _axis_point(lower_beam_start + lower_beam_length + 0.010, lower_angle)
    elbow_joint_xyz = _axis_point(lower_beam_start + lower_beam_length + 0.022, lower_angle)
    elbow_mount_xyz = _axis_point(lower_beam_start + lower_beam_length - 0.004, lower_angle)
    head_fork_xyz = _axis_point(upper_beam_start + upper_beam_length + 0.012, upper_angle)
    head_joint_xyz = _axis_point(upper_beam_start + upper_beam_length + 0.024, upper_angle)
    head_mount_xyz = _axis_point(upper_beam_start + upper_beam_length - 0.002, upper_angle)

    shade_outer = [
        (0.024, 0.000),
        (0.030, 0.010),
        (0.038, 0.040),
        (0.041, 0.078),
        (0.043, 0.106),
    ]
    shade_inner = [
        (0.019, 0.008),
        (0.025, 0.018),
        (0.033, 0.040),
        (0.036, 0.078),
        (0.038, 0.102),
    ]
    shade_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(shade_outer, shade_inner, segments=56),
        "lamp_head_shade",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.090, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_finish,
        name="weight",
    )
    base.visual(
        Box((0.050, 0.055, 0.050)),
        origin=Origin(xyz=(-0.040, 0.0, 0.043)),
        material=base_finish,
        name="pedestal",
    )
    for index, y_pos in enumerate((-0.017, 0.017)):
        base.visual(
            Box((0.024, 0.008, 0.030)),
            origin=Origin(xyz=(-0.016, y_pos, base_hinge_xyz[2])),
            material=base_finish,
            name=f"base_fork_{index}",
        )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_finish,
        name="base_barrel",
    )
    lower_arm.visual(
        Box((lower_beam_length, 0.016, 0.012)),
        origin=_pitched_origin(lower_beam_start + 0.5 * lower_beam_length, lower_angle),
        material=arm_finish,
        name="beam",
    )
    lower_arm.visual(
        Box((0.024, 0.028, 0.014)),
        origin=Origin(
            xyz=elbow_mount_xyz,
            rpy=(0.0, -lower_angle, 0.0),
        ),
        material=arm_finish,
        name="elbow_mount",
    )
    for index, y_pos in enumerate((-0.017, 0.017)):
        lower_arm.visual(
            Box((0.030, 0.008, 0.022)),
            origin=Origin(
                xyz=(elbow_fork_xyz[0], y_pos, elbow_fork_xyz[2]),
                rpy=(0.0, -lower_angle, 0.0),
            ),
            material=arm_finish,
            name=f"elbow_fork_{index}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.0085, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_finish,
        name="elbow_barrel",
    )
    upper_arm.visual(
        Box((upper_beam_length, 0.014, 0.010)),
        origin=_pitched_origin(upper_beam_start + 0.5 * upper_beam_length, upper_angle),
        material=arm_finish,
        name="beam",
    )
    upper_arm.visual(
        Box((0.020, 0.024, 0.012)),
        origin=Origin(
            xyz=head_mount_xyz,
            rpy=(0.0, -upper_angle, 0.0),
        ),
        material=arm_finish,
        name="head_mount",
    )
    for index, y_pos in enumerate((-0.014, 0.014)):
        upper_arm.visual(
            Box((0.026, 0.007, 0.020)),
            origin=Origin(
                xyz=(head_fork_xyz[0], y_pos, head_fork_xyz[2]),
                rpy=(0.0, -upper_angle, 0.0),
            ),
            material=arm_finish,
            name=f"head_fork_{index}",
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.007, length=0.021),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_finish,
        name="pivot_barrel",
    )
    head.visual(
        Box((0.034, 0.014, 0.012)),
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material=arm_finish,
        name="neck",
    )
    head.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_finish,
        name="socket",
    )
    head.visual(
        shade_shell,
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shade_finish,
        name="shade_shell",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=base_hinge_xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=math.radians(-35.0),
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=elbow_joint_xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=math.radians(-55.0),
            upper=math.radians(50.0),
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=head_joint_xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=math.radians(-40.0),
            upper=math.radians(32.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    head = object_model.get_part("head")

    base_hinge = object_model.get_articulation("base_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    head_tilt = object_model.get_articulation("head_tilt")

    base_limits = base_hinge.motion_limits
    elbow_limits = elbow_hinge.motion_limits
    head_limits = head_tilt.motion_limits

    ctx.expect_gap(
        head,
        base,
        axis="z",
        min_gap=0.30,
        name="lamp head sits clearly above the weighted base disk",
        negative_elem="weight",
    )

    with ctx.pose({base_hinge: base_limits.lower}):
        base_folded = ctx.part_world_aabb(head)
        ctx.expect_gap(
            head,
            base,
            axis="z",
            max_penetration=0.0,
            name="head still clears the base disk at the low base-hinge pose",
            negative_elem="weight",
        )
    with ctx.pose({base_hinge: base_limits.upper}):
        base_raised = ctx.part_world_aabb(head)
    ctx.check(
        "base hinge lifts the lamp",
        base_folded is not None
        and base_raised is not None
        and base_raised[1][2] > base_folded[1][2] + 0.10,
        details=f"folded={base_folded}, raised={base_raised}",
    )

    with ctx.pose({elbow_hinge: elbow_limits.lower}):
        elbow_folded = ctx.part_world_aabb(head)
    with ctx.pose({elbow_hinge: elbow_limits.upper}):
        elbow_open = ctx.part_world_aabb(head)
    ctx.check(
        "elbow hinge opens the upper arm",
        elbow_folded is not None
        and elbow_open is not None
        and elbow_open[1][2] > elbow_folded[1][2] + 0.06,
        details=f"folded={elbow_folded}, open={elbow_open}",
    )

    with ctx.pose({head_tilt: head_limits.lower}):
        shade_down = ctx.part_element_world_aabb(head, elem="shade_shell")
    with ctx.pose({head_tilt: head_limits.upper}):
        shade_up = ctx.part_element_world_aabb(head, elem="shade_shell")
    ctx.check(
        "lamp head tilts upward",
        shade_down is not None
        and shade_up is not None
        and shade_up[1][2] > shade_down[1][2] + 0.03,
        details=f"down={shade_down}, up={shade_up}",
    )

    return ctx.report()


object_model = build_object_model()
