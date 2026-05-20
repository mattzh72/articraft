from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _y_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_belgian_waffle_maker")

    stand_metal = model.material("stand_metal", rgba=(0.72, 0.72, 0.74, 1.0))
    shell_black = model.material("shell_black", rgba=(0.12, 0.12, 0.13, 1.0))
    plate_dark = model.material("plate_dark", rgba=(0.20, 0.20, 0.22, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_metal = model.material("accent_metal", rgba=(0.84, 0.84, 0.86, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.30, 0.34, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=stand_metal,
        name="base",
    )
    stand.visual(
        Box((0.12, 0.16, 0.08)),
        origin=Origin(xyz=(-0.02, 0.0, 0.06)),
        material=stand_metal,
        name="pedestal",
    )
    stand.visual(
        Box((0.10, 0.014, 0.22)),
        origin=Origin(xyz=(-0.01, -0.143, 0.13)),
        material=stand_metal,
        name="support_0",
    )
    stand.visual(
        Box((0.10, 0.014, 0.22)),
        origin=Origin(xyz=(-0.01, 0.143, 0.13)),
        material=stand_metal,
        name="support_1",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=_y_cylinder_origin((0.0, -0.132, 0.21)),
        material=accent_metal,
        name="bearing_0",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=_y_cylinder_origin((0.0, 0.132, 0.21)),
        material=accent_metal,
        name="bearing_1",
    )

    shell_lower = model.part("shell_lower")
    shell_lower.visual(
        Cylinder(radius=0.118, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=shell_black,
        name="lower_body",
    )
    shell_lower.visual(
        Cylinder(radius=0.108, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=trim_dark,
        name="lower_rim",
    )
    shell_lower.visual(
        Cylinder(radius=0.100, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=plate_dark,
        name="lower_plate",
    )
    shell_lower.visual(
        Box((0.052, 0.024, 0.026)),
        origin=Origin(xyz=(0.050, 0.106, -0.020)),
        material=shell_black,
        name="dial_pod",
    )
    shell_lower.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=_y_cylinder_origin((0.0, -0.124, -0.01)),
        material=accent_metal,
        name="trunnion_0",
    )
    shell_lower.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=_y_cylinder_origin((0.0, 0.124, -0.01)),
        material=accent_metal,
        name="trunnion_1",
    )
    shell_lower.visual(
        Cylinder(radius=0.0085, length=0.020),
        origin=_y_cylinder_origin((-0.095, -0.050, 0.004)),
        material=accent_metal,
        name="hinge_barrel_0",
    )
    shell_lower.visual(
        Cylinder(radius=0.0085, length=0.020),
        origin=_y_cylinder_origin((-0.095, 0.050, 0.004)),
        material=accent_metal,
        name="hinge_barrel_1",
    )

    shell_upper = model.part("shell_upper")
    shell_upper.visual(
        Cylinder(radius=0.105, length=0.030),
        origin=Origin(xyz=(0.115, 0.0, 0.022)),
        material=shell_black,
        name="upper_body",
    )
    shell_upper.visual(
        Cylinder(radius=0.094, length=0.012),
        origin=Origin(xyz=(0.117, 0.0, 0.039)),
        material=trim_dark,
        name="upper_cap",
    )
    shell_upper.visual(
        Box((0.034, 0.170, 0.016)),
        origin=Origin(xyz=(0.024, 0.0, 0.016)),
        material=shell_black,
        name="rear_bridge",
    )
    shell_upper.visual(
        Cylinder(radius=0.094, length=0.010),
        origin=Origin(xyz=(0.104, 0.0, 0.006)),
        material=plate_dark,
        name="upper_plate",
    )
    shell_upper.visual(
        Cylinder(radius=0.0085, length=0.064),
        origin=_y_cylinder_origin((0.0, 0.0, 0.004)),
        material=accent_metal,
        name="hinge_barrel",
    )
    shell_upper.visual(
        Box((0.060, 0.048, 0.010)),
        origin=Origin(xyz=(0.202, 0.0, 0.018)),
        material=handle_dark,
        name="handle_bridge",
    )
    shell_upper.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=_y_cylinder_origin((0.232, 0.0, 0.016)),
        material=handle_dark,
        name="handle_grip",
    )

    thermostat_dial = model.part("thermostat_dial")
    thermostat_dial.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=_y_cylinder_origin((0.0, 0.003, 0.0)),
        material=accent_metal,
        name="dial_shaft",
    )
    thermostat_dial.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=_y_cylinder_origin((0.0, 0.005, 0.0)),
        material=trim_dark,
        name="dial_skirt",
    )
    thermostat_dial.visual(
        Cylinder(radius=0.019, length=0.013),
        origin=_y_cylinder_origin((0.0, 0.0115, 0.0)),
        material=accent_metal,
        name="dial_cap",
    )
    thermostat_dial.visual(
        Box((0.004, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, 0.019, 0.010)),
        material=handle_dark,
        name="dial_pointer",
    )

    model.articulation(
        "stand_to_shell",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=shell_lower,
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )
    model.articulation(
        "lower_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell_lower,
        child=shell_upper,
        origin=Origin(xyz=(-0.095, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "lower_to_dial",
        ArticulationType.CONTINUOUS,
        parent=shell_lower,
        child=thermostat_dial,
        origin=Origin(xyz=(0.055, 0.118, -0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    shell_lower = object_model.get_part("shell_lower")
    shell_upper = object_model.get_part("shell_upper")
    thermostat_dial = object_model.get_part("thermostat_dial")
    flip_joint = object_model.get_articulation("stand_to_shell")
    lid_joint = object_model.get_articulation("lower_to_lid")
    dial_joint = object_model.get_articulation("lower_to_dial")

    lid_limits = lid_joint.motion_limits
    assert lid_limits is not None and lid_limits.upper is not None

    ctx.expect_gap(
        shell_upper,
        shell_lower,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        max_gap=0.003,
        max_penetration=0.0,
        name="lid closes onto the lower plate without collision",
    )
    ctx.expect_overlap(
        shell_upper,
        shell_lower,
        axes="xy",
        elem_a="upper_plate",
        elem_b="lower_plate",
        min_overlap=0.18,
        name="closed waffle plates stay aligned",
    )
    ctx.expect_gap(
        shell_lower,
        stand,
        axis="z",
        positive_elem="lower_body",
        negative_elem="base",
        min_gap=0.14,
        name="cooking shell rides above the base",
    )
    ctx.expect_origin_gap(
        thermostat_dial,
        shell_lower,
        axis="y",
        min_gap=0.10,
        name="thermostat dial stays mounted on the housing side",
    )

    with ctx.pose({lid_joint: lid_limits.upper}):
        closed_handle = None
        open_handle = _aabb_center(ctx.part_element_world_aabb(shell_upper, elem="handle_grip"))
        if open_handle is not None:
            closed_handle = open_handle

    closed_handle = _aabb_center(ctx.part_element_world_aabb(shell_upper, elem="handle_grip"))
    with ctx.pose({lid_joint: lid_limits.upper}):
        open_handle = _aabb_center(ctx.part_element_world_aabb(shell_upper, elem="handle_grip"))
    ctx.check(
        "lid handle rises when opened",
        closed_handle is not None
        and open_handle is not None
        and open_handle[2] > closed_handle[2] + 0.12
        and open_handle[0] < closed_handle[0] - 0.10,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    rest_pointer = _aabb_center(ctx.part_element_world_aabb(thermostat_dial, elem="dial_pointer"))
    with ctx.pose({dial_joint: math.pi / 2.0}):
        turned_pointer = _aabb_center(ctx.part_element_world_aabb(thermostat_dial, elem="dial_pointer"))
    ctx.check(
        "dial pointer moves around the knob axis",
        rest_pointer is not None
        and turned_pointer is not None
        and turned_pointer[0] > rest_pointer[0] + 0.007,
        details=f"rest_pointer={rest_pointer}, turned_pointer={turned_pointer}",
    )

    front_handle = _aabb_center(ctx.part_element_world_aabb(shell_upper, elem="handle_grip"))
    with ctx.pose({flip_joint: math.pi}):
        rear_handle = _aabb_center(ctx.part_element_world_aabb(shell_upper, elem="handle_grip"))
    ctx.check(
        "flip axis carries the shell through a full half turn",
        front_handle is not None
        and rear_handle is not None
        and rear_handle[0] < front_handle[0] - 0.24
        and rear_handle[2] < front_handle[2] - 0.02,
        details=f"front_handle={front_handle}, rear_handle={rear_handle}",
    )

    return ctx.report()


object_model = build_object_model()
