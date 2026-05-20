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
    tube_from_spline_points,
)


LOWER_ARM_LENGTH = 0.168
UPPER_LINK_LENGTH = 0.092
SHADE_LENGTH = 0.095


def _y_axis_cylinder(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def _x_axis_cylinder(radius: float, length: float, *, xyz: tuple[float, float, float]) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clip_on_task_lamp")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.67, 0.70, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    clip = model.part("clip")
    clip.visual(
        Box((0.014, 0.026, 0.040)),
        origin=Origin(xyz=(-0.008, 0.0, -0.019)),
        material=matte_black,
        name="neck",
    )
    for index, y_center in enumerate((-0.013, 0.013)):
        barrel_geom, barrel_origin = _y_axis_cylinder(0.007, 0.008)
        clip.visual(
            barrel_geom,
            origin=Origin(
                xyz=(0.0, y_center, 0.0),
                rpy=barrel_origin.rpy,
            ),
            material=steel,
            name=f"shoulder_barrel_{index}",
        )
    clip.visual(
        Box((0.126, 0.018, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, -0.035)),
        material=matte_black,
        name="upper_leaf",
    )
    clip.visual(
        Box((0.132, 0.020, 0.012)),
        origin=Origin(xyz=(0.014, 0.0, -0.065)),
        material=matte_black,
        name="lower_leaf",
    )
    clip.visual(
        Box((0.022, 0.024, 0.009)),
        origin=Origin(xyz=(0.057, 0.0, -0.043)),
        material=rubber,
        name="upper_pad",
    )
    clip.visual(
        Box((0.028, 0.026, 0.010)),
        origin=Origin(xyz=(0.061, 0.0, -0.058)),
        material=rubber,
        name="lower_pad",
    )
    clip.visual(
        Cylinder(radius=0.015, length=0.036),
        origin=Origin(xyz=(-0.014, 0.0, -0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="spring_coil",
    )
    clip.visual(
        Cylinder(radius=0.004, length=0.040),
        origin=Origin(xyz=(-0.014, 0.0, -0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="spring_pin",
    )
    clip.visual(
        Box((0.030, 0.020, 0.012)),
        origin=Origin(xyz=(-0.050, 0.0, -0.028)),
        material=matte_black,
        name="upper_handle",
    )
    clip.visual(
        Box((0.032, 0.022, 0.012)),
        origin=Origin(xyz=(-0.048, 0.0, -0.074)),
        material=matte_black,
        name="lower_handle",
    )

    lower_arm = model.part("lower_arm")
    lower_arm_path = tube_from_spline_points(
        [
            (0.012, 0.0, 0.000),
            (0.060, 0.0, 0.010),
            (0.122, 0.0, 0.020),
            (LOWER_ARM_LENGTH - 0.014, 0.0, 0.013),
        ],
        radius=0.0052,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    lower_arm.visual(
        mesh_from_geometry(lower_arm_path, "lower_arm_spine"),
        material=graphite,
        name="spine",
    )
    base_knuckle_geom, base_knuckle_origin = _y_axis_cylinder(0.0058, 0.012)
    lower_arm.visual(
        base_knuckle_geom,
        origin=Origin(rpy=base_knuckle_origin.rpy),
        material=steel,
        name="base_knuckle",
    )
    lower_arm.visual(
        Box((0.018, 0.014, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, 0.002)),
        material=graphite,
        name="base_collar",
    )
    lower_arm.visual(
        Box((0.014, 0.020, 0.018)),
        origin=Origin(xyz=(0.156, 0.0, 0.012)),
        material=graphite,
        name="elbow_block",
    )
    for index, y_center in enumerate((-0.013, 0.013)):
        elbow_barrel_geom, elbow_barrel_origin = _y_axis_cylinder(0.0055, 0.008)
        lower_arm.visual(
            elbow_barrel_geom,
            origin=Origin(
                xyz=(LOWER_ARM_LENGTH, y_center, 0.012),
                rpy=elbow_barrel_origin.rpy,
            ),
            material=steel,
            name=f"elbow_barrel_{index}",
        )

    upper_link = model.part("upper_link")
    upper_base_geom, upper_base_origin = _y_axis_cylinder(0.0050, 0.012)
    upper_link.visual(
        upper_base_geom,
        origin=Origin(rpy=upper_base_origin.rpy),
        material=steel,
        name="base_knuckle",
    )
    rod_geom, rod_origin = _x_axis_cylinder(0.0046, 0.076, xyz=(0.041, 0.0, 0.0))
    upper_link.visual(
        rod_geom,
        origin=rod_origin,
        material=graphite,
        name="rod",
    )
    upper_link.visual(
        Box((0.018, 0.014, 0.018)),
        origin=Origin(xyz=(0.079, 0.0, 0.0)),
        material=graphite,
        name="tip_block",
    )
    for index, y_center in enumerate((-0.009, 0.009)):
        tip_barrel_geom, tip_barrel_origin = _y_axis_cylinder(0.0045, 0.006)
        upper_link.visual(
            tip_barrel_geom,
            origin=Origin(
                xyz=(UPPER_LINK_LENGTH, y_center, 0.0),
                rpy=tip_barrel_origin.rpy,
            ),
            material=steel,
            name=f"tip_barrel_{index}",
        )

    shade = model.part("shade")
    shade_shell = LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.000),
            (0.023, 0.014),
            (0.034, 0.050),
            (0.050, SHADE_LENGTH),
        ],
        [
            (0.013, 0.000),
            (0.018, 0.014),
            (0.029, 0.048),
            (0.046, SHADE_LENGTH - 0.002),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    shade.visual(
        mesh_from_geometry(shade_shell, "task_lamp_shade_shell"),
        origin=Origin(xyz=(0.010, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="shade_shell",
    )
    rear_collar_geom, rear_collar_origin = _x_axis_cylinder(0.016, 0.018, xyz=(0.014, 0.0, -0.014))
    shade.visual(
        rear_collar_geom,
        origin=rear_collar_origin,
        material=graphite,
        name="rear_collar",
    )
    shade.visual(
        Box((0.012, 0.010, 0.022)),
        origin=Origin(xyz=(0.003, 0.0, -0.008)),
        material=graphite,
        name="pivot_web",
    )
    pivot_boss_geom, pivot_boss_origin = _y_axis_cylinder(0.0050, 0.010)
    shade.visual(
        pivot_boss_geom,
        origin=Origin(rpy=pivot_boss_origin.rpy),
        material=steel,
        name="pivot_boss",
    )
    socket_geom, socket_origin = _x_axis_cylinder(0.010, 0.014, xyz=(0.028, 0.0, -0.014))
    shade.visual(
        socket_geom,
        origin=socket_origin,
        material=steel,
        name="socket_body",
    )

    model.articulation(
        "clip_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=clip,
        child=lower_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.75,
            upper=1.10,
            effort=10.0,
            velocity=1.4,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_link",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_link,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.15,
            upper=1.20,
            effort=8.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "upper_link_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=shade,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.55,
            upper=0.95,
            effort=4.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clip = object_model.get_part("clip")
    shade = object_model.get_part("shade")
    shoulder = object_model.get_articulation("clip_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_link")
    shade_tilt = object_model.get_articulation("upper_link_to_shade")

    ctx.expect_origin_gap(
        shade,
        clip,
        axis="x",
        min_gap=0.22,
        max_gap=0.30,
        name="shade projects forward from the clip",
    )

    rest_pos = ctx.part_world_position(shade)
    shoulder_upper = shoulder.motion_limits.upper if shoulder.motion_limits is not None else None
    elbow_upper = elbow.motion_limits.upper if elbow.motion_limits is not None else None
    shade_down = shade_tilt.motion_limits.upper if shade_tilt.motion_limits is not None else None

    raised_pos = None
    if shoulder_upper is not None:
        with ctx.pose({shoulder: shoulder_upper}):
            raised_pos = ctx.part_world_position(shade)
    ctx.check(
        "shoulder hinge lifts the lamp head",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, shoulder_up={raised_pos}",
    )

    elbow_pos = None
    if elbow_upper is not None:
        with ctx.pose({elbow: elbow_upper}):
            elbow_pos = ctx.part_world_position(shade)
    ctx.check(
        "elbow hinge raises the shade",
        rest_pos is not None and elbow_pos is not None and elbow_pos[2] > rest_pos[2] + 0.04,
        details=f"rest={rest_pos}, elbow_up={elbow_pos}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    down_shell_aabb = None
    if shade_down is not None:
        with ctx.pose({shade_tilt: shade_down}):
            down_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    ctx.check(
        "shade hinge tilts the cone downward",
        rest_shell_aabb is not None
        and down_shell_aabb is not None
        and down_shell_aabb[0][2] < rest_shell_aabb[0][2] - 0.012,
        details=f"rest_aabb={rest_shell_aabb}, down_aabb={down_shell_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
