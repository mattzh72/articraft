from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


AXIS_Y_RPY = (math.pi / 2.0, 0.0, 0.0)


def _mat(name: str, color: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=color)


def _unit_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[tuple[float, float, float], float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("zero-length segment")
    return (dx / length, dy / length, dz / length), length


def _add_lateral_cylinder(part, name, radius, length, center, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=AXIS_Y_RPY),
        material=material,
        name=name,
    )


def _add_bar_between(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width_y: float,
    thickness: float,
    inset_start: float,
    inset_end: float,
    material: Material,
):
    direction, full_length = _unit_between(start, end)
    length = full_length - inset_start - inset_end
    if length <= 0.0:
        raise ValueError("bar insets leave no length")
    center = (
        start[0] + direction[0] * (inset_start + length / 2.0),
        start[1] + direction[1] * (inset_start + length / 2.0),
        start[2] + direction[2] * (inset_start + length / 2.0),
    )
    pitch = math.atan2(direction[0], direction[2])
    part.visual(
        Box((thickness, width_y, length)),
        origin=Origin(xyz=center, rpy=(0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def _add_yoke(
    part,
    name: str,
    joint_center: tuple[float, float, float],
    proximal_point: tuple[float, float, float],
    *,
    material: Material,
    accent: Material,
):
    direction, _ = _unit_between(joint_center, proximal_point)
    pitch = math.atan2(direction[0], direction[2])
    for side, y in (("side_0", -0.13), ("side_1", 0.13)):
        center = (
            joint_center[0] + direction[0] * 0.070,
            y,
            joint_center[2] + direction[2] * 0.070,
        )
        part.visual(
            Box((0.105, 0.045, 0.245)),
            origin=Origin(xyz=center, rpy=(0.0, pitch, 0.0)),
            material=material,
            name=f"{name}_{side}_plate",
        )
    bridge_center = (
        joint_center[0] + direction[0] * 0.145,
        0.0,
        joint_center[2] + direction[2] * 0.145,
    )
    part.visual(
        Box((0.115, 0.275, 0.070)),
        origin=Origin(xyz=bridge_center, rpy=(0.0, pitch, 0.0)),
        material=material,
        name=f"{name}_bridge",
    )
    _add_lateral_cylinder(
        part,
        f"{name}_outer_boss_0",
        0.060,
        0.045,
        (joint_center[0], -0.13, joint_center[2]),
        accent,
    )
    _add_lateral_cylinder(
        part,
        f"{name}_outer_boss_1",
        0.060,
        0.045,
        (joint_center[0], 0.13, joint_center[2]),
        accent,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    graphite = _mat("anodized_graphite", (0.08, 0.09, 0.10, 1.0))
    dark = _mat("matte_black", (0.015, 0.016, 0.018, 1.0))
    titanium = _mat("brushed_titanium", (0.58, 0.60, 0.58, 1.0))
    blue = _mat("blue_servo_accent", (0.05, 0.25, 0.85, 1.0))
    rubber = _mat("black_rubber", (0.01, 0.01, 0.009, 1.0))

    hip_housing = model.part("hip_housing")
    hip_housing.visual(
        Box((0.32, 0.32, 0.22)),
        origin=Origin(xyz=(0.015, 0.0, 0.205)),
        material=graphite,
        name="upper_motor_case",
    )
    hip_housing.visual(
        Box((0.22, 0.050, 0.270)),
        origin=Origin(xyz=(0.000, -0.130, 0.005)),
        material=graphite,
        name="hip_fork_plate_0",
    )
    hip_housing.visual(
        Box((0.22, 0.050, 0.270)),
        origin=Origin(xyz=(0.000, 0.130, 0.005)),
        material=graphite,
        name="hip_fork_plate_1",
    )
    hip_housing.visual(
        Box((0.12, 0.32, 0.050)),
        origin=Origin(xyz=(0.005, 0.0, 0.125)),
        material=graphite,
        name="hip_fork_bridge",
    )
    _add_lateral_cylinder(hip_housing, "hip_outer_boss_0", 0.078, 0.050, (0.0, -0.130, 0.0), blue)
    _add_lateral_cylinder(hip_housing, "hip_outer_boss_1", 0.078, 0.050, (0.0, 0.130, 0.0), blue)
    _add_lateral_cylinder(hip_housing, "hip_motor_cap", 0.105, 0.060, (0.015, 0.190, 0.170), dark)

    thigh = model.part("thigh_link")
    hip = (0.0, 0.0, 0.0)
    knee = (0.18, 0.0, -0.50)
    _add_lateral_cylinder(thigh, "hip_inner_hub", 0.066, 0.210, hip, titanium)
    _add_lateral_cylinder(thigh, "hip_blue_ring", 0.073, 0.020, (0.0, 0.0, 0.0), blue)
    _add_bar_between(
        thigh,
        "thigh_tapered_web",
        hip,
        knee,
        width_y=0.120,
        thickness=0.085,
        inset_start=0.055,
        inset_end=0.125,
        material=titanium,
    )
    _add_yoke(thigh, "knee_yoke", knee, hip, material=titanium, accent=blue)

    shank = model.part("shank_link")
    shank_knee = (0.0, 0.0, 0.0)
    ankle = (0.10, 0.0, -0.56)
    _add_lateral_cylinder(shank, "knee_inner_hub", 0.062, 0.215, shank_knee, titanium)
    _add_lateral_cylinder(shank, "knee_blue_ring", 0.069, 0.020, shank_knee, blue)
    _add_bar_between(
        shank,
        "shank_tapered_web",
        shank_knee,
        ankle,
        width_y=0.105,
        thickness=0.075,
        inset_start=0.055,
        inset_end=0.120,
        material=titanium,
    )
    _add_yoke(shank, "ankle_yoke", ankle, shank_knee, material=titanium, accent=blue)

    foot = model.part("ankle_foot")
    _add_lateral_cylinder(foot, "ankle_inner_hub", 0.056, 0.215, (0.0, 0.0, 0.0), titanium)
    _add_lateral_cylinder(foot, "ankle_blue_ring", 0.063, 0.020, (0.0, 0.0, 0.0), blue)
    foot.visual(
        Box((0.115, 0.130, 0.130)),
        origin=Origin(xyz=(0.035, 0.0, -0.060)),
        material=titanium,
        name="ankle_drop_block",
    )
    foot.visual(
        Box((0.390, 0.190, 0.055)),
        origin=Origin(xyz=(0.175, 0.0, -0.155)),
        material=rubber,
        name="foot_sole",
    )
    foot.visual(
        Box((0.180, 0.175, 0.035)),
        origin=Origin(xyz=(0.300, 0.0, -0.112)),
        material=dark,
        name="toe_guard",
    )
    foot.visual(
        Box((0.105, 0.175, 0.035)),
        origin=Origin(xyz=(-0.005, 0.0, -0.112)),
        material=dark,
        name="heel_guard",
    )

    model.articulation(
        "hip_joint",
        ArticulationType.REVOLUTE,
        parent=hip_housing,
        child=thigh,
        origin=Origin(xyz=hip),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=3.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "knee_joint",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=knee),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=4.0, lower=0.0, upper=1.70),
    )
    model.articulation(
        "ankle_joint",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=ankle),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=4.0, lower=-0.60, upper=0.60),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip = object_model.get_part("hip_housing")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")
    hip_joint = object_model.get_articulation("hip_joint")
    knee_joint = object_model.get_articulation("knee_joint")
    ankle_joint = object_model.get_articulation("ankle_joint")

    for joint in (hip_joint, knee_joint, ankle_joint):
        ctx.check(
            f"{joint.name} is a lateral revolute axis",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_origin_distance(
        thigh,
        shank,
        axes="xz",
        min_dist=0.50,
        max_dist=0.56,
        name="hip and knee axes are visibly spread apart",
    )
    ctx.expect_origin_distance(
        shank,
        foot,
        axes="xz",
        min_dist=0.54,
        max_dist=0.59,
        name="knee and ankle axes are visibly spread apart",
    )
    ctx.expect_origin_gap(
        hip,
        foot,
        axis="z",
        min_gap=0.90,
        name="ankle foot sits well below the upper housing",
    )

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({knee_joint: 0.55, ankle_joint: -0.25}):
        flexed_foot = ctx.part_world_position(foot)
    ctx.check(
        "knee and ankle motion changes the foot pose",
        rest_foot is not None
        and flexed_foot is not None
        and abs(flexed_foot[0] - rest_foot[0]) > 0.10,
        details=f"rest={rest_foot}, flexed={flexed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
