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


HALF_PI = math.pi / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lightweight_articulated_arm")

    aluminum = model.material("satin_aluminum", rgba=(0.72, 0.75, 0.76, 1.0))
    dark = model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    blue = model.material("blue_bearing_caps", rgba=(0.05, 0.28, 0.75, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.016, 1.0))

    shoulder_z = 0.365
    upper_len = 0.420
    fore_len = 0.340

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.165, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark,
        name="foot_plate",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=aluminum,
        name="pedestal",
    )
    base.visual(
        Box((0.130, 0.205, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.287)),
        material=dark,
        name="shoulder_block",
    )
    for cheek_name, cap_name, y in (
        ("shoulder_cheek_0", "shoulder_cap_0", -0.071),
        ("shoulder_cheek_1", "shoulder_cap_1", 0.071),
    ):
        base.visual(
            Box((0.090, 0.026, 0.170)),
            origin=Origin(xyz=(0.0, y, shoulder_z)),
            material=aluminum,
            name=cheek_name,
        )
        base.visual(
            Cylinder(radius=0.036, length=0.012),
            origin=Origin(xyz=(0.0, y * 1.25, shoulder_z), rpy=(HALF_PI, 0.0, 0.0)),
            material=blue,
            name=cap_name,
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.041, length=0.116),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=dark,
        name="shoulder_hub",
    )
    for rail_name, lug_name, y in (
        ("upper_rail_0", "elbow_lug_0", -0.038),
        ("upper_rail_1", "elbow_lug_1", 0.038),
    ):
        upper_arm.visual(
            Box((0.370, 0.018, 0.026)),
            origin=Origin(xyz=(0.205, y, 0.0)),
            material=aluminum,
            name=rail_name,
        )
        upper_arm.visual(
            Cylinder(radius=0.038, length=0.018),
            origin=Origin(xyz=(upper_len, y, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
            material=dark,
            name=lug_name,
        )
    upper_arm.visual(
        Box((0.160, 0.086, 0.010)),
        origin=Origin(xyz=(0.205, 0.0, -0.018)),
        material=blue,
        name="upper_web",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.031, length=0.058),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=dark,
        name="elbow_hub",
    )
    for rail_name, lug_name, y in (
        ("forearm_rail_0", "wrist_lug_0", -0.023),
        ("forearm_rail_1", "wrist_lug_1", 0.023),
    ):
        forearm.visual(
            Box((0.302, 0.012, 0.023)),
            origin=Origin(xyz=(0.165, y, 0.0)),
            material=aluminum,
            name=rail_name,
        )
        forearm.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(fore_len, y * 1.13, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
            material=dark,
            name=lug_name,
        )
    forearm.visual(
        Box((0.120, 0.056, 0.008)),
        origin=Origin(xyz=(0.180, 0.0, 0.0155)),
        material=blue,
        name="forearm_web",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.022, length=0.038),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=dark,
        name="wrist_hub",
    )
    wrist.visual(
        Cylinder(radius=0.012, length=0.088),
        origin=Origin(xyz=(0.054, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=aluminum,
        name="wrist_neck",
    )
    wrist.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(0.108, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=dark,
        name="tool_flange",
    )
    for suffix, y, z in (
        ("0", -0.018, -0.018),
        ("1", -0.018, 0.018),
        ("2", 0.018, -0.018),
        ("3", 0.018, 0.018),
    ):
        wrist.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(0.120, y, z), rpy=(0.0, HALF_PI, 0.0)),
            material=rubber,
            name=f"flange_bolt_{suffix}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=-0.70, upper=1.25),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(upper_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.8, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(fore_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4, lower=-1.60, upper=1.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist_joint = object_model.get_articulation("wrist")

    rotary_joints = (shoulder, elbow, wrist_joint)
    ctx.check(
        "three rotary joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in rotary_joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    for joint in rotary_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} has finite useful travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.upper - limits.lower > 1.5,
            details=f"limits={limits}",
        )

    ctx.expect_contact(
        base,
        upper_arm,
        contact_tol=0.0005,
        elem_a="shoulder_cheek_1",
        elem_b="shoulder_hub",
        name="shoulder hub clears positive yoke cheek",
    )
    ctx.expect_contact(
        upper_arm,
        base,
        contact_tol=0.0005,
        elem_a="shoulder_hub",
        elem_b="shoulder_cheek_0",
        name="shoulder hub clears negative yoke cheek",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        contact_tol=0.0005,
        elem_a="elbow_lug_1",
        elem_b="elbow_hub",
        name="elbow hub centered between lugs",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        contact_tol=0.0005,
        elem_a="elbow_hub",
        elem_b="elbow_lug_0",
        name="elbow hub clears opposite lug",
    )
    ctx.expect_contact(
        forearm,
        wrist,
        contact_tol=0.0005,
        elem_a="wrist_lug_1",
        elem_b="wrist_hub",
        name="wrist hub centered between lugs",
    )
    ctx.expect_contact(
        wrist,
        forearm,
        contact_tol=0.0005,
        elem_a="wrist_hub",
        elem_b="wrist_lug_0",
        name="wrist hub clears opposite lug",
    )

    rest_elbow_pos = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.70}):
        raised_elbow_pos = ctx.part_world_position(forearm)
    ctx.check(
        "positive shoulder raises elbow",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.20,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_wrist_pos = ctx.part_world_position(wrist)
    with ctx.pose({elbow: 0.85}):
        bent_wrist_pos = ctx.part_world_position(wrist)
    ctx.check(
        "positive elbow bends forearm upward",
        rest_wrist_pos is not None
        and bent_wrist_pos is not None
        and bent_wrist_pos[2] > rest_wrist_pos[2] + 0.20,
        details=f"rest={rest_wrist_pos}, bent={bent_wrist_pos}",
    )

    def _coord(vec, index: int, attr: str) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float(getattr(vec, attr))

    def _aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (_coord(lo, 2, "z") + _coord(hi, 2, "z"))

    rest_flange_z = _aabb_center_z(ctx.part_element_world_aabb(wrist, elem="tool_flange"))
    with ctx.pose({wrist_joint: 0.75}):
        pitched_flange_z = _aabb_center_z(ctx.part_element_world_aabb(wrist, elem="tool_flange"))
    ctx.check(
        "wrist joint pitches tool flange",
        rest_flange_z is not None
        and pitched_flange_z is not None
        and pitched_flange_z > rest_flange_z + 0.05,
        details=f"rest_z={rest_flange_z}, pitched_z={pitched_flange_z}",
    )

    return ctx.report()


object_model = build_object_model()
