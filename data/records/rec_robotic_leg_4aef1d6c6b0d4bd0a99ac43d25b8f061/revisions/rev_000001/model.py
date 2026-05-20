from __future__ import annotations

from math import pi

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


HIP_TO_KNEE = 0.545
KNEE_TO_ANKLE = 0.475


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_robotic_leg_module")

    model.material("cast_aluminum", rgba=(0.62, 0.65, 0.66, 1.0))
    model.material("dark_anodized", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("black_rubber", rgba=(0.018, 0.018, 0.016, 1.0))
    model.material("blue_status", rgba=(0.05, 0.28, 0.95, 1.0))
    model.material("steel_pin", rgba=(0.78, 0.80, 0.82, 1.0))

    housing = model.part("upper_housing")
    housing.visual(
        Box((0.240, 0.190, 0.150)),
        origin=Origin(xyz=(0.000, 0.000, 0.145)),
        material="dark_anodized",
        name="electronics_case",
    )
    housing.visual(
        Cylinder(radius=0.076, length=0.048),
        origin=Origin(xyz=(0.000, 0.112, 0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material="cast_aluminum",
        name="hip_drive_cap_0",
    )
    housing.visual(
        Cylinder(radius=0.076, length=0.048),
        origin=Origin(xyz=(0.000, -0.112, 0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material="cast_aluminum",
        name="hip_drive_cap_1",
    )
    housing.visual(
        Box((0.112, 0.030, 0.126)),
        origin=Origin(xyz=(0.000, 0.079, -0.010)),
        material="cast_aluminum",
        name="hip_cheek_0",
    )
    housing.visual(
        Box((0.112, 0.030, 0.126)),
        origin=Origin(xyz=(0.000, -0.079, -0.010)),
        material="cast_aluminum",
        name="hip_cheek_1",
    )
    housing.visual(
        Box((0.100, 0.186, 0.026)),
        origin=Origin(xyz=(0.000, 0.000, 0.062)),
        material="cast_aluminum",
        name="hip_upper_bridge",
    )
    housing.visual(
        Box((0.060, 0.040, 0.020)),
        origin=Origin(xyz=(0.095, 0.000, 0.229)),
        material="blue_status",
        name="status_window",
    )
    housing.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.080, 0.070, 0.222)),
        material="steel_pin",
        name="case_screw_0",
    )
    housing.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(-0.080, 0.070, 0.222)),
        material="steel_pin",
        name="case_screw_1",
    )

    thigh = model.part("thigh_link")
    thigh.visual(
        Cylinder(radius=0.048, length=0.130),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material="cast_aluminum",
        name="hip_barrel",
    )
    thigh.visual(
        Box((0.064, 0.058, 0.045)),
        origin=Origin(xyz=(0.000, 0.000, -0.056)),
        material="cast_aluminum",
        name="hip_neck",
    )
    thigh.visual(
        Box((0.078, 0.064, 0.410)),
        origin=Origin(xyz=(0.000, 0.000, -0.269)),
        material="dark_anodized",
        name="thigh_beam",
    )
    thigh.visual(
        Box((0.098, 0.164, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, -0.486)),
        material="cast_aluminum",
        name="knee_yoke_bridge",
    )
    thigh.visual(
        Box((0.092, 0.034, 0.104)),
        origin=Origin(xyz=(0.000, 0.070, -HIP_TO_KNEE)),
        material="cast_aluminum",
        name="knee_cheek_0",
    )
    thigh.visual(
        Box((0.092, 0.034, 0.104)),
        origin=Origin(xyz=(0.000, -0.070, -HIP_TO_KNEE)),
        material="cast_aluminum",
        name="knee_cheek_1",
    )
    thigh.visual(
        Cylinder(radius=0.043, length=0.034),
        origin=Origin(xyz=(0.000, 0.070, -HIP_TO_KNEE), rpy=(pi / 2.0, 0.0, 0.0)),
        material="steel_pin",
        name="knee_pin_cap_0",
    )
    thigh.visual(
        Cylinder(radius=0.043, length=0.034),
        origin=Origin(xyz=(0.000, -0.070, -HIP_TO_KNEE), rpy=(pi / 2.0, 0.0, 0.0)),
        material="steel_pin",
        name="knee_pin_cap_1",
    )

    shank = model.part("shank_link")
    shank.visual(
        Cylinder(radius=0.038, length=0.106),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material="cast_aluminum",
        name="knee_barrel",
    )
    shank.visual(
        Box((0.052, 0.050, 0.044)),
        origin=Origin(xyz=(0.000, 0.000, -0.050)),
        material="cast_aluminum",
        name="knee_neck",
    )
    shank.visual(
        Box((0.062, 0.056, 0.366)),
        origin=Origin(xyz=(0.000, 0.000, -0.250)),
        material="dark_anodized",
        name="shank_beam",
    )
    shank.visual(
        Box((0.080, 0.124, 0.040)),
        origin=Origin(xyz=(0.000, 0.000, -0.418)),
        material="cast_aluminum",
        name="ankle_yoke_bridge",
    )
    shank.visual(
        Box((0.074, 0.026, 0.096)),
        origin=Origin(xyz=(0.000, 0.052, -KNEE_TO_ANKLE)),
        material="cast_aluminum",
        name="ankle_cheek_0",
    )
    shank.visual(
        Box((0.074, 0.026, 0.096)),
        origin=Origin(xyz=(0.000, -0.052, -KNEE_TO_ANKLE)),
        material="cast_aluminum",
        name="ankle_cheek_1",
    )
    shank.visual(
        Cylinder(radius=0.031, length=0.026),
        origin=Origin(xyz=(0.000, 0.052, -KNEE_TO_ANKLE), rpy=(pi / 2.0, 0.0, 0.0)),
        material="steel_pin",
        name="ankle_pin_cap_0",
    )
    shank.visual(
        Cylinder(radius=0.031, length=0.026),
        origin=Origin(xyz=(0.000, -0.052, -KNEE_TO_ANKLE), rpy=(pi / 2.0, 0.0, 0.0)),
        material="steel_pin",
        name="ankle_pin_cap_1",
    )

    foot = model.part("ankle_foot")
    foot.visual(
        Cylinder(radius=0.027, length=0.078),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material="cast_aluminum",
        name="ankle_barrel",
    )
    foot.visual(
        Box((0.046, 0.048, 0.064)),
        origin=Origin(xyz=(0.000, 0.000, -0.047)),
        material="cast_aluminum",
        name="ankle_block",
    )
    foot.visual(
        Box((0.196, 0.108, 0.030)),
        origin=Origin(xyz=(0.054, 0.000, -0.088)),
        material="black_rubber",
        name="sole_pad",
    )
    foot.visual(
        Box((0.034, 0.104, 0.026)),
        origin=Origin(xyz=(0.158, 0.000, -0.083)),
        material="black_rubber",
        name="toe_bumper",
    )
    foot.visual(
        Box((0.042, 0.090, 0.020)),
        origin=Origin(xyz=(-0.050, 0.000, -0.080)),
        material="black_rubber",
        name="heel_bumper",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=thigh,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=4.0, lower=-0.95, upper=0.85),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.000, 0.000, -HIP_TO_KNEE)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=5.0, lower=-0.10, upper=1.75),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.000, 0.000, -KNEE_TO_ANKLE)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=6.0, lower=-0.70, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("upper_housing")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.expect_within(
        thigh,
        housing,
        axes="y",
        inner_elem="hip_barrel",
        outer_elem="hip_upper_bridge",
        margin=0.0,
        name="hip barrel sits inside upper yoke span",
    )
    ctx.expect_gap(
        thigh,
        housing,
        axis="y",
        positive_elem="hip_barrel",
        negative_elem="hip_cheek_1",
        max_gap=0.004,
        max_penetration=0.002,
        name="hip barrel clears lower-side cheek",
    )
    ctx.expect_gap(
        housing,
        thigh,
        axis="y",
        positive_elem="hip_cheek_0",
        negative_elem="hip_barrel",
        max_gap=0.004,
        max_penetration=0.002,
        name="hip barrel clears upper-side cheek",
    )
    ctx.expect_within(
        shank,
        thigh,
        axes="y",
        inner_elem="knee_barrel",
        outer_elem="knee_yoke_bridge",
        margin=0.0,
        name="knee barrel sits between thigh cheeks",
    )
    ctx.expect_gap(
        thigh,
        shank,
        axis="z",
        positive_elem="knee_yoke_bridge",
        negative_elem="knee_barrel",
        min_gap=0.001,
        max_gap=0.020,
        name="knee yoke bridge clears shank barrel",
    )
    ctx.expect_within(
        foot,
        shank,
        axes="y",
        inner_elem="ankle_barrel",
        outer_elem="ankle_yoke_bridge",
        margin=0.0,
        name="ankle barrel sits between shank cheeks",
    )
    ctx.expect_gap(
        shank,
        foot,
        axis="z",
        positive_elem="ankle_yoke_bridge",
        negative_elem="ankle_barrel",
        min_gap=0.004,
        max_gap=0.025,
        name="ankle yoke bridge clears compact foot barrel",
    )

    foot_aabb = ctx.part_world_aabb(foot)
    thigh_aabb = ctx.part_world_aabb(thigh)
    if foot_aabb is not None and thigh_aabb is not None:
        foot_dx = foot_aabb[1][0] - foot_aabb[0][0]
        thigh_dz = thigh_aabb[1][2] - thigh_aabb[0][2]
        ctx.check(
            "end section is compact",
            foot_dx < 0.45 * thigh_dz,
            details=f"foot_dx={foot_dx:.3f}, thigh_dz={thigh_dz:.3f}",
        )
    else:
        ctx.fail("end section is compact", "could not measure foot/thigh bounds")

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({hip: 0.35, knee: 0.60, ankle: -0.25}):
        posed_foot = ctx.part_world_position(foot)
        ctx.expect_within(
            foot,
            shank,
            axes="y",
            inner_elem="ankle_barrel",
            outer_elem="ankle_yoke_bridge",
            margin=0.0,
            name="posed ankle remains laterally captured",
        )
    ctx.check(
        "lateral-axis joints pitch the foot forward",
        rest_foot is not None and posed_foot is not None and posed_foot[0] < rest_foot[0] - 0.08,
        details=f"rest={rest_foot}, posed={posed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
