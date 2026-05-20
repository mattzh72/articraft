from __future__ import annotations

from math import pi

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


Y_AXIS = (0.0, 1.0, 0.0)
CYLINDER_ALONG_Y = Origin(rpy=(-pi / 2.0, 0.0, 0.0))


def _origin_y(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=CYLINDER_ALONG_Y.rpy)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    gunmetal = Material("anodized_gunmetal", rgba=(0.16, 0.18, 0.20, 1.0))
    dark = Material("matte_black_composite", rgba=(0.03, 0.035, 0.04, 1.0))
    silver = Material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    warning = Material("service_yellow", rgba=(0.95, 0.67, 0.12, 1.0))
    rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    upper = model.part("upper_housing")
    upper.visual(
        Box((0.38, 0.42, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=gunmetal,
        name="upper_case",
    )
    upper.visual(
        Box((0.44, 0.48, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.432)),
        material=dark,
        name="top_mount_plate",
    )
    upper.visual(
        Box((0.13, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.16, 0.20)),
        material=gunmetal,
        name="hip_saddle_0",
    )
    upper.visual(
        Box((0.13, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.16, 0.20)),
        material=gunmetal,
        name="hip_saddle_1",
    )
    upper.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=_origin_y(0.0, 0.16, 0.0),
        material=gunmetal,
        name="hip_outer_boss_0",
    )
    upper.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=_origin_y(0.0, -0.16, 0.0),
        material=gunmetal,
        name="hip_outer_boss_1",
    )
    upper.visual(
        Box((0.23, 0.015, 0.035)),
        origin=Origin(xyz=(0.12, 0.211, 0.34)),
        material=warning,
        name="service_label",
    )

    thigh = model.part("thigh_link")
    thigh.visual(
        Cylinder(radius=0.13, length=0.24),
        origin=CYLINDER_ALONG_Y,
        material=silver,
        name="hip_lug",
    )
    thigh.visual(
        Box((0.15, 0.11, 0.48)),
        origin=Origin(xyz=(0.0, 0.0, -0.31)),
        material=silver,
        name="primary_beam",
    )
    thigh.visual(
        Box((0.18, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=silver,
        name="hip_neck_block",
    )
    thigh.visual(
        Box((0.12, 0.08, 0.18)),
        origin=Origin(xyz=(0.0, 0.13, -0.62)),
        material=silver,
        name="knee_fork_arm_0",
    )
    thigh.visual(
        Box((0.12, 0.08, 0.18)),
        origin=Origin(xyz=(0.0, -0.13, -0.62)),
        material=silver,
        name="knee_fork_arm_1",
    )
    thigh.visual(
        Box((0.16, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.54)),
        material=silver,
        name="knee_fork_bridge",
    )
    thigh.visual(
        Cylinder(radius=0.11, length=0.07),
        origin=_origin_y(0.0, 0.13, -0.70),
        material=silver,
        name="knee_outer_boss_0",
    )
    thigh.visual(
        Cylinder(radius=0.11, length=0.07),
        origin=_origin_y(0.0, -0.13, -0.70),
        material=silver,
        name="knee_outer_boss_1",
    )
    thigh.visual(
        Box((0.17, 0.018, 0.35)),
        origin=Origin(xyz=(0.083, 0.0, -0.34)),
        material=dark,
        name="side_cover",
    )

    shank = model.part("shank_link")
    shank.visual(
        Cylinder(radius=0.095, length=0.18),
        origin=CYLINDER_ALONG_Y,
        material=dark,
        name="knee_lug",
    )
    shank.visual(
        Box((0.12, 0.10, 0.33)),
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
        material=dark,
        name="secondary_beam",
    )
    shank.visual(
        Box((0.09, 0.05, 0.10)),
        origin=Origin(xyz=(0.0, 0.095, -0.445)),
        material=dark,
        name="ankle_fork_arm_0",
    )
    shank.visual(
        Box((0.09, 0.05, 0.10)),
        origin=Origin(xyz=(0.0, -0.095, -0.445)),
        material=dark,
        name="ankle_fork_arm_1",
    )
    shank.visual(
        Box((0.11, 0.24, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.385)),
        material=dark,
        name="ankle_fork_bridge",
    )
    shank.visual(
        Cylinder(radius=0.075, length=0.05),
        origin=_origin_y(0.0, 0.095, -0.48),
        material=dark,
        name="ankle_outer_boss_0",
    )
    shank.visual(
        Cylinder(radius=0.075, length=0.05),
        origin=_origin_y(0.0, -0.095, -0.48),
        material=dark,
        name="ankle_outer_boss_1",
    )
    shank.visual(
        Box((0.018, 0.14, 0.26)),
        origin=Origin(xyz=(-0.069, 0.0, -0.24)),
        material=warning,
        name="calibration_strip",
    )

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.055, length=0.14),
        origin=CYLINDER_ALONG_Y,
        material=silver,
        name="ankle_lug",
    )
    foot.visual(
        Box((0.08, 0.10, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        material=silver,
        name="ankle_neck",
    )
    foot.visual(
        Box((0.34, 0.18, 0.06)),
        origin=Origin(xyz=(0.12, 0.0, -0.11)),
        material=gunmetal,
        name="foot_carrier",
    )
    foot.visual(
        Box((0.37, 0.21, 0.025)),
        origin=Origin(xyz=(0.13, 0.0, -0.15)),
        material=rubber,
        name="rubber_sole",
    )
    foot.visual(
        Box((0.09, 0.19, 0.045)),
        origin=Origin(xyz=(0.29, 0.0, -0.10)),
        material=dark,
        name="toe_bumper",
    )

    model.articulation(
        "hip",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=thigh,
        origin=Origin(),
        axis=Y_AXIS,
        motion_limits=MotionLimits(effort=180.0, velocity=5.0, lower=-0.7, upper=0.8),
    )
    model.articulation(
        "knee",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.70)),
        axis=Y_AXIS,
        motion_limits=MotionLimits(effort=150.0, velocity=5.0, lower=0.0, upper=1.8),
    )
    model.articulation(
        "ankle",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.48)),
        axis=Y_AXIS,
        motion_limits=MotionLimits(effort=75.0, velocity=6.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    hip = object_model.get_articulation("hip")
    knee = object_model.get_articulation("knee")
    ankle = object_model.get_articulation("ankle")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("foot")

    lateral_revolutes = [
        joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in joint.axis) == Y_AXIS
        for joint in (hip, knee, ankle)
    ]
    ctx.check(
        "hip knee and ankle are lateral revolutes",
        all(lateral_revolutes),
        details=f"axes={[hip.axis, knee.axis, ankle.axis]}",
    )
    ctx.expect_origin_gap(
        shank,
        thigh,
        axis="z",
        min_gap=-0.90,
        max_gap=-0.55,
        name="knee sits below hip by thigh length",
    )
    ctx.expect_origin_gap(
        foot,
        shank,
        axis="z",
        min_gap=-0.62,
        max_gap=-0.35,
        name="ankle sits below knee by shorter shank length",
    )

    thigh_aabb = ctx.part_world_aabb(thigh)
    shank_aabb = ctx.part_world_aabb(shank)
    if thigh_aabb is not None and shank_aabb is not None:
        thigh_len = thigh_aabb[1][2] - thigh_aabb[0][2]
        shank_len = shank_aabb[1][2] - shank_aabb[0][2]
        ctx.check(
            "primary thigh link dominates shank link",
            thigh_len > shank_len + 0.12,
            details=f"thigh_z={thigh_len:.3f}, shank_z={shank_len:.3f}",
        )
    else:
        ctx.fail("primary thigh link dominates shank link", "missing link AABBs")

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.0, ankle: -0.35}):
        bent_foot = ctx.part_world_position(foot)
    ctx.check(
        "knee and ankle move the compact foot forward",
        rest_foot is not None and bent_foot is not None and bent_foot[0] < rest_foot[0] - 0.10,
        details=f"rest={rest_foot}, bent={bent_foot}",
    )

    return ctx.report()


object_model = build_object_model()
