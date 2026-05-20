from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_field_bino_refractor_mount")

    aluminum = model.material("satin_anodized_aluminum", rgba=(0.16, 0.17, 0.18, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.025, 0.022, 0.02, 1.0))
    white = model.material("warm_white_tube_enamel", rgba=(0.88, 0.86, 0.78, 1.0))
    glass = model.material("blue_green_lens_glass", rgba=(0.18, 0.55, 0.72, 0.55))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.24, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=black,
        name="ground_disk",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.800),
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        material=aluminum,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.090, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.855)),
        material=black,
        name="lower_bearing",
    )

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.082, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=aluminum,
        name="bearing_cap",
    )
    for side, y in enumerate((-0.075, 0.075)):
        azimuth_head.visual(
            Box((0.082, 0.025, 0.220)),
            origin=Origin(xyz=(0.040, y, 0.150)),
            material=aluminum,
            name=f"side_standard_{side}",
        )
    azimuth_head.visual(
        Cylinder(radius=0.018, length=0.176),
        origin=Origin(xyz=(0.040, 0.0, 0.240), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="shoulder_pin",
    )

    arm = model.part("parallelogram_arm")
    arm.visual(
        Cylinder(radius=0.032, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="root_bushing",
    )
    arm.visual(
        Box((0.052, 0.104, 0.182)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=aluminum,
        name="root_plate",
    )
    for z, label in ((0.075, "upper_bar"), (-0.075, "lower_bar")):
        arm.visual(
            Cylinder(radius=0.018, length=0.960),
            origin=Origin(xyz=(0.500, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name=label,
        )
    arm.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(xyz=(1.000, -0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="far_bushing",
    )
    arm.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(xyz=(1.000, 0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="far_bushing_cap",
    )
    arm.visual(
        Box((0.052, 0.104, 0.182)),
        origin=Origin(xyz=(0.974, 0.0, 0.0)),
        material=aluminum,
        name="far_plate",
    )

    bridge = model.part("bridge_bar")
    bridge.visual(
        Cylinder(radius=0.018, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hinge_pin",
    )
    bridge.visual(
        Box((0.170, 0.070, 0.036)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=aluminum,
        name="mount_web",
    )
    bridge.visual(
        Cylinder(radius=0.015, length=0.590),
        origin=Origin(xyz=(0.160, 0.0, 0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="cross_bar",
    )
    for saddle_name, y in (("tube_saddle_0", -0.160), ("tube_saddle_1", 0.160)):
        bridge.visual(
            Box((0.090, 0.064, 0.050)),
            origin=Origin(xyz=(0.160, y, 0.048)),
            material=black,
            name=saddle_name,
        )

    for idx, y in enumerate((-0.160, 0.160)):
        tube = model.part(f"refractor_{idx}")
        tube.visual(
            Cylinder(radius=0.032, length=0.620),
            origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=white,
            name="main_tube",
        )
        tube.visual(
            Cylinder(radius=0.040, length=0.165),
            origin=Origin(xyz=(0.525, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name="dew_shield",
        )
        tube.visual(
            Cylinder(radius=0.027, length=0.008),
            origin=Origin(xyz=(0.607, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name="front_lens",
        )
        tube.visual(
            Cylinder(radius=0.024, length=0.125),
            origin=Origin(xyz=(-0.220, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name="rear_focuser",
        )
        tube.visual(
            Cylinder(radius=0.018, length=0.100),
            origin=Origin(xyz=(-0.282, 0.0, 0.045)),
            material=black,
            name="eyepiece_stem",
        )
        tube.visual(
            Cylinder(radius=0.021, length=0.026),
            origin=Origin(xyz=(-0.282, 0.0, 0.108)),
            material=rubber,
            name="eyepiece_cup",
        )
        model.articulation(
            f"bridge_to_refractor_{idx}",
            ArticulationType.FIXED,
            parent=bridge,
            child=tube,
            origin=Origin(xyz=(0.160, y, 0.105)),
        )

    model.articulation(
        "base_to_azimuth",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 0.875)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.9),
    )
    model.articulation(
        "azimuth_to_arm",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=arm,
        origin=Origin(xyz=(0.040, 0.0, 0.240)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=-0.35, upper=0.65),
    )
    model.articulation(
        "arm_to_bridge",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=bridge,
        origin=Origin(xyz=(1.000, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.8, lower=-0.65, upper=0.35),
        mimic=Mimic(joint="azimuth_to_arm", multiplier=-1.0, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_to_azimuth = object_model.get_articulation("base_to_azimuth")
    arm_joint = object_model.get_articulation("azimuth_to_arm")
    bridge_joint = object_model.get_articulation("arm_to_bridge")
    azimuth_head = object_model.get_part("azimuth_head")
    arm = object_model.get_part("parallelogram_arm")
    bridge = object_model.get_part("bridge_bar")
    tube_0 = object_model.get_part("refractor_0")
    tube_1 = object_model.get_part("refractor_1")

    ctx.allow_overlap(
        azimuth_head,
        arm,
        elem_a="shoulder_pin",
        elem_b="root_bushing",
        reason="The shoulder pin is intentionally modeled as a captured solid pin inside the root bushing.",
    )
    ctx.allow_overlap(
        azimuth_head,
        arm,
        elem_a="shoulder_pin",
        elem_b="root_plate",
        reason="The root hinge plate is a simplified solid clevis region around the captured shoulder pin.",
    )
    ctx.allow_overlap(
        arm,
        bridge,
        elem_a="far_bushing",
        elem_b="hinge_pin",
        reason="The bridge hinge pin is intentionally seated through the far bushing of the parallelogram arm.",
    )
    ctx.allow_overlap(
        arm,
        bridge,
        elem_a="far_bushing_cap",
        elem_b="hinge_pin",
        reason="The bridge hinge pin is intentionally seated through the opposite far bushing cap.",
    )
    ctx.allow_overlap(
        arm,
        bridge,
        elem_a="far_plate",
        elem_b="hinge_pin",
        reason="The far hinge plate is a simplified solid receiver around the bridge hinge pin.",
    )

    ctx.check(
        "continuous azimuth bearing",
        base_to_azimuth.articulation_type == ArticulationType.CONTINUOUS,
        details=str(base_to_azimuth.articulation_type),
    )
    ctx.check(
        "bridge hinge counter-rotates with arm",
        bridge_joint.mimic is not None
        and bridge_joint.mimic.joint == "azimuth_to_arm"
        and abs(bridge_joint.mimic.multiplier + 1.0) < 1.0e-6,
        details=str(bridge_joint.mimic),
    )
    ctx.expect_origin_gap(
        tube_1,
        tube_0,
        axis="y",
        min_gap=0.30,
        max_gap=0.34,
        name="refractor tubes are laterally separated",
    )
    ctx.expect_gap(
        tube_0,
        bridge,
        axis="z",
        positive_elem="main_tube",
        negative_elem="tube_saddle_0",
        min_gap=0.0,
        max_gap=0.003,
        name="first tube rests on its bridge saddle",
    )
    ctx.expect_gap(
        tube_1,
        bridge,
        axis="z",
        positive_elem="main_tube",
        negative_elem="tube_saddle_1",
        min_gap=0.0,
        max_gap=0.003,
        name="second tube rests on its bridge saddle",
    )
    ctx.expect_within(
        azimuth_head,
        arm,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="root_bushing",
        margin=0.002,
        name="shoulder pin is captured by the root bushing",
    )
    ctx.expect_within(
        bridge,
        arm,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="far_bushing",
        margin=0.002,
        name="bridge pin is captured by the far bushing",
    )

    rest_aabb = ctx.part_element_world_aabb(tube_0, elem="main_tube")
    rest_bridge = ctx.part_world_position(bridge)
    with ctx.pose({arm_joint: 0.55}):
        raised_aabb = ctx.part_element_world_aabb(tube_0, elem="main_tube")
        raised_bridge = ctx.part_world_position(bridge)
        if rest_aabb is not None and raised_aabb is not None:
            rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
            raised_dx = raised_aabb[1][0] - raised_aabb[0][0]
            raised_dz = raised_aabb[1][2] - raised_aabb[0][2]
            ctx.check(
                "parallelogram keeps refractor tubes level",
                raised_dx > 0.55 and raised_dz < 0.09 and abs(raised_dx - rest_dx) < 0.04,
                details=f"rest_dx={rest_dx:.3f}, raised_dx={raised_dx:.3f}, raised_dz={raised_dz:.3f}",
            )
        ctx.check(
            "arm lift raises bridge",
            rest_bridge is not None
            and raised_bridge is not None
            and raised_bridge[2] > rest_bridge[2] + 0.20,
            details=f"rest={rest_bridge}, raised={raised_bridge}",
        )

    return ctx.report()


object_model = build_object_model()
