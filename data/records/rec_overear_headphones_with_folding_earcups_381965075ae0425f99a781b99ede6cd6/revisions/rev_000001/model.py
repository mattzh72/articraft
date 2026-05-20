from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
)


def _headband_path(z_offset: float = 0.0):
    return [
        (-0.160, 0.0, 0.430 + z_offset),
        (-0.132, 0.0, 0.500 + z_offset),
        (-0.055, 0.0, 0.550 + z_offset),
        (0.000, 0.0, 0.562 + z_offset),
        (0.055, 0.0, 0.550 + z_offset),
        (0.132, 0.0, 0.500 + z_offset),
        (0.160, 0.0, 0.430 + z_offset),
    ]


def _headband_pad_path():
    return [
        (-0.115, 0.0, 0.462),
        (-0.060, 0.0, 0.524),
        (0.000, 0.0, 0.539),
        (0.060, 0.0, 0.524),
        (0.115, 0.0, 0.462),
    ]


def _add_headband(headband):
    spring_steel_band = sweep_profile_along_spline(
        _headband_path(),
        profile=rounded_rect_profile(0.064, 0.018, 0.004, corner_segments=5),
        samples_per_segment=10,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    headband.visual(
        mesh_from_geometry(spring_steel_band, "wide_headband"),
        material="brushed_steel",
        name="wide_headband",
    )

    underside_pad = sweep_profile_along_spline(
        _headband_pad_path(),
        profile=rounded_rect_profile(0.074, 0.016, 0.007, corner_segments=6),
        samples_per_segment=10,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    headband.visual(
        mesh_from_geometry(underside_pad, "headband_pad"),
        material="soft_black",
        name="headband_pad",
    )

    for x, sleeve_name, lip_name in (
        (-0.160, "left_sleeve", "left_sleeve_lip"),
        (0.160, "right_sleeve", "right_sleeve_lip"),
    ):
        headband.visual(
            Box((0.040, 0.058, 0.126)),
            origin=Origin(xyz=(x, 0.0, 0.365)),
            material="matte_black",
            name=sleeve_name,
        )
        for y, rail_name in ((-0.026, f"{lip_name}_front"), (0.026, f"{lip_name}_rear")):
            headband.visual(
                Box((0.046, 0.012, 0.014)),
                origin=Origin(xyz=(x, y, 0.306)),
                material="dark_rubber",
                name=rail_name,
            )

    for i, (x, z, h) in enumerate(((-0.108, 0.490, 0.058), (0.0, 0.550, 0.022), (0.108, 0.490, 0.058))):
        headband.visual(
            Box((0.018, 0.052, h)),
            origin=Origin(xyz=(x, 0.0, z)),
            material="soft_black",
            name=f"pad_anchor_{i}",
        )


def _add_side_arm(part):
    part.visual(
        Box((0.020, 0.013, 0.175)),
        origin=Origin(xyz=(0.0, 0.0, -0.0475)),
        material="brushed_steel",
        name="arm_blade",
    )
    part.visual(
        Box((0.026, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.144)),
        material="matte_black",
        name="hinge_stop",
    )
    for i, z in enumerate((-0.006, -0.026, -0.046, -0.066)):
        part.visual(
            Box((0.014, 0.0025, 0.003)),
            origin=Origin(xyz=(0.0, -0.00775, z)),
            material="dark_rubber",
            name=f"height_mark_{i}",
        )


def _add_yoke(part):
    part.visual(
        Cylinder(radius=0.0105, length=0.132),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="matte_black",
        name="fold_barrel",
    )
    for side_name, y in (("front", -0.061), ("rear", 0.061)):
        part.visual(
            Box((0.016, 0.012, 0.110)),
            origin=Origin(xyz=(0.0, y, -0.055)),
            material="matte_black",
            name=f"{side_name}_fork_tine",
        )
    part.visual(
        Cylinder(radius=0.006, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, -0.105), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="cup_pivot_pin",
    )
    part.visual(
        Box((0.022, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material="matte_black",
        name="barrel_bridge",
    )


def _add_earcup(part, side: float):
    cup_profile = superellipse_profile(0.126, 0.096, exponent=2.35, segments=56)
    cup_shell = ExtrudeGeometry(cup_profile, 0.044, cap=True, center=True)
    part.visual(
        mesh_from_geometry(cup_shell, f"{part.name}_cup_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_black",
        name="cup_shell",
    )

    pad_outer = superellipse_profile(0.132, 0.104, exponent=2.5, segments=56)
    pad_inner = superellipse_profile(0.078, 0.052, exponent=2.35, segments=48)
    ear_pad = ExtrudeWithHolesGeometry(pad_outer, [pad_inner], 0.019, cap=True, center=True)
    part.visual(
        mesh_from_geometry(ear_pad, f"{part.name}_ear_pad"),
        origin=Origin(xyz=(-side * 0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="soft_black",
        name="ear_pad",
    )

    outer_cap = ExtrudeGeometry(superellipse_profile(0.092, 0.070, exponent=2.4, segments=48), 0.008, cap=True, center=True)
    part.visual(
        mesh_from_geometry(outer_cap, f"{part.name}_outer_cap"),
        origin=Origin(xyz=(side * 0.0255, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="charcoal",
        name="outer_cap",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, -0.052, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="front_trunnion",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brushed_steel",
        name="rear_trunnion",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_headphone")
    model.material("matte_black", rgba=(0.015, 0.015, 0.018, 1.0))
    model.material("soft_black", rgba=(0.005, 0.005, 0.006, 1.0))
    model.material("dark_rubber", rgba=(0.030, 0.030, 0.034, 1.0))
    model.material("charcoal", rgba=(0.070, 0.074, 0.080, 1.0))
    model.material("brushed_steel", rgba=(0.640, 0.660, 0.680, 1.0))

    headband = model.part("headband")
    _add_headband(headband)

    left_arm = model.part("left_arm")
    right_arm = model.part("right_arm")
    _add_side_arm(left_arm)
    _add_side_arm(right_arm)

    left_yoke = model.part("left_yoke")
    right_yoke = model.part("right_yoke")
    _add_yoke(left_yoke)
    _add_yoke(right_yoke)

    left_earcup = model.part("left_earcup")
    right_earcup = model.part("right_earcup")
    _add_earcup(left_earcup, side=-1.0)
    _add_earcup(right_earcup, side=1.0)

    slide_limits = MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.055)
    fold_limits = MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.18)
    cup_tilt_limits = MotionLimits(effort=3.0, velocity=2.5, lower=-0.32, upper=0.32)

    model.articulation(
        "left_height_slide",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=left_arm,
        origin=Origin(xyz=(-0.160, 0.0, 0.365)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=slide_limits,
    )
    model.articulation(
        "right_height_slide",
        ArticulationType.PRISMATIC,
        parent=headband,
        child=right_arm,
        origin=Origin(xyz=(0.160, 0.0, 0.365)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=slide_limits,
    )

    model.articulation(
        "left_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=left_arm,
        child=left_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=fold_limits,
    )
    model.articulation(
        "right_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=right_arm,
        child=right_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=fold_limits,
    )

    model.articulation(
        "left_cup_tilt",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=cup_tilt_limits,
    )
    model.articulation(
        "right_cup_tilt",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=cup_tilt_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headband = object_model.get_part("headband")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")
    left_slide = object_model.get_articulation("left_height_slide")
    right_slide = object_model.get_articulation("right_height_slide")
    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")

    for arm, sleeve_name in ((left_arm, "left_sleeve"), (right_arm, "right_sleeve")):
        ctx.allow_overlap(
            headband,
            arm,
            elem_a=sleeve_name,
            elem_b="arm_blade",
            reason="The height-adjustment blade is intentionally retained inside the sliding sleeve.",
        )
        ctx.expect_within(
            arm,
            headband,
            axes="xy",
            inner_elem="arm_blade",
            outer_elem=sleeve_name,
            margin=0.002,
            name=f"{arm.name} blade is centered in its sleeve",
        )
        ctx.expect_overlap(
            arm,
            headband,
            axes="z",
            elem_a="arm_blade",
            elem_b=sleeve_name,
            min_overlap=0.070,
            name=f"{arm.name} remains inserted at minimum height",
        )

    for arm_name, yoke_name in (("left_arm", "left_yoke"), ("right_arm", "right_yoke")):
        ctx.allow_overlap(
            arm_name,
            yoke_name,
            elem_a="hinge_stop",
            elem_b="fold_barrel",
            reason="The fold barrel is intentionally captured through the side-arm hinge stop.",
        )
        ctx.expect_overlap(
            arm_name,
            yoke_name,
            axes="xyz",
            elem_a="hinge_stop",
            elem_b="fold_barrel",
            min_overlap=0.010,
            name=f"{yoke_name} barrel is captured by hinge stop",
        )

    for cup_name, yoke_name in (("left_earcup", "left_yoke"), ("right_earcup", "right_yoke")):
        ctx.allow_overlap(
            cup_name,
            yoke_name,
            elem_a="cup_shell",
            elem_b="cup_pivot_pin",
            reason="The yoke pivot pin intentionally passes through the earcup trunnion boss and shell proxy.",
        )
        ctx.expect_overlap(
            cup_name,
            yoke_name,
            axes="xyz",
            elem_a="cup_shell",
            elem_b="cup_pivot_pin",
            min_overlap=0.010,
            name=f"{cup_name} is retained on the yoke pivot pin",
        )
        for trunnion_name in ("front_trunnion", "rear_trunnion"):
            ctx.allow_overlap(
                cup_name,
                yoke_name,
                elem_a=trunnion_name,
                elem_b="cup_pivot_pin",
                reason="The pivot pin intentionally runs through the earcup trunnion bore.",
            )
            ctx.expect_overlap(
                cup_name,
                yoke_name,
                axes="xyz",
                elem_a=trunnion_name,
                elem_b="cup_pivot_pin",
                min_overlap=0.006,
                name=f"{cup_name} {trunnion_name} surrounds pivot pin",
            )

    left_rest = ctx.part_world_position(left_earcup)
    right_rest = ctx.part_world_position(right_earcup)
    arm_rest = ctx.part_world_position(left_arm)

    with ctx.pose({left_slide: 0.055, right_slide: 0.055}):
        ctx.expect_overlap(
            left_arm,
            headband,
            axes="z",
            elem_a="arm_blade",
            elem_b="left_sleeve",
            min_overlap=0.030,
            name="left arm remains retained when extended",
        )
        ctx.expect_overlap(
            right_arm,
            headband,
            axes="z",
            elem_a="arm_blade",
            elem_b="right_sleeve",
            min_overlap=0.030,
            name="right arm remains retained when extended",
        )
        arm_extended = ctx.part_world_position(left_arm)

    ctx.check(
        "height slide lowers side arms",
        arm_rest is not None and arm_extended is not None and arm_extended[2] < arm_rest[2] - 0.045,
        details=f"rest={arm_rest}, extended={arm_extended}",
    )

    with ctx.pose({left_fold: 1.05, right_fold: 1.05}):
        left_folded = ctx.part_world_position(left_earcup)
        right_folded = ctx.part_world_position(right_earcup)

    ctx.check(
        "fold hinges swing earcups inward",
        (
            left_rest is not None
            and right_rest is not None
            and left_folded is not None
            and right_folded is not None
            and left_folded[0] > left_rest[0] + 0.045
            and right_folded[0] < right_rest[0] - 0.045
        ),
        details=f"left rest/fold={left_rest}/{left_folded}, right rest/fold={right_rest}/{right_folded}",
    )

    return ctx.report()


object_model = build_object_model()
