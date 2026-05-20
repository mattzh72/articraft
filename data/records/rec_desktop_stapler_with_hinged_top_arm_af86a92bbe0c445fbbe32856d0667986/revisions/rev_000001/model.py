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


BASE_L = 0.43
BASE_W = 0.072
BASE_H = 0.016

MAG_L = 0.345
MAG_FLOOR_W = 0.020
MAG_OUTER_W = 0.026
MAG_FLOOR_T = 0.0025
MAG_SIDE_T = 0.003
MAG_SIDE_H = 0.010

TOP_L = 0.365


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_stapler")

    dark_body = model.material("dark_body", rgba=(0.14, 0.15, 0.17, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.08, 0.09, 0.10, 1.0))
    plated_steel = model.material("plated_steel", rgba=(0.74, 0.76, 0.80, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.67, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_L, BASE_W, BASE_H)),
        origin=Origin(xyz=(BASE_L * 0.5, 0.0, BASE_H * 0.5)),
        material=dark_body,
        name="base_shell",
    )
    base.visual(
        Box((0.070, 0.060, 0.010)),
        origin=Origin(xyz=(0.045, 0.0, 0.021)),
        material=dark_body,
        name="rear_pad",
    )
    for name, y in (("rear_riser_0", -0.023), ("rear_riser_1", 0.023)):
        base.visual(
            Box((0.024, 0.012, 0.044)),
            origin=Origin(xyz=(0.028, y, 0.038)),
            material=dark_body,
            name=name,
        )
    for name, y in (("hinge_ear_0", -0.023), ("hinge_ear_1", 0.023)):
        base.visual(
            Box((0.010, 0.012, 0.012)),
            origin=Origin(xyz=(0.029, y, 0.063)),
            material=dark_body,
            name=name,
        )
    base.visual(
        Box((0.080, 0.042, 0.008)),
        origin=Origin(xyz=(0.355, 0.0, 0.020)),
        material=dark_body,
        name="nose_deck",
    )
    base.visual(
        Box((0.022, 0.050, 0.020)),
        origin=Origin(xyz=(0.404, 0.0, 0.010)),
        material=dark_body,
        name="front_bumper",
    )

    magazine = model.part("magazine")
    magazine.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=plated_steel,
        name="hinge_barrel",
    )
    magazine.visual(
        Box((0.016, MAG_OUTER_W, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, -0.004)),
        material=plated_steel,
        name="rear_bridge",
    )
    magazine.visual(
        Box((MAG_L, MAG_FLOOR_W, MAG_FLOOR_T)),
        origin=Origin(xyz=(MAG_L * 0.5, 0.0, -0.010)),
        material=plated_steel,
        name="channel_floor",
    )
    side_x = 0.190
    side_y = MAG_FLOOR_W * 0.5 + MAG_SIDE_T * 0.5
    for name, y in (("channel_side_0", -side_y), ("channel_side_1", side_y)):
        magazine.visual(
            Box((0.310, MAG_SIDE_T, MAG_SIDE_H)),
            origin=Origin(xyz=(side_x, y, -0.004)),
            material=plated_steel,
            name=name,
        )
    magazine.visual(
        Box((0.024, 0.022, 0.010)),
        origin=Origin(xyz=(0.333, 0.0, -0.004)),
        material=plated_steel,
        name="magazine_nose",
    )

    top_arm = model.part("top_arm")
    top_arm.visual(
        Cylinder(radius=0.007, length=0.046),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_body,
        name="arm_hub",
    )
    top_arm.visual(
        Box((0.034, 0.048, 0.018)),
        origin=Origin(xyz=(0.023, 0.0, 0.004)),
        material=dark_body,
        name="rear_body",
    )
    top_arm.visual(
        Box((0.335, 0.048, 0.016)),
        origin=Origin(xyz=(0.185, 0.0, 0.006)),
        material=dark_body,
        name="arm_shell",
    )
    top_arm.visual(
        Box((0.240, 0.040, 0.004)),
        origin=Origin(xyz=(0.205, 0.0, 0.0155)),
        material=grip_rubber,
        name="grip_pad",
    )
    top_arm.visual(
        Box((0.052, 0.036, 0.022)),
        origin=Origin(xyz=(0.339, 0.0, 0.002)),
        material=dark_body,
        name="arm_nose",
    )

    follower = model.part("follower")
    follower.visual(
        Box((0.028, 0.016, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=satin_steel,
        name="follower_body",
    )
    follower.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(-0.004, 0.0, 0.005)),
        material=satin_steel,
        name="thumb_tab",
    )

    clincher = model.part("clincher")
    clincher.visual(
        Box((0.020, 0.014, 0.003)),
        material=plated_steel,
        name="clincher_plate",
    )
    clincher.visual(
        Box((0.007, 0.006, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, 0.0035)),
        material=plated_steel,
        name="selector_tab",
    )
    clincher.visual(
        Cylinder(radius=0.002, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=satin_steel,
        name="pivot_pin",
    )

    model.articulation(
        "base_to_magazine",
        ArticulationType.REVOLUTE,
        parent=base,
        child=magazine,
        origin=Origin(xyz=(0.028, 0.0, 0.061)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "magazine_to_top_arm",
        ArticulationType.REVOLUTE,
        parent=magazine,
        child=top_arm,
        origin=Origin(xyz=(0.014, 0.0, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=1.0),
    )
    model.articulation(
        "magazine_to_follower",
        ArticulationType.PRISMATIC,
        parent=magazine,
        child=follower,
        origin=Origin(xyz=(0.026, 0.0, -0.00575)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.35, lower=0.0, upper=0.215),
    )
    model.articulation(
        "base_to_clincher",
        ArticulationType.REVOLUTE,
        parent=base,
        child=clincher,
        origin=Origin(xyz=(0.356, 0.0, 0.0255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    magazine = object_model.get_part("magazine")
    top_arm = object_model.get_part("top_arm")
    follower = object_model.get_part("follower")
    clincher = object_model.get_part("clincher")

    magazine_hinge = object_model.get_articulation("base_to_magazine")
    arm_hinge = object_model.get_articulation("magazine_to_top_arm")
    follower_slide = object_model.get_articulation("magazine_to_follower")
    clincher_pivot = object_model.get_articulation("base_to_clincher")

    ctx.expect_overlap(
        magazine,
        base,
        axes="xy",
        min_overlap=0.020,
        name="closed magazine remains over the base footprint",
    )
    ctx.expect_gap(
        magazine,
        base,
        axis="z",
        positive_elem="channel_floor",
        negative_elem="rear_pad",
        min_gap=0.020,
        max_gap=0.040,
        name="closed magazine rides above the base deck",
    )
    ctx.expect_gap(
        top_arm,
        magazine,
        axis="z",
        positive_elem="arm_nose",
        negative_elem="magazine_nose",
        min_gap=0.0,
        max_gap=0.010,
        name="top arm rests slightly above the magazine channel",
    )
    ctx.expect_within(
        follower,
        magazine,
        axes="y",
        margin=0.003,
        name="follower stays centered between the magazine rails",
    )
    ctx.expect_overlap(
        follower,
        magazine,
        axes="x",
        min_overlap=0.025,
        name="follower remains inserted in the magazine at rest",
    )
    ctx.expect_overlap(
        clincher,
        base,
        axes="xy",
        min_overlap=0.010,
        name="clincher sits within the stapling nose zone",
    )

    mag_limits = magazine_hinge.motion_limits
    if mag_limits is not None and mag_limits.upper is not None:
        closed_mag = ctx.part_element_world_aabb(magazine, elem="magazine_nose")
        with ctx.pose({magazine_hinge: mag_limits.upper}):
            open_mag = ctx.part_element_world_aabb(magazine, elem="magazine_nose")
            ctx.expect_gap(
                magazine,
                base,
                axis="z",
                positive_elem="magazine_nose",
                negative_elem="nose_deck",
                min_gap=0.250,
                name="opened magazine lifts well above the base",
            )
        closed_mag_center = _aabb_center(closed_mag)
        open_mag_center = _aabb_center(open_mag)
        ctx.check(
            "magazine nose swings upward",
            closed_mag_center is not None
            and open_mag_center is not None
            and open_mag_center[2] > closed_mag_center[2] + 0.12,
            details=f"closed={closed_mag_center}, open={open_mag_center}",
        )

    arm_limits = arm_hinge.motion_limits
    if arm_limits is not None and arm_limits.upper is not None:
        closed_nose = ctx.part_element_world_aabb(top_arm, elem="arm_nose")
        with ctx.pose({arm_hinge: arm_limits.upper}):
            open_nose = ctx.part_element_world_aabb(top_arm, elem="arm_nose")
            ctx.expect_gap(
                top_arm,
                magazine,
                axis="z",
                positive_elem="arm_nose",
                negative_elem="magazine_nose",
                min_gap=0.220,
                name="opened top arm clears the magazine channel",
            )
        closed_nose_center = _aabb_center(closed_nose)
        open_nose_center = _aabb_center(open_nose)
        ctx.check(
            "top arm nose rises when opened",
            closed_nose_center is not None
            and open_nose_center is not None
            and open_nose_center[2] > closed_nose_center[2] + 0.10,
            details=f"closed={closed_nose_center}, open={open_nose_center}",
        )

    slide_limits = follower_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        rest_pos = ctx.part_world_position(follower)
        with ctx.pose({follower_slide: slide_limits.upper}):
            extended_pos = ctx.part_world_position(follower)
            ctx.expect_within(
                follower,
                magazine,
                axes="y",
                margin=0.003,
                name="extended follower stays between the magazine rails",
            )
            ctx.expect_overlap(
                follower,
                magazine,
                axes="x",
                min_overlap=0.025,
                name="extended follower still remains in the channel",
            )
        ctx.check(
            "follower advances toward the stapling nose",
            rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.18,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    clincher_limits = clincher_pivot.motion_limits
    if clincher_limits is not None and clincher_limits.upper is not None:
        rest_tab = ctx.part_element_world_aabb(clincher, elem="selector_tab")
        with ctx.pose({clincher_pivot: clincher_limits.upper}):
            turned_tab = ctx.part_element_world_aabb(clincher, elem="selector_tab")
            ctx.expect_overlap(
                clincher,
                base,
                axes="xy",
                min_overlap=0.010,
                name="rotated clincher remains within the nose deck",
            )
        rest_tab_center = _aabb_center(rest_tab)
        turned_tab_center = _aabb_center(turned_tab)
        ctx.check(
            "clincher selector flips across the pivot",
            rest_tab_center is not None
            and turned_tab_center is not None
            and turned_tab_center[0] < rest_tab_center[0] - 0.015,
            details=f"rest={rest_tab_center}, turned={turned_tab_center}",
        )

    return ctx.report()


object_model = build_object_model()
