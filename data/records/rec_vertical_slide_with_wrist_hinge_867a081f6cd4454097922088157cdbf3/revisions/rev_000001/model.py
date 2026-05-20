from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_lift_wrist")

    frame_mat = model.material("frame_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    carriage_mat = model.material("carriage_gray", rgba=(0.61, 0.63, 0.66, 1.0))
    bracket_mat = model.material("bracket_black", rgba=(0.16, 0.17, 0.18, 1.0))

    plate_w = 0.32
    plate_d = 0.16
    plate_t = 0.018

    guide_outer_w = 0.09
    guide_d = 0.078
    roof_t = 0.018
    wall_t = 0.010
    wall_h = 0.055
    guide_inner_w = guide_outer_w - 2.0 * wall_t

    slide_w = 0.066
    slide_d = 0.052
    slide_h = 0.028
    body_w = 0.064
    body_d = 0.060
    body_h = 0.060
    yoke_w = 0.082
    yoke_d = 0.070
    yoke_h = 0.025
    ear_w = 0.050
    ear_t = 0.010
    ear_h = 0.030
    ear_gap = 0.034

    barrel_r = 0.010
    barrel_l = ear_gap
    arm_w = 0.036
    arm_d = 0.028
    arm_h = 0.050
    pad_w = 0.056
    pad_d = 0.038
    pad_t = 0.010

    roof_center_z = -(plate_t / 2.0 + roof_t / 2.0)
    wall_center_z = roof_center_z - roof_t / 2.0 - wall_h / 2.0
    pivot_z = -(slide_h + body_h + yoke_h + ear_h / 2.0)

    top_support = model.part("top_support")
    top_support.visual(
        Box((plate_w, plate_d, plate_t)),
        material=frame_mat,
        name="top_plate",
    )
    top_support.visual(
        Box((guide_outer_w, guide_d, roof_t)),
        origin=Origin(xyz=(0.0, 0.0, roof_center_z)),
        material=frame_mat,
        name="guide_roof",
    )
    top_support.visual(
        Box((wall_t, guide_d, wall_h)),
        origin=Origin(
            xyz=((guide_outer_w / 2.0) - (wall_t / 2.0), 0.0, wall_center_z)
        ),
        material=frame_mat,
        name="right_guide_cheek",
    )
    top_support.visual(
        Box((wall_t, guide_d, wall_h)),
        origin=Origin(
            xyz=(-(guide_outer_w / 2.0) + (wall_t / 2.0), 0.0, wall_center_z)
        ),
        material=frame_mat,
        name="left_guide_cheek",
    )
    top_support.visual(
        Box((guide_inner_w, wall_t, wall_h)),
        origin=Origin(
            xyz=(0.0, -(guide_d / 2.0) + (wall_t / 2.0), wall_center_z)
        ),
        material=frame_mat,
        name="guide_back",
    )
    top_support.inertial = Inertial.from_geometry(
        Box((plate_w, plate_d, 0.11)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((slide_w, slide_d, slide_h)),
        origin=Origin(xyz=(0.0, 0.0, -slide_h / 2.0)),
        material=carriage_mat,
        name="slider_head",
    )
    carriage.visual(
        Box((body_w, body_d, body_h)),
        origin=Origin(xyz=(0.0, 0.0, -(slide_h + body_h / 2.0))),
        material=carriage_mat,
        name="carriage_body",
    )
    carriage.visual(
        Box((yoke_w, yoke_d, yoke_h)),
        origin=Origin(xyz=(0.0, 0.0, -(slide_h + body_h + yoke_h / 2.0))),
        material=carriage_mat,
        name="yoke_block",
    )
    carriage.visual(
        Box((ear_w, ear_t, ear_h)),
        origin=Origin(
            xyz=(0.0, (ear_gap / 2.0) + (ear_t / 2.0), pivot_z),
        ),
        material=carriage_mat,
        name="left_ear",
    )
    carriage.visual(
        Box((ear_w, ear_t, ear_h)),
        origin=Origin(
            xyz=(0.0, -((ear_gap / 2.0) + (ear_t / 2.0)), pivot_z),
        ),
        material=carriage_mat,
        name="right_ear",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((yoke_w, yoke_d, slide_h + body_h + yoke_h + ear_h)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.0715)),
    )

    wrist_bracket = model.part("wrist_bracket")
    wrist_bracket.visual(
        Cylinder(radius=barrel_r, length=barrel_l),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bracket_mat,
        name="pivot_barrel",
    )
    wrist_bracket.visual(
        Box((arm_w, arm_d, arm_h)),
        origin=Origin(xyz=(arm_w / 2.0, 0.0, -arm_h / 2.0)),
        material=bracket_mat,
        name="wrist_arm",
    )
    wrist_bracket.visual(
        Box((pad_w, pad_d, pad_t)),
        origin=Origin(xyz=(pad_w / 2.0, 0.0, -(arm_h + pad_t / 2.0))),
        material=bracket_mat,
        name="tool_pad",
    )
    wrist_bracket.inertial = Inertial.from_geometry(
        Box((pad_w, pad_d, arm_h + pad_t)),
        mass=0.45,
        origin=Origin(xyz=(0.028, 0.0, -0.03)),
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, roof_center_z - roof_t / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "carriage_to_bracket",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_bracket,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    carriage = object_model.get_part("carriage")
    wrist_bracket = object_model.get_part("wrist_bracket")
    lift = object_model.get_articulation("support_to_carriage")
    wrist = object_model.get_articulation("carriage_to_bracket")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({lift: 0.0, wrist: 0.0}):
        ctx.expect_contact(
            carriage,
            top_support,
            elem_a="slider_head",
            elem_b="guide_roof",
            name="slider head seats against guide roof",
        )
        ctx.expect_overlap(
            carriage,
            top_support,
            elem_a="slider_head",
            elem_b="guide_roof",
            axes="xy",
            min_overlap=0.05,
            name="slider stays centered under guide roof",
        )
        ctx.expect_contact(
            wrist_bracket,
            carriage,
            elem_a="pivot_barrel",
            elem_b="left_ear",
            name="pivot barrel bears on left clevis ear",
        )
        ctx.expect_contact(
            wrist_bracket,
            carriage,
            elem_a="pivot_barrel",
            elem_b="right_ear",
            name="pivot barrel bears on right clevis ear",
        )
        ctx.expect_origin_gap(
            top_support,
            carriage,
            axis="z",
            min_gap=0.02,
            max_gap=0.04,
            name="carriage hangs directly below support",
        )
        ctx.expect_origin_gap(
            carriage,
            wrist_bracket,
            axis="z",
            min_gap=0.11,
            max_gap=0.14,
            name="wrist pivot sits below carriage head",
        )

    with ctx.pose({lift: 0.0, wrist: 0.0}):
        closed_carriage_pos = ctx.part_world_position(carriage)
        neutral_bracket_aabb = ctx.part_world_aabb(wrist_bracket)

    with ctx.pose({lift: 0.16, wrist: 0.0}):
        extended_carriage_pos = ctx.part_world_position(carriage)

    with ctx.pose({lift: 0.04, wrist: 0.3}):
        rotated_pad_aabb = ctx.part_element_world_aabb(wrist_bracket, elem="tool_pad")
        ctx.expect_gap(
            carriage,
            wrist_bracket,
            axis="z",
            positive_elem="yoke_block",
            negative_elem="wrist_arm",
            min_gap=0.001,
            name="opened wrist arm stays below yoke block",
        )

    closed_carriage_z = closed_carriage_pos[2] if closed_carriage_pos else None
    extended_carriage_z = extended_carriage_pos[2] if extended_carriage_pos else None
    ctx.check(
        "lift stroke moves carriage downward",
        closed_carriage_z is not None
        and extended_carriage_z is not None
        and extended_carriage_z < closed_carriage_z - 0.14,
        details=(
            f"expected extended carriage z below closed by at least 0.14 m, "
            f"got closed={closed_carriage_z}, extended={extended_carriage_z}"
        ),
    )

    neutral_pad_aabb = ctx.part_element_world_aabb(wrist_bracket, elem="tool_pad")
    neutral_center_x = (
        (neutral_pad_aabb[0][0] + neutral_pad_aabb[1][0]) * 0.5 if neutral_pad_aabb else None
    )
    rotated_center_x = (
        (rotated_pad_aabb[0][0] + rotated_pad_aabb[1][0]) * 0.5 if rotated_pad_aabb else None
    )
    ctx.check(
        "wrist swings bracket forward",
        neutral_center_x is not None
        and rotated_center_x is not None
        and rotated_center_x > neutral_center_x + 0.012,
        details=(
            f"expected rotated bracket center x to move forward, "
            f"got neutral={neutral_center_x}, rotated={rotated_center_x}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
