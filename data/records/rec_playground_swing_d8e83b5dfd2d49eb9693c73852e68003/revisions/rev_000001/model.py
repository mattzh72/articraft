from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

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


def _add_box(part, size, xyz, *, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(part, radius, length, xyz, *, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_swing")

    frame_steel = model.material("frame_steel", rgba=(0.30, 0.35, 0.39, 1.0))
    link_steel = model.material("link_steel", rgba=(0.19, 0.20, 0.22, 1.0))
    platform_blue = model.material("platform_blue", rgba=(0.18, 0.44, 0.72, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.47, 0.50, 0.53, 1.0))
    hand_bar_color = model.material("hand_bar_color", rgba=(0.96, 0.72, 0.20, 1.0))

    beam_z = 1.66
    beam_radius = 0.035
    beam_half_width = 0.60
    pivot_half_width = 0.27
    top_pivot_z = 1.60
    lower_pivot_z = 0.78
    foot_x = 0.42
    foot_z = 0.04

    boss_radius = 0.022
    boss_length = 0.040
    link_span = top_pivot_z - lower_pivot_z
    link_body_length = 0.780
    link_body_thickness = 0.018
    link_body_width = 0.028

    plate_thickness = 0.006
    plate_offset = boss_length / 2.0 + plate_thickness / 2.0

    platform_depth = 0.48
    pivot_spacing = 2.0 * pivot_half_width
    platform_rail_height = 0.08
    side_rail_center_y = 0.07
    right_side_rail_center_y = pivot_spacing - side_rail_center_y
    front_rail_center_x = 0.22
    back_rail_center_x = -0.22
    platform_center_y = pivot_spacing / 2.0
    cross_rail_width = 0.440
    deck_width = 0.340

    frame = model.part("frame")

    _add_cylinder(
        frame,
        radius=beam_radius,
        length=1.26,
        xyz=(0.0, 0.0, beam_z),
        rpy=(-pi / 2.0, 0.0, 0.0),
        material=frame_steel,
        name="top_beam",
    )

    leg_length = sqrt(foot_x**2 + (beam_z - beam_radius - foot_z) ** 2)
    leg_pitch = atan2(foot_x, beam_z - beam_radius - foot_z)
    for side_y, side_name in ((-beam_half_width, "left"), (beam_half_width, "right")):
        _add_box(
            frame,
            size=(0.130, 0.100, 0.120),
            xyz=(0.0, side_y, beam_z - 0.050),
            material=frame_steel,
            name=f"{side_name}_apex_block",
        )
        _add_cylinder(
            frame,
            radius=0.030,
            length=leg_length,
            xyz=(foot_x / 2.0, side_y, (foot_z + (beam_z - beam_radius)) / 2.0),
            rpy=(0.0, -leg_pitch, 0.0),
            material=frame_steel,
            name=f"{side_name}_front_leg",
        )
        _add_cylinder(
            frame,
            radius=0.030,
            length=leg_length,
            xyz=(-foot_x / 2.0, side_y, (foot_z + (beam_z - beam_radius)) / 2.0),
            rpy=(0.0, leg_pitch, 0.0),
            material=frame_steel,
            name=f"{side_name}_rear_leg",
        )
        _add_box(
            frame,
            size=(0.920, 0.090, 0.050),
            xyz=(0.0, side_y, 0.025),
            material=frame_steel,
            name=f"{side_name}_ground_skid",
        )
        _add_cylinder(
            frame,
            radius=0.045,
            length=0.040,
            xyz=(foot_x, side_y, 0.020),
            material=frame_steel,
            name=f"{side_name}_front_foot",
        )
        _add_cylinder(
            frame,
            radius=0.045,
            length=0.040,
            xyz=(-foot_x, side_y, 0.020),
            material=frame_steel,
            name=f"{side_name}_rear_foot",
        )

    hanger_center_z = top_pivot_z - 0.008
    hanger_height = 0.070
    for pivot_y, prefix in ((-pivot_half_width, "left"), (pivot_half_width, "right")):
        _add_box(
            frame,
            size=(0.050, plate_thickness, hanger_height),
            xyz=(0.0, pivot_y - plate_offset, hanger_center_z),
            material=frame_steel,
            name=f"{prefix}_hanger_outboard",
        )
        _add_box(
            frame,
            size=(0.050, plate_thickness, hanger_height),
            xyz=(0.0, pivot_y + plate_offset, hanger_center_z),
            material=frame_steel,
            name=f"{prefix}_hanger_inboard",
        )

    left_link = model.part("left_link")
    right_link = model.part("right_link")
    for link in (left_link, right_link):
        _add_box(
            link,
            size=(link_body_thickness, link_body_width, link_body_length),
            xyz=(0.0, 0.0, -link_span / 2.0),
            material=link_steel,
            name="main_bar",
        )
        _add_cylinder(
            link,
            radius=boss_radius,
            length=boss_length,
            xyz=(0.0, 0.0, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
            material=link_steel,
            name="top_boss",
        )
        _add_cylinder(
            link,
            radius=boss_radius,
            length=boss_length,
            xyz=(0.0, 0.0, -link_span),
            rpy=(-pi / 2.0, 0.0, 0.0),
            material=link_steel,
            name="lower_boss",
        )

    platform = model.part("platform")

    _add_box(
        platform,
        size=(platform_depth, 0.040, platform_rail_height),
        xyz=(0.0, side_rail_center_y, -0.045),
        material=platform_blue,
        name="left_side_rail",
    )
    _add_box(
        platform,
        size=(platform_depth, 0.040, platform_rail_height),
        xyz=(0.0, right_side_rail_center_y, -0.045),
        material=platform_blue,
        name="right_side_rail",
    )
    _add_box(
        platform,
        size=(0.040, cross_rail_width, platform_rail_height),
        xyz=(front_rail_center_x, platform_center_y, -0.045),
        material=platform_blue,
        name="front_rail",
    )
    _add_box(
        platform,
        size=(0.040, cross_rail_width, platform_rail_height),
        xyz=(back_rail_center_x, platform_center_y, -0.045),
        material=platform_blue,
        name="rear_rail",
    )
    _add_box(
        platform,
        size=(0.410, deck_width, 0.018),
        xyz=(0.0, platform_center_y, -0.012),
        material=deck_gray,
        name="deck",
    )

    for pivot_y, prefix in ((0.0, "left"), (pivot_spacing, "right")):
        _add_box(
            platform,
            size=(0.080, 0.120, 0.025),
            xyz=(0.0, pivot_y, -0.055),
            material=platform_blue,
            name=f"{prefix}_pivot_block",
        )
        _add_box(
            platform,
            size=(0.055, plate_thickness, 0.070),
            xyz=(0.0, pivot_y - plate_offset, -0.015),
            material=platform_blue,
            name=f"{prefix}_clip_outboard",
        )
        _add_box(
            platform,
            size=(0.055, plate_thickness, 0.070),
            xyz=(0.0, pivot_y + plate_offset, -0.015),
            material=platform_blue,
            name=f"{prefix}_clip_inboard",
        )

    post_height = 0.430
    post_center_z = -0.005 + post_height / 2.0
    for post_y, name in ((0.120, "left_post"), (0.420, "right_post")):
        _add_cylinder(
            platform,
            radius=0.015,
            length=post_height,
            xyz=(0.200, post_y, post_center_z),
            material=hand_bar_color,
            name=name,
        )
    _add_cylinder(
        platform,
        radius=0.016,
        length=0.300,
        xyz=(0.200, platform_center_y, -0.005 + post_height),
        rpy=(-pi / 2.0, 0.0, 0.0),
        material=hand_bar_color,
        name="hand_bar",
    )

    model.articulation(
        "frame_to_left_link",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_link,
        origin=Origin(xyz=(0.0, -pivot_half_width, top_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "frame_to_right_link",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_link,
        origin=Origin(xyz=(0.0, pivot_half_width, top_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "left_link_to_platform",
        ArticulationType.REVOLUTE,
        parent=left_link,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, -link_span)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=-1.00,
            upper=1.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_link = object_model.get_part("left_link")
    right_link = object_model.get_part("right_link")
    platform = object_model.get_part("platform")

    left_top = object_model.get_articulation("frame_to_left_link")
    right_top = object_model.get_articulation("frame_to_right_link")
    left_bottom = object_model.get_articulation("left_link_to_platform")

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

    ctx.check(
        "swing joints use beam axis",
        left_top.axis == (0.0, 1.0, 0.0)
        and right_top.axis == (0.0, 1.0, 0.0)
        and left_bottom.axis == (0.0, 1.0, 0.0),
        details="Top and lower pivots should all rotate around the beam direction (world Y).",
    )

    with ctx.pose({left_top: 0.0, right_top: 0.0, left_bottom: 0.0}):
        ctx.expect_contact(
            frame,
            left_link,
            elem_a="left_hanger_outboard",
            elem_b="top_boss",
            contact_tol=5e-4,
            name="left top boss touches outer hanger ear",
        )
        ctx.expect_contact(
            frame,
            left_link,
            elem_a="left_hanger_inboard",
            elem_b="top_boss",
            contact_tol=5e-4,
            name="left top boss touches inner hanger ear",
        )
        ctx.expect_contact(
            frame,
            right_link,
            elem_a="right_hanger_outboard",
            elem_b="top_boss",
            contact_tol=5e-4,
            name="right top boss touches outer hanger ear",
        )
        ctx.expect_contact(
            frame,
            right_link,
            elem_a="right_hanger_inboard",
            elem_b="top_boss",
            contact_tol=5e-4,
            name="right top boss touches inner hanger ear",
        )
        ctx.expect_contact(
            left_link,
            platform,
            elem_a="lower_boss",
            elem_b="left_clip_outboard",
            contact_tol=5e-4,
            name="left lower boss touches outer platform clip",
        )
        ctx.expect_contact(
            left_link,
            platform,
            elem_a="lower_boss",
            elem_b="left_clip_inboard",
            contact_tol=5e-4,
            name="left lower boss touches inner platform clip",
        )
        ctx.expect_gap(
            frame,
            platform,
            axis="z",
            positive_elem="top_beam",
            negative_elem="hand_bar",
            min_gap=0.30,
            name="hand bar stays below the top beam at rest",
        )

    with ctx.pose({left_top: 0.32, right_top: 0.32, left_bottom: -0.32}):
        ctx.expect_contact(
            right_link,
            platform,
            elem_a="lower_boss",
            elem_b="right_clip_outboard",
            contact_tol=1e-3,
            name="right lower boss stays clipped at swing pose outer plate",
        )
        ctx.expect_contact(
            right_link,
            platform,
            elem_a="lower_boss",
            elem_b="right_clip_inboard",
            contact_tol=1e-3,
            name="right lower boss stays clipped at swing pose inner plate",
        )
        ctx.expect_gap(
            frame,
            platform,
            axis="z",
            positive_elem="top_beam",
            negative_elem="hand_bar",
            min_gap=0.25,
            name="hand bar stays below the top beam while swung",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
