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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="popup_sunroof_cassette")

    # Overall cassette dimensions in meters.
    overall_length = 0.82
    overall_width = 0.52
    overall_radius = 0.030
    opening_length = 0.73
    opening_width = 0.44
    opening_radius = 0.024
    cassette_height = 0.050
    bezel_thickness = 0.004
    housing_height = cassette_height - bezel_thickness

    # Panel and guide geometry.
    panel_length = 0.71
    panel_width = 0.425
    panel_radius = 0.022
    glass_thickness = 0.005
    shoe_length = 0.060
    shoe_width = 0.012
    shoe_height = 0.006
    shoe_center_x = -0.105
    shoe_center_y = opening_width * 0.5 + 0.008
    shoe_center_z = -0.020
    hinge_x = opening_length * 0.5
    hinge_z = 0.045

    aluminum = model.material("aluminum", rgba=(0.75, 0.77, 0.79, 1.0))
    anodized = model.material("anodized_black", rgba=(0.15, 0.16, 0.17, 1.0))
    guide_polymer = model.material("guide_polymer", rgba=(0.10, 0.11, 0.12, 1.0))
    seal_black = model.material("seal_black", rgba=(0.05, 0.05, 0.06, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.18, 0.26, 0.30, 0.45))

    outer_profile = rounded_rect_profile(
        overall_length,
        overall_width,
        overall_radius,
        corner_segments=8,
    )
    opening_profile = rounded_rect_profile(
        opening_length,
        opening_width,
        opening_radius,
        corner_segments=8,
    )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((overall_length, overall_width, cassette_height)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, cassette_height * 0.5)),
    )
    frame.visual(
        _mesh(
            "sunroof_top_bezel",
            ExtrudeWithHolesGeometry(
                outer_profile,
                [opening_profile],
                height=bezel_thickness,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, cassette_height - bezel_thickness * 0.5)),
        material=aluminum,
        name="top_bezel",
    )
    frame.visual(
        _mesh(
            "sunroof_seal_ring",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(opening_length + 0.020, opening_width + 0.020, opening_radius + 0.004, corner_segments=8),
                [opening_profile],
                height=0.002,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=seal_black,
        name="seal_ring",
    )

    rail_span = overall_length - 0.090
    side_rail_width = 0.018
    front_rail_width = (overall_length - opening_length) * 0.5
    frame.visual(
        Box((front_rail_width, overall_width, housing_height)),
        origin=Origin(xyz=(overall_length * 0.5 - front_rail_width * 0.5, 0.0, housing_height * 0.5)),
        material=aluminum,
        name="front_rail",
    )
    frame.visual(
        Box((front_rail_width, overall_width, housing_height)),
        origin=Origin(xyz=(-overall_length * 0.5 + front_rail_width * 0.5, 0.0, housing_height * 0.5)),
        material=aluminum,
        name="rear_rail",
    )
    frame.visual(
        Box((rail_span, side_rail_width, housing_height)),
        origin=Origin(xyz=(0.0, overall_width * 0.5 - side_rail_width * 0.5, housing_height * 0.5)),
        material=aluminum,
        name="left_outer_rail",
    )
    frame.visual(
        Box((rail_span, side_rail_width, housing_height)),
        origin=Origin(xyz=(0.0, -overall_width * 0.5 + side_rail_width * 0.5, housing_height * 0.5)),
        material=aluminum,
        name="right_outer_rail",
    )

    # Side guide tracks are modeled as open-top aluminum channels with a low
    # floor and an inboard retaining lip. The glass-side shoe assembly hangs
    # down into this channel from the panel trim.
    track_length = 0.67
    track_floor_width = 0.020
    track_floor_thickness = 0.004
    track_floor_z = 0.020
    track_center_y = opening_width * 0.5 + 0.011
    track_lip_width = 0.010
    track_lip_thickness = 0.004
    track_lip_z = 0.030
    track_lip_center_y = track_center_y + 0.005
    wall_width = 0.004
    wall_height = 0.028
    wall_center_y = track_center_y + 0.010

    for side, sign in (("left", 1.0), ("right", -1.0)):
        frame.visual(
            Box((track_length, track_floor_width, track_floor_thickness)),
            origin=Origin(xyz=(-0.010, sign * track_center_y, track_floor_z)),
            material=guide_polymer,
            name=f"{side}_channel_floor",
        )
        frame.visual(
            Box((track_length, wall_width, wall_height)),
            origin=Origin(xyz=(-0.010, sign * wall_center_y, wall_height * 0.5)),
            material=anodized,
            name=f"{side}_channel_outer_wall",
        )
        frame.visual(
            Box((track_length, track_lip_width, track_lip_thickness)),
            origin=Origin(xyz=(-0.010, sign * track_lip_center_y, track_lip_z)),
            material=anodized,
            name=f"{side}_channel_lip",
        )

    glass_panel = model.part("glass_panel")
    glass_panel.inertial = Inertial.from_geometry(
        Box((panel_length, panel_width, 0.035)),
        mass=7.0,
        origin=Origin(xyz=(-panel_length * 0.5, 0.0, -0.006)),
    )
    panel_profile = rounded_rect_profile(panel_length, panel_width, panel_radius, corner_segments=10)
    glass_panel.visual(
        _mesh(
            "sunroof_glass_panel",
            ExtrudeGeometry.from_z0(panel_profile, glass_thickness, cap=True, closed=True),
        ),
        origin=Origin(xyz=(-panel_length * 0.5, 0.0, 0.0)),
        material=glass_tint,
        name="glass",
    )
    glass_panel.visual(
        _mesh(
            "sunroof_glass_frit",
            ExtrudeWithHolesGeometry(
                panel_profile,
                [rounded_rect_profile(panel_length - 0.058, panel_width - 0.052, max(panel_radius - 0.005, 0.006), corner_segments=8)],
                height=0.0008,
                center=True,
            ),
        ),
        origin=Origin(xyz=(-panel_length * 0.5, 0.0, glass_thickness - 0.0004)),
        material=seal_black,
        name="glass_frit",
    )

    side_trim_length = panel_length - 0.034
    side_trim_y = panel_width * 0.5 - 0.008
    glass_panel.visual(
        Box((0.032, panel_width - 0.020, 0.008)),
        origin=Origin(xyz=(-0.016, 0.0, -0.004)),
        material=anodized,
        name="front_header",
    )
    glass_panel.visual(
        Box((side_trim_length, 0.016, 0.008)),
        origin=Origin(xyz=(-panel_length * 0.5 + 0.017, side_trim_y, -0.004)),
        material=anodized,
        name="left_side_trim",
    )
    glass_panel.visual(
        Box((side_trim_length, 0.016, 0.008)),
        origin=Origin(xyz=(-panel_length * 0.5 + 0.017, -side_trim_y, -0.004)),
        material=anodized,
        name="right_side_trim",
    )
    glass_panel.visual(
        Box((0.030, panel_width - 0.020, 0.008)),
        origin=Origin(xyz=(-panel_length + 0.015, 0.0, -0.004)),
        material=anodized,
        name="rear_trim",
    )
    glass_panel.visual(
        Cylinder(radius=0.006, length=panel_width - 0.090),
        origin=Origin(xyz=(-0.008, 0.0, -0.006), rpy=(pi * 0.5, 0.0, 0.0)),
        material=anodized,
        name="hinge_barrel",
    )

    bracket_length = 0.070
    bracket_width = 0.008
    bracket_height = 0.012
    for side, sign in (("left", 1.0), ("right", -1.0)):
        glass_panel.visual(
            Box((bracket_length, 0.009, bracket_height)),
            origin=Origin(xyz=(shoe_center_x, sign * (side_trim_y + 0.0125), -0.012)),
            material=anodized,
            name=f"{side}_shoe_hanger",
        )
        glass_panel.visual(
            Box((bracket_length, bracket_width, bracket_height)),
            origin=Origin(xyz=(shoe_center_x, sign * (shoe_center_y - 0.005), -0.012)),
            material=anodized,
            name=f"{side}_shoe_bracket",
        )
        glass_panel.visual(
            Box((shoe_length, shoe_width, shoe_height)),
            origin=Origin(xyz=(shoe_center_x, sign * shoe_center_y, shoe_center_z)),
            material=guide_polymer,
            name=f"{side}_shoe",
        )

    model.articulation(
        "frame_to_glass_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=glass_panel,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.30,
            lower=0.0,
            upper=0.075,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    glass_panel = object_model.get_part("glass_panel")
    hinge = object_model.get_articulation("frame_to_glass_panel")

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

    hinge_limits = hinge.motion_limits
    ctx.check(
        "hinge_axis_runs_left_to_right",
        tuple(round(value, 6) for value in hinge.axis) == (0.0, 1.0, 0.0),
        details=f"expected hinge axis (0, 1, 0), got {hinge.axis}",
    )
    ctx.check(
        "popup_range_is_small_realistic_tilt",
        hinge_limits is not None
        and hinge_limits.lower == 0.0
        and hinge_limits.upper is not None
        and 0.05 <= hinge_limits.upper <= 0.09,
        details=f"expected popup tilt upper limit in [0.05, 0.09], got {hinge_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            glass_panel,
            frame,
            elem_a="left_shoe",
            elem_b="left_channel_floor",
            contact_tol=0.001,
            name="left_guide_shoe_seats_on_track_floor",
        )
        ctx.expect_contact(
            glass_panel,
            frame,
            elem_a="right_shoe",
            elem_b="right_channel_floor",
            contact_tol=0.001,
            name="right_guide_shoe_seats_on_track_floor",
        )
        ctx.expect_gap(
            glass_panel,
            frame,
            axis="z",
            positive_elem="glass",
            negative_elem="left_channel_lip",
            min_gap=0.010,
            name="glass_clears_left_track_hardware",
        )
        ctx.expect_gap(
            glass_panel,
            frame,
            axis="z",
            positive_elem="glass",
            negative_elem="right_channel_lip",
            min_gap=0.010,
            name="glass_clears_right_track_hardware",
        )

    with ctx.pose({hinge: hinge_limits.upper if hinge_limits and hinge_limits.upper is not None else 0.075}):
        ctx.expect_gap(
            glass_panel,
            frame,
            axis="z",
            positive_elem="rear_trim",
            negative_elem="rear_rail",
            min_gap=0.035,
            name="rear_edge_lifts_above_rear_rail",
        )
        ctx.expect_overlap(
            glass_panel,
            frame,
            axes="xy",
            elem_a="left_shoe",
            elem_b="left_channel_floor",
            min_overlap=0.006,
            name="left_shoe_stays_over_track_footprint",
        )
        ctx.expect_overlap(
            glass_panel,
            frame,
            axes="xy",
            elem_a="right_shoe",
            elem_b="right_channel_floor",
            min_overlap=0.006,
            name="right_shoe_stays_over_track_footprint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
