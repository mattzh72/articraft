from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_tilt_moonroof_cassette")

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    aluminium = model.material("aluminium", rgba=(0.76, 0.79, 0.81, 1.0))
    anodized_dark = model.material("anodized_dark", rgba=(0.21, 0.23, 0.25, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.34, 0.46, 0.54, 0.40))
    frit_black = model.material("frit_black", rgba=(0.08, 0.08, 0.09, 1.0))
    shade_cloth = model.material("shade_cloth", rgba=(0.19, 0.20, 0.21, 1.0))
    seal_black = model.material("seal_black", rgba=(0.05, 0.05, 0.05, 1.0))

    outer_width = 0.94
    outer_length = 0.65
    inner_width = 0.74
    inner_length = 0.45
    frame_thickness = 0.028

    frame = model.part("frame")
    frame_ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_length, 0.055),
        [rounded_rect_profile(inner_width, inner_length, 0.034)],
        height=frame_thickness,
        center=True,
    ).translate(0.0, 0.0, frame_thickness * 0.5)
    frame.visual(
        save_mesh("moonroof_frame_ring", frame_ring),
        material=aluminium,
        name="frame_ring",
    )

    seal_ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.80, 0.51, 0.048),
        [rounded_rect_profile(0.72, 0.43, 0.036)],
        height=0.004,
        center=True,
    ).translate(0.0, 0.0, frame_thickness + 0.002)
    frame.visual(
        save_mesh("moonroof_seal_ring", seal_ring),
        material=seal_black,
        name="seal_ring",
    )
    frame.visual(
        Box((0.80, 0.080, 0.018)),
        origin=Origin(xyz=(0.0, -0.235, 0.006)),
        material=aluminium,
        name="front_cover",
    )
    frame.visual(
        Box((0.80, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.269, -0.004)),
        material=aluminium,
        name="front_fascia",
    )
    frame.visual(
        Box((0.030, 0.450, 0.018)),
        origin=Origin(xyz=(0.356, 0.005, -0.009)),
        material=anodized_dark,
        name="left_track",
    )
    frame.visual(
        Box((0.030, 0.450, 0.018)),
        origin=Origin(xyz=(-0.356, 0.005, -0.009)),
        material=anodized_dark,
        name="right_track",
    )
    frame.visual(
        Box((0.76, 0.050, 0.016)),
        origin=Origin(xyz=(0.0, 0.238, -0.008)),
        material=aluminium,
        name="rear_tray",
    )

    glass_panel = model.part("glass_panel")
    glass_shell = ExtrudeGeometry(
        rounded_rect_profile(0.78, 0.52, 0.050),
        0.006,
        center=True,
    )
    frit_border = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.78, 0.52, 0.050),
        [rounded_rect_profile(0.71, 0.45, 0.040)],
        height=0.0012,
        center=True,
    ).translate(0.0, 0.0, -0.0024)
    glass_panel.visual(
        save_mesh("moonroof_glass_panel", glass_shell),
        origin=Origin(xyz=(0.0, 0.248, 0.0)),
        material=glass_tint,
        name="glass_shell",
    )
    glass_panel.visual(
        save_mesh("moonroof_glass_frit", frit_border),
        origin=Origin(xyz=(0.0, 0.248, 0.0)),
        material=frit_black,
        name="frit_border",
    )
    glass_panel.visual(
        Box((0.70, 0.028, 0.009)),
        origin=Origin(xyz=(0.0, 0.004, -0.0005)),
        material=frit_black,
        name="hinge_beam",
    )
    glass_panel.visual(
        Box((0.64, 0.018, 0.005)),
        origin=Origin(xyz=(0.0, 0.499, -0.001)),
        material=frit_black,
        name="rear_edge_strip",
    )

    shade_roller = model.part("shade_roller")
    shade_roller.visual(
        Cylinder(radius=0.017, length=0.72),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_dark,
        name="roller_tube",
    )
    shade_roller.visual(
        Box((0.028, 0.045, 0.022)),
        origin=Origin(xyz=(-0.374, 0.0, 0.008)),
        material=aluminium,
        name="left_bracket",
    )
    shade_roller.visual(
        Box((0.028, 0.045, 0.022)),
        origin=Origin(xyz=(0.374, 0.0, 0.008)),
        material=aluminium,
        name="right_bracket",
    )

    sun_shade = model.part("sun_shade")
    sun_shade.visual(
        Box((0.66, 0.410, 0.002)),
        origin=Origin(xyz=(0.0, 0.075, -0.001)),
        material=shade_cloth,
        name="cloth_panel",
    )
    sun_shade.visual(
        Box((0.672, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.288, -0.006)),
        material=anodized_dark,
        name="pull_bar",
    )
    sun_shade.visual(
        Box((0.672, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.296, -0.012)),
        material=seal_black,
        name="pull_grip",
    )

    model.articulation(
        "frame_to_glass_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=glass_panel,
        origin=Origin(xyz=(0.0, -0.225, 0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(10.5),
        ),
    )
    model.articulation(
        "frame_to_shade_roller",
        ArticulationType.FIXED,
        parent=frame,
        child=shade_roller,
        origin=Origin(xyz=(0.0, -0.245, -0.022)),
    )
    model.articulation(
        "frame_to_sun_shade",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sun_shade,
        origin=Origin(xyz=(0.0, -0.195, -0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.25,
            lower=0.0,
            upper=0.13,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    glass_panel = object_model.get_part("glass_panel")
    shade_roller = object_model.get_part("shade_roller")
    sun_shade = object_model.get_part("sun_shade")
    glass_tilt = object_model.get_articulation("frame_to_glass_tilt")
    shade_slide = object_model.get_articulation("frame_to_sun_shade")

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

    ctx.expect_contact(
        glass_panel,
        frame,
        elem_a="hinge_beam",
        name="glass_hinge_beam_seats_on_front_cross_rail",
    )
    ctx.expect_contact(
        shade_roller,
        frame,
        elem_a="left_bracket",
        name="roller_left_bracket_mounts_to_frame",
    )
    ctx.expect_contact(
        shade_roller,
        frame,
        elem_a="right_bracket",
        name="roller_right_bracket_mounts_to_frame",
    )
    ctx.expect_origin_gap(
        glass_panel,
        frame,
        axis="z",
        min_gap=0.03,
        max_gap=0.04,
        name="glass_hinge_axis_is_above_frame_plane",
    )
    ctx.expect_origin_gap(
        frame,
        shade_roller,
        axis="y",
        min_gap=0.22,
        max_gap=0.27,
        name="roller_sits_on_front_frame_rail",
    )
    ctx.expect_within(
        sun_shade,
        frame,
        axes="x",
        inner_elem="cloth_panel",
        margin=0.03,
        name="shade_cloth_runs_between_side_tracks",
    )
    ctx.expect_within(
        sun_shade,
        frame,
        axes="x",
        inner_elem="pull_bar",
        margin=0.02,
        name="pull_bar_stays_between_side_tracks",
    )

    with ctx.pose({glass_tilt: 0.0}):
        ctx.expect_gap(
            glass_panel,
            frame,
            axis="z",
            positive_elem="rear_edge_strip",
            negative_elem="frame_ring",
            min_gap=0.001,
            max_gap=0.015,
            name="closed_glass_rear_edge_sits_close_to_rear_frame_ring",
        )

    glass_closed_aabb = ctx.part_world_aabb(glass_panel)
    assert glass_closed_aabb is not None
    with ctx.pose({glass_tilt: math.radians(10.5)}):
        glass_open_aabb = ctx.part_world_aabb(glass_panel)
        assert glass_open_aabb is not None
        assert glass_open_aabb[1][2] > glass_closed_aabb[1][2] + 0.07
        ctx.expect_gap(
            glass_panel,
            frame,
            axis="z",
            positive_elem="rear_edge_strip",
            min_gap=0.055,
            name="tilted_glass_rear_edge_lifts_clear_of_frame",
        )

    shade_rest_position = ctx.part_world_position(sun_shade)
    assert shade_rest_position is not None
    with ctx.pose({shade_slide: 0.13}):
        shade_extended_position = ctx.part_world_position(sun_shade)
        assert shade_extended_position is not None
        assert shade_extended_position[1] > shade_rest_position[1] + 0.12
        ctx.expect_overlap(
            sun_shade,
            frame,
            axes="x",
            elem_a="cloth_panel",
            min_overlap=0.64,
            name="extended_shade_spans_the_opening_width",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
