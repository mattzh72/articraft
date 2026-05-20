from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panoramic_sunroof_cassette")

    aluminium = model.material("aluminium", rgba=(0.73, 0.75, 0.78, 1.0))
    track_finish = model.material("track_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.18, 0.26, 0.32, 0.45))
    shoe_black = model.material("shoe_black", rgba=(0.10, 0.10, 0.11, 1.0))
    shade_frame_finish = model.material("shade_frame_finish", rgba=(0.18, 0.18, 0.19, 1.0))
    shade_fabric = model.material("shade_fabric", rgba=(0.29, 0.30, 0.31, 1.0))

    frame_length = 1.70
    frame_width = 1.32
    frame_top_thickness = 0.014
    opening_length = 1.50
    opening_width = 1.12
    opening_center_x = 0.02
    rail_length = 1.56
    rail_width = 0.11
    rail_center_y = 0.605

    def offset_profile(
        profile: list[tuple[float, float]], dx: float = 0.0, dy: float = 0.0
    ) -> list[tuple[float, float]]:
        return [(x + dx, y + dy) for x, y in profile]

    cassette_frame = model.part("cassette_frame")

    outer_profile = rounded_rect_profile(frame_length, frame_width, 0.075)
    opening_profile = offset_profile(
        rounded_rect_profile(opening_length, opening_width, 0.085),
        dx=opening_center_x,
    )
    frame_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [opening_profile],
            frame_top_thickness,
            center=True,
            closed=True,
        ),
        "panoramic_sunroof_frame_plate_v2",
    )
    cassette_frame.visual(
        frame_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=aluminium,
        name="frame_plate",
    )
    cassette_frame.visual(
        Box((rail_length, rail_width, 0.082)),
        origin=Origin(xyz=(0.0, rail_center_y, 0.041)),
        material=aluminium,
        name="left_rail_housing",
    )
    cassette_frame.visual(
        Box((rail_length, rail_width, 0.082)),
        origin=Origin(xyz=(0.0, -rail_center_y, 0.041)),
        material=aluminium,
        name="right_rail_housing",
    )
    cassette_frame.visual(
        Box((rail_length, 0.050, 0.002)),
        origin=Origin(xyz=(0.0, 0.530, 0.061)),
        material=track_finish,
        name="left_glass_track",
    )
    cassette_frame.visual(
        Box((rail_length, 0.050, 0.002)),
        origin=Origin(xyz=(0.0, -0.530, 0.061)),
        material=track_finish,
        name="right_glass_track",
    )
    cassette_frame.visual(
        Box((rail_length, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.520, 0.036)),
        material=track_finish,
        name="left_shade_track",
    )
    cassette_frame.visual(
        Box((rail_length, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, -0.520, 0.036)),
        material=track_finish,
        name="right_shade_track",
    )
    cassette_frame.visual(
        Box((0.18, opening_width, 0.082)),
        origin=Origin(xyz=(-0.800, 0.0, 0.112)),
        material=aluminium,
        name="front_header",
    )
    cassette_frame.visual(
        Box((0.64, 1.12, 0.024)),
        origin=Origin(xyz=(0.520, 0.0, 0.094)),
        material=aluminium,
        name="rear_cartridge",
    )
    cassette_frame.inertial = Inertial.from_geometry(
        Box((frame_length, frame_width, 0.10)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        Box((0.98, 1.08, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=glass_tint,
        name="glass_lite",
    )
    glass_panel.visual(
        Box((0.080, 0.040, 0.010)),
        origin=Origin(xyz=(-0.360, 0.530, 0.005)),
        material=shoe_black,
        name="front_left_shoe",
    )
    glass_panel.visual(
        Box((0.080, 0.040, 0.010)),
        origin=Origin(xyz=(0.360, 0.530, 0.005)),
        material=shoe_black,
        name="rear_left_shoe",
    )
    glass_panel.visual(
        Box((0.080, 0.040, 0.010)),
        origin=Origin(xyz=(-0.360, -0.530, 0.005)),
        material=shoe_black,
        name="front_right_shoe",
    )
    glass_panel.visual(
        Box((0.080, 0.040, 0.010)),
        origin=Origin(xyz=(0.360, -0.530, 0.005)),
        material=shoe_black,
        name="rear_right_shoe",
    )
    glass_panel.visual(
        Box((0.040, 0.020, 0.012)),
        origin=Origin(xyz=(-0.360, 0.530, 0.010)),
        material=shoe_black,
        name="front_left_bracket",
    )
    glass_panel.visual(
        Box((0.040, 0.020, 0.012)),
        origin=Origin(xyz=(0.360, 0.530, 0.010)),
        material=shoe_black,
        name="rear_left_bracket",
    )
    glass_panel.visual(
        Box((0.040, 0.020, 0.012)),
        origin=Origin(xyz=(-0.360, -0.530, 0.010)),
        material=shoe_black,
        name="front_right_bracket",
    )
    glass_panel.visual(
        Box((0.040, 0.020, 0.012)),
        origin=Origin(xyz=(0.360, -0.530, 0.010)),
        material=shoe_black,
        name="rear_right_bracket",
    )
    glass_panel.inertial = Inertial.from_geometry(
        Box((0.98, 1.08, 0.020)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    sun_shade = model.part("sun_shade")
    sun_shade.visual(
        Box((0.96, 1.08, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=shade_frame_finish,
        name="shade_frame",
    )
    sun_shade.visual(
        Box((0.92, 1.04, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0015)),
        material=shade_fabric,
        name="shade_fabric",
    )
    sun_shade.visual(
        Box((0.045, 0.94, 0.016)),
        origin=Origin(xyz=(-0.457, 0.0, 0.001)),
        material=shade_frame_finish,
        name="shade_pull_bar",
    )
    sun_shade.visual(
        Box((0.10, 0.040, 0.008)),
        origin=Origin(xyz=(-0.400, 0.530, 0.004)),
        material=shoe_black,
        name="front_left_runner",
    )
    sun_shade.visual(
        Box((0.10, 0.040, 0.008)),
        origin=Origin(xyz=(0.400, 0.530, 0.004)),
        material=shoe_black,
        name="rear_left_runner",
    )
    sun_shade.visual(
        Box((0.10, 0.040, 0.008)),
        origin=Origin(xyz=(-0.400, -0.530, 0.004)),
        material=shoe_black,
        name="front_right_runner",
    )
    sun_shade.visual(
        Box((0.10, 0.040, 0.008)),
        origin=Origin(xyz=(0.400, -0.530, 0.004)),
        material=shoe_black,
        name="rear_right_runner",
    )
    sun_shade.inertial = Inertial.from_geometry(
        Box((0.96, 1.08, 0.020)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_glass_panel",
        ArticulationType.PRISMATIC,
        parent=cassette_frame,
        child=glass_panel,
        origin=Origin(xyz=(opening_center_x, 0.0, 0.062)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=0.36,
        ),
    )
    model.articulation(
        "frame_to_sun_shade",
        ArticulationType.PRISMATIC,
        parent=cassette_frame,
        child=sun_shade,
        origin=Origin(xyz=(opening_center_x, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.35,
            lower=0.0,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette_frame = object_model.get_part("cassette_frame")
    glass_panel = object_model.get_part("glass_panel")
    sun_shade = object_model.get_part("sun_shade")

    glass_slide = object_model.get_articulation("frame_to_glass_panel")
    shade_slide = object_model.get_articulation("frame_to_sun_shade")

    frame_plate = cassette_frame.get_visual("frame_plate")
    left_glass_track = cassette_frame.get_visual("left_glass_track")
    right_glass_track = cassette_frame.get_visual("right_glass_track")
    left_shade_track = cassette_frame.get_visual("left_shade_track")
    right_shade_track = cassette_frame.get_visual("right_shade_track")

    glass_lite = glass_panel.get_visual("glass_lite")
    front_left_shoe = glass_panel.get_visual("front_left_shoe")
    rear_left_shoe = glass_panel.get_visual("rear_left_shoe")
    front_right_shoe = glass_panel.get_visual("front_right_shoe")
    rear_right_shoe = glass_panel.get_visual("rear_right_shoe")

    shade_frame = sun_shade.get_visual("shade_frame")
    shade_pull_bar = sun_shade.get_visual("shade_pull_bar")
    front_left_runner = sun_shade.get_visual("front_left_runner")
    rear_left_runner = sun_shade.get_visual("rear_left_runner")
    front_right_runner = sun_shade.get_visual("front_right_runner")
    rear_right_runner = sun_shade.get_visual("rear_right_runner")

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
        cassette_frame,
        elem_a=front_left_shoe,
        elem_b=left_glass_track,
        name="glass_front_left_shoe_on_track",
    )
    ctx.expect_contact(
        glass_panel,
        cassette_frame,
        elem_a=rear_left_shoe,
        elem_b=left_glass_track,
        name="glass_rear_left_shoe_on_track",
    )
    ctx.expect_contact(
        glass_panel,
        cassette_frame,
        elem_a=front_right_shoe,
        elem_b=right_glass_track,
        name="glass_front_right_shoe_on_track",
    )
    ctx.expect_contact(
        glass_panel,
        cassette_frame,
        elem_a=rear_right_shoe,
        elem_b=right_glass_track,
        name="glass_rear_right_shoe_on_track",
    )

    ctx.expect_contact(
        sun_shade,
        cassette_frame,
        elem_a=front_left_runner,
        elem_b=left_shade_track,
        name="shade_front_left_runner_on_track",
    )
    ctx.expect_contact(
        sun_shade,
        cassette_frame,
        elem_a=rear_left_runner,
        elem_b=left_shade_track,
        name="shade_rear_left_runner_on_track",
    )
    ctx.expect_contact(
        sun_shade,
        cassette_frame,
        elem_a=front_right_runner,
        elem_b=right_shade_track,
        name="shade_front_right_runner_on_track",
    )
    ctx.expect_contact(
        sun_shade,
        cassette_frame,
        elem_a=rear_right_runner,
        elem_b=right_shade_track,
        name="shade_rear_right_runner_on_track",
    )

    ctx.expect_within(
        glass_panel,
        cassette_frame,
        axes="y",
        margin=0.0,
        name="glass_panel_within_frame_width",
    )
    ctx.expect_within(
        sun_shade,
        cassette_frame,
        axes="y",
        margin=0.0,
        name="shade_within_frame_width",
    )
    ctx.expect_gap(
        glass_panel,
        sun_shade,
        axis="z",
        positive_elem=glass_lite,
        negative_elem=shade_frame,
        min_gap=0.028,
        max_gap=0.040,
        name="shade_stays_below_glass",
    )
    ctx.expect_within(
        glass_panel,
        cassette_frame,
        axes="xy",
        inner_elem=glass_lite,
        outer_elem=frame_plate,
        margin=0.15,
        name="glass_stays_inside_cassette_envelope",
    )
    ctx.expect_within(
        sun_shade,
        cassette_frame,
        axes="xy",
        inner_elem=shade_frame,
        outer_elem=frame_plate,
        margin=0.20,
        name="shade_stays_inside_cassette_envelope",
    )

    glass_rest = ctx.part_world_position(glass_panel)
    shade_rest = ctx.part_world_position(sun_shade)
    ctx.check(
        "glass_closed_station",
        glass_rest is not None and abs(glass_rest[0] - 0.02) < 1e-6,
        f"Unexpected closed glass position: {glass_rest}",
    )
    ctx.check(
        "shade_closed_station",
        shade_rest is not None and abs(shade_rest[0] - 0.02) < 1e-6,
        f"Unexpected closed shade position: {shade_rest}",
    )

    glass_limits = glass_slide.motion_limits
    if glass_limits is not None and glass_limits.lower is not None and glass_limits.upper is not None:
        with ctx.pose({glass_slide: glass_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="glass_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="glass_lower_no_floating")
        with ctx.pose({glass_slide: glass_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="glass_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="glass_upper_no_floating")
            ctx.expect_contact(
                glass_panel,
                cassette_frame,
                elem_a=front_left_shoe,
                elem_b=left_glass_track,
                name="glass_upper_front_left_contact",
            )
            ctx.expect_contact(
                glass_panel,
                cassette_frame,
                elem_a=rear_right_shoe,
                elem_b=right_glass_track,
                name="glass_upper_rear_right_contact",
            )
            glass_open = ctx.part_world_position(glass_panel)
            ctx.check(
                "glass_moves_rearward",
                glass_rest is not None
                and glass_open is not None
                and glass_open[0] > glass_rest[0] + 0.30,
                f"Closed/open glass positions: closed={glass_rest}, open={glass_open}",
            )
            ctx.check(
                "glass_slide_stays_level",
                glass_rest is not None
                and glass_open is not None
                and abs(glass_open[1] - glass_rest[1]) < 1e-6
                and abs(glass_open[2] - glass_rest[2]) < 1e-6,
                f"Glass drifted off its rail plane: closed={glass_rest}, open={glass_open}",
            )

    shade_limits = shade_slide.motion_limits
    if shade_limits is not None and shade_limits.lower is not None and shade_limits.upper is not None:
        with ctx.pose({shade_slide: shade_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_lower_no_floating")
        with ctx.pose({shade_slide: shade_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="shade_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="shade_upper_no_floating")
            ctx.expect_contact(
                sun_shade,
                cassette_frame,
                elem_a=front_left_runner,
                elem_b=left_shade_track,
                name="shade_upper_front_left_contact",
            )
            ctx.expect_contact(
                sun_shade,
                cassette_frame,
                elem_a=rear_right_runner,
                elem_b=right_shade_track,
                name="shade_upper_rear_right_contact",
            )
            shade_open = ctx.part_world_position(sun_shade)
            ctx.check(
                "shade_moves_rearward",
                shade_rest is not None
                and shade_open is not None
                and shade_open[0] > shade_rest[0] + 0.30,
                f"Closed/open shade positions: closed={shade_rest}, open={shade_open}",
            )
            ctx.check(
                "shade_slide_stays_level",
                shade_rest is not None
                and shade_open is not None
                and abs(shade_open[1] - shade_rest[1]) < 1e-6
                and abs(shade_open[2] - shade_rest[2]) < 1e-6,
                f"Shade drifted off its rail plane: closed={shade_rest}, open={shade_open}",
            )

    if (
        glass_limits is not None
        and glass_limits.upper is not None
        and shade_limits is not None
        and shade_limits.upper is not None
    ):
        with ctx.pose({glass_slide: glass_limits.upper, shade_slide: shade_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="both_open_no_overlap")
            ctx.fail_if_isolated_parts(name="both_open_no_floating")
            ctx.expect_gap(
                glass_panel,
                sun_shade,
                axis="z",
                positive_elem=glass_lite,
                negative_elem=shade_frame,
                min_gap=0.028,
                max_gap=0.040,
                name="both_open_glass_above_shade",
            )
            ctx.expect_within(
                sun_shade,
                cassette_frame,
                axes="x",
                inner_elem=shade_pull_bar,
                outer_elem=frame_plate,
                margin=0.25,
                name="shade_pull_bar_retracted_inside_frame",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
