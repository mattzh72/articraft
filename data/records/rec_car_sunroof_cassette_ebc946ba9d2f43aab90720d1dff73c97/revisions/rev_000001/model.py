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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box_visual(part, name, size, xyz, material, *, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _cylinder_visual(part, name, radius, length, xyz, material, *, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panoramic_moonroof_cassette")

    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.16, 0.18, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.18, 0.24, 0.29, 0.45))
    blind_fabric = model.material("blind_fabric", rgba=(0.19, 0.20, 0.18, 1.0))

    outer_w = 1.16
    outer_l = 1.30
    frame_h = 0.072
    side_w = 0.10
    cross_t = 0.10

    side_x = outer_w / 2.0 - side_w / 2.0
    cross_w = outer_w - 2.0 * side_w + 0.010

    glass_guide_w = 0.032
    glass_guide_l = 1.06
    glass_guide_h = 0.012
    glass_guide_x = outer_w / 2.0 - side_w - glass_guide_w / 2.0 + 0.003
    glass_guide_z = 0.046

    blind_track_w = 0.030
    blind_track_l = 1.02
    blind_track_h = 0.012
    blind_track_x = outer_w / 2.0 - side_w - blind_track_w / 2.0 + 0.003
    blind_track_z = 0.018

    frame = model.part("frame")
    _box_visual(
        frame,
        "left_side_rail",
        (side_w, outer_l, frame_h),
        (-side_x, 0.0, frame_h / 2.0),
        aluminum,
    )
    _box_visual(
        frame,
        "right_side_rail",
        (side_w, outer_l, frame_h),
        (side_x, 0.0, frame_h / 2.0),
        aluminum,
    )
    _box_visual(
        frame,
        "front_crossmember",
        (cross_w, cross_t, frame_h),
        (0.0, outer_l / 2.0 - cross_t / 2.0, frame_h / 2.0),
        aluminum,
    )
    _box_visual(
        frame,
        "rear_crossmember",
        (cross_w, cross_t, frame_h),
        (0.0, -outer_l / 2.0 + cross_t / 2.0, frame_h / 2.0),
        aluminum,
    )
    _box_visual(
        frame,
        "left_glass_guide",
        (glass_guide_w, glass_guide_l, glass_guide_h),
        (-glass_guide_x, 0.0, glass_guide_z),
        dark_trim,
    )
    _box_visual(
        frame,
        "right_glass_guide",
        (glass_guide_w, glass_guide_l, glass_guide_h),
        (glass_guide_x, 0.0, glass_guide_z),
        dark_trim,
    )
    _box_visual(
        frame,
        "left_blind_track",
        (blind_track_w, blind_track_l, blind_track_h),
        (-blind_track_x, 0.0, blind_track_z),
        dark_trim,
    )
    _box_visual(
        frame,
        "right_blind_track",
        (blind_track_w, blind_track_l, blind_track_h),
        (blind_track_x, 0.0, blind_track_z),
        dark_trim,
    )
    _cylinder_visual(
        frame,
        "roller_housing",
        radius=0.018,
        length=0.90,
        xyz=(0.0, 0.548, 0.028),
        material=dark_trim,
        rpy=(0.0, pi / 2.0, 0.0),
    )

    glass_panel = model.part("glass_panel")
    glass_w = 0.90
    glass_l = 0.78
    glass_t = 0.006
    carriage_w = 0.028
    carriage_l = 0.70
    carriage_h = 0.013
    carriage_x = 0.462

    _box_visual(
        glass_panel,
        "glass_lite",
        (glass_w, glass_l, glass_t),
        (0.0, 0.0, 0.067),
        glass_tint,
    )
    _box_visual(
        glass_panel,
        "left_carriage",
        (carriage_w, carriage_l, carriage_h),
        (-carriage_x, 0.0, 0.0585),
        dark_trim,
    )
    _box_visual(
        glass_panel,
        "right_carriage",
        (carriage_w, carriage_l, carriage_h),
        (carriage_x, 0.0, 0.0585),
        dark_trim,
    )
    _box_visual(
        glass_panel,
        "front_runner_bridge",
        (0.82, 0.030, 0.010),
        (0.0, glass_l / 2.0 - 0.020, 0.060),
        dark_trim,
    )
    _box_visual(
        glass_panel,
        "rear_runner_bridge",
        (0.82, 0.030, 0.010),
        (0.0, -glass_l / 2.0 + 0.020, 0.060),
        dark_trim,
    )

    sun_blind = model.part("sun_blind")
    blind_w = 0.88
    blind_l = 0.72
    blind_t = 0.004
    slider_w = 0.024
    slider_l = 0.62
    slider_h = 0.006
    slider_x = 0.468

    _box_visual(
        sun_blind,
        "blind_sheet",
        (blind_w, blind_l, blind_t),
        (0.0, 0.0, 0.030),
        blind_fabric,
    )
    _box_visual(
        sun_blind,
        "pull_bar",
        (0.88, 0.028, 0.018),
        (0.0, blind_l / 2.0 - 0.012, 0.021),
        dark_trim,
    )
    _box_visual(
        sun_blind,
        "rear_hem_bar",
        (0.86, 0.020, 0.010),
        (0.0, -blind_l / 2.0 + 0.010, 0.027),
        dark_trim,
    )
    _box_visual(
        sun_blind,
        "left_slider",
        (slider_w, slider_l, slider_h),
        (-slider_x, 0.0, 0.027),
        dark_trim,
    )
    _box_visual(
        sun_blind,
        "right_slider",
        (slider_w, slider_l, slider_h),
        (slider_x, 0.0, 0.027),
        dark_trim,
    )
    _box_visual(
        sun_blind,
        "left_front_hanger",
        (0.020, 0.720, 0.004),
        (-0.448, 0.0, 0.029),
        dark_trim,
    )
    _box_visual(
        sun_blind,
        "right_front_hanger",
        (0.020, 0.720, 0.004),
        (0.448, 0.0, 0.029),
        dark_trim,
    )

    model.articulation(
        "frame_to_glass_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=glass_panel,
        origin=Origin(xyz=(0.0, 0.08, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=0.32,
        ),
    )
    model.articulation(
        "frame_to_sun_blind",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sun_blind,
        origin=Origin(xyz=(0.0, 0.08, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.40,
            lower=0.0,
            upper=0.38,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    glass_panel = object_model.get_part("glass_panel")
    sun_blind = object_model.get_part("sun_blind")
    glass_slide = object_model.get_articulation("frame_to_glass_panel")
    blind_slide = object_model.get_articulation("frame_to_sun_blind")

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
        elem_a="left_carriage",
        elem_b="left_glass_guide",
        name="left glass carriage is supported by the guide rail",
    )
    ctx.expect_contact(
        glass_panel,
        frame,
        elem_a="right_carriage",
        elem_b="right_glass_guide",
        name="right glass carriage is supported by the guide rail",
    )
    ctx.expect_contact(
        sun_blind,
        frame,
        elem_a="left_slider",
        elem_b="left_blind_track",
        name="left blind slider is supported by the interior track",
    )
    ctx.expect_contact(
        sun_blind,
        frame,
        elem_a="right_slider",
        elem_b="right_blind_track",
        name="right blind slider is supported by the interior track",
    )
    ctx.expect_gap(
        glass_panel,
        sun_blind,
        axis="z",
        positive_elem="glass_lite",
        negative_elem="blind_sheet",
        min_gap=0.028,
        max_gap=0.040,
        name="glass panel stays above the interior blind",
    )
    ctx.expect_overlap(
        glass_panel,
        frame,
        axes="x",
        elem_a="left_carriage",
        elem_b="left_glass_guide",
        min_overlap=0.020,
        name="left glass carriage remains laterally captured in the guide",
    )
    ctx.expect_overlap(
        sun_blind,
        frame,
        axes="x",
        elem_a="left_slider",
        elem_b="left_blind_track",
        min_overlap=0.010,
        name="left blind slider remains laterally captured in the track",
    )

    glass_upper = glass_slide.motion_limits.upper if glass_slide.motion_limits else None
    blind_upper = blind_slide.motion_limits.upper if blind_slide.motion_limits else None

    rest_glass_pos = ctx.part_world_position(glass_panel)
    rest_blind_pos = ctx.part_world_position(sun_blind)

    open_glass_pos = None
    static_blind_pos = None
    if glass_upper is not None:
        with ctx.pose({glass_slide: glass_upper}):
            ctx.expect_overlap(
                glass_panel,
                frame,
                axes="y",
                elem_a="left_carriage",
                elem_b="left_glass_guide",
                min_overlap=0.60,
                name="left glass carriage retains insertion at full rearward travel",
            )
            ctx.expect_overlap(
                glass_panel,
                frame,
                axes="y",
                elem_a="right_carriage",
                elem_b="right_glass_guide",
                min_overlap=0.60,
                name="right glass carriage retains insertion at full rearward travel",
            )
            open_glass_pos = ctx.part_world_position(glass_panel)
            static_blind_pos = ctx.part_world_position(sun_blind)

    open_blind_pos = None
    static_glass_pos = None
    if blind_upper is not None:
        with ctx.pose({blind_slide: blind_upper}):
            ctx.expect_overlap(
                sun_blind,
                frame,
                axes="y",
                elem_a="left_slider",
                elem_b="left_blind_track",
                min_overlap=0.50,
                name="left blind slider retains insertion at full rearward travel",
            )
            ctx.expect_overlap(
                sun_blind,
                frame,
                axes="y",
                elem_a="right_slider",
                elem_b="right_blind_track",
                min_overlap=0.50,
                name="right blind slider retains insertion at full rearward travel",
            )
            open_blind_pos = ctx.part_world_position(sun_blind)
            static_glass_pos = ctx.part_world_position(glass_panel)

    ctx.check(
        "glass panel slides rearward",
        rest_glass_pos is not None
        and open_glass_pos is not None
        and open_glass_pos[1] < rest_glass_pos[1] - 0.25,
        details=f"rest={rest_glass_pos}, open={open_glass_pos}",
    )
    ctx.check(
        "sun blind slides rearward",
        rest_blind_pos is not None
        and open_blind_pos is not None
        and open_blind_pos[1] < rest_blind_pos[1] - 0.30,
        details=f"rest={rest_blind_pos}, open={open_blind_pos}",
    )
    ctx.check(
        "blind position is independent of the glass slide",
        rest_blind_pos is not None
        and static_blind_pos is not None
        and abs(static_blind_pos[1] - rest_blind_pos[1]) < 1e-6,
        details=f"rest={rest_blind_pos}, glass_only_pose={static_blind_pos}",
    )
    ctx.check(
        "glass position is independent of the blind slide",
        rest_glass_pos is not None
        and static_glass_pos is not None
        and abs(static_glass_pos[1] - rest_glass_pos[1]) < 1e-6,
        details=f"rest={rest_glass_pos}, blind_only_pose={static_glass_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
