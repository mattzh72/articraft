from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


OVERALL_W = 1.20
OVERALL_H = 1.00
FRAME_D = 0.12
JAMB_W = 0.055
HEAD_H = 0.065
SILL_H = 0.070
TRACK_DEPTH = 0.018
TRACK_HEIGHT = 0.010
OUTER_TRACK_Y = -0.018
INNER_TRACK_Y = 0.018
TRACK_Z = 0.432
PANEL_DEPTH = 0.022
PANEL_RAIL = 0.045
PANEL_H = 0.830
FIXED_W = 0.550
SLIDING_W = 0.600
GLASS_THICKNESS = 0.006
GLASS_BITE = 0.012
SLIDE_TRAVEL = 0.440


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _make_sill_wedge(width: float, depth: float, front_h: float, back_h: float) -> MeshGeometry:
    geom = MeshGeometry()
    half_w = width * 0.5
    half_d = depth * 0.5
    points = [
        (-half_w, -half_d, 0.0),
        (half_w, -half_d, 0.0),
        (half_w, half_d, 0.0),
        (-half_w, half_d, 0.0),
        (-half_w, -half_d, front_h),
        (half_w, -half_d, front_h),
        (half_w, half_d, back_h),
        (-half_w, half_d, back_h),
    ]
    ids = [geom.add_vertex(*point) for point in points]
    _add_quad(geom, ids[0], ids[1], ids[2], ids[3])  # bottom
    _add_quad(geom, ids[4], ids[7], ids[6], ids[5])  # top
    _add_quad(geom, ids[0], ids[4], ids[5], ids[1])  # front
    _add_quad(geom, ids[3], ids[2], ids[6], ids[7])  # back
    _add_quad(geom, ids[0], ids[3], ids[7], ids[4])  # left
    _add_quad(geom, ids[1], ids[5], ids[6], ids[2])  # right
    return geom


def _add_panel_frame(
    part,
    *,
    outer_w: float,
    outer_h: float,
    depth: float,
    rail: float,
    frame_material,
    glass_material,
    seal_material,
    glass_name: str,
    bottom_capture_name: str,
    top_capture_name: str,
    meeting_side: str | None = None,
    add_handle: bool = False,
    hardware_material=None,
) -> None:
    part.visual(
        Box((outer_w, depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, outer_h * 0.5 - rail * 0.5)),
        material=frame_material,
        name="top_rail",
    )
    part.visual(
        Box((outer_w, depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h * 0.5 + rail * 0.5)),
        material=frame_material,
        name="bottom_rail",
    )
    part.visual(
        Box((rail, depth, outer_h)),
        origin=Origin(xyz=(-outer_w * 0.5 + rail * 0.5, 0.0, 0.0)),
        material=frame_material,
        name="left_stile",
    )
    part.visual(
        Box((rail, depth, outer_h)),
        origin=Origin(xyz=(outer_w * 0.5 - rail * 0.5, 0.0, 0.0)),
        material=frame_material,
        name="right_stile",
    )

    glass_w = outer_w - 2.0 * (rail - GLASS_BITE)
    glass_h = outer_h - 2.0 * (rail - GLASS_BITE)
    part.visual(
        Box((glass_w, GLASS_THICKNESS, glass_h)),
        origin=Origin(),
        material=glass_material,
        name=glass_name,
    )

    capture_w = outer_w - 0.080
    capture_d = 0.014
    capture_h = 0.012
    part.visual(
        Box((capture_w, capture_d, capture_h)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h * 0.5 - capture_h * 0.5)),
        material=seal_material,
        name=bottom_capture_name,
    )
    part.visual(
        Box((capture_w, capture_d, capture_h)),
        origin=Origin(xyz=(0.0, 0.0, outer_h * 0.5 + capture_h * 0.5)),
        material=seal_material,
        name=top_capture_name,
    )

    part.visual(
        Box((outer_w - 0.090, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, depth * 0.5 - 0.002, outer_h * 0.5 - rail + 0.004)),
        material=seal_material,
        name="head_pile_seal",
    )
    part.visual(
        Box((outer_w - 0.090, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, depth * 0.5 - 0.002, -outer_h * 0.5 + rail - 0.004)),
        material=seal_material,
        name="sill_pile_seal",
    )

    if meeting_side == "right":
        lip_x = outer_w * 0.5 - 0.009
        lip_y = depth * 0.5 - 0.003
        part.visual(
            Box((0.018, 0.006, outer_h - 0.120)),
            origin=Origin(xyz=(lip_x, lip_y, 0.0)),
            material=frame_material,
            name="meeting_lip",
        )
        part.visual(
            Box((0.004, 0.004, outer_h - 0.160)),
            origin=Origin(xyz=(outer_w * 0.5 - 0.002, depth * 0.5 - 0.002, 0.0)),
            material=seal_material,
            name="meeting_seal",
        )
    elif meeting_side == "left":
        lip_x = -outer_w * 0.5 + 0.009
        lip_y = -depth * 0.5 + 0.003
        part.visual(
            Box((0.018, 0.006, outer_h - 0.120)),
            origin=Origin(xyz=(lip_x, lip_y, 0.0)),
            material=frame_material,
            name="meeting_lip",
        )
        part.visual(
            Box((0.004, 0.004, outer_h - 0.160)),
            origin=Origin(xyz=(-outer_w * 0.5 + 0.002, -depth * 0.5 + 0.002, 0.0)),
            material=seal_material,
            name="meeting_seal",
        )

    if add_handle and hardware_material is not None:
        handle_x = -outer_w * 0.5 + 0.020
        part.visual(
            Box((0.008, 0.014, 0.026)),
            origin=Origin(xyz=(handle_x, depth * 0.5 + 0.005, 0.080)),
            material=hardware_material,
            name="latch_mount_upper",
        )
        part.visual(
            Box((0.008, 0.014, 0.026)),
            origin=Origin(xyz=(handle_x, depth * 0.5 + 0.005, -0.080)),
            material=hardware_material,
            name="latch_mount_lower",
        )
        part.visual(
            Box((0.012, 0.010, 0.200)),
            origin=Origin(xyz=(handle_x, depth * 0.5 + 0.012, 0.0)),
            material=hardware_material,
            name="latch_pull",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_sliding_window")

    aluminum = model.material("aluminum", rgba=(0.74, 0.75, 0.77, 1.0))
    anodized = model.material("anodized_frame", rgba=(0.62, 0.64, 0.67, 1.0))
    seal_black = model.material("seal_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.58, 0.73, 0.78, 0.35))
    hardware = model.material("hardware", rgba=(0.82, 0.84, 0.86, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((OVERALL_W, FRAME_D, OVERALL_H)),
        mass=28.0,
        origin=Origin(),
    )

    frame.visual(
        Box((JAMB_W, FRAME_D, OVERALL_H)),
        origin=Origin(xyz=(-OVERALL_W * 0.5 + JAMB_W * 0.5, 0.0, 0.0)),
        material=aluminum,
        name="left_jamb",
    )
    frame.visual(
        Box((JAMB_W, FRAME_D, OVERALL_H)),
        origin=Origin(xyz=(OVERALL_W * 0.5 - JAMB_W * 0.5, 0.0, 0.0)),
        material=aluminum,
        name="right_jamb",
    )
    frame.visual(
        Box((OVERALL_W, FRAME_D, HEAD_H)),
        origin=Origin(xyz=(0.0, 0.0, OVERALL_H * 0.5 - HEAD_H * 0.5)),
        material=aluminum,
        name="head",
    )
    frame.visual(
        Box((OVERALL_W, FRAME_D, SILL_H)),
        origin=Origin(xyz=(0.0, 0.0, -OVERALL_H * 0.5 + SILL_H * 0.5)),
        material=aluminum,
        name="sill_base",
    )

    frame.visual(
        Box((OVERALL_W + 0.040, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, -FRAME_D * 0.5 + 0.016, OVERALL_H * 0.5 + 0.005)),
        material=aluminum,
        name="head_rain_cap",
    )
    frame.visual(
        Box((OVERALL_W + 0.040, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -FRAME_D * 0.5 + 0.003, OVERALL_H * 0.5 - 0.003)),
        material=aluminum,
        name="head_drip_lip",
    )

    sill_wedge = mesh_from_geometry(
        _make_sill_wedge(OVERALL_W, 0.038, 0.006, 0.014),
        "window_sill_wedge",
    )
    frame.visual(
        sill_wedge,
        origin=Origin(xyz=(0.0, -0.032, -0.455)),
        material=aluminum,
        name="sill_slope",
    )
    frame.visual(
        Box((OVERALL_W, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -FRAME_D * 0.5 + 0.005, -OVERALL_H * 0.5 + 0.044)),
        material=aluminum,
        name="sill_drip_nose",
    )

    opening_w = OVERALL_W - 2.0 * JAMB_W
    frame.visual(
        Box((opening_w, TRACK_DEPTH, TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, OUTER_TRACK_Y, -TRACK_Z)),
        material=anodized,
        name="sill_outer_track",
    )
    frame.visual(
        Box((opening_w, TRACK_DEPTH, TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, OUTER_TRACK_Y, TRACK_Z)),
        material=anodized,
        name="head_outer_track",
    )
    frame.visual(
        Box((opening_w, TRACK_DEPTH, TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, INNER_TRACK_Y, -TRACK_Z)),
        material=anodized,
        name="sill_inner_track",
    )
    frame.visual(
        Box((opening_w, TRACK_DEPTH, TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, INNER_TRACK_Y, TRACK_Z)),
        material=anodized,
        name="head_inner_track",
    )
    frame.visual(
        Box((opening_w, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.421)),
        material=anodized,
        name="sill_track_divider",
    )
    frame.visual(
        Box((opening_w, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.421)),
        material=anodized,
        name="head_track_divider",
    )
    frame.visual(
        Box((0.025, 0.024, 0.720)),
        origin=Origin(xyz=(-0.5325, INNER_TRACK_Y, 0.0)),
        material=seal_black,
        name="left_open_stop",
    )
    frame.visual(
        Box((0.025, 0.024, 0.720)),
        origin=Origin(xyz=(0.5325, INNER_TRACK_Y, 0.0)),
        material=seal_black,
        name="right_closed_stop",
    )

    fixed_lite = model.part("fixed_lite")
    _add_panel_frame(
        fixed_lite,
        outer_w=FIXED_W,
        outer_h=PANEL_H,
        depth=PANEL_DEPTH,
        rail=PANEL_RAIL,
        frame_material=anodized,
        glass_material=glass,
        seal_material=seal_black,
        glass_name="fixed_glass",
        bottom_capture_name="bottom_seat",
        top_capture_name="top_seat",
        meeting_side="right",
    )
    fixed_lite.inertial = Inertial.from_geometry(
        Box((FIXED_W, PANEL_DEPTH, PANEL_H)),
        mass=13.0,
        origin=Origin(),
    )

    sliding_sash = model.part("sliding_sash")
    _add_panel_frame(
        sliding_sash,
        outer_w=SLIDING_W,
        outer_h=PANEL_H,
        depth=PANEL_DEPTH,
        rail=PANEL_RAIL,
        frame_material=aluminum,
        glass_material=glass,
        seal_material=seal_black,
        glass_name="sliding_glass",
        bottom_capture_name="bottom_guide",
        top_capture_name="top_guide",
        meeting_side="left",
        add_handle=True,
        hardware_material=hardware,
    )
    sliding_sash.visual(
        Box((0.004, 0.004, PANEL_H - 0.140)),
        origin=Origin(xyz=(SLIDING_W * 0.5 - 0.002, PANEL_DEPTH * 0.5 - 0.002, 0.0)),
        material=seal_black,
        name="jamb_seal",
    )
    sliding_sash.inertial = Inertial.from_geometry(
        Box((SLIDING_W, PANEL_DEPTH, PANEL_H)),
        mass=16.0,
        origin=Origin(),
    )

    model.articulation(
        "frame_to_fixed_lite",
        ArticulationType.FIXED,
        parent=frame,
        child=fixed_lite,
        origin=Origin(xyz=(-0.265, OUTER_TRACK_Y, 0.0)),
    )
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sliding_sash,
        origin=Origin(xyz=(0.220, INNER_TRACK_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.30,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    fixed_lite = object_model.get_part("fixed_lite")
    sliding_sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sliding_sash")

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
        fixed_lite,
        frame,
        elem_a="bottom_seat",
        elem_b="sill_outer_track",
        name="fixed_lite_bottom_is_seated_in_outer_track",
    )
    ctx.expect_contact(
        fixed_lite,
        frame,
        elem_a="top_seat",
        elem_b="head_outer_track",
        name="fixed_lite_top_is_captured_in_outer_track",
    )
    ctx.expect_within(
        fixed_lite,
        frame,
        axes="xz",
        margin=0.0,
        name="fixed_lite_stays_within_outer_frame_projection",
    )

    closed_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a="bottom_guide",
            elem_b="sill_inner_track",
            name="sliding_sash_bottom_guide_runs_on_inner_sill_track_closed",
        )
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a="top_guide",
            elem_b="head_inner_track",
            name="sliding_sash_top_guide_stays_captured_closed",
        )
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a="right_stile",
            elem_b="right_closed_stop",
            name="closed_sash_meets_right_stop",
        )
        ctx.expect_gap(
            sliding_sash,
            fixed_lite,
            axis="y",
            min_gap=0.012,
            max_gap=0.018,
            name="sliding_sash_stays_on_separate_weather_track",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_lite,
            axes="xz",
            min_overlap=0.070,
            name="closed_panels_have_believable_meeting_overlap",
        )
        ctx.expect_within(
            sliding_sash,
            frame,
            axes="xz",
            margin=0.0,
            name="closed_sash_stays_within_frame_projection",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        open_pos = ctx.part_world_position(sliding_sash)
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a="bottom_guide",
            elem_b="sill_inner_track",
            name="sliding_sash_bottom_guide_runs_on_inner_sill_track_open",
        )
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a="top_guide",
            elem_b="head_inner_track",
            name="sliding_sash_top_guide_stays_captured_open",
        )
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a="left_stile",
            elem_b="left_open_stop",
            name="open_sash_meets_left_stop",
        )
        ctx.expect_within(
            sliding_sash,
            frame,
            axes="xz",
            margin=0.0,
            name="open_sash_stays_within_frame_projection",
        )

    slide_ok = (
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] < closed_pos[0] - 0.35
        and abs(open_pos[1] - closed_pos[1]) < 1e-6
        and abs(open_pos[2] - closed_pos[2]) < 1e-6
    )
    ctx.check(
        "positive_prismatic_travel_opens_sash_leftward_without_drift",
        slide_ok,
        details=(
            f"closed={closed_pos}, open={open_pos}; expected substantial negative-x travel "
            "with no y/z drift."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
