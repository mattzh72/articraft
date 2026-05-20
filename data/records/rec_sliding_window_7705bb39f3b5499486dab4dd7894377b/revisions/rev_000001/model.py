from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_WIDTH = 0.54
FRAME_DEPTH = 0.048
FRAME_HEIGHT = 0.38
JAMB_WIDTH = 0.028
HEAD_HEIGHT = 0.030
SILL_HEIGHT = 0.032

INNER_WIDTH = FRAME_WIDTH - 2.0 * JAMB_WIDTH
OPENING_BOTTOM = SILL_HEIGHT
OPENING_TOP = FRAME_HEIGHT - HEAD_HEIGHT
OPENING_HEIGHT = OPENING_TOP - OPENING_BOTTOM

FIXED_PANEL_WIDTH = 0.250
FIXED_PANEL_HEIGHT = OPENING_HEIGHT
FIXED_PANEL_DEPTH = 0.014
FIXED_PANEL_CENTER = (-0.117, -0.010, OPENING_BOTTOM + OPENING_HEIGHT * 0.5)

SASH_WIDTH = 0.250
SASH_HEIGHT = 0.304
SASH_DEPTH = 0.014
SASH_CLOSED_CENTER = (0.110, 0.008, 0.192)
SASH_TRAVEL = 0.176

FRAME_MEMBER = 0.022
SASH_LEFT_STILE = 0.026
SASH_RIGHT_STILE = 0.022
SASH_RAIL = 0.022


def _add_panel_frame(
    part,
    *,
    prefix: str,
    center: tuple[float, float, float],
    width: float,
    height: float,
    depth: float,
    left_stile: float,
    right_stile: float,
    rail: float,
    frame_material,
    glass_material,
    glass_name: str,
) -> None:
    cx, cy, cz = center
    half_w = width * 0.5
    half_h = height * 0.5

    part.visual(
        Box((left_stile, depth, height)),
        origin=Origin(xyz=(cx - half_w + left_stile * 0.5, cy, cz)),
        material=frame_material,
        name=f"{prefix}left_stile",
    )
    part.visual(
        Box((right_stile, depth, height)),
        origin=Origin(xyz=(cx + half_w - right_stile * 0.5, cy, cz)),
        material=frame_material,
        name=f"{prefix}right_stile",
    )
    part.visual(
        Box((width, depth, rail)),
        origin=Origin(xyz=(cx, cy, cz + half_h - rail * 0.5)),
        material=frame_material,
        name=f"{prefix}top_rail",
    )
    part.visual(
        Box((width, depth, rail)),
        origin=Origin(xyz=(cx, cy, cz - half_h + rail * 0.5)),
        material=frame_material,
        name=f"{prefix}bottom_rail",
    )

    glass_width = width - left_stile - right_stile
    glass_height = height - 2.0 * rail
    glass_center_x = cx + (left_stile - right_stile) * 0.5
    part.visual(
        Box((glass_width, 0.004, glass_height)),
        origin=Origin(xyz=(glass_center_x, cy, cz)),
        material=glass_material,
        name=glass_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_sliding_window")

    frame_aluminum = model.material("frame_aluminum", rgba=(0.74, 0.75, 0.77, 1.0))
    track_aluminum = model.material("track_aluminum", rgba=(0.62, 0.64, 0.67, 1.0))
    gasket = model.material("gasket", rgba=(0.18, 0.19, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.67, 0.82, 0.91, 0.42))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT * 0.5)),
    )

    frame.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-FRAME_WIDTH * 0.5 + JAMB_WIDTH * 0.5, 0.0, FRAME_HEIGHT * 0.5)),
        material=frame_aluminum,
        name="left_jamb",
    )
    frame.visual(
        Box((JAMB_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(FRAME_WIDTH * 0.5 - JAMB_WIDTH * 0.5, 0.0, FRAME_HEIGHT * 0.5)),
        material=frame_aluminum,
        name="right_jamb",
    )
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT - HEAD_HEIGHT * 0.5)),
        material=frame_aluminum,
        name="head",
    )
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, SILL_HEIGHT * 0.5)),
        material=frame_aluminum,
        name="sill",
    )

    _add_panel_frame(
        frame,
        prefix="fixed_",
        center=FIXED_PANEL_CENTER,
        width=FIXED_PANEL_WIDTH,
        height=FIXED_PANEL_HEIGHT,
        depth=FIXED_PANEL_DEPTH,
        left_stile=FRAME_MEMBER,
        right_stile=FRAME_MEMBER,
        rail=FRAME_MEMBER,
        frame_material=track_aluminum,
        glass_material=glass,
        glass_name="fixed_glass",
    )

    guide_width = INNER_WIDTH
    separator_depth = 0.004
    keeper_depth = 0.008
    bottom_guide_height = 0.008
    top_guide_height = 0.006

    frame.visual(
        Box((guide_width, separator_depth, bottom_guide_height)),
        origin=Origin(xyz=(0.0, -0.001, OPENING_BOTTOM + bottom_guide_height * 0.5)),
        material=track_aluminum,
        name="separator_bottom",
    )
    frame.visual(
        Box((guide_width, separator_depth, top_guide_height)),
        origin=Origin(xyz=(0.0, -0.001, OPENING_TOP - top_guide_height * 0.5)),
        material=track_aluminum,
        name="separator_top",
    )
    frame.visual(
        Box((guide_width, keeper_depth, bottom_guide_height)),
        origin=Origin(xyz=(0.0, 0.019, OPENING_BOTTOM + bottom_guide_height * 0.5)),
        material=gasket,
        name="front_bottom_guide",
    )
    frame.visual(
        Box((guide_width, keeper_depth, top_guide_height)),
        origin=Origin(xyz=(0.0, 0.019, OPENING_TOP - top_guide_height * 0.5)),
        material=gasket,
        name="front_top_guide",
    )

    stop_width = 0.008
    stop_depth = 0.014
    stop_height = 0.018
    frame.visual(
        Box((stop_width, stop_depth, stop_height)),
        origin=Origin(xyz=(0.239, 0.008, 0.049)),
        material=gasket,
        name="right_stop_bottom",
    )
    frame.visual(
        Box((stop_width, stop_depth, stop_height)),
        origin=Origin(xyz=(0.239, 0.008, 0.335)),
        material=gasket,
        name="right_stop_top",
    )
    frame.visual(
        Box((stop_width, stop_depth, stop_height)),
        origin=Origin(xyz=(-0.195, 0.008, 0.049)),
        material=gasket,
        name="left_stop_bottom",
    )
    frame.visual(
        Box((stop_width, stop_depth, stop_height)),
        origin=Origin(xyz=(-0.195, 0.008, 0.335)),
        material=gasket,
        name="left_stop_top",
    )

    sash = model.part("sliding_sash")
    sash.inertial = Inertial.from_geometry(
        Box((SASH_WIDTH, SASH_DEPTH, SASH_HEIGHT)),
        mass=1.15,
    )

    _add_panel_frame(
        sash,
        prefix="",
        center=(0.0, 0.0, 0.0),
        width=SASH_WIDTH,
        height=SASH_HEIGHT,
        depth=SASH_DEPTH,
        left_stile=SASH_LEFT_STILE,
        right_stile=SASH_RIGHT_STILE,
        rail=SASH_RAIL,
        frame_material=track_aluminum,
        glass_material=glass,
        glass_name="glass",
    )
    sash.visual(
        Box((0.014, 0.006, 0.080)),
        origin=Origin(xyz=(-0.100, 0.010, 0.0)),
        material=gasket,
        name="pull",
    )

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=SASH_CLOSED_CENTER),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.25,
            lower=0.0,
            upper=SASH_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sliding_sash")
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            sash,
            frame,
            axes="xyz",
            margin=0.0,
            name="closed_sash_stays_inside_frame_envelope",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            min_gap=0.010,
            max_gap=0.016,
            positive_elem="glass",
            negative_elem="fixed_glass",
            name="closed_front_and_back_glass_have_safe_clearance",
        )
        ctx.expect_contact(
            frame,
            sash,
            elem_a="right_stop_bottom",
            elem_b="right_stile",
            name="closed_sash_seats_against_right_stop",
        )

    with ctx.pose({slide: SASH_TRAVEL}):
        ctx.expect_within(
            sash,
            frame,
            axes="xyz",
            margin=0.0,
            name="open_sash_stays_inside_frame_envelope",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="left_stile",
            elem_b="left_stop_bottom",
            name="open_sash_seats_against_left_stop",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            min_gap=0.010,
            max_gap=0.016,
            positive_elem="glass",
            negative_elem="fixed_glass",
            name="open_front_and_back_glass_keep_track_clearance",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="z",
            min_overlap=0.25,
            elem_a="glass",
            elem_b="fixed_glass",
            name="sash_and_fixed_lite_remain_vertically_aligned",
        )

    with ctx.pose({slide: 0.0}):
        closed_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: SASH_TRAVEL}):
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "sash_motion_is_pure_leftward_slide",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] < closed_pos[0] - 0.15
        and abs(open_pos[1] - closed_pos[1]) < 1e-6
        and abs(open_pos[2] - closed_pos[2]) < 1e-6,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
