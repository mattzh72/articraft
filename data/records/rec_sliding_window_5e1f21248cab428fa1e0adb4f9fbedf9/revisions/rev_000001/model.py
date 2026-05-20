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


FRAME_W = 1.20
FRAME_H = 1.00
FRAME_D = 0.09
FRAME_RAIL = 0.05

SASH_W = 0.62
SASH_FRAME_H = 0.86
SASH_TOTAL_H = 0.90
SASH_D = 0.024

SASH_CLOSED_X = 0.24
SASH_TRACK_Y = 0.0145
SASH_CENTER_Z = 0.50
SASH_TRAVEL = 0.48


def _add_box(part, size, xyz, material, name):
    return part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_sliding_window")

    frame_mat = model.material("frame_vinyl", rgba=(0.92, 0.92, 0.90, 1.0))
    bead_mat = model.material("bead_vinyl", rgba=(0.87, 0.87, 0.85, 1.0))
    glass_mat = model.material("glass", rgba=(0.62, 0.78, 0.88, 0.35))

    frame = model.part("frame_assembly")

    # One welded perimeter extrusion set with integral head/sill tracks.
    _add_box(
        frame,
        (FRAME_RAIL, FRAME_D, FRAME_H),
        (-0.5 * FRAME_W + 0.5 * FRAME_RAIL, 0.0, 0.5 * FRAME_H),
        frame_mat,
        "left_jamb",
    )
    _add_box(
        frame,
        (FRAME_RAIL, FRAME_D, FRAME_H),
        (0.5 * FRAME_W - 0.5 * FRAME_RAIL, 0.0, 0.5 * FRAME_H),
        frame_mat,
        "right_jamb",
    )
    _add_box(
        frame,
        (FRAME_W - 2.0 * FRAME_RAIL, FRAME_D, FRAME_RAIL),
        (0.0, 0.0, FRAME_H - 0.5 * FRAME_RAIL),
        frame_mat,
        "head",
    )
    _add_box(
        frame,
        (FRAME_W - 2.0 * FRAME_RAIL, FRAME_D, FRAME_RAIL),
        (0.0, 0.0, 0.5 * FRAME_RAIL),
        frame_mat,
        "sill",
    )

    # Exterior mounting flange for obvious wall attachment without extra parts.
    flange_t = 0.01
    flange_w = 0.015
    _add_box(
        frame,
        (flange_w, flange_t, FRAME_H + 0.08),
        (-0.5 * FRAME_W - 0.5 * flange_w, -0.5 * FRAME_D + 0.5 * flange_t, 0.5 * FRAME_H),
        frame_mat,
        "left_flange",
    )
    _add_box(
        frame,
        (flange_w, flange_t, FRAME_H + 0.08),
        (0.5 * FRAME_W + 0.5 * flange_w, -0.5 * FRAME_D + 0.5 * flange_t, 0.5 * FRAME_H),
        frame_mat,
        "right_flange",
    )
    _add_box(
        frame,
        (FRAME_W, flange_t, flange_w),
        (0.0, -0.5 * FRAME_D + 0.5 * flange_t, FRAME_H + 0.5 * flange_w),
        frame_mat,
        "head_flange",
    )
    _add_box(
        frame,
        (FRAME_W, flange_t, flange_w),
        (0.0, -0.5 * FRAME_D + 0.5 * flange_t, -0.5 * flange_w),
        frame_mat,
        "sill_flange",
    )

    # Two-track guide ribs are integrated into the head and sill extrusions.
    guide_wall_t = 0.008
    _add_box(
        frame,
        (FRAME_W - 2.0 * FRAME_RAIL, guide_wall_t, 0.025),
        (0.0, -0.005, 0.0625),
        frame_mat,
        "sill_outer_guide",
    )
    _add_box(
        frame,
        (FRAME_W - 2.0 * FRAME_RAIL, guide_wall_t, 0.035),
        (0.0, 0.034, 0.0675),
        frame_mat,
        "sill_inner_guide",
    )
    _add_box(
        frame,
        (FRAME_W - 2.0 * FRAME_RAIL, guide_wall_t, 0.025),
        (0.0, -0.005, 0.9375),
        frame_mat,
        "head_outer_guide",
    )
    _add_box(
        frame,
        (FRAME_W - 2.0 * FRAME_RAIL, guide_wall_t, 0.035),
        (0.0, 0.034, 0.9325),
        frame_mat,
        "head_inner_guide",
    )

    # Fixed lite is captured directly in the outer frame to keep part count low.
    fixed_glass_w = 0.56
    _add_box(
        frame,
        (fixed_glass_w, 0.006, FRAME_H - 2.0 * FRAME_RAIL),
        (-0.27, -0.023, 0.50),
        glass_mat,
        "fixed_glass",
    )
    _add_box(
        frame,
        (0.02, 0.012, FRAME_H - 2.0 * FRAME_RAIL),
        (0.02, -0.018, 0.50),
        bead_mat,
        "fixed_meeting_bead",
    )

    frame.inertial = Inertial.from_geometry(
        Box((FRAME_W + 0.03, FRAME_D, FRAME_H + 0.08)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    sash = model.part("sliding_sash")

    stile_w = 0.04
    rail_h = 0.04
    glass_w = SASH_W - 2.0 * stile_w
    glass_h = SASH_FRAME_H - 2.0 * rail_h

    _add_box(
        sash,
        (stile_w, SASH_D, 0.78),
        (-0.5 * SASH_W + 0.5 * stile_w, 0.0, 0.0),
        frame_mat,
        "left_stile",
    )
    _add_box(
        sash,
        (stile_w, SASH_D, 0.78),
        (0.5 * SASH_W - 0.5 * stile_w, 0.0, 0.0),
        frame_mat,
        "right_stile",
    )
    _add_box(
        sash,
        (glass_w, SASH_D, rail_h),
        (0.0, 0.0, 0.5 * SASH_FRAME_H - 0.5 * rail_h),
        frame_mat,
        "top_rail",
    )
    _add_box(
        sash,
        (glass_w, SASH_D, rail_h),
        (0.0, 0.0, -0.5 * SASH_FRAME_H + 0.5 * rail_h),
        frame_mat,
        "bottom_rail",
    )
    _add_box(
        sash,
        (glass_w, 0.004, glass_h),
        (0.0, -0.006, 0.0),
        glass_mat,
        "glass",
    )
    _add_box(
        sash,
        (SASH_W - 0.04, 0.014, 0.02),
        (0.0, 0.0, 0.5 * SASH_TOTAL_H - 0.01),
        frame_mat,
        "top_shoe",
    )
    _add_box(
        sash,
        (SASH_W - 0.04, 0.014, 0.02),
        (0.0, 0.0, -0.5 * SASH_TOTAL_H + 0.01),
        frame_mat,
        "bottom_shoe",
    )
    _add_box(
        sash,
        (0.08, 0.008, 0.03),
        (-0.23, 0.016, 0.0),
        frame_mat,
        "pull_lip",
    )

    sash.inertial = Inertial.from_geometry(
        Box((SASH_W, SASH_D, SASH_TOTAL_H)),
        mass=9.0,
        origin=Origin(),
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(SASH_CLOSED_X, SASH_TRACK_Y, SASH_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=SASH_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame_assembly")
    sash = object_model.get_part("sliding_sash")
    slide = object_model.get_articulation("frame_to_sash")

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

    limits = slide.motion_limits
    upper = SASH_TRAVEL if limits is None or limits.upper is None else limits.upper

    ctx.check(
        "slide_axis_points_left",
        tuple(slide.axis) == (-1.0, 0.0, 0.0),
        details=f"Expected leftward prismatic axis, got {slide.axis}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            sash,
            frame,
            elem_a="bottom_shoe",
            elem_b="sill",
            name="closed_bottom_shoe_supported",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="top_shoe",
            elem_b="head",
            name="closed_top_shoe_retained",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="right_stile",
            elem_b="right_jamb",
            name="closed_sash_hits_right_stop",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="z",
            margin=0.0,
            name="closed_sash_within_frame_height",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem="glass",
            negative_elem="fixed_glass",
            min_gap=0.02,
            name="closed_sash_clear_of_fixed_lite_plane",
        )
        ctx.expect_overlap(
            sash,
            frame,
            axes="z",
            elem_a="left_stile",
            elem_b="fixed_glass",
            min_overlap=0.70,
            name="closed_meeting_overlap_has_full_height_coverage",
        )

    with ctx.pose({slide: upper}):
        ctx.expect_contact(
            sash,
            frame,
            elem_a="bottom_shoe",
            elem_b="sill",
            name="open_bottom_shoe_supported",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="top_shoe",
            elem_b="head",
            name="open_top_shoe_retained",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="left_stile",
            elem_b="left_jamb",
            name="open_sash_hits_left_stop",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem="glass",
            negative_elem="fixed_glass",
            min_gap=0.02,
            name="open_sash_clear_of_fixed_lite_plane",
        )

    with ctx.pose({slide: 0.0}):
        closed_x = ctx.part_world_position(sash)[0]
    with ctx.pose({slide: upper}):
        open_x = ctx.part_world_position(sash)[0]
    ctx.check(
        "sash_opens_left_by_full_travel",
        open_x < closed_x - 0.45,
        details=f"Closed x={closed_x:.3f}, open x={open_x:.3f}, expected strong leftward travel.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
