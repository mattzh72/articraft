from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
FRAME_W = 1.60
FRAME_H = 1.25
FRAME_D = 0.14
JAMB_T = 0.06
HEAD_T = 0.055
SILL_T = 0.07
OPEN_W = FRAME_W - (2.0 * JAMB_T)

SASH_Z0 = 0.10
SASH_H = 1.05
FIXED_TRACK_Y = -0.029
SLIDING_TRACK_Y = 0.029
TRACK_D = 0.036
TRACK_BASE_H = 0.028
HEAD_GUIDE_H = 0.043

FRAME_FINISH = Material("powder_coated_aluminum", (0.93, 0.94, 0.95, 1.0))
TRACK_FINISH = Material("anodized_track", (0.79, 0.80, 0.82, 1.0))
GASKET_FINISH = Material("gasket_black", (0.10, 0.10, 0.11, 1.0))
GLASS_FINISH = Material("lowe_glass", (0.74, 0.82, 0.90, 0.35))
HARDWARE_FINISH = Material("hardware_dark", (0.17, 0.18, 0.20, 1.0))


def _add_box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_glazed_panel(
    part,
    *,
    panel_name,
    outer_left,
    outer_right,
    bottom_z,
    height,
    center_y,
    depth,
    left_stile,
    right_stile,
    rail_height,
    frame_material,
    glass_material,
    gasket_material,
):
    width = outer_right - outer_left
    center_x = (outer_left + outer_right) * 0.5
    center_z = bottom_z + (height * 0.5)

    _add_box(
        part,
        (width, depth, rail_height),
        (center_x, center_y, bottom_z + (rail_height * 0.5)),
        frame_material,
        f"{panel_name}_bottom_rail",
    )
    _add_box(
        part,
        (width, depth, rail_height),
        (center_x, center_y, bottom_z + height - (rail_height * 0.5)),
        frame_material,
        f"{panel_name}_top_rail",
    )
    _add_box(
        part,
        (left_stile, depth, height),
        (outer_left + (left_stile * 0.5), center_y, center_z),
        frame_material,
        f"{panel_name}_left_stile",
    )
    _add_box(
        part,
        (right_stile, depth, height),
        (outer_right - (right_stile * 0.5), center_y, center_z),
        frame_material,
        f"{panel_name}_right_stile",
    )

    clear_left = outer_left + left_stile
    clear_right = outer_right - right_stile
    clear_bottom = bottom_z + rail_height
    clear_top = bottom_z + height - rail_height
    clear_width = clear_right - clear_left
    clear_height = clear_top - clear_bottom

    gasket_depth = depth * 0.42
    gasket_t = 0.012
    gasket_center_y = center_y

    _add_box(
        part,
        (gasket_t, gasket_depth, clear_height),
        (clear_left + (gasket_t * 0.5), gasket_center_y, clear_bottom + (clear_height * 0.5)),
        gasket_material,
        f"{panel_name}_gasket_left",
    )
    _add_box(
        part,
        (gasket_t, gasket_depth, clear_height),
        (clear_right - (gasket_t * 0.5), gasket_center_y, clear_bottom + (clear_height * 0.5)),
        gasket_material,
        f"{panel_name}_gasket_right",
    )
    _add_box(
        part,
        (clear_width, gasket_depth, gasket_t),
        ((clear_left + clear_right) * 0.5, gasket_center_y, clear_bottom + (gasket_t * 0.5)),
        gasket_material,
        f"{panel_name}_gasket_bottom",
    )
    _add_box(
        part,
        (clear_width, gasket_depth, gasket_t),
        ((clear_left + clear_right) * 0.5, gasket_center_y, clear_top - (gasket_t * 0.5)),
        gasket_material,
        f"{panel_name}_gasket_top",
    )

    _add_box(
        part,
        (clear_width + 0.002, depth * 0.24, clear_height + 0.002),
        ((clear_left + clear_right) * 0.5, center_y, (clear_bottom + clear_top) * 0.5),
        glass_material,
        f"{panel_name}_glass",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_window", assets=ASSETS)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, FRAME_D, FRAME_H)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_H * 0.5)),
    )

    _add_box(frame, (FRAME_W, FRAME_D, SILL_T), (0.0, 0.0, SILL_T * 0.5), FRAME_FINISH, "sill")
    _add_box(
        frame,
        (FRAME_W, FRAME_D, HEAD_T),
        (0.0, 0.0, FRAME_H - (HEAD_T * 0.5)),
        FRAME_FINISH,
        "head",
    )
    jamb_height = FRAME_H - HEAD_T - SILL_T + 0.004
    _add_box(
        frame,
        (JAMB_T, FRAME_D, jamb_height),
        (-(FRAME_W * 0.5) + (JAMB_T * 0.5), 0.0, SILL_T + (jamb_height * 0.5) - 0.002),
        FRAME_FINISH,
        "left_jamb",
    )
    _add_box(
        frame,
        (JAMB_T, FRAME_D, jamb_height),
        ((FRAME_W * 0.5) - (JAMB_T * 0.5), 0.0, SILL_T + (jamb_height * 0.5) - 0.002),
        FRAME_FINISH,
        "right_jamb",
    )

    track_base_z = SILL_T + (TRACK_BASE_H * 0.5)
    head_guide_z = FRAME_H - HEAD_T + (HEAD_GUIDE_H * 0.5) - 0.004
    _add_box(
        frame,
        (OPEN_W, TRACK_D, TRACK_BASE_H),
        (0.0, FIXED_TRACK_Y, track_base_z),
        TRACK_FINISH,
        "fixed_track_base",
    )
    _add_box(
        frame,
        (OPEN_W, TRACK_D, TRACK_BASE_H),
        (0.0, SLIDING_TRACK_Y, track_base_z),
        TRACK_FINISH,
        "sliding_track_base",
    )
    _add_box(
        frame, (OPEN_W, 0.012, 0.034), (0.0, 0.0, SILL_T + 0.017), TRACK_FINISH, "track_separator"
    )
    _add_box(
        frame, (OPEN_W, 0.010, 0.034), (0.0, 0.055, SILL_T + 0.017), FRAME_FINISH, "interior_stop"
    )
    _add_box(
        frame, (OPEN_W, 0.010, 0.030), (0.0, -0.055, SILL_T + 0.015), FRAME_FINISH, "exterior_stop"
    )

    _add_box(
        frame,
        (OPEN_W, TRACK_D, HEAD_GUIDE_H),
        (0.0, FIXED_TRACK_Y, head_guide_z),
        TRACK_FINISH,
        "fixed_head_guide",
    )
    _add_box(
        frame,
        (OPEN_W, TRACK_D, HEAD_GUIDE_H),
        (0.0, SLIDING_TRACK_Y, head_guide_z),
        TRACK_FINISH,
        "sliding_head_guide",
    )
    _add_box(
        frame,
        (OPEN_W, 0.012, 0.032),
        (0.0, 0.0, FRAME_H - HEAD_T - 0.016),
        TRACK_FINISH,
        "head_separator",
    )

    _add_box(frame, (0.16, 0.022, 0.012), (-0.48, -0.059, 0.046), TRACK_FINISH, "weep_cover_left")
    _add_box(frame, (0.16, 0.022, 0.012), (0.48, -0.059, 0.046), TRACK_FINISH, "weep_cover_right")

    _add_glazed_panel(
        frame,
        panel_name="fixed_panel",
        outer_left=-0.74,
        outer_right=0.03,
        bottom_z=SASH_Z0,
        height=SASH_H,
        center_y=FIXED_TRACK_Y,
        depth=0.040,
        left_stile=0.050,
        right_stile=0.046,
        rail_height=0.060,
        frame_material=FRAME_FINISH,
        glass_material=GLASS_FINISH,
        gasket_material=GASKET_FINISH,
    )
    _add_box(frame, (0.018, 0.010, 0.120), (0.024, -0.003, 0.63), HARDWARE_FINISH, "keeper_plate")

    sliding_sash = model.part("sliding_sash")
    sliding_sash.inertial = Inertial.from_geometry(
        Box((0.74, 0.042, SASH_H)),
        mass=9.5,
        origin=Origin(xyz=(-0.37, 0.0, SASH_H * 0.5)),
    )

    _add_glazed_panel(
        sliding_sash,
        panel_name="sliding_panel",
        outer_left=-0.74,
        outer_right=0.0,
        bottom_z=0.0,
        height=SASH_H,
        center_y=0.0,
        depth=0.042,
        left_stile=0.058,
        right_stile=0.048,
        rail_height=0.060,
        frame_material=FRAME_FINISH,
        glass_material=GLASS_FINISH,
        gasket_material=GASKET_FINISH,
    )
    _add_box(
        sliding_sash, (0.016, 0.012, 0.180), (-0.045, 0.027, 0.60), HARDWARE_FINISH, "pull_handle"
    )
    _add_box(
        sliding_sash,
        (0.020, 0.010, 0.046),
        (-0.040, 0.023, 0.73),
        HARDWARE_FINISH,
        "latch_thumbturn",
    )
    _add_box(
        sliding_sash,
        (0.036, 0.012, 0.018),
        (-0.090, 0.0, 0.012),
        HARDWARE_FINISH,
        "roller_cover_left",
    )
    _add_box(
        sliding_sash,
        (0.036, 0.012, 0.018),
        (-0.650, 0.0, 0.012),
        HARDWARE_FINISH,
        "roller_cover_right",
    )

    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent="frame",
        child="sliding_sash",
        origin=Origin(xyz=(0.73, SLIDING_TRACK_Y, SASH_Z0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.45,
            lower=-0.56,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "frame_to_sliding_sash",
        "sliding_sash",
        world_axis="x",
        direction="positive",
        min_delta=0.10,
    )

    closed_pos = ctx.part_world_position("sliding_sash")
    if not (0.68 <= closed_pos[0] <= 0.76):
        raise AssertionError("Closed sash origin should sit near the right jamb.")
    if abs(closed_pos[1] - SLIDING_TRACK_Y) > 1e-6:
        raise AssertionError("Closed sash should sit on the interior sliding track centerline.")
    if abs(closed_pos[2] - SASH_Z0) > 1e-6:
        raise AssertionError("Closed sash base should ride at the sill track elevation.")

    with ctx.pose(frame_to_sliding_sash=-0.28):
        mid_pos = ctx.part_world_position("sliding_sash")

    with ctx.pose(frame_to_sliding_sash=-0.56):
        open_pos = ctx.part_world_position("sliding_sash")

    if not (0.42 <= mid_pos[0] <= 0.48):
        raise AssertionError("Half-open sash should sit near the center of its travel.")
    if not (0.14 <= open_pos[0] <= 0.20):
        raise AssertionError(
            "Fully open sash should stack over the fixed lite near the centerline."
        )
    if mid_pos[0] >= closed_pos[0] - 0.20:
        raise AssertionError("Half-open sash should translate substantially leftward.")
    if open_pos[0] >= mid_pos[0] - 0.20:
        raise AssertionError("Fully open sash should continue leftward beyond the half-open pose.")
    if abs((closed_pos[0] - open_pos[0]) - 0.56) > 0.02:
        raise AssertionError("Full sash travel should match the intended clear opening width.")

    for pose_name, pos in (("mid", mid_pos), ("open", open_pos)):
        if abs(pos[1] - closed_pos[1]) > 1e-6:
            raise AssertionError(f"{pose_name} pose should not drift out of its track depth.")
        if abs(pos[2] - closed_pos[2]) > 1e-6:
            raise AssertionError(f"{pose_name} pose should stay level in the frame.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
