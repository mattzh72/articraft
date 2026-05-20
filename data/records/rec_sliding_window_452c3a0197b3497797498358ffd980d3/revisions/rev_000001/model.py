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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sliding_window")

    painted_metal = model.material("painted_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    glass = model.material("glass", rgba=(0.68, 0.78, 0.84, 0.35))
    polymer = model.material("polymer", rgba=(0.16, 0.17, 0.18, 1.0))
    elastomer = model.material("elastomer", rgba=(0.07, 0.07, 0.08, 1.0))

    window_w = 1.38
    window_h = 1.18
    frame_d = 0.106
    frame_border_x = 0.06
    frame_border_z = 0.06
    opening_w = window_w - 2.0 * frame_border_x
    opening_h = window_h - 2.0 * frame_border_z

    lip_w = 0.018
    lip_d = 0.015
    web_w = 0.012
    web_d = 0.012
    same_part_overlap = 0.004

    rear_track_y = -0.022
    front_track_y = 0.022

    panel_w = 0.66
    panel_h = 1.03
    panel_d = 0.024
    panel_bottom = frame_border_z + 0.010
    panel_center_z = panel_bottom + panel_h / 2.0

    fixed_center_x = -0.27
    sliding_closed_x = 0.27
    sliding_travel = 0.54

    def add_opening_ring(
        part,
        *,
        prefix: str,
        ring_w: float,
        y_center: float,
        depth: float,
        material,
    ) -> None:
        part.visual(
            Box((ring_w, depth, opening_h + same_part_overlap)),
            origin=Origin(
                xyz=(
                    -opening_w / 2.0 + ring_w / 2.0,
                    y_center,
                    frame_border_z + opening_h / 2.0,
                )
            ),
            material=material,
            name=f"{prefix}_left",
        )
        part.visual(
            Box((ring_w, depth, opening_h + same_part_overlap)),
            origin=Origin(
                xyz=(
                    opening_w / 2.0 - ring_w / 2.0,
                    y_center,
                    frame_border_z + opening_h / 2.0,
                )
            ),
            material=material,
            name=f"{prefix}_right",
        )
        part.visual(
            Box((opening_w + same_part_overlap, depth, ring_w)),
            origin=Origin(
                xyz=(0.0, y_center, frame_border_z + ring_w / 2.0)
            ),
            material=material,
            name=f"{prefix}_bottom",
        )
        part.visual(
            Box((opening_w + same_part_overlap, depth, ring_w)),
            origin=Origin(
                xyz=(0.0, y_center, window_h - frame_border_z - ring_w / 2.0)
            ),
            material=material,
            name=f"{prefix}_top",
        )

    def build_panel(
        *,
        name: str,
        left_stile_w: float,
        right_stile_w: float,
        rail_h: float,
        meeting_side: str,
        add_pull: bool,
    ):
        part = model.part(name)
        clear_w = panel_w - left_stile_w - right_stile_w
        clear_h = panel_h - 2.0 * rail_h
        glass_th = 0.006
        infill_center_x = (left_stile_w - right_stile_w) / 2.0

        part.visual(
            Box((left_stile_w, panel_d, panel_h)),
            origin=Origin(xyz=(-panel_w / 2.0 + left_stile_w / 2.0, 0.0, 0.0)),
            material=painted_metal,
            name="left_stile",
        )
        part.visual(
            Box((right_stile_w, panel_d, panel_h)),
            origin=Origin(xyz=(panel_w / 2.0 - right_stile_w / 2.0, 0.0, 0.0)),
            material=painted_metal,
            name="right_stile",
        )
        part.visual(
            Box((clear_w + same_part_overlap, panel_d, rail_h)),
            origin=Origin(
                xyz=(
                    infill_center_x,
                    0.0,
                    -panel_h / 2.0 + rail_h / 2.0,
                )
            ),
            material=painted_metal,
            name="bottom_rail",
        )
        part.visual(
            Box((clear_w + same_part_overlap, panel_d, rail_h)),
            origin=Origin(
                xyz=(
                    infill_center_x,
                    0.0,
                    panel_h / 2.0 - rail_h / 2.0,
                )
            ),
            material=painted_metal,
            name="top_rail",
        )
        part.visual(
            Box((clear_w + 0.002, glass_th, clear_h + 0.002)),
            origin=Origin(xyz=(infill_center_x, 0.0, 0.0)),
            material=glass,
            name="glass",
        )

        if meeting_side == "right":
            seal_x = panel_w / 2.0 - 0.005
        else:
            seal_x = -panel_w / 2.0 + 0.005

        seal_y = 0.013 if meeting_side == "right" else -0.013
        part.visual(
            Box((0.010, 0.004, clear_h * 0.88)),
            origin=Origin(xyz=(seal_x, seal_y, 0.0)),
            material=elastomer,
            name="meeting_seal",
        )

        if add_pull:
            part.visual(
                Box((0.020, 0.004, 0.32)),
                origin=Origin(
                    xyz=(panel_w / 2.0 - 0.022, 0.013, 0.0)
                ),
                material=polymer,
                name="pull_rail",
            )

        part.inertial = Inertial.from_geometry(
            Box((panel_w, panel_d, panel_h)),
            mass=8.5 if add_pull else 8.0,
        )
        return part

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((frame_border_x, frame_d, window_h)),
        origin=Origin(xyz=(-window_w / 2.0 + frame_border_x / 2.0, 0.0, window_h / 2.0)),
        material=painted_metal,
        name="left_jamb",
    )
    outer_frame.visual(
        Box((frame_border_x, frame_d, window_h)),
        origin=Origin(xyz=(window_w / 2.0 - frame_border_x / 2.0, 0.0, window_h / 2.0)),
        material=painted_metal,
        name="right_jamb",
    )
    outer_frame.visual(
        Box((opening_w + same_part_overlap, frame_d, frame_border_z)),
        origin=Origin(xyz=(0.0, 0.0, frame_border_z / 2.0)),
        material=painted_metal,
        name="sill",
    )
    outer_frame.visual(
        Box((opening_w + same_part_overlap, frame_d, frame_border_z)),
        origin=Origin(xyz=(0.0, 0.0, window_h - frame_border_z / 2.0)),
        material=painted_metal,
        name="head",
    )

    add_opening_ring(
        outer_frame,
        prefix="front_lip",
        ring_w=lip_w,
        y_center=frame_d / 2.0 - lip_d / 2.0,
        depth=lip_d,
        material=painted_metal,
    )
    add_opening_ring(
        outer_frame,
        prefix="rear_lip",
        ring_w=lip_w,
        y_center=-frame_d / 2.0 + lip_d / 2.0,
        depth=lip_d,
        material=painted_metal,
    )
    add_opening_ring(
        outer_frame,
        prefix="center_web",
        ring_w=web_w,
        y_center=0.0,
        depth=web_d,
        material=painted_metal,
    )

    support_w = 1.20
    support_d = 0.022
    support_h = 0.010
    head_strip_h = 0.020
    stop_w = 0.022

    outer_frame.visual(
        Box((support_w, support_d, support_h)),
        origin=Origin(xyz=(0.0, rear_track_y, frame_border_z + support_h / 2.0)),
        material=polymer,
        name="rear_sill_guide",
    )
    outer_frame.visual(
        Box((support_w, support_d, support_h)),
        origin=Origin(xyz=(0.0, front_track_y, frame_border_z + support_h / 2.0)),
        material=polymer,
        name="front_sill_guide",
    )
    outer_frame.visual(
        Box((support_w, support_d, head_strip_h)),
        origin=Origin(
            xyz=(0.0, rear_track_y, window_h - head_strip_h / 2.0)
        ),
        material=elastomer,
        name="rear_head_sweep",
    )
    outer_frame.visual(
        Box((support_w, support_d, head_strip_h)),
        origin=Origin(
            xyz=(0.0, front_track_y, window_h - head_strip_h / 2.0)
        ),
        material=elastomer,
        name="front_head_sweep",
    )
    outer_frame.visual(
        Box((stop_w, 0.016, panel_h)),
        origin=Origin(
            xyz=(-0.611, front_track_y, panel_center_z)
        ),
        material=elastomer,
        name="open_stop",
    )
    outer_frame.visual(
        Box((stop_w, 0.016, panel_h)),
        origin=Origin(
            xyz=(0.611, front_track_y, panel_center_z)
        ),
        material=elastomer,
        name="close_stop",
    )
    outer_frame.inertial = Inertial.from_geometry(
        Box((window_w, frame_d, window_h)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, window_h / 2.0)),
    )

    fixed_lite = build_panel(
        name="fixed_lite",
        left_stile_w=0.045,
        right_stile_w=0.050,
        rail_h=0.045,
        meeting_side="right",
        add_pull=False,
    )
    sliding_sash = build_panel(
        name="sliding_sash",
        left_stile_w=0.058,
        right_stile_w=0.045,
        rail_h=0.045,
        meeting_side="left",
        add_pull=True,
    )

    model.articulation(
        "frame_to_fixed_lite",
        ArticulationType.FIXED,
        parent=outer_frame,
        child=fixed_lite,
        origin=Origin(xyz=(fixed_center_x, rear_track_y, panel_center_z)),
    )
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=sliding_sash,
        origin=Origin(xyz=(sliding_closed_x, front_track_y, panel_center_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.35,
            lower=0.0,
            upper=sliding_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    fixed_lite = object_model.get_part("fixed_lite")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_joint = object_model.get_articulation("frame_to_sliding_sash")

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

    ctx.expect_contact(fixed_lite, outer_frame, name="fixed_lite_supported_by_frame")
    ctx.expect_contact(sliding_sash, outer_frame, name="sliding_sash_supported_by_guides")
    ctx.check(
        "sash_joint_axis_is_horizontal",
        tuple(sash_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"Expected horizontal leftward prismatic axis, got {sash_joint.axis}",
    )

    with ctx.pose({sash_joint: 0.0}):
        ctx.expect_within(
            fixed_lite,
            outer_frame,
            axes="xz",
            margin=0.0,
            name="fixed_lite_within_frame_bounds",
        )
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="xz",
            margin=0.0,
            name="closed_sash_within_frame_bounds",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_lite,
            axes="x",
            min_overlap=0.10,
            name="closed_interlock_overlap",
        )
        ctx.expect_gap(
            sliding_sash,
            fixed_lite,
            axis="y",
            min_gap=0.034,
            max_gap=0.042,
            positive_elem="glass",
            negative_elem="glass",
            name="front_and_rear_glazing_planes_stay_separate",
        )

    upper = sash_joint.motion_limits.upper if sash_joint.motion_limits is not None else 0.0
    closed_pos = ctx.part_world_position(sliding_sash)
    with ctx.pose({sash_joint: upper}):
        open_pos = ctx.part_world_position(sliding_sash)
        ctx.expect_within(
            sliding_sash,
            outer_frame,
            axes="xz",
            margin=0.0,
            name="open_sash_within_frame_bounds",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_lite,
            axes="x",
            min_overlap=0.62,
            name="open_sash_stacks_over_fixed_lite",
        )
        ctx.expect_contact(
            sliding_sash,
            outer_frame,
            name="open_sash_remains_supported_by_guides",
        )

    moved_left = (
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] < closed_pos[0] - 0.50
        and abs(open_pos[1] - closed_pos[1]) < 1e-6
        and abs(open_pos[2] - closed_pos[2]) < 1e-6
    )
    ctx.check(
        "sash_slides_left_without_leaving_track_plane",
        moved_left,
        details=f"Closed pose {closed_pos}, open pose {open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
