from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

WINDOW_WIDTH = 1.60
WINDOW_HEIGHT = 1.26
WINDOW_DEPTH = 0.14
JAMB_WIDTH = 0.07
HEAD_HEIGHT = 0.055
SILL_HEIGHT = 0.065
CLEAR_WIDTH = WINDOW_WIDTH - 2.0 * JAMB_WIDTH
CLEAR_HEIGHT = WINDOW_HEIGHT - HEAD_HEIGHT - SILL_HEIGHT

SASH_WIDTH = 0.79
SASH_HEIGHT = 1.09
SASH_DEPTH = 0.032
RUNNER_HEIGHT = 0.012
GUIDE_HEIGHT = 0.012
TRACK_HEIGHT = 0.014
TRACK_CENTER_Y = 0.027
FIXED_SASH_X = -0.335
MOVING_SASH_X_CLOSED = 0.335
MOVING_SASH_TRAVEL = 0.54
SASH_CENTER_Z = SILL_HEIGHT + TRACK_HEIGHT + RUNNER_HEIGHT + SASH_HEIGHT / 2.0


def _aabb_size(aabb):
    return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))


def _add_glazed_sash(
    part,
    *,
    width: float,
    height: float,
    depth: float,
    left_stile: float,
    right_stile: float,
    top_rail: float,
    bottom_rail: float,
    meeting_side: str,
    frame_material,
    bead_material,
    glass_material,
    seal_material,
    runner_material,
    hardware_material,
    add_keeper: bool,
) -> None:
    half_w = width / 2.0
    half_h = height / 2.0
    half_d = depth / 2.0

    opening_x_min = -half_w + left_stile
    opening_x_max = half_w - right_stile
    opening_z_min = -half_h + bottom_rail
    opening_z_max = half_h - top_rail
    opening_w = opening_x_max - opening_x_min
    opening_h = opening_z_max - opening_z_min
    opening_center_x = (opening_x_min + opening_x_max) * 0.5
    opening_center_z = (opening_z_min + opening_z_max) * 0.5

    left_name = "meeting_stile" if meeting_side == "left" else "outer_stile"
    right_name = "meeting_stile" if meeting_side == "right" else "outer_stile"

    part.visual(
        Box((left_stile, depth, height)),
        origin=Origin(xyz=(-half_w + left_stile / 2.0, 0.0, 0.0)),
        material=frame_material,
        name=left_name,
    )
    part.visual(
        Box((right_stile, depth, height)),
        origin=Origin(xyz=(half_w - right_stile / 2.0, 0.0, 0.0)),
        material=frame_material,
        name=right_name,
    )
    part.visual(
        Box((width, depth, top_rail)),
        origin=Origin(xyz=(0.0, 0.0, half_h - top_rail / 2.0)),
        material=frame_material,
        name="top_rail",
    )
    part.visual(
        Box((width, depth, bottom_rail)),
        origin=Origin(xyz=(0.0, 0.0, -half_h + bottom_rail / 2.0)),
        material=frame_material,
        name="bottom_rail",
    )

    bead_depth = 0.004
    bead_width = 0.012
    bead_y = -half_d + bead_depth / 2.0
    part.visual(
        Box((bead_width, bead_depth, opening_h)),
        origin=Origin(xyz=(opening_x_min + bead_width / 2.0, bead_y, opening_center_z)),
        material=bead_material,
        name="bead_left",
    )
    part.visual(
        Box((bead_width, bead_depth, opening_h)),
        origin=Origin(xyz=(opening_x_max - bead_width / 2.0, bead_y, opening_center_z)),
        material=bead_material,
        name="bead_right",
    )
    part.visual(
        Box((opening_w, bead_depth, bead_width)),
        origin=Origin(xyz=(opening_center_x, bead_y, opening_z_max - bead_width / 2.0)),
        material=bead_material,
        name="bead_top",
    )
    part.visual(
        Box((opening_w, bead_depth, bead_width)),
        origin=Origin(xyz=(opening_center_x, bead_y, opening_z_min + bead_width / 2.0)),
        material=bead_material,
        name="bead_bottom",
    )

    gasket_thickness = 0.010
    gasket_depth = 0.022
    glass_width = opening_w - 2.0 * gasket_thickness
    glass_height = opening_h - 2.0 * gasket_thickness
    glass_depth = 0.018
    part.visual(
        Box((glass_width, glass_depth, glass_height)),
        origin=Origin(xyz=(opening_center_x, 0.0, opening_center_z)),
        material=glass_material,
        name="glass",
    )
    part.visual(
        Box((gasket_thickness, gasket_depth, glass_height)),
        origin=Origin(xyz=(opening_x_min + gasket_thickness / 2.0, 0.0, opening_center_z)),
        material=seal_material,
        name="gasket_left",
    )
    part.visual(
        Box((gasket_thickness, gasket_depth, glass_height)),
        origin=Origin(xyz=(opening_x_max - gasket_thickness / 2.0, 0.0, opening_center_z)),
        material=seal_material,
        name="gasket_right",
    )
    part.visual(
        Box((opening_w, gasket_depth, gasket_thickness)),
        origin=Origin(xyz=(opening_center_x, 0.0, opening_z_max - gasket_thickness / 2.0)),
        material=seal_material,
        name="gasket_top",
    )
    part.visual(
        Box((opening_w, gasket_depth, gasket_thickness)),
        origin=Origin(xyz=(opening_center_x, 0.0, opening_z_min + gasket_thickness / 2.0)),
        material=seal_material,
        name="gasket_bottom",
    )

    runner_length = 0.12
    runner_width = 0.012
    runner_x = width * 0.28
    runner_z = -half_h - RUNNER_HEIGHT / 2.0
    guide_length = 0.10
    guide_width = 0.014
    guide_z = half_h + GUIDE_HEIGHT / 2.0
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x_pos = side_sign * runner_x
        part.visual(
            Box((runner_length, runner_width, RUNNER_HEIGHT)),
            origin=Origin(xyz=(x_pos, 0.0, runner_z)),
            material=runner_material,
            name=f"runner_{side_name}",
        )
        part.visual(
            Box((guide_length, guide_width, GUIDE_HEIGHT)),
            origin=Origin(xyz=(x_pos, 0.0, guide_z)),
            material=runner_material,
            name=f"guide_{side_name}",
        )

    tongue_x = 0.016
    tongue_y = 0.006
    tongue_z = 0.86
    seal_y = 0.002
    if meeting_side == "right":
        tongue_x_pos = half_w - right_stile + tongue_x / 2.0
        tongue_y_pos = -half_d - tongue_y / 2.0
        seal_y_pos = -half_d - tongue_y - seal_y / 2.0
        keeper_x = half_w - right_stile / 2.0
    else:
        tongue_x_pos = -half_w + left_stile - tongue_x / 2.0
        tongue_y_pos = half_d + tongue_y / 2.0
        seal_y_pos = half_d + tongue_y + seal_y / 2.0
        keeper_x = -half_w + left_stile / 2.0

    part.visual(
        Box((tongue_x, tongue_y, tongue_z)),
        origin=Origin(xyz=(tongue_x_pos, tongue_y_pos, 0.0)),
        material=frame_material,
        name="meeting_interlock",
    )
    part.visual(
        Box((tongue_x, seal_y, tongue_z - 0.02)),
        origin=Origin(xyz=(tongue_x_pos, seal_y_pos, 0.0)),
        material=seal_material,
        name="meeting_seal",
    )

    if add_keeper:
        part.visual(
            Box((0.024, 0.004, 0.072)),
            origin=Origin(xyz=(keeper_x, -half_d + 0.002, 0.02)),
            material=hardware_material,
            name="keeper",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sliding_window", assets=ASSETS)

    frame_matte = model.material("frame_matte", rgba=(0.33, 0.34, 0.36, 1.0))
    sash_satin = model.material("sash_satin", rgba=(0.56, 0.57, 0.60, 1.0))
    bead_matte = model.material("bead_matte", rgba=(0.24, 0.25, 0.27, 1.0))
    track_satin = model.material("track_satin", rgba=(0.69, 0.70, 0.72, 1.0))
    seal_black = model.material("seal_black", rgba=(0.06, 0.06, 0.07, 1.0))
    glass_low_e = model.material("glass_low_e", rgba=(0.67, 0.80, 0.87, 0.30))
    hardware_satin = model.material("hardware_satin", rgba=(0.77, 0.78, 0.80, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((WINDOW_WIDTH, WINDOW_DEPTH, WINDOW_HEIGHT)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, WINDOW_HEIGHT / 2.0)),
    )
    frame.visual(
        Box((WINDOW_WIDTH, WINDOW_DEPTH, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, WINDOW_HEIGHT - HEAD_HEIGHT / 2.0)),
        material=frame_matte,
        name="head",
    )
    frame.visual(
        Box((WINDOW_WIDTH, WINDOW_DEPTH, SILL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, SILL_HEIGHT / 2.0)),
        material=frame_matte,
        name="sill",
    )
    frame.visual(
        Box((JAMB_WIDTH, WINDOW_DEPTH, CLEAR_HEIGHT)),
        origin=Origin(
            xyz=(-WINDOW_WIDTH / 2.0 + JAMB_WIDTH / 2.0, 0.0, SILL_HEIGHT + CLEAR_HEIGHT / 2.0)
        ),
        material=frame_matte,
        name="left_jamb",
    )
    frame.visual(
        Box((JAMB_WIDTH, WINDOW_DEPTH, CLEAR_HEIGHT)),
        origin=Origin(
            xyz=(WINDOW_WIDTH / 2.0 - JAMB_WIDTH / 2.0, 0.0, SILL_HEIGHT + CLEAR_HEIGHT / 2.0)
        ),
        material=frame_matte,
        name="right_jamb",
    )

    frame.visual(
        Box((CLEAR_WIDTH, 0.014, TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, TRACK_CENTER_Y, SILL_HEIGHT + TRACK_HEIGHT / 2.0)),
        material=track_satin,
        name="outer_track_rail",
    )
    frame.visual(
        Box((CLEAR_WIDTH, 0.014, TRACK_HEIGHT)),
        origin=Origin(xyz=(0.0, -TRACK_CENTER_Y, SILL_HEIGHT + TRACK_HEIGHT / 2.0)),
        material=track_satin,
        name="inner_track_rail",
    )
    frame.visual(
        Box((CLEAR_WIDTH, 0.018, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, TRACK_CENTER_Y, WINDOW_HEIGHT - HEAD_HEIGHT - GUIDE_HEIGHT / 2.0)),
        material=track_satin,
        name="outer_head_lip",
    )
    frame.visual(
        Box((CLEAR_WIDTH, 0.018, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, -TRACK_CENTER_Y, WINDOW_HEIGHT - HEAD_HEIGHT - GUIDE_HEIGHT / 2.0)),
        material=track_satin,
        name="inner_head_lip",
    )

    frame.visual(
        Box((CLEAR_WIDTH, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, SILL_HEIGHT + 0.013)),
        material=frame_matte,
        name="center_sill_divider",
    )
    frame.visual(
        Box((CLEAR_WIDTH, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, 0.056, SILL_HEIGHT + 0.016)),
        material=frame_matte,
        name="outer_sill_stop",
    )
    frame.visual(
        Box((CLEAR_WIDTH, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, -0.056, SILL_HEIGHT + 0.016)),
        material=frame_matte,
        name="inner_sill_stop",
    )
    frame.visual(
        Box((CLEAR_WIDTH, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, WINDOW_HEIGHT - HEAD_HEIGHT - 0.010)),
        material=frame_matte,
        name="center_head_divider",
    )
    frame.visual(
        Box((CLEAR_WIDTH, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.056, WINDOW_HEIGHT - HEAD_HEIGHT - 0.014)),
        material=frame_matte,
        name="outer_head_stop",
    )
    frame.visual(
        Box((CLEAR_WIDTH, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.056, WINDOW_HEIGHT - HEAD_HEIGHT - 0.014)),
        material=frame_matte,
        name="inner_head_stop",
    )

    fixed_sash = model.part("fixed_sash")
    fixed_sash.inertial = Inertial.from_geometry(
        Box((SASH_WIDTH, SASH_DEPTH + 0.012, SASH_HEIGHT + RUNNER_HEIGHT + GUIDE_HEIGHT)),
        mass=18.0,
    )
    _add_glazed_sash(
        fixed_sash,
        width=SASH_WIDTH,
        height=SASH_HEIGHT,
        depth=SASH_DEPTH,
        left_stile=0.052,
        right_stile=0.062,
        top_rail=0.056,
        bottom_rail=0.062,
        meeting_side="right",
        frame_material=sash_satin,
        bead_material=bead_matte,
        glass_material=glass_low_e,
        seal_material=seal_black,
        runner_material=track_satin,
        hardware_material=hardware_satin,
        add_keeper=True,
    )

    moving_sash = model.part("moving_sash")
    moving_sash.inertial = Inertial.from_geometry(
        Box((SASH_WIDTH, SASH_DEPTH + 0.012, SASH_HEIGHT + RUNNER_HEIGHT + GUIDE_HEIGHT)),
        mass=18.5,
    )
    _add_glazed_sash(
        moving_sash,
        width=SASH_WIDTH,
        height=SASH_HEIGHT,
        depth=SASH_DEPTH,
        left_stile=0.062,
        right_stile=0.052,
        top_rail=0.056,
        bottom_rail=0.062,
        meeting_side="left",
        frame_material=sash_satin,
        bead_material=bead_matte,
        glass_material=glass_low_e,
        seal_material=seal_black,
        runner_material=track_satin,
        hardware_material=hardware_satin,
        add_keeper=False,
    )

    latch = model.part("latch")
    latch.inertial = Inertial.from_geometry(
        Box((0.040, 0.020, 0.110)),
        mass=0.15,
        origin=Origin(xyz=(0.0, -0.008, -0.028)),
    )
    latch.visual(
        Box((0.032, 0.004, 0.096)),
        origin=Origin(xyz=(0.0, -0.002, -0.018)),
        material=hardware_satin,
        name="escutcheon",
    )
    latch.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_satin,
        name="pivot_boss",
    )
    latch.visual(
        Box((0.012, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, -0.012, -0.030)),
        material=hardware_satin,
        name="thumbturn",
    )
    latch.visual(
        Box((0.020, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.012, -0.062)),
        material=hardware_satin,
        name="thumb_pad",
    )

    model.articulation(
        "frame_to_fixed_sash",
        ArticulationType.FIXED,
        parent=frame,
        child=fixed_sash,
        origin=Origin(xyz=(FIXED_SASH_X, TRACK_CENTER_Y, SASH_CENTER_Z)),
    )
    model.articulation(
        "frame_to_moving_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=moving_sash,
        origin=Origin(xyz=(MOVING_SASH_X_CLOSED, -TRACK_CENTER_Y, SASH_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=-MOVING_SASH_TRAVEL,
            upper=0.0,
        ),
    )
    model.articulation(
        "moving_sash_to_latch",
        ArticulationType.REVOLUTE,
        parent=moving_sash,
        child=latch,
        origin=Origin(xyz=(-0.352, -SASH_DEPTH / 2.0, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-1.05,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    frame = object_model.get_part("frame")
    fixed_sash = object_model.get_part("fixed_sash")
    moving_sash = object_model.get_part("moving_sash")
    latch = object_model.get_part("latch")

    fixed_mount = object_model.get_articulation("frame_to_fixed_sash")
    slide = object_model.get_articulation("frame_to_moving_sash")
    latch_turn = object_model.get_articulation("moving_sash_to_latch")

    left_jamb = frame.get_visual("left_jamb")
    right_jamb = frame.get_visual("right_jamb")
    outer_track = frame.get_visual("outer_track_rail")
    inner_track = frame.get_visual("inner_track_rail")
    outer_head_lip = frame.get_visual("outer_head_lip")
    inner_head_lip = frame.get_visual("inner_head_lip")

    fixed_outer_stile = fixed_sash.get_visual("outer_stile")
    fixed_runner_left = fixed_sash.get_visual("runner_left")
    fixed_guide_right = fixed_sash.get_visual("guide_right")
    moving_meeting_stile = moving_sash.get_visual("meeting_stile")
    moving_runner_right = moving_sash.get_visual("runner_right")
    moving_guide_left = moving_sash.get_visual("guide_left")
    latch_escutcheon = latch.get_visual("escutcheon")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_isolated_parts(max_pose_samples=18, name="sampled_pose_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.check(
        "fixed_sash_mount_type",
        fixed_mount.articulation_type == ArticulationType.FIXED,
        details=f"Expected fixed sash mount, got {fixed_mount.articulation_type!r}.",
    )
    ctx.check(
        "moving_sash_slide_type_and_axis",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"Expected x-axis prismatic slide, got type={slide.articulation_type!r}, axis={slide.axis!r}.",
    )
    ctx.check(
        "latch_turn_type_and_axis",
        latch_turn.articulation_type == ArticulationType.REVOLUTE and tuple(latch_turn.axis) == (0.0, 1.0, 0.0),
        details=f"Expected y-axis revolute latch, got type={latch_turn.articulation_type!r}, axis={latch_turn.axis!r}.",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is None:
        ctx.fail("frame_aabb_available", "Frame AABB could not be resolved.")
    else:
        size_x, size_y, size_z = _aabb_size(frame_aabb)
        ctx.check(
            "window_realistic_scale",
            1.55 <= size_x <= 1.65 and 0.13 <= size_y <= 0.15 and 1.24 <= size_z <= 1.28,
            details=f"Observed size = ({size_x:.3f}, {size_y:.3f}, {size_z:.3f}) m.",
        )

    ctx.expect_contact(fixed_sash, frame, name="fixed_sash_contacts_frame")
    ctx.expect_contact(moving_sash, frame, name="moving_sash_contacts_frame")
    ctx.expect_contact(
        latch,
        moving_sash,
        elem_a=latch_escutcheon,
        elem_b=moving_meeting_stile,
        name="latch_plate_contacts_moving_sash",
    )
    ctx.expect_contact(
        fixed_sash,
        frame,
        elem_a=fixed_runner_left,
        elem_b=outer_track,
        name="fixed_runner_guided_on_outer_track",
    )
    ctx.expect_contact(
        fixed_sash,
        frame,
        elem_a=fixed_guide_right,
        elem_b=outer_head_lip,
        name="fixed_top_guide_captured_by_outer_head_lip",
    )
    ctx.expect_contact(
        moving_sash,
        frame,
        elem_a=moving_runner_right,
        elem_b=inner_track,
        name="moving_runner_guided_on_inner_track",
    )
    ctx.expect_contact(
        moving_sash,
        frame,
        elem_a=moving_guide_left,
        elem_b=inner_head_lip,
        name="moving_top_guide_captured_by_inner_head_lip",
    )
    ctx.expect_gap(
        fixed_sash,
        moving_sash,
        axis="y",
        min_gap=0.004,
        max_gap=0.008,
        name="meeting_stile_seal_break_gap",
    )
    ctx.expect_overlap(
        fixed_sash,
        moving_sash,
        axes="x",
        min_overlap=0.10,
        name="closed_meeting_stiles_overlap_in_x",
    )
    ctx.expect_gap(
        fixed_sash,
        frame,
        axis="x",
        positive_elem=fixed_outer_stile,
        negative_elem=left_jamb,
        max_gap=0.001,
        max_penetration=0.0,
        name="fixed_sash_seats_to_left_jamb",
    )
    ctx.expect_gap(
        frame,
        moving_sash,
        axis="x",
        positive_elem=right_jamb,
        max_gap=0.001,
        max_penetration=0.0,
        name="closed_moving_sash_seats_to_right_jamb",
    )

    slide_limits = slide.motion_limits
    if slide_limits is None or slide_limits.lower is None or slide_limits.upper is None:
        ctx.fail("slide_limits_present", "Sliding sash requires finite travel limits.")
    else:
        moving_rest_pos = ctx.part_world_position(moving_sash)
        moving_open_pos = None
        with ctx.pose({slide: slide_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="slide_lower_no_floating")
            ctx.expect_contact(
                moving_sash,
                frame,
                elem_a=moving_runner_right,
                elem_b=inner_track,
                name="open_runner_stays_on_inner_track",
            )
            ctx.expect_contact(
                moving_sash,
                frame,
                elem_a=moving_guide_left,
                elem_b=inner_head_lip,
                name="open_top_guide_stays_in_inner_channel",
            )
            ctx.expect_gap(
                frame,
                moving_sash,
                axis="x",
                positive_elem=right_jamb,
                min_gap=0.52,
                max_gap=0.56,
                name="open_clear_width_at_right_side",
            )
            moving_open_pos = ctx.part_world_position(moving_sash)
        with ctx.pose({slide: slide_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="slide_upper_no_floating")
            ctx.expect_gap(
                frame,
                moving_sash,
                axis="x",
                positive_elem=right_jamb,
                max_gap=0.001,
                max_penetration=0.0,
                name="slide_upper_returns_to_closed_jamb_contact",
            )
        if moving_rest_pos is None or moving_open_pos is None:
            ctx.fail("slide_positions_available", "Could not resolve moving sash positions for travel check.")
        else:
            ctx.check(
                "moving_sash_travel_amount",
                moving_open_pos[0] <= moving_rest_pos[0] - 0.52,
                details=(
                    f"Rest x={moving_rest_pos[0]:.3f}, "
                    f"open x={moving_open_pos[0]:.3f}, "
                    f"travel={moving_rest_pos[0] - moving_open_pos[0]:.3f}."
                ),
            )

    latch_limits = latch_turn.motion_limits
    if latch_limits is None or latch_limits.lower is None or latch_limits.upper is None:
        ctx.fail("latch_limits_present", "Latch requires finite turn limits.")
    else:
        latch_rest_aabb = ctx.part_world_aabb(latch)
        if latch_rest_aabb is None:
            ctx.fail("latch_rest_aabb_available", "Latch AABB could not be resolved at rest.")
        else:
            rest_dx, _, _ = _aabb_size(latch_rest_aabb)
            with ctx.pose({latch_turn: latch_limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name="latch_lower_no_overlap")
                ctx.fail_if_isolated_parts(name="latch_lower_no_floating")
                ctx.expect_contact(
                    latch,
                    moving_sash,
                    elem_a=latch_escutcheon,
                    elem_b=moving_meeting_stile,
                    name="latch_stays_mounted_when_unlatched",
                )
                latch_open_aabb = ctx.part_world_aabb(latch)
                if latch_open_aabb is None:
                    ctx.fail("latch_open_aabb_available", "Latch AABB could not be resolved when unlatched.")
                else:
                    open_dx, _, _ = _aabb_size(latch_open_aabb)
                    ctx.check(
                        "latch_turn_changes_plan_extent",
                        open_dx >= rest_dx + 0.02,
                        details=f"Rest dx={rest_dx:.3f}, open dx={open_dx:.3f}.",
                    )
            with ctx.pose({latch_turn: latch_limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name="latch_upper_no_overlap")
                ctx.fail_if_isolated_parts(name="latch_upper_no_floating")
                ctx.expect_contact(
                    latch,
                    moving_sash,
                    elem_a=latch_escutcheon,
                    elem_b=moving_meeting_stile,
                    name="latch_stays_mounted_at_upper_limit",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
