from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


FRAME_W = 1.24
FRAME_H = 0.82
FRAME_D = 0.11
JAMB_W = 0.06
HEAD_H = 0.06
SILL_H = 0.06
OPEN_W = FRAME_W - 2.0 * JAMB_W
OPEN_H = FRAME_H - HEAD_H - SILL_H

SASH_W = 0.59
SASH_H = 0.66
SASH_D = 0.032
TRACK_Y = 0.033
TRACK_RUNNER_W = 0.014
TRACK_RUNNER_H = 0.012
TRACK_LIP_H = 0.028
TRACK_CENTER_LIP_W = 0.012
TRACK_OUTER_LIP_W = 0.008

FIXED_X = -0.265
SLIDER_CLOSED_X = 0.265
SLIDER_TRAVEL = 0.50


def _add_fasteners(
    part,
    *,
    xs: tuple[float, ...],
    zs: tuple[float, ...],
    y: float,
    radius: float,
    length: float,
    material,
    prefix: str,
) -> None:
    idx = 0
    for x in xs:
        for z in zs:
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=material,
                name=f"{prefix}_{idx}",
            )
            idx += 1


def _add_corner_plates(part, *, width: float, height: float, y: float, material, prefix: str) -> None:
    plate_w = 0.085
    plate_h = 0.085
    plate_t = 0.004
    positions = (
        (-width / 2.0 + plate_w / 2.0 + 0.018, height / 2.0 - plate_h / 2.0 - 0.018),
        (width / 2.0 - plate_w / 2.0 - 0.018, height / 2.0 - plate_h / 2.0 - 0.018),
        (-width / 2.0 + plate_w / 2.0 + 0.018, -height / 2.0 + plate_h / 2.0 + 0.018),
        (width / 2.0 - plate_w / 2.0 - 0.018, -height / 2.0 + plate_h / 2.0 + 0.018),
    )
    for idx, (x, z) in enumerate(positions):
        part.visual(
            Box((plate_w, plate_t, plate_h)),
            origin=Origin(xyz=(x, y, z)),
            material=material,
            name=f"{prefix}_{idx}",
        )


def _build_sash(
    part,
    *,
    meeting_left: bool,
    frame_material,
    reinforcement_material,
    seal_material,
    glide_material,
    glass_material,
    keeper_material=None,
    moving: bool = False,
) -> None:
    outer_stile_w = 0.050
    meeting_stile_w = 0.065 if moving else 0.060
    rail_h = 0.050
    glass_y = -0.003 if moving else 0.003
    glass_w = SASH_W - outer_stile_w - meeting_stile_w
    glass_h = SASH_H - 2.0 * rail_h

    left_w = meeting_stile_w if meeting_left else outer_stile_w
    right_w = outer_stile_w if meeting_left else meeting_stile_w

    part.visual(
        Box((left_w, SASH_D, SASH_H)),
        origin=Origin(xyz=(-SASH_W / 2.0 + left_w / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="left_stile",
    )
    part.visual(
        Box((right_w, SASH_D, SASH_H)),
        origin=Origin(xyz=(SASH_W / 2.0 - right_w / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="right_stile",
    )
    part.visual(
        Box((SASH_W - left_w - right_w, SASH_D, rail_h)),
        origin=Origin(xyz=((right_w - left_w) / 2.0, 0.0, SASH_H / 2.0 - rail_h / 2.0)),
        material=frame_material,
        name="top_rail",
    )
    part.visual(
        Box((SASH_W - left_w - right_w, SASH_D, rail_h)),
        origin=Origin(xyz=((right_w - left_w) / 2.0, 0.0, -SASH_H / 2.0 + rail_h / 2.0)),
        material=frame_material,
        name="bottom_rail",
    )
    part.visual(
        Box((glass_w, 0.008, glass_h)),
        origin=Origin(xyz=((right_w - left_w) / 2.0, glass_y, 0.0)),
        material=glass_material,
        name="glass",
    )
    part.visual(
        Box((glass_w + 0.012, 0.004, glass_h + 0.012)),
        origin=Origin(xyz=((right_w - left_w) / 2.0, glass_y + 0.002, 0.0)),
        material=seal_material,
        name="glazing_gasket",
    )

    interlock_w = 0.016
    interlock_d = 0.012
    interlock_h = 0.58
    seal_w = 0.006
    if meeting_left:
        interlock_x = -SASH_W / 2.0 + meeting_stile_w + interlock_w / 2.0
        interlock_y = -0.010
        seal_x = interlock_x + 0.007
        seal_y = -0.014
    else:
        interlock_x = SASH_W / 2.0 - meeting_stile_w - interlock_w / 2.0
        interlock_y = 0.010
        seal_x = interlock_x - 0.007
        seal_y = 0.014
    part.visual(
        Box((interlock_w, interlock_d, interlock_h)),
        origin=Origin(xyz=(interlock_x, interlock_y, 0.0)),
        material=reinforcement_material,
        name="interlock_fin",
    )
    part.visual(
        Box((seal_w, 0.004, 0.56)),
        origin=Origin(xyz=(seal_x, seal_y, 0.0)),
        material=seal_material,
        name="meeting_seal",
    )

    shoe_x = 0.175
    shoe_z = SASH_H / 2.0 + 0.001
    part.visual(
        Box((0.085, TRACK_RUNNER_W, 0.014)),
        origin=Origin(xyz=(-shoe_x, 0.0, -shoe_z)),
        material=glide_material,
        name="bottom_glide_left",
    )
    part.visual(
        Box((0.085, TRACK_RUNNER_W, 0.014)),
        origin=Origin(xyz=(shoe_x, 0.0, -shoe_z)),
        material=glide_material,
        name="bottom_glide_right",
    )
    part.visual(
        Box((0.070, TRACK_RUNNER_W, 0.014)),
        origin=Origin(xyz=(-shoe_x, 0.0, shoe_z)),
        material=glide_material,
        name="top_guide_left",
    )
    part.visual(
        Box((0.070, TRACK_RUNNER_W, 0.014)),
        origin=Origin(xyz=(shoe_x, 0.0, shoe_z)),
        material=glide_material,
        name="top_guide_right",
    )

    _add_corner_plates(
        part,
        width=SASH_W,
        height=SASH_H,
        y=SASH_D / 2.0 + 0.002,
        material=reinforcement_material,
        prefix="front_plate",
    )
    _add_corner_plates(
        part,
        width=SASH_W,
        height=SASH_H,
        y=-SASH_D / 2.0 - 0.002,
        material=reinforcement_material,
        prefix="rear_plate",
    )

    if moving:
        part.visual(
            Box((0.066, 0.012, 0.120)),
            origin=Origin(xyz=(-SASH_W / 2.0 + 0.034, SASH_D / 2.0 + 0.006, 0.0)),
            material=glide_material,
            name="pull_handle",
        )
        part.visual(
            Box((0.046, 0.008, 0.085)),
            origin=Origin(xyz=(-SASH_W / 2.0 + 0.036, SASH_D / 2.0 + 0.016, 0.0)),
            material=reinforcement_material,
            name="handle_rib",
        )
        part.visual(
            Box((0.040, 0.010, 0.060)),
            origin=Origin(xyz=(-SASH_W / 2.0 + 0.040, SASH_D / 2.0 + 0.005, -0.060)),
            material=reinforcement_material,
            name="latch_mount",
        )
        part.visual(
            Box((0.078, 0.018, 0.024)),
            origin=Origin(xyz=(-0.175, 0.0, -SASH_H / 2.0 + 0.008)),
            material=glide_material,
            name="left_roller_housing",
        )
        part.visual(
            Box((0.078, 0.018, 0.024)),
            origin=Origin(xyz=(0.175, 0.0, -SASH_H / 2.0 + 0.008)),
            material=glide_material,
            name="right_roller_housing",
        )
    elif keeper_material is not None:
        part.visual(
            Box((0.042, 0.010, 0.050)),
            origin=Origin(xyz=(SASH_W / 2.0 - 0.040, 0.020, -0.060)),
            material=keeper_material,
            name="keeper_block",
        )
        part.visual(
            Box((0.022, 0.006, 0.022)),
            origin=Origin(xyz=(SASH_W / 2.0 - 0.030, 0.026, -0.060)),
            material=reinforcement_material,
            name="keeper_lip",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_sliding_window", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.31, 0.33, 0.36, 1.0))
    liner_black = model.material("liner_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.60, 0.72, 0.80, 0.34))
    galvanized = model.material("galvanized", rgba=(0.75, 0.77, 0.79, 1.0))
    molded_handle = model.material("molded_handle", rgba=(0.18, 0.19, 0.20, 1.0))
    reinforcement = model.material("reinforcement", rgba=(0.42, 0.44, 0.47, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(Box((FRAME_W, FRAME_D, FRAME_H)), mass=32.0)

    frame.visual(
        Box((JAMB_W, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(-FRAME_W / 2.0 + JAMB_W / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((JAMB_W, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(FRAME_W / 2.0 - JAMB_W / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((OPEN_W, FRAME_D, HEAD_H)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_H / 2.0 - HEAD_H / 2.0)),
        material=frame_paint,
        name="head",
    )
    frame.visual(
        Box((OPEN_W, FRAME_D, SILL_H)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_H / 2.0 + SILL_H / 2.0)),
        material=frame_paint,
        name="sill",
    )

    flange_t = 0.006
    flange_y = FRAME_D / 2.0 - flange_t / 2.0
    for y_sign, prefix in ((1.0, "front"), (-1.0, "rear")):
        y = y_sign * flange_y
        frame.visual(
            Box((FRAME_W + 0.04, flange_t, 0.08)),
            origin=Origin(xyz=(0.0, y, FRAME_H / 2.0 - 0.04)),
            material=frame_paint,
            name=f"{prefix}_top_flange",
        )
        frame.visual(
            Box((FRAME_W + 0.04, flange_t, 0.08)),
            origin=Origin(xyz=(0.0, y, -FRAME_H / 2.0 + 0.04)),
            material=frame_paint,
            name=f"{prefix}_bottom_flange",
        )
        frame.visual(
            Box((0.08, flange_t, FRAME_H - 0.12)),
            origin=Origin(xyz=(-FRAME_W / 2.0 + 0.04, y, 0.0)),
            material=frame_paint,
            name=f"{prefix}_left_flange",
        )
        frame.visual(
            Box((0.08, flange_t, FRAME_H - 0.12)),
            origin=Origin(xyz=(FRAME_W / 2.0 - 0.04, y, 0.0)),
            material=frame_paint,
            name=f"{prefix}_right_flange",
        )

    runner_z = -OPEN_H / 2.0 + TRACK_RUNNER_H / 2.0
    guide_z = OPEN_H / 2.0 - TRACK_RUNNER_H / 2.0
    for y, prefix in ((-TRACK_Y, "rear"), (TRACK_Y, "front")):
        frame.visual(
            Box((OPEN_W - 0.10, TRACK_RUNNER_W, TRACK_RUNNER_H)),
            origin=Origin(xyz=(0.0, y, runner_z)),
            material=liner_black,
            name=f"{prefix}_runner",
        )
        frame.visual(
            Box((OPEN_W - 0.10, TRACK_RUNNER_W, TRACK_RUNNER_H)),
            origin=Origin(xyz=(0.0, y, guide_z)),
            material=liner_black,
            name=f"{prefix}_head_guide",
        )

    track_lip_z = -OPEN_H / 2.0 + TRACK_LIP_H / 2.0
    upper_lip_z = OPEN_H / 2.0 - TRACK_LIP_H / 2.0
    for z, prefix in ((track_lip_z, "sill"), (upper_lip_z, "head")):
        frame.visual(
            Box((OPEN_W - 0.08, TRACK_OUTER_LIP_W, TRACK_LIP_H)),
            origin=Origin(xyz=(0.0, TRACK_Y + 0.022, z)),
            material=frame_paint,
            name=f"{prefix}_front_outer_lip",
        )
        frame.visual(
            Box((OPEN_W - 0.08, TRACK_CENTER_LIP_W, TRACK_LIP_H)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=frame_paint,
            name=f"{prefix}_center_lip",
        )
        frame.visual(
            Box((OPEN_W - 0.08, TRACK_OUTER_LIP_W, TRACK_LIP_H)),
            origin=Origin(xyz=(0.0, -TRACK_Y - 0.022, z)),
            material=frame_paint,
            name=f"{prefix}_rear_outer_lip",
        )

    frame.visual(
        Box((0.16, 0.012, 0.020)),
        origin=Origin(xyz=(-0.42, FRAME_D / 2.0 - 0.010, -FRAME_H / 2.0 + 0.070)),
        material=liner_black,
        name="drain_cap_left",
    )
    frame.visual(
        Box((0.16, 0.012, 0.020)),
        origin=Origin(xyz=(0.42, FRAME_D / 2.0 - 0.010, -FRAME_H / 2.0 + 0.070)),
        material=liner_black,
        name="drain_cap_right",
    )

    _add_corner_plates(
        frame,
        width=FRAME_W - 0.04,
        height=FRAME_H - 0.04,
        y=FRAME_D / 2.0 + 0.002,
        material=reinforcement,
        prefix="frame_front_plate",
    )
    _add_corner_plates(
        frame,
        width=FRAME_W - 0.04,
        height=FRAME_H - 0.04,
        y=-FRAME_D / 2.0 - 0.002,
        material=reinforcement,
        prefix="frame_rear_plate",
    )
    _add_fasteners(
        frame,
        xs=(-0.54, 0.54),
        zs=(-0.33, 0.33),
        y=FRAME_D / 2.0 + 0.002,
        radius=0.007,
        length=0.006,
        material=galvanized,
        prefix="front_bolt",
    )
    _add_fasteners(
        frame,
        xs=(-0.54, 0.54),
        zs=(-0.34, 0.34),
        y=-FRAME_D / 2.0 - 0.003,
        radius=0.007,
        length=0.006,
        material=galvanized,
        prefix="rear_bolt",
    )

    fixed_lite = model.part("fixed_lite")
    fixed_lite.inertial = Inertial.from_geometry(Box((SASH_W, SASH_D, SASH_H)), mass=12.0)
    _build_sash(
        fixed_lite,
        meeting_left=False,
        frame_material=frame_paint,
        reinforcement_material=reinforcement,
        seal_material=rubber_black,
        glide_material=liner_black,
        glass_material=glass_tint,
        keeper_material=galvanized,
        moving=False,
    )

    sliding_sash = model.part("sliding_sash")
    sliding_sash.inertial = Inertial.from_geometry(Box((SASH_W, SASH_D, SASH_H)), mass=13.0)
    _build_sash(
        sliding_sash,
        meeting_left=True,
        frame_material=frame_paint,
        reinforcement_material=reinforcement,
        seal_material=rubber_black,
        glide_material=molded_handle,
        glass_material=glass_tint,
        moving=True,
    )

    model.articulation(
        "frame_to_fixed_lite",
        ArticulationType.FIXED,
        parent=frame,
        child=fixed_lite,
        origin=Origin(xyz=(FIXED_X, -TRACK_Y, 0.0)),
    )
    model.articulation(
        "frame_to_sliding_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sliding_sash,
        origin=Origin(xyz=(SLIDER_CLOSED_X, TRACK_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.6,
            lower=-SLIDER_TRAVEL,
            upper=0.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    fixed_lite = object_model.get_part("fixed_lite")
    sliding_sash = object_model.get_part("sliding_sash")
    sash_slide = object_model.get_articulation("frame_to_sliding_sash")

    rear_runner = frame.get_visual("rear_runner")
    rear_head_guide = frame.get_visual("rear_head_guide")
    front_runner = frame.get_visual("front_runner")
    front_head_guide = frame.get_visual("front_head_guide")
    left_jamb = frame.get_visual("left_jamb")
    right_jamb = frame.get_visual("right_jamb")

    fixed_bottom_left = fixed_lite.get_visual("bottom_glide_left")
    fixed_bottom_right = fixed_lite.get_visual("bottom_glide_right")
    fixed_top_left = fixed_lite.get_visual("top_guide_left")
    fixed_keeper = fixed_lite.get_visual("keeper_block")
    slider_bottom_left = sliding_sash.get_visual("bottom_glide_left")
    slider_bottom_right = sliding_sash.get_visual("bottom_glide_right")
    slider_top_right = sliding_sash.get_visual("top_guide_right")
    slider_left_stile = sliding_sash.get_visual("left_stile")
    slider_right_stile = sliding_sash.get_visual("right_stile")
    slider_mount = sliding_sash.get_visual("latch_mount")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        fixed_lite,
        frame,
        elem_a=fixed_bottom_left,
        elem_b=rear_runner,
        name="fixed_left_glide_on_runner",
    )
    ctx.expect_contact(
        fixed_lite,
        frame,
        elem_a=fixed_bottom_right,
        elem_b=rear_runner,
        name="fixed_right_glide_on_runner",
    )
    ctx.expect_contact(
        fixed_lite,
        frame,
        elem_a=fixed_top_left,
        elem_b=rear_head_guide,
        name="fixed_top_guide_captured",
    )
    ctx.expect_contact(
        sliding_sash,
        frame,
        elem_a=slider_bottom_left,
        elem_b=front_runner,
        name="slider_left_glide_on_runner",
    )
    ctx.expect_contact(
        sliding_sash,
        frame,
        elem_a=slider_bottom_right,
        elem_b=front_runner,
        name="slider_right_glide_on_runner",
    )
    ctx.expect_contact(
        sliding_sash,
        frame,
        elem_a=slider_top_right,
        elem_b=front_head_guide,
        name="slider_top_guide_captured",
    )

    with ctx.pose({sash_slide: 0.0}):
        ctx.expect_contact(
            fixed_lite,
            frame,
            elem_a="left_stile",
            elem_b=left_jamb,
            name="fixed_sash_against_left_jamb",
        )
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a=slider_right_stile,
            elem_b=right_jamb,
            name="slider_closed_against_right_jamb",
        )
        ctx.expect_gap(
            sliding_sash,
            fixed_lite,
            axis="y",
            min_gap=0.010,
            max_gap=0.018,
            name="meeting_rails_have_legible_seal_gap",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_lite,
            axes="xz",
            min_overlap=0.045,
            name="closed_sashes_overlap_in_projection",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_lite,
            axes="xz",
            elem_a=slider_mount,
            elem_b=fixed_keeper,
            min_overlap=0.015,
            name="latch_zone_aligned_with_keeper",
        )

    slide_limits = sash_slide.motion_limits
    assert slide_limits is not None and slide_limits.lower is not None
    rest_pos = ctx.part_world_position(sliding_sash)
    assert rest_pos is not None
    with ctx.pose({sash_slide: slide_limits.lower}):
        open_pos = ctx.part_world_position(sliding_sash)
        assert open_pos is not None
        assert open_pos[0] < rest_pos[0] - 0.45
        ctx.fail_if_parts_overlap_in_current_pose(name="slider_open_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="slider_open_pose_no_floating")
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a=slider_bottom_left,
            elem_b=front_runner,
            name="slider_open_still_supported_left",
        )
        ctx.expect_contact(
            sliding_sash,
            frame,
            elem_a=slider_bottom_right,
            elem_b=front_runner,
            name="slider_open_still_supported_right",
        )
        ctx.expect_overlap(
            sliding_sash,
            fixed_lite,
            axes="xz",
            min_overlap=0.40,
            name="slider_open_stacks_over_fixed_lite",
        )
        ctx.expect_gap(
            sliding_sash,
            fixed_lite,
            axis="y",
            positive_elem=slider_left_stile,
            negative_elem=fixed_keeper,
            min_gap=0.010,
            name="open_sash_moves_latch_zone_clear_of_keeper",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
