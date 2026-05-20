from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


FRAME_W = 1.20
FRAME_H = 1.00
FRAME_D = 0.12
FRAME_BAR = 0.06
OPEN_W = FRAME_W - 2.0 * FRAME_BAR
OPEN_H = FRAME_H - 2.0 * FRAME_BAR

SASH_W = 0.62
SASH_H = 0.84
SASH_D = 0.026
SASH_HALF_W = SASH_W / 2.0

CLOSED_SASH_X = -0.215
SASH_Y = -0.001
SASH_TRAVEL = 0.43


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_sliding_calibration_window")

    anodized = model.material("anodized_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.28, 0.30, 0.33, 1.0))
    glass = model.material("instrument_glass", rgba=(0.73, 0.84, 0.92, 0.35))
    seal = model.material("seal_black", rgba=(0.10, 0.11, 0.12, 1.0))
    mark = model.material("index_white", rgba=(0.94, 0.95, 0.97, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_BAR, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(-0.57, 0.0, 0.0)),
        material=anodized,
        name="left_jamb",
    )
    frame.visual(
        Box((FRAME_BAR, FRAME_D, FRAME_H)),
        origin=Origin(xyz=(0.57, 0.0, 0.0)),
        material=anodized,
        name="right_jamb",
    )
    frame.visual(
        Box((FRAME_W, FRAME_D, FRAME_BAR)),
        origin=Origin(xyz=(0.0, 0.0, -0.47)),
        material=anodized,
        name="sill",
    )
    frame.visual(
        Box((FRAME_W, FRAME_D, FRAME_BAR)),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=anodized,
        name="head",
    )

    frame.visual(
        Box((OPEN_W, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, SASH_Y, -0.440)),
        material=dark_anodized,
        name="lower_datum_way",
    )
    frame.visual(
        Box((OPEN_W, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, SASH_Y, 0.440)),
        material=dark_anodized,
        name="upper_datum_way",
    )
    frame.visual(
        Box((0.30, 0.010, 0.020)),
        origin=Origin(xyz=(-0.39, 0.018, -0.426)),
        material=anodized,
        name="lower_front_lip_left",
    )
    frame.visual(
        Box((0.30, 0.010, 0.020)),
        origin=Origin(xyz=(0.39, 0.018, -0.426)),
        material=anodized,
        name="lower_front_lip_right",
    )
    frame.visual(
        Box((OPEN_W, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.018, 0.426)),
        material=anodized,
        name="upper_front_lip",
    )

    frame.visual(
        Box((0.015, 0.028, 0.20)),
        origin=Origin(xyz=(-0.5325, SASH_Y, 0.0)),
        material=dark_anodized,
        name="closed_datum_stop",
    )
    frame.visual(
        Box((0.015, 0.028, 0.20)),
        origin=Origin(xyz=(0.5325, SASH_Y, 0.0)),
        material=dark_anodized,
        name="open_travel_stop",
    )

    frame.visual(
        Box((0.018, 0.020, 0.18)),
        origin=Origin(xyz=(-0.531, SASH_Y, 0.0)),
        material=anodized,
        name="left_reference_pad",
    )
    frame.visual(
        Box((0.52, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, 0.025, -0.424)),
        material=dark_anodized,
        name="scale_strip",
    )
    for idx, x_pos in enumerate((-0.20, -0.10, 0.0, 0.10, 0.20)):
        frame.visual(
            Box((0.004, 0.004, 0.016)),
            origin=Origin(xyz=(x_pos, 0.025, -0.421)),
            material=mark,
            name=f"index_mark_{idx}",
        )

    frame.visual(
        Box((0.018, 0.024, 0.09)),
        origin=Origin(xyz=(0.531, SASH_Y, 0.23)),
        material=anodized,
        name="right_adjuster_block",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(0.531, SASH_Y, 0.279)),
        material=seal,
        name="right_adjuster_screw",
    )

    sash = model.part("sash")
    sash.visual(
        Box((0.06, SASH_D, SASH_H)),
        origin=Origin(xyz=(-0.28, 0.0, 0.0)),
        material=anodized,
        name="left_stile",
    )
    sash.visual(
        Box((0.06, SASH_D, SASH_H)),
        origin=Origin(xyz=(0.28, 0.0, 0.0)),
        material=anodized,
        name="right_stile",
    )
    sash.visual(
        Box((SASH_W, SASH_D, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.39)),
        material=anodized,
        name="bottom_rail",
    )
    sash.visual(
        Box((SASH_W, SASH_D, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=anodized,
        name="top_rail",
    )
    sash.visual(
        Box((0.50, 0.006, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="glazing",
    )

    sash.visual(
        Box((0.46, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.001, -0.428)),
        material=seal,
        name="lower_guide_land",
    )
    sash.visual(
        Box((0.46, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.001, 0.428)),
        material=seal,
        name="upper_guide_land",
    )
    sash.visual(
        Box((0.012, 0.028, 0.20)),
        origin=Origin(xyz=(-0.304, 0.0, 0.0)),
        material=seal,
        name="left_stop_pad",
    )
    sash.visual(
        Box((0.012, 0.028, 0.20)),
        origin=Origin(xyz=(0.304, 0.0, 0.0)),
        material=seal,
        name="right_stop_pad",
    )
    sash.visual(
        Box((0.06, 0.022, 0.022)),
        origin=Origin(xyz=(-0.18, 0.0, -0.389)),
        material=dark_anodized,
        name="left_preload_block",
    )
    sash.visual(
        Box((0.06, 0.022, 0.022)),
        origin=Origin(xyz=(0.18, 0.0, -0.389)),
        material=dark_anodized,
        name="right_preload_block",
    )
    sash.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(-0.18, 0.0, -0.374)),
        material=seal,
        name="left_preload_screw",
    )
    sash.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(0.18, 0.0, -0.374)),
        material=seal,
        name="right_preload_screw",
    )
    sash.visual(
        Box((0.10, 0.010, 0.28)),
        origin=Origin(xyz=(0.23, 0.018, 0.0)),
        material=dark_anodized,
        name="pull_rail",
    )
    sash.visual(
        Box((0.012, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, 0.016, -0.421)),
        material=mark,
        name="index_pointer",
    )

    model.articulation(
        "frame_to_sash_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(CLOSED_SASH_X, SASH_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
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
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    slide = object_model.get_articulation("frame_to_sash_slide")

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

    ctx.check(
        "sash_slide_is_prismatic_x",
        slide.articulation_type == ArticulationType.PRISMATIC and slide.axis == (1.0, 0.0, 0.0),
        details=f"Expected x-axis prismatic slide, got type={slide.articulation_type}, axis={slide.axis}",
    )
    limits = slide.motion_limits
    ctx.check(
        "sash_slide_limits_are_repeatable",
        limits is not None
        and limits.lower == 0.0
        and limits.upper == SASH_TRAVEL,
        details=f"Unexpected slider limits: {limits}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            frame,
            sash,
            elem_a="closed_datum_stop",
            elem_b="left_stop_pad",
            name="closed_pose_seats_on_datum_stop",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="lower_guide_land",
            elem_b="lower_datum_way",
            name="closed_pose_lower_guide_bears_on_way",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="upper_guide_land",
            elem_b="upper_datum_way",
            name="closed_pose_upper_guide_bears_on_way",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="lower_front_lip_left",
            negative_elem="lower_guide_land",
            min_gap=0.0015,
            max_gap=0.0025,
            name="closed_pose_lower_front_gap_is_controlled",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="upper_front_lip",
            negative_elem="upper_guide_land",
            min_gap=0.0015,
            max_gap=0.0025,
            name="closed_pose_upper_front_gap_is_controlled",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="yz",
            margin=0.0,
            name="closed_pose_sash_stays_within_frame_envelope",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="x",
            inner_elem="index_pointer",
            outer_elem="scale_strip",
            margin=0.0,
            name="closed_pose_pointer_stays_over_scale",
        )

    with ctx.pose({slide: SASH_TRAVEL}):
        ctx.expect_contact(
            frame,
            sash,
            elem_a="open_travel_stop",
            elem_b="right_stop_pad",
            name="open_pose_seats_on_travel_stop",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="lower_guide_land",
            elem_b="lower_datum_way",
            name="open_pose_lower_guide_bears_on_way",
        )
        ctx.expect_contact(
            sash,
            frame,
            elem_a="upper_guide_land",
            elem_b="upper_datum_way",
            name="open_pose_upper_guide_bears_on_way",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="lower_front_lip_right",
            negative_elem="lower_guide_land",
            min_gap=0.0015,
            max_gap=0.0025,
            name="open_pose_lower_front_gap_is_controlled",
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            positive_elem="upper_front_lip",
            negative_elem="upper_guide_land",
            min_gap=0.0015,
            max_gap=0.0025,
            name="open_pose_upper_front_gap_is_controlled",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="yz",
            margin=0.0,
            name="open_pose_sash_stays_within_frame_envelope",
        )
        ctx.expect_within(
            sash,
            frame,
            axes="x",
            inner_elem="index_pointer",
            outer_elem="scale_strip",
            margin=0.0,
            name="open_pose_pointer_stays_over_scale",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
