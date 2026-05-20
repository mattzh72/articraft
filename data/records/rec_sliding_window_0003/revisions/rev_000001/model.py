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

FRAME_W = 1.56
FRAME_H = 1.24
FRAME_D = 0.108
JAMB_W = 0.082
HEAD_H = 0.084
OPEN_W = FRAME_W - (2.0 * JAMB_W)
OPEN_H = FRAME_H - (2.0 * HEAD_H)

FRONT_TRACK_Y = 0.015
REAR_TRACK_Y = -0.019
SASH_CLOSED_X = -0.340
SASH_BASE_Z = 0.102
SLIDE_TRAVEL = 0.530

SASH_W = 0.708
SASH_H = 0.996
SASH_T = 0.028
SASH_STILE_W = 0.044
SASH_BOTTOM_RAIL_H = 0.054
SASH_TOP_RAIL_H = 0.048

FIXED_LEFT = 0.006
FIXED_RIGHT = 0.690
FIXED_W = FIXED_RIGHT - FIXED_LEFT
FIXED_STILE_W = 0.040
FIXED_RAIL_H = 0.060


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    rpy: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sliding_window", assets=ASSETS)

    frame_matte = model.material("frame_matte", rgba=(0.67, 0.69, 0.71, 1.0))
    frame_satin = model.material("frame_satin", rgba=(0.78, 0.79, 0.80, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    roller_nylon = model.material("roller_nylon", rgba=(0.84, 0.84, 0.82, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.71, 0.82, 0.88, 0.28))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, FRAME_D + 0.008, FRAME_H)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_H * 0.5)),
    )

    _add_box(
        frame,
        "left_jamb",
        (JAMB_W, FRAME_D, FRAME_H),
        (-FRAME_W * 0.5 + JAMB_W * 0.5, 0.0, FRAME_H * 0.5),
        frame_matte,
    )
    _add_box(
        frame,
        "right_jamb",
        (JAMB_W, FRAME_D, FRAME_H),
        (FRAME_W * 0.5 - JAMB_W * 0.5, 0.0, FRAME_H * 0.5),
        frame_matte,
    )
    _add_box(
        frame,
        "head",
        (OPEN_W, FRAME_D, HEAD_H),
        (0.0, 0.0, FRAME_H - HEAD_H * 0.5),
        frame_matte,
    )
    _add_box(
        frame,
        "sill",
        (OPEN_W, FRAME_D, HEAD_H),
        (0.0, 0.0, HEAD_H * 0.5),
        frame_matte,
    )

    _add_box(
        frame,
        "left_jamb_face_cap",
        (JAMB_W - 0.014, 0.008, FRAME_H - 0.060),
        (-FRAME_W * 0.5 + JAMB_W * 0.5, FRAME_D * 0.5 + 0.004, FRAME_H * 0.5),
        frame_satin,
    )
    _add_box(
        frame,
        "right_jamb_face_cap",
        (JAMB_W - 0.014, 0.008, FRAME_H - 0.060),
        (FRAME_W * 0.5 - JAMB_W * 0.5, FRAME_D * 0.5 + 0.004, FRAME_H * 0.5),
        frame_satin,
    )
    _add_box(
        frame,
        "head_face_cap",
        (OPEN_W - 0.020, 0.008, HEAD_H - 0.020),
        (0.0, FRAME_D * 0.5 + 0.004, FRAME_H - HEAD_H * 0.5),
        frame_satin,
    )
    _add_box(
        frame,
        "sill_face_cap",
        (OPEN_W - 0.020, 0.008, HEAD_H - 0.020),
        (0.0, FRAME_D * 0.5 + 0.004, HEAD_H * 0.5),
        frame_satin,
    )

    _add_box(
        frame,
        "rear_track_base",
        (OPEN_W, 0.030, 0.010),
        (0.0, REAR_TRACK_Y, 0.089),
        frame_satin,
    )
    _add_box(
        frame,
        "rear_head_stop",
        (OPEN_W, 0.030, 0.016),
        (0.0, REAR_TRACK_Y, FRAME_H - HEAD_H - 0.008),
        frame_satin,
    )

    _add_box(
        frame,
        "front_track_base",
        (OPEN_W, 0.039, 0.012),
        (0.0, FRONT_TRACK_Y, 0.090),
        frame_satin,
    )
    _add_box(
        frame,
        "front_roller_rail",
        (OPEN_W, 0.010, 0.006),
        (0.0, FRONT_TRACK_Y, 0.099),
        frame_satin,
    )
    _add_box(
        frame,
        "front_track_backstop",
        (OPEN_W, 0.006, 0.024),
        (0.0, FRONT_TRACK_Y - 0.019, 0.096),
        frame_matte,
    )
    _add_box(
        frame,
        "front_track_lip",
        (OPEN_W, 0.006, 0.024),
        (0.0, FRONT_TRACK_Y + 0.020, 0.096),
        frame_satin,
    )
    _add_box(
        frame,
        "front_head_ceiling",
        (OPEN_W, 0.039, 0.028),
        (0.0, FRONT_TRACK_Y, 1.142),
        frame_satin,
    )
    _add_box(
        frame,
        "front_head_backstop",
        (OPEN_W, 0.006, 0.028),
        (0.0, FRONT_TRACK_Y - 0.019, 1.142),
        frame_matte,
    )
    _add_box(
        frame,
        "front_head_lip",
        (OPEN_W, 0.006, 0.028),
        (0.0, FRONT_TRACK_Y + 0.020, 1.142),
        frame_satin,
    )

    fixed_outer_center_x = (FIXED_LEFT + FIXED_RIGHT) * 0.5
    fixed_stile_center_z = 0.620
    fixed_stile_h = 0.982
    fixed_rail_center_x = 0.348
    fixed_rail_w = 0.664

    _add_box(
        frame,
        "fixed_meeting_stile",
        (FIXED_STILE_W, 0.020, fixed_stile_h),
        (FIXED_LEFT + FIXED_STILE_W * 0.5, REAR_TRACK_Y, fixed_stile_center_z),
        frame_satin,
    )
    _add_box(
        frame,
        "fixed_right_stile",
        (FIXED_STILE_W, 0.020, fixed_stile_h),
        (FIXED_RIGHT - FIXED_STILE_W * 0.5, REAR_TRACK_Y, fixed_stile_center_z),
        frame_satin,
    )
    _add_box(
        frame,
        "fixed_top_rail",
        (fixed_rail_w, 0.020, FIXED_RAIL_H),
        (fixed_rail_center_x, REAR_TRACK_Y, 1.126),
        frame_satin,
    )
    _add_box(
        frame,
        "fixed_bottom_rail",
        (fixed_rail_w, 0.020, FIXED_RAIL_H),
        (fixed_rail_center_x, REAR_TRACK_Y, 0.114),
        frame_satin,
    )
    _add_box(
        frame,
        "fixed_glass",
        (0.576, 0.008, 0.882),
        (fixed_outer_center_x, REAR_TRACK_Y, 0.620),
        glass_tint,
    )
    _add_box(
        frame,
        "fixed_glass_bead_left",
        (0.014, 0.012, 0.900),
        (0.053, REAR_TRACK_Y, 0.620),
        frame_satin,
    )
    _add_box(
        frame,
        "fixed_glass_bead_right",
        (0.014, 0.012, 0.900),
        (0.643, REAR_TRACK_Y, 0.620),
        frame_satin,
    )
    _add_box(
        frame,
        "fixed_glass_bead_top",
        (0.590, 0.012, 0.036),
        (fixed_outer_center_x, REAR_TRACK_Y, 1.079),
        frame_satin,
    )
    _add_box(
        frame,
        "fixed_glass_bead_bottom",
        (0.590, 0.012, 0.036),
        (fixed_outer_center_x, REAR_TRACK_Y, 0.161),
        frame_satin,
    )
    _add_box(
        frame,
        "fixed_glass_gasket_left",
        (0.006, 0.012, 0.888),
        (0.058, REAR_TRACK_Y, 0.620),
        gasket_dark,
    )
    _add_box(
        frame,
        "fixed_glass_gasket_right",
        (0.006, 0.012, 0.888),
        (0.638, REAR_TRACK_Y, 0.620),
        gasket_dark,
    )
    _add_box(
        frame,
        "fixed_glass_gasket_top",
        (0.580, 0.012, 0.010),
        (fixed_outer_center_x, REAR_TRACK_Y, 1.066),
        gasket_dark,
    )
    _add_box(
        frame,
        "fixed_glass_gasket_bottom",
        (0.580, 0.012, 0.010),
        (fixed_outer_center_x, REAR_TRACK_Y, 0.174),
        gasket_dark,
    )
    _add_box(
        frame,
        "fixed_meeting_seal",
        (0.014, 0.006, 0.900),
        (0.006, -0.006, 0.620),
        gasket_dark,
    )

    sash = model.part("sash")
    sash.inertial = Inertial.from_geometry(
        Box((SASH_W, SASH_T + 0.012, SASH_H + 0.030)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
    )

    _add_box(
        sash,
        "bottom_rail",
        (SASH_W, SASH_T, SASH_BOTTOM_RAIL_H),
        (0.0, 0.0, 0.039),
        frame_satin,
    )
    _add_box(
        sash,
        "top_rail",
        (SASH_W, SASH_T, SASH_TOP_RAIL_H),
        (0.0, 0.0, 0.972),
        frame_satin,
    )
    _add_box(
        sash,
        "left_stile",
        (SASH_STILE_W, SASH_T, 0.890),
        (-SASH_W * 0.5 + SASH_STILE_W * 0.5, 0.0, 0.506),
        frame_satin,
    )
    _add_box(
        sash,
        "right_stile",
        (SASH_STILE_W, SASH_T, 0.890),
        (SASH_W * 0.5 - SASH_STILE_W * 0.5, 0.0, 0.506),
        frame_satin,
    )
    _add_box(
        sash,
        "sash_glass",
        (0.584, 0.010, 0.822),
        (0.0, 0.0, 0.506),
        glass_tint,
    )
    _add_box(
        sash,
        "glass_bead_left",
        (0.018, 0.012, 0.840),
        (-0.301, 0.0, 0.506),
        frame_satin,
    )
    _add_box(
        sash,
        "glass_bead_right",
        (0.018, 0.012, 0.840),
        (0.301, 0.0, 0.506),
        frame_satin,
    )
    _add_box(
        sash,
        "glass_bead_top",
        (0.602, 0.012, 0.031),
        (0.0, 0.0, 0.932),
        frame_satin,
    )
    _add_box(
        sash,
        "glass_bead_bottom",
        (0.602, 0.012, 0.030),
        (0.0, 0.0, 0.081),
        frame_satin,
    )
    _add_box(
        sash,
        "glass_gasket_left",
        (0.006, 0.012, 0.832),
        (-0.292, 0.0, 0.506),
        gasket_dark,
    )
    _add_box(
        sash,
        "glass_gasket_right",
        (0.006, 0.012, 0.832),
        (0.292, 0.0, 0.506),
        gasket_dark,
    )
    _add_box(
        sash,
        "glass_gasket_top",
        (0.584, 0.012, 0.010),
        (0.0, 0.0, 0.917),
        gasket_dark,
    )
    _add_box(
        sash,
        "glass_gasket_bottom",
        (0.584, 0.012, 0.010),
        (0.0, 0.0, 0.095),
        gasket_dark,
    )

    _add_box(
        sash,
        "roller_housing_left",
        (0.044, 0.014, 0.020),
        (-0.220, 0.0, 0.020),
        hardware_dark,
    )
    _add_box(
        sash,
        "roller_housing_right",
        (0.044, 0.014, 0.020),
        (0.220, 0.0, 0.020),
        hardware_dark,
    )
    _add_cylinder(
        sash,
        "roller_left",
        0.010,
        0.010,
        (-0.220, 0.0, 0.010),
        (math.pi * 0.5, 0.0, 0.0),
        roller_nylon,
    )
    _add_cylinder(
        sash,
        "roller_right",
        0.010,
        0.010,
        (0.220, 0.0, 0.010),
        (math.pi * 0.5, 0.0, 0.0),
        roller_nylon,
    )
    _add_box(
        sash,
        "top_guide_left",
        (0.018, 0.018, 0.026),
        (-0.250, 0.0, 1.009),
        gasket_dark,
    )
    _add_box(
        sash,
        "top_guide_right",
        (0.018, 0.018, 0.026),
        (0.250, 0.0, 1.009),
        gasket_dark,
    )
    _add_box(
        sash,
        "interlock_fin",
        (0.018, 0.008, 0.900),
        (0.352, -0.011, 0.506),
        gasket_dark,
    )
    _add_box(
        sash,
        "pull_plate",
        (0.016, 0.004, 0.150),
        (0.315, 0.012, 0.555),
        hardware_dark,
    )
    _add_cylinder(
        sash,
        "pull_grip",
        0.006,
        0.094,
        (0.305, 0.011, 0.555),
        (0.0, 0.0, 0.0),
        frame_satin,
    )

    model.articulation(
        "sash_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(SASH_CLOSED_X, FRONT_TRACK_Y, SASH_BASE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    sash_slide = object_model.get_articulation("sash_slide")

    front_track_base = frame.get_visual("front_track_base")
    front_roller_rail = frame.get_visual("front_roller_rail")
    front_track_backstop = frame.get_visual("front_track_backstop")
    front_track_lip = frame.get_visual("front_track_lip")
    front_head_ceiling = frame.get_visual("front_head_ceiling")
    fixed_meeting_seal = frame.get_visual("fixed_meeting_seal")

    bottom_rail = sash.get_visual("bottom_rail")
    roller_left = sash.get_visual("roller_left")
    roller_right = sash.get_visual("roller_right")
    top_guide_left = sash.get_visual("top_guide_left")
    top_guide_right = sash.get_visual("top_guide_right")
    interlock_fin = sash.get_visual("interlock_fin")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(sash, frame, elem_a=roller_left, elem_b=front_roller_rail)
    ctx.expect_contact(sash, frame, elem_a=roller_right, elem_b=front_roller_rail)
    ctx.expect_gap(
        sash,
        frame,
        axis="z",
        min_gap=0.017,
        max_gap=0.0195,
        positive_elem=bottom_rail,
        negative_elem=front_track_base,
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        min_gap=0.0015,
        max_gap=0.0030,
        positive_elem=bottom_rail,
        negative_elem=front_track_backstop,
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="y",
        min_gap=0.0020,
        max_gap=0.0045,
        positive_elem=front_track_lip,
        negative_elem=bottom_rail,
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="z",
        min_gap=0.0030,
        max_gap=0.0055,
        positive_elem=front_head_ceiling,
        negative_elem=top_guide_left,
    )
    ctx.expect_gap(
        frame,
        sash,
        axis="z",
        min_gap=0.0030,
        max_gap=0.0055,
        positive_elem=front_head_ceiling,
        negative_elem=top_guide_right,
    )
    ctx.expect_within(
        sash,
        frame,
        axes="x",
        margin=0.0,
        inner_elem=bottom_rail,
        outer_elem=front_track_base,
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="xz",
        min_overlap=0.008,
        elem_a=interlock_fin,
        elem_b=fixed_meeting_seal,
    )
    ctx.expect_gap(
        sash,
        frame,
        axis="y",
        min_gap=0.0025,
        max_gap=0.0045,
        positive_elem=interlock_fin,
        negative_elem=fixed_meeting_seal,
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({sash_slide: SLIDE_TRAVEL}):
        open_pos = ctx.part_world_position(sash)
        ctx.fail_if_parts_overlap_in_current_pose(name="open_pose_no_part_overlaps")
        ctx.expect_contact(sash, frame, elem_a=roller_left, elem_b=front_roller_rail)
        ctx.expect_contact(sash, frame, elem_a=roller_right, elem_b=front_roller_rail)
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            min_gap=0.0015,
            max_gap=0.0030,
            positive_elem=bottom_rail,
            negative_elem=front_track_backstop,
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="y",
            min_gap=0.0020,
            max_gap=0.0045,
            positive_elem=front_track_lip,
            negative_elem=bottom_rail,
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            min_gap=0.0030,
            max_gap=0.0055,
            positive_elem=front_head_ceiling,
            negative_elem=top_guide_left,
        )
        ctx.expect_gap(
            frame,
            sash,
            axis="z",
            min_gap=0.0030,
            max_gap=0.0055,
            positive_elem=front_head_ceiling,
            negative_elem=top_guide_right,
        )
        ctx.expect_within(
            sash,
            frame,
            axes="x",
            margin=0.0,
            inner_elem=bottom_rail,
            outer_elem=front_track_base,
        )

    travel_ok = (
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] > rest_pos[0] + (SLIDE_TRAVEL - 0.005)
        and abs(open_pos[1] - rest_pos[1]) < 1e-4
        and abs(open_pos[2] - rest_pos[2]) < 1e-4
    )
    ctx.check(
        "sash_travel_is_horizontal",
        travel_ok,
        details=(
            f"expected x travel of about {SLIDE_TRAVEL:.3f} m with negligible y/z drift; "
            f"rest={rest_pos}, open={open_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
