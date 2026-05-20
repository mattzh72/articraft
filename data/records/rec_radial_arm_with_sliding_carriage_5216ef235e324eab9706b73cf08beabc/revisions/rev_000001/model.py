from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.56
BASE_Y = 0.46
BASE_T = 0.08
PLINTH_D = 0.24
PLINTH_H = 0.04

COLUMN_D = 0.26
COLUMN_R = COLUMN_D / 2.0
COLUMN_H = 1.42

BEAM_SEAT_Z = 0.96
SUPPORT_COLLAR_H = 0.06
SUPPORT_COLLAR_OD = 0.36

LOWER_BAND_Z = 0.84
LOWER_BAND_H = 0.04
LOWER_BAND_OD = 0.28

TOP_BAND_Z = 1.32
TOP_BAND_H = 0.04
TOP_BAND_OD = 0.274

TOP_CAP_D = 0.19
TOP_CAP_H = 0.045

SLEEVE_ID = 0.278
SLEEVE_OD = 0.42
FLANGE_OD = 0.44
ROOT_H = 0.28
FLANGE_H = 0.04
CLAMP_BAND_OD = 0.44
CLAMP_BAND_Z = 0.225
CLAMP_BAND_H = 0.055

BEAM_X0 = 0.24
BEAM_L = 1.06
BEAM_D = 0.18
BEAM_H = 0.18
BEAM_Z0 = 0.09

BEAM_INNER_X0 = 0.36
BEAM_INNER_L = 0.82
BEAM_INNER_D = 0.108
BEAM_INNER_H = 0.108
BEAM_INNER_Z0 = 0.126

RIB_T = 0.016
ROOT_BRIDGE_X0 = 0.18
ROOT_BRIDGE_L = 0.18
ROOT_BRIDGE_D = 0.18
ROOT_BRIDGE_H = 0.18
GUIDE_PAD_X0 = 0.38
GUIDE_PAD_L = 0.64
GUIDE_PAD_D = 0.10
GUIDE_PAD_H = 0.012
GUIDE_TOP_Z = BEAM_Z0 + BEAM_H + (GUIDE_PAD_H / 2.0)
GUIDE_BOTTOM_Z = BEAM_Z0 - (GUIDE_PAD_H / 2.0)

STOP_L = 0.03
STOP_D = 0.14
STOP_H = 0.04
INNER_STOP_X = 0.34
OUTER_STOP_X = 1.035
STOP_Z = BEAM_Z0 + BEAM_H + (STOP_H / 2.0)

SADDLE_X0 = 0.60
SADDLE_Z = BEAM_Z0 + BEAM_H + GUIDE_PAD_H
SADDLE_L = 0.22
SADDLE_D = 0.24
SADDLE_H = 0.17
SADDLE_TOP_BLOCK_H = 0.13
SADDLE_SHOE_L = 0.18
SADDLE_SHOE_D = 0.10
SADDLE_SHOE_H = 0.012
SADDLE_CHEEK_T = 0.03
SADDLE_CHEEK_Y = 0.105
SADDLE_CHEEK_H = 0.11
SADDLE_CHEEK_Z = 0.01
SADDLE_HEAD_H = 0.032
SADDLE_HEAD_D = 0.18
SADDLE_HEAD_Z = SADDLE_TOP_BLOCK_H
SADDLE_FRONT_RELIEF_L = 0.09
SADDLE_FRONT_RELIEF_D = 0.10
SADDLE_FRONT_RELIEF_H = 0.05
SADDLE_FRONT_RELIEF_Z = 0.03

SWING_LIMIT = 1.8
SLIDE_LOWER = -0.07
SLIDE_UPPER = 0.24


def _ring(outer_d: float, inner_d: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").cylinder(height, outer_d / 2.0, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .cylinder(height + 0.004, inner_d / 2.0, centered=(True, True, False))
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(inner)


def _column_body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_T, centered=(True, True, False))
    plinth = cq.Workplane("XY").circle(PLINTH_D / 2.0).extrude(PLINTH_H).translate((0.0, 0.0, BASE_T))
    column = cq.Workplane("XY").circle(COLUMN_R).extrude(COLUMN_H).translate((0.0, 0.0, BASE_T))
    top_cap = (
        cq.Workplane("XY")
        .circle(TOP_CAP_D / 2.0)
        .extrude(TOP_CAP_H)
        .translate((0.0, 0.0, BASE_T + COLUMN_H))
    )
    return base.union(plinth).union(column).union(top_cap)


def _support_collar_shape() -> cq.Workplane:
    return _ring(SUPPORT_COLLAR_OD, COLUMN_D, SUPPORT_COLLAR_H).translate(
        (0.0, 0.0, BEAM_SEAT_Z - SUPPORT_COLLAR_H)
    )


def _lower_band_shape() -> cq.Workplane:
    return _ring(LOWER_BAND_OD, COLUMN_D, LOWER_BAND_H).translate((0.0, 0.0, LOWER_BAND_Z))


def _top_band_shape() -> cq.Workplane:
    return _ring(TOP_BAND_OD, COLUMN_D, TOP_BAND_H).translate((0.0, 0.0, TOP_BAND_Z))


def _beam_body_shape() -> cq.Workplane:
    sleeve = _ring(SLEEVE_OD, SLEEVE_ID, ROOT_H)
    clamp_band = _ring(CLAMP_BAND_OD, SLEEVE_ID, CLAMP_BAND_H).translate((0.0, 0.0, CLAMP_BAND_Z))
    root_bridge = (
        cq.Workplane("XY")
        .box(ROOT_BRIDGE_L, ROOT_BRIDGE_D, ROOT_BRIDGE_H, centered=(False, True, False))
        .translate((ROOT_BRIDGE_X0, 0.0, 0.0))
    )

    beam_outer = (
        cq.Workplane("XY")
        .box(BEAM_L, BEAM_D, BEAM_H, centered=(False, True, False))
        .translate((BEAM_X0, 0.0, BEAM_Z0))
    )
    beam_inner = (
        cq.Workplane("XY")
        .box(BEAM_INNER_L, BEAM_INNER_D, BEAM_INNER_H, centered=(False, True, False))
        .translate((BEAM_INNER_X0, 0.0, BEAM_INNER_Z0))
    )
    beam_shell = beam_outer.cut(beam_inner)
    top_pad = (
        cq.Workplane("XY")
        .box(GUIDE_PAD_L, GUIDE_PAD_D, GUIDE_PAD_H, centered=(False, True, True))
        .translate((GUIDE_PAD_X0, 0.0, GUIDE_TOP_Z))
    )
    bottom_pad = (
        cq.Workplane("XY")
        .box(GUIDE_PAD_L, GUIDE_PAD_D, GUIDE_PAD_H, centered=(False, True, True))
        .translate((GUIDE_PAD_X0, 0.0, GUIDE_BOTTOM_Z))
    )
    inner_stop = (
        cq.Workplane("XY")
        .box(STOP_L, STOP_D, STOP_H, centered=(False, True, False))
        .translate((INNER_STOP_X, 0.0, BEAM_Z0 + BEAM_H))
    )
    outer_stop = (
        cq.Workplane("XY")
        .box(STOP_L, STOP_D, STOP_H, centered=(False, True, False))
        .translate((OUTER_STOP_X, 0.0, BEAM_Z0 + BEAM_H))
    )

    rib_profile = [
        (0.22, 0.05),
        (0.36, 0.05),
        (0.46, 0.13),
        (0.48, 0.28),
        (0.28, 0.28),
        (0.22, 0.16),
    ]
    side_rib = cq.Workplane("XZ").polyline(rib_profile).close().extrude(RIB_T)
    side_rib_pos = side_rib.translate((0.0, (BEAM_D / 2.0), 0.0))
    side_rib_neg = side_rib.translate((0.0, -(BEAM_D / 2.0) - RIB_T, 0.0))

    lower_web = (
        cq.Workplane("XZ")
        .polyline([(0.20, 0.05), (0.38, 0.05), (0.30, 0.12), (0.22, 0.12)])
        .close()
        .extrude(0.10)
        .translate((0.0, -0.05, 0.0))
    )

    return (
        sleeve
        .union(clamp_band)
        .union(root_bridge)
        .union(beam_shell)
        .union(top_pad)
        .union(bottom_pad)
        .union(inner_stop)
        .union(outer_stop)
        .union(side_rib_pos)
        .union(side_rib_neg)
        .union(lower_web)
    )


def _saddle_shape() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(
        SADDLE_SHOE_L,
        SADDLE_SHOE_D,
        SADDLE_SHOE_H,
        centered=(True, True, False),
    )
    top_block = (
        cq.Workplane("XY")
        .box(SADDLE_L, SADDLE_D, SADDLE_TOP_BLOCK_H, centered=(True, True, False))
        .translate((0.0, 0.0, SADDLE_SHOE_H))
    )
    head = (
        cq.Workplane("XY")
        .box(0.12, SADDLE_HEAD_D, SADDLE_HEAD_H, centered=(True, True, False))
        .translate((0.0, 0.0, SADDLE_HEAD_Z))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(SADDLE_L * 0.92, SADDLE_CHEEK_T, SADDLE_CHEEK_H, centered=(True, True, False))
        .translate((0.0, SADDLE_CHEEK_Y, SADDLE_CHEEK_Z))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(SADDLE_L * 0.92, SADDLE_CHEEK_T, SADDLE_CHEEK_H, centered=(True, True, False))
        .translate((0.0, -SADDLE_CHEEK_Y, SADDLE_CHEEK_Z))
    )
    front_relief = (
        cq.Workplane("XY")
        .box(
            SADDLE_FRONT_RELIEF_L,
            SADDLE_FRONT_RELIEF_D,
            SADDLE_FRONT_RELIEF_H,
            centered=(True, True, False),
        )
        .translate((0.045, 0.0, SADDLE_FRONT_RELIEF_Z))
    )
    rear_relief = (
        cq.Workplane("XY")
        .box(
            SADDLE_FRONT_RELIEF_L,
            SADDLE_FRONT_RELIEF_D,
            SADDLE_FRONT_RELIEF_H,
            centered=(True, True, False),
        )
        .translate((-0.045, 0.0, SADDLE_FRONT_RELIEF_Z))
    )

    return shoe.union(top_block).union(head).union(left_cheek).union(right_cheek).cut(front_relief).cut(rear_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_drill_arm_module")

    model.material("machine_green", rgba=(0.38, 0.46, 0.40, 1.0))
    model.material("beam_blue", rgba=(0.34, 0.42, 0.54, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("carriage_gray", rgba=(0.27, 0.30, 0.33, 1.0))
    model.material("dark_steel", rgba=(0.20, 0.22, 0.25, 1.0))

    column_base = model.part("column_base")
    column_base.visual(
        mesh_from_cadquery(_column_body_shape(), "column_body"),
        material="machine_green",
        name="column_body",
    )
    column_base.visual(
        mesh_from_cadquery(_support_collar_shape(), "support_collar"),
        material="machined_steel",
        name="support_collar",
    )
    column_base.visual(
        mesh_from_cadquery(_lower_band_shape(), "lower_band"),
        material="machined_steel",
        name="lower_band",
    )
    column_base.visual(
        mesh_from_cadquery(_top_band_shape(), "top_band"),
        material="machined_steel",
        name="top_band",
    )

    beam_arm = model.part("beam_arm")
    beam_arm.visual(
        mesh_from_cadquery(_beam_body_shape(), "beam_body"),
        material="beam_blue",
        name="beam_body",
    )
    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_cadquery(_saddle_shape(), "saddle_body"),
        material="carriage_gray",
        name="saddle_body",
    )

    model.articulation(
        "column_to_beam",
        ArticulationType.REVOLUTE,
        parent=column_base,
        child=beam_arm,
        origin=Origin(xyz=(0.0, 0.0, BEAM_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.8,
            lower=-SWING_LIMIT,
            upper=SWING_LIMIT,
        ),
    )
    model.articulation(
        "beam_to_saddle",
        ArticulationType.PRISMATIC,
        parent=beam_arm,
        child=saddle,
        origin=Origin(xyz=(SADDLE_X0, 0.0, SADDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.35,
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column_base = object_model.get_part("column_base")
    beam_arm = object_model.get_part("beam_arm")
    saddle = object_model.get_part("saddle")
    swing = object_model.get_articulation("column_to_beam")
    slide = object_model.get_articulation("beam_to_saddle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0008)
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
        "column_to_beam_axis_is_vertical",
        tuple(round(value, 6) for value in swing.axis) == (0.0, 0.0, 1.0),
        f"expected vertical revolute axis, got {swing.axis}",
    )
    ctx.check(
        "beam_to_saddle_axis_is_along_beam",
        tuple(round(value, 6) for value in slide.axis) == (1.0, 0.0, 0.0),
        f"expected beam-axis prismatic motion, got {slide.axis}",
    )
    ctx.check(
        "slide_limit_order",
        slide.motion_limits is not None
        and slide.motion_limits.lower is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.lower < 0.0 < slide.motion_limits.upper,
        "expected the saddle rest pose to sit between inner and outer travel limits",
    )

    ctx.expect_contact(
        beam_arm,
        column_base,
        contact_tol=0.0015,
        name="beam_root_is_supported_on_column_collar",
    )
    ctx.expect_contact(
        saddle,
        beam_arm,
        contact_tol=0.0015,
        name="saddle_stays_guided_on_beam_rails",
    )

    with ctx.pose({slide: SLIDE_LOWER}):
        ctx.expect_gap(
            saddle,
            column_base,
            axis="x",
            positive_elem="saddle_body",
            min_gap=0.03,
            name="saddle_clears_column_module_at_inboard_limit",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_inboard_slide_limit")

    with ctx.pose({slide: SLIDE_UPPER}):
        ctx.expect_origin_gap(
            saddle,
            column_base,
            axis="x",
            min_gap=0.55,
            name="saddle_reaches_clear_outboard_position",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_outboard_slide_limit")

    with ctx.pose({swing: 0.8 * SWING_LIMIT, slide: SLIDE_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_rotated_outboard_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
