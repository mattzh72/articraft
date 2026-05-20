from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.24
BASE_D = 0.17
BASE_T = 0.055

COLUMN_W = 0.118
COLUMN_D = 0.068
COLUMN_H = 0.62
HEAD_W = 0.148
HEAD_D = 0.094
HEAD_H = 0.070

GUIDE_BAR_W = 0.012
GUIDE_BAR_D = 0.012
GUIDE_BAR_H = 0.58
GUIDE_BAR_X = 0.034
GUIDE_BAR_Y = 0.010
GUIDE_BAR_Z = 0.44

SLIDE_TRAVEL = 0.20
SLIDE_JOINT_Z = 0.18

WRIST_AXIS_Y = 0.122
WRIST_AXIS_Z = 0.082
WRIST_LOWER = -0.35
WRIST_UPPER = 1.05


def _make_column_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_T).translate((0.0, 0.0, BASE_T / 2.0))
    rear_spine = cq.Workplane("XY").box(0.092, 0.026, COLUMN_H).translate((0.0, -0.034, BASE_T + (COLUMN_H / 2.0)))
    left_carrier = cq.Workplane("XY").box(0.020, 0.016, 0.50).translate((-0.048, -0.002, 0.145 + 0.25))
    right_carrier = cq.Workplane("XY").box(0.020, 0.016, 0.50).translate((0.048, -0.002, 0.145 + 0.25))
    lower_bridge = cq.Workplane("XY").box(0.096, 0.014, 0.060).translate((0.0, -0.010, 0.155))
    head = cq.Workplane("XY").box(HEAD_W, 0.040, HEAD_H).translate((0.0, -0.010, BASE_T + COLUMN_H - 0.010 + (HEAD_H / 2.0)))
    motor_cap = cq.Workplane("XY").box(0.100, 0.070, 0.038).translate((0.0, -0.024, BASE_T + COLUMN_H + 0.034))
    top_bevel = (
        cq.Workplane("XZ")
        .center(0.0, BASE_T + COLUMN_H - 0.024)
        .polyline([(-0.048, 0.0), (0.048, 0.0), (0.030, 0.038), (-0.030, 0.038)])
        .close()
        .extrude(0.08, both=True)
    )
    column = (
        base.union(rear_spine)
        .union(left_carrier)
        .union(right_carrier)
        .union(lower_bridge)
        .union(head)
        .union(motor_cap)
        .cut(top_bevel)
    )
    return column


def _make_slide_shape() -> cq.Workplane:
    upper_body = cq.Workplane("XY").box(0.104, 0.052, 0.30).translate((0.0, 0.058, 0.22))
    left_runner = cq.Workplane("XY").box(0.024, 0.022, 0.26).translate((-GUIDE_BAR_X, 0.027, 0.22))
    right_runner = cq.Workplane("XY").box(0.024, 0.022, 0.26).translate((GUIDE_BAR_X, 0.027, 0.22))
    lower_saddle = cq.Workplane("XY").box(0.086, 0.022, 0.042).translate((0.0, 0.045, 0.060))
    left_arm = cq.Workplane("XY").box(0.014, 0.024, 0.108).translate((-0.046, 0.110, 0.048))
    right_arm = cq.Workplane("XY").box(0.014, 0.024, 0.108).translate((0.046, 0.110, 0.048))
    left_gusset = cq.Workplane("XY").box(0.020, 0.020, 0.060).translate((-0.034, 0.090, 0.112))
    right_gusset = cq.Workplane("XY").box(0.020, 0.020, 0.060).translate((0.034, 0.090, 0.112))
    top_bridge = cq.Workplane("XY").box(0.074, 0.012, 0.016).translate((0.0, 0.112, 0.103))
    rear_bridge = cq.Workplane("XY").box(0.074, 0.010, 0.016).translate((0.0, 0.096, 0.022))
    stop_bridge = cq.Workplane("XY").box(0.060, 0.010, 0.012).translate((0.0, 0.106, 0.068))
    left_stop = cq.Workplane("XY").box(0.018, 0.010, 0.018).translate((-0.022, 0.106, 0.068))
    right_stop = cq.Workplane("XY").box(0.018, 0.010, 0.018).translate((0.022, 0.106, 0.068))
    left_axis_cap = (
        cq.Workplane("YZ")
        .circle(0.013)
        .extrude(0.006, both=True)
        .translate((-0.055, WRIST_AXIS_Y, WRIST_AXIS_Z))
    )
    right_axis_cap = (
        cq.Workplane("YZ")
        .circle(0.013)
        .extrude(0.006, both=True)
        .translate((0.055, WRIST_AXIS_Y, WRIST_AXIS_Z))
    )
    left_foot = cq.Workplane("XY").box(0.018, 0.018, 0.022).translate((-0.034, 0.046, 0.21))
    right_foot = cq.Workplane("XY").box(0.018, 0.018, 0.022).translate((0.034, 0.046, 0.21))

    slide = (
        upper_body.union(left_runner)
        .union(right_runner)
        .union(lower_saddle)
        .union(left_arm)
        .union(right_arm)
        .union(left_gusset)
        .union(right_gusset)
        .union(top_bridge)
        .union(rear_bridge)
        .union(stop_bridge)
        .union(left_stop)
        .union(right_stop)
        .union(left_axis_cap)
        .union(right_axis_cap)
        .union(left_foot)
        .union(right_foot)
    )

    front_service_pocket = cq.Workplane("XY").box(0.056, 0.010, 0.16).translate((0.0, 0.074, 0.22))
    fork_window = cq.Workplane("XY").box(0.070, 0.026, 0.070).translate((0.0, 0.110, 0.048))
    wrist_clearance = (
        cq.Workplane("YZ")
        .circle(0.0105)
        .extrude(0.049, both=True)
        .translate((0.0, WRIST_AXIS_Y, WRIST_AXIS_Z))
    )
    slide = slide.cut(front_service_pocket).cut(fork_window).cut(wrist_clearance)
    return slide


def _make_end_plate_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.070, 0.010, 0.106).translate((0.0, 0.055, -0.050))
    hinge_barrel = cq.Workplane("YZ").circle(0.0085).extrude(0.020, both=True)
    strut = cq.Workplane("XY").box(0.018, 0.010, 0.090).translate((0.0, 0.040, -0.044))
    front_boss = cq.Workplane("XY").box(0.046, 0.008, 0.040).translate((0.0, 0.064, -0.050))
    front_pocket = cq.Workplane("XY").box(0.046, 0.004, 0.074).translate((0.0, 0.055, -0.054))
    plate = panel.cut(front_pocket).union(hinge_barrel).union(strut).union(front_boss)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_tool_axis")

    model.material("painted_frame", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("ground_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("anodized_slide", rgba=(0.61, 0.65, 0.69, 1.0))
    model.material("tool_plate", rgba=(0.83, 0.85, 0.88, 1.0))

    column = model.part("column")
    column.visual(mesh_from_cadquery(_make_column_shape(), "column_body"), material="painted_frame", name="column_body")
    column.visual(
        Box((GUIDE_BAR_W, GUIDE_BAR_D, GUIDE_BAR_H)),
        origin=Origin(xyz=(-GUIDE_BAR_X, GUIDE_BAR_Y, GUIDE_BAR_Z)),
        material="ground_steel",
        name="left_guide",
    )
    column.visual(
        Box((GUIDE_BAR_W, GUIDE_BAR_D, GUIDE_BAR_H)),
        origin=Origin(xyz=(GUIDE_BAR_X, GUIDE_BAR_Y, GUIDE_BAR_Z)),
        material="ground_steel",
        name="right_guide",
    )
    column.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_T + COLUMN_H + HEAD_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )

    slide = model.part("slide")
    slide.visual(mesh_from_cadquery(_make_slide_shape(), "z_slide"), material="anodized_slide", name="slide_body")
    slide.inertial = Inertial.from_geometry(
        Box((0.112, 0.126, 0.34)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.063, 0.17)),
    )

    end_plate = model.part("end_plate")
    end_plate.visual(mesh_from_cadquery(_make_end_plate_shape(), "end_plate"), material="tool_plate", name="end_plate_body")
    end_plate.inertial = Inertial.from_geometry(
        Box((0.10, 0.028, 0.12)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
    )

    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=column,
        child=slide,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=900.0, velocity=0.30),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=end_plate,
        origin=Origin(xyz=(0.0, WRIST_AXIS_Y, WRIST_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=WRIST_LOWER, upper=WRIST_UPPER, effort=60.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    slide = object_model.get_part("slide")
    end_plate = object_model.get_part("end_plate")
    z_slide = object_model.get_articulation("z_slide")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_isolated_part(
        end_plate,
        reason="Visible hinge caps imply a retained wrist pin and bearing clearance; the end plate is intentionally shown with a small non-contact running clearance around the supported wrist axis.",
    )

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
        "z_slide_axis_vertical",
        z_slide.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical prismatic axis, got {z_slide.axis}",
    )
    ctx.check(
        "wrist_axis_horizontal",
        wrist_pitch.axis == (1.0, 0.0, 0.0),
        details=f"Expected supported wrist axis along X, got {wrist_pitch.axis}",
    )
    ctx.check(
        "z_slide_limits",
        (
            z_slide.motion_limits is not None
            and isclose(z_slide.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
            and isclose(z_slide.motion_limits.upper or 0.0, SLIDE_TRAVEL, abs_tol=1e-9)
        ),
        details="Unexpected lift travel limits",
    )
    ctx.check(
        "wrist_limits",
        (
            wrist_pitch.motion_limits is not None
            and isclose(wrist_pitch.motion_limits.lower or 0.0, WRIST_LOWER, abs_tol=1e-9)
            and isclose(wrist_pitch.motion_limits.upper or 0.0, WRIST_UPPER, abs_tol=1e-9)
        ),
        details="Unexpected wrist sweep limits",
    )

    ctx.expect_overlap(slide, column, axes="xz", elem_b="left_guide", min_overlap=0.010, name="left_guide_captured_at_rest")
    ctx.expect_overlap(slide, column, axes="xz", elem_b="right_guide", min_overlap=0.010, name="right_guide_captured_at_rest")
    ctx.expect_gap(slide, column, axis="y", min_gap=0.0, max_gap=0.02, positive_elem="slide_body", negative_elem="left_guide", name="slide_runs_close_to_guides")
    ctx.expect_within(end_plate, slide, axes="x", margin=0.004, name="end_plate_stays_between_hinge_cheeks")

    with ctx.pose(z_slide=SLIDE_TRAVEL):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_top_of_lift")
        ctx.expect_overlap(slide, column, axes="xz", elem_b="left_guide", min_overlap=0.010, name="left_guide_captured_at_top")
        ctx.expect_overlap(slide, column, axes="xz", elem_b="right_guide", min_overlap=0.010, name="right_guide_captured_at_top")
        ctx.expect_overlap(slide, column, axes="z", elem_b="left_guide", min_overlap=0.20, name="left_guide_still_engaged_at_top")
        ctx.expect_overlap(slide, column, axes="z", elem_b="right_guide", min_overlap=0.20, name="right_guide_still_engaged_at_top")

    with ctx.pose(wrist_pitch=WRIST_UPPER):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_forward_wrist")

    with ctx.pose(z_slide=SLIDE_TRAVEL, wrist_pitch=WRIST_LOWER):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_back_tilt_top_of_lift")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
