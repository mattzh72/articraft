from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BODY_W = 0.34
BODY_D = 0.22
BODY_H = 0.18
WALL_T = 0.01
BOTTOM_T = 0.01
FRONT_SILL_H = 0.065

LID_W = 0.358
LID_D = 0.224
LID_PANEL_D = 0.209
LID_TOP_T = 0.008
LID_SKIRT_T = 0.008
LID_SKIRT_DROP = 0.048

HINGE_Y = 0.109
HINGE_Z = 0.180
HINGE_R = 0.0075

CRADLE_AXIS_Z = 0.098
CRADLE_SHAFT_LEN = 0.118
CRADLE_SHAFT_R = 0.008
CRADLE_HUB_LEN = 0.078
CRADLE_HUB_R = 0.036


def _x_cylinder(part, *, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(part, *, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_watch_winder")

    model.material("body_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("lid_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    model.material("safety_yellow", rgba=(0.86, 0.72, 0.16, 1.0))
    model.material("cradle_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("fastener_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("lockout_red", rgba=(0.66, 0.12, 0.10, 1.0))

    body = model.part("body")
    lid = model.part("lid")
    cradle = model.part("cradle")

    # Root enclosure and welded internal frame.
    body.visual(
        Box((BODY_W, BODY_D, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T / 2.0)),
        material="body_steel",
        name="base_plate",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H - BOTTOM_T)),
        origin=Origin(xyz=(BODY_W / 2.0 - WALL_T / 2.0, 0.0, 0.085)),
        material="body_steel",
        name="right_side_wall",
    )
    body.visual(
        Box((WALL_T, BODY_D, BODY_H - BOTTOM_T)),
        origin=Origin(xyz=(-(BODY_W / 2.0) + WALL_T / 2.0, 0.0, 0.085)),
        material="body_steel",
        name="left_side_wall",
    )
    body.visual(
        Box((BODY_W, WALL_T, BODY_H - BOTTOM_T)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - WALL_T / 2.0, 0.085)),
        material="body_steel",
        name="rear_wall",
    )
    body.visual(
        Box((BODY_W, WALL_T, FRONT_SILL_H)),
        origin=Origin(xyz=(0.0, -(BODY_D / 2.0) + WALL_T / 2.0, FRONT_SILL_H / 2.0)),
        material="body_steel",
        name="front_sill",
    )
    # Hinge reinforcement and rear-wall-mounted over-travel stop brackets.
    body.visual(
        Box((0.050, 0.022, 0.050)),
        origin=Origin(xyz=(-0.134, 0.099, 0.145)),
        material="body_steel",
        name="left_hinge_brace",
    )
    body.visual(
        Box((0.050, 0.022, 0.050)),
        origin=Origin(xyz=(0.134, 0.099, 0.145)),
        material="body_steel",
        name="right_hinge_brace",
    )
    body.visual(
        Box((0.070, 0.018, 0.046)),
        origin=Origin(xyz=(0.0, 0.099, 0.147)),
        material="body_steel",
        name="center_hinge_brace",
    )
    body.visual(
        Box((0.024, 0.016, 0.018)),
        origin=Origin(xyz=(-0.056, 0.106, 0.165)),
        material="safety_yellow",
        name="left_overtravel_stop",
    )
    body.visual(
        Box((0.024, 0.016, 0.018)),
        origin=Origin(xyz=(0.056, 0.106, 0.165)),
        material="safety_yellow",
        name="right_overtravel_stop",
    )
    body.visual(
        Box((0.012, 0.016, 0.030)),
        origin=Origin(xyz=(-0.056, 0.098, 0.157)),
        material="body_steel",
        name="left_stop_post",
    )
    body.visual(
        Box((0.012, 0.016, 0.030)),
        origin=Origin(xyz=(0.056, 0.098, 0.157)),
        material="body_steel",
        name="right_stop_post",
    )
    _x_cylinder(
        body,
        radius=HINGE_R,
        length=0.052,
        xyz=(-0.115, HINGE_Y, HINGE_Z),
        material="body_steel",
        name="left_hinge_barrel",
    )
    _x_cylinder(
        body,
        radius=HINGE_R,
        length=0.040,
        xyz=(0.0, HINGE_Y, HINGE_Z),
        material="body_steel",
        name="center_hinge_barrel",
    )
    _x_cylinder(
        body,
        radius=HINGE_R,
        length=0.052,
        xyz=(0.115, HINGE_Y, HINGE_Z),
        material="body_steel",
        name="right_hinge_barrel",
    )
    body.visual(
        Box((0.022, 0.016, 0.020)),
        origin=Origin(xyz=(-0.115, 0.102, 0.170)),
        material="body_steel",
        name="left_hinge_lug",
    )
    body.visual(
        Box((0.020, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.102, 0.170)),
        material="body_steel",
        name="center_hinge_lug",
    )
    body.visual(
        Box((0.022, 0.016, 0.020)),
        origin=Origin(xyz=(0.115, 0.102, 0.170)),
        material="body_steel",
        name="right_hinge_lug",
    )

    # Internal guard and cradle support frame.
    body.visual(
        Box((0.240, 0.012, 0.150)),
        origin=Origin(xyz=(0.0, 0.063, 0.085)),
        material="body_steel",
        name="rear_bulkhead",
    )
    body.visual(
        Box((0.020, 0.012, 0.140)),
        origin=Origin(xyz=(-0.095, -0.074, 0.080)),
        material="safety_yellow",
        name="front_guard_left_post",
    )
    body.visual(
        Box((0.020, 0.012, 0.140)),
        origin=Origin(xyz=(0.095, -0.074, 0.080)),
        material="safety_yellow",
        name="front_guard_right_post",
    )
    body.visual(
        Box((0.210, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.074, 0.020)),
        material="safety_yellow",
        name="front_guard_lower_beam",
    )
    body.visual(
        Box((0.210, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.074, 0.158)),
        material="safety_yellow",
        name="front_guard_upper_beam",
    )
    body.visual(
        Box((0.065, 0.012, 0.016)),
        origin=Origin(xyz=(-0.0525, -0.074, CRADLE_AXIS_Z)),
        material="safety_yellow",
        name="front_guard_left_brace",
    )
    body.visual(
        Box((0.065, 0.012, 0.016)),
        origin=Origin(xyz=(0.0525, -0.074, CRADLE_AXIS_Z)),
        material="safety_yellow",
        name="front_guard_right_brace",
    )
    _y_cylinder(
        body,
        radius=0.018,
        length=0.018,
        xyz=(0.0, -0.068, CRADLE_AXIS_Z),
        material="body_steel",
        name="front_bearing_collar",
    )
    body.visual(
        Box((0.040, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.074, CRADLE_AXIS_Z)),
        material="body_steel",
        name="front_bearing_mount_plate",
    )
    _y_cylinder(
        body,
        radius=0.018,
        length=0.018,
        xyz=(0.0, 0.068, CRADLE_AXIS_Z),
        material="body_steel",
        name="rear_bearing_collar",
    )

    # Rear drive housing and side lockout enclosure.
    body.visual(
        Box((0.120, 0.038, 0.090)),
        origin=Origin(xyz=(0.0, 0.129, 0.082)),
        material="body_steel",
        name="rear_drive_housing",
    )
    body.visual(
        Box((0.030, 0.040, 0.050)),
        origin=Origin(xyz=(0.185, 0.055, 0.085)),
        material="lockout_red",
        name="service_lockout_box",
    )
    body.visual(
        Box((0.014, 0.050, 0.058)),
        origin=Origin(xyz=(0.202, 0.055, 0.085)),
        material="body_steel",
        name="service_lockout_guard",
    )
    body.visual(
        Box((0.006, 0.020, 0.014)),
        origin=Origin(xyz=(0.210, 0.055, 0.107)),
        material="fastener_steel",
        name="service_lockout_tab",
    )

    # Visible fastener heads on the hinge braces.
    _x_cylinder(
        body,
        radius=0.0055,
        length=0.006,
        xyz=(-0.173, 0.086, 0.154),
        material="fastener_steel",
        name="left_brace_upper_bolt",
    )
    _x_cylinder(
        body,
        radius=0.0055,
        length=0.006,
        xyz=(-0.173, 0.086, 0.128),
        material="fastener_steel",
        name="left_brace_lower_bolt",
    )
    _x_cylinder(
        body,
        radius=0.0055,
        length=0.006,
        xyz=(0.173, 0.086, 0.154),
        material="fastener_steel",
        name="right_brace_upper_bolt",
    )
    _x_cylinder(
        body,
        radius=0.0055,
        length=0.006,
        xyz=(0.173, 0.086, 0.128),
        material="fastener_steel",
        name="right_brace_lower_bolt",
    )

    # Lid as a shallow service cover with side doublers and stop pads.
    lid.visual(
        Box((LID_W, LID_PANEL_D, LID_TOP_T)),
        origin=Origin(xyz=(0.0, -0.1195, LID_TOP_T / 2.0)),
        material="lid_steel",
        name="lid_top_plate",
    )
    lid.visual(
        Box((LID_SKIRT_T, 0.160, LID_SKIRT_DROP)),
        origin=Origin(xyz=(0.175, -0.104, -LID_SKIRT_DROP / 2.0)),
        material="lid_steel",
        name="right_lid_skirt",
    )
    lid.visual(
        Box((LID_SKIRT_T, 0.160, LID_SKIRT_DROP)),
        origin=Origin(xyz=(-0.175, -0.104, -LID_SKIRT_DROP / 2.0)),
        material="lid_steel",
        name="left_lid_skirt",
    )
    lid.visual(
        Box((LID_W, LID_SKIRT_T, LID_SKIRT_DROP)),
        origin=Origin(xyz=(0.0, -(LID_D - LID_SKIRT_T / 2.0), -LID_SKIRT_DROP / 2.0)),
        material="lid_steel",
        name="front_lid_skirt",
    )
    lid.visual(
        Box((0.006, 0.100, 0.030)),
        origin=Origin(xyz=(-0.182, -0.055, -0.015)),
        material="lid_steel",
        name="left_side_doubler",
    )
    lid.visual(
        Box((0.006, 0.100, 0.030)),
        origin=Origin(xyz=(0.182, -0.055, -0.015)),
        material="lid_steel",
        name="right_side_doubler",
    )
    lid.visual(
        Box((0.026, 0.014, 0.014)),
        origin=Origin(xyz=(-0.056, 0.012, -0.010)),
        material="safety_yellow",
        name="left_stop_pad",
    )
    lid.visual(
        Box((0.026, 0.014, 0.014)),
        origin=Origin(xyz=(0.056, 0.012, -0.010)),
        material="safety_yellow",
        name="right_stop_pad",
    )
    lid.visual(
        Box((0.026, 0.018, 0.012)),
        origin=Origin(xyz=(-0.056, -0.009, 0.0)),
        material="lid_steel",
        name="left_hinge_strap",
    )
    lid.visual(
        Box((0.026, 0.018, 0.012)),
        origin=Origin(xyz=(0.056, -0.009, 0.0)),
        material="lid_steel",
        name="right_hinge_strap",
    )
    _x_cylinder(
        lid,
        radius=HINGE_R,
        length=0.064,
        xyz=(-0.056, 0.0, 0.0),
        material="lid_steel",
        name="left_hinge_sleeve",
    )
    _x_cylinder(
        lid,
        radius=HINGE_R,
        length=0.064,
        xyz=(0.056, 0.0, 0.0),
        material="lid_steel",
        name="right_hinge_sleeve",
    )
    _x_cylinder(
        lid,
        radius=0.0045,
        length=0.005,
        xyz=(-0.1855, -0.060, -0.010),
        material="fastener_steel",
        name="left_doubler_upper_bolt",
    )
    _x_cylinder(
        lid,
        radius=0.0045,
        length=0.005,
        xyz=(-0.1855, -0.092, -0.010),
        material="fastener_steel",
        name="left_doubler_lower_bolt",
    )
    _x_cylinder(
        lid,
        radius=0.0045,
        length=0.005,
        xyz=(0.1855, -0.060, -0.010),
        material="fastener_steel",
        name="right_doubler_upper_bolt",
    )
    _x_cylinder(
        lid,
        radius=0.0045,
        length=0.005,
        xyz=(0.1855, -0.092, -0.010),
        material="fastener_steel",
        name="right_doubler_lower_bolt",
    )

    # Rotating cradle with explicit shaft, cheeks, and restraint rails.
    _y_cylinder(
        cradle,
        radius=CRADLE_SHAFT_R,
        length=CRADLE_SHAFT_LEN,
        xyz=(0.0, 0.0, 0.0),
        material="fastener_steel",
        name="drive_shaft",
    )
    _y_cylinder(
        cradle,
        radius=CRADLE_HUB_R,
        length=CRADLE_HUB_LEN,
        xyz=(0.0, 0.0, 0.0),
        material="cradle_dark",
        name="watch_cushion_core",
    )
    cradle.visual(
        Box((0.110, 0.008, 0.090)),
        origin=Origin(xyz=(0.0, -0.043, 0.0)),
        material="cradle_dark",
        name="front_cheek_plate",
    )
    cradle.visual(
        Box((0.110, 0.008, 0.090)),
        origin=Origin(xyz=(0.0, 0.043, 0.0)),
        material="cradle_dark",
        name="rear_cheek_plate",
    )
    cradle.visual(
        Box((0.110, 0.086, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material="cradle_dark",
        name="upper_retention_rail",
    )
    cradle.visual(
        Box((0.110, 0.086, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material="cradle_dark",
        name="lower_retention_rail",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=0.0,
            upper=1.12,
        ),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, CRADLE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")

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
        "service_lockout_present",
        any(v.name == "service_lockout_tab" for v in body.visuals),
        "Expected a visible service lockout tab on the body-side disconnect enclosure.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            min_gap=0.0,
            max_gap=0.0011,
            positive_elem="left_hinge_sleeve",
            negative_elem="left_hinge_barrel",
            name="left_hinge_axial_clearance_is_tight_when_closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            min_overlap=0.014,
            elem_a="left_hinge_sleeve",
            elem_b="left_hinge_barrel",
            name="left_hinge_support_point_is_coaxially_supported",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="x",
            min_gap=0.0,
            max_gap=0.0011,
            positive_elem="right_hinge_barrel",
            negative_elem="right_hinge_sleeve",
            name="right_hinge_axial_clearance_is_tight_when_closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            min_overlap=0.014,
            elem_a="right_hinge_sleeve",
            elem_b="right_hinge_barrel",
            name="right_hinge_support_point_is_coaxially_supported",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            name="closed_lid_covers_body_plan_footprint",
        )
        ctx.expect_contact(
            cradle,
            body,
            elem_a="drive_shaft",
            elem_b="front_bearing_collar",
            name="front_cradle_bearing_support_contact",
        )
        ctx.expect_contact(
            cradle,
            body,
            elem_a="drive_shaft",
            elem_b="rear_bearing_collar",
            name="rear_cradle_bearing_support_contact",
        )
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            margin=0.0,
            name="cradle_stays_inside_body_opening_in_rest_pose",
        )

    with ctx.pose({cradle_spin: pi / 2.0}):
        ctx.expect_contact(
            cradle,
            body,
            elem_a="drive_shaft",
            elem_b="front_bearing_collar",
            name="front_bearing_stays_supported_at_quarter_turn",
        )
        ctx.expect_contact(
            cradle,
            body,
            elem_a="drive_shaft",
            elem_b="rear_bearing_collar",
            name="rear_bearing_stays_supported_at_quarter_turn",
        )
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            margin=0.0,
            name="cradle_clears_frame_at_quarter_turn",
        )

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        body_aabb = ctx.part_world_aabb(body)
        lid_aabb = ctx.part_world_aabb(lid)
        lid_lifts_clear = (
            body_aabb is not None
            and lid_aabb is not None
            and lid_aabb[1][2] > body_aabb[1][2] + 0.08
        )
        ctx.check(
            "lid_opens_upward",
            lid_lifts_clear,
            "Upper-limit lid pose should raise the cover well above the body shell.",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="left_stop_pad",
            elem_b="left_overtravel_stop",
            name="left_overtravel_stop_engages",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="right_stop_pad",
            elem_b="right_overtravel_stop",
            name="right_overtravel_stop_engages",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
