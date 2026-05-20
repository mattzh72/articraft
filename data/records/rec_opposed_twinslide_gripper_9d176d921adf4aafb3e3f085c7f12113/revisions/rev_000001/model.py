from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.094
BODY_DEPTH = 0.046
BODY_HEIGHT = 0.052
BODY_CENTER_Y = -0.011

REAR_FLANGE_WIDTH = 0.060
REAR_FLANGE_DEPTH = 0.012
REAR_FLANGE_HEIGHT = 0.068
REAR_FLANGE_Y = -0.040

CARRIER_WIDTH = 0.092
CARRIER_DEPTH = 0.020
CARRIER_HEIGHT = 0.016
CARRIER_Y = 0.020
CARRIER_Z = -0.018

WINDOW_WIDTH = 0.030
WINDOW_DEPTH = 0.024
WINDOW_HEIGHT = 0.018
WINDOW_Y = 0.021
WINDOW_Z = -0.014

RAIL_LENGTH = 0.068
RAIL_DEPTH = 0.020
RAIL_HEIGHT = 0.014
RAIL_CENTER_X = 0.053
RAIL_Y = 0.030
RAIL_Z = -0.013

CARRIAGE_LENGTH = 0.044
CARRIAGE_DEPTH = 0.034
CARRIAGE_HEIGHT = 0.032

ARM_LENGTH = 0.028
ARM_DEPTH = 0.022
ARM_HEIGHT = 0.018
ARM_Y = 0.022
ARM_Z = -0.007

FINGER_THICKNESS = 0.012
FINGER_DEPTH = 0.026
FINGER_HEIGHT = 0.046
FINGER_Y = 0.045
FINGER_Z = 0.007

TOP_CAP_LENGTH = 0.022
TOP_CAP_DEPTH = 0.016
TOP_CAP_HEIGHT = 0.008
TOP_CAP_Y = -0.006
TOP_CAP_Z = 0.018

JAW_OPEN_X = 0.073
JAW_TRAVEL = 0.021
JAW_ORIGIN_Y = RAIL_Y
JAW_ORIGIN_Z = RAIL_Z + 0.5 * RAIL_HEIGHT + 0.5 * CARRIAGE_HEIGHT


def _body_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT).translate(
        (0.0, BODY_CENTER_Y, 0.0)
    )
    rear_flange = cq.Workplane("XY").box(
        REAR_FLANGE_WIDTH, REAR_FLANGE_DEPTH, REAR_FLANGE_HEIGHT
    ).translate((0.0, REAR_FLANGE_Y, 0.0))
    carrier = cq.Workplane("XY").box(
        CARRIER_WIDTH, CARRIER_DEPTH, CARRIER_HEIGHT
    ).translate((0.0, CARRIER_Y, CARRIER_Z))
    left_rail = cq.Workplane("XY").box(RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT).translate(
        (-RAIL_CENTER_X, RAIL_Y, RAIL_Z)
    )
    right_rail = cq.Workplane("XY").box(RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT).translate(
        (RAIL_CENTER_X, RAIL_Y, RAIL_Z)
    )

    body = housing.union(rear_flange).union(carrier).union(left_rail).union(right_rail)

    center_window = cq.Workplane("XY").box(
        WINDOW_WIDTH, WINDOW_DEPTH, WINDOW_HEIGHT
    ).translate((0.0, WINDOW_Y, WINDOW_Z))
    top_relief = cq.Workplane("XY").box(0.050, 0.014, 0.018).translate((0.0, 0.005, 0.016))

    body = body.cut(center_window).cut(top_relief)
    return body


def _jaw_shape(side: str) -> cq.Workplane:
    direction = 1.0 if side == "left" else -1.0
    profile_points = [
        (-0.022 * direction, -0.016),
        (-0.022 * direction, 0.016),
        (-0.010 * direction, 0.016),
        (-0.010 * direction, 0.022),
        (0.010 * direction, 0.022),
        (0.010 * direction, 0.016),
        (0.014 * direction, 0.016),
        (0.014 * direction, 0.004),
        (0.033 * direction, 0.004),
        (0.033 * direction, 0.030),
        (0.045 * direction, 0.030),
        (0.045 * direction, -0.016),
    ]

    core = (
        cq.Workplane("XZ")
        .polyline(profile_points)
        .close()
        .extrude(0.013, both=True)
    )
    carriage = cq.Workplane("XY").box(CARRIAGE_LENGTH, CARRIAGE_DEPTH, CARRIAGE_HEIGHT)
    keeper = cq.Workplane("XY").box(0.018, 0.018, 0.006).translate((0.0, 0.0, 0.019))

    return core.union(carriage).union(keeper)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="opposed_twin_slide_gripper")

    body_mat = model.material("body_anodized", rgba=(0.36, 0.38, 0.41, 1.0))
    jaw_mat = model.material("jaw_steel", rgba=(0.16, 0.17, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "center_body"),
        origin=Origin(),
        material=body_mat,
        name="body_shell",
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_cadquery(_jaw_shape("left"), "left_jaw"),
        origin=Origin(),
        material=jaw_mat,
        name="jaw_shell",
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(_jaw_shape("right"), "right_jaw"),
        origin=Origin(),
        material=jaw_mat,
        name="jaw_shell",
    )

    model.articulation(
        "body_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(-JAW_OPEN_X, JAW_ORIGIN_Y, JAW_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_jaw,
        origin=Origin(xyz=(JAW_OPEN_X, JAW_ORIGIN_Y, JAW_ORIGIN_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("body_to_left_jaw")
    right_slide = object_model.get_articulation("body_to_right_jaw")

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
        "mirrored_prismatic_axes",
        left_slide.axis == (1.0, 0.0, 0.0) and right_slide.axis == (-1.0, 0.0, 0.0),
        details=f"left axis={left_slide.axis}, right axis={right_slide.axis}",
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_contact(
            left_jaw,
            body,
            contact_tol=0.0005,
            name="left_jaw_supported_on_guideway_open",
        )
        ctx.expect_contact(
            right_jaw,
            body,
            contact_tol=0.0005,
            name="right_jaw_supported_on_guideway_open",
        )
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.050,
            max_gap=0.058,
            name="pickup_zone_open_gap",
        )
        ctx.expect_overlap(
            left_jaw,
            right_jaw,
            axes="yz",
            min_overlap=0.024,
            name="open_jaws_share_pickup_zone",
        )

    with ctx.pose({left_slide: JAW_TRAVEL, right_slide: JAW_TRAVEL}):
        ctx.expect_contact(
            left_jaw,
            body,
            contact_tol=0.0005,
            name="left_jaw_supported_on_guideway_closed",
        )
        ctx.expect_contact(
            right_jaw,
            body,
            contact_tol=0.0005,
            name="right_jaw_supported_on_guideway_closed",
        )
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.009,
            max_gap=0.015,
            name="pickup_zone_closed_gap",
        )
        ctx.expect_overlap(
            left_jaw,
            right_jaw,
            axes="yz",
            min_overlap=0.024,
            name="closed_jaws_remain_opposed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
