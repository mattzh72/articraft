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


BODY_WIDTH = 0.070
BODY_DEPTH = 0.048
BODY_HEIGHT = 0.090
BODY_CENTER_Y = -0.014

MOUNT_FLANGE_WIDTH = 0.102
MOUNT_FLANGE_DEPTH = 0.012
MOUNT_FLANGE_HEIGHT = 0.068
MOUNT_FLANGE_CENTER_Y = -0.037
MOUNT_FLANGE_BASE_Z = 0.010

ROD_LENGTH = 0.190
ROD_RADIUS = 0.0065
ROD_CENTER_Y = 0.018
LOWER_ROD_Z = 0.028
UPPER_ROD_Z = 0.060
ROD_MID_Z = (LOWER_ROD_Z + UPPER_ROD_Z) / 2.0
ROD_SPACING = UPPER_ROD_Z - LOWER_ROD_Z

JAW_OPEN_X = 0.070
JAW_TRAVEL = 0.035

CARRIAGE_LENGTH = 0.044
CARRIAGE_DEPTH = 0.026
CARRIAGE_HEIGHT = 0.054
BORE_RADIUS = 0.0072

FINGER_WIDTH = 0.012
FINGER_DEPTH = 0.015
FINGER_HEIGHT = 0.088
FINGER_OFFSET_X = 0.023
FINGER_CENTER_Y = 0.016
FINGER_BASE_Z = 0.024

FINGER_PAD_WIDTH = 0.016
FINGER_PAD_DEPTH = 0.007
FINGER_PAD_HEIGHT = 0.040
FINGER_PAD_CENTER_Z = 0.040


def _body_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .translate((0.0, BODY_CENTER_Y, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )

    front_pocket = (
        cq.Workplane("XY")
        .box(0.040, 0.020, 0.038, centered=(True, True, False))
        .translate((0.0, 0.001, 0.020))
    )
    housing = housing.cut(front_pocket)

    mount_flange = (
        cq.Workplane("XY")
        .box(
            MOUNT_FLANGE_WIDTH,
            MOUNT_FLANGE_DEPTH,
            MOUNT_FLANGE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, MOUNT_FLANGE_CENTER_Y, MOUNT_FLANGE_BASE_Z))
        .edges("|Z")
        .fillet(0.0025)
    )

    upper_rod = (
        cq.Workplane("YZ")
        .center(ROD_CENTER_Y, UPPER_ROD_Z)
        .circle(ROD_RADIUS)
        .extrude(ROD_LENGTH / 2.0, both=True)
    )
    lower_rod = (
        cq.Workplane("YZ")
        .center(ROD_CENTER_Y, LOWER_ROD_Z)
        .circle(ROD_RADIUS)
        .extrude(ROD_LENGTH / 2.0, both=True)
    )

    upper_bushing = (
        cq.Workplane("XY")
        .box(0.024, 0.010, 0.016, centered=(True, True, True))
        .translate((0.0, 0.008, UPPER_ROD_Z))
        .edges("|Z")
        .fillet(0.0015)
    )
    lower_bushing = (
        cq.Workplane("XY")
        .box(0.024, 0.010, 0.016, centered=(True, True, True))
        .translate((0.0, 0.008, LOWER_ROD_Z))
        .edges("|Z")
        .fillet(0.0015)
    )

    cable_boss = (
        cq.Workplane("XY")
        .box(0.030, 0.020, 0.018, centered=(True, True, False))
        .translate((0.0, BODY_CENTER_Y - 0.005, BODY_HEIGHT))
        .edges("|Z")
        .fillet(0.002)
    )

    return (
        housing.union(mount_flange)
        .union(upper_rod)
        .union(lower_rod)
        .union(upper_bushing)
        .union(lower_bushing)
        .union(cable_boss)
    )


def _jaw_shape(side: str) -> cq.Workplane:
    sign = 1.0 if side == "left" else -1.0

    carriage = cq.Workplane("XY").box(
        CARRIAGE_LENGTH, CARRIAGE_DEPTH, CARRIAGE_HEIGHT, centered=(True, True, True)
    )

    lower_bore = (
        cq.Workplane("YZ")
        .center(0.0, -ROD_SPACING / 2.0)
        .circle(BORE_RADIUS)
        .extrude(CARRIAGE_LENGTH, both=True)
    )
    upper_bore = (
        cq.Workplane("YZ")
        .center(0.0, ROD_SPACING / 2.0)
        .circle(BORE_RADIUS)
        .extrude(CARRIAGE_LENGTH, both=True)
    )
    carriage = carriage.cut(lower_bore).cut(upper_bore)

    front_relief = (
        cq.Workplane("XY")
        .box(0.016, 0.010, 0.024, centered=(True, True, True))
        .translate((-sign * 0.008, 0.008, -0.004))
    )
    carriage = carriage.cut(front_relief)

    finger = (
        cq.Workplane("XY")
        .box(FINGER_WIDTH, FINGER_DEPTH, FINGER_HEIGHT, centered=(True, True, False))
        .translate((sign * FINGER_OFFSET_X, FINGER_CENTER_Y, FINGER_BASE_Z))
        .edges("|Z")
        .fillet(0.0015)
    )

    finger_pad = (
        cq.Workplane("XY")
        .box(
            FINGER_PAD_WIDTH,
            FINGER_PAD_DEPTH,
            FINGER_PAD_HEIGHT,
            centered=(True, True, True),
        )
        .translate(
            (
                sign * (FINGER_OFFSET_X - 0.0015),
                FINGER_CENTER_Y - (FINGER_DEPTH + FINGER_PAD_DEPTH) / 2.0 + 0.001,
                FINGER_PAD_CENTER_Z,
            )
        )
        .edges("|Z")
        .fillet(0.001)
    )

    return carriage.union(finger).union(finger_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_rail_parallel_gripper_head")

    body_mat = model.material("body_anodized", rgba=(0.29, 0.31, 0.34, 1.0))
    jaw_mat = model.material("jaw_black", rgba=(0.14, 0.15, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=body_mat,
        name="body_shell",
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_cadquery(_jaw_shape("left"), "left_jaw_shell"),
        material=jaw_mat,
        name="jaw_shell",
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(_jaw_shape("right"), "right_jaw_shell"),
        material=jaw_mat,
        name="jaw_shell",
    )

    model.articulation(
        "body_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(-JAW_OPEN_X, ROD_CENTER_Y, ROD_MID_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
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
        origin=Origin(xyz=(JAW_OPEN_X, ROD_CENTER_Y, ROD_MID_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.20,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    body_shell = body.get_visual("body_shell")
    left_jaw_shell = left_jaw.get_visual("jaw_shell")
    right_jaw_shell = right_jaw.get_visual("jaw_shell")
    left_joint = object_model.get_articulation("body_to_left_jaw")
    right_joint = object_model.get_articulation("body_to_right_jaw")

    ctx.allow_overlap(
        body,
        left_jaw,
        elem_a=body_shell,
        elem_b=left_jaw_shell,
        reason=(
            "The guide interface is intentionally simplified as hidden rail-bearing "
            "overlap inside the left carriage rather than modeling separate bushings."
        ),
    )
    ctx.allow_overlap(
        body,
        right_jaw,
        elem_a=body_shell,
        elem_b=right_jaw_shell,
        reason=(
            "The guide interface is intentionally simplified as hidden rail-bearing "
            "overlap inside the right carriage rather than modeling separate bushings."
        ),
    )

    ctx.check(
        "left jaw uses prismatic closing axis",
        left_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(left_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={left_joint.articulation_type}, axis={left_joint.axis}",
    )
    ctx.check(
        "right jaw uses mirrored prismatic closing axis",
        right_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(right_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"type={right_joint.articulation_type}, axis={right_joint.axis}",
    )
    ctx.check(
        "jaw travel limits are matched",
        left_joint.motion_limits is not None
        and right_joint.motion_limits is not None
        and left_joint.motion_limits.lower == 0.0
        and right_joint.motion_limits.lower == 0.0
        and abs(left_joint.motion_limits.upper - JAW_TRAVEL) < 1e-9
        and abs(right_joint.motion_limits.upper - JAW_TRAVEL) < 1e-9,
        details=(
            f"left_limits={left_joint.motion_limits}, "
            f"right_limits={right_joint.motion_limits}"
        ),
    )

    ctx.expect_gap(
        right_jaw,
        left_jaw,
        axis="x",
        min_gap=0.070,
        max_gap=0.090,
        name="open jaws leave a realistic grasp gap",
    )
    ctx.expect_overlap(
        left_jaw,
        right_jaw,
        axes="yz",
        min_overlap=0.010,
        name="tall fingers remain aligned in the grasp plane",
    )

    left_rest = ctx.part_world_position(left_jaw)
    right_rest = ctx.part_world_position(right_jaw)
    body_pos = ctx.part_world_position(body)

    ctx.check(
        "jaw carriages share the common guide plane",
        left_rest is not None
        and right_rest is not None
        and body_pos is not None
        and abs(left_rest[1] - right_rest[1]) < 1e-6
        and abs(left_rest[2] - right_rest[2]) < 1e-6
        and abs(left_rest[1] - ROD_CENTER_Y) < 1e-6
        and abs(right_rest[2] - ROD_MID_Z) < 1e-6,
        details=f"left_rest={left_rest}, right_rest={right_rest}, body={body_pos}",
    )

    with ctx.pose({left_joint: JAW_TRAVEL}):
        left_closed = ctx.part_world_position(left_jaw)
        right_held = ctx.part_world_position(right_jaw)
        ctx.check(
            "left jaw closes inward independently",
            left_rest is not None
            and left_closed is not None
            and left_closed[0] > left_rest[0] + 0.030
            and abs(left_closed[1] - left_rest[1]) < 1e-6
            and abs(left_closed[2] - left_rest[2]) < 1e-6,
            details=f"left_rest={left_rest}, left_closed={left_closed}",
        )
        ctx.check(
            "right jaw remains stationary when only left jaw moves",
            right_rest is not None
            and right_held is not None
            and abs(right_held[0] - right_rest[0]) < 1e-6
            and abs(right_held[1] - right_rest[1]) < 1e-6
            and abs(right_held[2] - right_rest[2]) < 1e-6,
            details=f"right_rest={right_rest}, right_held={right_held}",
        )

    with ctx.pose({right_joint: JAW_TRAVEL}):
        left_held = ctx.part_world_position(left_jaw)
        right_closed = ctx.part_world_position(right_jaw)
        ctx.check(
            "right jaw closes inward independently",
            right_rest is not None
            and right_closed is not None
            and right_closed[0] < right_rest[0] - 0.030
            and abs(right_closed[1] - right_rest[1]) < 1e-6
            and abs(right_closed[2] - right_rest[2]) < 1e-6,
            details=f"right_rest={right_rest}, right_closed={right_closed}",
        )
        ctx.check(
            "left jaw remains stationary when only right jaw moves",
            left_rest is not None
            and left_held is not None
            and abs(left_held[0] - left_rest[0]) < 1e-6
            and abs(left_held[1] - left_rest[1]) < 1e-6
            and abs(left_held[2] - left_rest[2]) < 1e-6,
            details=f"left_rest={left_rest}, left_held={left_held}",
        )

    with ctx.pose({left_joint: JAW_TRAVEL, right_joint: JAW_TRAVEL}):
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.008,
            max_gap=0.020,
            name="closed jaws nearly meet without colliding",
        )
        ctx.expect_overlap(
            left_jaw,
            right_jaw,
            axes="yz",
            min_overlap=0.010,
            name="fingers stay vertically aligned at full close",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
