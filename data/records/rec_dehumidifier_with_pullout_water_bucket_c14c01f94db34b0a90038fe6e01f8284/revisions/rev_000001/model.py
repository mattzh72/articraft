from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_DEPTH = 0.25
BODY_WIDTH = 0.38
BODY_HEIGHT = 0.62
BODY_CORNER_RADIUS = 0.022

CAVITY_DEPTH = 0.162
CAVITY_WIDTH = 0.318
CAVITY_HEIGHT = 0.222
CAVITY_BOTTOM = 0.062

POD_DEPTH = 0.11
POD_WIDTH = 0.22
POD_HEIGHT = 0.05
POD_X = 0.07

HANDLE_PIVOT_X = -0.11
HANDLE_PIVOT_Z = BODY_HEIGHT + 0.002
HANDLE_Y = 0.135


def _body_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS)
    )

    bucket_cavity = (
        cq.Workplane("XY")
        .box(CAVITY_DEPTH, CAVITY_WIDTH, CAVITY_HEIGHT, centered=(True, True, False))
        .translate((BODY_DEPTH / 2 - CAVITY_DEPTH / 2 + 0.002, 0.0, CAVITY_BOTTOM))
    )

    control_pod = (
        cq.Workplane("XY")
        .box(POD_DEPTH, POD_WIDTH, POD_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.01)
        .translate((POD_X, 0.0, BODY_HEIGHT))
    )

    pod_front_lip = (
        cq.Workplane("XY")
        .box(0.024, POD_WIDTH - 0.02, 0.018, centered=(True, True, False))
        .translate((POD_X + POD_DEPTH / 2 - 0.012, 0.0, BODY_HEIGHT + 0.006))
    )

    return housing.cut(bucket_cavity).union(control_pod).union(pod_front_lip)
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="basement_dehumidifier")

    body_white = model.material("body_white", rgba=(0.86, 0.87, 0.84, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.55, 0.57, 0.58, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    bucket_gray = model.material("bucket_gray", rgba=(0.73, 0.76, 0.78, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "dehumidifier_body"),
        material=body_white,
        name="housing",
    )

    for suffix, y_sign in (("0", -1.0), ("1", 1.0)):
        body.visual(
            Box((0.018, 0.022, 0.016)),
            origin=Origin(xyz=(HANDLE_PIVOT_X, y_sign * 0.142, BODY_HEIGHT + 0.008)),
            material=trim_gray,
            name=f"handle_mount_{suffix}",
        )

    for suffix, y in (("0", -0.052), ("1", 0.052)):
        body.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(POD_X - 0.01, y, BODY_HEIGHT + POD_HEIGHT + 0.003)),
            material=trim_gray,
            name=f"button_collar_{suffix}",
        )

    body.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(POD_X + 0.02, 0.0, BODY_HEIGHT + POD_HEIGHT + 0.002)),
        material=trim_gray,
        name="dial_base",
    )

    body.visual(
        Box((0.028, 0.008, 0.048)),
        origin=Origin(xyz=(POD_X - 0.015, POD_WIDTH / 2 + 0.004, BODY_HEIGHT + 0.026)),
        material=trim_gray,
        name="rocker_bezel",
    )

    for index, z in enumerate((0.49, 0.465, 0.44, 0.415, 0.39)):
        body.visual(
            Box((0.004, 0.25, 0.008)),
            origin=Origin(xyz=(BODY_DEPTH / 2 + 0.002, 0.0, z)),
            material=trim_gray,
            name=f"front_grille_{index}",
        )

    for suffix, y in (("0", -0.12), ("1", 0.12)):
        body.visual(
            Box((0.048, 0.035, 0.012)),
            origin=Origin(xyz=(-0.07, y, 0.006)),
            material=dark_trim,
            name=f"foot_{suffix}",
        )

    bucket = model.part("bucket")
    bucket.visual(
        Box((0.024, 0.334, 0.228)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=bucket_gray,
        name="bucket_face",
    )
    bucket.visual(
        Box((0.148, 0.296, 0.008)),
        origin=Origin(xyz=(-0.072, 0.0, -0.102)),
        material=bucket_gray,
        name="bucket_bottom",
    )
    bucket.visual(
        Box((0.148, 0.006, 0.178)),
        origin=Origin(xyz=(-0.072, -0.145, -0.011)),
        material=bucket_gray,
        name="bucket_wall_0",
    )
    bucket.visual(
        Box((0.148, 0.006, 0.178)),
        origin=Origin(xyz=(-0.072, 0.145, -0.011)),
        material=bucket_gray,
        name="bucket_wall_1",
    )
    bucket.visual(
        Box((0.006, 0.296, 0.178)),
        origin=Origin(xyz=(-0.145, 0.0, -0.011)),
        material=bucket_gray,
        name="bucket_rear",
    )
    bucket.visual(
        Box((0.016, 0.112, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, 0.056)),
        material=trim_gray,
        name="bucket_pull",
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.10, 0.016, 0.012)),
        origin=Origin(xyz=(0.05, -0.123, 0.006)),
        material=dark_trim,
        name="arm_0",
    )
    handle.visual(
        Box((0.10, 0.016, 0.012)),
        origin=Origin(xyz=(0.05, 0.123, 0.006)),
        material=dark_trim,
        name="arm_1",
    )
    handle.visual(
        Box((0.018, 0.262, 0.016)),
        origin=Origin(xyz=(0.10, 0.0, 0.008)),
        material=dark_trim,
        name="grip_bar",
    )
    model.articulation(
        "body_to_bucket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(BODY_DEPTH / 2, 0.0, CAVITY_BOTTOM + CAVITY_HEIGHT / 2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.10, effort=60.0, velocity=0.20),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, HANDLE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=20.0, velocity=2.5),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_trim,
        name="dial_body",
    )
    timer_dial.visual(
        Cylinder(radius=0.028, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=trim_gray,
        name="dial_skirt",
    )
    timer_dial.visual(
        Box((0.020, 0.004, 0.003)),
        origin=Origin(xyz=(0.010, 0.0, 0.0175)),
        material=body_white,
        name="dial_marker",
    )

    rocker = model.part("power_rocker")
    rocker.visual(
        Box((0.024, 0.012, 0.044)),
        material=dark_trim,
        name="rocker_cap",
    )
    rocker.visual(
        Box((0.018, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=dark_trim,
        name="rocker_pivot",
    )
    rocker.visual(
        Box((0.014, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, 0.0075, 0.012)),
        material=body_white,
        name="rocker_indicator",
    )

    moisture_button_0 = model.part("moisture_button_0")
    moisture_button_0.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_trim,
        name="button_cap",
    )

    moisture_button_1 = model.part("moisture_button_1")
    moisture_button_1.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_trim,
        name="button_cap",
    )

    model.articulation(
        "body_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_dial,
        origin=Origin(xyz=(POD_X + 0.02, 0.0, BODY_HEIGHT + POD_HEIGHT + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(POD_X - 0.015, POD_WIDTH / 2 + 0.018, BODY_HEIGHT + 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.22, upper=0.22, effort=1.0, velocity=3.0),
    )

    model.articulation(
        "body_to_moisture_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=moisture_button_0,
        origin=Origin(xyz=(POD_X - 0.01, -0.052, BODY_HEIGHT + POD_HEIGHT + 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.003, effort=8.0, velocity=0.08),
    )

    model.articulation(
        "body_to_moisture_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=moisture_button_1,
        origin=Origin(xyz=(POD_X - 0.01, 0.052, BODY_HEIGHT + POD_HEIGHT + 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.003, effort=8.0, velocity=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    handle = object_model.get_part("handle")
    timer_dial = object_model.get_part("timer_dial")
    rocker = object_model.get_part("power_rocker")
    moisture_button_0 = object_model.get_part("moisture_button_0")
    moisture_button_1 = object_model.get_part("moisture_button_1")

    bucket_slide = object_model.get_articulation("body_to_bucket")
    handle_hinge = object_model.get_articulation("body_to_handle")
    dial_spin = object_model.get_articulation("body_to_timer_dial")
    rocker_hinge = object_model.get_articulation("body_to_power_rocker")
    button_joint_0 = object_model.get_articulation("body_to_moisture_button_0")
    button_joint_1 = object_model.get_articulation("body_to_moisture_button_1")

    bucket_limits = bucket_slide.motion_limits
    handle_limits = handle_hinge.motion_limits

    if bucket_limits is not None and bucket_limits.upper is not None:
        closed_bucket_pos = None
        extended_bucket_pos = None

        with ctx.pose({bucket_slide: bucket_limits.lower or 0.0}):
            ctx.expect_within(
                bucket,
                body,
                axes="yz",
                margin=0.03,
                name="bucket stays laterally aligned with the front cavity",
            )
            closed_bucket_pos = ctx.part_world_position(bucket)

        with ctx.pose({bucket_slide: bucket_limits.upper}):
            ctx.expect_within(
                bucket,
                body,
                axes="yz",
                margin=0.03,
                name="bucket stays aligned while pulled outward",
            )
            ctx.expect_overlap(
                bucket,
                body,
                axes="x",
                min_overlap=0.04,
                name="bucket remains retained in the body at full extension",
            )
            extended_bucket_pos = ctx.part_world_position(bucket)

        ctx.check(
            "bucket pulls out toward the front",
            closed_bucket_pos is not None
            and extended_bucket_pos is not None
            and extended_bucket_pos[0] > closed_bucket_pos[0] + 0.08,
            details=f"closed={closed_bucket_pos}, extended={extended_bucket_pos}",
        )

    if handle_limits is not None and handle_limits.upper is not None:
        closed_grip = None
        open_grip = None

        with ctx.pose({handle_hinge: handle_limits.lower or 0.0}):
            closed_grip = ctx.part_element_world_aabb(handle, elem="grip_bar")

        with ctx.pose({handle_hinge: handle_limits.upper}):
            open_grip = ctx.part_element_world_aabb(handle, elem="grip_bar")

        ctx.check(
            "handle folds up from the top deck",
            closed_grip is not None
            and open_grip is not None
            and open_grip[1][2] > closed_grip[1][2] + 0.08,
            details=f"closed={closed_grip}, open={open_grip}",
        )

    dial_rest = ctx.part_element_world_aabb(timer_dial, elem="dial_marker")
    dial_quarter = None
    with ctx.pose({dial_spin: math.pi / 2}):
        dial_quarter = ctx.part_element_world_aabb(timer_dial, elem="dial_marker")

    def aabb_center_x(aabb):
        return None if aabb is None else 0.5 * (aabb[0][0] + aabb[1][0])

    def aabb_center_y(aabb):
        return None if aabb is None else 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "timer dial marker rotates around the pod axis",
        dial_rest is not None
        and dial_quarter is not None
        and aabb_center_x(dial_rest) is not None
        and aabb_center_x(dial_quarter) is not None
        and aabb_center_y(dial_rest) is not None
        and aabb_center_y(dial_quarter) is not None
        and aabb_center_x(dial_rest) > aabb_center_x(dial_quarter) + 0.006
        and aabb_center_y(dial_quarter) > aabb_center_y(dial_rest) + 0.006,
        details=f"rest={dial_rest}, quarter={dial_quarter}",
    )

    rocker_limits = rocker_hinge.motion_limits
    if rocker_limits is not None and rocker_limits.lower is not None and rocker_limits.upper is not None:
        rocker_low = None
        rocker_high = None
        with ctx.pose({rocker_hinge: rocker_limits.lower}):
            rocker_low = ctx.part_element_world_aabb(rocker, elem="rocker_indicator")
        with ctx.pose({rocker_hinge: rocker_limits.upper}):
            rocker_high = ctx.part_element_world_aabb(rocker, elem="rocker_indicator")

        ctx.check(
            "power rocker tilts through its short-axis throw",
            rocker_low is not None
            and rocker_high is not None
            and aabb_center_y(rocker_high) is not None
            and aabb_center_y(rocker_low) is not None
            and abs(aabb_center_y(rocker_high) - aabb_center_y(rocker_low)) > 0.004,
            details=f"low={rocker_low}, high={rocker_high}",
        )

    for button, joint, label in (
        (moisture_button_0, button_joint_0, "moisture button 0 depresses downward"),
        (moisture_button_1, button_joint_1, "moisture button 1 depresses downward"),
    ):
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        rest_pos = ctx.part_world_position(button)
        pressed_pos = None
        with ctx.pose({joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            label,
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
