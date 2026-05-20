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


BODY_DEPTH = 0.285
BODY_WIDTH = 0.405
BODY_HEIGHT = 0.620
BODY_BASE_Z = 0.034

BUCKET_FRONT_PROJECTION = 0.024
BUCKET_REAR_DEPTH = 0.206
BUCKET_DEPTH = BUCKET_FRONT_PROJECTION + BUCKET_REAR_DEPTH
BUCKET_WIDTH = 0.318
BUCKET_HEIGHT = 0.228
BUCKET_LIP_X = BODY_DEPTH * 0.5 - 0.020


def _body_shell_mesh() -> object:
    shell = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z").fillet(0.030)
        .edges(">Z").fillet(0.020)
    )

    bucket_cavity = (
        cq.Workplane("XY")
        .box(0.248, 0.338, 0.258, centered=(True, True, False))
        .translate((0.020, 0.0, 0.046))
    )
    upper_recess = (
        cq.Workplane("XY")
        .box(0.090, 0.250, 0.018, centered=(True, True, False))
        .translate((0.005, 0.0, BODY_HEIGHT - 0.010))
    )
    return shell.cut(bucket_cavity).cut(upper_recess)


def _bucket_mesh() -> object:
    outer_center_x = (BUCKET_FRONT_PROJECTION - BUCKET_REAR_DEPTH) * 0.5
    bucket = (
        cq.Workplane("XY")
        .box(BUCKET_DEPTH, BUCKET_WIDTH, BUCKET_HEIGHT)
        .translate((outer_center_x, 0.0, 0.0))
        .edges("|Z").fillet(0.012)
        .edges("%Line and >X").fillet(0.006)
    )

    inner_rear = BUCKET_REAR_DEPTH - 0.003
    inner_front = BUCKET_FRONT_PROJECTION - 0.006
    inner_depth = inner_rear + inner_front
    inner_center_x = (inner_front - inner_rear) * 0.5
    cavity = (
        cq.Workplane("XY")
        .box(inner_depth, BUCKET_WIDTH - 0.010, BUCKET_HEIGHT - 0.006)
        .translate((inner_center_x, 0.0, 0.004))
    )

    grip = (
        cq.Workplane("XY")
        .box(0.028, 0.150, 0.050)
        .translate((BUCKET_FRONT_PROJECTION - 0.008, 0.0, 0.038))
    )
    return bucket.cut(cavity).cut(grip)


def _bucket_bezel_mesh() -> object:
    plate = (
        cq.Workplane("YZ")
        .rect(0.340, 0.270)
        .extrude(0.008)
        .translate((BODY_DEPTH * 0.5 - 0.004, 0.0, BODY_BASE_Z + 0.190))
    )
    opening = (
        cq.Workplane("YZ")
        .rect(0.322, 0.236)
        .extrude(0.012)
        .translate((BODY_DEPTH * 0.5 - 0.006, 0.0, BODY_BASE_Z + 0.182))
    )
    return plate.cut(opening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_dehumidifier")

    body_white = model.material("body_white", rgba=(0.90, 0.92, 0.93, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.68, 0.71, 0.74, 1.0))
    bucket_grey = model.material("bucket_grey", rgba=(0.83, 0.85, 0.87, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.18, 0.20, 0.22, 1.0))
    wheel_grey = model.material("wheel_grey", rgba=(0.28, 0.29, 0.31, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_mesh(), "dehumidifier_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_Z)),
        material=body_white,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_bucket_bezel_mesh(), "dehumidifier_bucket_bezel"),
        material=trim_grey,
        name="bucket_bezel",
    )
    for index in range(7):
        body.visual(
            Box((0.008, 0.280 - 0.014 * abs(index - 3), 0.008)),
            origin=Origin(
                xyz=(
                    BODY_DEPTH * 0.5 - 0.002,
                    0.0,
                    BODY_BASE_Z + 0.382 + index * 0.027,
                )
            ),
            material=trim_grey,
            name=f"front_grille_{index}",
        )
    body.visual(
        Box((0.214, 0.180, 0.006)),
        origin=Origin(xyz=(0.005, 0.0, BODY_BASE_Z + BODY_HEIGHT - 0.007)),
        material=trim_grey,
        name="control_panel",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.003),
        origin=Origin(xyz=(0.010, -0.055, BODY_BASE_Z + BODY_HEIGHT - 0.0045)),
        material=body_white,
        name="dial_seat",
    )
    for index, y_pos in enumerate((-0.012, 0.022, 0.056)):
        body.visual(
            Box((0.034, 0.020, 0.002)),
            origin=Origin(xyz=(0.010, y_pos, BODY_BASE_Z + BODY_HEIGHT - 0.005)),
            material=body_white,
            name=f"button_pad_{index}",
        )
    body.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(
            xyz=(-BODY_DEPTH * 0.5 + 0.002, 0.106, BODY_BASE_Z + 0.168),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_grey,
        name="hose_port",
    )
    for y_sign in (-1.0, 1.0):
        body.visual(
            Box((0.006, 0.004, 0.010)),
            origin=Origin(
                xyz=(
                    -BODY_DEPTH * 0.5 + 0.003,
                    0.106 + y_sign * 0.011,
                    BODY_BASE_Z + 0.192,
                )
            ),
            material=trim_grey,
            name=f"hose_hinge_ear_{0 if y_sign < 0 else 1}",
        )

    wheel_positions = [
        (0.102, -0.150),
        (0.102, 0.150),
        (-0.102, -0.150),
        (-0.102, 0.150),
    ]
    for index, (x_pos, y_pos) in enumerate(wheel_positions):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Box((0.018, 0.003, 0.026)),
                origin=Origin(xyz=(x_pos, y_pos + y_sign * 0.011, 0.021)),
                material=trim_grey,
                name=f"fork_plate_{index}_{0 if y_sign < 0 else 1}",
            )
        body.visual(
            Box((0.018, 0.025, 0.004)),
            origin=Origin(xyz=(x_pos, y_pos, 0.032)),
            material=trim_grey,
            name=f"fork_bridge_{index}",
        )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_bucket_mesh(), "dehumidifier_bucket"),
        material=bucket_grey,
        name="bucket_shell",
    )
    bucket.visual(
        Box((0.002, 0.326, 0.238)),
        origin=Origin(xyz=(BUCKET_FRONT_PROJECTION - 0.001, 0.0, 0.0)),
        material=bucket_grey,
        name="bucket_face",
    )
    bucket.visual(
        Box((0.008, 0.120, 0.024)),
        origin=Origin(xyz=(BUCKET_FRONT_PROJECTION - 0.005, 0.0, 0.060)),
        material=dark_grey,
        name="bucket_handle",
    )

    model.articulation(
        "body_to_bucket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(BUCKET_LIP_X, 0.0, BODY_BASE_Z + 0.182)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.18,
            lower=0.0,
            upper=0.145,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_grey,
        name="dial_skirt",
    )
    dial.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=trim_grey,
        name="dial_body",
    )
    dial.visual(
        Box((0.016, 0.004, 0.002)),
        origin=Origin(xyz=(0.010, 0.0, 0.019)),
        material=body_white,
        name="dial_pointer",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.010, -0.055, BODY_BASE_Z + BODY_HEIGHT - 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=6.0,
        ),
    )

    button_positions = (-0.012, 0.022, 0.056)
    for index, y_pos in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.028, 0.014, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=dark_grey,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.010, y_pos, BODY_BASE_Z + BODY_HEIGHT - 0.004)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0018,
            ),
        )

    hose_cap = model.part("hose_cap")
    hose_cap.visual(
        Cylinder(radius=0.003, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=trim_grey,
        name="cap_barrel",
    )
    hose_cap.visual(
        Box((0.004, 0.032, 0.032)),
        origin=Origin(xyz=(-0.002, 0.0, -0.018)),
        material=trim_grey,
        name="cap_cover",
    )
    model.articulation(
        "body_to_hose_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hose_cap,
        origin=Origin(xyz=(-BODY_DEPTH * 0.5 - 0.002, 0.106, BODY_BASE_Z + 0.192)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.55,
        ),
    )

    for index, (x_pos, y_pos) in enumerate(wheel_positions):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.017, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=wheel_grey,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=trim_grey,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.003, length=0.019),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=trim_grey,
            name="axle",
        )
        model.articulation(
            f"body_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x_pos, y_pos, 0.017)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=12.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    dial = object_model.get_part("dial")
    hose_cap = object_model.get_part("hose_cap")
    button_0 = object_model.get_part("button_0")
    bucket_slide = object_model.get_articulation("body_to_bucket")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_joint = object_model.get_articulation("body_to_button_0")
    hose_cap_joint = object_model.get_articulation("body_to_hose_cap")
    limits = bucket_slide.motion_limits

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    if limits is not None and limits.upper is not None:
        rest_pos = ctx.part_world_position(bucket)
        with ctx.pose({bucket_slide: limits.upper}):
            extended_pos = ctx.part_world_position(bucket)
            ctx.expect_overlap(
                bucket,
                body,
                axes="yz",
                min_overlap=0.180,
                name="bucket stays aligned with the lower opening",
            )
            ctx.expect_overlap(
                bucket,
                body,
                axes="x",
                min_overlap=0.055,
                name="bucket retains insertion when extended",
            )
        ctx.check(
            "bucket pulls forward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.12,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    rest_pointer = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_pointer"))
    with ctx.pose({dial_joint: math.pi * 0.5}):
        quarter_pointer = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_pointer"))
    ctx.check(
        "humidistat dial rotates about its center axis",
        rest_pointer is not None
        and quarter_pointer is not None
        and quarter_pointer[1] > rest_pointer[1] + 0.008
        and quarter_pointer[0] < rest_pointer[0] - 0.008,
        details=f"rest={rest_pointer}, quarter_turn={quarter_pointer}",
    )

    button_limits = button_joint.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        rest_button_pos = ctx.part_world_position(button_0)
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_button_pos = ctx.part_world_position(button_0)
        ctx.check(
            "preset button presses downward",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[2] < rest_button_pos[2] - 0.0015,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    hose_limits = hose_cap_joint.motion_limits
    if hose_limits is not None and hose_limits.upper is not None:
        rest_cap_aabb = ctx.part_world_aabb(hose_cap)
        with ctx.pose({hose_cap_joint: min(1.2, hose_limits.upper)}):
            open_cap_aabb = ctx.part_world_aabb(hose_cap)
        ctx.check(
            "hose cap swings outward from the rear panel",
            rest_cap_aabb is not None
            and open_cap_aabb is not None
            and float(open_cap_aabb[0][0]) < float(rest_cap_aabb[0][0]) - 0.020,
            details=f"rest={rest_cap_aabb}, open={open_cap_aabb}",
        )

    wheel_checks_ok = True
    wheel_details: list[str] = []
    for index in range(4):
        wheel_joint = object_model.get_articulation(f"body_to_wheel_{index}")
        wheel_limits = wheel_joint.motion_limits
        joint_ok = (
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and wheel_limits is not None
            and wheel_limits.lower is None
            and wheel_limits.upper is None
        )
        wheel_checks_ok = wheel_checks_ok and joint_ok
        wheel_details.append(
            f"{wheel_joint.name}: type={wheel_joint.articulation_type}, "
            f"lower={None if wheel_limits is None else wheel_limits.lower}, "
            f"upper={None if wheel_limits is None else wheel_limits.upper}"
        )
    ctx.check(
        "caster wheels use continuous axle joints",
        wheel_checks_ok,
        details="; ".join(wheel_details),
    )

    return ctx.report()


object_model = build_object_model()
