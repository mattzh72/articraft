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

BODY_WIDTH_X = 0.064
BODY_WIDTH_Y = 0.042
SWIVEL_AXIS_Z = 0.136
PITCH_AXIS_Z = 0.034
DOOR_WIDTH_X = 0.030
DOOR_HEIGHT_Z = 0.074
DOOR_THICKNESS_Y = 0.0024
DOOR_CENTER_X = -0.002
DOOR_CENTER_Z = 0.102
DOOR_HINGE_X = DOOR_CENTER_X - (DOOR_WIDTH_X * 0.5)
DOOR_HINGE_Y = (BODY_WIDTH_Y * 0.5) + 0.0006


def _rounded_box(size: tuple[float, float, float], center: tuple[float, float, float], fillet: float) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size).translate(center)
    if fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid


def _build_body_shape() -> cq.Workplane:
    foot = _rounded_box((0.060, 0.042, 0.008), (0.0, 0.0, 0.004), 0.002)
    shoe_block = _rounded_box((0.034, 0.020, 0.020), (0.0, 0.0, 0.014), 0.0015)
    lower_neck = _rounded_box((0.042, 0.030, 0.016), (0.0, 0.0, 0.030), 0.002)
    lower_body = _rounded_box((BODY_WIDTH_X, BODY_WIDTH_Y, 0.056), (0.0, 0.0, 0.064), 0.005)
    upper_body = _rounded_box((0.058, 0.040, 0.038), (-0.003, 0.0, 0.111), 0.0045)
    swivel_socket = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.006)
        .translate((0.0, 0.0, 0.130))
    )

    shell = foot.union(shoe_block).union(lower_neck).union(lower_body).union(upper_body).union(swivel_socket)
    door_pocket = cq.Workplane("XY").box(
        DOOR_WIDTH_X + 0.003,
        0.0042,
        DOOR_HEIGHT_Z + 0.004,
    ).translate((DOOR_CENTER_X, 0.0190, DOOR_CENTER_Z))
    return shell.cut(door_pocket)


def _build_yoke_shape() -> cq.Workplane:
    swivel_collar = cq.Workplane("XY").circle(0.020).extrude(0.012)
    center_post = _rounded_box((0.010, 0.026, 0.016), (-0.010, 0.0, 0.018), 0.001)
    rear_bridge = _rounded_box((0.008, 0.096, 0.010), (-0.013, 0.0, 0.014), 0.001)
    left_arm = _rounded_box((0.010, 0.006, 0.028), (-0.006, 0.045, 0.026), 0.001)
    right_arm = _rounded_box((0.010, 0.006, 0.028), (-0.006, -0.045, 0.026), 0.001)
    left_trunnion = cq.Workplane("XZ").circle(0.009).extrude(0.006).translate((0.0, 0.042, PITCH_AXIS_Z))
    right_trunnion = cq.Workplane("XZ").circle(0.009).extrude(0.006).translate((0.0, -0.048, PITCH_AXIS_Z))
    return (
        swivel_collar.union(center_post)
        .union(rear_bridge)
        .union(left_arm)
        .union(right_arm)
        .union(left_trunnion)
        .union(right_trunnion)
    )


def _build_head_shape() -> cq.Workplane:
    main_shell = _rounded_box((0.050, 0.078, 0.036), (0.024, 0.0, 0.004), 0.0035)
    rear_cap = _rounded_box((0.006, 0.070, 0.030), (0.001, 0.0, 0.004), 0.0012)
    return main_shell.union(rear_cap)


def _build_door_shape() -> cq.Workplane:
    panel = _rounded_box((DOOR_WIDTH_X, DOOR_THICKNESS_Y, DOOR_HEIGHT_Z), (DOOR_WIDTH_X * 0.5, -DOOR_THICKNESS_Y * 0.5, 0.0), 0.0008)
    upper_knuckle = cq.Workplane("XY").circle(0.0014).extrude(0.018).translate((0.0, 0.0, 0.019))
    lower_knuckle = cq.Workplane("XY").circle(0.0014).extrude(0.018).translate((0.0, 0.0, -0.037))
    thumb_relief = _rounded_box((0.005, 0.0012, 0.016), (DOOR_WIDTH_X - 0.0025, -0.0018, -0.020), 0.0004)
    return panel.union(upper_knuckle).union(lower_knuckle).union(thumb_relief)


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return (
        float(maxs[0] - mins[0]),
        float(maxs[1] - mins[1]),
        float(maxs[2] - mins[2]),
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return (
        float((mins[0] + maxs[0]) * 0.5),
        float((mins[1] + maxs[1]) * 0.5),
        float((mins[2] + maxs[2]) * 0.5),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_camera_flash")

    matte_black = model.material("matte_black", rgba=(0.13, 0.14, 0.15, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.09, 0.10, 0.11, 1.0))
    diffuser = model.material("diffuser", rgba=(0.90, 0.91, 0.92, 0.95))
    screen_glass = model.material("screen_glass", rgba=(0.14, 0.20, 0.24, 0.55))
    metal = model.material("shoe_metal", rgba=(0.55, 0.57, 0.60, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "flash_body_shell"),
        material=charcoal,
        name="body_shell",
    )
    body.visual(
        Box((0.024, 0.016, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.002, 0.026, 0.040)),
        origin=Origin(xyz=(-0.031, 0.0, 0.101)),
        material=screen_glass,
        name="rear_screen",
    )
    body.visual(
        Box((0.0025, 0.028, 0.018)),
        origin=Origin(xyz=(-0.0315, 0.0, 0.066)),
        material=dark_trim,
        name="rear_pad",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=matte_black,
        name="swivel_collar",
    )
    yoke.visual(
        Box((0.010, 0.026, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, 0.017)),
        material=matte_black,
        name="center_post",
    )
    yoke.visual(
        Box((0.008, 0.096, 0.010)),
        origin=Origin(xyz=(-0.013, 0.0, 0.014)),
        material=matte_black,
        name="rear_bridge",
    )
    yoke.visual(
        Box((0.010, 0.006, 0.028)),
        origin=Origin(xyz=(-0.006, 0.045, 0.026)),
        material=matte_black,
        name="arm_0",
    )
    yoke.visual(
        Box((0.010, 0.006, 0.028)),
        origin=Origin(xyz=(-0.006, -0.045, 0.026)),
        material=matte_black,
        name="arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.045, PITCH_AXIS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="trunnion_0",
    )
    yoke.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, -0.045, PITCH_AXIS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="trunnion_1",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_build_head_shape(), "flash_head_shell"),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.072, 0.028)),
        origin=Origin(xyz=(0.047, 0.0, 0.004)),
        material=diffuser,
        name="flash_window",
    )
    head.visual(
        Cylinder(radius=0.0085, length=0.006),
        origin=Origin(xyz=(0.0, 0.039, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="pivot_0",
    )
    head.visual(
        Cylinder(radius=0.0085, length=0.006),
        origin=Origin(xyz=(0.0, -0.039, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="pivot_1",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        mesh_from_cadquery(_build_door_shape(), "flash_battery_door"),
        material=charcoal,
        name="panel_skin",
    )

    model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-0.20,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=0.0,
            upper=1.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    head = object_model.get_part("head")
    battery_door = object_model.get_part("battery_door")
    yoke = object_model.get_part("yoke")
    swivel = object_model.get_articulation("body_to_yoke")
    pitch = object_model.get_articulation("yoke_to_head")
    door_hinge = object_model.get_articulation("body_to_battery_door")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "dslr_flash_body_scale",
        body_aabb is not None and 0.12 <= _aabb_size(body_aabb)[2] <= 0.15,
        details=f"body_aabb={body_aabb!r}",
    )

    ctx.expect_within(
        head,
        yoke,
        axes="y",
        margin=0.010,
        elem_a="head_shell",
        name="head stays between yoke sides",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.015,
        name="head rests clearly above the body shell",
    )
    ctx.expect_within(
        battery_door,
        body,
        axes="xz",
        margin=0.006,
        elem_a="panel_skin",
        elem_b="body_shell",
        name="battery door sits within the side opening footprint",
    )

    closed_body_aabb = ctx.part_world_aabb(body)
    closed_door_aabb = ctx.part_world_aabb(battery_door)
    ctx.check(
        "battery_door_closed_flush",
        closed_body_aabb is not None
        and closed_door_aabb is not None
        and closed_door_aabb[0][1] >= closed_body_aabb[1][1] - 0.0035
        and closed_door_aabb[1][1] <= closed_body_aabb[1][1] + 0.0025,
        details=f"body_aabb={closed_body_aabb!r}, door_aabb={closed_door_aabb!r}",
    )

    pitch_limits = pitch.motion_limits
    if pitch_limits is not None and pitch_limits.lower is not None and pitch_limits.upper is not None:
        rest_head_aabb = ctx.part_world_aabb(head)
        with ctx.pose({pitch: pitch_limits.upper}):
            pitched_up_aabb = ctx.part_world_aabb(head)
            ctx.check(
                "head_pitch_raises_flash_window",
                rest_head_aabb is not None
                and pitched_up_aabb is not None
                and pitched_up_aabb[1][2] > rest_head_aabb[1][2] + 0.012,
                details=f"rest={rest_head_aabb!r}, pitched={pitched_up_aabb!r}",
            )
        with ctx.pose({pitch: pitch_limits.lower}):
            ctx.expect_gap(
                head,
                body,
                axis="z",
                min_gap=0.008,
                name="downward pitch still clears the body",
            )

    with ctx.pose({swivel: math.pi / 2.0}):
        swiveled_head_aabb = ctx.part_world_aabb(head)
        ctx.check(
            "head_swivel_turns_sideways",
            swiveled_head_aabb is not None and _aabb_center(swiveled_head_aabb)[1] > 0.010,
            details=f"swiveled_head_aabb={swiveled_head_aabb!r}",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            open_door_aabb = ctx.part_world_aabb(battery_door)
            open_body_aabb = ctx.part_world_aabb(body)
            ctx.check(
                "battery_door_swings_outward",
                open_door_aabb is not None
                and open_body_aabb is not None
                and open_door_aabb[1][1] > open_body_aabb[1][1] + 0.015,
                details=f"body_aabb={open_body_aabb!r}, open_door_aabb={open_door_aabb!r}",
            )

    return ctx.report()


object_model = build_object_model()
