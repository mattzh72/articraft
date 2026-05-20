from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


ARM_DARK = Material("graphite_black", rgba=(0.015, 0.016, 0.018, 1.0))
ARM_EDGE = Material("soft_black_edges", rgba=(0.045, 0.047, 0.050, 1.0))
CAST_METAL = Material("dark_cast_metal", rgba=(0.12, 0.125, 0.13, 1.0))
WASHER = Material("brushed_pin_caps", rgba=(0.52, 0.54, 0.56, 1.0))
PLATE = Material("matte_display_plate", rgba=(0.025, 0.027, 0.030, 1.0))


def _arm_bar_origin(end_xyz: tuple[float, float, float], y: float) -> Origin:
    dx, _, dz = end_xyz
    length = math.hypot(dx, dz)
    pitch = -math.atan2(dz, dx)
    return Origin(xyz=(dx * 0.5, y, dz * 0.5), rpy=(0.0, pitch, 0.0))


def _arm_bar_length(end_xyz: tuple[float, float, float]) -> float:
    dx, _, dz = end_xyz
    return math.hypot(dx, dz)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_monitor_mount")

    shoulder_xyz = (0.08, 0.0, 0.82)
    lower_end = (0.46, 0.0, 0.14)
    lower_bar_end = (0.405, 0.0, 0.123)
    upper_end = (0.38, 0.0, -0.10)
    tilt_origin = (0.085, 0.0, 0.060)

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((0.040, 0.420, 0.050)),
        origin=Origin(xyz=(-0.050, 0.0, 1.140)),
        material=CAST_METAL,
        name="top_bridge",
    )
    rear_frame.visual(
        Box((0.040, 0.420, 0.050)),
        origin=Origin(xyz=(-0.050, 0.0, 0.500)),
        material=CAST_METAL,
        name="bottom_bridge",
    )
    for idx, y in enumerate((-0.185, 0.185)):
        rear_frame.visual(
            Box((0.040, 0.050, 0.660)),
            origin=Origin(xyz=(-0.050, y, 0.820)),
            material=CAST_METAL,
            name=f"side_rail_{idx}",
        )
    rear_frame.visual(
        Box((0.038, 0.070, 0.660)),
        origin=Origin(xyz=(-0.050, 0.0, 0.820)),
        material=CAST_METAL,
        name="center_spine",
    )
    rear_frame.visual(
        Box((0.160, 0.230, 0.035)),
        origin=Origin(xyz=(0.015, 0.0, 0.890)),
        material=ARM_EDGE,
        name="upper_shoulder_bridge",
    )
    rear_frame.visual(
        Box((0.160, 0.230, 0.035)),
        origin=Origin(xyz=(0.015, 0.0, 0.750)),
        material=ARM_EDGE,
        name="lower_shoulder_bridge",
    )
    for idx, (y, cheek_name, cap_name) in enumerate(
        (
            (-0.095, "shoulder_cheek_0", "shoulder_cap_0"),
            (0.095, "shoulder_cheek_1", "shoulder_cap_1"),
        )
    ):
        rear_frame.visual(
            Box((0.075, 0.035, 0.160)),
            origin=Origin(xyz=(shoulder_xyz[0], y, shoulder_xyz[2])),
            material=ARM_EDGE,
            name=cheek_name,
        )
        cap_y = y + (0.020 if y > 0.0 else -0.020)
        rear_frame.visual(
            Cylinder(radius=0.032, length=0.008),
            origin=Origin(xyz=(shoulder_xyz[0], cap_y, shoulder_xyz[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=WASHER,
            name=cap_name,
        )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.052, length=0.155),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ARM_EDGE,
        name="shoulder_hub",
    )
    lower_len = _arm_bar_length(lower_bar_end)
    for idx, y in enumerate((-0.058, 0.058)):
        lower_arm.visual(
            Box((lower_len, 0.024, 0.032)),
            origin=_arm_bar_origin(lower_bar_end, y),
            material=ARM_DARK,
            name=f"outer_bar_{idx}",
        )
    lower_arm.visual(
        Box((lower_len * 0.72, 0.116, 0.020)),
        origin=Origin(
            xyz=(lower_bar_end[0] * 0.50, 0.0, lower_bar_end[2] * 0.50),
            rpy=(0.0, -math.atan2(lower_bar_end[2], lower_bar_end[0]), 0.0),
        ),
        material=ARM_EDGE,
        name="center_web",
    )
    for y, cheek_name in ((-0.084, "elbow_cheek_0"), (0.084, "elbow_cheek_1")):
        lower_arm.visual(
            Box((0.110, 0.036, 0.130)),
            origin=Origin(xyz=(lower_end[0], y, lower_end[2])),
            material=ARM_EDGE,
            name=cheek_name,
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.046, length=0.132),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ARM_EDGE,
        name="elbow_hub",
    )
    upper_len = _arm_bar_length(upper_end)
    for idx, y in enumerate((-0.055, 0.055)):
        upper_arm.visual(
            Box((upper_len, 0.018, 0.028)),
            origin=_arm_bar_origin(upper_end, y),
            material=ARM_DARK,
            name=f"inner_bar_{idx}",
        )
    upper_arm.visual(
        Box((upper_len * 0.70, 0.110, 0.018)),
        origin=Origin(
            xyz=(upper_end[0] * 0.50, 0.0, upper_end[2] * 0.50),
            rpy=(0.0, -math.atan2(upper_end[2], upper_end[0]), 0.0),
        ),
        material=ARM_EDGE,
        name="narrow_web",
    )
    upper_arm.visual(
        Box((0.090, 0.120, 0.016)),
        origin=Origin(xyz=(upper_end[0], 0.0, upper_end[2] - 0.008)),
        material=ARM_EDGE,
        name="swivel_seat",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.042, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=CAST_METAL,
        name="turntable",
    )
    swivel.visual(
        Box((0.075, 0.040, 0.012)),
        origin=Origin(xyz=(0.038, 0.0, 0.026)),
        material=CAST_METAL,
        name="short_neck",
    )
    swivel.visual(
        Box((0.055, 0.145, 0.020)),
        origin=Origin(xyz=(0.075, 0.0, 0.025)),
        material=CAST_METAL,
        name="tilt_bridge",
    )
    for idx, y in enumerate((-0.060, 0.060)):
        swivel.visual(
            Box((0.040, 0.024, 0.090)),
            origin=Origin(xyz=(tilt_origin[0], y, tilt_origin[2])),
            material=CAST_METAL,
            name=f"tilt_ear_{idx}",
        )

    display_plate = model.part("display_plate")
    display_plate.visual(
        Cylinder(radius=0.023, length=0.096),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ARM_EDGE,
        name="tilt_barrel",
    )
    display_plate.visual(
        Box((0.068, 0.055, 0.035)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=ARM_EDGE,
        name="plate_neck",
    )
    display_plate.visual(
        Box((0.014, 0.200, 0.145)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=PLATE,
        name="vesa_plate",
    )
    screw_positions = (
        (-0.065, -0.045),
        (-0.065, 0.045),
        (0.065, -0.045),
        (0.065, 0.045),
    )
    for idx, (y, z) in enumerate(screw_positions):
        display_plate.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(0.085, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=WASHER,
            name=f"screw_boss_{idx}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=lower_arm,
        origin=Origin(xyz=shoulder_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.55, upper=0.80),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=lower_end),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=swivel,
        origin=Origin(xyz=upper_end),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=display_plate,
        origin=Origin(xyz=tilt_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.6, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_frame = object_model.get_part("rear_frame")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    swivel = object_model.get_part("swivel")
    display_plate = object_model.get_part("display_plate")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "four revolute user mechanisms",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (shoulder, elbow, head_swivel, head_tilt)),
        details="shoulder, elbow, head swivel, and head tilt must all be revolute joints",
    )
    ctx.expect_overlap(
        lower_arm,
        rear_frame,
        axes="xz",
        elem_a="shoulder_hub",
        elem_b="shoulder_cheek_0",
        min_overlap=0.050,
        name="shoulder hub is captured by rear yoke",
    )
    ctx.expect_overlap(
        upper_arm,
        lower_arm,
        axes="xz",
        elem_a="elbow_hub",
        elem_b="elbow_cheek_0",
        min_overlap=0.050,
        name="elbow hub is captured by lower arm clevis",
    )
    ctx.expect_gap(
        swivel,
        upper_arm,
        axis="z",
        positive_elem="turntable",
        negative_elem="swivel_seat",
        min_gap=0.0,
        max_gap=0.001,
        name="swivel turntable sits on compact seat",
    )

    rear_aabb = ctx.part_world_aabb(rear_frame)
    plate_aabb = ctx.part_world_aabb(display_plate)
    if rear_aabb is not None and plate_aabb is not None:
        rear_depth = rear_aabb[1][0] - rear_aabb[0][0]
        plate_width = plate_aabb[1][1] - plate_aabb[0][1]
        ctx.check(
            "head plate remains compact compared with arm reach",
            plate_width < 0.55 and rear_depth < 0.25,
            details=f"rear_depth={rear_depth:.3f}, plate_width={plate_width:.3f}",
        )

    rest_plate = ctx.part_world_position(display_plate)
    with ctx.pose({shoulder: 0.35, elbow: -0.40}):
        posed_plate = ctx.part_world_position(display_plate)
    ctx.check(
        "shoulder and elbow reposition the head",
        rest_plate is not None
        and posed_plate is not None
        and math.dist(posed_plate, rest_plate) > 0.050,
        details=f"rest={rest_plate}, posed={posed_plate}",
    )

    with ctx.pose({head_swivel: 0.60}):
        swiveled_plate = ctx.part_world_position(display_plate)
    ctx.check(
        "head swivel yaws the plate bracket",
        rest_plate is not None
        and swiveled_plate is not None
        and abs(swiveled_plate[1] - rest_plate[1]) > 0.030,
        details=f"rest={rest_plate}, swiveled={swiveled_plate}",
    )

    rest_plate_aabb = ctx.part_world_aabb(display_plate)
    with ctx.pose({head_tilt: 0.55}):
        tilted_plate_aabb = ctx.part_world_aabb(display_plate)
    if rest_plate_aabb is not None and tilted_plate_aabb is not None:
        rest_x = rest_plate_aabb[1][0] - rest_plate_aabb[0][0]
        tilted_x = tilted_plate_aabb[1][0] - tilted_plate_aabb[0][0]
        ctx.check(
            "head tilt pitches the display plate",
            tilted_x > rest_x + 0.015,
            details=f"rest_x={rest_x:.3f}, tilted_x={tilted_x:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
