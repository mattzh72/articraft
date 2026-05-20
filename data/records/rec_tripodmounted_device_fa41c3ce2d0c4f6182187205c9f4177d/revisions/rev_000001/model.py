from __future__ import annotations

import math

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


def add_box(
    part,
    size,
    xyz,
    *,
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_cylinder(
    part,
    radius,
    length,
    xyz,
    *,
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_leg_geometry(part, *, tube_material, collar_material, foot_material) -> None:
    leg_pitch = math.radians(157.0)
    direction_x = math.sin(leg_pitch)
    direction_z = math.cos(leg_pitch)

    def along(distance: float) -> tuple[float, float, float]:
        return (distance * direction_x, 0.0, distance * direction_z)

    add_box(
        part,
        (0.040, 0.0482, 0.030),
        (0.020, 0.0, -0.012),
        material=collar_material,
        name="hinge_collar",
    )
    add_cylinder(
        part,
        0.028,
        0.542,
        along(0.290),
        rpy=(0.0, leg_pitch, 0.0),
        material=tube_material,
        name="upper_tube",
    )
    add_box(
        part,
        (0.060, 0.054, 0.040),
        along(0.50),
        rpy=(0.0, leg_pitch, 0.0),
        material=collar_material,
        name="leg_clamp",
    )
    add_cylinder(
        part,
        0.022,
        0.72,
        along(0.78),
        rpy=(0.0, leg_pitch, 0.0),
        material=tube_material,
        name="lower_tube",
    )
    add_cylinder(
        part,
        0.027,
        0.05,
        along(1.13),
        rpy=(0.0, leg_pitch, 0.0),
        material=foot_material,
        name="foot",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pro_tripod_camera")

    cast_metal = model.material("cast_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    tube_metal = model.material("tube_metal", rgba=(0.15, 0.16, 0.17, 1.0))
    brushed_alloy = model.material("brushed_alloy", rgba=(0.58, 0.60, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    camera_black = model.material("camera_black", rgba=(0.09, 0.09, 0.10, 1.0))
    gear_accent = model.material("gear_accent", rgba=(0.78, 0.14, 0.14, 1.0))

    crown = model.part("crown")
    add_box(
        crown,
        (0.090, 0.010, 0.220),
        (0.0, 0.035, 0.0),
        material=cast_metal,
        name="sleeve_front",
    )
    add_box(
        crown,
        (0.090, 0.010, 0.220),
        (0.0, -0.035, 0.0),
        material=cast_metal,
        name="sleeve_back",
    )
    add_box(
        crown,
        (0.010, 0.064, 0.220),
        (0.040, 0.0, 0.0),
        material=cast_metal,
        name="sleeve_right",
    )
    add_box(
        crown,
        (0.010, 0.064, 0.220),
        (-0.040, 0.0, 0.0),
        material=cast_metal,
        name="sleeve_left",
    )
    add_box(
        crown,
        (0.022, 0.030, 0.012),
        (0.0, 0.070, 0.014),
        material=brushed_alloy,
        name="crank_boss_upper",
    )
    add_box(
        crown,
        (0.022, 0.030, 0.012),
        (0.0, 0.070, -0.024),
        material=brushed_alloy,
        name="crank_boss_lower",
    )
    add_box(
        crown,
        (0.018, 0.040, 0.056),
        (0.0, 0.050, -0.005),
        material=brushed_alloy,
        name="crank_brace",
    )

    leg_yaws = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, yaw in enumerate(leg_yaws):
        add_box(
            crown,
            (0.140, 0.060, 0.030),
            (0.108 * math.cos(yaw), 0.108 * math.sin(yaw), -0.070),
            rpy=(0.0, 0.0, yaw),
            material=cast_metal,
            name=f"arm_{index}",
        )
        add_box(
            crown,
            (0.080, 0.012, 0.040),
            (
                0.160 * math.cos(yaw) - 0.034 * math.sin(yaw),
                0.160 * math.sin(yaw) + 0.034 * math.cos(yaw),
                -0.070,
            ),
            rpy=(0.0, 0.0, yaw),
            material=brushed_alloy,
            name=f"leg_cheek_{index}_0",
        )
        add_box(
            crown,
            (0.080, 0.012, 0.040),
            (
                0.160 * math.cos(yaw) + 0.034 * math.sin(yaw),
                0.160 * math.sin(yaw) - 0.034 * math.cos(yaw),
                -0.070,
            ),
            rpy=(0.0, 0.0, yaw),
            material=brushed_alloy,
            name=f"leg_cheek_{index}_1",
        )

    column = model.part("column")
    add_box(
        column,
        (0.050, 0.050, 0.880),
        (0.0, 0.0, 0.080),
        material=tube_metal,
        name="mast",
    )
    add_box(
        column,
        (0.008, 0.003, 0.660),
        (0.029, 0.0, 0.030),
        material=gear_accent,
        name="rack",
    )
    add_box(
        column,
        (0.072, 0.072, 0.030),
        (0.0, 0.0, 0.535),
        material=brushed_alloy,
        name="carriage",
    )
    add_box(
        column,
        (0.080, 0.080, 0.008),
        (0.0, 0.0, 0.004),
        material=brushed_alloy,
        name="guide_plate",
    )

    crank = model.part("crank")
    add_cylinder(
        crank,
        0.012,
        0.024,
        (0.0, 0.012, 0.0),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        material=brushed_alloy,
        name="hub",
    )
    add_box(
        crank,
        (0.016, 0.020, 0.080),
        (0.0, 0.018, -0.070),
        material=cast_metal,
        name="arm",
    )
    add_box(
        crank,
        (0.012, 0.020, 0.040),
        (0.0, 0.030, -0.020),
        material=cast_metal,
        name="arm_root",
    )
    add_cylinder(
        crank,
        0.014,
        0.050,
        (0.0, 0.018, -0.112),
        rpy=(0.0, math.pi / 2.0, 0.0),
        material=rubber,
        name="grip",
    )

    pan_head = model.part("pan_head")
    add_cylinder(
        pan_head,
        0.060,
        0.024,
        (0.0, 0.0, 0.012),
        material=brushed_alloy,
        name="pan_base",
    )
    add_cylinder(
        pan_head,
        0.028,
        0.042,
        (0.0, 0.0, 0.036),
        material=brushed_alloy,
        name="pan_neck",
    )
    add_box(
        pan_head,
        (0.090, 0.180, 0.020),
        (0.0, 0.0, 0.055),
        material=cast_metal,
        name="tilt_bridge",
    )
    add_box(
        pan_head,
        (0.082, 0.010, 0.140),
        (0.0, 0.085, 0.130),
        material=cast_metal,
        name="yoke_side_0",
    )
    add_box(
        pan_head,
        (0.082, 0.010, 0.140),
        (0.0, -0.085, 0.130),
        material=cast_metal,
        name="yoke_side_1",
    )

    camera = model.part("camera")
    add_box(
        camera,
        (0.220, 0.120, 0.120),
        (0.050, 0.0, 0.0),
        material=camera_black,
        name="body",
    )
    add_box(
        camera,
        (0.070, 0.100, 0.105),
        (-0.085, 0.0, 0.0),
        material=cast_metal,
        name="battery",
    )
    add_cylinder(
        camera,
        0.035,
        0.140,
        (0.190, 0.0, 0.0),
        rpy=(0.0, math.pi / 2.0, 0.0),
        material=camera_black,
        name="lens_barrel",
    )
    add_box(
        camera,
        (0.120, 0.025, 0.030),
        (0.040, 0.0, 0.075),
        material=cast_metal,
        name="top_handle",
    )
    add_box(
        camera,
        (0.070, 0.030, 0.040),
        (-0.010, 0.0, 0.080),
        material=camera_black,
        name="viewfinder",
    )
    add_cylinder(
        camera,
        0.015,
        0.020,
        (0.0, 0.070, 0.0),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        material=brushed_alloy,
        name="hub_0",
    )
    add_cylinder(
        camera,
        0.015,
        0.020,
        (0.0, -0.070, 0.0),
        rpy=(-math.pi / 2.0, 0.0, 0.0),
        material=brushed_alloy,
        name="hub_1",
    )

    legs = []
    for index in range(3):
        leg = model.part(f"leg_{index}")
        add_leg_geometry(
            leg,
            tube_material=tube_metal,
            collar_material=brushed_alloy,
            foot_material=rubber,
        )
        legs.append(leg)

    leg_radius = 0.1925
    for index, (leg, yaw) in enumerate(zip(legs, leg_yaws)):
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(leg_radius * math.cos(yaw), leg_radius * math.sin(yaw), -0.070),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=1.0,
                lower=-0.45,
                upper=0.25,
            ),
        )

    model.articulation(
        "crown_to_column",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.08,
            lower=0.0,
            upper=0.26,
        ),
    )
    model.articulation(
        "crown_to_crank",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=crank,
        origin=Origin(xyz=(0.0, 0.070, -0.005)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=6.0),
    )
    model.articulation(
        "column_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=5.0),
    )
    model.articulation(
        "pan_head_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-1.0,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    column = object_model.get_part("column")
    camera = object_model.get_part("camera")

    lift = object_model.get_articulation("crown_to_column")
    crank = object_model.get_articulation("crown_to_crank")
    pan = object_model.get_articulation("column_to_pan_head")
    tilt = object_model.get_articulation("pan_head_to_camera")

    def aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    mechanism_ok = (
        lift.articulation_type == ArticulationType.PRISMATIC
        and crank.articulation_type == ArticulationType.CONTINUOUS
        and pan.articulation_type == ArticulationType.CONTINUOUS
        and tilt.articulation_type == ArticulationType.REVOLUTE
    )
    ctx.check(
        "primary tripod mechanisms are articulated",
        mechanism_ok,
        details=(
            f"lift={lift.articulation_type}, crank={crank.articulation_type}, "
            f"pan={pan.articulation_type}, tilt={tilt.articulation_type}"
        ),
    )

    lift_upper = 0.0
    if lift.motion_limits is not None and lift.motion_limits.upper is not None:
        lift_upper = lift.motion_limits.upper

    rest_column_origin = ctx.part_world_position(column)
    ctx.expect_overlap(
        column,
        crown,
        axes="z",
        elem_a="mast",
        elem_b="sleeve_front",
        min_overlap=0.20,
        name="column stays inserted in the sleeve at rest",
    )
    with ctx.pose({lift: lift_upper}):
        extended_column_origin = ctx.part_world_position(column)
        ctx.expect_overlap(
            column,
            crown,
            axes="z",
            elem_a="mast",
            elem_b="sleeve_front",
            min_overlap=0.09,
            name="column retains insertion at full lift",
        )
    ctx.check(
        "column raises upward",
        rest_column_origin is not None
        and extended_column_origin is not None
        and extended_column_origin[2] > rest_column_origin[2] + 0.20,
        details=f"rest={rest_column_origin}, extended={extended_column_origin}",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(camera, elem="lens_barrel")
    with ctx.pose({pan: math.pi / 2.0}):
        panned_lens_aabb = ctx.part_element_world_aabb(camera, elem="lens_barrel")
    rest_lens_center = aabb_center(rest_lens_aabb) if rest_lens_aabb is not None else None
    panned_lens_center = aabb_center(panned_lens_aabb) if panned_lens_aabb is not None else None
    ctx.check(
        "pan head swings the lens around the vertical axis",
        rest_lens_center is not None
        and panned_lens_center is not None
        and abs(rest_lens_center[0] - panned_lens_center[0]) > 0.10
        and abs(rest_lens_center[1] - panned_lens_center[1]) > 0.10,
        details=f"rest={rest_lens_center}, panned={panned_lens_center}",
    )

    tilt_upper = 0.0
    if tilt.motion_limits is not None and tilt.motion_limits.upper is not None:
        tilt_upper = tilt.motion_limits.upper

    level_lens_aabb = ctx.part_element_world_aabb(camera, elem="lens_barrel")
    with ctx.pose({tilt: tilt_upper}):
        tilted_lens_aabb = ctx.part_element_world_aabb(camera, elem="lens_barrel")
    level_lens_center = aabb_center(level_lens_aabb) if level_lens_aabb is not None else None
    tilted_lens_center = aabb_center(tilted_lens_aabb) if tilted_lens_aabb is not None else None
    ctx.check(
        "tilt joint lifts the camera nose",
        level_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > level_lens_center[2] + 0.06,
        details=f"level={level_lens_center}, tilted={tilted_lens_center}",
    )

    crown_aabb = ctx.part_world_aabb(crown)
    foot_ok = crown_aabb is not None
    foot_details = []
    for index in range(3):
        leg = object_model.get_part(f"leg_{index}")
        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot")
        if crown_aabb is None or foot_aabb is None:
            foot_ok = False
            foot_details.append((index, None))
            continue
        foot_center = aabb_center(foot_aabb)
        radial_reach = math.hypot(foot_center[0], foot_center[1])
        foot_details.append((index, foot_center, radial_reach))
        foot_ok = foot_ok and foot_center[2] < crown_aabb[0][2] - 0.85 and radial_reach > 0.35
    ctx.check(
        "tripod feet form a wide grounded stance",
        foot_ok,
        details=str(foot_details),
    )

    return ctx.report()


object_model = build_object_model()
