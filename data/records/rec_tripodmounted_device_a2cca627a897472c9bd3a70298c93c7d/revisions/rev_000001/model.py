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


LEG_DEPLOY_ANGLE = 1.13
LEG_HINGE_Z = 1.02
LEG_HINGE_RADIUS = 0.13
PAN_AXIS_Z = 1.16
TILT_AXIS_Z = 0.25


def _build_leg_mesh() -> cq.Workplane:
    knuckle = cq.Workplane("XZ").cylinder(0.036, 0.014)
    upper = cq.Workplane("XY").box(0.720, 0.038, 0.042, centered=(False, True, True))
    lower = cq.Workplane("XY").box(0.560, 0.030, 0.034, centered=(False, True, True)).translate((0.520, 0.0, 0.0))
    spike = cq.Workplane("XY").box(0.180, 0.010, 0.010, centered=(False, True, True)).translate((1.060, 0.0, 0.0))
    return knuckle.union(upper).union(lower).union(spike)


def _build_body_shell() -> cq.Workplane:
    main = (
        cq.Workplane("XY")
        .box(0.240, 0.170, 0.180)
        .edges("|Z")
        .fillet(0.014)
        .edges("|Y")
        .fillet(0.010)
        .translate((0.020, 0.0, 0.0))
    )
    rear_pack = cq.Workplane("XY").box(0.060, 0.140, 0.140).translate((-0.130, 0.0, -0.010))
    top_cowl = cq.Workplane("XY").box(0.140, 0.120, 0.050).translate((0.005, 0.0, 0.095))
    return main.union(rear_pack).union(top_cowl)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_device_tripod")

    tripod_dark = model.material("tripod_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    cast_gray = model.material("cast_gray", rgba=(0.30, 0.31, 0.34, 1.0))
    body_green = model.material("body_green", rgba=(0.50, 0.57, 0.29, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))
    grip_gray = model.material("grip_gray", rgba=(0.14, 0.15, 0.16, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.14, 0.24, 0.30, 1.0))

    tripod_crown = model.part("tripod_crown")
    tripod_crown.visual(
        Cylinder(radius=0.055, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.930)),
        material=tripod_dark,
        name="center_column",
    )
    tripod_crown.visual(
        Cylinder(radius=0.078, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.040)),
        material=cast_gray,
        name="crown_hub",
    )
    tripod_crown.visual(
        Cylinder(radius=0.040, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 1.100)),
        material=cast_gray,
        name="pan_bearing",
    )

    leg_specs = (
        ("front_leg", 0.0),
        ("rear_leg_0", 2.35),
        ("rear_leg_1", -2.35),
    )
    for index, (_, angle) in enumerate(leg_specs):
        c = math.cos(angle)
        s = math.sin(angle)
        tripod_crown.visual(
            Box((0.100, 0.040, 0.034)),
            origin=Origin(
                xyz=(0.050 * c, 0.050 * s, LEG_HINGE_Z - 0.010),
                rpy=(0.0, 0.0, angle),
            ),
            material=cast_gray,
            name=f"crown_rib_{index}",
        )
        tripod_crown.visual(
            Box((0.040, 0.060, 0.048)),
            origin=Origin(
                xyz=(0.095 * c, 0.095 * s, LEG_HINGE_Z - 0.020),
                rpy=(0.0, 0.0, angle),
            ),
            material=cast_gray,
            name=f"hinge_block_{index}",
        )

    leg_mesh = mesh_from_cadquery(_build_leg_mesh(), "tripod_leg")
    for part_name, angle in leg_specs:
        leg = model.part(part_name)
        leg.visual(
            leg_mesh,
            origin=Origin(rpy=(0.0, LEG_DEPLOY_ANGLE, 0.0)),
            material=tripod_dark,
            name="leg_shell",
        )
        model.articulation(
            f"{part_name}_hinge",
            ArticulationType.REVOLUTE,
            parent=tripod_crown,
            child=leg,
            origin=Origin(
                xyz=(LEG_HINGE_RADIUS * math.cos(angle), LEG_HINGE_RADIUS * math.sin(angle), LEG_HINGE_Z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=150.0,
                velocity=1.2,
                lower=-0.92,
                upper=0.20,
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.072, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=tripod_dark,
        name="turntable_drum",
    )
    pan_head.visual(
        Cylinder(radius=0.090, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=cast_gray,
        name="pan_plate",
    )
    pan_head.visual(
        Cylinder(radius=0.032, length=0.160),
        origin=Origin(xyz=(-0.028, 0.0, 0.120)),
        material=cast_gray,
        name="tilt_pedestal",
    )
    pan_head.visual(
        Box((0.085, 0.300, 0.050)),
        origin=Origin(xyz=(-0.055, 0.0, 0.180)),
        material=cast_gray,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.040, 0.028, 0.180)),
        origin=Origin(xyz=(-0.040, 0.140, 0.265)),
        material=cast_gray,
        name="yoke_arm_0",
    )
    pan_head.visual(
        Box((0.040, 0.028, 0.180)),
        origin=Origin(xyz=(-0.040, -0.140, 0.265)),
        material=cast_gray,
        name="yoke_arm_1",
    )
    pan_head.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, 0.126, TILT_AXIS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tripod_dark,
        name="tilt_boss_0",
    )
    pan_head.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, -0.126, TILT_AXIS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tripod_dark,
        name="tilt_boss_1",
    )

    device_body = model.part("device_body")
    device_body.visual(
        mesh_from_cadquery(_build_body_shell(), "device_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=body_green,
        name="body_shell",
    )
    device_body.visual(
        Box((0.120, 0.085, 0.040)),
        origin=Origin(xyz=(0.010, 0.0, 0.173)),
        material=matte_black,
        name="top_console",
    )
    device_body.visual(
        Cylinder(radius=0.055, length=0.072),
        origin=Origin(xyz=(0.170, 0.0, 0.077), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="sensor_barrel",
    )
    device_body.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(0.208, 0.0, 0.077), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_hood",
    )
    device_body.visual(
        Box((0.030, 0.020, 0.074)),
        origin=Origin(xyz=(-0.010, 0.093, 0.157)),
        material=matte_black,
        name="handle_mount_0",
    )
    device_body.visual(
        Box((0.030, 0.020, 0.074)),
        origin=Origin(xyz=(-0.010, -0.093, 0.157)),
        material=matte_black,
        name="handle_mount_1",
    )
    device_body.visual(
        Cylinder(radius=0.016, length=0.042),
        origin=Origin(xyz=(0.000, 0.104, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="trunnion_0",
    )
    device_body.visual(
        Cylinder(radius=0.016, length=0.042),
        origin=Origin(xyz=(0.000, -0.104, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="trunnion_1",
    )

    carry_handle = model.part("carry_handle")
    carry_handle.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.000, 0.078, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="pivot_0",
    )
    carry_handle.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.000, -0.078, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="pivot_1",
    )
    carry_handle.visual(
        Box((0.020, 0.016, 0.105)),
        origin=Origin(xyz=(0.010, 0.078, 0.052)),
        material=grip_gray,
        name="side_bar_0",
    )
    carry_handle.visual(
        Box((0.020, 0.016, 0.105)),
        origin=Origin(xyz=(0.010, -0.078, 0.052)),
        material=grip_gray,
        name="side_bar_1",
    )
    carry_handle.visual(
        Cylinder(radius=0.010, length=0.160),
        origin=Origin(xyz=(0.024, 0.0, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="grip_bar",
    )

    model.articulation(
        "pan_axis",
        ArticulationType.CONTINUOUS,
        parent=tripod_crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, PAN_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.2),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=device_body,
        origin=Origin(xyz=(0.000, 0.0, TILT_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=-0.45,
            upper=0.85,
        ),
    )
    model.articulation(
        "handle_axis",
        ArticulationType.REVOLUTE,
        parent=device_body,
        child=carry_handle,
        origin=Origin(xyz=(0.000, 0.0, 0.189)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-1.35,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod_crown = object_model.get_part("tripod_crown")
    front_leg = object_model.get_part("front_leg")
    rear_leg_0 = object_model.get_part("rear_leg_0")
    rear_leg_1 = object_model.get_part("rear_leg_1")
    device_body = object_model.get_part("device_body")
    carry_handle = object_model.get_part("carry_handle")

    front_leg_hinge = object_model.get_articulation("front_leg_hinge")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")
    handle_axis = object_model.get_articulation("handle_axis")

    ctx.allow_overlap(
        tripod_crown,
        front_leg,
        reason="The deployed front leg's captured hinge knuckle is simplified as a mesh that seats slightly into the crown hinge cheek.",
    )
    ctx.allow_overlap(
        tripod_crown,
        rear_leg_0,
        reason="The rear leg hinge is modeled as a simplified captured knuckle seated into the cast crown cheek.",
    )
    ctx.allow_overlap(
        tripod_crown,
        rear_leg_1,
        reason="The rear leg hinge is modeled as a simplified captured knuckle seated into the cast crown cheek.",
    )

    ctx.expect_origin_gap(
        device_body,
        tripod_crown,
        axis="z",
        min_gap=1.20,
        name="device_body_sits_high_above_ground_origin",
    )

    front_leg_aabb = ctx.part_world_aabb(front_leg)
    ctx.check(
        "front_leg_reaches_ground",
        front_leg_aabb is not None and front_leg_aabb[0][2] <= 0.06,
        details=f"front_leg_aabb={front_leg_aabb!r}",
    )

    folded_front_leg_aabb = None
    with ctx.pose({front_leg_hinge: -0.82}):
        folded_front_leg_aabb = ctx.part_world_aabb(front_leg)
    ctx.check(
        "front_leg_folds_upward",
        front_leg_aabb is not None
        and folded_front_leg_aabb is not None
        and folded_front_leg_aabb[0][2] > front_leg_aabb[0][2] + 0.30,
        details=f"rest={front_leg_aabb!r}, folded={folded_front_leg_aabb!r}",
    )

    lens_rest = _aabb_center(ctx.part_element_world_aabb(device_body, elem="lens_hood"))
    lens_pan = None
    with ctx.pose({pan_axis: math.pi / 3.0}):
        lens_pan = _aabb_center(ctx.part_element_world_aabb(device_body, elem="lens_hood"))
    ctx.check(
        "pan_axis_swings_sensor_around_vertical",
        lens_rest is not None
        and lens_pan is not None
        and abs(lens_pan[1] - lens_rest[1]) > 0.12
        and abs(lens_pan[2] - lens_rest[2]) < 0.01,
        details=f"rest={lens_rest!r}, pan={lens_pan!r}",
    )

    lens_tilt = None
    with ctx.pose({tilt_axis: 0.60}):
        lens_tilt = _aabb_center(ctx.part_element_world_aabb(device_body, elem="lens_hood"))
    ctx.check(
        "device_tilts_upward",
        lens_rest is not None and lens_tilt is not None and lens_tilt[2] > lens_rest[2] + 0.08,
        details=f"rest={lens_rest!r}, tilt={lens_tilt!r}",
    )

    grip_rest = _aabb_center(ctx.part_element_world_aabb(carry_handle, elem="grip_bar"))
    grip_folded = None
    with ctx.pose({handle_axis: -1.20}):
        grip_folded = _aabb_center(ctx.part_element_world_aabb(carry_handle, elem="grip_bar"))
    ctx.check(
        "carry_handle_folds_down_from_raised_position",
        grip_rest is not None and grip_folded is not None and grip_folded[2] < grip_rest[2] - 0.045,
        details=f"rest={grip_rest!r}, folded={grip_folded!r}",
    )

    return ctx.report()


object_model = build_object_model()
