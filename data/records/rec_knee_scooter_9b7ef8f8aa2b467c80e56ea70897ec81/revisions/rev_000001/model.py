from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    ExtrudeGeometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_knee_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.72, 1.0))
    dark_tube = model.material("dark_gray_tube", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    cushion = model.material("matte_black_cushion", rgba=(0.015, 0.014, 0.014, 1.0))
    rim_material = model.material("silver_wheel_rim", rgba=(0.78, 0.78, 0.74, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.061,
            0.046,
            rim=WheelRim(inner_radius=0.041, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.018,
                width=0.036,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.026, hole_diameter=0.0035),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.009),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "small_scooter_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.085,
            0.045,
            inner_radius=0.060,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.005, radius=0.003),
        ),
        "small_scooter_tire",
    )
    pad_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.52, 0.24, 0.045, corner_segments=10), 0.060, center=True),
        "rounded_knee_pad",
    )

    lower_frame = model.part("lower_frame")
    lower_frame.visual(Box((0.72, 0.035, 0.035)), origin=Origin(xyz=(-0.08, 0.18, 0.20)), material=aluminum, name="side_rail_0")
    lower_frame.visual(Box((0.72, 0.035, 0.035)), origin=Origin(xyz=(-0.08, -0.18, 0.20)), material=aluminum, name="side_rail_1")
    lower_frame.visual(Box((0.035, 0.40, 0.035)), origin=Origin(xyz=(0.26, 0.00, 0.20)), material=aluminum, name="front_crossbar")
    lower_frame.visual(Box((0.035, 0.40, 0.035)), origin=Origin(xyz=(-0.40, 0.00, 0.20)), material=aluminum, name="rear_crossbar")
    lower_frame.visual(Box((0.72, 0.040, 0.035)), origin=Origin(xyz=(-0.03, 0.00, 0.20)), material=aluminum, name="center_spine")
    lower_frame.visual(Cylinder(radius=0.012, length=0.474), origin=Origin(xyz=(-0.40, 0.00, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_tube, name="rear_axle")
    lower_frame.visual(Box((0.035, 0.035, 0.115)), origin=Origin(xyz=(-0.40, 0.18, 0.145)), material=aluminum, name="rear_strut_0")
    lower_frame.visual(Box((0.035, 0.035, 0.115)), origin=Origin(xyz=(-0.40, -0.18, 0.145)), material=aluminum, name="rear_strut_1")
    lower_frame.visual(Cylinder(radius=0.034, length=0.17), origin=Origin(xyz=(0.42, 0.0, 0.285)), material=dark_tube, name="head_tube")
    lower_frame.visual(Box((0.16, 0.018, 0.035)), origin=Origin(xyz=(0.35, 0.043, 0.235)), material=aluminum, name="head_gusset_0")
    lower_frame.visual(Box((0.16, 0.018, 0.035)), origin=Origin(xyz=(0.35, -0.043, 0.235)), material=aluminum, name="head_gusset_1")

    steering = model.part("steering_assembly")
    steering.visual(Cylinder(radius=0.018, length=0.88), origin=Origin(xyz=(0.0, 0.0, 0.30)), material=dark_tube, name="steerer")
    steering.visual(Cylinder(radius=0.016, length=0.56), origin=Origin(xyz=(0.0, 0.0, 0.74), rpy=(math.pi / 2.0, 0.0, 0.0)), material=aluminum, name="handlebar")
    steering.visual(Cylinder(radius=0.022, length=0.11), origin=Origin(xyz=(0.0, 0.265, 0.74), rpy=(math.pi / 2.0, 0.0, 0.0)), material=black_rubber, name="grip_0")
    steering.visual(Cylinder(radius=0.022, length=0.11), origin=Origin(xyz=(0.0, -0.265, 0.74), rpy=(math.pi / 2.0, 0.0, 0.0)), material=black_rubber, name="grip_1")
    steering.visual(Box((0.20, 0.055, 0.030)), origin=Origin(xyz=(0.09, 0.0, -0.125)), material=aluminum, name="fork_bridge")
    steering.visual(Box((0.055, 0.29, 0.030)), origin=Origin(xyz=(0.18, 0.0, -0.125)), material=aluminum, name="fork_crown")
    steering.visual(Box((0.028, 0.028, 0.110)), origin=Origin(xyz=(0.18, 0.13, -0.14)), material=aluminum, name="fork_leg_0")
    steering.visual(Box((0.028, 0.028, 0.110)), origin=Origin(xyz=(0.18, -0.13, -0.14)), material=aluminum, name="fork_leg_1")
    steering.visual(Cylinder(radius=0.012, length=0.474), origin=Origin(xyz=(0.18, 0.0, -0.18), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_tube, name="front_axle")

    knee_platform = model.part("knee_platform")
    knee_platform.visual(Cylinder(radius=0.018, length=0.29), origin=Origin(xyz=(0.0, 0.0, 0.145)), material=dark_tube, name="support_post")
    knee_platform.visual(Box((0.42, 0.17, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.3025)), material=aluminum, name="pad_plate")
    knee_platform.visual(pad_mesh, origin=Origin(xyz=(0.0, 0.0, 0.345)), material=cushion, name="pad")

    wheel_positions = {
        "front_wheel_0": (steering, Origin(xyz=(0.18, 0.255, -0.18))),
        "front_wheel_1": (steering, Origin(xyz=(0.18, -0.255, -0.18))),
        "rear_wheel_0": (lower_frame, Origin(xyz=(-0.40, 0.255, 0.10))),
        "rear_wheel_1": (lower_frame, Origin(xyz=(-0.40, -0.255, 0.10))),
    }
    for wheel_name in wheel_positions:
        wheel = model.part(wheel_name)
        wheel_yaw = -math.pi / 2.0 if wheel_name.endswith("_1") else math.pi / 2.0
        wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, 0.0, wheel_yaw)), material=black_rubber, name="tire")
        wheel.visual(wheel_mesh, origin=Origin(rpy=(0.0, 0.0, wheel_yaw)), material=rim_material, name="rim")

    model.articulation(
        "frame_to_steering",
        ArticulationType.REVOLUTE,
        parent=lower_frame,
        child=steering,
        origin=Origin(xyz=(0.42, 0.0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-math.radians(45.0), upper=math.radians(45.0)),
    )
    model.articulation(
        "frame_to_knee_platform",
        ArticulationType.FIXED,
        parent=lower_frame,
        child=knee_platform,
        origin=Origin(xyz=(-0.08, 0.0, 0.2175)),
    )

    for wheel_name, (parent, joint_origin) in wheel_positions.items():
        model.articulation(
            f"{parent.name}_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel_name,
            origin=joint_origin,
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    steering = object_model.get_part("steering_assembly")
    knee_platform = object_model.get_part("knee_platform")
    steer_joint = object_model.get_articulation("frame_to_steering")

    ctx.allow_overlap(
        lower_frame,
        steering,
        elem_a="head_tube",
        elem_b="steerer",
        reason="The steerer shaft is intentionally captured inside the fixed head-tube bearing.",
    )
    ctx.expect_within(
        steering,
        lower_frame,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_tube",
        margin=0.001,
        name="steerer is centered inside head tube",
    )
    ctx.expect_overlap(
        steering,
        lower_frame,
        axes="z",
        elem_a="steerer",
        elem_b="head_tube",
        min_overlap=0.12,
        name="steerer passes through head tube",
    )

    ctx.expect_gap(
        knee_platform,
        lower_frame,
        axis="z",
        positive_elem="support_post",
        negative_elem="center_spine",
        max_gap=0.001,
        max_penetration=0.00001,
        name="knee support post is seated on the center frame",
    )
    ctx.expect_gap(
        knee_platform,
        lower_frame,
        axis="z",
        positive_elem="pad",
        negative_elem="center_spine",
        min_gap=0.08,
        name="padded knee rest sits above the lower frame",
    )

    limits = steer_joint.motion_limits
    ctx.check(
        "steering range is roughly plus-minus forty five degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + math.radians(45.0)) < 0.02
        and abs(limits.upper - math.radians(45.0)) < 0.02,
        details=f"limits={limits}",
    )

    wheel_joints = [
        object_model.get_articulation("steering_assembly_to_front_wheel_0"),
        object_model.get_articulation("steering_assembly_to_front_wheel_1"),
        object_model.get_articulation("lower_frame_to_rear_wheel_0"),
        object_model.get_articulation("lower_frame_to_rear_wheel_1"),
    ]
    ctx.check(
        "four wheels have continuous spin joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in wheel_joints),
        details=", ".join(str(j.articulation_type) for j in wheel_joints),
    )

    front_wheel = object_model.get_part("front_wheel_0")
    rear_wheel = object_model.get_part("rear_wheel_0")
    rest_front = ctx.part_world_position(front_wheel)
    rest_rear = ctx.part_world_position(rear_wheel)
    with ctx.pose({steer_joint: math.radians(45.0)}):
        yawed_front = ctx.part_world_position(front_wheel)
        yawed_rear = ctx.part_world_position(rear_wheel)
    front_shift = 0.0
    rear_shift = 0.0
    if rest_front is not None and yawed_front is not None:
        front_shift = math.hypot(yawed_front[0] - rest_front[0], yawed_front[1] - rest_front[1])
    if rest_rear is not None and yawed_rear is not None:
        rear_shift = math.hypot(yawed_rear[0] - rest_rear[0], yawed_rear[1] - rest_rear[1])
    ctx.check(
        "front fork yaws while rear axle stays fixed",
        front_shift > 0.05 and rear_shift < 0.001,
        details=f"front_shift={front_shift:.4f}, rear_shift={rear_shift:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
