from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wheel_knee_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.63, 0.66, 0.68, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    rim_mat = model.material("silver_rim", rgba=(0.78, 0.78, 0.74, 1.0))
    cushion_mat = model.material("textured_knee_pad", rgba=(0.025, 0.027, 0.032, 1.0))

    # Root link: the rigid welded scooter frame, rear axle, and the supported knee pad.
    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.026, length=0.995),
        origin=Origin(xyz=(-0.0325, 0.0, 0.36), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="main_spine",
    )
    frame.visual(
        Cylinder(radius=0.035, length=0.17),
        origin=Origin(xyz=(0.50, 0.0, 0.43)),
        material=dark_metal,
        name="head_tube",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.25),
        origin=Origin(xyz=(-0.47, 0.0, 0.255)),
        material=aluminum,
        name="rear_upright",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.62),
        origin=Origin(xyz=(-0.47, 0.0, 0.14), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.30),
        origin=Origin(xyz=(-0.32, 0.0, 0.255), rpy=(0.0, -0.55, 0.0)),
        material=aluminum,
        name="rear_diagonal",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.25),
        origin=Origin(xyz=(-0.06, 0.0, 0.475)),
        material=aluminum,
        name="pad_post",
    )
    frame.visual(
        Box((0.30, 0.17, 0.032)),
        origin=Origin(xyz=(-0.06, 0.0, 0.578)),
        material=dark_metal,
        name="pad_plate",
    )
    knee_pad_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.46, 0.24, 0.055, corner_segments=8),
            0.072,
            cap=True,
            center=True,
        ),
        "rounded_knee_pad",
    )
    frame.visual(
        knee_pad_mesh,
        origin=Origin(xyz=(-0.06, 0.0, 0.628)),
        material=cushion_mat,
        name="knee_pad",
    )

    # Steering unit link: handlebar, steerer, fork crown, legs, and the through-axle.
    steering_fork = model.part("steering_fork")
    steering_fork.visual(
        Cylinder(radius=0.018, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=dark_metal,
        name="steerer_tube",
    )
    steering_fork.visual(
        Box((0.25, 0.15, 0.038)),
        origin=Origin(xyz=(0.095, 0.0, -0.080)),
        material=dark_metal,
        name="fork_crown",
    )
    steering_fork.visual(
        Box((0.030, 0.018, 0.275)),
        origin=Origin(xyz=(0.19, 0.058, -0.2325)),
        material=dark_metal,
        name="fork_leg_0",
    )
    steering_fork.visual(
        Box((0.030, 0.018, 0.275)),
        origin=Origin(xyz=(0.19, -0.058, -0.2325)),
        material=dark_metal,
        name="fork_leg_1",
    )
    steering_fork.visual(
        Cylinder(radius=0.014, length=0.138),
        origin=Origin(xyz=(0.19, 0.0, -0.25), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_axle",
    )
    steering_fork.visual(
        Cylinder(radius=0.015, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.70), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handlebar",
    )
    for suffix, y in (("0", 0.265), ("1", -0.265)):
        steering_fork.visual(
            Cylinder(radius=0.023, length=0.10),
            origin=Origin(xyz=(0.0, y, 0.70), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{suffix}",
        )

    model.articulation(
        "frame_to_steering_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering_fork,
        origin=Origin(xyz=(0.50, 0.0, 0.39)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-0.75, upper=0.75),
    )

    def add_wheel(part_name: str, tire_name: str, rim_name: str, *, radius: float, width: float) -> object:
        wheel_part = model.part(part_name)
        rim = WheelGeometry(
            radius * 0.78,
            width * 0.88,
            rim=WheelRim(
                inner_radius=radius * 0.50,
                flange_height=radius * 0.045,
                flange_thickness=0.004,
                bead_seat_depth=0.003,
            ),
            hub=WheelHub(
                radius=radius * 0.23,
                width=width * 0.62,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=radius * 0.30,
                    hole_diameter=0.0045,
                ),
            ),
            face=WheelFace(dish_depth=0.005, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0035, window_radius=radius * 0.11),
            bore=WheelBore(style="round", diameter=0.026),
        )
        tire = TireGeometry(
            radius,
            width,
            inner_radius=radius * 0.70,
            carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.0045, count=20, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        )
        wheel_part.visual(
            mesh_from_geometry(tire, tire_name),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name=tire_name,
        )
        wheel_part.visual(
            mesh_from_geometry(rim, rim_name),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_mat,
            name="front_rim" if part_name == "front_wheel" else rim_name,
        )
        return wheel_part

    front_wheel = add_wheel(
        "front_wheel",
        "front_tire",
        "front_rim",
        radius=0.14,
        width=0.056,
    )
    model.articulation(
        "steering_fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.19, 0.0, -0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )

    for index, y in enumerate((0.255, -0.255)):
        rear_wheel = add_wheel(
            f"rear_wheel_{index}",
            f"rear_tire_{index}",
            f"rear_rim_{index}",
            radius=0.125,
            width=0.052,
        )
        model.articulation(
            f"frame_to_rear_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=rear_wheel,
            origin=Origin(xyz=(-0.47, y, 0.14)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    fork = object_model.get_part("steering_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")
    yaw = object_model.get_articulation("frame_to_steering_fork")
    front_spin = object_model.get_articulation("steering_fork_to_front_wheel")

    ctx.allow_overlap(
        frame,
        fork,
        elem_a="head_tube",
        elem_b="steerer_tube",
        reason="The steerable column is intentionally nested through the fixed head tube bearing.",
    )
    ctx.expect_within(
        fork,
        frame,
        axes="xy",
        inner_elem="steerer_tube",
        outer_elem="head_tube",
        margin=0.002,
        name="steerer is centered in head tube",
    )
    ctx.expect_overlap(
        fork,
        frame,
        axes="z",
        elem_a="steerer_tube",
        elem_b="head_tube",
        min_overlap=0.12,
        name="steerer passes through bearing length",
    )
    ctx.allow_overlap(
        front_wheel,
        fork,
        elem_a="front_rim",
        elem_b="front_axle",
        reason="The front axle intentionally passes through the wheel hub so the wheel is clipped between the fork legs.",
    )
    ctx.expect_overlap(
        front_wheel,
        fork,
        axes="y",
        elem_a="front_rim",
        elem_b="front_axle",
        min_overlap=0.045,
        name="front axle passes through front wheel",
    )

    for index, rear_wheel in enumerate((rear_wheel_0, rear_wheel_1)):
        ctx.allow_overlap(
            frame,
            rear_wheel,
            elem_a="rear_axle",
            elem_b=f"rear_rim_{index}",
            reason="The fixed rear axle intentionally passes through the wheel hub and bore.",
        )
        ctx.expect_overlap(
            rear_wheel,
            frame,
            axes="y",
            elem_a=f"rear_rim_{index}",
            elem_b="rear_axle",
            min_overlap=0.035,
            name=f"rear axle passes through wheel {index}",
        )

    ctx.expect_gap(
        fork,
        front_wheel,
        axis="y",
        positive_elem="fork_leg_0",
        min_gap=0.010,
        name="front tire clears first fork leg",
    )
    ctx.expect_gap(
        front_wheel,
        fork,
        axis="y",
        negative_elem="fork_leg_1",
        min_gap=0.010,
        name="front tire clears second fork leg",
    )
    ctx.expect_within(
        front_wheel,
        fork,
        axes="y",
        outer_elem="front_axle",
        margin=0.001,
        name="front axle spans across tire width",
    )

    rest_front = ctx.part_world_position(front_wheel)
    with ctx.pose({yaw: 0.65}):
        yawed_front = ctx.part_world_position(front_wheel)
        ctx.expect_origin_distance(
            front_wheel,
            fork,
            axes="xy",
            min_dist=0.18,
            max_dist=0.20,
            name="front wheel remains captured in yawing fork",
        )
    ctx.check(
        "front fork yaws the wheel laterally",
        rest_front is not None
        and yawed_front is not None
        and abs(yawed_front[1] - rest_front[1]) > 0.025,
        details=f"rest={rest_front}, yawed={yawed_front}",
    )

    ctx.expect_origin_gap(
        rear_wheel_0,
        rear_wheel_1,
        axis="y",
        min_gap=0.45,
        max_gap=0.60,
        name="rear wheels are spread on fixed axle",
    )
    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel_0,
        axis="x",
        min_gap=0.85,
        name="single front wheel sits ahead of rear pair",
    )

    wheel_joint_names = (
        "steering_fork_to_front_wheel",
        "frame_to_rear_wheel_0",
        "frame_to_rear_wheel_1",
    )
    ctx.check(
        "all three wheels have continuous spin joints",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in wheel_joint_names
        ),
        details=str(wheel_joint_names),
    )
    ctx.check(
        "wheel spin axes follow their axle supports",
        all(
            tuple(round(v, 6) for v in object_model.get_articulation(name).axis) == (0.0, 1.0, 0.0)
            for name in wheel_joint_names
        ),
        details=str(wheel_joint_names),
    )
    ctx.check(
        "steering yaw is realistically limited",
        yaw.motion_limits is not None
        and yaw.motion_limits.lower is not None
        and yaw.motion_limits.upper is not None
        and yaw.motion_limits.lower < -0.6
        and yaw.motion_limits.upper > 0.6,
        details=str(yaw.motion_limits),
    )

    return ctx.report()


object_model = build_object_model()
