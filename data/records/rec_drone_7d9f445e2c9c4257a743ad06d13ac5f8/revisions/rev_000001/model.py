from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _superellipse_section(
    x: float,
    width: float,
    height: float,
    z_center: float,
    *,
    count: int = 32,
    exponent: float = 2.6,
) -> list[tuple[float, float, float]]:
    """Rounded fuselage section loop in the YZ plane."""

    pts: list[tuple[float, float, float]] = []
    for i in range(count):
        t = 2.0 * math.pi * i / count
        c = math.cos(t)
        s = math.sin(t)
        y = 0.5 * width * math.copysign(abs(c) ** (2.0 / exponent), c)
        z = z_center + 0.5 * height * math.copysign(abs(s) ** (2.0 / exponent), s)
        pts.append((x, y, z))
    return pts


def _arm_yaw(name: str) -> float:
    yaws = {
        "front_left": math.radians(50.0),
        "front_right": math.radians(-50.0),
        "rear_left": math.radians(130.0),
        "rear_right": math.radians(-130.0),
    }
    return yaws[name]


def _arm_root(name: str) -> tuple[float, float, float]:
    x = 0.118 if name.startswith("front") else -0.118
    y = 0.095 if name.endswith("left") else -0.095
    return (x, y, 0.040)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinematic_folding_quadcopter")

    body_mat = model.material("warm_gray_shell", rgba=(0.30, 0.31, 0.30, 1.0))
    dark_mat = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    carbon_mat = model.material("carbon_black", rgba=(0.03, 0.035, 0.04, 1.0))
    metal_mat = model.material("dark_anodized_metal", rgba=(0.09, 0.095, 0.10, 1.0))
    prop_mat = model.material("smoked_propeller", rgba=(0.02, 0.025, 0.03, 0.62))
    glass_mat = model.material("blue_black_glass", rgba=(0.02, 0.05, 0.085, 1.0))

    body = model.part("body")

    fuselage = section_loft(
        [
            _superellipse_section(-0.190, 0.070, 0.040, 0.035),
            _superellipse_section(-0.135, 0.120, 0.060, 0.039),
            _superellipse_section(0.040, 0.145, 0.070, 0.041),
            _superellipse_section(0.145, 0.108, 0.058, 0.037),
            _superellipse_section(0.190, 0.045, 0.030, 0.034),
        ]
    )
    body.visual(
        mesh_from_geometry(fuselage, "fuselage_shell"),
        material=body_mat,
        name="fuselage_shell",
    )
    body.visual(
        Box((0.165, 0.092, 0.020)),
        origin=Origin(xyz=(-0.015, 0.0, 0.079)),
        material=dark_mat,
        name="battery_hatch",
    )
    body.visual(
        Sphere(0.030),
        origin=Origin(xyz=(-0.102, 0.0, 0.085), rpy=(0.0, 0.0, 0.0)),
        material=dark_mat,
        name="gps_dome",
    )
    body.visual(
        Box((0.082, 0.064, 0.014)),
        origin=Origin(xyz=(0.122, 0.0, 0.000)),
        material=metal_mat,
        name="gimbal_mount_plate",
    )
    for side, y in (("left", 0.025), ("right", -0.025)):
        body.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.193, y, 0.037), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass_mat,
            name=f"front_sensor_{side}",
        )
    body.visual(
        Box((0.060, 0.017, 0.022)),
        origin=Origin(xyz=(0.118, 0.0685, 0.040)),
        material=metal_mat,
        name="front_left_hinge_socket",
    )
    body.visual(
        Box((0.060, 0.017, 0.022)),
        origin=Origin(xyz=(0.118, -0.0685, 0.040)),
        material=metal_mat,
        name="front_right_hinge_socket",
    )
    body.visual(
        Box((0.060, 0.017, 0.022)),
        origin=Origin(xyz=(-0.118, 0.0685, 0.040)),
        material=metal_mat,
        name="rear_left_hinge_socket",
    )
    body.visual(
        Box((0.060, 0.017, 0.022)),
        origin=Origin(xyz=(-0.118, -0.0685, 0.040)),
        material=metal_mat,
        name="rear_right_hinge_socket",
    )

    arm_length = 0.420
    prop_radius = 0.130
    rotor_geometry = FanRotorGeometry(
        prop_radius,
        0.022,
        2,
        thickness=0.006,
        blade_pitch_deg=17.0,
        blade_sweep_deg=32.0,
        blade_root_chord=0.040,
        blade_tip_chord=0.030,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=8.0, camber=0.10),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.004, rear_collar_radius=0.018),
    )

    arm_limits = {
        "front_left": MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=2.10),
        "front_right": MotionLimits(effort=8.0, velocity=2.0, lower=-2.10, upper=0.0),
        "rear_left": MotionLimits(effort=8.0, velocity=2.0, lower=-2.10, upper=0.0),
        "rear_right": MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=2.10),
    }

    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        arm = model.part(f"{name}_arm")
        arm.visual(
            Cylinder(radius=0.018, length=0.030),
            origin=Origin(),
            material=metal_mat,
            name="hinge_barrel",
        )
        arm.visual(
            Cylinder(radius=0.011, length=arm_length - 0.045),
            origin=Origin(xyz=((arm_length - 0.045) / 2.0 + 0.030, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=carbon_mat,
            name="carbon_boom",
        )
        arm.visual(
            Box((0.070, 0.026, 0.012)),
            origin=Origin(xyz=(0.044, 0.0, -0.004)),
            material=metal_mat,
            name="root_link_plate",
        )
        arm.visual(
            Cylinder(radius=0.030, length=0.052),
            origin=Origin(xyz=(arm_length, 0.0, 0.008)),
            material=metal_mat,
            name="motor_pod",
        )
        arm.visual(
            Cylinder(radius=0.013, length=0.072),
            origin=Origin(xyz=(arm_length - 0.012, 0.0, -0.052)),
            material=carbon_mat,
            name="landing_strut",
        )
        arm.visual(
            Box((0.070, 0.020, 0.010)),
            origin=Origin(xyz=(arm_length - 0.012, 0.0, -0.093)),
            material=dark_mat,
            name="landing_foot",
        )

        model.articulation(
            f"body_to_{name}_arm",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=_arm_root(name), rpy=(0.0, 0.0, _arm_yaw(name))),
            axis=(0.0, 0.0, 1.0),
            motion_limits=arm_limits[name],
        )

        propeller = model.part(f"{name}_propeller")
        propeller.visual(
            mesh_from_geometry(rotor_geometry, f"{name}_propeller_rotor"),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=prop_mat,
            name="rotor",
        )
        model.articulation(
            f"{name}_arm_to_propeller",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=propeller,
            origin=Origin(xyz=(arm_length, 0.0, 0.041118)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=220.0),
        )

    gimbal_pan = model.part("gimbal_pan")
    gimbal_pan.visual(
        Cylinder(radius=0.029, length=0.008),
        origin=Origin(),
        material=metal_mat,
        name="pan_top_plate",
    )
    gimbal_pan.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=metal_mat,
        name="pan_motor",
    )
    gimbal_pan.visual(
        Box((0.062, 0.088, 0.012)),
        origin=Origin(xyz=(0.024, 0.0, -0.031)),
        material=metal_mat,
        name="tilt_crossbar",
    )
    gimbal_pan.visual(
        Box((0.014, 0.010, 0.058)),
        origin=Origin(xyz=(0.035, 0.041, -0.055)),
        material=metal_mat,
        name="tilt_yoke_left",
    )
    gimbal_pan.visual(
        Box((0.014, 0.010, 0.058)),
        origin=Origin(xyz=(0.035, -0.041, -0.055)),
        material=metal_mat,
        name="tilt_yoke_right",
    )
    model.articulation(
        "body_to_gimbal_pan",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=gimbal_pan,
        origin=Origin(xyz=(0.122, 0.0, -0.011)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    gimbal_tilt = model.part("gimbal_tilt")
    gimbal_tilt.visual(
        Cylinder(radius=0.017, length=0.072),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="tilt_motor",
    )
    gimbal_tilt.visual(
        Box((0.064, 0.016, 0.010)),
        origin=Origin(xyz=(0.028, 0.0, -0.008)),
        material=metal_mat,
        name="roll_bridge",
    )
    gimbal_tilt.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.042, 0.0, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="roll_motor",
    )
    model.articulation(
        "gimbal_pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=gimbal_pan,
        child=gimbal_tilt,
        origin=Origin(xyz=(0.035, 0.0, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-0.75, upper=1.15),
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.060, 0.041, 0.035)),
        origin=Origin(xyz=(0.035, 0.0, -0.002)),
        material=dark_mat,
        name="camera_body",
    )
    camera.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(0.076, 0.0, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.093, 0.0, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_mat,
        name="lens_glass",
    )
    model.articulation(
        "gimbal_tilt_to_camera",
        ArticulationType.REVOLUTE,
        parent=gimbal_tilt,
        child=camera,
        origin=Origin(xyz=(0.050, 0.0, -0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = object_model.articulations
    arm_joints = [j for j in joints if j.name.startswith("body_to_") and j.name.endswith("_arm")]
    prop_joints = [j for j in joints if j.name.endswith("_arm_to_propeller")]

    ctx.check("four folding arm hinges", len(arm_joints) == 4)
    ctx.check(
        "arm hinges are revolute",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in arm_joints),
    )
    ctx.check("four continuous propellers", len(prop_joints) == 4)
    ctx.check(
        "propellers are continuous",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in prop_joints),
    )
    ctx.check(
        "stacked three axis gimbal",
        object_model.get_articulation("body_to_gimbal_pan").articulation_type == ArticulationType.CONTINUOUS
        and object_model.get_articulation("gimbal_pan_to_tilt").articulation_type == ArticulationType.REVOLUTE
        and object_model.get_articulation("gimbal_tilt_to_camera").articulation_type == ArticulationType.REVOLUTE,
    )

    body = object_model.get_part("body")
    front_left_arm = object_model.get_part("front_left_arm")
    front_left_propeller = object_model.get_part("front_left_propeller")
    pan = object_model.get_part("gimbal_pan")
    tilt = object_model.get_part("gimbal_tilt")
    camera = object_model.get_part("camera")

    ctx.expect_contact(
        front_left_arm,
        body,
        elem_a="hinge_barrel",
        elem_b="front_left_hinge_socket",
        contact_tol=0.003,
        name="arm hinge barrel is seated at body socket",
    )
    ctx.expect_gap(
        front_left_propeller,
        front_left_arm,
        axis="z",
        positive_elem="rotor",
        negative_elem="motor_pod",
        min_gap=0.000,
        max_gap=0.020,
        name="propeller rotor sits above motor pod",
    )
    ctx.expect_gap(
        body,
        pan,
        axis="z",
        positive_elem="gimbal_mount_plate",
        negative_elem="pan_top_plate",
        max_gap=0.004,
        max_penetration=0.001,
        name="gimbal pan motor is mounted below fuselage",
    )
    ctx.expect_contact(
        tilt,
        pan,
        elem_a="tilt_motor",
        elem_b="tilt_yoke_left",
        contact_tol=0.002,
        name="tilt motor is captured between yoke cheeks",
    )
    ctx.expect_overlap(
        camera,
        tilt,
        axes="yz",
        elem_a="camera_body",
        elem_b="roll_motor",
        min_overlap=0.010,
        name="camera roll axis passes through camera block",
    )

    fold_joint = object_model.get_articulation("body_to_front_left_arm")
    rest = ctx.part_world_position(front_left_propeller)
    with ctx.pose({fold_joint: fold_joint.motion_limits.upper}):
        folded = ctx.part_world_position(front_left_propeller)
    ctx.check(
        "front arm folds inward toward fuselage",
        rest is not None
        and folded is not None
        and math.hypot(folded[0], folded[1]) < math.hypot(rest[0], rest[1]) - 0.10,
        details=f"rest={rest}, folded={folded}",
    )

    return ctx.report()


object_model = build_object_model()
