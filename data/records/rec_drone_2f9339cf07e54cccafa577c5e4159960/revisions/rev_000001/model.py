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
    superellipse_side_loft,
)


ARM_SPECS = (
    ("front_left", (0.088, 0.074, 0.085), math.radians(45.0), (-1.70, 0.18)),
    ("front_right", (-0.088, 0.074, 0.085), math.radians(135.0), (-0.18, 1.70)),
    ("rear_left", (0.088, -0.074, 0.085), math.radians(-45.0), (-0.18, 1.70)),
    ("rear_right", (-0.088, -0.074, 0.085), math.radians(-135.0), (-1.70, 0.18)),
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_folding_quadcopter")

    carbon = model.material("carbon_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_plastic = model.material("graphite_shell", rgba=(0.075, 0.080, 0.088, 1.0))
    warm_gray = model.material("warm_gray_panel", rgba=(0.43, 0.45, 0.46, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    metal = model.material("brushed_motor_metal", rgba=(0.58, 0.60, 0.61, 1.0))
    glass = model.material("camera_glass", rgba=(0.02, 0.04, 0.07, 0.82))
    led_green = model.material("green_status_led", rgba=(0.0, 0.95, 0.38, 1.0))
    led_red = model.material("red_status_led", rgba=(1.0, 0.08, 0.04, 1.0))
    rotor_sweep = model.material("translucent_rotor_sweep", rgba=(0.20, 0.22, 0.25, 0.18))

    body = model.part("body")

    shell_sections = (
        (-0.135, 0.055, 0.090, 0.050),
        (-0.108, 0.045, 0.108, 0.115),
        (-0.035, 0.040, 0.120, 0.150),
        (0.048, 0.043, 0.116, 0.136),
        (0.112, 0.051, 0.101, 0.086),
        (0.138, 0.061, 0.088, 0.034),
    )
    body_shell = superellipse_side_loft(shell_sections, exponents=2.9, segments=72)
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        material=dark_plastic,
        name="rounded_shell",
    )

    body.visual(
        Box((0.106, 0.124, 0.004)),
        origin=Origin(xyz=(0.0, -0.014, 0.122)),
        material=warm_gray,
        name="battery_lid",
    )
    body.visual(
        Box((0.012, 0.100, 0.003)),
        origin=Origin(xyz=(0.0, -0.018, 0.1255)),
        material=carbon,
        name="lid_center_seam",
    )
    body.visual(
        Box((0.088, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.046, 0.1255)),
        material=carbon,
        name="rear_lid_seam",
    )

    # Landing gear: two rubber skid rails under the body with four compression struts.
    for x in (-0.055, 0.055):
        body.visual(
            Cylinder(radius=0.006, length=0.230),
            origin=Origin(xyz=(x, -0.004, 0.017), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"skid_rail_{'left' if x > 0.0 else 'right'}",
        )
        for y in (-0.063, 0.063):
            body.visual(
                Cylinder(radius=0.0045, length=0.044),
                origin=Origin(xyz=(x, y, 0.038), rpy=(0.0, 0.0, 0.0)),
                material=carbon,
                name=f"skid_strut_{'left' if x > 0.0 else 'right'}_{'front' if y > 0.0 else 'rear'}",
            )

    # Body-side hinge knuckles and reinforced pads.  Arms carry the center knuckle.
    for name, hinge_xyz, yaw, _limits in ARM_SPECS:
        x, y, z = hinge_xyz
        outward_x = math.cos(yaw)
        outward_y = math.sin(yaw)
        body.visual(
            Box((0.030, 0.046, 0.060)),
            origin=Origin(xyz=(x - outward_x * 0.032, y - outward_y * 0.032, z), rpy=(0.0, 0.0, yaw)),
            material=dark_plastic,
            name=f"{name}_hinge_pad",
        )
        for leaf_z, leaf_name in ((z - 0.020, "lower_leaf"), (z + 0.020, "upper_leaf")):
            body.visual(
                Box((0.040, 0.020, 0.010)),
                origin=Origin(
                    xyz=(x - outward_x * 0.026, y - outward_y * 0.026, leaf_z),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=dark_plastic,
                name=f"{name}_{leaf_name}",
            )
        body.visual(
            Cylinder(radius=0.017, length=0.016),
            origin=Origin(xyz=(x, y, z - 0.020)),
            material=metal,
            name=f"{name}_lower_knuckle",
        )
        body.visual(
            Cylinder(radius=0.017, length=0.016),
            origin=Origin(xyz=(x, y, z + 0.020)),
            material=metal,
            name=f"{name}_upper_knuckle",
        )
        body.visual(
            Cylinder(radius=0.0045, length=0.060),
            origin=Origin(xyz=(x, y, z)),
            material=metal,
            name=f"{name}_hinge_pin",
        )

    # Nose camera yoke, fixed to the body shell.
    body.visual(
        Box((0.076, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.137, 0.073)),
        material=dark_plastic,
        name="camera_yoke_bridge",
    )
    for x in (-0.034, 0.034):
        body.visual(
            Box((0.008, 0.030, 0.034)),
            origin=Origin(xyz=(x, 0.153, 0.071)),
            material=dark_plastic,
            name=f"camera_yoke_{'left' if x > 0.0 else 'right'}",
        )

    # Front status lights.
    body.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.034, 0.117, 0.091)),
        material=led_green,
        name="front_green_led",
    )
    body.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(-0.034, 0.117, 0.091)),
        material=led_red,
        name="front_red_led",
    )

    rotor_geometry = FanRotorGeometry(
        0.076,
        0.014,
        2,
        thickness=0.008,
        blade_pitch_deg=24.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=10.0, camber=0.10, tip_clearance=0.002),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.003, rear_collar_radius=0.012),
    )
    rotor_mesh = mesh_from_geometry(rotor_geometry, "two_blade_propeller")

    for prefix, hinge_xyz, yaw, (lower, upper) in ARM_SPECS:
        arm = model.part(f"{prefix}_arm")
        arm.visual(
            Cylinder(radius=0.0145, length=0.012),
            origin=Origin(),
            material=metal,
            name="center_knuckle",
        )
        arm.visual(
            Box((0.186, 0.024, 0.016)),
            origin=Origin(xyz=(0.101, 0.0, 0.0)),
            material=carbon,
            name="carbon_beam",
        )
        arm.visual(
            Box((0.060, 0.037, 0.020)),
            origin=Origin(xyz=(0.043, 0.0, 0.0)),
            material=dark_plastic,
            name="root_fairing",
        )
        arm.visual(
            Cylinder(radius=0.031, length=0.036),
            origin=Origin(xyz=(0.215, 0.0, 0.004)),
            material=carbon,
            name="motor_pod",
        )
        arm.visual(
            Cylinder(radius=0.024, length=0.010),
            origin=Origin(xyz=(0.215, 0.0, 0.027)),
            material=metal,
            name="motor_cap",
        )
        arm.visual(
            Sphere(radius=0.005),
            origin=Origin(xyz=(0.227, 0.022, 0.004)),
            material=led_green if "left" in prefix else led_red,
            name="arm_tip_led",
        )

        model.articulation(
            f"body_to_{prefix}_arm",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.4, velocity=2.2, lower=lower, upper=upper),
        )

        rotor = model.part(f"{prefix}_rotor")
        rotor.visual(
            rotor_mesh,
            origin=Origin(),
            material=carbon,
            name="propeller",
        )
        rotor.visual(
            Cylinder(radius=0.077, length=0.0012),
            origin=Origin(),
            material=rotor_sweep,
            name="motion_disk",
        )
        rotor.visual(
            Cylinder(radius=0.004, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material=metal,
            name="drive_shaft",
        )

        model.articulation(
            f"{prefix}_arm_to_rotor",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=rotor,
            origin=Origin(xyz=(0.215, 0.0, 0.038)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.08, velocity=220.0),
        )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="tilt_barrel",
    )
    camera.visual(
        Box((0.042, 0.034, 0.030)),
        origin=Origin(xyz=(0.0, 0.020, -0.012)),
        material=dark_plastic,
        name="camera_block",
    )
    camera.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.041, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="lens_glass",
    )

    model.articulation(
        "body_to_camera",
        ArticulationType.REVOLUTE,
        parent=body,
        child=camera,
        origin=Origin(xyz=(0.0, 0.153, 0.073)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=1.6, lower=-0.65, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    arm_joint_names = [f"body_to_{prefix}_arm" for prefix, *_ in ARM_SPECS]
    rotor_joint_names = [f"{prefix}_arm_to_rotor" for prefix, *_ in ARM_SPECS]
    for joint_name in arm_joint_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a limited folding hinge",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper - joint.motion_limits.lower > 1.7,
            details=f"type={joint.articulation_type}, limits={joint.motion_limits}",
        )

    for joint_name in rotor_joint_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} spins continuously",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    camera_joint = object_model.get_articulation("body_to_camera")
    ctx.check(
        "camera has horizontal tilt axis",
        camera_joint.articulation_type == ArticulationType.REVOLUTE and tuple(camera_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={camera_joint.articulation_type}, axis={camera_joint.axis}",
    )

    body = object_model.get_part("body")
    camera = object_model.get_part("camera")
    for lug_name in ("camera_yoke_left", "camera_yoke_right"):
        ctx.allow_overlap(
            body,
            camera,
            elem_a=lug_name,
            elem_b="tilt_barrel",
            reason="The camera tilt barrel is intentionally captured inside the nose yoke lugs.",
        )
        ctx.expect_overlap(
            body,
            camera,
            axes="xyz",
            elem_a=lug_name,
            elem_b="tilt_barrel",
            min_overlap=0.003,
            name=f"{lug_name} captures the camera tilt barrel",
        )

    for prefix, *_ in ARM_SPECS:
        arm = object_model.get_part(f"{prefix}_arm")
        rotor = object_model.get_part(f"{prefix}_rotor")
        ctx.allow_overlap(
            body,
            arm,
            elem_a=f"{prefix}_hinge_pin",
            elem_b="center_knuckle",
            reason="The body-mounted hinge pin passes through the arm's center knuckle.",
        )
        ctx.expect_within(
            body,
            arm,
            axes="xy",
            inner_elem=f"{prefix}_hinge_pin",
            outer_elem="center_knuckle",
            margin=0.001,
            name=f"{prefix} hinge pin is concentric with the arm knuckle",
        )
        ctx.expect_overlap(
            body,
            arm,
            axes="z",
            elem_a=f"{prefix}_hinge_pin",
            elem_b="center_knuckle",
            min_overlap=0.010,
            name=f"{prefix} hinge pin passes through the center knuckle",
        )
        ctx.allow_overlap(
            rotor,
            arm,
            elem_a="drive_shaft",
            elem_b="motor_cap",
            reason="The rotor drive shaft is intentionally seated through the motor cap.",
        )
        ctx.expect_within(
            rotor,
            arm,
            axes="xy",
            inner_elem="drive_shaft",
            outer_elem="motor_cap",
            margin=0.001,
            name=f"{prefix} rotor shaft is centered in the motor cap",
        )
        ctx.expect_overlap(
            rotor,
            arm,
            axes="z",
            elem_a="drive_shaft",
            elem_b="motor_cap",
            min_overlap=0.003,
            name=f"{prefix} rotor shaft remains seated in the motor cap",
        )
        ctx.expect_overlap(
            rotor,
            arm,
            axes="xy",
            elem_a="propeller",
            elem_b="motor_pod",
            min_overlap=0.020,
            name=f"{prefix} rotor is centered above its motor pod",
        )
        ctx.expect_gap(
            rotor,
            arm,
            axis="z",
            positive_elem="propeller",
            negative_elem="motor_cap",
            min_gap=0.0,
            max_gap=0.010,
            name=f"{prefix} rotor clears the fixed motor cap",
        )

    return ctx.report()


object_model = build_object_model()
