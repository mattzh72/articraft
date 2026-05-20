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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_folding_quadcopter")

    model.material("matte_graphite", color=(0.03, 0.035, 0.04, 1.0))
    model.material("dark_plastic", color=(0.01, 0.012, 0.014, 1.0))
    model.material("warm_gray", color=(0.26, 0.27, 0.28, 1.0))
    model.material("rubber_black", color=(0.0, 0.0, 0.0, 1.0))
    model.material("glass", color=(0.02, 0.05, 0.08, 0.65))

    body = model.part("body")
    body.visual(Box((0.22, 0.16, 0.055)), origin=Origin(xyz=(0, 0, 0.04)), material="matte_graphite", name="main_shell")
    body.visual(Box((0.16, 0.105, 0.018)), origin=Origin(xyz=(0.005, 0, 0.076)), material="warm_gray", name="top_battery")
    body.visual(Box((0.19, 0.018, 0.018)), origin=Origin(xyz=(0, 0.083, 0.018)), material="rubber_black", name="skid_0")
    body.visual(Box((0.19, 0.018, 0.018)), origin=Origin(xyz=(0, -0.083, 0.018)), material="rubber_black", name="skid_1")
    body.visual(Box((0.018, 0.014, 0.05)), origin=Origin(xyz=(0.055, 0.083, 0.037)), material="rubber_black", name="skid_post_0")
    body.visual(Box((0.018, 0.014, 0.05)), origin=Origin(xyz=(-0.055, 0.083, 0.037)), material="rubber_black", name="skid_post_1")
    body.visual(Box((0.018, 0.014, 0.05)), origin=Origin(xyz=(0.055, -0.083, 0.037)), material="rubber_black", name="skid_post_2")
    body.visual(Box((0.018, 0.014, 0.05)), origin=Origin(xyz=(-0.055, -0.083, 0.037)), material="rubber_black", name="skid_post_3")

    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.105, 0.018, 3, thickness=0.007, blade_pitch_deg=29, blade_sweep_deg=25,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=12, camber=0.12),
            hub=FanRotorHub(style="spinner", bore_diameter=0.004),
        ),
        "scimitar_rotor",
    )

    hinge_data = [
        ("arm_0", "rotor_0", (0.085, 0.062, 0.048), math.radians(35), -1.75),
        ("arm_1", "rotor_1", (0.085, -0.062, 0.048), math.radians(-35), 1.75),
        ("arm_2", "rotor_2", (-0.085, 0.062, 0.048), math.radians(145), 1.75),
        ("arm_3", "rotor_3", (-0.085, -0.062, 0.048), math.radians(-145), -1.75),
    ]
    for arm_name, rotor_name, xyz, yaw, fold in hinge_data:
        arm = model.part(arm_name)
        arm.visual(Cylinder(0.019, 0.038), origin=Origin(xyz=(0, 0, 0), rpy=(0, 0, 0)), material="warm_gray", name="hinge_barrel")
        arm.visual(Box((0.055, 0.020, 0.018)), origin=Origin(xyz=(0.035, 0, 0)), material="warm_gray", name="hinge_knuckle")
        arm.visual(Box((0.245, 0.026, 0.022)), origin=Origin(xyz=(0.175, 0, 0)), material="matte_graphite", name="carbon_arm")
        arm.visual(Box((0.06, 0.038, 0.014)), origin=Origin(xyz=(0.285, 0, 0.006)), material="warm_gray", name="motor_mount")
        arm.visual(Cylinder(0.038, 0.040), origin=Origin(xyz=(0.32, 0, 0.025)), material="dark_plastic", name="motor_pod")
        arm.visual(Cylinder(0.018, 0.055), origin=Origin(xyz=(0.32, 0, 0.065)), material="warm_gray", name="motor_cap")
        model.articulation(
            f"body_to_{arm_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=xyz, rpy=(0, 0, yaw)),
            axis=(0, 0, 1),
            motion_limits=MotionLimits(effort=4, velocity=2.5, lower=min(0, fold), upper=max(0, fold)),
        )

        rotor = model.part(rotor_name)
        rotor.visual(rotor_mesh, origin=Origin(), material="dark_plastic", name="propeller")
        model.articulation(
            f"{arm_name}_to_{rotor_name}",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=rotor,
            origin=Origin(xyz=(0.32, 0, 0.094)),
            axis=(0, 0, 1),
            motion_limits=MotionLimits(effort=0.4, velocity=80.0),
        )

    camera = model.part("camera")
    camera.visual(Box((0.052, 0.042, 0.034)), origin=Origin(xyz=(0.028, 0, -0.006)), material="dark_plastic", name="camera_body")
    camera.visual(Cylinder(0.017, 0.008), origin=Origin(xyz=(0.057, 0, -0.006), rpy=(0, math.pi / 2, 0)), material="glass", name="lens")
    camera.visual(Sphere(0.019), origin=Origin(xyz=(0.018, 0, 0.014)), material="warm_gray", name="tilt_yoke")
    model.articulation(
        "body_to_camera",
        ArticulationType.REVOLUTE,
        parent=body,
        child=camera,
        origin=Origin(xyz=(0.110, 0, 0.038)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5, lower=-0.55, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    camera = object_model.get_part("camera")
    ctx.expect_contact(body, camera, elem_a="main_shell", elem_b="tilt_yoke", contact_tol=0.025, name="camera is mounted in the nose yoke")

    for i in range(4):
        arm = object_model.get_part(f"arm_{i}")
        rotor = object_model.get_part(f"rotor_{i}")
        spin = object_model.get_articulation(f"arm_{i}_to_rotor_{i}")
        fold = object_model.get_articulation(f"body_to_arm_{i}")
        ctx.allow_overlap(body, arm, elem_a="main_shell", elem_b="hinge_barrel", reason="The arm hinge barrel is intentionally seated through the body corner hinge boss.")
        ctx.allow_overlap(body, arm, elem_a="main_shell", elem_b="hinge_knuckle", reason="The hinge knuckle is intentionally recessed into the body corner socket.")
        ctx.allow_overlap(arm, rotor, elem_a="motor_cap", elem_b="propeller", reason="The propeller hub is intentionally captured on the motor shaft/cap.")
        ctx.expect_overlap(body, arm, axes="xy", elem_a="main_shell", elem_b="hinge_barrel", min_overlap=0.015, name=f"arm_{i} hinge barrel seated in body")
        ctx.expect_overlap(body, arm, axes="xy", elem_a="main_shell", elem_b="hinge_knuckle", min_overlap=0.010, name=f"arm_{i} hinge knuckle retained")
        ctx.expect_overlap(arm, rotor, axes="xy", elem_a="motor_pod", elem_b="propeller", min_overlap=0.02, name=f"rotor_{i} centered over motor")
        ctx.expect_gap(rotor, arm, axis="z", positive_elem="propeller", negative_elem="motor_cap", max_penetration=0.008, name=f"rotor_{i} hub lightly captured on cap")
        with ctx.pose({spin: math.pi / 2}):
            ctx.expect_overlap(arm, rotor, axes="xy", elem_a="motor_pod", elem_b="propeller", min_overlap=0.02, name=f"rotor_{i} spins about motor axis")
        with ctx.pose({fold: fold.motion_limits.upper if fold.motion_limits.upper > 0 else fold.motion_limits.lower}):
            ctx.expect_overlap(body, arm, axes="z", elem_a="main_shell", elem_b="hinge_barrel", min_overlap=0.005, name=f"arm_{i} remains hinged while folded")

    return ctx.report()


object_model = build_object_model()
