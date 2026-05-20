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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_X = 0.34
BODY_Y = 0.22
BODY_Z = 0.040
HINGE_X = 0.180
HINGE_Y = 0.120
HINGE_Z = BODY_Z
ARM_LENGTH = 0.260
ARM_Z = 0.034
MOTOR_TOP_Z = 0.067

ARM_SPECS = {
    "front_right": {
        "corner": (HINGE_X, HINGE_Y),
        "yaw": math.radians(45.0),
        "fold": -math.radians(135.0),
    },
    "front_left": {
        "corner": (-HINGE_X, HINGE_Y),
        "yaw": math.radians(135.0),
        "fold": math.radians(135.0),
    },
    "rear_left": {
        "corner": (-HINGE_X, -HINGE_Y),
        "yaw": math.radians(-135.0),
        "fold": -math.radians(135.0),
    },
    "rear_right": {
        "corner": (HINGE_X, -HINGE_Y),
        "yaw": math.radians(-45.0),
        "fold": math.radians(135.0),
    },
}


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_quadrotor")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    carbon = model.material("carbon_fiber_dark", rgba=(0.035, 0.038, 0.043, 1.0))
    graphite = model.material("graphite_body", rgba=(0.12, 0.13, 0.14, 1.0))
    warm_gray = model.material("warm_gray_panel", rgba=(0.34, 0.35, 0.34, 1.0))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.07, 0.07, 0.075, 1.0))
    camera_glass = model.material("camera_glass", rgba=(0.02, 0.025, 0.030, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_X, BODY_Y, BODY_Z)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z / 2.0)),
        material=graphite,
        name="main_shell",
    )
    body.visual(
        Box((0.230, 0.125, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z + 0.004)),
        material=warm_gray,
        name="top_battery_panel",
    )
    body.visual(
        Box((0.090, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.058, BODY_Z + 0.005)),
        material=matte_black,
        name="front_status_bar",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, BODY_Y / 2.0 + 0.006, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=camera_glass,
        name="front_camera",
    )

    for tag, spec in ARM_SPECS.items():
        x, y = spec["corner"]
        body.visual(
            Cylinder(radius=0.030, length=BODY_Z),
            origin=Origin(xyz=(x, y, BODY_Z / 2.0)),
            material=hinge_metal,
            name=f"{tag}_hinge_pad",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(x, y, 0.040)),
            material=hinge_metal,
            name=f"{tag}_hinge_boss",
        )

    prop_mesh = FanRotorGeometry(
        outer_radius=0.074,
        hub_radius=0.017,
        blade_count=2,
        thickness=0.009,
        blade_pitch_deg=22.0,
        blade_sweep_deg=12.0,
        blade=FanRotorBlade(shape="narrow", tip_pitch_deg=10.0, camber=0.08),
        hub=FanRotorHub(style="capped", rear_collar_height=0.004, rear_collar_radius=0.014),
        center=False,
    )

    for tag, spec in ARM_SPECS.items():
        arm = model.part(f"{tag}_arm")
        arm.visual(
            Cylinder(radius=0.020, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, 0.028)),
            material=hinge_metal,
            name="hinge_knuckle",
        )
        arm.visual(
            Box((0.228, 0.026, 0.018)),
            origin=Origin(xyz=(0.129, 0.0, ARM_Z)),
            material=carbon,
            name="folding_beam",
        )
        arm.visual(
            Box((0.184, 0.010, 0.006)),
            origin=Origin(xyz=(0.134, 0.0, ARM_Z + 0.012)),
            material=warm_gray,
            name="top_reinforcement",
        )
        arm.visual(
            Cylinder(radius=0.033, length=0.050),
            origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.042)),
            material=matte_black,
            name="motor_pod",
        )
        arm.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(ARM_LENGTH, 0.0, MOTOR_TOP_Z - 0.006)),
            material=hinge_metal,
            name="propeller_shaft",
        )

        lower = min(0.0, spec["fold"])
        upper = max(0.0, spec["fold"])
        model.articulation(
            f"body_to_{tag}_arm",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=(spec["corner"][0], spec["corner"][1], HINGE_Z), rpy=(0.0, 0.0, spec["yaw"])),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=lower, upper=upper),
        )

        prop = model.part(f"{tag}_propeller")
        prop.visual(
            mesh_from_geometry(prop_mesh, f"{tag}_two_blade_propeller"),
            origin=Origin(),
            material=matte_black,
            name="two_blade_rotor",
        )
        model.articulation(
            f"{tag}_arm_to_propeller",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=prop,
            origin=Origin(xyz=(ARM_LENGTH, 0.0, MOTOR_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=80.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    ctx.check(
        "four folding arms and four propellers",
        all(object_model.get_part(f"{tag}_arm") is not None for tag in ARM_SPECS)
        and all(object_model.get_part(f"{tag}_propeller") is not None for tag in ARM_SPECS),
    )

    for tag, spec in ARM_SPECS.items():
        arm = object_model.get_part(f"{tag}_arm")
        prop = object_model.get_part(f"{tag}_propeller")
        hinge = object_model.get_articulation(f"body_to_{tag}_arm")
        rotor_joint = object_model.get_articulation(f"{tag}_arm_to_propeller")

        ctx.allow_overlap(
            body,
            arm,
            elem_a=f"{tag}_hinge_boss",
            elem_b="hinge_knuckle",
            reason="The folding-arm hinge knuckle is intentionally captured around the corner pivot boss.",
        )
        ctx.expect_overlap(
            body,
            arm,
            axes="xy",
            elem_a=f"{tag}_hinge_boss",
            elem_b="hinge_knuckle",
            min_overlap=0.020,
            name=f"{tag} captured hinge footprint",
        )
        ctx.expect_overlap(
            body,
            arm,
            axes="z",
            elem_a=f"{tag}_hinge_boss",
            elem_b="hinge_knuckle",
            min_overlap=0.020,
            name=f"{tag} captured hinge height",
        )

        ctx.check(
            f"{tag} arm hinge swings inward",
            hinge.motion_limits is not None
            and hinge.motion_limits.lower is not None
            and hinge.motion_limits.upper is not None
            and abs((hinge.motion_limits.upper - hinge.motion_limits.lower) - math.radians(135.0)) < 1e-6,
        )
        ctx.check(
            f"{tag} propeller spins about motor",
            rotor_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(rotor_joint.axis) == (0.0, 0.0, 1.0),
        )

        rest_pos = ctx.part_world_position(prop)
        with ctx.pose({hinge: spec["fold"]}):
            folded_pos = ctx.part_world_position(prop)
        x0, y0 = spec["corner"]
        ctx.check(
            f"{tag} folded rotor moves inward",
            rest_pos is not None
            and folded_pos is not None
            and abs(rest_pos[0]) > abs(x0) + 0.10
            and abs(folded_pos[0]) < abs(rest_pos[0]) - 0.10,
            details=f"rest={rest_pos}, folded={folded_pos}",
        )

    return ctx.report()


object_model = build_object_model()
