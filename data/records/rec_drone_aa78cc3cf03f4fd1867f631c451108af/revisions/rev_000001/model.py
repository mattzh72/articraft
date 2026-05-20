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


BODY_LENGTH = 0.190
BODY_WIDTH = 0.090
BODY_HEIGHT = 0.048
HINGE_X = 0.074
HINGE_Y = 0.064
HINGE_Z = 0.032
ARM_MOTOR_X = 0.180
PROP_JOINT_Z = 0.033


def _add_body_hinge_lugs(body, *, x: float, sign_y: int, material: Material) -> None:
    """Two clevis plates on the airframe side that hold one folding-arm collar."""
    lug_y = sign_y * (BODY_WIDTH / 2.0 + 0.015)
    for name, z in (("lower", 0.013), ("upper", 0.051)):
        body.visual(
            Box((0.036, 0.032, 0.018)),
            origin=Origin(xyz=(x, lug_y, z)),
            material=material,
            name=f"{name}_hinge_lug_{'p' if sign_y > 0 else 'n'}_{'f' if x > 0 else 'r'}",
        )


def _add_arm_geometry(arm, *, material: Material, accent: Material, z_offset: float) -> None:
    arm.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(),
        material=material,
        name="hinge_collar",
    )
    arm.visual(
        Box((0.166, 0.014, 0.008)),
        origin=Origin(xyz=(0.088, 0.0, z_offset)),
        material=material,
        name="arm_beam",
    )
    arm.visual(
        Box((0.120, 0.006, 0.002)),
        origin=Origin(xyz=(0.096, 0.0, z_offset + 0.005)),
        material=accent,
        name="top_inlay",
    )
    arm.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(ARM_MOTOR_X, 0.0, z_offset + 0.006)),
        material=material,
        name="motor_pod",
    )
    arm.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(ARM_MOTOR_X, 0.0, z_offset + 0.019)),
        material=accent,
        name="motor_cap",
    )


def _add_propeller_geometry(propeller, *, rotor_mesh, material: Material, shaft_material: Material) -> None:
    propeller.visual(
        Cylinder(radius=0.004, length=0.037),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=shaft_material,
        name="drive_shaft",
    )
    propeller.visual(
        rotor_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=material,
        name="two_blade_rotor",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_travel_quadcopter")

    matte_graphite = model.material("matte_graphite", rgba=(0.055, 0.060, 0.065, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.34, 0.36, 0.36, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    smoked_prop = model.material("smoked_propeller", rgba=(0.030, 0.035, 0.040, 0.62))
    glass = model.material("camera_glass", rgba=(0.02, 0.045, 0.075, 1.0))

    propeller_mesh = mesh_from_geometry(
        FanRotorGeometry(
            outer_radius=0.070,
            hub_radius=0.014,
            blade_count=2,
            thickness=0.006,
            blade_pitch_deg=22.0,
            blade_sweep_deg=32.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=10.0, camber=0.10),
            hub=FanRotorHub(style="spinner"),
        ),
        "folding_quadcopter_two_blade_propeller",
    )

    body = model.part("body")
    body.visual(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material=matte_graphite,
        name="rectangular_fuselage",
    )
    body.visual(
        Box((0.122, 0.064, 0.008)),
        origin=Origin(xyz=(-0.005, 0.0, BODY_HEIGHT + 0.004)),
        material=warm_gray,
        name="top_battery_panel",
    )
    body.visual(
        Box((0.038, 0.032, 0.020)),
        origin=Origin(xyz=(BODY_LENGTH / 2.0 + 0.012, 0.0, 0.020)),
        material=black_rubber,
        name="front_gimbal_block",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(BODY_LENGTH / 2.0 + 0.031, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_camera_lens",
    )

    for hinge_x in (HINGE_X, -HINGE_X):
        for sign_y in (1, -1):
            _add_body_hinge_lugs(body, x=hinge_x, sign_y=sign_y, material=warm_gray)

    arm_specs = (
        ("front_left_arm", "body_to_front_left_arm", HINGE_X, HINGE_Y, 0.60, 0.0, math.pi - 0.60, -0.005),
        ("front_right_arm", "body_to_front_right_arm", HINGE_X, -HINGE_Y, -0.60, -math.pi + 0.60, 0.0, -0.005),
        ("rear_left_arm", "body_to_rear_left_arm", -HINGE_X, HINGE_Y, math.pi - 0.60, -math.pi + 0.60, 0.0, 0.005),
        ("rear_right_arm", "body_to_rear_right_arm", -HINGE_X, -HINGE_Y, -math.pi + 0.60, 0.0, math.pi - 0.60, 0.005),
    )

    for arm_name, joint_name, hinge_x, hinge_y, deployed_yaw, lower, upper, z_offset in arm_specs:
        arm = model.part(arm_name)
        _add_arm_geometry(arm, material=matte_graphite, accent=warm_gray, z_offset=z_offset)
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=(hinge_x, hinge_y, HINGE_Z), rpy=(0.0, 0.0, deployed_yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=lower, upper=upper),
        )

        prop_name = arm_name.replace("_arm", "_propeller")
        propeller = model.part(prop_name)
        _add_propeller_geometry(
            propeller,
            rotor_mesh=propeller_mesh,
            material=smoked_prop,
            shaft_material=black_rubber,
        )
        model.articulation(
            f"{prop_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=propeller,
            origin=Origin(xyz=(ARM_MOTOR_X, 0.0, z_offset + PROP_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.20, velocity=80.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    arm_names = (
        "front_left_arm",
        "front_right_arm",
        "rear_left_arm",
        "rear_right_arm",
    )
    propeller_names = (
        "front_left_propeller",
        "front_right_propeller",
        "rear_left_propeller",
        "rear_right_propeller",
    )
    arm_joints = (
        "body_to_front_left_arm",
        "body_to_front_right_arm",
        "body_to_rear_left_arm",
        "body_to_rear_right_arm",
    )
    propeller_joints = (
        "front_left_propeller_spin",
        "front_right_propeller_spin",
        "rear_left_propeller_spin",
        "rear_right_propeller_spin",
    )

    ctx.check("four folding arms", all(object_model.get_part(name) is not None for name in arm_names))
    ctx.check("four spinning propellers", all(object_model.get_part(name) is not None for name in propeller_names))

    for joint_name in arm_joints:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is vertical revolute hinge",
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 0.0, 1.0),
        )

    for joint_name in propeller_joints:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous vertical rotor",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
        )

    body = object_model.get_part("body")
    for arm_name in arm_names:
        arm = object_model.get_part(arm_name)
        propeller_name = arm_name.replace("_arm", "_propeller")
        propeller = object_model.get_part(propeller_name)
        ctx.expect_contact(
            arm,
            body,
            elem_a="hinge_collar",
            elem_b="lower_hinge_lug_p_f" if arm_name == "front_left_arm" else None,
            contact_tol=0.003,
            name=f"{arm_name} hinge is carried by body clevis",
        )
        ctx.expect_contact(
            propeller,
            arm,
            elem_a="drive_shaft",
            elem_b="motor_cap",
            contact_tol=0.003,
            name=f"{propeller_name} shaft sits on motor",
        )

    folded_pose = {
        "body_to_front_left_arm": math.pi - 0.60,
        "body_to_front_right_arm": -math.pi + 0.60,
        "body_to_rear_left_arm": -math.pi + 0.60,
        "body_to_rear_right_arm": math.pi - 0.60,
    }
    with ctx.pose(folded_pose):
        ctx.expect_overlap(
            "front_left_arm",
            body,
            axes="x",
            elem_a="arm_beam",
            elem_b="rectangular_fuselage",
            min_overlap=0.080,
            name="front left arm folds back alongside body length",
        )
        ctx.expect_overlap(
            "rear_right_arm",
            body,
            axes="x",
            elem_a="arm_beam",
            elem_b="rectangular_fuselage",
            min_overlap=0.080,
            name="rear right arm folds forward alongside body length",
        )
        ctx.expect_overlap(
            "front_right_arm",
            body,
            axes="x",
            elem_a="arm_beam",
            elem_b="rectangular_fuselage",
            min_overlap=0.080,
            name="front right arm folds back alongside body length",
        )
        ctx.expect_overlap(
            "rear_left_arm",
            body,
            axes="x",
            elem_a="arm_beam",
            elem_b="rectangular_fuselage",
            min_overlap=0.080,
            name="rear left arm folds forward alongside body length",
        )
        ctx.expect_gap(
            "front_left_arm",
            body,
            axis="y",
            positive_elem="arm_beam",
            negative_elem="rectangular_fuselage",
            min_gap=0.004,
            name="folded positive-side arm clears body side",
        )
        ctx.expect_gap(
            body,
            "front_right_arm",
            axis="y",
            positive_elem="rectangular_fuselage",
            negative_elem="arm_beam",
            min_gap=0.004,
            name="folded negative-side arm clears body side",
        )
        ctx.expect_gap(
            "rear_left_arm",
            "front_left_arm",
            axis="z",
            positive_elem="arm_beam",
            negative_elem="arm_beam",
            min_gap=0.001,
            name="folded left arms stack without beam collision",
        )
        ctx.expect_gap(
            "rear_right_arm",
            "front_right_arm",
            axis="z",
            positive_elem="arm_beam",
            negative_elem="arm_beam",
            min_gap=0.001,
            name="folded right arms stack without beam collision",
        )
        ctx.expect_gap(
            "rear_left_propeller",
            "front_left_propeller",
            axis="x",
            min_gap=0.050,
            name="folded left propellers do not overlap",
        )
        ctx.expect_gap(
            "rear_right_propeller",
            "front_right_propeller",
            axis="x",
            min_gap=0.050,
            name="folded right propellers do not overlap",
        )

    return ctx.report()


object_model = build_object_model()
