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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_delivery_drone")

    carbon = model.material("satin_black_carbon", rgba=(0.03, 0.035, 0.04, 1.0))
    graphite = model.material("dark_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    pale_panel = model.material("warm_white_composite", rgba=(0.82, 0.84, 0.80, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    metal = model.material("brushed_aluminum", rgba=(0.55, 0.57, 0.56, 1.0))
    lens = model.material("smoked_lens", rgba=(0.02, 0.04, 0.07, 0.78))
    red = model.material("red_nav_lens", rgba=(1.0, 0.05, 0.02, 1.0))
    green = model.material("green_nav_lens", rgba=(0.05, 0.9, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.62, 0.40, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pale_panel,
        name="rectangular_fuselage",
    )
    body.visual(
        Box((0.46, 0.28, 0.022)),
        origin=Origin(xyz=(0.02, 0.0, 0.091)),
        material=graphite,
        name="top_battery_hatch",
    )
    body.visual(
        Box((0.32, 0.22, 0.045)),
        origin=Origin(xyz=(-0.03, 0.0, -0.102)),
        material=graphite,
        name="belly_payload_plate",
    )
    body.visual(
        Cylinder(radius=0.038, length=0.035),
        origin=Origin(xyz=(0.327, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens,
        name="front_camera",
    )
    body.visual(
        Box((0.045, 0.018, 0.016)),
        origin=Origin(xyz=(0.305, 0.145, 0.071)),
        material=green,
        name="nav_lamp_0",
    )
    body.visual(
        Box((0.045, 0.018, 0.016)),
        origin=Origin(xyz=(0.305, -0.145, 0.071)),
        material=red,
        name="nav_lamp_1",
    )

    # Corner pads and underside clevis ears make the folding pivots read as real
    # mounted hinge hardware without occupying the same space as the moving links.
    for index, (sx, sy) in enumerate(((1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0))):
        body.visual(
            Box((0.048, 0.050, 0.038)),
            origin=Origin(xyz=(sx * 0.310, sy * 0.225, 0.042)),
            material=graphite,
            name=f"arm_hinge_pad_{index}",
        )

    body.visual(
        Box((0.070, 0.024, 0.038)),
        origin=Origin(xyz=(-0.230, 0.178, -0.095)),
        material=metal,
        name="skid_clevis_0",
    )
    body.visual(
        Box((0.070, 0.024, 0.038)),
        origin=Origin(xyz=(-0.230, -0.178, -0.095)),
        material=metal,
        name="skid_clevis_1",
    )

    arm_specs = (
        ("arm_0", (0.360, 0.240, 0.045), math.radians(43.0), (0.0, 0.0, 1.0)),
        ("arm_1", (0.360, -0.240, 0.045), math.radians(-43.0), (0.0, 0.0, -1.0)),
        ("arm_2", (-0.360, 0.240, 0.045), math.radians(137.0), (0.0, 0.0, -1.0)),
        ("arm_3", (-0.360, -0.240, 0.045), math.radians(-137.0), (0.0, 0.0, 1.0)),
    )

    propeller_mesh = mesh_from_geometry(
        FanRotorGeometry(
            outer_radius=0.145,
            hub_radius=0.026,
            blade_count=2,
            thickness=0.012,
            blade_pitch_deg=23.0,
            blade_sweep_deg=34.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=10.0, camber=0.10),
            hub=FanRotorHub(style="spinner", bore_diameter=0.006),
        ),
        "propeller_scimitar_blades",
    )

    for index, (arm_name, hinge_xyz, deployed_yaw, fold_axis) in enumerate(arm_specs):
        arm = model.part(arm_name)
        arm.visual(
            Cylinder(radius=0.026, length=0.074),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=metal,
            name="hinge_barrel",
        )
        arm.visual(
            Box((0.520, 0.034, 0.026)),
            origin=Origin(xyz=(0.285, 0.0, 0.0)),
            material=carbon,
            name="carbon_spar",
        )
        arm.visual(
            Box((0.260, 0.018, 0.016)),
            origin=Origin(xyz=(0.210, 0.0, 0.021)),
            material=graphite,
            name="wire_fairing",
        )
        arm.visual(
            Cylinder(radius=0.068, length=0.046),
            origin=Origin(xyz=(0.580, 0.0, 0.0)),
            material=graphite,
            name="motor_pod",
        )
        arm.visual(
            Cylinder(radius=0.074, length=0.009),
            origin=Origin(xyz=(0.580, 0.0, 0.0275)),
            material=metal,
            name="motor_cap",
        )

        model.articulation(
            f"body_to_{arm_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, 0.0, deployed_yaw)),
            axis=fold_axis,
            motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.55),
        )

        propeller = model.part(f"propeller_{index}")
        propeller.visual(
            propeller_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=rubber,
            name="rotor_blades",
        )
        propeller.visual(
            Cylinder(radius=0.010, length=0.0175),
            origin=Origin(xyz=(0.0, 0.0, -0.00875)),
            material=metal,
            name="drive_shaft",
        )

        model.articulation(
            f"{arm_name}_to_propeller_{index}",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=propeller,
            origin=Origin(xyz=(0.580, 0.0, 0.0495)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=120.0),
        )

    skid = model.part("payload_skid")
    skid.visual(
        Cylinder(radius=0.018, length=0.332),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_tube",
    )
    for index, y in enumerate((0.135, -0.135)):
        skid.visual(
            Box((0.026, 0.026, 0.240)),
            origin=Origin(xyz=(0.020, y, -0.120)),
            material=metal,
            name=f"drop_strut_{index}",
        )
        skid.visual(
            Box((0.455, 0.030, 0.026)),
            origin=Origin(xyz=(0.225, y, -0.252)),
            material=rubber,
            name=f"runner_{index}",
        )
    skid.visual(
        Box((0.026, 0.322, 0.022)),
        origin=Origin(xyz=(0.020, 0.0, -0.252)),
        material=metal,
        name="front_crossbar",
    )
    skid.visual(
        Box((0.026, 0.322, 0.022)),
        origin=Origin(xyz=(0.425, 0.0, -0.252)),
        material=metal,
        name="rear_crossbar",
    )

    model.articulation(
        "body_to_payload_skid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=skid,
        origin=Origin(xyz=(-0.230, 0.0, -0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    arm_joints = [object_model.get_articulation(f"body_to_arm_{i}") for i in range(4)]
    prop_joints = [object_model.get_articulation(f"arm_{i}_to_propeller_{i}") for i in range(4)]
    skid_joint = object_model.get_articulation("body_to_payload_skid")

    ctx.check(
        "four folding arm hinges",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in arm_joints),
        details=f"arm joint types={[j.articulation_type for j in arm_joints]}",
    )
    ctx.check(
        "four continuous propellers",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in prop_joints),
        details=f"prop joint types={[j.articulation_type for j in prop_joints]}",
    )
    ctx.check(
        "skid has retract hinge",
        skid_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"skid joint type={skid_joint.articulation_type}",
    )

    for i in range(4):
        ctx.expect_gap(
            f"propeller_{i}",
            f"arm_{i}",
            axis="z",
            positive_elem="drive_shaft",
            negative_elem="motor_cap",
            max_gap=0.002,
            max_penetration=0.0001,
            name=f"propeller_{i} shaft seats on motor",
        )

    arm_0 = object_model.get_part("arm_0")
    rest_motor = ctx.part_element_world_aabb(arm_0, elem="motor_pod")
    with ctx.pose({arm_joints[0]: 1.10}):
        folded_motor = ctx.part_element_world_aabb(arm_0, elem="motor_pod")
    if rest_motor is not None and folded_motor is not None:
        rest_center_x = (rest_motor[0][0] + rest_motor[1][0]) * 0.5
        folded_center_x = (folded_motor[0][0] + folded_motor[1][0]) * 0.5
        ctx.check(
            "front arm folds rearward",
            folded_center_x < rest_center_x - 0.18,
            details=f"rest_x={rest_center_x:.3f}, folded_x={folded_center_x:.3f}",
        )
    else:
        ctx.fail("front arm folds rearward", "could not measure arm_0 motor pod")

    skid = object_model.get_part("payload_skid")
    rest_runner = ctx.part_element_world_aabb(skid, elem="runner_0")
    with ctx.pose({skid_joint: 0.55}):
        retracted_runner = ctx.part_element_world_aabb(skid, elem="runner_0")
    if rest_runner is not None and retracted_runner is not None:
        rest_center_z = (rest_runner[0][2] + rest_runner[1][2]) * 0.5
        retracted_center_z = (retracted_runner[0][2] + retracted_runner[1][2]) * 0.5
        ctx.check(
            "payload skid retracts upward",
            retracted_center_z > rest_center_z + 0.12,
            details=f"rest_z={rest_center_z:.3f}, retracted_z={retracted_center_z:.3f}",
        )
    else:
        ctx.fail("payload skid retracts upward", "could not measure skid runner")

    return ctx.report()


object_model = build_object_model()
