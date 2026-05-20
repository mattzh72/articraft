from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_RADIUS = 0.42
BODY_THICKNESS = 0.09
ROTOR_CENTER_RADIUS = 0.86
PROPELLER_RADIUS = 0.18
MOTOR_TOP_Z = 0.12


def _regular_hex_profile(radius: float) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((math.tau * index) / 6.0),
            radius * math.sin((math.tau * index) / 6.0),
        )
        for index in range(6)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hexarotor_delivery_drone")

    carbon = model.material("matte_carbon", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_composite = model.material("dark_composite", rgba=(0.10, 0.11, 0.12, 1.0))
    graphite = model.material("graphite_motor", rgba=(0.20, 0.21, 0.22, 1.0))
    metal = model.material("brushed_metal", rgba=(0.58, 0.60, 0.61, 1.0))
    delivery_yellow = model.material("delivery_yellow", rgba=(0.95, 0.68, 0.16, 1.0))
    beacon_blue = model.material("beacon_blue", rgba=(0.05, 0.30, 0.85, 1.0))

    body_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(_regular_hex_profile(BODY_RADIUS), BODY_THICKNESS),
        "flat_hex_body",
    )
    hatch_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(_regular_hex_profile(0.24), 0.014),
        "top_hex_hatch",
    )
    propeller_mesh = mesh_from_geometry(
        FanRotorGeometry(
            PROPELLER_RADIUS,
            0.034,
            2,
            thickness=0.016,
            blade_pitch_deg=22.0,
            blade_sweep_deg=12.0,
            blade_root_chord=0.060,
            blade_tip_chord=0.040,
            blade=FanRotorBlade(shape="narrow", tip_pitch_deg=11.0, camber=0.08),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.028),
        ),
        "two_blade_propeller",
    )

    frame = model.part("frame")
    frame.visual(body_mesh, material=dark_composite, name="hex_body")
    frame.visual(
        hatch_mesh,
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS / 2.0 + 0.007)),
        material=carbon,
        name="top_hatch",
    )
    frame.visual(
        Cylinder(radius=0.065, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS / 2.0 + 0.029)),
        material=beacon_blue,
        name="gps_puck",
    )
    frame.visual(
        Box((0.34, 0.25, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=delivery_yellow,
        name="parcel_box",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            frame.visual(
                Box((0.026, 0.026, 0.07)),
                origin=Origin(xyz=(0.13 * sx, 0.09 * sy, -0.075)),
                material=metal,
                name=f"parcel_post_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    arm_length = 0.66
    arm_center_radius = 0.57
    for index in range(6):
        angle = (math.tau * index) / 6.0
        ca = math.cos(angle)
        sa = math.sin(angle)
        motor_x = ROTOR_CENTER_RADIUS * ca
        motor_y = ROTOR_CENTER_RADIUS * sa

        frame.visual(
            Box((arm_length, 0.070, 0.046)),
            origin=Origin(
                xyz=(arm_center_radius * ca, arm_center_radius * sa, 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=carbon,
            name=f"arm_{index}",
        )
        frame.visual(
            Box((0.18, 0.100, 0.030)),
            origin=Origin(
                xyz=((ROTOR_CENTER_RADIUS - 0.14) * ca, (ROTOR_CENTER_RADIUS - 0.14) * sa, -0.010),
                rpy=(0.0, 0.0, angle),
            ),
            material=metal,
            name=f"arm_clamp_{index}",
        )
        frame.visual(
            Cylinder(radius=0.105, length=0.140),
            origin=Origin(xyz=(motor_x, motor_y, 0.050)),
            material=graphite,
            name=f"motor_{index}",
        )
        frame.visual(
            Cylinder(radius=0.045, length=0.030),
            origin=Origin(xyz=(motor_x, motor_y, MOTOR_TOP_Z - 0.015)),
            material=metal,
            name=f"motor_cap_{index}",
        )

        propeller = model.part(f"propeller_{index}")
        propeller.visual(
            Cylinder(radius=0.018, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, 0.016)),
            material=metal,
            name="propeller_shaft",
        )
        propeller.visual(
            propeller_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.036), rpy=(0.0, 0.0, math.pi / 2.0 if index % 2 else 0.0)),
            material=carbon,
            name="propeller_rotor",
        )

        model.articulation(
            f"propeller_axis_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=propeller,
            origin=Origin(xyz=(motor_x, motor_y, MOTOR_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=220.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    propellers = [object_model.get_part(f"propeller_{index}") for index in range(6)]
    joints = [object_model.get_articulation(f"propeller_axis_{index}") for index in range(6)]

    ctx.check("six propellers", len(propellers) == 6, details=f"count={len(propellers)}")
    ctx.check("six spin joints", len(joints) == 6, details=f"count={len(joints)}")
    for index, joint in enumerate(joints):
        ctx.check(
            f"propeller {index} rotates about vertical axis",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.expect_gap(
            propellers[index],
            frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="propeller_shaft",
            negative_elem=f"motor_{index}",
            name=f"propeller {index} shaft seats on motor",
        )
        ctx.expect_gap(
            propellers[index],
            frame,
            axis="z",
            min_gap=0.020,
            positive_elem="propeller_rotor",
            negative_elem=f"motor_{index}",
            name=f"propeller {index} blades clear motor",
        )

    return ctx.report()


object_model = build_object_model()
