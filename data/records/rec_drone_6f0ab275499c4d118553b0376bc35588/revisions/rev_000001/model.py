from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PLATE_R = 0.140
PLATE_T = 0.018
HINGE_Z = PLATE_T / 2.0 + 0.030
HINGE_BARREL_R = 0.018
ARM_LENGTH = 0.420
ARM_ROOT_X = HINGE_BARREL_R
ARM_ROOT_LEN = 0.052
BOOM_START_X = 0.055
BOOM_RADIUS = 0.011
MOTOR_RADIUS = 0.045
MOTOR_DEPTH = 0.040
PROP_RADIUS = 0.088
PROP_THICKNESS = 0.012


def _radial_xy(radius: float, angle: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), 0.0)


def _hex_plate_shape() -> cq.Workplane:
    points = [
        (PLATE_R * math.cos(i * math.tau / 6.0), PLATE_R * math.sin(i * math.tau / 6.0))
        for i in range(6)
    ]
    return (
        cq.Workplane("XY")
        .polyline(points)
        .close()
        .extrude(PLATE_T)
        .translate((0.0, 0.0, -PLATE_T / 2.0))
    )


def _motor_cap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.021)
        .circle(0.009)
        .extrude(0.010)
        .translate((0.0, 0.0, -0.005))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_hexarotor")

    model.material("carbon_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("matte_graphite", rgba=(0.075, 0.080, 0.088, 1.0))
    model.material("hinge_metal", rgba=(0.46, 0.48, 0.50, 1.0))
    model.material("motor_dark", rgba=(0.030, 0.034, 0.040, 1.0))
    model.material("prop_smoke", rgba=(0.05, 0.06, 0.07, 0.72))
    model.material("blue_detail", rgba=(0.03, 0.16, 0.42, 1.0))

    center_plate = model.part("center_plate")
    center_plate.visual(
        mesh_from_cadquery(_hex_plate_shape(), "hex_center_plate"),
        material="carbon_black",
        name="hex_plate",
    )
    center_plate.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, PLATE_T / 2.0 + 0.010)),
        material="matte_graphite",
        name="flight_stack",
    )
    center_plate.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, PLATE_T / 2.0 + 0.024)),
        material="blue_detail",
        name="gps_puck",
    )

    motor_cap_mesh = mesh_from_cadquery(_motor_cap_shape(), "motor_cap_ring")
    propeller_meshes = []
    for i in range(6):
        propeller_meshes.append(
            mesh_from_geometry(
                FanRotorGeometry(
                    PROP_RADIUS,
                    0.018,
                    2,
                    thickness=PROP_THICKNESS,
                    blade_pitch_deg=18.0,
                    blade_sweep_deg=8.0,
                    blade_root_chord=0.027,
                    blade_tip_chord=0.021,
                    blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=10.0, camber=0.10),
                    hub=FanRotorHub(style="spinner", bore_diameter=0.006),
                ),
                f"two_blade_propeller_{i}",
            )
        )

    for i in range(6):
        angle = i * math.tau / 6.0
        hinge_xy = _radial_xy(PLATE_R, angle)
        pad_center_xy = _radial_xy(PLATE_R + 0.005, angle)

        center_plate.visual(
            Box((0.082, 0.040, 0.010)),
            origin=Origin(
                xyz=(pad_center_xy[0], pad_center_xy[1], PLATE_T / 2.0 + 0.005),
                rpy=(0.0, 0.0, angle),
            ),
            material="hinge_metal",
            name=f"hinge_pad_{i}",
        )
        center_plate.visual(
            Cylinder(radius=HINGE_BARREL_R, length=0.054),
            origin=Origin(xyz=(hinge_xy[0], hinge_xy[1], HINGE_Z)),
            material="hinge_metal",
            name=f"hinge_barrel_{i}",
        )

        arm = model.part(f"arm_{i}")
        arm.visual(
            Box((ARM_ROOT_LEN, 0.044, 0.018)),
            origin=Origin(xyz=(ARM_ROOT_X + ARM_ROOT_LEN / 2.0, 0.0, 0.0)),
            material="hinge_metal",
            name="root_lug",
        )
        arm.visual(
            Cylinder(radius=BOOM_RADIUS, length=ARM_LENGTH - BOOM_START_X),
            origin=Origin(
                xyz=((BOOM_START_X + ARM_LENGTH) / 2.0, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="carbon_black",
            name="carbon_boom",
        )
        arm.visual(
            Box((0.100, 0.022, 0.012)),
            origin=Origin(xyz=(0.230, 0.0, 0.0)),
            material="blue_detail",
            name="wire_fairing",
        )
        arm.visual(
            Cylinder(radius=MOTOR_RADIUS, length=MOTOR_DEPTH),
            origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.0)),
            material="motor_dark",
            name="motor_pod",
        )
        arm.visual(
            motor_cap_mesh,
            origin=Origin(xyz=(ARM_LENGTH, 0.0, MOTOR_DEPTH / 2.0 + 0.005)),
            material="hinge_metal",
            name="motor_cap",
        )

        model.articulation(
            f"arm_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=center_plate,
            child=arm,
            origin=Origin(xyz=(hinge_xy[0], hinge_xy[1], HINGE_Z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=math.pi),
        )

        propeller = model.part(f"propeller_{i}")
        propeller.visual(
            Cylinder(radius=0.007, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material="hinge_metal",
            name="prop_shaft",
        )
        propeller.visual(
            propeller_meshes[i],
            origin=Origin(xyz=(0.0, 0.0, 0.022)),
            material="prop_smoke",
            name="two_blade_rotor",
        )

        model.articulation(
            f"prop_axle_{i}",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=propeller,
            origin=Origin(xyz=(ARM_LENGTH, 0.0, MOTOR_DEPTH / 2.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=120.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center = object_model.get_part("center_plate")
    ctx.check("six_folding_arms", all(object_model.get_part(f"arm_{i}") is not None for i in range(6)))
    ctx.check("six_propellers", all(object_model.get_part(f"propeller_{i}") is not None for i in range(6)))
    ctx.check("six_arm_hinges", all(object_model.get_articulation(f"arm_hinge_{i}") is not None for i in range(6)))
    ctx.check("six_propeller_axles", all(object_model.get_articulation(f"prop_axle_{i}") is not None for i in range(6)))

    for i in range(6):
        arm = object_model.get_part(f"arm_{i}")
        propeller = object_model.get_part(f"propeller_{i}")
        hinge = object_model.get_articulation(f"arm_hinge_{i}")
        axle = object_model.get_articulation(f"prop_axle_{i}")
        if arm is None or propeller is None or hinge is None or axle is None:
            continue
        ctx.check(
            f"arm_hinge_{i}_is_revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={hinge.articulation_type!r}",
        )
        ctx.check(
            f"prop_axle_{i}_spins",
            axle.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={axle.articulation_type!r}",
        )
        ctx.expect_contact(
            arm,
            center,
            elem_a="root_lug",
            elem_b=f"hinge_barrel_{i}",
            contact_tol=0.0015,
            name=f"arm_{i}_root_sits_on_hinge_barrel",
        )
        ctx.expect_gap(
            propeller,
            arm,
            axis="z",
            positive_elem="prop_shaft",
            negative_elem="motor_pod",
            max_gap=0.001,
            max_penetration=0.000001,
            name=f"propeller_{i}_shaft_seats_on_motor",
        )

    arm_0 = object_model.get_part("arm_0")
    hinge_0 = object_model.get_articulation("arm_hinge_0")
    if arm_0 is not None and hinge_0 is not None:
        rest_aabb = ctx.part_element_world_aabb(arm_0, elem="motor_pod")
        with ctx.pose({hinge_0: math.pi}):
            folded_aabb = ctx.part_element_world_aabb(arm_0, elem="motor_pod")

        def _xy_radius(aabb) -> float | None:
            if aabb is None:
                return None
            mins, maxs = aabb
            cx = (float(mins[0]) + float(maxs[0])) * 0.5
            cy = (float(mins[1]) + float(maxs[1])) * 0.5
            return math.hypot(cx, cy)

        rest_radius = _xy_radius(rest_aabb)
        folded_radius = _xy_radius(folded_aabb)
        ctx.check(
            "arm_0_folds_inward",
            rest_radius is not None
            and folded_radius is not None
            and folded_radius < rest_radius - 0.20,
            details=f"rest_radius={rest_radius}, folded_radius={folded_radius}",
        )

    return ctx.report()


object_model = build_object_model()
