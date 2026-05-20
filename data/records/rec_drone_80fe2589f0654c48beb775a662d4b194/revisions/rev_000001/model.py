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
    model = ArticulatedObject(name="fixed_arm_racing_quadrotor")

    carbon = model.material("matte_carbon", rgba=(0.015, 0.016, 0.018, 1.0))
    black = model.material("anodized_black", rgba=(0.02, 0.02, 0.025, 1.0))
    graphite = model.material("graphite_edge", rgba=(0.10, 0.105, 0.11, 1.0))
    motor_metal = model.material("dark_motor_metal", rgba=(0.18, 0.18, 0.19, 1.0))
    copper = model.material("copper_windings", rgba=(0.75, 0.34, 0.12, 1.0))
    battery = model.material("battery_pack", rgba=(0.08, 0.08, 0.075, 1.0))
    strap = model.material("red_battery_strap", rgba=(0.80, 0.03, 0.03, 1.0))
    lens_glass = model.material("smoked_lens", rgba=(0.03, 0.06, 0.08, 1.0))
    prop_material = model.material("translucent_lime_prop", rgba=(0.20, 0.95, 0.38, 0.62))

    frame = model.part("frame")

    # X-frame carbon plates: two narrow plates separated by standoffs.
    frame.visual(
        Box((0.132, 0.082, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=carbon,
        name="lower_plate",
    )
    frame.visual(
        Box((0.112, 0.068, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=carbon,
        name="upper_plate",
    )

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            frame.visual(
                Cylinder(radius=0.0045, length=0.034),
                origin=Origin(xyz=(sx * 0.045, sy * 0.027, 0.033)),
                material=graphite,
                name=f"standoff_{sx:+.0f}_{sy:+.0f}",
            )

    # The fixed diagonal arms run from the body to each motor pod.
    motor_centers = {
        "front_left": (0.112, 0.112),
        "front_right": (0.112, -0.112),
        "rear_left": (-0.112, 0.112),
        "rear_right": (-0.112, -0.112),
    }
    for corner, (x, y) in motor_centers.items():
        yaw = math.atan2(y, x)
        arm_length = math.hypot(x, y) + 0.032
        frame.visual(
            Box((arm_length, 0.018, 0.012)),
            origin=Origin(xyz=(x * 0.50, y * 0.50, 0.018), rpy=(0.0, 0.0, yaw)),
            material=carbon,
            name=f"{corner}_arm",
        )
        frame.visual(
            Box((0.045, 0.018, 0.004)),
            origin=Origin(xyz=(x * 0.88, y * 0.88, 0.028), rpy=(0.0, 0.0, yaw)),
            material=graphite,
            name=f"{corner}_arm_cap",
        )
        frame.visual(
            Cylinder(radius=0.028, length=0.028),
            origin=Origin(xyz=(x, y, 0.038)),
            material=motor_metal,
            name=f"{corner}_motor",
        )
        frame.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(x, y, 0.057)),
            material=black,
            name=f"{corner}_motor_cap",
        )
        frame.visual(
            Cylinder(radius=0.024, length=0.003),
            origin=Origin(xyz=(x, y, 0.0245)),
            material=copper,
            name=f"{corner}_winding_band",
        )

    # Racing-quad accessories mounted to the frame give the body a front.
    frame.visual(
        Box((0.075, 0.042, 0.028)),
        origin=Origin(xyz=(-0.006, 0.0, 0.061)),
        material=battery,
        name="battery",
    )
    frame.visual(
        Box((0.020, 0.092, 0.006)),
        origin=Origin(xyz=(-0.006, 0.0, 0.078)),
        material=strap,
        name="battery_strap",
    )
    frame.visual(
        Box((0.020, 0.026, 0.018)),
        origin=Origin(xyz=(0.061, 0.0, 0.052)),
        material=black,
        name="front_camera_body",
    )
    frame.visual(
        Cylinder(radius=0.0085, length=0.014),
        origin=Origin(xyz=(0.078, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_camera_lens",
    )

    prop_mesh = mesh_from_geometry(
        FanRotorGeometry(
            outer_radius=0.064,
            hub_radius=0.014,
            blade_count=2,
            thickness=0.012,
            blade_pitch_deg=33.0,
            blade_sweep_deg=14.0,
            blade_root_chord=0.029,
            blade_tip_chord=0.018,
            blade=FanRotorBlade(
                shape="scimitar",
                tip_pitch_deg=18.0,
                camber=0.10,
                tip_clearance=0.0015,
            ),
            hub=FanRotorHub(style="spinner", bore_diameter=0.004),
        ),
        "two_blade_propeller",
    )

    for corner, (x, y) in motor_centers.items():
        propeller = model.part(f"{corner}_propeller")
        propeller.visual(
            prop_mesh,
            origin=Origin(),
            material=prop_material,
            name="rotor",
        )
        propeller.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=black,
            name="hub_collar",
        )
        # A short rotating axle visually seats the propeller on the motor pod.
        propeller.visual(
            Cylinder(radius=0.004, length=0.017),
            origin=Origin(xyz=(0.0, 0.0, -0.0145)),
            material=black,
            name="axle",
        )
        model.articulation(
            f"{corner}_axle",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=propeller,
            origin=Origin(xyz=(x, y, 0.085)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.12, velocity=900.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    corners = ("front_left", "front_right", "rear_left", "rear_right")
    for corner in corners:
        propeller = object_model.get_part(f"{corner}_propeller")
        axle = object_model.get_articulation(f"{corner}_axle")

        ctx.expect_contact(
            propeller,
            "frame",
            elem_a="axle",
            elem_b=f"{corner}_motor_cap",
            contact_tol=1e-5,
            name=f"{corner} rotating axle is seated on the motor cap",
        )
        ctx.expect_within(
            propeller,
            "frame",
            axes="xy",
            inner_elem="axle",
            outer_elem=f"{corner}_motor_cap",
            margin=0.001,
            name=f"{corner} axle is centered inside the motor pod footprint",
        )
        ctx.expect_gap(
            propeller,
            "frame",
            axis="z",
            min_gap=0.010,
            positive_elem="rotor",
            negative_elem=f"{corner}_motor_cap",
            name=f"{corner} propeller disk sits above its motor pod",
        )

        rest_position = ctx.part_world_position(propeller)
        with ctx.pose({axle: math.pi / 2.0}):
            spun_position = ctx.part_world_position(propeller)
            ctx.expect_gap(
                propeller,
                "frame",
                axis="z",
                min_gap=0.010,
                positive_elem="rotor",
                negative_elem=f"{corner}_motor_cap",
                name=f"{corner} propeller clearance remains after a quarter turn",
            )
        ctx.check(
            f"{corner} revolute axle spins without translating the propeller",
            rest_position is not None
            and spun_position is not None
            and all(abs(a - b) < 1e-6 for a, b in zip(rest_position, spun_position)),
            details=f"rest={rest_position}, spun={spun_position}",
        )

    ctx.check(
        "quadrotor has four independent propeller axles",
        len(corners) == 4
        and all(object_model.get_articulation(f"{corner}_axle") is not None for corner in corners),
        details="Expected one continuous propeller axle above each of the four motor pods.",
    )

    return ctx.report()


object_model = build_object_model()
