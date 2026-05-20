from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    galvanized = model.material("galvanized_steel", color=(0.62, 0.66, 0.67, 1.0))
    dark_base = model.material("dark_powder_coat", color=(0.06, 0.07, 0.08, 1.0))
    bearing_black = model.material("black_bearing", color=(0.015, 0.016, 0.018, 1.0))
    safety_red = model.material("red_safety_caps", color=(0.78, 0.05, 0.03, 1.0))

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.82, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_base,
        name="floor_plate",
    )

    # Three low outriggers make the support visibly wide and stable without
    # raising the rotating mass far above the floor plane.
    for i, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        frame.visual(
            Box((0.56, 0.13, 0.035)),
            origin=Origin(
                xyz=(0.66 * math.cos(yaw), 0.66 * math.sin(yaw), 0.0175),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_base,
            name=f"floor_outrigger_{i}",
        )

    frame.visual(
        Cylinder(radius=0.055, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.1775)),
        material=galvanized,
        name="center_column",
    )
    frame.visual(
        Cylinder(radius=0.105, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=bearing_black,
        name="fixed_bearing_seat",
    )

    guard_ring_shape = (
        cq.Workplane("XY")
        .circle(0.780)
        .circle(0.680)
        .extrude(0.040)
        .translate((0.0, 0.0, 0.360))
    )
    frame.visual(
        mesh_from_cadquery(guard_ring_shape, "guard_ring"),
        material=galvanized,
        name="guard_ring",
    )

    for i in range(6):
        yaw = i * math.pi / 3.0
        frame.visual(
            Cylinder(radius=0.025, length=0.310),
            origin=Origin(
                xyz=(0.735 * math.cos(yaw), 0.735 * math.sin(yaw), 0.210),
            ),
            material=galvanized,
            name=f"guard_post_{i}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.092, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="rotor_hub",
    )
    rotor.visual(
        Cylinder(radius=0.112, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=bearing_black,
        name="hub_cap",
    )

    arm_center_radius = 0.345
    arm_length = 0.520
    arm_tip_radius = 0.600
    for i, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            Cylinder(radius=0.026, length=arm_length),
            origin=Origin(
                xyz=(arm_center_radius * math.cos(yaw), arm_center_radius * math.sin(yaw), 0.0),
                rpy=(0.0, math.pi / 2.0, yaw),
            ),
            material=galvanized,
            name=f"arm_{i}",
        )
        rotor.visual(
            Sphere(radius=0.038),
            origin=Origin(xyz=(arm_tip_radius * math.cos(yaw), arm_tip_radius * math.sin(yaw), 0.0)),
            material=safety_red,
            name=f"arm_cap_{i}",
        )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.check(
        "hub uses continuous vertical rotation",
        hub_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(float(v), 6) for v in hub_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={hub_spin.articulation_type}, axis={hub_spin.axis}",
    )

    ctx.expect_contact(
        frame,
        rotor,
        elem_a="fixed_bearing_seat",
        elem_b="rotor_hub",
        contact_tol=0.001,
        name="rotor hub sits on fixed bearing seat",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        outer_elem="guard_ring",
        margin=0.0,
        name="three arms remain inside the fixed guard frame",
    )

    rotor_aabb = ctx.part_world_aabb(rotor)
    ctx.check(
        "rotating mass remains low",
        rotor_aabb is not None and rotor_aabb[1][2] < 0.43,
        details=f"rotor_aabb={rotor_aabb}",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({hub_spin: 2.0 * math.pi / 3.0}):
        spun_pos = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            outer_elem="guard_ring",
            margin=0.0,
            name="rotated arms remain inside the fixed guard frame",
        )
    ctx.check(
        "continuous spin keeps hub on center axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
