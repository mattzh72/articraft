from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roll_axis_rotary_module")

    frame_paint = model.material("black_anodized_frame", rgba=(0.06, 0.065, 0.07, 1.0))
    bearing_body = model.material("bearing_cartridge_blue", rgba=(0.10, 0.18, 0.30, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.54, 0.57, 0.60, 1.0))
    bolt_black = model.material("black_socket_heads", rgba=(0.015, 0.015, 0.016, 1.0))
    shaft_steel = model.material("ground_shaft_steel", rgba=(0.70, 0.72, 0.73, 1.0))
    rotor_orange = model.material("rotor_anodized_orange", rgba=(0.92, 0.38, 0.08, 1.0))
    rotor_dark = model.material("rotor_dark_cap", rgba=(0.18, 0.18, 0.19, 1.0))

    outer_frame = model.part("outer_frame")
    # The rectangular cage is one fixed, connected housing.  Each side cheek is
    # built as four real webs around the shaft opening so the rotating member
    # passes through actual clearance instead of a hidden solid plate.
    for x, suffix in ((-0.245, "0"), (0.245, "1")):
        outer_frame.visual(
            Box((0.052, 0.160, 0.092)),
            origin=Origin(xyz=(x, 0.0, 0.134)),
            material=frame_paint,
            name=f"side_upper_web_{suffix}",
        )
        outer_frame.visual(
            Box((0.052, 0.160, 0.092)),
            origin=Origin(xyz=(x, 0.0, -0.134)),
            material=frame_paint,
            name=f"side_lower_web_{suffix}",
        )
        outer_frame.visual(
            Box((0.052, 0.032, 0.176)),
            origin=Origin(xyz=(x, -0.064, 0.0)),
            material=frame_paint,
            name=f"side_rear_web_{suffix}",
        )
        outer_frame.visual(
            Box((0.052, 0.032, 0.176)),
            origin=Origin(xyz=(x, 0.064, 0.0)),
            material=frame_paint,
            name=f"side_front_web_{suffix}",
        )
    outer_frame.visual(
        Box((0.545, 0.120, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=frame_paint,
        name="top_rail",
    )
    outer_frame.visual(
        Box((0.545, 0.120, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.174)),
        material=frame_paint,
        name="bottom_rail",
    )
    outer_frame.visual(
        Box((0.090, 0.180, 0.030)),
        origin=Origin(xyz=(-0.245, 0.0, -0.214)),
        material=frame_paint,
        name="mount_foot_0",
    )
    outer_frame.visual(
        Box((0.090, 0.180, 0.030)),
        origin=Origin(xyz=(0.245, 0.0, -0.214)),
        material=frame_paint,
        name="mount_foot_1",
    )

    for x, suffix in ((-0.205, "0"), (0.205, "1")):
        outer_frame.visual(
            Box((0.050, 0.132, 0.034)),
            origin=Origin(xyz=(x, 0.0, 0.053)),
            material=bearing_body,
            name=f"bearing_top_block_{suffix}",
        )
        outer_frame.visual(
            Box((0.050, 0.132, 0.034)),
            origin=Origin(xyz=(x, 0.0, -0.053)),
            material=bearing_body,
            name=f"bearing_bottom_block_{suffix}",
        )
        outer_frame.visual(
            Box((0.050, 0.034, 0.072)),
            origin=Origin(xyz=(x, -0.049, 0.0)),
            material=bearing_body,
            name=f"bearing_rear_block_{suffix}",
        )
        outer_frame.visual(
            Box((0.050, 0.034, 0.072)),
            origin=Origin(xyz=(x, 0.049, 0.0)),
            material=bearing_body,
            name=f"bearing_front_block_{suffix}",
        )
        outer_frame.visual(
            Box((0.020, 0.062, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.029)),
            material=bearing_steel,
            name=f"bearing_race_top_{suffix}",
        )
        outer_frame.visual(
            Box((0.020, 0.062, 0.014)),
            origin=Origin(xyz=(x, 0.0, -0.029)),
            material=bearing_steel,
            name=f"bearing_race_bottom_{suffix}",
        )
        outer_frame.visual(
            Box((0.020, 0.010, 0.044)),
            origin=Origin(xyz=(x, -0.027, 0.0)),
            material=bearing_steel,
            name=f"bearing_race_rear_{suffix}",
        )
        outer_frame.visual(
            Box((0.020, 0.010, 0.044)),
            origin=Origin(xyz=(x, 0.027, 0.0)),
            material=bearing_steel,
            name=f"bearing_race_front_{suffix}",
        )
        bolt_x = x + (-0.024 if x < 0 else 0.024)
        for bolt_i, (yi, zi) in enumerate(
            ((-0.044, -0.044), (-0.044, 0.044), (0.044, -0.044), (0.044, 0.044))
        ):
            outer_frame.visual(
                Cylinder(radius=0.0065, length=0.008),
                origin=Origin(xyz=(bolt_x, yi, zi), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bolt_black,
                name=f"cartridge_bolt_{suffix}_{bolt_i}",
            )

    outer_frame.inertial = Inertial.from_geometry(
        Box((0.56, 0.20, 0.50)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    rotating_member = model.part("rotating_member")
    rotating_member.visual(
        Cylinder(radius=0.022, length=0.610),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="central_shaft",
    )
    rotating_member.visual(
        Box((0.150, 0.128, 0.066)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rotor_orange,
        name="rectangular_rotor",
    )
    rotating_member.visual(
        Box((0.082, 0.156, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rotor_dark,
        name="payload_face",
    )
    for x, suffix in ((-0.145, "0"), (0.145, "1")):
        rotating_member.visual(
            Cylinder(radius=0.039, length=0.040),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rotor_dark,
            name=f"shaft_collar_{suffix}",
        )
        rotating_member.visual(
            Box((0.012, 0.102, 0.044)),
            origin=Origin(xyz=(x * 0.72, 0.0, 0.0)),
            material=rotor_orange,
            name=f"web_rib_{suffix}",
        )

    rotating_member.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.61),
        mass=1.1,
        origin=Origin(),
    )

    model.articulation(
        "roll_axis",
        ArticulationType.CONTINUOUS,
        parent=outer_frame,
        child=rotating_member,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("outer_frame")
    rotor = object_model.get_part("rotating_member")
    roll = object_model.get_articulation("roll_axis")

    ctx.expect_contact(
        rotor,
        frame,
        elem_a="central_shaft",
        elem_b="bearing_race_top_0",
        contact_tol=0.001,
        name="shaft is seated in first bearing cartridge",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="central_shaft",
        elem_b="bearing_race_bottom_1",
        contact_tol=0.001,
        name="shaft is seated in second bearing cartridge",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="yz",
        margin=0.0,
        name="rotating member fits inside the rectangular frame envelope",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="x",
        elem_a="central_shaft",
        min_overlap=0.50,
        name="central shaft spans both bearing cheeks",
    )

    rest_aabb = ctx.part_world_aabb(rotor)
    with ctx.pose({roll: math.pi / 2.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="yz",
            margin=0.0,
            name="rotor clears frame after a quarter roll",
        )
        rolled_aabb = ctx.part_world_aabb(rotor)

    if rest_aabb is not None and rolled_aabb is not None:
        rest_y = rest_aabb[1][1] - rest_aabb[0][1]
        rolled_y = rolled_aabb[1][1] - rolled_aabb[0][1]
        ctx.check(
            "roll joint rotates the rectangular member",
            abs(rest_y - rolled_y) > 0.030,
            details=f"rest_y={rest_y:.4f}, rolled_y={rolled_y:.4f}",
        )
    else:
        ctx.fail("roll joint rotates the rectangular member", "AABB unavailable for rotor.")

    return ctx.report()


object_model = build_object_model()
