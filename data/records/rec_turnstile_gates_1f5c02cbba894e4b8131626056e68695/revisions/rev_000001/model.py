from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    dark_paint = model.material("dark_paint", rgba=(0.05, 0.055, 0.06, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.08, 1.0))
    black = model.material("black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))

    frame = model.part("frame")

    # A compact floor-mounted guard frame: broad base, central support pedestal,
    # and perimeter rails that keep the rotating three-arm stage enclosed.
    frame.visual(
        Box((1.55, 1.55, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_paint,
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=galvanized,
        name="central_column",
    )
    frame.visual(
        Cylinder(radius=0.085, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.788)),
        material=brushed,
        name="bearing_cup",
    )

    half = 0.72
    for ix, x in enumerate((-half, half)):
        for iy, y in enumerate((-half, half)):
            frame.visual(
                Cylinder(radius=0.035, length=1.42),
                origin=Origin(xyz=(x, y, 0.735)),
                material=dark_paint,
                name=f"corner_post_{ix}_{iy}",
            )

    for y in (-half, half):
        for z, radius, label in ((0.63, 0.022, "lower"), (1.42, 0.026, "top")):
            frame.visual(
                Cylinder(radius=radius, length=2.0 * half),
                origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=galvanized,
                name=f"{label}_rail_x_{'pos' if y > 0 else 'neg'}",
            )

    for x in (-half, half):
        for z, radius, label in ((0.63, 0.022, "lower"), (1.42, 0.026, "top")):
            frame.visual(
                Cylinder(radius=radius, length=2.0 * half),
                origin=Origin(xyz=(x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=galvanized,
                name=f"{label}_rail_y_{'pos' if x > 0 else 'neg'}",
            )

    rotor = model.part("rotor")

    rotor.visual(
        Cylinder(radius=0.105, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed,
        name="hub_body",
    )
    rotor.visual(
        Cylinder(radius=0.125, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=black,
        name="upper_hub_cap",
    )
    rotor.visual(
        Cylinder(radius=0.125, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.1375)),
        material=black,
        name="lower_hub_cap",
    )

    arm_length = 0.60
    arm_center_radius = 0.35
    tip_radius = 0.035
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        x = arm_center_radius * math.cos(angle)
        y = arm_center_radius * math.sin(angle)
        tip_x = (arm_center_radius + arm_length / 2.0) * math.cos(angle)
        tip_y = (arm_center_radius + arm_length / 2.0) * math.sin(angle)
        rotor.visual(
            Cylinder(radius=0.025, length=arm_length),
            origin=Origin(xyz=(x, y, 0.0), rpy=(0.0, math.pi / 2.0, angle)),
            material=brushed,
            name=f"arm_{idx}",
        )
        rotor.visual(
            Sphere(radius=tip_radius),
            origin=Origin(xyz=(tip_x, tip_y, 0.0)),
            material=yellow,
            name=f"arm_tip_{idx}",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.963)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "rotor uses continuous vertical joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="lower_hub_cap",
        negative_elem="bearing_cup",
        max_gap=0.002,
        max_penetration=1e-6,
        name="hub sits on support bearing",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        outer_elem="base_plate",
        margin=0.0,
        name="rotating arms remain inside fixed frame footprint",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({spin: math.pi / 3.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            outer_elem="base_plate",
            margin=0.0,
            name="rotor stays inside frame while turning",
        )
        turned_pos = ctx.part_world_position(rotor)
    ctx.check(
        "continuous joint rotates about fixed center",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
