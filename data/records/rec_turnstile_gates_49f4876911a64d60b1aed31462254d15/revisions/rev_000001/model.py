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

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.62, 0.62, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))

    frame = model.part("frame")

    # A broad floor plate makes the fixed cage, the posts, and the single central
    # pedestal one physically connected support assembly.
    frame.visual(
        Box((2.35, 2.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="base_plate",
    )

    # The one fixed support directly under the rotating stage.
    frame.visual(
        Cylinder(radius=0.075, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=galvanized,
        name="central_support",
    )
    frame.visual(
        Cylinder(radius=0.135, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.9275)),
        material=dark_steel,
        name="bearing_seat",
    )

    # Four corner posts and a rectangular overhead frame keep the rotor visibly
    # inside a fixed gate structure while leaving generous sweep clearance.
    post_x = 1.07
    post_y = 0.92
    for ix, x in enumerate((-post_x, post_x)):
        for iy, y in enumerate((-post_y, post_y)):
            frame.visual(
                Cylinder(radius=0.045, length=1.40),
                origin=Origin(xyz=(x, y, 0.75)),
                material=galvanized,
                name=f"corner_post_{ix}_{iy}",
            )

    for y in (-post_y, post_y):
        frame.visual(
            Cylinder(radius=0.035, length=2.22),
            origin=Origin(xyz=(0.0, y, 1.45), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"top_side_rail_{'front' if y < 0 else 'rear'}",
        )
        frame.visual(
            Cylinder(radius=0.030, length=2.22),
            origin=Origin(xyz=(0.0, y, 0.72), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"low_side_rail_{'front' if y < 0 else 'rear'}",
        )

    for x in (-post_x, post_x):
        frame.visual(
            Cylinder(radius=0.035, length=1.90),
            origin=Origin(xyz=(x, 0.0, 1.45), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"top_end_rail_{'west' if x < 0 else 'east'}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.13, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_steel,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1525)),
        material=galvanized,
        name="top_cap",
    )
    rotor.visual(
        Cylinder(radius=0.10, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=yellow,
        name="index_cap",
    )

    arm_length = 0.76
    arm_center_radius = 0.49
    arm_outer_radius = arm_center_radius + arm_length / 2.0
    arm_z = 0.085
    for i in range(3):
        angle = i * 2.0 * math.pi / 3.0
        cx = arm_center_radius * math.cos(angle)
        cy = arm_center_radius * math.sin(angle)
        ex = arm_outer_radius * math.cos(angle)
        ey = arm_outer_radius * math.sin(angle)
        rotor.visual(
            Cylinder(radius=0.032, length=arm_length),
            origin=Origin(xyz=(cx, cy, arm_z), rpy=(0.0, math.pi / 2.0, angle)),
            material=galvanized,
            name=f"arm_{i}",
        )
        rotor.visual(
            Sphere(radius=0.046),
            origin=Origin(xyz=(ex, ey, arm_z)),
            material=rubber,
            name=f"arm_tip_{i}",
        )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("hub_spin")

    ctx.check(
        "hub joint is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {spin.articulation_type}",
    )
    ctx.check(
        "hub joint uses vertical axis",
        tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="hub_drum",
        elem_b="bearing_seat",
        contact_tol=0.001,
        name="rotor hub rests on central bearing seat",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="arm_0",
        outer_elem="base_plate",
        margin=0.02,
        name="straight arm remains inside fixed frame footprint",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({spin: 2.0 * math.pi / 3.0}):
        turned_pos = ctx.part_world_position(rotor)
        ctx.expect_contact(
            rotor,
            frame,
            elem_a="hub_drum",
            elem_b="bearing_seat",
            contact_tol=0.001,
            name="rotated hub stays seated on central support",
        )

    ctx.check(
        "continuous spin keeps hub centered",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
