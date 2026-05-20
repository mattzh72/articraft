from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _hollow_hub_mesh():
    """A short tubular sleeve so the fixed axle can be visibly exposed through it."""
    hub_height = 0.24
    return (
        cq.Workplane("XY")
        .circle(0.130)
        .circle(0.065)
        .extrude(hub_height)
        .translate((0.0, 0.0, -hub_height / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    dark_paint = Material("dark_powder_coat", rgba=(0.035, 0.040, 0.045, 1.0))
    floor_dark = Material("matte_black_floor_steel", rgba=(0.015, 0.017, 0.018, 1.0))
    brushed_steel = Material("brushed_stainless_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    rubber_yellow = Material("safety_yellow_rubber", rgba=(1.0, 0.76, 0.05, 1.0))

    frame = model.part("frame")

    # A light open portal frame: four corner posts, slim top/base rails, and
    # waist-height side rails keep the rotating stage exposed rather than boxed in.
    for y, name in ((-0.98, "floor_rail_0"), (0.98, "floor_rail_1")):
        frame.visual(
            Box((2.18, 0.055, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0275)),
            material=floor_dark,
            name=name,
        )
        frame.visual(
            Box((2.18, 0.055, 0.055)),
            origin=Origin(xyz=(0.0, y, 1.47)),
            material=dark_paint,
            name=name.replace("floor", "top"),
        )
        frame.visual(
            Box((2.18, 0.040, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.84)),
            material=dark_paint,
            name=name.replace("floor", "side_guard"),
        )

    for x, name in ((-1.09, "floor_cross_0"), (1.09, "floor_cross_1")):
        frame.visual(
            Box((0.055, 2.02, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0275)),
            material=floor_dark,
            name=name,
        )
        frame.visual(
            Box((0.055, 2.02, 0.055)),
            origin=Origin(xyz=(x, 0.0, 1.47)),
            material=dark_paint,
            name=name.replace("floor", "top"),
        )

    frame.visual(
        Box((2.18, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=floor_dark,
        name="floor_spine_x",
    )
    frame.visual(
        Box((0.045, 2.02, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=floor_dark,
        name="floor_spine_y",
    )

    post_i = 0
    for x in (-1.09, 1.09):
        for y in (-0.98, 0.98):
            frame.visual(
                Cylinder(radius=0.035, length=1.45),
                origin=Origin(xyz=(x, y, 0.755)),
                material=dark_paint,
                name=f"corner_post_{post_i}",
            )
            post_i += 1

    # Central fixed column and bearing collars.  The rotor sleeve rotates around
    # this exposed axle, with small clearance rather than hidden interpenetration.
    frame.visual(
        Cylinder(radius=0.160, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=floor_dark,
        name="central_base",
    )
    frame.visual(
        Cylinder(radius=0.045, length=1.35),
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        material=brushed_steel,
        name="axle",
    )
    frame.visual(
        Cylinder(radius=0.076, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.905)),
        material=brushed_steel,
        name="lower_bearing",
    )
    frame.visual(
        Cylinder(radius=0.076, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.195)),
        material=brushed_steel,
        name="upper_bearing",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_cadquery(_hollow_hub_mesh(), "hub_shell"),
        origin=Origin(),
        material=brushed_steel,
        name="hub_shell",
    )

    arm_length = 0.80
    arm_start = 0.105
    arm_center = arm_start + arm_length / 2.0
    arm_tip = arm_start + arm_length
    for i in range(3):
        angle = i * 2.0 * math.pi / 3.0
        ca = math.cos(angle)
        sa = math.sin(angle)
        rotor.visual(
            Cylinder(radius=0.032, length=arm_length),
            origin=Origin(
                xyz=(arm_center * ca, arm_center * sa, 0.0),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=brushed_steel,
            name=f"arm_{i}",
        )
        rotor.visual(
            Sphere(radius=0.045),
            origin=Origin(xyz=(arm_tip * ca, arm_tip * sa, 0.0)),
            material=rubber_yellow,
            name=f"end_cap_{i}",
        )

    model.articulation(
        "hub_axis",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    hub_axis = object_model.get_articulation("hub_axis")

    ctx.check(
        "rotor uses a continuous vertical axis",
        hub_axis.articulation_type == ArticulationType.CONTINUOUS
        and tuple(hub_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={hub_axis.articulation_type}, axis={hub_axis.axis}",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        margin=0.005,
        name="three arms stay inside the open frame footprint",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_shell",
        negative_elem="lower_bearing",
        min_gap=0.0,
        max_gap=0.001,
        name="hub is supported just above the lower bearing",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="upper_bearing",
        negative_elem="hub_shell",
        min_gap=0.0,
        max_gap=0.001,
        name="upper bearing captures the hub without overlap",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({hub_axis: math.pi / 3.0}):
        turned_pos = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            margin=0.005,
            name="rotated arms remain inside the frame",
        )
    ctx.check(
        "continuous rotation keeps the hub centered",
        rest_pos is not None
        and turned_pos is not None
        and max(abs(rest_pos[i] - turned_pos[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_pos}, rotated={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
