from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="framed_louver_bank")

    dark_anodized = model.material("dark_anodized", rgba=(0.05, 0.06, 0.065, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.66, 0.66, 1.0))
    rubber_foot = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    frame_width = 1.20
    frame_height = 1.05
    frame_depth = 0.14
    post_width = 0.10
    rail_height = 0.10
    side_x = frame_width / 2.0 - post_width / 2.0

    frame = model.part("frame")
    frame.visual(
        Box((post_width, frame_depth, frame_height)),
        origin=Origin(xyz=(-side_x, 0.0, frame_height / 2.0)),
        material=dark_anodized,
        name="side_post_0",
    )
    frame.visual(
        Box((post_width, frame_depth, frame_height)),
        origin=Origin(xyz=(side_x, 0.0, frame_height / 2.0)),
        material=dark_anodized,
        name="side_post_1",
    )
    frame.visual(
        Box((frame_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, rail_height / 2.0)),
        material=dark_anodized,
        name="bottom_rail",
    )
    frame.visual(
        Box((frame_width, frame_depth, rail_height)),
        origin=Origin(xyz=(0.0, 0.0, frame_height - rail_height / 2.0)),
        material=dark_anodized,
        name="top_rail",
    )
    for index, x in enumerate((-side_x, side_x)):
        frame.visual(
            Box((0.17, 0.24, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.0175)),
            material=rubber_foot,
            name=f"foot_{index}",
        )

    slat_count = 6
    slat_span = 0.98
    slat_chord = 0.115
    slat_thickness = 0.018
    axle_length = 1.18
    axle_radius = 0.012
    pivot_z_values = [0.20 + i * 0.13 for i in range(slat_count)]
    rest_slat_angle = -0.35

    for slat_index, z in enumerate(pivot_z_values):
        for side_index, x in enumerate((-side_x, side_x)):
            frame.visual(
                Cylinder(radius=0.030, length=0.035),
                origin=Origin(
                    xyz=(x, -frame_depth / 2.0 - 0.012, z),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=brushed_aluminum,
                name=f"bearing_{slat_index}_{side_index}",
            )

        slat = model.part(f"slat_{slat_index}")
        slat.visual(
            Box((slat_span, slat_chord, slat_thickness)),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(rest_slat_angle, 0.0, 0.0)),
            material=brushed_aluminum,
            name="blade",
        )
        slat.visual(
            Cylinder(radius=axle_radius, length=axle_length),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_aluminum,
            name="pivot_axle",
        )

        model.articulation(
            f"frame_to_slat_{slat_index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-0.45, upper=0.75),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    joints = [object_model.get_articulation(f"frame_to_slat_{i}") for i in range(6)]

    ctx.check(
        "six independent vane pivots",
        len(joints) == 6 and len({joint.child for joint in joints}) == 6 and all(joint.mimic is None for joint in joints),
        details=f"children={[joint.child for joint in joints]}",
    )

    for index, joint in enumerate(joints):
        slat = object_model.get_part(f"slat_{index}")
        limits = joint.motion_limits
        axis_is_long = tuple(round(value, 6) for value in joint.axis) == (1.0, 0.0, 0.0)
        ctx.check(
            f"slat_{index} has a bounded long-axis revolute joint",
            joint.articulation_type == ArticulationType.REVOLUTE
            and axis_is_long
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

        for side_name in ("side_post_0", "side_post_1"):
            ctx.allow_overlap(
                frame,
                slat,
                elem_a=side_name,
                elem_b="pivot_axle",
                reason="Each slat axle is intentionally captured inside the side-post bearing line.",
            )
            ctx.expect_overlap(
                frame,
                slat,
                axes="x",
                elem_a=side_name,
                elem_b="pivot_axle",
                min_overlap=0.040,
                name=f"slat_{index} axle is retained in {side_name}",
            )
            ctx.expect_within(
                slat,
                frame,
                axes="yz",
                inner_elem="pivot_axle",
                outer_elem=side_name,
                margin=0.0,
                name=f"slat_{index} axle is centered in {side_name} bearing height",
            )

    return ctx.report()


object_model = build_object_model()
