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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_carriage_spindle")

    dark_anodized = Material("dark_anodized_aluminum", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    black_polymer = Material("black_polymer", rgba=(0.015, 0.015, 0.018, 1.0))
    warm_steel = Material("tool_steel", rgba=(0.78, 0.74, 0.66, 1.0))
    red_mark = Material("red_orientation_mark", rgba=(0.9, 0.04, 0.03, 1.0))

    rail_base = model.part("rail_base")
    rail_base.visual(
        Box((1.12, 0.18, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_anodized,
        name="base_prism",
    )
    rail_base.visual(
        Cylinder(radius=0.030, length=1.00),
        origin=Origin(xyz=(0.0, 0.0, 0.16), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="guide_rail",
    )
    for i, x in enumerate((-0.48, 0.48)):
        rail_base.visual(
            Box((0.065, 0.12, 0.145)),
            origin=Origin(xyz=(x, 0.0, 0.109)),
            material=dark_anodized,
            name=f"end_support_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.18, 0.18, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=black_polymer,
        name="top_bridge",
    )
    carriage.visual(
        Box((0.145, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=brushed_steel,
        name="top_wear_pad",
    )
    carriage.visual(
        Box((0.040, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=black_polymer,
        name="pad_web",
    )
    carriage.visual(
        Box((0.17, 0.030, 0.120)),
        origin=Origin(xyz=(0.0, 0.069, 0.0)),
        material=black_polymer,
        name="side_cheek_0",
    )
    carriage.visual(
        Box((0.17, 0.030, 0.120)),
        origin=Origin(xyz=(0.0, -0.069, 0.0)),
        material=black_polymer,
        name="side_cheek_1",
    )
    carriage.visual(
        Box((0.14, 0.030, 0.120)),
        origin=Origin(xyz=(0.0, 0.094, 0.035)),
        material=black_polymer,
        name="bearing_plate",
    )
    carriage.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.100, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="front_bearing_ring",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="spindle_flange",
    )
    spindle.visual(
        Cylinder(radius=0.033, length=0.130),
        origin=Origin(xyz=(0.0, 0.083, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="motor_body",
    )
    spindle.visual(
        Box((0.012, 0.080, 0.006)),
        origin=Origin(xyz=(0.0, 0.085, 0.034)),
        material=red_mark,
        name="orientation_marker",
    )
    spindle.visual(
        Cylinder(radius=0.019, length=0.045),
        origin=Origin(xyz=(0.0, 0.170, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="tool_chuck",
    )
    spindle.visual(
        Cylinder(radius=0.006, length=0.074),
        origin=Origin(xyz=(0.0, 0.227, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_steel,
        name="end_mill",
    )

    model.articulation(
        "linear_slide",
        ArticulationType.PRISMATIC,
        parent=rail_base,
        child=carriage,
        origin=Origin(xyz=(-0.30, 0.0, 0.16)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.60),
    )
    model.articulation(
        "spindle_joint",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.109, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=30.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_base = object_model.get_part("rail_base")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    linear_slide = object_model.get_articulation("linear_slide")
    spindle_joint = object_model.get_articulation("spindle_joint")

    ctx.expect_overlap(
        carriage,
        rail_base,
        axes="x",
        min_overlap=0.12,
        elem_a="top_bridge",
        elem_b="guide_rail",
        name="carriage overlaps the rail length at rest",
    )
    ctx.expect_gap(
        carriage,
        rail_base,
        axis="z",
        min_gap=0.015,
        max_gap=0.035,
        positive_elem="top_bridge",
        negative_elem="guide_rail",
        name="carriage bridge clears the round rail",
    )
    ctx.expect_contact(
        carriage,
        rail_base,
        elem_a="top_wear_pad",
        elem_b="guide_rail",
        contact_tol=1e-5,
        name="wear pad rides directly on the rail",
    )
    ctx.expect_gap(
        spindle,
        carriage,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="spindle_flange",
        negative_elem="bearing_plate",
        name="spindle flange seats on carriage bearing plate",
    )

    rest_pos = ctx.part_world_position(spindle)
    with ctx.pose({linear_slide: 0.60}):
        extended_pos = ctx.part_world_position(spindle)
        ctx.expect_overlap(
            carriage,
            rail_base,
            axes="x",
            min_overlap=0.12,
            elem_a="top_bridge",
            elem_b="guide_rail",
            name="carriage remains captured on the rail at full travel",
        )

    ctx.check(
        "prismatic carriage carries spindle along the rail",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.55,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "spindle has independent revolute motion",
        getattr(spindle_joint, "articulation_type", None) == ArticulationType.REVOLUTE
        and spindle_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={getattr(spindle_joint, 'articulation_type', None)}, axis={spindle_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
