from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="pedestal_telescoping_arm")

    cast_iron = Material("dark_cast_iron", rgba=(0.07, 0.075, 0.08, 1.0))
    black_oxide = Material("black_oxide", rgba=(0.015, 0.017, 0.02, 1.0))
    machined_steel = Material("machined_steel", rgba=(0.56, 0.58, 0.58, 1.0))
    satin_aluminum = Material("satin_aluminum", rgba=(0.68, 0.70, 0.68, 1.0))
    dark_rail = Material("dark_linear_rail", rgba=(0.10, 0.11, 0.12, 1.0))
    wear_pad = Material("bronze_wear_pad", rgba=(0.70, 0.48, 0.22, 1.0))

    # Fixed pedestal: a compact floor-mounted bearing stack.  The upper cap is
    # the thrust surface for the rotary carrier above it.
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.28, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_iron,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.075, length=0.505),
        origin=Origin(xyz=(0.0, 0.0, 0.312)),
        material=cast_iron,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.126, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        material=machined_steel,
        name="bearing_cap",
    )
    pedestal.visual(
        Cylinder(radius=0.090, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.565)),
        material=black_oxide,
        name="seal_ring",
    )
    for i, (x, y) in enumerate(
        ((0.18, 0.12), (-0.18, 0.12), (-0.18, -0.12), (0.18, -0.12))
    ):
        pedestal.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, y, 0.064)),
            material=black_oxide,
            name=f"anchor_bolt_{i}",
        )

    # Rotating carrier: one rigid root housing and one straight rectangular
    # guide beam.  The beam is built as a box-section sleeve with a real open
    # cavity, so the prismatic nose is visibly guided rather than hinged.
    carrier = model.part("carrier")
    carrier.visual(
        Cylinder(radius=0.128, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=machined_steel,
        name="rotary_hub",
    )
    carrier.visual(
        Cylinder(radius=0.056, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=black_oxide,
        name="hub_core",
    )
    carrier.visual(
        Box((0.220, 0.235, 0.130)),
        origin=Origin(xyz=(0.120, 0.0, 0.085)),
        material=satin_aluminum,
        name="root_housing",
    )

    beam_len = 0.940
    beam_cx = 0.560
    carrier.visual(
        Box((beam_len, 0.196, 0.028)),
        origin=Origin(xyz=(beam_cx, 0.0, 0.144)),
        material=satin_aluminum,
        name="beam_top_wall",
    )
    carrier.visual(
        Box((beam_len, 0.196, 0.028)),
        origin=Origin(xyz=(beam_cx, 0.0, 0.036)),
        material=satin_aluminum,
        name="beam_bottom_wall",
    )
    carrier.visual(
        Box((beam_len, 0.026, 0.128)),
        origin=Origin(xyz=(beam_cx, 0.0865, 0.090)),
        material=satin_aluminum,
        name="guide_wall_0",
    )
    carrier.visual(
        Box((beam_len, 0.026, 0.128)),
        origin=Origin(xyz=(beam_cx, -0.0865, 0.090)),
        material=satin_aluminum,
        name="guide_wall_1",
    )
    carrier.visual(
        Box((0.042, 0.210, 0.158)),
        origin=Origin(xyz=(0.100, 0.0, 0.090)),
        material=machined_steel,
        name="root_end_cap",
    )
    carrier.visual(
        Box((0.046, 0.210, 0.030)),
        origin=Origin(xyz=(1.035, 0.0, 0.145)),
        material=machined_steel,
        name="nose_end_cap_top",
    )
    carrier.visual(
        Box((0.046, 0.210, 0.030)),
        origin=Origin(xyz=(1.035, 0.0, 0.035)),
        material=machined_steel,
        name="nose_end_cap_bottom",
    )
    carrier.visual(
        Box((0.046, 0.028, 0.146)),
        origin=Origin(xyz=(1.035, 0.0875, 0.090)),
        material=machined_steel,
        name="nose_end_cap_0",
    )
    carrier.visual(
        Box((0.046, 0.028, 0.146)),
        origin=Origin(xyz=(1.035, -0.0875, 0.090)),
        material=machined_steel,
        name="nose_end_cap_1",
    )
    carrier.visual(
        Box((0.620, 0.022, 0.018)),
        origin=Origin(xyz=(0.610, 0.090, 0.158)),
        material=dark_rail,
        name="top_rib_0",
    )
    carrier.visual(
        Box((0.620, 0.022, 0.018)),
        origin=Origin(xyz=(0.610, -0.090, 0.158)),
        material=dark_rail,
        name="top_rib_1",
    )

    # Sliding nose: the long hidden bar remains captured in the beam sleeve at
    # full extension.  Bronze shoes touch the guide walls and top/bottom ways,
    # making the mounting path visible without allowing the stage to clip.
    nose = model.part("nose")
    nose.visual(
        Box((0.870, 0.124, 0.065)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        material=dark_rail,
        name="slider_bar",
    )
    nose.visual(
        Box((0.730, 0.012, 0.034)),
        origin=Origin(xyz=(-0.065, 0.0675, 0.0)),
        material=wear_pad,
        name="side_shoe_0",
    )
    nose.visual(
        Box((0.730, 0.012, 0.034)),
        origin=Origin(xyz=(-0.065, -0.0675, 0.0)),
        material=wear_pad,
        name="side_shoe_1",
    )
    nose.visual(
        Box((0.730, 0.070, 0.008)),
        origin=Origin(xyz=(-0.065, 0.0, 0.036)),
        material=wear_pad,
        name="upper_shoe",
    )
    nose.visual(
        Box((0.730, 0.070, 0.008)),
        origin=Origin(xyz=(-0.065, 0.0, -0.036)),
        material=wear_pad,
        name="lower_shoe",
    )
    nose.visual(
        Box((0.090, 0.100, 0.080)),
        origin=Origin(xyz=(0.390, 0.0, 0.0)),
        material=machined_steel,
        name="nose_collar",
    )
    nose.visual(
        Box((0.085, 0.080, 0.060)),
        origin=Origin(xyz=(0.430, 0.0, 0.0)),
        material=machined_steel,
        name="pad_stem",
    )
    nose.visual(
        Box((0.036, 0.240, 0.140)),
        origin=Origin(xyz=(0.485, 0.0, 0.0)),
        material=machined_steel,
        name="tool_pad",
    )

    model.articulation(
        "root_joint",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.6225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-1.70, upper=1.70),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=nose,
        origin=Origin(xyz=(0.780, 0.0, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.320),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    carrier = object_model.get_part("carrier")
    nose = object_model.get_part("nose")
    root_joint = object_model.get_articulation("root_joint")
    nose_slide = object_model.get_articulation("nose_slide")

    # The rotary carrier sits on the pedestal thrust cap without penetrating it.
    ctx.expect_gap(
        carrier,
        pedestal,
        axis="z",
        positive_elem="rotary_hub",
        negative_elem="bearing_cap",
        min_gap=0.0,
        max_gap=0.003,
        name="rotary hub seats on bearing cap",
    )

    # The nose stage is a retained prismatic slider, not a second hinged link:
    # it stays captured inside the fixed beam at both ends of travel.
    ctx.expect_overlap(
        nose,
        carrier,
        axes="x",
        elem_a="slider_bar",
        elem_b="beam_top_wall",
        min_overlap=0.55,
        name="collapsed nose remains inserted",
    )
    ctx.expect_gap(
        carrier,
        nose,
        axis="y",
        positive_elem="guide_wall_0",
        negative_elem="side_shoe_0",
        max_penetration=0.00005,
        max_gap=0.001,
        name="positive side guide shoe runs in wall",
    )
    ctx.expect_gap(
        nose,
        carrier,
        axis="y",
        positive_elem="side_shoe_1",
        negative_elem="guide_wall_1",
        max_penetration=0.00005,
        max_gap=0.001,
        name="negative side guide shoe runs in wall",
    )
    ctx.expect_gap(
        carrier,
        nose,
        axis="z",
        positive_elem="beam_top_wall",
        negative_elem="upper_shoe",
        max_penetration=0.00005,
        max_gap=0.001,
        name="upper slide shoe clears top way",
    )
    ctx.expect_gap(
        nose,
        carrier,
        axis="z",
        positive_elem="lower_shoe",
        negative_elem="beam_bottom_wall",
        max_penetration=0.00005,
        max_gap=0.001,
        name="lower slide shoe clears bottom way",
    )

    rest_pos = ctx.part_world_position(nose)
    with ctx.pose({nose_slide: 0.320}):
        ctx.expect_overlap(
            nose,
            carrier,
            axes="x",
            elem_a="slider_bar",
            elem_b="beam_top_wall",
            min_overlap=0.25,
            name="extended nose keeps retained insertion",
        )
        ctx.expect_gap(
            carrier,
            nose,
            axis="y",
            positive_elem="guide_wall_0",
            negative_elem="side_shoe_0",
            max_penetration=0.00005,
            max_gap=0.001,
            name="extended positive guide stays clear",
        )
        ctx.expect_gap(
            nose,
            carrier,
            axis="y",
            positive_elem="side_shoe_1",
            negative_elem="guide_wall_1",
            max_penetration=0.00005,
            max_gap=0.001,
            name="extended negative guide stays clear",
        )
        extended_pos = ctx.part_world_position(nose)
    ctx.check(
        "nose slide extends along beam axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.300,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_aabb = ctx.part_element_world_aabb(carrier, elem="beam_top_wall")
    with ctx.pose({root_joint: 1.0}):
        rotated_aabb = ctx.part_element_world_aabb(carrier, elem="beam_top_wall")
    if rest_aabb is not None and rotated_aabb is not None:
        rest_y = 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
        rotated_y = 0.5 * (rotated_aabb[0][1] + rotated_aabb[1][1])
        ctx.check(
            "root joint rotates straight beam",
            rotated_y > rest_y + 0.35,
            details=f"rest_y={rest_y}, rotated_y={rotated_y}",
        )
    else:
        ctx.fail("root joint rotates straight beam", "beam_top_wall AABB unavailable")

    return ctx.report()


object_model = build_object_model()
