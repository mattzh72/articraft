from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_swing_slide")

    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    zinc = model.material("brushed_zinc", rgba=(0.55, 0.58, 0.58, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.78, 0.76, 0.70, 1.0))
    slider_blue = model.material("blue_slider", rgba=(0.12, 0.27, 0.56, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.018, 0.220, 0.340)),
        origin=Origin(xyz=(-0.065, 0.0, 0.0)),
        material=dark_steel,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.095, 0.105, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, 0.055)),
        material=dark_steel,
        name="upper_lug",
    )
    side_plate.visual(
        Box((0.095, 0.105, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, -0.055)),
        material=dark_steel,
        name="lower_lug",
    )
    side_plate.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_steel,
        name="pivot_pin",
    )
    side_plate.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=pin_steel,
        name="pin_top_cap",
    )
    side_plate.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=pin_steel,
        name="pin_bottom_cap",
    )
    for index, (y, z) in enumerate(((-0.075, -0.115), (0.075, -0.115), (-0.075, 0.115), (0.075, 0.115))):
        side_plate.visual(
            Cylinder(radius=0.012, length=0.007),
            origin=Origin(xyz=(-0.053, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=pin_steel,
            name=f"wall_bolt_{index}",
        )

    arm = model.part("arm")
    hub_outer = cq.Workplane("XY").circle(0.035).extrude(0.092)
    hub_hole = cq.Workplane("XY").circle(0.012).extrude(0.110).translate((0.0, 0.0, -0.009))
    hub_ring = (
        hub_outer.cut(hub_hole)
        .translate((0.0, 0.0, -0.046))
    )
    arm.visual(
        mesh_from_cadquery(hub_ring, "pivot_hub"),
        origin=Origin(),
        material=zinc,
        name="pivot_hub",
    )
    arm.visual(
        Box((0.440, 0.035, 0.014)),
        origin=Origin(xyz=(0.235, 0.0, 0.033)),
        material=zinc,
        name="arm_web",
    )
    arm.visual(
        Box((0.240, 0.074, 0.008)),
        origin=Origin(xyz=(0.550, 0.0, 0.024)),
        material=dark_steel,
        name="sleeve_top",
    )
    arm.visual(
        Box((0.240, 0.074, 0.008)),
        origin=Origin(xyz=(0.550, 0.0, -0.024)),
        material=dark_steel,
        name="sleeve_bottom",
    )
    arm.visual(
        Box((0.240, 0.008, 0.056)),
        origin=Origin(xyz=(0.550, 0.037, 0.0)),
        material=dark_steel,
        name="sleeve_side_pos_y",
    )
    arm.visual(
        Box((0.240, 0.008, 0.056)),
        origin=Origin(xyz=(0.550, -0.037, 0.0)),
        material=dark_steel,
        name="sleeve_side_neg_y",
    )
    arm.visual(
        Box((0.055, 0.074, 0.014)),
        origin=Origin(xyz=(0.445, 0.0, 0.032)),
        material=dark_steel,
        name="sleeve_bridge",
    )

    extension = model.part("extension")
    extension.visual(
        Box((0.400, 0.040, 0.026)),
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material=slider_blue,
        name="slide_bar",
    )
    extension.visual(
        Box((0.160, 0.013, 0.010)),
        origin=Origin(xyz=(0.090, 0.0265, 0.0)),
        material=rubber,
        name="side_pad_pos_y",
    )
    extension.visual(
        Box((0.160, 0.013, 0.010)),
        origin=Origin(xyz=(0.090, -0.0265, 0.0)),
        material=rubber,
        name="side_pad_neg_y",
    )
    extension.visual(
        Box((0.055, 0.074, 0.046)),
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        material=slider_blue,
        name="front_mount",
    )
    extension.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.390, -0.020, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="front_socket_0",
    )
    extension.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.390, 0.020, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="front_socket_1",
    )

    model.articulation(
        "side_plate_to_arm",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "arm_to_extension",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=extension,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.140),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    side_plate = object_model.get_part("side_plate")
    arm = object_model.get_part("arm")
    extension = object_model.get_part("extension")
    swing = object_model.get_articulation("side_plate_to_arm")
    slide = object_model.get_articulation("arm_to_extension")

    ctx.check(
        "root joint is revolute",
        swing.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={swing.articulation_type}",
    )
    ctx.check(
        "carried stage joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    ctx.expect_contact(
        side_plate,
        arm,
        elem_a="upper_lug",
        elem_b="pivot_hub",
        contact_tol=0.001,
        name="upper lug captures rotating hub",
    )
    ctx.expect_contact(
        arm,
        side_plate,
        elem_a="pivot_hub",
        elem_b="lower_lug",
        contact_tol=0.001,
        name="lower lug captures rotating hub",
    )
    ctx.expect_within(
        side_plate,
        arm,
        axes="xy",
        inner_elem="pivot_pin",
        outer_elem="pivot_hub",
        margin=0.0,
        name="pivot pin is centered through hub",
    )
    ctx.expect_within(
        extension,
        arm,
        axes="yz",
        inner_elem="slide_bar",
        margin=0.0,
        name="slide bar stays inside sleeve cross section",
    )
    ctx.expect_gap(
        arm,
        extension,
        axis="z",
        positive_elem="sleeve_top",
        negative_elem="slide_bar",
        min_gap=0.004,
        name="slide clears top of sleeve",
    )
    ctx.expect_gap(
        extension,
        arm,
        axis="z",
        positive_elem="slide_bar",
        negative_elem="sleeve_bottom",
        min_gap=0.004,
        name="slide clears bottom of sleeve",
    )
    ctx.expect_contact(
        extension,
        arm,
        elem_a="side_pad_pos_y",
        elem_b="sleeve_side_pos_y",
        contact_tol=0.001,
        name="positive side pad rides in sleeve",
    )
    ctx.expect_contact(
        extension,
        arm,
        elem_a="side_pad_neg_y",
        elem_b="sleeve_side_neg_y",
        contact_tol=0.001,
        name="negative side pad rides in sleeve",
    )
    ctx.expect_overlap(
        extension,
        arm,
        axes="x",
        elem_a="slide_bar",
        elem_b="sleeve_top",
        min_overlap=0.180,
        name="retracted slide remains deeply inserted",
    )

    retracted_pos = ctx.part_world_position(extension)
    with ctx.pose({slide: 0.140}):
        ctx.expect_overlap(
            extension,
            arm,
            axes="x",
            elem_a="slide_bar",
            elem_b="sleeve_top",
            min_overlap=0.080,
            name="extended slide keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(extension)
    ctx.check(
        "prismatic stage extends forward",
        retracted_pos is not None
        and extended_pos is not None
        and extended_pos[0] > retracted_pos[0] + 0.120,
        details=f"retracted={retracted_pos}, extended={extended_pos}",
    )

    straight_pos = ctx.part_world_position(extension)
    with ctx.pose({swing: 0.75}):
        swung_pos = ctx.part_world_position(extension)
    ctx.check(
        "arm swings stage away from side plate plane",
        straight_pos is not None
        and swung_pos is not None
        and swung_pos[1] > straight_pos[1] + 0.250,
        details=f"straight={straight_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
