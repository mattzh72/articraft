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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# Object frame: +X is the pull-out front direction, +Y spans the width, +Z is up.
LOWER_DEPTH = 0.220
LOWER_WIDTH = 0.310
LOWER_HEIGHT = 0.285

HEAD_DEPTH = 0.232
HEAD_WIDTH = 0.320
HEAD_HEIGHT = 0.115
TOTAL_HEIGHT = LOWER_HEIGHT + HEAD_HEIGHT

OPENING_HEIGHT = 0.240
OPENING_BOTTOM = 0.022

BIN_DEPTH = 0.188
BIN_WIDTH = 0.266
BIN_HEIGHT = 0.236
BIN_BOTTOM = 0.018

def _waste_bin_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BIN_DEPTH, BIN_WIDTH, BIN_HEIGHT)
        .translate((-BIN_DEPTH * 0.5, 0.0, BIN_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.008)
    )

    inner_depth = BIN_DEPTH - 0.016 - 0.008
    inner_width = BIN_WIDTH - 0.010
    inner = (
        cq.Workplane("XY")
        .box(inner_depth, inner_width, BIN_HEIGHT)
        .translate((-(0.016 + inner_depth * 0.5), 0.0, 0.005 + BIN_HEIGHT * 0.5))
    )
    grip = (
        cq.Workplane("XY")
        .box(0.012, 0.086, 0.020)
        .translate((-0.006, 0.0, BIN_HEIGHT * 0.72))
    )
    return outer.cut(inner).cut(grip)


def _button_bezel_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.013).circle(0.0092).extrude(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_office_shredder")

    body_grey = model.material("body_grey", rgba=(0.45, 0.47, 0.49, 1.0))
    head_black = model.material("head_black", rgba=(0.12, 0.12, 0.13, 1.0))
    bin_grey = model.material("bin_grey", rgba=(0.33, 0.35, 0.37, 1.0))
    strip_black = model.material("strip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    slider_black = model.material("slider_black", rgba=(0.18, 0.18, 0.19, 1.0))
    button_red = model.material("button_red", rgba=(0.57, 0.18, 0.16, 1.0))
    button_grey = model.material("button_grey", rgba=(0.72, 0.72, 0.73, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.75, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((LOWER_DEPTH, LOWER_WIDTH, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=body_grey,
        name="floor_pan",
    )
    housing.visual(
        Box((LOWER_DEPTH, 0.019, LOWER_HEIGHT - 0.018)),
        origin=Origin(xyz=(0.0, 0.1455, 0.1515)),
        material=body_grey,
        name="side_wall_0",
    )
    housing.visual(
        Box((LOWER_DEPTH, 0.019, LOWER_HEIGHT - 0.018)),
        origin=Origin(xyz=(0.0, -0.1455, 0.1515)),
        material=body_grey,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.020, LOWER_WIDTH - 0.038, LOWER_HEIGHT - 0.018)),
        origin=Origin(xyz=(-0.100, 0.0, 0.1515)),
        material=body_grey,
        name="back_wall",
    )
    housing.visual(
        Box((0.020, LOWER_WIDTH - 0.038, LOWER_HEIGHT - (OPENING_BOTTOM + OPENING_HEIGHT))),
        origin=Origin(
            xyz=(
                0.100,
                0.0,
                OPENING_BOTTOM
                + OPENING_HEIGHT
                + (LOWER_HEIGHT - (OPENING_BOTTOM + OPENING_HEIGHT)) * 0.5,
            )
        ),
        material=body_grey,
        name="front_rail",
    )
    housing.visual(
        Box((HEAD_DEPTH, 0.016, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.152, LOWER_HEIGHT + HEAD_HEIGHT * 0.5)),
        material=head_black,
        name="head_cheek_0",
    )
    housing.visual(
        Box((HEAD_DEPTH, 0.016, HEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.152, LOWER_HEIGHT + HEAD_HEIGHT * 0.5)),
        material=head_black,
        name="head_cheek_1",
    )
    housing.visual(
        Box((0.028, HEAD_WIDTH - 0.032, HEAD_HEIGHT)),
        origin=Origin(xyz=(-0.102, 0.0, LOWER_HEIGHT + HEAD_HEIGHT * 0.5)),
        material=head_black,
        name="head_back",
    )
    housing.visual(
        Box((0.106, HEAD_WIDTH - 0.032, 0.020)),
        origin=Origin(xyz=(-0.053, 0.0, TOTAL_HEIGHT - 0.010)),
        material=head_black,
        name="control_deck",
    )
    housing.visual(
        Box((0.088, HEAD_WIDTH - 0.032, 0.020)),
        origin=Origin(xyz=(0.062, 0.0, TOTAL_HEIGHT - 0.010)),
        material=head_black,
        name="slot_front_deck",
    )
    housing.visual(
        Box((0.018, HEAD_WIDTH - 0.032, 0.034)),
        origin=Origin(xyz=(0.107, 0.0, LOWER_HEIGHT + 0.051)),
        material=head_black,
        name="front_lip",
    )

    waste_bin = model.part("waste_bin")
    waste_bin.visual(
        mesh_from_cadquery(_waste_bin_shape(), "shredder_waste_bin"),
        material=bin_grey,
        name="bin_shell",
    )

    front_drum = model.part("front_drum")
    front_drum.visual(
        Cylinder(radius=0.0065, length=0.288),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="drum_core",
    )
    for index in range(13):
        y = -0.114 + index * 0.019
        front_drum.visual(
            Cylinder(radius=0.0098, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=steel,
            name=f"cutter_{index}",
        )

    rear_drum = model.part("rear_drum")
    rear_drum.visual(
        Cylinder(radius=0.0065, length=0.288),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="drum_core",
    )
    for index in range(13):
        y = -0.114 + index * 0.019
        rear_drum.visual(
            Cylinder(radius=0.0098, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=steel,
            name=f"cutter_{index}",
        )

    control_strip = model.part("control_strip")
    bezel_ring = mesh_from_cadquery(_button_bezel_shape(), "button_bezel_ring")
    control_strip.visual(
        Box((0.095, 0.112, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=strip_black,
        name="base_plate",
    )
    control_strip.visual(
        Box((0.004, 0.068, 0.008)),
        origin=Origin(xyz=(-0.031, -0.018, 0.010)),
        material=strip_black,
        name="guide_rail_0",
    )
    control_strip.visual(
        Box((0.004, 0.068, 0.008)),
        origin=Origin(xyz=(-0.013, -0.018, 0.010)),
        material=strip_black,
        name="guide_rail_1",
    )
    control_strip.visual(
        bezel_ring,
        origin=Origin(xyz=(0.020, 0.018, 0.006)),
        material=strip_black,
        name="bezel_0",
    )
    control_strip.visual(
        bezel_ring,
        origin=Origin(xyz=(0.020, 0.050, 0.006)),
        material=strip_black,
        name="bezel_1",
    )

    control_slider = model.part("control_slider")
    control_slider.visual(
        Box((0.012, 0.016, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=slider_black,
        name="slider_foot",
    )
    control_slider.visual(
        Box((0.018, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=slider_black,
        name="slider_cap",
    )
    control_slider.visual(
        Box((0.008, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=slider_black,
        name="slider_ridge",
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Cylinder(radius=0.0092, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=button_red,
        name="button_cap",
    )

    button_1 = model.part("button_1")
    button_1.visual(
        Cylinder(radius=0.0092, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=button_grey,
        name="button_cap",
    )

    model.articulation(
        "housing_to_waste_bin",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=waste_bin,
        origin=Origin(xyz=(LOWER_DEPTH * 0.5, 0.0, BIN_BOTTOM)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.18,
            lower=0.0,
            upper=0.125,
        ),
    )

    model.articulation(
        "housing_to_front_drum",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=front_drum,
        origin=Origin(xyz=(0.019, 0.0, 0.365)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=40.0,
        ),
    )
    model.articulation(
        "housing_to_rear_drum",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rear_drum,
        origin=Origin(xyz=(-0.003, 0.0, 0.365)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=40.0,
        ),
    )
    model.articulation(
        "housing_to_control_strip",
        ArticulationType.FIXED,
        parent=housing,
        child=control_strip,
        origin=Origin(xyz=(-0.053, 0.0, TOTAL_HEIGHT)),
    )
    model.articulation(
        "control_strip_to_control_slider",
        ArticulationType.PRISMATIC,
        parent=control_strip,
        child=control_slider,
        origin=Origin(xyz=(-0.022, -0.018, 0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.10,
            lower=-0.018,
            upper=0.018,
        ),
    )
    model.articulation(
        "control_strip_to_button_0",
        ArticulationType.PRISMATIC,
        parent=control_strip,
        child=button_0,
        origin=Origin(xyz=(0.020, 0.018, 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0015,
        ),
    )
    model.articulation(
        "control_strip_to_button_1",
        ArticulationType.PRISMATIC,
        parent=control_strip,
        child=button_1,
        origin=Origin(xyz=(0.020, 0.050, 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0015,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    waste_bin = object_model.get_part("waste_bin")
    bin_slide = object_model.get_articulation("housing_to_waste_bin")
    front_drum = object_model.get_part("front_drum")
    rear_drum = object_model.get_part("rear_drum")
    control_slider = object_model.get_part("control_slider")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    slider_joint = object_model.get_articulation("control_strip_to_control_slider")
    button_0_joint = object_model.get_articulation("control_strip_to_button_0")
    button_1_joint = object_model.get_articulation("control_strip_to_button_1")

    ctx.expect_within(
        waste_bin,
        housing,
        axes="yz",
        elem_a="bin_shell",
        margin=0.010,
        name="waste bin stays centered in the lower body",
    )
    ctx.expect_overlap(
        waste_bin,
        housing,
        axes="x",
        elem_a="bin_shell",
        min_overlap=0.170,
        name="closed waste bin remains deeply inserted",
    )

    rest_pos = ctx.part_world_position(waste_bin)
    with ctx.pose({bin_slide: 0.125}):
        ctx.expect_within(
            waste_bin,
            housing,
            axes="yz",
            elem_a="bin_shell",
            margin=0.010,
            name="extended waste bin stays guided by the housing",
        )
        ctx.expect_overlap(
            waste_bin,
            housing,
            axes="x",
            elem_a="bin_shell",
            min_overlap=0.055,
            name="extended waste bin keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(waste_bin)

    ctx.check(
        "waste bin pulls forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_origin_gap(
        front_drum,
        rear_drum,
        axis="x",
        min_gap=0.018,
        max_gap=0.028,
        name="drums sit on parallel fore-aft axes",
    )
    drum_front_pos = ctx.part_world_position(front_drum)
    drum_rear_pos = ctx.part_world_position(rear_drum)
    ctx.check(
        "drums share one cutter height",
        drum_front_pos is not None
        and drum_rear_pos is not None
        and abs(drum_front_pos[2] - drum_rear_pos[2]) <= 0.002,
        details=f"front={drum_front_pos}, rear={drum_rear_pos}",
    )

    with ctx.pose({slider_joint: slider_joint.motion_limits.lower}):
        slider_lower_pos = ctx.part_world_position(control_slider)
    with ctx.pose({slider_joint: slider_joint.motion_limits.upper}):
        slider_upper_pos = ctx.part_world_position(control_slider)
    ctx.check(
        "control slider travels along its guide",
        slider_lower_pos is not None
        and slider_upper_pos is not None
        and slider_upper_pos[1] > slider_lower_pos[1] + 0.030,
        details=f"lower={slider_lower_pos}, upper={slider_upper_pos}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_0_joint: button_0_joint.motion_limits.upper}):
        button_0_pressed = ctx.part_world_position(button_0)
    ctx.check(
        "button 0 presses downward",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.001,
        details=f"rest={button_0_rest}, pressed={button_0_pressed}",
    )

    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: button_1_joint.motion_limits.upper}):
        button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "button 1 presses downward",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.001,
        details=f"rest={button_1_rest}, pressed={button_1_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
