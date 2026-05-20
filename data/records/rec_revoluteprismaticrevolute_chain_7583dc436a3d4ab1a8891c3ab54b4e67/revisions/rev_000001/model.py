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
    model = ArticulatedObject(name="bridge_backed_wristed_slider")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    blue_frame = model.material("blue_frame", rgba=(0.05, 0.20, 0.42, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    bearing = model.material("black_bearing", rgba=(0.015, 0.016, 0.018, 1.0))
    output_orange = model.material("output_orange", rgba=(0.95, 0.42, 0.08, 1.0))

    hinge_x = 0.06
    hinge_z = 0.46

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.34, 0.56, 0.035)),
        origin=Origin(xyz=(0.04, 0.0, 0.0175)),
        material=dark_steel,
        name="base_plate",
    )
    for y in (-0.215, 0.215):
        rear_support.visual(
            Box((0.055, 0.065, 0.59)),
            origin=Origin(xyz=(hinge_x, y, 0.315)),
            material=dark_steel,
            name=f"upright_{0 if y < 0 else 1}",
        )
    rear_support.visual(
        Box((0.075, 0.49, 0.065)),
        origin=Origin(xyz=(hinge_x, 0.0, 0.610)),
        material=dark_steel,
        name="top_bridge",
    )
    rear_support.visual(
        Box((0.18, 0.055, 0.25)),
        origin=Origin(xyz=(-0.005, -0.215, 0.19), rpy=(0.0, 0.32, 0.0)),
        material=dark_steel,
        name="brace_0",
    )
    rear_support.visual(
        Box((0.18, 0.055, 0.25)),
        origin=Origin(xyz=(-0.005, 0.215, 0.19), rpy=(0.0, 0.32, 0.0)),
        material=dark_steel,
        name="brace_1",
    )
    rear_support.visual(
        Cylinder(radius=0.014, length=0.43),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="hinge_pin",
    )
    for y in (-0.165, 0.165):
        rear_support.visual(
            Cylinder(radius=0.036, length=0.085),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"outer_barrel_{0 if y < 0 else 1}",
        )

    pivot_frame = model.part("pivot_frame")
    pivot_frame.visual(
        Cylinder(radius=0.032, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue_frame,
        name="hinge_barrel",
    )
    pivot_frame.visual(
        Box((0.055, 0.204, 0.045)),
        origin=Origin(xyz=(0.047, 0.0, 0.0)),
        material=blue_frame,
        name="rear_bridge",
    )
    for name, y in (("side_rail_0", -0.105), ("side_rail_1", 0.105)):
        pivot_frame.visual(
            Box((0.405, 0.032, 0.045)),
            origin=Origin(xyz=(0.235, y, 0.0)),
            material=blue_frame,
            name=name,
        )
    pivot_frame.visual(
        Box((0.055, 0.245, 0.045)),
        origin=Origin(xyz=(0.418, 0.0, 0.0)),
        material=blue_frame,
        name="front_crossbar",
    )
    pivot_frame.visual(
        Box((0.23, 0.19, 0.012)),
        origin=Origin(xyz=(0.245, 0.0, -0.027)),
        material=bearing,
        name="slide_wear_strip",
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.465, 0.050, 0.035)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=brushed,
        name="center_beam",
    )
    for name, y in (("guide_shoe_0", -0.105), ("guide_shoe_1", 0.105)):
        slider.visual(
            Box((0.265, 0.026, 0.030)),
            origin=Origin(xyz=(0.085, y, -0.015)),
            material=brushed,
            name=name,
        )
    for x in (-0.020, 0.170):
        slider.visual(
            Box((0.045, 0.200, 0.020)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=brushed,
            name=f"carriage_web_{0 if x < 0 else 1}",
        )
    slider.visual(
        Box((0.090, 0.105, 0.080)),
        origin=Origin(xyz=(0.370, 0.0, 0.0)),
        material=brushed,
        name="wrist_socket",
    )
    slider.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.406, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="wrist_bearing_face",
    )

    output = model.part("output")
    output.visual(
        Cylinder(radius=0.038, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing,
        name="wrist_hub",
    )
    output.visual(
        Cylinder(radius=0.065, length=0.026),
        origin=Origin(xyz=(0.051, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=output_orange,
        name="output_disc",
    )
    output.visual(
        Box((0.014, 0.030, 0.080)),
        origin=Origin(xyz=(0.064, 0.0, 0.045)),
        material=output_orange,
        name="index_vane",
    )

    model.articulation(
        "support_to_frame",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=pivot_frame,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-0.35, upper=1.15),
    )
    model.articulation(
        "frame_to_slider",
        ArticulationType.PRISMATIC,
        parent=pivot_frame,
        child=slider,
        origin=Origin(xyz=(0.140, 0.0, 0.0525)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.24),
    )
    model.articulation(
        "slider_to_output",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=output,
        origin=Origin(xyz=(0.415, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rear_support = object_model.get_part("rear_support")
    pivot_frame = object_model.get_part("pivot_frame")
    slider = object_model.get_part("slider")
    output = object_model.get_part("output")
    support_to_frame = object_model.get_articulation("support_to_frame")
    frame_to_slider = object_model.get_articulation("frame_to_slider")
    slider_to_output = object_model.get_articulation("slider_to_output")

    ctx.check(
        "joint chain is revolute prismatic revolute",
        support_to_frame.articulation_type == ArticulationType.REVOLUTE
        and frame_to_slider.articulation_type == ArticulationType.PRISMATIC
        and slider_to_output.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"types={[support_to_frame.articulation_type, frame_to_slider.articulation_type, slider_to_output.articulation_type]}"
        ),
    )

    ctx.allow_overlap(
        rear_support,
        pivot_frame,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The rear support's captured hinge pin is intentionally nested through the pivot frame barrel.",
    )
    ctx.expect_within(
        rear_support,
        pivot_frame,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.003,
        name="hinge pin is centered inside frame barrel",
    )
    ctx.expect_overlap(
        rear_support,
        pivot_frame,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.16,
        name="hinge pin passes through frame barrel",
    )

    ctx.expect_within(
        slider,
        pivot_frame,
        axes="y",
        inner_elem="guide_shoe_0",
        outer_elem="side_rail_0",
        margin=0.003,
        name="lower guide shoe stays inside rail envelope",
    )
    ctx.expect_overlap(
        slider,
        pivot_frame,
        axes="x",
        elem_a="center_beam",
        elem_b="side_rail_0",
        min_overlap=0.25,
        name="collapsed slider remains supported by the frame",
    )
    with ctx.pose({frame_to_slider: 0.24}):
        ctx.expect_overlap(
            slider,
            pivot_frame,
            axes="x",
            elem_a="center_beam",
            elem_b="side_rail_0",
            min_overlap=0.11,
            name="extended slider keeps retained insertion",
        )

    closed_output_pos = ctx.part_world_position(output)
    with ctx.pose({support_to_frame: 0.8, frame_to_slider: 0.18, slider_to_output: math.pi / 2.0}):
        raised_output_pos = ctx.part_world_position(output)
    ctx.check(
        "hinged frame raises the wristed tip",
        closed_output_pos is not None
        and raised_output_pos is not None
        and raised_output_pos[2] > closed_output_pos[2] + 0.10,
        details=f"closed={closed_output_pos}, raised={raised_output_pos}",
    )

    ctx.expect_contact(
        slider,
        output,
        elem_a="wrist_bearing_face",
        elem_b="wrist_hub",
        contact_tol=0.002,
        name="output hub seats against slider wrist bearing",
    )

    return ctx.report()


object_model = build_object_model()
