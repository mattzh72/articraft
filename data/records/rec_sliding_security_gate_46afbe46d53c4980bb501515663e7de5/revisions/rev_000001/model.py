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
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = Material("galvanized_steel", rgba=(0.48, 0.52, 0.53, 1.0))
    dark_steel = Material("black_powder_coated_steel", rgba=(0.03, 0.035, 0.035, 1.0))
    guide_steel = Material("worn_track_steel", rgba=(0.24, 0.25, 0.24, 1.0))
    rubber = Material("dark_rubber_tires", rgba=(0.015, 0.015, 0.013, 1.0))
    concrete = Material("poured_concrete", rgba=(0.45, 0.43, 0.39, 1.0))

    fixed_frame = model.part("fixed_frame")
    fixed_frame.visual(
        Box((2.85, 0.34, 0.08)),
        origin=Origin(xyz=(0.20, 0.0, -0.04)),
        material=concrete,
        name="concrete_sill",
    )
    fixed_frame.visual(
        Box((0.12, 0.12, 2.30)),
        origin=Origin(xyz=(-1.03, 0.0, 1.15)),
        material=galvanized,
        name="fixed_post",
    )
    fixed_frame.visual(
        Box((0.10, 0.10, 2.18)),
        origin=Origin(xyz=(1.43, 0.0, 1.09)),
        material=galvanized,
        name="guide_post",
    )
    fixed_frame.visual(
        Box((2.58, 0.10, 0.08)),
        origin=Origin(xyz=(0.20, 0.0, 2.24)),
        material=galvanized,
        name="top_support_beam",
    )
    fixed_frame.visual(
        Box((2.62, 0.16, 0.045)),
        origin=Origin(xyz=(0.20, 0.0, 2.17)),
        material=guide_steel,
        name="top_channel_cap",
    )
    fixed_frame.visual(
        Box((2.62, 0.022, 0.15)),
        origin=Origin(xyz=(0.20, -0.069, 2.08)),
        material=guide_steel,
        name="top_channel_front_lip",
    )
    fixed_frame.visual(
        Box((2.62, 0.022, 0.15)),
        origin=Origin(xyz=(0.20, 0.069, 2.08)),
        material=guide_steel,
        name="top_channel_rear_lip",
    )
    fixed_frame.visual(
        Box((2.62, 0.09, 0.04)),
        origin=Origin(xyz=(0.20, 0.0, 0.02)),
        material=guide_steel,
        name="bottom_guide_base",
    )
    fixed_frame.visual(
        Box((2.62, 0.022, 0.10)),
        origin=Origin(xyz=(0.20, -0.071, 0.085)),
        material=guide_steel,
        name="bottom_guide_front_lip",
    )
    fixed_frame.visual(
        Box((2.62, 0.022, 0.10)),
        origin=Origin(xyz=(0.20, 0.071, 0.085)),
        material=guide_steel,
        name="bottom_guide_rear_lip",
    )
    fixed_frame.visual(
        Box((0.16, 0.055, 0.18)),
        origin=Origin(xyz=(-0.97, -0.082, 1.06)),
        material=guide_steel,
        name="latch_receiver",
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((1.25, 0.055, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=dark_steel,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((1.25, 0.055, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.90)),
        material=dark_steel,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((0.06, 0.055, 1.82)),
        origin=Origin(xyz=(-0.625, 0.0, 1.03)),
        material=dark_steel,
        name="leading_stile",
    )
    gate_leaf.visual(
        Box((0.06, 0.055, 1.82)),
        origin=Origin(xyz=(0.625, 0.0, 1.03)),
        material=dark_steel,
        name="trailing_stile",
    )
    gate_leaf.visual(
        Box((1.20, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
        material=dark_steel,
        name="middle_crossbar",
    )
    for index, x in enumerate((-0.42, -0.21, 0.0, 0.21, 0.42)):
        gate_leaf.visual(
            Box((0.026, 0.04, 1.74)),
            origin=Origin(xyz=(x, 0.0, 1.03)),
            material=dark_steel,
            name=f"vertical_bar_{index}",
        )

    wheel_rotation = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    gate_leaf.visual(
        Box((0.034, 0.04, 0.17)),
        origin=Origin(xyz=(-0.43, 0.0, 2.005)),
        material=dark_steel,
        name="top_roller_hanger_0",
    )
    gate_leaf.visual(
        Cylinder(radius=0.034, length=0.075),
        origin=Origin(xyz=(-0.43, 0.0, 2.045), rpy=wheel_rotation.rpy),
        material=rubber,
        name="top_roller_0",
    )
    gate_leaf.visual(
        Cylinder(radius=0.052, length=0.048),
        origin=Origin(xyz=(-0.43, 0.0, 0.095), rpy=wheel_rotation.rpy),
        material=rubber,
        name="bottom_wheel_0",
    )
    gate_leaf.visual(
        Box((0.034, 0.04, 0.17)),
        origin=Origin(xyz=(0.43, 0.0, 2.005)),
        material=dark_steel,
        name="top_roller_hanger_1",
    )
    gate_leaf.visual(
        Cylinder(radius=0.034, length=0.075),
        origin=Origin(xyz=(0.43, 0.0, 2.045), rpy=wheel_rotation.rpy),
        material=rubber,
        name="top_roller_1",
    )
    gate_leaf.visual(
        Cylinder(radius=0.052, length=0.048),
        origin=Origin(xyz=(0.43, 0.0, 0.095), rpy=wheel_rotation.rpy),
        material=rubber,
        name="bottom_wheel_1",
    )

    gate_leaf.visual(
        Box((0.14, 0.028, 0.11)),
        origin=Origin(xyz=(-0.58, -0.037, 1.08)),
        material=guide_steel,
        name="lock_box",
    )
    gate_leaf.visual(
        Box((0.14, 0.018, 0.018)),
        origin=Origin(xyz=(-0.65, -0.045, 1.055)),
        material=guide_steel,
        name="latch_tongue",
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=gate_leaf,
        origin=Origin(xyz=(-0.25, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.65, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("gate_slide")

    ctx.check(
        "gate leaf has bounded prismatic travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == 0.75,
        details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
    )
    ctx.expect_within(
        gate_leaf,
        fixed_frame,
        axes="xy",
        inner_elem="top_roller_0",
        outer_elem="top_channel_cap",
        margin=0.0,
        name="top roller is captured under the channel at rest",
    )
    ctx.expect_gap(
        gate_leaf,
        fixed_frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="bottom_wheel_0",
        negative_elem="bottom_guide_base",
        name="bottom wheel runs just above the guide rail",
    )

    rest_position = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: 0.75}):
        ctx.expect_within(
            gate_leaf,
            fixed_frame,
            axes="xy",
            inner_elem="top_roller_1",
            outer_elem="top_channel_cap",
            margin=0.0,
            name="top roller remains captured when open",
        )
        ctx.expect_gap(
            gate_leaf,
            fixed_frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.010,
            positive_elem="bottom_wheel_1",
            negative_elem="bottom_guide_base",
            name="bottom wheel still rides above the guide rail when open",
        )
        extended_position = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate leaf translates along the track axis",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.70
        and abs(extended_position[1] - rest_position[1]) < 0.001
        and abs(extended_position[2] - rest_position[2]) < 0.001,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
