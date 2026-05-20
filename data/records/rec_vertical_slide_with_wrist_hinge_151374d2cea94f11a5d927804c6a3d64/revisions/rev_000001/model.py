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
    model = ArticulatedObject(name="gantry_z_carriage_pitch_wrist")

    painted = Material("graphite_powder_coat", color=(0.12, 0.13, 0.14, 1.0))
    rail_steel = Material("ground_bearing_steel", color=(0.78, 0.78, 0.74, 1.0))
    carriage_blue = Material("anodized_blue_carriage", color=(0.05, 0.20, 0.43, 1.0))
    guide_red = Material("red_guide_block", color=(0.72, 0.08, 0.05, 1.0))
    bracket_steel = Material("brushed_wrist_bracket", color=(0.46, 0.47, 0.45, 1.0))
    tool_dark = Material("dark_tool_plate", color=(0.06, 0.065, 0.07, 1.0))
    rubber = Material("black_rubber_stops", color=(0.01, 0.01, 0.012, 1.0))

    frame = model.part("gantry_frame")
    frame.visual(
        Box((0.36, 0.040, 0.80)),
        origin=Origin(xyz=(0.0, -0.035, 0.40)),
        material=painted,
        name="back_plate",
    )
    frame.visual(
        Box((0.32, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, -0.005, 0.765)),
        material=painted,
        name="top_crosshead",
    )
    frame.visual(
        Box((0.32, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, -0.005, 0.035)),
        material=painted,
        name="bottom_crosshead",
    )
    frame.visual(
        Box((0.034, 0.022, 0.700)),
        origin=Origin(xyz=(-0.10, -0.015, 0.40)),
        material=painted,
        name="rail_0_seat",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.700),
        origin=Origin(xyz=(-0.10, 0.005, 0.40)),
        material=rail_steel,
        name="rail_0",
    )
    frame.visual(
        Box((0.050, 0.052, 0.026)),
        origin=Origin(xyz=(-0.10, 0.000, 0.735)),
        material=painted,
        name="rail_0_top_clamp",
    )
    frame.visual(
        Box((0.050, 0.052, 0.026)),
        origin=Origin(xyz=(-0.10, 0.000, 0.065)),
        material=painted,
        name="rail_0_bottom_clamp",
    )
    frame.visual(
        Box((0.034, 0.022, 0.700)),
        origin=Origin(xyz=(0.10, -0.015, 0.40)),
        material=painted,
        name="rail_1_seat",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.700),
        origin=Origin(xyz=(0.10, 0.005, 0.40)),
        material=rail_steel,
        name="rail_1",
    )
    frame.visual(
        Box((0.050, 0.052, 0.026)),
        origin=Origin(xyz=(0.10, 0.000, 0.735)),
        material=painted,
        name="rail_1_top_clamp",
    )
    frame.visual(
        Box((0.050, 0.052, 0.026)),
        origin=Origin(xyz=(0.10, 0.000, 0.065)),
        material=painted,
        name="rail_1_bottom_clamp",
    )
    frame.visual(
        Box((0.075, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.000, 0.710)),
        material=rubber,
        name="upper_stop",
    )
    frame.visual(
        Box((0.075, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.000, 0.190)),
        material=rubber,
        name="lower_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.250, 0.028, 0.205)),
        origin=Origin(xyz=(0.0, -0.025, 0.0)),
        material=carriage_blue,
        name="guide_backplate",
    )
    carriage.visual(
        Box((0.105, 0.048, 0.220)),
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        material=carriage_blue,
        name="narrow_cover",
    )
    carriage.visual(
        Box((0.132, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.010, -0.130)),
        material=carriage_blue,
        name="wrist_mount_boss",
    )
    carriage.visual(
        Box((0.054, 0.024, 0.070)),
        origin=Origin(xyz=(-0.10, -0.024, -0.058)),
        material=guide_red,
        name="rail_0_lower_guide_front",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.070)),
        origin=Origin(xyz=(-0.117, -0.039, -0.058)),
        material=guide_red,
        name="rail_0_lower_guide_inner_pad",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.070)),
        origin=Origin(xyz=(-0.083, -0.039, -0.058)),
        material=guide_red,
        name="rail_0_lower_guide_outer_pad",
    )
    carriage.visual(
        Box((0.054, 0.024, 0.070)),
        origin=Origin(xyz=(-0.10, -0.024, 0.058)),
        material=guide_red,
        name="rail_0_upper_guide_front",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.070)),
        origin=Origin(xyz=(-0.117, -0.039, 0.058)),
        material=guide_red,
        name="rail_0_upper_guide_inner_pad",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.070)),
        origin=Origin(xyz=(-0.083, -0.039, 0.058)),
        material=guide_red,
        name="rail_0_upper_guide_outer_pad",
    )
    carriage.visual(
        Box((0.054, 0.024, 0.070)),
        origin=Origin(xyz=(0.10, -0.024, -0.058)),
        material=guide_red,
        name="rail_1_lower_guide_front",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.070)),
        origin=Origin(xyz=(0.083, -0.039, -0.058)),
        material=guide_red,
        name="rail_1_lower_guide_inner_pad",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.070)),
        origin=Origin(xyz=(0.117, -0.039, -0.058)),
        material=guide_red,
        name="rail_1_lower_guide_outer_pad",
    )
    carriage.visual(
        Box((0.054, 0.024, 0.070)),
        origin=Origin(xyz=(0.10, -0.024, 0.058)),
        material=guide_red,
        name="rail_1_upper_guide_front",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.070)),
        origin=Origin(xyz=(0.083, -0.039, 0.058)),
        material=guide_red,
        name="rail_1_upper_guide_inner_pad",
    )
    carriage.visual(
        Box((0.010, 0.032, 0.070)),
        origin=Origin(xyz=(0.117, -0.039, 0.058)),
        material=guide_red,
        name="rail_1_upper_guide_outer_pad",
    )

    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.060, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.35, lower=0.0, upper=0.260),
    )

    bracket = model.part("wrist_bracket")
    bracket.visual(
        Box((0.130, 0.052, 0.040)),
        origin=Origin(xyz=(0.0, 0.000, 0.045)),
        material=bracket_steel,
        name="top_mount",
    )
    bracket.visual(
        Box((0.020, 0.072, 0.120)),
        origin=Origin(xyz=(-0.062, 0.000, 0.000)),
        material=bracket_steel,
        name="cheek_0",
    )
    bracket.visual(
        Box((0.020, 0.072, 0.120)),
        origin=Origin(xyz=(0.062, 0.000, 0.000)),
        material=bracket_steel,
        name="cheek_1",
    )
    bracket.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(-0.075, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="pivot_cap_0",
    )
    bracket.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.075, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="pivot_cap_1",
    )

    model.articulation(
        "bracket_mount",
        ArticulationType.FIXED,
        parent=carriage,
        child=bracket,
        origin=Origin(xyz=(0.0, 0.018, -0.215)),
    )

    tool = model.part("tool_plate")
    tool.visual(
        Cylinder(radius=0.016, length=0.104),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="trunnion_hub",
    )
    tool.visual(
        Box((0.070, 0.028, 0.034)),
        origin=Origin(xyz=(0.0, 0.020, -0.028)),
        material=tool_dark,
        name="neck_block",
    )
    tool.visual(
        Box((0.100, 0.016, 0.078)),
        origin=Origin(xyz=(0.0, 0.040, -0.078)),
        material=tool_dark,
        name="mounting_plate",
    )
    for x in (-0.032, 0.032):
        for z in (-0.056, -0.100):
            tool.visual(
                Cylinder(radius=0.0045, length=0.004),
                origin=Origin(xyz=(x, 0.049, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rail_steel,
                name=f"plate_screw_{x:+.3f}_{z:+.3f}",
            )

    model.articulation(
        "pitch_wrist",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=tool,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("gantry_frame")
    carriage = object_model.get_part("carriage")
    bracket = object_model.get_part("wrist_bracket")
    tool = object_model.get_part("tool_plate")
    z_slide = object_model.get_articulation("z_slide")
    wrist = object_model.get_articulation("pitch_wrist")

    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="rail_0_upper_guide_front",
        elem_b="rail_0",
        min_overlap=0.060,
        name="upper guide remains on the rail at the lower z pose",
    )
    ctx.expect_gap(
        carriage,
        bracket,
        axis="z",
        positive_elem="wrist_mount_boss",
        negative_elem="top_mount",
        max_gap=0.001,
        max_penetration=0.0,
        name="wrist bracket seats under the carriage boss",
    )
    ctx.expect_within(
        tool,
        bracket,
        axes="x",
        inner_elem="trunnion_hub",
        outer_elem="top_mount",
        margin=0.0,
        name="tool trunnion stays inside the bracket width",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({z_slide: 0.260}):
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="rail_0_lower_guide_front",
            elem_b="rail_0",
            min_overlap=0.060,
            name="lower guide remains on the rail at the upper z pose",
        )
        upper_pos = ctx.part_world_position(carriage)

    ctx.check(
        "z carriage travels upward on its rails",
        rest_pos is not None and upper_pos is not None and upper_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )

    for q, label in [(-0.85, "backward"), (0.85, "forward")]:
        with ctx.pose({wrist: q}):
            ctx.expect_gap(
                bracket,
                tool,
                axis="z",
                positive_elem="top_mount",
                negative_elem="mounting_plate",
                min_gap=0.012,
                name=f"tool plate clears top mount at {label} wrist limit",
            )

    return ctx.report()


object_model = build_object_model()
