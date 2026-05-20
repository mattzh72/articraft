from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_panel_axis")

    dark_steel = model.material("dark_powder_coat", rgba=(0.09, 0.10, 0.11, 1.0))
    back_plate = model.material("matte_service_panel", rgba=(0.20, 0.23, 0.25, 1.0))
    rail_steel = model.material("brushed_linear_rail", rgba=(0.72, 0.75, 0.74, 1.0))
    carriage_blue = model.material("blue_crosshead", rgba=(0.04, 0.22, 0.48, 1.0))
    slide_orange = model.material("safety_orange_slide", rgba=(0.95, 0.42, 0.08, 1.0))
    rubber_black = model.material("black_end_stop", rgba=(0.015, 0.015, 0.018, 1.0))

    frame = model.part("back_frame")
    frame.visual(
        Box((1.42, 0.040, 0.88)),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=back_plate,
        name="backing_panel",
    )
    frame.visual(
        Box((1.52, 0.075, 0.060)),
        origin=Origin(xyz=(0.0, -0.008, 0.87)),
        material=dark_steel,
        name="top_frame_bar",
    )
    frame.visual(
        Box((1.52, 0.075, 0.060)),
        origin=Origin(xyz=(0.0, -0.008, 0.03)),
        material=dark_steel,
        name="bottom_frame_bar",
    )
    frame.visual(
        Box((0.060, 0.075, 0.88)),
        origin=Origin(xyz=(-0.73, -0.008, 0.44)),
        material=dark_steel,
        name="side_frame_0",
    )
    frame.visual(
        Box((0.060, 0.075, 0.88)),
        origin=Origin(xyz=(0.73, -0.008, 0.44)),
        material=dark_steel,
        name="side_frame_1",
    )
    frame.visual(
        Box((1.22, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.035, 0.68)),
        material=rail_steel,
        name="upper_linear_rail",
    )
    frame.visual(
        Box((1.22, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.035, 0.50)),
        material=rail_steel,
        name="lower_linear_rail",
    )
    for x in (-0.6375, 0.6375):
        suffix = "0" if x < 0.0 else "1"
        frame.visual(
            Box((0.055, 0.050, 0.080)),
            origin=Origin(xyz=(x, -0.055, 0.68)),
            material=rubber_black,
            name=f"upper_end_stop_{suffix}",
        )
        frame.visual(
            Box((0.055, 0.050, 0.080)),
            origin=Origin(xyz=(x, -0.055, 0.50)),
            material=rubber_black,
            name=f"lower_end_stop_{suffix}",
        )

    crosshead = model.part("crosshead")
    crosshead.visual(
        Box((0.44, 0.035, 0.260)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_blue,
        name="front_plate",
    )
    # Shallow C-shaped shoes wrap the fixed rails with visible clearance.
    for rail_z, label in ((0.100, "upper"), (-0.080, "lower")):
        crosshead.visual(
            Box((0.36, 0.025, 0.014)),
            origin=Origin(xyz=(0.0, 0.020, rail_z + 0.0245)),
            material=carriage_blue,
            name=f"{label}_shoe_top_lip",
        )
        crosshead.visual(
            Box((0.36, 0.025, 0.014)),
            origin=Origin(xyz=(0.0, 0.020, rail_z - 0.0245)),
            material=carriage_blue,
            name=f"{label}_shoe_bottom_lip",
        )
    crosshead.visual(
        Box((0.075, 0.040, 0.180)),
        origin=Origin(xyz=(-0.145, -0.010, -0.020)),
        material=dark_steel,
        name="bearing_block_0",
    )
    crosshead.visual(
        Box((0.075, 0.040, 0.180)),
        origin=Origin(xyz=(0.145, -0.010, -0.020)),
        material=dark_steel,
        name="bearing_block_1",
    )
    # The hanging yoke keeps the vertical slide physically supported while
    # leaving a clear channel for the sliding member.
    crosshead.visual(
        Box((0.036, 0.080, 0.170)),
        origin=Origin(xyz=(-0.096, -0.040, -0.115)),
        material=carriage_blue,
        name="hanger_arm_0",
    )
    crosshead.visual(
        Box((0.026, 0.055, 0.340)),
        origin=Origin(xyz=(-0.071, -0.060, -0.210)),
        material=carriage_blue,
        name="z_guide_cheek_0",
    )
    crosshead.visual(
        Box((0.036, 0.080, 0.170)),
        origin=Origin(xyz=(0.096, -0.040, -0.115)),
        material=carriage_blue,
        name="hanger_arm_1",
    )
    crosshead.visual(
        Box((0.026, 0.055, 0.340)),
        origin=Origin(xyz=(0.071, -0.060, -0.210)),
        material=carriage_blue,
        name="z_guide_cheek_1",
    )
    crosshead.visual(
        Box((0.185, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, -0.060, -0.025)),
        material=carriage_blue,
        name="z_guide_bridge",
    )
    crosshead.visual(
        Box((0.165, 0.060, 0.035)),
        origin=Origin(xyz=(0.0, -0.1125, -0.390)),
        material=dark_steel,
        name="z_guide_lower_tie",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((0.085, 0.035, 0.500)),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=slide_orange,
        name="vertical_bar",
    )
    z_slide.visual(
        Box((0.160, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.405)),
        material=dark_steel,
        name="lower_tool_plate",
    )
    z_slide.visual(
        Box((0.115, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -0.006, 0.125)),
        material=rubber_black,
        name="upper_stop_pad",
    )

    model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(0.0, -0.080, 0.580)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=-0.30, upper=0.30),
    )
    model.articulation(
        "crosshead_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=z_slide,
        origin=Origin(xyz=(0.0, -0.060, -0.180)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("back_frame")
    crosshead = object_model.get_part("crosshead")
    z_slide = object_model.get_part("z_slide")
    x_axis = object_model.get_articulation("frame_to_crosshead")
    z_axis = object_model.get_articulation("crosshead_to_z_slide")

    ctx.expect_within(
        crosshead,
        frame,
        axes="x",
        inner_elem="front_plate",
        outer_elem="upper_linear_rail",
        margin=0.005,
        name="centered crosshead rides within the horizontal rail span",
    )
    ctx.expect_within(
        z_slide,
        crosshead,
        axes="xy",
        inner_elem="vertical_bar",
        outer_elem="z_guide_bridge",
        margin=0.006,
        name="z slide is centered in the hanging guide mouth",
    )
    ctx.expect_overlap(
        z_slide,
        crosshead,
        axes="z",
        elem_a="vertical_bar",
        elem_b="z_guide_cheek_0",
        min_overlap=0.080,
        name="retracted z slide remains captured in the guide",
    )

    left_pos = None
    right_pos = None
    with ctx.pose({x_axis: -0.30}):
        ctx.expect_within(
            crosshead,
            frame,
            axes="x",
            inner_elem="front_plate",
            outer_elem="upper_linear_rail",
            margin=0.005,
            name="left travel stop retains crosshead on rail",
        )
        left_pos = ctx.part_world_position(crosshead)
    with ctx.pose({x_axis: 0.30}):
        ctx.expect_within(
            crosshead,
            frame,
            axes="x",
            inner_elem="front_plate",
            outer_elem="upper_linear_rail",
            margin=0.005,
            name="right travel stop retains crosshead on rail",
        )
        right_pos = ctx.part_world_position(crosshead)
    ctx.check(
        "crosshead translates horizontally",
        left_pos is not None
        and right_pos is not None
        and right_pos[0] > left_pos[0] + 0.55
        and abs(right_pos[2] - left_pos[2]) < 1e-6,
        details=f"left={left_pos}, right={right_pos}",
    )

    rest_pos = ctx.part_world_position(z_slide)
    with ctx.pose({z_axis: 0.22}):
        ctx.expect_overlap(
            z_slide,
            crosshead,
            axes="z",
            elem_a="vertical_bar",
            elem_b="z_guide_cheek_0",
            min_overlap=0.070,
            name="extended z slide keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(z_slide)
    ctx.check(
        "z slide lowers on an orthogonal axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] < rest_pos[2] - 0.20
        and abs(extended_pos[0] - rest_pos[0]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
