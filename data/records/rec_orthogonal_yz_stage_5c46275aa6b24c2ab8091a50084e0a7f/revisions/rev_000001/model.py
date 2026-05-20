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
    model = ArticulatedObject(name="side_wall_yz_positioning_module")

    wall_paint = model.material("blue_gray_powdercoat", rgba=(0.18, 0.23, 0.28, 1.0))
    dark_paint = model.material("dark_anodized_guide", rgba=(0.06, 0.07, 0.075, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    carriage_orange = model.material("orange_carriage", rgba=(0.95, 0.42, 0.08, 1.0))
    slider_yellow = model.material("safety_yellow_slider", rgba=(0.95, 0.78, 0.12, 1.0))
    black = model.material("black_fasteners", rgba=(0.015, 0.015, 0.015, 1.0))

    side_wall = model.part("side_wall")
    side_wall.visual(
        Box((0.05, 1.20, 0.90)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=wall_paint,
        name="wall_panel",
    )
    side_wall.visual(
        Box((0.16, 1.26, 0.045)),
        origin=Origin(xyz=(0.045, 0.0, 0.0225)),
        material=dark_paint,
        name="floor_foot",
    )
    side_wall.visual(
        Box((0.018, 1.10, 0.72)),
        origin=Origin(xyz=(0.015, 0.0, 0.45)),
        material=wall_paint,
        name="raised_wall_rib",
    )

    # Two hardened horizontal guide rods on the wall face carry the lateral stage.
    rail_rpy = (-math.pi / 2.0, 0.0, 0.0)
    for z, rail_name in ((0.34, "lower_y_rail"), (0.56, "upper_y_rail")):
        side_wall.visual(
            Cylinder(radius=0.016, length=0.95),
            origin=Origin(xyz=(0.055, 0.0, z), rpy=rail_rpy),
            material=steel,
            name=rail_name,
        )
        for index, y in enumerate((-0.49, 0.49)):
            side_wall.visual(
                Box((0.050, 0.025, 0.050)),
                origin=Origin(xyz=(0.032, y, z)),
                material=dark_paint,
                name=f"{rail_name}_standoff_{index}",
            )

    for y, stop_name in ((-0.495, "negative_y_stop"), (0.495, "positive_y_stop")):
        side_wall.visual(
            Box((0.070, 0.042, 0.34)),
            origin=Origin(xyz=(0.055, y, 0.45)),
            material=black,
            name=stop_name,
        )

    lateral_carriage = model.part("lateral_carriage")
    lateral_carriage.visual(
        Box((0.035, 0.200, 0.300)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_orange,
        name="saddle_plate",
    )
    # Four recirculating shoe pads touch the two wall rods without intersecting them.
    for z, shoe_name in (
        (0.133, "upper_top_shoe"),
        (0.087, "upper_bottom_shoe"),
        (-0.087, "lower_top_shoe"),
        (-0.133, "lower_bottom_shoe"),
    ):
        lateral_carriage.visual(
            Box((0.110, 0.180, 0.014)),
            origin=Origin(xyz=(-0.045, 0.0, z)),
            material=dark_paint,
            name=shoe_name,
        )
    lateral_carriage.visual(
        Box((0.340, 0.080, 0.050)),
        origin=Origin(xyz=(0.160, 0.0, 0.030)),
        material=carriage_orange,
        name="lateral_arm",
    )
    lateral_carriage.visual(
        Box((0.180, 0.026, 0.090)),
        origin=Origin(xyz=(0.070, 0.048, -0.020)),
        material=carriage_orange,
        name="upper_gusset",
    )
    lateral_carriage.visual(
        Box((0.180, 0.026, 0.090)),
        origin=Origin(xyz=(0.070, -0.048, -0.020)),
        material=carriage_orange,
        name="lower_gusset",
    )
    lateral_carriage.visual(
        Box((0.060, 0.120, 0.440)),
        origin=Origin(xyz=(0.320, 0.0, 0.030)),
        material=dark_paint,
        name="vertical_track_backing",
    )
    for y, rail_name in ((0.045, "front_guide_rail_0"), (-0.045, "front_guide_rail_1")):
        lateral_carriage.visual(
            Box((0.014, 0.014, 0.440)),
            origin=Origin(xyz=(0.357, y, 0.030)),
            material=steel,
            name=rail_name,
        )
    lateral_carriage.visual(
        Box((0.080, 0.135, 0.035)),
        origin=Origin(xyz=(0.330, 0.0, 0.265)),
        material=dark_paint,
        name="top_slider_stop",
    )
    lateral_carriage.visual(
        Box((0.080, 0.135, 0.035)),
        origin=Origin(xyz=(0.330, 0.0, -0.205)),
        material=dark_paint,
        name="bottom_slider_stop",
    )

    vertical_slider = model.part("vertical_slider")
    vertical_slider.visual(
        Box((0.040, 0.080, 0.260)),
        origin=Origin(xyz=(0.026, 0.0, 0.100)),
        material=slider_yellow,
        name="moving_slide_plate",
    )
    for y, pad_name in ((0.045, "side_wear_pad_0"), (-0.045, "side_wear_pad_1")):
        vertical_slider.visual(
            Box((0.008, 0.018, 0.210)),
            origin=Origin(xyz=(0.003, y, 0.100)),
            material=black,
            name=pad_name,
        )
    vertical_slider.visual(
        Box((0.065, 0.120, 0.055)),
        origin=Origin(xyz=(0.062, 0.0, 0.100)),
        material=slider_yellow,
        name="front_tool_mount",
    )
    vertical_slider.visual(
        Box((0.030, 0.085, 0.018)),
        origin=Origin(xyz=(0.100, 0.0, 0.100)),
        material=steel,
        name="tool_face_plate",
    )

    model.articulation(
        "wall_to_carriage",
        ArticulationType.PRISMATIC,
        parent=side_wall,
        child=lateral_carriage,
        origin=Origin(xyz=(0.125, -0.320, 0.450)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.640),
    )

    model.articulation(
        "carriage_to_slider",
        ArticulationType.PRISMATIC,
        parent=lateral_carriage,
        child=vertical_slider,
        origin=Origin(xyz=(0.365, 0.0, -0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_wall = object_model.get_part("side_wall")
    lateral_carriage = object_model.get_part("lateral_carriage")
    vertical_slider = object_model.get_part("vertical_slider")
    y_joint = object_model.get_articulation("wall_to_carriage")
    z_joint = object_model.get_articulation("carriage_to_slider")

    ctx.check(
        "uses exactly two prismatic positioning joints",
        len(object_model.articulations) == 2
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    ctx.expect_overlap(
        lateral_carriage,
        side_wall,
        axes="y",
        elem_a="saddle_plate",
        elem_b="upper_y_rail",
        min_overlap=0.16,
        name="lateral saddle remains engaged on Y rail at home",
    )
    ctx.expect_contact(
        lateral_carriage,
        side_wall,
        elem_a="upper_bottom_shoe",
        elem_b="upper_y_rail",
        contact_tol=0.002,
        name="upper carriage shoe touches the upper wall rail",
    )
    ctx.expect_overlap(
        vertical_slider,
        lateral_carriage,
        axes="z",
        elem_a="side_wear_pad_0",
        elem_b="front_guide_rail_0",
        min_overlap=0.18,
        name="vertical slider is retained in the compact guide at home",
    )
    ctx.expect_contact(
        vertical_slider,
        lateral_carriage,
        elem_a="side_wear_pad_0",
        elem_b="front_guide_rail_0",
        contact_tol=0.002,
        name="vertical wear pad bears on the guide rail",
    )

    home_carriage_pos = ctx.part_world_position(lateral_carriage)
    with ctx.pose({y_joint: 0.640}):
        extended_carriage_pos = ctx.part_world_position(lateral_carriage)
        ctx.expect_overlap(
            lateral_carriage,
            side_wall,
            axes="y",
            elem_a="saddle_plate",
            elem_b="upper_y_rail",
            min_overlap=0.16,
            name="lateral saddle stays on rail at far Y travel",
        )
    ctx.check(
        "horizontal prismatic joint moves carriage along positive Y",
        home_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[1] > home_carriage_pos[1] + 0.60,
        details=f"home={home_carriage_pos}, extended={extended_carriage_pos}",
    )

    home_slider_pos = ctx.part_world_position(vertical_slider)
    with ctx.pose({z_joint: 0.220}):
        raised_slider_pos = ctx.part_world_position(vertical_slider)
        ctx.expect_overlap(
            vertical_slider,
            lateral_carriage,
            axes="z",
            elem_a="side_wear_pad_0",
            elem_b="front_guide_rail_0",
            min_overlap=0.18,
            name="vertical slider remains retained at raised travel",
        )
    ctx.check(
        "vertical prismatic joint raises the compact slider",
        home_slider_pos is not None
        and raised_slider_pos is not None
        and raised_slider_pos[2] > home_slider_pos[2] + 0.20,
        details=f"home={home_slider_pos}, raised={raised_slider_pos}",
    )

    return ctx.report()


object_model = build_object_model()
