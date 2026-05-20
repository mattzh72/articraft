from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_tool_axis")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    slide_blue = model.material("blue_machined_slide", rgba=(0.10, 0.22, 0.34, 1.0))
    pale_plate = model.material("pale_end_plate", rgba=(0.70, 0.73, 0.72, 1.0))
    black = model.material("black_fasteners", rgba=(0.015, 0.015, 0.014, 1.0))
    rubber = model.material("matte_rubber_stops", rgba=(0.02, 0.018, 0.016, 1.0))

    column = model.part("column")
    column.visual(
        Box((0.34, 0.26, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=anodized,
        name="base_plate",
    )
    column.visual(
        Box((0.22, 0.08, 0.98)),
        origin=Origin(xyz=(0.0, 0.005, 0.53)),
        material=anodized,
        name="upright_body",
    )
    column.visual(
        Box((0.17, 0.012, 0.94)),
        origin=Origin(xyz=(0.0, -0.036, 0.57)),
        material=brushed,
        name="front_guide",
    )
    for i, x in enumerate((-0.087, 0.087)):
        column.visual(
            Box((0.018, 0.026, 0.94)),
            origin=Origin(xyz=(x, -0.052, 0.57)),
            material=brushed,
            name=f"guide_rail_{i}",
        )
    column.visual(
        Box((0.16, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -0.107, 0.173)),
        material=rubber,
        name="lower_stop",
    )
    for i, x in enumerate((-0.087, 0.087)):
        column.visual(
            Box((0.018, 0.070, 0.014)),
            origin=Origin(xyz=(x, -0.075, 0.173)),
            material=rubber,
            name=f"lower_stop_arm_{i}",
        )
    column.visual(
        Box((0.16, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, -0.064, 1.007)),
        material=rubber,
        name="upper_stop",
    )
    column.visual(
        Box((0.20, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, -0.026, 1.045)),
        material=anodized,
        name="top_cap",
    )
    for i, (x, y, z) in enumerate(
        (
            (-0.125, -0.120, 0.035),
            (0.125, -0.120, 0.035),
            (-0.092, -0.058, 0.965),
            (0.092, -0.058, 0.965),
        )
    ):
        column.visual(
            Cylinder(radius=0.008, length=0.005),
            origin=Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"socket_head_{i}",
        )

    slide = model.part("slide")
    slide.visual(
        Box((0.135, 0.035, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=slide_blue,
        name="main_slide",
    )
    slide.visual(
        Box((0.126, 0.014, 0.46)),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=brushed,
        name="rear_bearing",
    )
    for i, x in enumerate((-0.047, 0.047)):
        slide.visual(
            Box((0.016, 0.004, 0.44)),
            origin=Origin(xyz=(x, -0.019, 0.015)),
            material=black,
            name=f"front_gib_{i}",
        )
    slide.visual(
        Box((0.078, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, -0.012, -0.242)),
        material=black,
        name="bottom_impact_pad",
    )
    slide.visual(
        Box((0.078, 0.052, 0.016)),
        origin=Origin(xyz=(0.0, -0.012, 0.242)),
        material=black,
        name="top_impact_pad",
    )

    hinge_frame = model.part("hinge_frame")
    hinge_frame.visual(
        Box((0.13, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=brushed,
        name="upper_mount",
    )
    for i, x in enumerate((-0.064, 0.064)):
        hinge_frame.visual(
            Box((0.018, 0.042, 0.016)),
            origin=Origin(xyz=(x, 0.0, 0.027)),
            material=brushed,
            name=f"cheek_upper_{i}",
        )
        hinge_frame.visual(
            Box((0.018, 0.042, 0.044)),
            origin=Origin(xyz=(x, 0.0, -0.044)),
            material=brushed,
            name=f"cheek_lower_{i}",
        )
        hinge_frame.visual(
            Cylinder(radius=0.023, length=0.026),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed,
            name=f"wrist_cap_{i}",
        )
    hinge_frame.visual(
        Box((0.13, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.026, -0.071)),
        material=brushed,
        name="rear_tie",
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Cylinder(radius=0.008, length=0.152),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="wrist_pin",
    )
    end_plate.visual(
        Cylinder(radius=0.017, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="hinge_barrel",
    )
    end_plate.visual(
        Box((0.075, 0.018, 0.075)),
        origin=Origin(xyz=(0.0, -0.019, -0.0545)),
        material=pale_plate,
        name="tool_plate",
    )
    end_plate.visual(
        Box((0.060, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, -0.012, -0.020)),
        material=pale_plate,
        name="plate_lug",
    )
    for i, (x, z) in enumerate(((-0.022, -0.040), (0.022, -0.040), (-0.022, -0.075), (0.022, -0.075))):
        end_plate.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, -0.029, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"plate_bolt_{i}",
        )

    model.articulation(
        "column_to_slide",
        ArticulationType.PRISMATIC,
        parent=column,
        child=slide,
        origin=Origin(xyz=(0.0, -0.075, 0.43)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.18, lower=0.0, upper=0.32),
        motion_properties=MotionProperties(damping=4.0, friction=1.0),
    )
    model.articulation(
        "slide_to_hinge_frame",
        ArticulationType.FIXED,
        parent=slide,
        child=hinge_frame,
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
    )
    model.articulation(
        "hinge_frame_to_end_plate",
        ArticulationType.REVOLUTE,
        parent=hinge_frame,
        child=end_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.25, friction=0.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    slide = object_model.get_part("slide")
    hinge_frame = object_model.get_part("hinge_frame")
    end_plate = object_model.get_part("end_plate")
    lift = object_model.get_articulation("column_to_slide")
    wrist = object_model.get_articulation("hinge_frame_to_end_plate")

    for cap_name in ("wrist_cap_0", "wrist_cap_1"):
        ctx.allow_overlap(
            hinge_frame,
            end_plate,
            elem_a=cap_name,
            elem_b="wrist_pin",
            reason=(
                "The wrist pin is intentionally captured inside the machined "
                "bearing cap, which is represented as a solid sleeve proxy."
            ),
        )
        ctx.expect_within(
            end_plate,
            hinge_frame,
            axes="yz",
            inner_elem="wrist_pin",
            outer_elem=cap_name,
            margin=0.0,
            name=f"{cap_name} centers the captured wrist pin",
        )
        ctx.expect_overlap(
            hinge_frame,
            end_plate,
            axes="x",
            elem_a=cap_name,
            elem_b="wrist_pin",
            min_overlap=0.020,
            name=f"{cap_name} has axial bearing engagement",
        )

    ctx.expect_gap(
        slide,
        column,
        axis="z",
        positive_elem="bottom_impact_pad",
        negative_elem="lower_stop",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower stop seats the slide at zero lift",
    )
    ctx.expect_gap(
        column,
        slide,
        axis="y",
        positive_elem="front_guide",
        negative_elem="rear_bearing",
        min_gap=0.001,
        max_gap=0.006,
        name="rear bearing clears the fixed guide face",
    )
    ctx.expect_within(
        slide,
        column,
        axes="x",
        inner_elem="main_slide",
        outer_elem="front_guide",
        margin=0.0,
        name="slide is centered between machined guide rails",
    )
    ctx.expect_overlap(
        slide,
        column,
        axes="z",
        elem_a="rear_bearing",
        elem_b="front_guide",
        min_overlap=0.42,
        name="guide support overlaps the slide through the lower position",
    )
    ctx.expect_gap(
        slide,
        hinge_frame,
        axis="z",
        positive_elem="main_slide",
        negative_elem="upper_mount",
        max_gap=0.001,
        max_penetration=1e-6,
        name="hinge frame is seated to the bottom of the moving slide",
    )
    ctx.expect_gap(
        column,
        end_plate,
        axis="y",
        positive_elem="front_guide",
        negative_elem="tool_plate",
        min_gap=0.020,
        name="end plate clears the guide housing when vertical",
    )

    rest_slide_pos = ctx.part_world_position(slide)
    with ctx.pose({lift: 0.32}):
        raised_slide_pos = ctx.part_world_position(slide)
        ctx.expect_gap(
            column,
            slide,
            axis="z",
            positive_elem="upper_stop",
            negative_elem="top_impact_pad",
            max_gap=0.001,
            max_penetration=1e-6,
            name="upper stop catches the lifted slide",
        )
        ctx.expect_gap(
            column,
            slide,
            axis="y",
            positive_elem="front_guide",
            negative_elem="rear_bearing",
            min_gap=0.001,
            max_gap=0.006,
            name="rear bearing clears the guide face at full lift",
        )
        ctx.expect_overlap(
            slide,
            column,
            axes="z",
            elem_a="rear_bearing",
            elem_b="front_guide",
            min_overlap=0.42,
            name="guide support overlaps the slide at full lift",
        )
    ctx.check(
        "prismatic stage raises only along z",
        rest_slide_pos is not None
        and raised_slide_pos is not None
        and raised_slide_pos[2] > rest_slide_pos[2] + 0.315
        and abs(raised_slide_pos[0] - rest_slide_pos[0]) < 1e-6
        and abs(raised_slide_pos[1] - rest_slide_pos[1]) < 1e-6,
        details=f"rest={rest_slide_pos}, raised={raised_slide_pos}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(end_plate, elem="tool_plate")
    with ctx.pose({wrist: 1.35}):
        swept_plate_aabb = ctx.part_element_world_aabb(end_plate, elem="tool_plate")
        ctx.expect_gap(
            hinge_frame,
            end_plate,
            axis="y",
            positive_elem="rear_tie",
            negative_elem="tool_plate",
            min_gap=0.010,
            name="hinged plate sweeps clear of the rear tie",
        )
        ctx.expect_gap(
            column,
            end_plate,
            axis="y",
            positive_elem="front_guide",
            negative_elem="tool_plate",
            min_gap=0.020,
            name="hinged plate sweeps away from the guide housing",
        )
    ctx.check(
        "wrist hinge swings the plate forward",
        rest_plate_aabb is not None
        and swept_plate_aabb is not None
        and swept_plate_aabb[0][1] < rest_plate_aabb[0][1] - 0.04,
        details=f"rest_aabb={rest_plate_aabb}, swept_aabb={swept_plate_aabb}",
    )

    with ctx.pose({lift: 0.32, wrist: 1.35}):
        ctx.expect_gap(
            column,
            end_plate,
            axis="y",
            positive_elem="front_guide",
            negative_elem="tool_plate",
            min_gap=0.020,
            name="full lift and wrist sweep stay clear of the column",
        )

    return ctx.report()


object_model = build_object_model()
