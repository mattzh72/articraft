from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="orthogonal_yz_stage")

    frame_mat = model.material("powder_coated_frame", color=(0.08, 0.09, 0.10, 1.0))
    rail_mat = model.material("ground_steel_rails", color=(0.70, 0.72, 0.74, 1.0))
    carriage_mat = model.material("blue_anodized_carriages", color=(0.08, 0.20, 0.48, 1.0))
    bearing_mat = model.material("black_bearing_blocks", color=(0.015, 0.015, 0.018, 1.0))
    plate_mat = model.material("brushed_tool_plate", color=(0.82, 0.78, 0.68, 1.0))
    fastener_mat = model.material("dark_fasteners", color=(0.02, 0.02, 0.025, 1.0))

    back_frame = model.part("back_frame")
    back_frame.visual(
        Box((0.30, 1.50, 0.08)),
        origin=Origin(xyz=(0.05, 0.0, 0.04)),
        material=frame_mat,
        name="floor_foot",
    )
    for y in (-0.68, 0.68):
        back_frame.visual(
            Box((0.10, 0.08, 1.20)),
            origin=Origin(xyz=(0.0, y, 0.665)),
            material=frame_mat,
            name=f"upright_{0 if y < 0 else 1}",
        )
    back_frame.visual(
        Box((0.10, 1.45, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.25)),
        material=frame_mat,
        name="top_crossbar",
    )
    back_frame.visual(
        Box((0.08, 1.38, 0.28)),
        origin=Origin(xyz=(0.04, 0.0, 0.97)),
        material=frame_mat,
        name="rail_backbone",
    )
    for y in (-0.575, 0.575):
        back_frame.visual(
            Box((0.12, 0.06, 0.28)),
            origin=Origin(xyz=(0.06, y, 0.97)),
            material=frame_mat,
            name=f"rail_end_block_{0 if y < 0 else 1}",
        )
    for z, name in ((0.91, "lower_y_rail"), (1.05, "upper_y_rail")):
        back_frame.visual(
            Cylinder(radius=0.016, length=1.10),
            origin=Origin(xyz=(0.095, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rail_mat,
            name=name,
        )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        Box((0.14, 0.24, 0.20)),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        material=carriage_mat,
        name="y_saddle",
    )
    for z, name in ((-0.07, "lower_bearing_pad"), (0.07, "upper_bearing_pad")):
        y_carriage.visual(
            Box((0.030, 0.19, 0.035)),
            origin=Origin(xyz=(-0.054, 0.0, z)),
            material=bearing_mat,
            name=name,
        )
    y_carriage.visual(
        Box((0.055, 0.20, 0.68)),
        origin=Origin(xyz=(0.095, 0.0, -0.20)),
        material=carriage_mat,
        name="z_guide_backbone",
    )
    for y, name in ((-0.065, "z_rail_0"), (0.065, "z_rail_1")):
        y_carriage.visual(
            Cylinder(radius=0.012, length=0.64),
            origin=Origin(xyz=(0.128, y, -0.20)),
            material=rail_mat,
            name=name,
        )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.065, 0.17, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bearing_mat,
        name="z_slider_block",
    )
    z_carriage.visual(
        Box((0.025, 0.20, 0.32)),
        origin=Origin(xyz=(0.035, 0.0, -0.20)),
        material=plate_mat,
        name="hanging_tool_plate",
    )
    z_carriage.visual(
        Box((0.055, 0.12, 0.040)),
        origin=Origin(xyz=(0.055, 0.0, -0.37)),
        material=carriage_mat,
        name="tool_clamp_lip",
    )
    for y, z, name in (
        (-0.055, -0.14, "plate_bolt_0"),
        (0.055, -0.14, "plate_bolt_1"),
        (-0.055, -0.27, "plate_bolt_2"),
        (0.055, -0.27, "plate_bolt_3"),
    ):
        z_carriage.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.0505, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=fastener_mat,
            name=name,
        )

    model.articulation(
        "frame_to_y_carriage",
        ArticulationType.PRISMATIC,
        parent=back_frame,
        child=y_carriage,
        origin=Origin(xyz=(0.18, 0.0, 0.98)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=-0.34, upper=0.34),
    )
    model.articulation(
        "y_carriage_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.1725, 0.0, -0.02)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    back_frame = object_model.get_part("back_frame")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")
    y_slide = object_model.get_articulation("frame_to_y_carriage")
    z_slide = object_model.get_articulation("y_carriage_to_z_carriage")

    ctx.check(
        "stage has two prismatic axes",
        len(object_model.articulations) == 2
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    y_axis = tuple(float(v) for v in y_slide.axis)
    z_axis = tuple(float(v) for v in z_slide.axis)
    ctx.check("Y slide axis is horizontal Y", y_axis == (0.0, 1.0, 0.0), details=str(y_axis))
    ctx.check("Z slide axis is vertical", z_axis == (0.0, 0.0, -1.0), details=str(z_axis))
    ctx.check(
        "slide axes are orthogonal",
        abs(sum(a * b for a, b in zip(y_axis, z_axis))) < 1.0e-9,
        details=f"y_axis={y_axis}, z_axis={z_axis}",
    )

    ctx.expect_gap(
        y_carriage,
        back_frame,
        axis="x",
        positive_elem="upper_bearing_pad",
        negative_elem="upper_y_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="Y carriage bearing is seated on upper rail",
    )
    ctx.expect_gap(
        z_carriage,
        y_carriage,
        axis="x",
        positive_elem="z_slider_block",
        negative_elem="z_rail_0",
        max_gap=0.001,
        max_penetration=1.0e-6,
        name="Z carriage bearing is seated on vertical rail",
    )
    ctx.expect_overlap(
        z_carriage,
        y_carriage,
        axes="z",
        elem_a="z_slider_block",
        elem_b="z_rail_0",
        min_overlap=0.12,
        name="Z slider block remains engaged at rest",
    )

    y_rest = ctx.part_world_position(y_carriage)
    z_rest = ctx.part_world_position(z_carriage)

    with ctx.pose({y_slide: 0.34}):
        y_at_limit = ctx.part_world_position(y_carriage)
        z_with_y = ctx.part_world_position(z_carriage)
    ctx.check(
        "Y carriage travels along +Y",
        y_rest is not None
        and y_at_limit is not None
        and y_at_limit[1] > y_rest[1] + 0.30
        and abs(y_at_limit[0] - y_rest[0]) < 1.0e-6
        and abs(y_at_limit[2] - y_rest[2]) < 1.0e-6,
        details=f"rest={y_rest}, at_limit={y_at_limit}",
    )
    ctx.check(
        "Z carriage follows the Y carriage",
        z_rest is not None and z_with_y is not None and z_with_y[1] > z_rest[1] + 0.30,
        details=f"rest={z_rest}, with_y={z_with_y}",
    )

    with ctx.pose({z_slide: 0.32}):
        z_at_limit = ctx.part_world_position(z_carriage)
        ctx.expect_overlap(
            z_carriage,
            y_carriage,
            axes="z",
            elem_a="z_slider_block",
            elem_b="z_rail_0",
            min_overlap=0.12,
            name="Z slider block remains engaged when lowered",
        )
    ctx.check(
        "Z carriage travels downward",
        z_rest is not None
        and z_at_limit is not None
        and z_at_limit[2] < z_rest[2] - 0.30
        and abs(z_at_limit[0] - z_rest[0]) < 1.0e-6
        and abs(z_at_limit[1] - z_rest[1]) < 1.0e-6,
        details=f"rest={z_rest}, at_limit={z_at_limit}",
    )

    return ctx.report()


object_model = build_object_model()
