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
    model = ArticulatedObject(name="hinged_extension_service_module")

    dark_steel = model.material("dark_steel", color=(0.09, 0.10, 0.11, 1.0))
    cast_metal = model.material("cast_metal", color=(0.42, 0.45, 0.47, 1.0))
    rail_black = model.material("rail_black", color=(0.015, 0.017, 0.020, 1.0))
    slide_chrome = model.material("slide_chrome", color=(0.72, 0.76, 0.78, 1.0))
    rubber_blue = model.material("rubber_blue", color=(0.05, 0.22, 0.65, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.34, 0.24, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="base_plate",
    )
    support.visual(
        Cylinder(radius=0.047, length=0.215),
        origin=Origin(xyz=(0.0, 0.0, 0.1325)),
        material=cast_metal,
        name="pedestal",
    )
    support.visual(
        Cylinder(radius=0.077, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.244)),
        material=dark_steel,
        name="lower_bearing",
    )
    support.visual(
        Cylinder(radius=0.073, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.316)),
        material=dark_steel,
        name="upper_bearing",
    )
    support.visual(
        Cylinder(radius=0.024, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        material=slide_chrome,
        name="hinge_pin",
    )
    for ix, x in enumerate((-0.125, 0.125)):
        for iy, y in enumerate((-0.080, 0.080)):
            support.visual(
                Cylinder(radius=0.014, length=0.009),
                origin=Origin(xyz=(x, y, 0.0285)),
                material=cast_metal,
                name=f"mount_bolt_{ix}_{iy}",
            )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.066, length=0.055),
        origin=Origin(),
        material=cast_metal,
        name="hinge_hub",
    )
    arm.visual(
        Box((0.540, 0.116, 0.020)),
        origin=Origin(xyz=(0.350, 0.0, 0.040)),
        material=rail_black,
        name="upper_rail",
    )
    arm.visual(
        Box((0.540, 0.116, 0.020)),
        origin=Origin(xyz=(0.350, 0.0, -0.040)),
        material=rail_black,
        name="lower_rail",
    )
    arm.visual(
        Box((0.540, 0.020, 0.070)),
        origin=Origin(xyz=(0.350, 0.058, 0.0)),
        material=rail_black,
        name="side_rail_0",
    )
    arm.visual(
        Box((0.540, 0.020, 0.070)),
        origin=Origin(xyz=(0.350, -0.058, 0.0)),
        material=rail_black,
        name="side_rail_1",
    )
    arm.visual(
        Box((0.055, 0.128, 0.050)),
        origin=Origin(xyz=(0.063, 0.0, 0.0)),
        material=cast_metal,
        name="root_clamp",
    )
    arm.visual(
        Box((0.040, 0.128, 0.018)),
        origin=Origin(xyz=(0.615, 0.0, 0.043)),
        material=cast_metal,
        name="distal_collar_top",
    )
    arm.visual(
        Box((0.040, 0.128, 0.018)),
        origin=Origin(xyz=(0.615, 0.0, -0.043)),
        material=cast_metal,
        name="distal_collar_bottom",
    )
    arm.visual(
        Box((0.040, 0.018, 0.072)),
        origin=Origin(xyz=(0.615, 0.064, 0.0)),
        material=cast_metal,
        name="distal_collar_side_0",
    )
    arm.visual(
        Box((0.040, 0.018, 0.072)),
        origin=Origin(xyz=(0.615, -0.064, 0.0)),
        material=cast_metal,
        name="distal_collar_side_1",
    )

    output = model.part("output_member")
    output.visual(
        Box((0.440, 0.040, 0.060)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
        material=slide_chrome,
        name="slide_bar",
    )
    output.visual(
        Box((0.060, 0.082, 0.062)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=cast_metal,
        name="end_block",
    )
    output.visual(
        Cylinder(radius=0.038, length=0.030),
        origin=Origin(xyz=(0.215, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_blue,
        name="output_pad",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "extension_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=output,
        origin=Origin(xyz=(0.580, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support")
    arm = object_model.get_part("arm")
    output = object_model.get_part("output_member")
    base_hinge = object_model.get_articulation("base_hinge")
    extension_slide = object_model.get_articulation("extension_slide")

    ctx.allow_overlap(
        support,
        arm,
        elem_a="hinge_pin",
        elem_b="hinge_hub",
        reason="The fixed hinge pin is intentionally captured inside the arm hub bore proxy.",
    )
    ctx.expect_within(
        support,
        arm,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="hinge_hub",
        margin=0.002,
        name="hinge pin is concentric with arm hub",
    )
    ctx.expect_overlap(
        support,
        arm,
        axes="z",
        elem_a="hinge_pin",
        elem_b="hinge_hub",
        min_overlap=0.050,
        name="hinge pin spans the captured hub",
    )

    ctx.expect_within(
        output,
        arm,
        axes="yz",
        inner_elem="slide_bar",
        margin=0.004,
        name="slide bar stays inside guide envelope",
    )
    ctx.expect_overlap(
        output,
        arm,
        axes="x",
        elem_a="slide_bar",
        min_overlap=0.250,
        name="retracted output remains deeply guided",
    )

    retracted_pos = ctx.part_world_position(output)
    with ctx.pose({extension_slide: 0.220}):
        ctx.expect_within(
            output,
            arm,
            axes="yz",
            inner_elem="slide_bar",
            margin=0.004,
            name="extended slide remains on arm centerline",
        )
        ctx.expect_overlap(
            output,
            arm,
            axes="x",
            elem_a="slide_bar",
            min_overlap=0.090,
            name="extended output keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(output)
    ctx.check(
        "extension slide moves outward along arm",
        retracted_pos is not None
        and extended_pos is not None
        and extended_pos[0] > retracted_pos[0] + 0.200,
        details=f"retracted={retracted_pos}, extended={extended_pos}",
    )

    distal_rest = ctx.part_element_world_aabb(arm, elem="distal_collar_top")
    with ctx.pose({base_hinge: 0.80}):
        distal_swept = ctx.part_element_world_aabb(arm, elem="distal_collar_top")
    rest_y = None if distal_rest is None else (distal_rest[0][1] + distal_rest[1][1]) / 2.0
    swept_y = None if distal_swept is None else (distal_swept[0][1] + distal_swept[1][1]) / 2.0
    ctx.check(
        "base hinge sweeps arm about vertical pin",
        rest_y is not None and swept_y is not None and swept_y > rest_y + 0.30,
        details=f"rest_y={rest_y}, swept_y={swept_y}",
    )

    return ctx.report()


object_model = build_object_model()
