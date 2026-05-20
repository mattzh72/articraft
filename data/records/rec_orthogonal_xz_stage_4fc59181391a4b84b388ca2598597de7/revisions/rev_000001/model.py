from __future__ import annotations

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
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lift_transfer_stage")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.05, 0.20, 0.36, 1.0))
    polished = model.material("polished_steel", rgba=(0.74, 0.76, 0.73, 1.0))
    bronze = model.material("bronze_wear_pad", rgba=(0.72, 0.48, 0.18, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.93, 0.36, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.015, 1.0))

    # The fixed datum is a compact machine beam with two linear ways on top.
    # +X is the transfer direction, -Y is the front face, and +Z is lift.
    base = model.part("base_beam")
    base.visual(
        Box((0.72, 0.18, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="beam",
    )
    for rail_name, y in (("x_rail_0", -0.052), ("x_rail_1", 0.052)):
        base.visual(
            Box((0.66, 0.026, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.0925)),
            material=polished,
            name=rail_name,
        )
    for i, x in enumerate((-0.335, 0.335)):
        base.visual(
            Box((0.026, 0.17, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.1075)),
            material=dark_steel,
            name=f"end_stop_{i}",
        )
    for i, x in enumerate((-0.26, 0.26)):
        base.visual(
            Box((0.12, 0.23, 0.022)),
            origin=Origin(xyz=(x, 0.0, 0.011)),
            material=rubber,
            name=f"foot_{i}",
        )

    saddle = model.part("saddle")
    # The saddle frame sits just above the base rails.  Bearing pads are kept
    # with a small visible running clearance from the fixed rails.
    for pad_name, y in (("bearing_pad_0", -0.052), ("bearing_pad_1", 0.052)):
        saddle.visual(
            Box((0.165, 0.032, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.006)),
            material=bronze,
            name=pad_name,
        )
    saddle.visual(
        Box((0.260, 0.220, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=blued_steel,
        name="saddle_deck",
    )
    saddle.visual(
        Box((0.210, 0.026, 0.300)),
        origin=Origin(xyz=(0.0, -0.123, 0.202)),
        material=blued_steel,
        name="front_guide_face",
    )
    for i, x in enumerate((-0.084, 0.084)):
        saddle.visual(
            Box((0.035, 0.075, 0.085)),
            origin=Origin(xyz=(x, -0.095, 0.0955)),
            material=blued_steel,
            name=f"guide_gusset_{i}",
        )
    for rod_name, x in (("z_guide_rod_0", -0.074), ("z_guide_rod_1", 0.074)):
        saddle.visual(
            Cylinder(radius=0.008, length=0.280),
            origin=Origin(xyz=(x, -0.143, 0.203)),
            material=polished,
            name=rod_name,
        )
    saddle.visual(
        Box((0.185, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.147, 0.058)),
        material=dark_steel,
        name="lower_z_stop",
    )
    saddle.visual(
        Box((0.185, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.147, 0.348)),
        material=dark_steel,
        name="upper_z_stop",
    )

    ram = model.part("ram_plate")
    ram.visual(
        Box((0.112, 0.020, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=safety_orange,
        name="ram_plate",
    )
    for i, x in enumerate((-0.071, 0.071)):
        ram.visual(
            Box((0.035, 0.014, 0.150)),
            origin=Origin(xyz=(x, -0.002, 0.113)),
            material=bronze,
            name=f"ram_shoe_{i}",
        )
    ram.visual(
        Box((0.140, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, -0.006, 0.020)),
        material=safety_orange,
        name="transfer_lip",
    )
    ram.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(xyz=(0.0, -0.085, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_tool_pin",
    )

    model.articulation(
        "base_to_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=saddle,
        origin=Origin(xyz=(-0.18, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.32),
    )
    model.articulation(
        "saddle_to_ram",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=ram,
        origin=Origin(xyz=(0.0, -0.164, 0.073)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.25, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_beam")
    saddle = object_model.get_part("saddle")
    ram = object_model.get_part("ram_plate")
    x_slide = object_model.get_articulation("base_to_saddle")
    z_slide = object_model.get_articulation("saddle_to_ram")

    ctx.check(
        "lower stage is prismatic along X",
        tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and x_slide.motion_limits is not None
        and x_slide.motion_limits.upper is not None
        and x_slide.motion_limits.upper >= 0.30,
        details=f"axis={x_slide.axis}, limits={x_slide.motion_limits}",
    )
    ctx.check(
        "upper stage is prismatic along Z",
        tuple(z_slide.axis) == (0.0, 0.0, 1.0)
        and z_slide.motion_limits is not None
        and z_slide.motion_limits.upper is not None
        and z_slide.motion_limits.upper >= 0.15,
        details=f"axis={z_slide.axis}, limits={z_slide.motion_limits}",
    )

    ctx.expect_gap(
        saddle,
        base,
        axis="z",
        positive_elem="bearing_pad_0",
        negative_elem="x_rail_0",
        min_gap=0.0,
        max_gap=0.0002,
        name="saddle pad rides on base rail",
    )
    ctx.expect_overlap(
        saddle,
        base,
        axes="xy",
        elem_a="bearing_pad_0",
        elem_b="x_rail_0",
        min_overlap=0.020,
        name="saddle pad remains over base rail",
    )
    ctx.expect_gap(
        saddle,
        ram,
        axis="y",
        positive_elem="z_guide_rod_0",
        negative_elem="ram_plate",
        min_gap=0.001,
        max_gap=0.004,
        name="ram runs just in front of guide rods",
    )
    ctx.expect_overlap(
        ram,
        saddle,
        axes="z",
        elem_a="ram_plate",
        elem_b="z_guide_rod_0",
        min_overlap=0.18,
        name="lowered ram is captured by guide rods",
    )

    rest_saddle_pos = ctx.part_world_position(saddle)
    with ctx.pose({x_slide: x_slide.motion_limits.upper}):
        extended_saddle_pos = ctx.part_world_position(saddle)
        ctx.expect_overlap(
            saddle,
            base,
            axes="xy",
            elem_a="bearing_pad_0",
            elem_b="x_rail_0",
            min_overlap=0.020,
            name="extended saddle still bears on rail",
        )
    ctx.check(
        "lower stage travels in positive X",
        rest_saddle_pos is not None
        and extended_saddle_pos is not None
        and extended_saddle_pos[0] > rest_saddle_pos[0] + 0.25,
        details=f"rest={rest_saddle_pos}, extended={extended_saddle_pos}",
    )

    rest_ram_pos = ctx.part_world_position(ram)
    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        raised_ram_pos = ctx.part_world_position(ram)
        ctx.expect_overlap(
            ram,
            saddle,
            axes="z",
            elem_a="ram_plate",
            elem_b="z_guide_rod_0",
            min_overlap=0.08,
            name="raised ram stays engaged with guide rods",
        )
    ctx.check(
        "upper stage travels in positive Z",
        rest_ram_pos is not None
        and raised_ram_pos is not None
        and raised_ram_pos[2] > rest_ram_pos[2] + 0.12,
        details=f"rest={rest_ram_pos}, raised={raised_ram_pos}",
    )

    return ctx.report()


object_model = build_object_model()
