from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="box_rail_carriage")

    painted_steel = Material("painted_steel", color=(0.20, 0.23, 0.25, 1.0))
    machined_steel = Material("machined_steel", color=(0.62, 0.64, 0.66, 1.0))
    dark_steel = Material("dark_steel", color=(0.06, 0.07, 0.08, 1.0))
    blue_paint = Material("blue_paint", color=(0.05, 0.19, 0.42, 1.0))
    bronze = Material("bronze_wear", color=(0.78, 0.55, 0.25, 1.0))
    rubber_black = Material("blackened_screw", color=(0.01, 0.01, 0.012, 1.0))

    # Grounded rectangular box rail, roughly the scale of a compact linear stage.
    rail = model.part("rail")
    rail.visual(
        Box((1.25, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=painted_steel,
        name="base_plate",
    )
    rail.visual(
        Box((1.15, 0.12, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=machined_steel,
        name="rail_body",
    )
    rail.visual(
        Box((1.14, 0.085, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=dark_steel,
        name="rail_top_strip",
    )
    for i, x in enumerate((-0.50, -0.25, 0.25, 0.50)):
        for side, y in enumerate((-0.125, 0.125)):
            rail.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(x, y, 0.038)),
                material=dark_steel,
                name=f"mount_bolt_{i}_{side}",
            )

    # The carriage frame is placed at the running surface of the top rail strip.
    # Its bronze pads touch the top and sides, while the blue bridge and cheeks
    # make one wrapped sliding saddle.
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.260, 0.220, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=blue_paint,
        name="bridge",
    )
    for suffix, y in (("0", -0.082), ("1", 0.082)):
        carriage.visual(
            Box((0.260, 0.040, 0.125)),
            origin=Origin(xyz=(0.0, y, -0.0355)),
            material=blue_paint,
            name=f"side_cheek_{suffix}",
        )
    carriage.visual(
        Box((0.225, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.025, 0.010)),
        material=bronze,
        name="top_wear_pad_0",
    )
    carriage.visual(
        Box((0.225, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, 0.025, 0.010)),
        material=bronze,
        name="top_wear_pad_1",
    )
    carriage.visual(
        Box((0.210, 0.003, 0.070)),
        origin=Origin(xyz=(0.0, -0.0615, -0.055)),
        material=bronze,
        name="side_wear_pad_0",
    )
    carriage.visual(
        Box((0.210, 0.003, 0.070)),
        origin=Origin(xyz=(0.0, 0.0615, -0.055)),
        material=bronze,
        name="side_wear_pad_1",
    )
    carriage.visual(
        Box((0.026, 0.210, 0.130)),
        origin=Origin(xyz=(0.137, 0.0, 0.069)),
        material=blue_paint,
        name="front_face",
    )
    carriage.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.152, 0.0, 0.053), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="bearing_cap",
    )

    # Short rotary nose spindle.  The off-axis black set screw makes rotation
    # visually evident rather than a perfectly symmetric cylinder.
    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.026, length=0.100),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.033, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="drive_collar",
    )
    spindle.visual(
        Cylinder(radius=0.018, length=0.048),
        origin=Origin(xyz=(0.124, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_steel,
        name="nose_tip",
    )
    spindle.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.055, 0.0, 0.031)),
        material=rubber_black,
        name="set_screw",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(-0.37, 0.0, 0.143)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.70),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.167, 0.0, 0.053)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("rail_to_carriage")
    spin = object_model.get_articulation("carriage_to_spindle")

    ctx.check(
        "carriage joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "spindle joint is rotary",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ctx.check(
        "slide axis follows box rail",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "spindle axis points out front face",
        tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )

    ctx.expect_contact(
        carriage,
        rail,
        elem_a="top_wear_pad_0",
        elem_b="rail_top_strip",
        contact_tol=0.001,
        name="carriage top pad rides on rail",
    )
    ctx.expect_contact(
        carriage,
        rail,
        elem_a="side_wear_pad_1",
        elem_b="rail_body",
        contact_tol=0.001,
        name="carriage side pad guides rail side",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        elem_a="spindle_shaft",
        elem_b="bearing_cap",
        contact_tol=0.001,
        name="spindle seats on front bearing cap",
    )
    ctx.expect_overlap(
        carriage,
        rail,
        axes="x",
        elem_a="top_wear_pad_0",
        elem_b="rail_top_strip",
        min_overlap=0.20,
        name="carriage has retained rail engagement at rest",
    )
    ctx.expect_within(
        carriage,
        rail,
        axes="y",
        inner_elem="top_wear_pad_0",
        outer_elem="rail_top_strip",
        margin=0.001,
        name="top pad sits within rail strip width",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.70}):
        ctx.expect_overlap(
            carriage,
            rail,
            axes="x",
            elem_a="top_wear_pad_0",
            elem_b="rail_top_strip",
            min_overlap=0.20,
            name="carriage remains on rail at full travel",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along rail",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.65,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    spindle_rest = ctx.part_world_position(spindle)
    with ctx.pose({spin: pi / 2.0}):
        spindle_rotated = ctx.part_world_position(spindle)
        ctx.expect_contact(
            spindle,
            carriage,
            elem_a="spindle_shaft",
            elem_b="bearing_cap",
            contact_tol=0.001,
            name="spindle remains seated while rotated",
        )

    ctx.check(
        "spindle rotates about fixed nose axis",
        spindle_rest is not None
        and spindle_rotated is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(spindle_rest, spindle_rotated)),
        details=f"rest={spindle_rest}, rotated={spindle_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
