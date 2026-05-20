from __future__ import annotations

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
    model = ArticulatedObject(name="orthogonal_xy_stage")

    dark_anodized = Material("dark_anodized_aluminum", rgba=(0.025, 0.028, 0.032, 1.0))
    brushed_aluminum = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    hardened_steel = Material("hardened_steel", rgba=(0.58, 0.60, 0.57, 1.0))
    black_oxide = Material("black_oxide", rgba=(0.006, 0.006, 0.007, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.60, 0.38, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_anodized,
        name="base_plate",
    )
    for rail_name, cap_name, y in (
        ("x_rail_0", "x_rail_cap_0", -0.12),
        ("x_rail_1", "x_rail_cap_1", 0.12),
    ):
        base.visual(
            Box((0.54, 0.032, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.050)),
            material=hardened_steel,
            name=rail_name,
        )
        base.visual(
            Box((0.54, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.068)),
            material=brushed_aluminum,
            name=cap_name,
        )
    for i, x in enumerate((-0.285, 0.285)):
        base.visual(
            Box((0.018, 0.29, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.049)),
            material=black_oxide,
            name=f"x_end_stop_{i}",
        )

    x_saddle = model.part("x_saddle")
    for bearing_name, x, y in (
        ("x_bearing_0_0", -0.115, -0.12),
        ("x_bearing_0_1", -0.115, 0.12),
        ("x_bearing_1_0", 0.115, -0.12),
        ("x_bearing_1_1", 0.115, 0.12),
    ):
        x_saddle.visual(
            Box((0.110, 0.056, 0.038)),
            origin=Origin(xyz=(x, y, 0.019)),
            material=black_oxide,
            name=bearing_name,
        )
    x_saddle.visual(
        Box((0.38, 0.32, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=brushed_aluminum,
        name="saddle_plate",
    )
    for rail_name, x in (("y_rail_0", -0.105), ("y_rail_1", 0.105)):
        x_saddle.visual(
            Box((0.030, 0.31, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.080)),
            material=hardened_steel,
            name=rail_name,
        )
    for i, y in enumerate((-0.166, 0.166)):
        x_saddle.visual(
            Box((0.28, 0.014, 0.022)),
            origin=Origin(xyz=(0.0, y, 0.077)),
            material=black_oxide,
            name=f"y_end_stop_{i}",
        )

    top_plate = model.part("top_plate")
    for bearing_name, x, y in (
        ("y_bearing_0_0", -0.105, -0.065),
        ("y_bearing_0_1", -0.105, 0.065),
        ("y_bearing_1_0", 0.105, -0.065),
        ("y_bearing_1_1", 0.105, 0.065),
    ):
        top_plate.visual(
            Box((0.052, 0.085, 0.034)),
            origin=Origin(xyz=(x, y, 0.017)),
            material=black_oxide,
            name=bearing_name,
        )
    top_plate.visual(
        Box((0.31, 0.22, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=brushed_aluminum,
        name="work_plate",
    )
    for ix, x in enumerate((-0.115, 0.115)):
        for iy, y in enumerate((-0.075, 0.075)):
            top_plate.visual(
                Cylinder(radius=0.0075, length=0.003),
                origin=Origin(xyz=(x, y, 0.0595)),
                material=black_oxide,
                name=f"cap_screw_{ix}_{iy}",
            )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.20, lower=-0.070, upper=0.070),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=x_saddle,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.16, lower=-0.045, upper=0.045),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_saddle = object_model.get_part("x_saddle")
    top_plate = object_model.get_part("top_plate")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")

    ctx.expect_contact(
        x_saddle,
        base,
        elem_a="x_bearing_0_0",
        elem_b="x_rail_cap_0",
        name="x saddle rides on first base rail",
    )
    ctx.expect_contact(
        x_saddle,
        base,
        elem_a="x_bearing_0_1",
        elem_b="x_rail_cap_1",
        name="x saddle rides on second base rail",
    )
    ctx.expect_contact(
        top_plate,
        x_saddle,
        elem_a="y_bearing_0_0",
        elem_b="y_rail_0",
        name="top plate rides on first saddle rail",
    )
    ctx.expect_contact(
        top_plate,
        x_saddle,
        elem_a="y_bearing_1_0",
        elem_b="y_rail_1",
        name="top plate rides on second saddle rail",
    )

    for q in (-0.070, 0.070):
        with ctx.pose({x_slide: q}):
            ctx.expect_overlap(
                x_saddle,
                base,
                axes="xy",
                min_overlap=0.010,
                elem_a="x_bearing_0_0",
                elem_b="x_rail_cap_0",
                name=f"x rail support retained at q={q}",
            )

    for q in (-0.045, 0.045):
        with ctx.pose({y_slide: q}):
            ctx.expect_overlap(
                top_plate,
                x_saddle,
                axes="xy",
                min_overlap=0.020,
                elem_a="y_bearing_0_0",
                elem_b="y_rail_0",
                name=f"y rail support retained at q={q}",
            )

    x_rest = ctx.part_world_position(x_saddle)
    with ctx.pose({x_slide: 0.070}):
        x_extended = ctx.part_world_position(x_saddle)
    ctx.check(
        "x slide moves saddle along X",
        x_rest is not None and x_extended is not None and x_extended[0] > x_rest[0] + 0.060,
        details=f"rest={x_rest}, extended={x_extended}",
    )

    y_rest = ctx.part_world_position(top_plate)
    with ctx.pose({y_slide: 0.045}):
        y_extended = ctx.part_world_position(top_plate)
    ctx.check(
        "y slide moves top plate along Y",
        y_rest is not None and y_extended is not None and y_extended[1] > y_rest[1] + 0.040,
        details=f"rest={y_rest}, extended={y_extended}",
    )

    return ctx.report()


object_model = build_object_model()
