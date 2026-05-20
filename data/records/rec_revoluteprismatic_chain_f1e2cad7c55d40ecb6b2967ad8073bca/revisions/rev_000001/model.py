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
    model = ArticulatedObject(name="wall_swing_slide_fixture")

    wall = Material("painted_wall", rgba=(0.82, 0.82, 0.78, 1.0))
    plate_finish = Material("dark_powder_coat", rgba=(0.08, 0.09, 0.10, 1.0))
    worn_steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    ram_chrome = Material("polished_ram", rgba=(0.86, 0.88, 0.86, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.018, 0.42, 0.48)),
        origin=Origin(xyz=(-0.084, 0.0, 0.0)),
        material=wall,
        name="wall_panel",
    )
    backplate.visual(
        Box((0.020, 0.20, 0.32)),
        origin=Origin(xyz=(-0.065, 0.0, 0.0)),
        material=plate_finish,
        name="mounting_plate",
    )
    for z, suffix in ((0.075, "upper"), (-0.075, "lower")):
        backplate.visual(
            Cylinder(radius=0.032, length=0.044),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=worn_steel,
            name=f"{suffix}_hinge_barrel",
        )
        backplate.visual(
            Box((0.062, 0.032, 0.040)),
            origin=Origin(xyz=(-0.030, 0.0, z)),
            material=plate_finish,
            name=f"{suffix}_hinge_web",
        )
    for y, z, suffix in (
        (-0.072, 0.120, "screw_0"),
        (0.072, 0.120, "screw_1"),
        (-0.072, -0.120, "screw_2"),
        (0.072, -0.120, "screw_3"),
    ):
        backplate.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(-0.052, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=worn_steel,
            name=suffix,
        )

    main_link = model.part("main_link")
    main_link.visual(
        Cylinder(radius=0.027, length=0.106),
        origin=Origin(),
        material=worn_steel,
        name="center_hinge_barrel",
    )
    for y, suffix in ((0.034, "strap_0"), (-0.034, "strap_1")):
        main_link.visual(
            Box((0.58, 0.014, 0.036)),
            origin=Origin(xyz=(0.305, y, 0.0)),
            material=plate_finish,
            name=suffix,
        )
    main_link.visual(
        Box((0.040, 0.088, 0.036)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=plate_finish,
        name="hinge_crosshead",
    )
    main_link.visual(
        Box((0.220, 0.094, 0.012)),
        origin=Origin(xyz=(0.600, 0.0, 0.036)),
        material=plate_finish,
        name="sleeve_top",
    )
    main_link.visual(
        Box((0.220, 0.094, 0.012)),
        origin=Origin(xyz=(0.600, 0.0, -0.024)),
        material=plate_finish,
        name="sleeve_bottom",
    )
    for y, suffix in ((0.041, "sleeve_side_0"), (-0.041, "sleeve_side_1")):
        main_link.visual(
            Box((0.220, 0.012, 0.084)),
            origin=Origin(xyz=(0.600, y, 0.0)),
            material=plate_finish,
            name=suffix,
        )

    ram = model.part("ram")
    ram.visual(
        Cylinder(radius=0.018, length=0.340),
        origin=Origin(xyz=(0.170, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=ram_chrome,
        name="shaft",
    )
    ram.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.344, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="nose_pad",
    )

    model.articulation(
        "swing_hinge",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=main_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-0.75, upper=0.95),
    )
    model.articulation(
        "ram_slide",
        ArticulationType.PRISMATIC,
        parent=main_link,
        child=ram,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.140),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    main_link = object_model.get_part("main_link")
    ram = object_model.get_part("ram")
    swing = object_model.get_articulation("swing_hinge")
    slide = object_model.get_articulation("ram_slide")

    ctx.check(
        "main link is revolute",
        swing.articulation_type == ArticulationType.REVOLUTE,
        details=f"found {swing.articulation_type}",
    )
    ctx.check(
        "ram is distal prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"found {slide.articulation_type}",
    )
    ctx.expect_gap(
        main_link,
        backplate,
        axis="x",
        positive_elem="center_hinge_barrel",
        negative_elem="mounting_plate",
        min_gap=0.020,
        max_gap=0.040,
        name="hinge barrel stands proud of wall plate",
    )
    ctx.expect_within(
        ram,
        main_link,
        axes="yz",
        inner_elem="shaft",
        margin=0.002,
        name="ram shaft stays centered in guide envelope",
    )
    ctx.expect_gap(
        main_link,
        ram,
        axis="z",
        positive_elem="sleeve_top",
        negative_elem="shaft",
        min_gap=0.006,
        max_gap=0.020,
        name="ram clears upper guide jaw",
    )
    ctx.expect_overlap(
        ram,
        main_link,
        axes="x",
        elem_a="shaft",
        elem_b="sleeve_top",
        min_overlap=0.18,
        name="collapsed ram remains captured in distal sleeve",
    )

    rest_slide_pos = ctx.part_world_position(ram)
    with ctx.pose({slide: 0.140}):
        extended_slide_pos = ctx.part_world_position(ram)
        ctx.expect_overlap(
            ram,
            main_link,
            axes="x",
            elem_a="shaft",
            elem_b="sleeve_top",
            min_overlap=0.065,
            name="extended ram retains sleeve insertion",
        )
        ctx.expect_gap(
            main_link,
            ram,
            axis="z",
            positive_elem="sleeve_top",
            negative_elem="shaft",
            min_gap=0.006,
            max_gap=0.020,
            name="extended ram clears upper guide jaw",
        )
    ctx.check(
        "prismatic joint drives ram outward",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.12,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    rest_swing_pos = ctx.part_world_position(ram)
    with ctx.pose({swing: 0.90}):
        swung_pos = ctx.part_world_position(ram)
    ctx.check(
        "revolute joint swings distal ram sideways",
        rest_swing_pos is not None
        and swung_pos is not None
        and swung_pos[1] > rest_swing_pos[1] + 0.35,
        details=f"rest={rest_swing_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
