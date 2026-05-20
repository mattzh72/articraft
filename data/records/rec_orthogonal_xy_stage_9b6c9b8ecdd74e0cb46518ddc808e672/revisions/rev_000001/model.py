from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="microscope_translation_table")

    dark_cast = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    chrome = model.material("polished_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    blue_slide = model.material("blue_anodized_slide", rgba=(0.08, 0.17, 0.30, 1.0))
    brass_way = model.material("brass_upper_ways", rgba=(0.86, 0.63, 0.25, 1.0))
    graphite = model.material("graphite_saddle", rgba=(0.20, 0.22, 0.23, 1.0))
    satin_top = model.material("satin_perforated_top", rgba=(0.68, 0.70, 0.68, 1.0))
    black_screw = model.material("blackened_screws", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.340, 0.230, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_cast,
        name="base_plate",
    )
    for y, rod_name in ((-0.072, "x_rod_0"), (0.072, "x_rod_1")):
        base.visual(
            Cylinder(radius=0.008, length=0.305),
            origin=Origin(xyz=(0.0, y, 0.042), rpy=(0.0, 1.57079632679, 0.0)),
            material=chrome,
            name=rod_name,
        )
    for x, stop_name in ((-0.162, "x_end_stop_0"), (0.162, "x_end_stop_1")):
        base.visual(
            Box((0.018, 0.205, 0.027)),
            origin=Origin(xyz=(x, 0.0, 0.0485)),
            material=dark_cast,
            name=stop_name,
        )

    lower_slide = model.part("lower_slide")
    lower_slide.visual(
        Box((0.250, 0.172, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=blue_slide,
        name="x_carriage",
    )
    lower_slide.visual(
        Box((0.215, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.072, 0.003)),
        material=chrome,
        name="x_bearing_0",
    )
    lower_slide.visual(
        Box((0.215, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.072, 0.003)),
        material=chrome,
        name="x_bearing_1",
    )
    for x, way_name in ((-0.052, "y_way_0"), (0.052, "y_way_1")):
        lower_slide.visual(
            Box((0.016, 0.164, 0.016)),
            origin=Origin(xyz=(x, 0.0, 0.036)),
            material=brass_way,
            name=way_name,
        )
    lower_slide.visual(
        Box((0.178, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=black_screw,
        name="cross_scale_mark",
    )

    upper_saddle = model.part("upper_saddle")
    upper_saddle.visual(
        Box((0.134, 0.188, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=graphite,
        name="y_saddle",
    )
    upper_saddle.visual(
        Box((0.018, 0.150, 0.006)),
        origin=Origin(xyz=(-0.052, 0.0, 0.003)),
        material=brass_way,
        name="y_bearing_0",
    )
    upper_saddle.visual(
        Box((0.018, 0.150, 0.006)),
        origin=Origin(xyz=(0.052, 0.0, 0.003)),
        material=brass_way,
        name="y_bearing_1",
    )

    top_plate = model.part("top_plate")
    perforated_deck = PerforatedPanelGeometry(
        (0.168, 0.142),
        0.010,
        hole_diameter=0.008,
        pitch=(0.018, 0.018),
        frame=0.015,
        corner_radius=0.006,
        stagger=True,
        center=False,
    )
    top_plate.visual(
        mesh_from_geometry(perforated_deck, "perforated_top_plate"),
        material=satin_top,
        name="perforated_deck",
    )
    for x in (-0.064, 0.064):
        for y in (-0.051, 0.051):
            top_plate.visual(
                Cylinder(radius=0.0042, length=0.003),
                origin=Origin(xyz=(x, y, 0.0105)),
                material=black_screw,
                name=f"deck_screw_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    model.articulation(
        "base_to_lower_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.05, lower=-0.045, upper=0.045),
    )
    model.articulation(
        "lower_slide_to_upper_saddle",
        ArticulationType.PRISMATIC,
        parent=lower_slide,
        child=upper_saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.05, lower=-0.036, upper=0.036),
    )
    model.articulation(
        "upper_saddle_to_top_plate",
        ArticulationType.FIXED,
        parent=upper_saddle,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_slide = object_model.get_part("lower_slide")
    upper_saddle = object_model.get_part("upper_saddle")
    top_plate = object_model.get_part("top_plate")
    x_axis = object_model.get_articulation("base_to_lower_slide")
    y_axis = object_model.get_articulation("lower_slide_to_upper_saddle")

    ctx.expect_gap(
        lower_slide,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="x_carriage",
        negative_elem="x_rod_0",
        name="lower slide rides on the X rod",
    )
    ctx.expect_gap(
        upper_saddle,
        lower_slide,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="y_saddle",
        negative_elem="y_way_0",
        name="upper saddle rides on the Y way",
    )
    ctx.expect_gap(
        top_plate,
        upper_saddle,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="perforated_deck",
        negative_elem="y_saddle",
        name="perforated plate is seated on the saddle",
    )
    ctx.expect_overlap(
        lower_slide,
        base,
        axes="x",
        min_overlap=0.20,
        elem_a="x_carriage",
        elem_b="x_rod_0",
        name="lower X slide has retained rod engagement",
    )
    ctx.expect_overlap(
        upper_saddle,
        lower_slide,
        axes="y",
        min_overlap=0.13,
        elem_a="y_saddle",
        elem_b="y_way_0",
        name="upper Y saddle has retained way engagement",
    )

    rest_lower = ctx.part_world_position(lower_slide)
    rest_upper = ctx.part_world_position(upper_saddle)
    with ctx.pose({x_axis: 0.045, y_axis: 0.036}):
        extended_lower = ctx.part_world_position(lower_slide)
        extended_upper = ctx.part_world_position(upper_saddle)
        ctx.expect_overlap(
            lower_slide,
            base,
            axes="x",
            min_overlap=0.19,
            elem_a="x_carriage",
            elem_b="x_rod_0",
            name="extended X slide stays captured",
        )
        ctx.expect_overlap(
            upper_saddle,
            lower_slide,
            axes="y",
            min_overlap=0.12,
            elem_a="y_saddle",
            elem_b="y_way_0",
            name="extended Y saddle stays captured",
        )

    ctx.check(
        "lower slide moves along positive X",
        rest_lower is not None
        and extended_lower is not None
        and extended_lower[0] > rest_lower[0] + 0.040,
        details=f"rest={rest_lower}, extended={extended_lower}",
    )
    ctx.check(
        "upper saddle moves along positive Y",
        rest_upper is not None
        and extended_upper is not None
        and extended_upper[1] > rest_upper[1] + 0.032,
        details=f"rest={rest_upper}, extended={extended_upper}",
    )

    return ctx.report()


object_model = build_object_model()
