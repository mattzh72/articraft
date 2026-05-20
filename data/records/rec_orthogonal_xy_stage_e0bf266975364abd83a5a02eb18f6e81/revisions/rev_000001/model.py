from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="microscope_xy_translation_table")

    dark_cast = model.material("dark_cast_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.22, 0.25, 0.27, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.72, 0.74, 0.70, 1.0))
    black = model.material("black_knurled_plastic", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.42, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_cast,
        name="base_plate",
    )
    base.visual(
        Box((0.35, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.085, 0.052)),
        material=bright_steel,
        name="base_rail_0",
    )
    base.visual(
        Box((0.35, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.085, 0.052)),
        material=bright_steel,
        name="base_rail_1",
    )
    base.visual(
        Box((0.010, 0.14, 0.04)),
        origin=Origin(xyz=(-0.205, 0.0, 0.06)),
        material=dark_cast,
        name="x_stop_0",
    )
    base.visual(
        Box((0.010, 0.14, 0.04)),
        origin=Origin(xyz=(0.205, 0.0, 0.06)),
        material=dark_cast,
        name="x_stop_1",
    )
    base.visual(
        Box((0.035, 0.045, 0.05)),
        origin=Origin(xyz=(0.195, -0.1225, 0.065)),
        material=dark_cast,
        name="x_knob_bracket",
    )

    lower = model.part("lower_carriage")
    lower.visual(
        Box((0.32, 0.19, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0815)),
        material=blued_steel,
        name="lower_body",
    )
    lower.visual(
        Box((0.34, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.085, 0.070)),
        material=bright_steel,
        name="lower_rail_shoe_0",
    )
    lower.visual(
        Box((0.34, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.085, 0.070)),
        material=bright_steel,
        name="lower_rail_shoe_1",
    )
    lower.visual(
        Box((0.018, 0.165, 0.02)),
        origin=Origin(xyz=(-0.075, 0.0, 0.109)),
        material=bright_steel,
        name="upper_way_0",
    )
    lower.visual(
        Box((0.018, 0.165, 0.02)),
        origin=Origin(xyz=(0.075, 0.0, 0.109)),
        material=bright_steel,
        name="upper_way_1",
    )
    lower.visual(
        Box((0.04, 0.035, 0.04)),
        origin=Origin(xyz=(-0.13, 0.105, 0.119)),
        material=blued_steel,
        name="y_knob_bracket",
    )

    upper = model.part("upper_saddle")
    upper.visual(
        Box((0.18, 0.22, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        material=blued_steel,
        name="upper_body",
    )
    upper.visual(
        Box((0.012, 0.20, 0.012)),
        origin=Origin(xyz=(-0.075, 0.0, 0.125)),
        material=bright_steel,
        name="upper_rail_shoe_0",
    )
    upper.visual(
        Box((0.012, 0.20, 0.012)),
        origin=Origin(xyz=(0.075, 0.0, 0.125)),
        material=bright_steel,
        name="upper_rail_shoe_1",
    )
    upper.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.205, 0.165),
                0.008,
                hole_diameter=0.012,
                pitch=(0.034, 0.034),
                frame=0.024,
                corner_radius=0.004,
                stagger=True,
            ),
            "perforated_stage_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=bright_steel,
        name="top_plate",
    )

    x_knob = model.part("x_knob")
    x_knob.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="knob_grip",
    )
    x_knob.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bright_steel,
        name="lead_screw",
    )

    y_knob = model.part("y_knob")
    y_knob.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_grip",
    )
    y_knob.visual(
        Cylinder(radius=0.005, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="lead_screw",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.08, lower=-0.04, upper=0.04),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=upper,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.08, lower=-0.035, upper=0.035),
    )
    model.articulation(
        "x_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=x_knob,
        origin=Origin(xyz=(0.2125, -0.1225, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "y_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=lower,
        child=y_knob,
        origin=Origin(xyz=(-0.13, 0.1225, 0.135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_carriage")
    upper = object_model.get_part("upper_saddle")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")

    ctx.check(
        "orthogonal prismatic slides",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 3) for v in x_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 3) for v in y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"x_axis={x_slide.axis}, y_axis={y_slide.axis}",
    )

    ctx.expect_contact(
        lower,
        base,
        elem_a="lower_rail_shoe_0",
        elem_b="base_rail_0",
        contact_tol=0.001,
        name="lower carriage sits on first base rail",
    )
    ctx.expect_contact(
        lower,
        base,
        elem_a="lower_rail_shoe_1",
        elem_b="base_rail_1",
        contact_tol=0.001,
        name="lower carriage sits on second base rail",
    )
    ctx.expect_contact(
        upper,
        lower,
        elem_a="upper_rail_shoe_0",
        elem_b="upper_way_0",
        contact_tol=0.001,
        name="upper saddle sits on first cross way",
    )
    ctx.expect_contact(
        upper,
        lower,
        elem_a="upper_rail_shoe_1",
        elem_b="upper_way_1",
        contact_tol=0.001,
        name="upper saddle sits on second cross way",
    )

    rest_lower = ctx.part_world_position(lower)
    rest_upper = ctx.part_world_position(upper)
    with ctx.pose({x_slide: 0.04}):
        moved_lower = ctx.part_world_position(lower)
        ctx.expect_overlap(
            lower,
            base,
            axes="x",
            min_overlap=0.26,
            elem_a="lower_rail_shoe_0",
            elem_b="base_rail_0",
            name="x slide remains captured at travel limit",
        )
    with ctx.pose({y_slide: 0.03}):
        moved_upper = ctx.part_world_position(upper)
        ctx.expect_overlap(
            upper,
            lower,
            axes="y",
            min_overlap=0.13,
            elem_a="upper_rail_shoe_0",
            elem_b="upper_way_0",
            name="y slide remains captured at travel limit",
        )

    ctx.check(
        "lower carriage translates horizontally in x",
        rest_lower is not None
        and moved_lower is not None
        and moved_lower[0] > rest_lower[0] + 0.035
        and abs(moved_lower[1] - rest_lower[1]) < 0.001
        and abs(moved_lower[2] - rest_lower[2]) < 0.001,
        details=f"rest={rest_lower}, moved={moved_lower}",
    )
    ctx.check(
        "upper saddle translates horizontally in y",
        rest_upper is not None
        and moved_upper is not None
        and moved_upper[1] > rest_upper[1] + 0.025
        and abs(moved_upper[0] - rest_upper[0]) < 0.001
        and abs(moved_upper[2] - rest_upper[2]) < 0.001,
        details=f"rest={rest_upper}, moved={moved_upper}",
    )

    return ctx.report()


object_model = build_object_model()
