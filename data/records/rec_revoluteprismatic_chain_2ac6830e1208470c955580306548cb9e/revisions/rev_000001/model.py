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


WALL_TO_PIVOT_X = 0.16
MAIN_LINK_LENGTH = 0.70
SLIDE_ORIGIN_X = 0.55
TIP_TRAVEL = 0.20


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_hinged_support_arm")

    powder_black = Material("powder_black", rgba=(0.035, 0.038, 0.042, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.55, 0.57, 0.56, 1.0))
    anodized_link = Material("anodized_link", rgba=(0.36, 0.42, 0.46, 1.0))
    carriage_orange = Material("carriage_orange", rgba=(0.95, 0.47, 0.13, 1.0))
    rubber_black = Material("rubber_black", rgba=(0.01, 0.012, 0.014, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.035, 0.24, 0.30)),
        origin=Origin(xyz=(-WALL_TO_PIVOT_X, 0.0, 0.0)),
        material=powder_black,
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.095, 0.120, 0.110)),
        origin=Origin(xyz=(-0.096, 0.0, 0.0)),
        material=dark_steel,
        name="root_standoff",
    )
    wall_bracket.visual(
        Box((0.130, 0.120, 0.020)),
        origin=Origin(xyz=(0.005, 0.0, 0.040)),
        material=dark_steel,
        name="upper_hinge_ear",
    )
    wall_bracket.visual(
        Box((0.130, 0.120, 0.020)),
        origin=Origin(xyz=(0.005, 0.0, -0.040)),
        material=dark_steel,
        name="lower_hinge_ear",
    )
    wall_bracket.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=brushed_steel,
        name="upper_pin_cap",
    )
    wall_bracket.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=brushed_steel,
        name="lower_pin_cap",
    )
    for screw_i, (y, z) in enumerate(
        ((-0.075, -0.095), (-0.075, 0.095), (0.075, -0.095), (0.075, 0.095))
    ):
        wall_bracket.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(-0.141, y, z), rpy=(0.0, 1.57079632679, 0.0)),
            material=brushed_steel,
            name=f"wall_screw_{screw_i}",
        )

    main_link = model.part("main_link")
    main_link.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_bushing",
    )
    main_link.visual(
        Box((MAIN_LINK_LENGTH, 0.060, 0.045)),
        origin=Origin(xyz=(0.030 + MAIN_LINK_LENGTH / 2.0, 0.0, 0.0)),
        material=anodized_link,
        name="arm_tube",
    )
    main_link.visual(
        Box((0.34, 0.010, 0.012)),
        origin=Origin(xyz=(0.56, 0.032, 0.0285)),
        material=dark_steel,
        name="slide_rail_0",
    )
    main_link.visual(
        Box((0.34, 0.010, 0.012)),
        origin=Origin(xyz=(0.56, -0.032, 0.0285)),
        material=dark_steel,
        name="slide_rail_1",
    )
    main_link.visual(
        Box((0.08, 0.050, 0.025)),
        origin=Origin(xyz=(0.725, 0.0, 0.012)),
        material=dark_steel,
        name="end_stop_block",
    )

    tip_carriage = model.part("tip_carriage")
    tip_carriage.visual(
        Box((0.52, 0.032, 0.018)),
        origin=Origin(xyz=(0.160, 0.0, 0.0315)),
        material=rubber_black,
        name="sliding_shoe",
    )
    tip_carriage.visual(
        Box((0.20, 0.090, 0.055)),
        origin=Origin(xyz=(0.320, 0.0, 0.0680)),
        material=carriage_orange,
        name="tip_block",
    )
    tip_carriage.visual(
        Box((0.070, 0.105, 0.018)),
        origin=Origin(xyz=(0.430, 0.0, 0.0680)),
        material=dark_steel,
        name="front_clamp_face",
    )

    model.articulation(
        "bracket_to_link",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=main_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "link_to_carriage",
        ArticulationType.PRISMATIC,
        parent=main_link,
        child=tip_carriage,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=TIP_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("wall_bracket")
    link = object_model.get_part("main_link")
    carriage = object_model.get_part("tip_carriage")
    swing = object_model.get_articulation("bracket_to_link")
    slide = object_model.get_articulation("link_to_carriage")

    ctx.check(
        "motion stack is swing first extension second",
        swing.parent == "wall_bracket"
        and swing.child == "main_link"
        and slide.parent == "main_link"
        and slide.child == "tip_carriage",
        details=f"swing=({swing.parent}->{swing.child}), slide=({slide.parent}->{slide.child})",
    )
    ctx.check(
        "link has revolute swing and carriage has prismatic extension",
        swing.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"swing={swing.articulation_type}, slide={slide.articulation_type}",
    )
    ctx.check(
        "prismatic axis follows the main link",
        tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"slide axis={slide.axis}",
    )

    ctx.expect_contact(
        bracket,
        link,
        elem_a="upper_hinge_ear",
        elem_b="pivot_bushing",
        contact_tol=0.001,
        name="hinge ear supports the pivot bushing",
    )
    ctx.expect_contact(
        carriage,
        link,
        elem_a="sliding_shoe",
        elem_b="arm_tube",
        contact_tol=0.001,
        name="carriage shoe sits on the link track",
    )
    ctx.expect_overlap(
        carriage,
        link,
        axes="x",
        elem_a="sliding_shoe",
        elem_b="arm_tube",
        min_overlap=0.20,
        name="collapsed carriage is retained on the link",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: TIP_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            link,
            axes="x",
            elem_a="sliding_shoe",
            elem_b="arm_tube",
            min_overlap=0.06,
            name="extended carriage keeps shoe engaged on the link",
        )
        extended_position = ctx.part_world_position(carriage)

    ctx.check(
        "carriage extends outward along the link",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.18,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    with ctx.pose({swing: 0.75}):
        swung_position = ctx.part_world_position(carriage)
    ctx.check(
        "wall hinge swings the whole downstream carriage",
        rest_position is not None
        and swung_position is not None
        and abs(swung_position[1] - rest_position[1]) > 0.30,
        details=f"rest={rest_position}, swung={swung_position}",
    )

    return ctx.report()


object_model = build_object_model()
