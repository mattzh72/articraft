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
    model = ArticulatedObject(name="compact_service_portal")

    frame_mat = Material("powder_coated_dark_frame", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_mat = Material("brushed_steel_rails", rgba=(0.70, 0.72, 0.70, 1.0))
    carriage_mat = Material("blue_carriage_casting", rgba=(0.05, 0.22, 0.42, 1.0))
    shoe_mat = Material("black_linear_bearings", rgba=(0.015, 0.016, 0.018, 1.0))
    recess_mat = Material("shadowed_recess", rgba=(0.01, 0.012, 0.014, 1.0))
    stage_mat = Material("safety_orange_lift_stage", rgba=(0.95, 0.45, 0.08, 1.0))
    wear_mat = Material("dark_wear_strips", rgba=(0.04, 0.045, 0.05, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.22, 0.28, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=frame_mat,
        name="floor_plinth",
    )
    for x in (-0.55, 0.55):
        frame.visual(
            Box((0.08, 0.14, 0.78)),
            origin=Origin(xyz=(x, 0.0, 0.45)),
            material=frame_mat,
            name=f"upright_{'negative' if x < 0 else 'positive'}",
        )
    frame.visual(
        Box((1.18, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
        material=frame_mat,
        name="top_beam",
    )
    for z, name in ((0.40, "lower_rail"), (0.64, "upper_rail")):
        frame.visual(
            Cylinder(radius=0.025, length=1.08),
            origin=Origin(xyz=(0.0, -0.020, z), rpy=(0.0, 1.57079632679, 0.0)),
            material=rail_mat,
            name=name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.46, 0.07, 0.36)),
        origin=Origin(),
        material=carriage_mat,
        name="carriage_plate",
    )
    for z, name in ((-0.12, "lower_bearing"), (0.12, "upper_bearing")):
        carriage.visual(
            Box((0.30, 0.035, 0.050)),
            origin=Origin(xyz=(0.0, -0.0525, z)),
            material=shoe_mat,
            name=name,
        )
    carriage.visual(
        Box((0.31, 0.006, 0.30)),
        origin=Origin(xyz=(0.0, 0.038, 0.0)),
        material=recess_mat,
        name="front_recess",
    )
    for x, name in ((-0.17, "guide_0"), (0.17, "guide_1")):
        carriage.visual(
            Box((0.025, 0.030, 0.33)),
            origin=Origin(xyz=(x, 0.050, 0.0)),
            material=shoe_mat,
            name=name,
        )

    stage = model.part("stage")
    stage.visual(
        Box((0.23, 0.035, 0.20)),
        origin=Origin(),
        material=stage_mat,
        name="stage_panel",
    )
    stage.visual(
        Box((0.18, 0.012, 0.15)),
        origin=Origin(xyz=(0.0, 0.0235, 0.0)),
        material=wear_mat,
        name="service_plate",
    )
    for x, name in ((-0.092, "wear_strip_0"), (0.092, "wear_strip_1")):
        stage.visual(
            Box((0.012, 0.014, 0.18)),
            origin=Origin(xyz=(x, 0.0365, 0.0)),
            material=wear_mat,
            name=name,
        )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(-0.23, 0.075, 0.52)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.30, lower=0.0, upper=0.46),
    )

    model.articulation(
        "carriage_to_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0585, -0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    stage = object_model.get_part("stage")
    horizontal = object_model.get_articulation("frame_to_carriage")
    vertical = object_model.get_articulation("carriage_to_stage")

    ctx.check(
        "two prismatic service axes",
        horizontal.articulation_type == ArticulationType.PRISMATIC
        and vertical.articulation_type == ArticulationType.PRISMATIC,
        details=f"horizontal={horizontal.articulation_type}, vertical={vertical.articulation_type}",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "frame is grounded",
        frame_aabb is not None and abs(frame_aabb[0][2]) < 1.0e-6,
        details=f"frame_aabb={frame_aabb}",
    )

    ctx.expect_gap(
        carriage,
        frame,
        axis="y",
        positive_elem="lower_bearing",
        negative_elem="lower_rail",
        max_gap=0.0015,
        max_penetration=0.00001,
        name="lower bearing sits on lower rail tangent",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="y",
        positive_elem="upper_bearing",
        negative_elem="upper_rail",
        max_gap=0.0015,
        max_penetration=0.00001,
        name="upper bearing sits on upper rail tangent",
    )
    ctx.expect_gap(
        stage,
        carriage,
        axis="y",
        positive_elem="stage_panel",
        negative_elem="front_recess",
        min_gap=0.0,
        max_gap=0.0005,
        name="vertical stage is seated against the carriage face",
    )
    ctx.expect_within(
        stage,
        carriage,
        axes="x",
        inner_elem="stage_panel",
        outer_elem="front_recess",
        margin=0.0,
        name="vertical stage is narrower than the face recess",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({horizontal: 0.46}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            frame,
            axes="x",
            inner_elem="lower_bearing",
            outer_elem="lower_rail",
            margin=0.0,
            name="horizontal carriage remains captured on the rail",
        )
    ctx.check(
        "carriage travels horizontally",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.40,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_stage = ctx.part_world_position(stage)
    with ctx.pose({vertical: 0.16}):
        raised_stage = ctx.part_world_position(stage)
        ctx.expect_gap(
            stage,
            carriage,
            axis="y",
            positive_elem="stage_panel",
            negative_elem="front_recess",
            min_gap=0.0,
            max_gap=0.0005,
            name="raised stage stays seated against the carriage face",
        )
    ctx.check(
        "stage travels vertically",
        rest_stage is not None and raised_stage is not None and raised_stage[2] > rest_stage[2] + 0.14,
        details=f"rest={rest_stage}, raised={raised_stage}",
    )

    return ctx.report()


object_model = build_object_model()
