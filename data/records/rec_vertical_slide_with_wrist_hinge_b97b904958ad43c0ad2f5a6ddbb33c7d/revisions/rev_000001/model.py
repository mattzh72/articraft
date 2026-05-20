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
    model = ArticulatedObject(name="vertical_tool_slide")

    painted = model.material("machine_blue", rgba=(0.08, 0.19, 0.34, 1.0))
    cast = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    black = model.material("blackened_fasteners", rgba=(0.01, 0.01, 0.01, 1.0))

    column = model.part("column")
    column.visual(
        Box((0.44, 0.34, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=cast,
        name="base_plate",
    )
    column.visual(
        Box((0.10, 0.14, 1.06)),
        origin=Origin(xyz=(-0.07, 0.0, 0.55)),
        material=painted,
        name="upright_column",
    )
    column.visual(
        Box((0.026, 0.018, 0.86)),
        origin=Origin(xyz=(-0.013, -0.055, 0.52)),
        material=steel,
        name="guide_rail_0",
    )
    column.visual(
        Box((0.026, 0.018, 0.86)),
        origin=Origin(xyz=(-0.013, 0.055, 0.52)),
        material=steel,
        name="guide_rail_1",
    )
    column.visual(
        Box((0.16, 0.22, 0.035)),
        origin=Origin(xyz=(-0.035, 0.0, 0.985)),
        material=cast,
        name="top_stop",
    )
    column.visual(
        Box((0.15, 0.22, 0.030)),
        origin=Origin(xyz=(-0.035, 0.0, 0.105)),
        material=cast,
        name="bottom_stop",
    )

    carriage = model.part("carriage")
    for index, y in enumerate((-0.055, 0.055)):
        carriage.visual(
            Box((0.036, 0.046, 0.20)),
            origin=Origin(xyz=(0.018, y, 0.0)),
            material=cast,
            name=f"bearing_block_{index}",
        )
    carriage.visual(
        Box((0.040, 0.170, 0.18)),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material=painted,
        name="front_slide_plate",
    )
    carriage.visual(
        Box((0.045, 0.130, 0.090)),
        origin=Origin(xyz=(0.084, 0.0, 0.030)),
        material=painted,
        name="wrist_mount_boss",
    )
    for index, y in enumerate((-0.068, 0.068)):
        carriage.visual(
            Box((0.090, 0.014, 0.120)),
            origin=Origin(xyz=(0.140, y, 0.030)),
            material=painted,
            name=f"side_bracket_{index}",
        )
        carriage.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(0.150, y * 1.14, 0.030), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"hinge_boss_{index}",
        )
    carriage.visual(
        Cylinder(radius=0.006, length=0.160),
        origin=Origin(xyz=(0.150, 0.0, 0.030), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    wrist_plate = model.part("wrist_plate")
    wrist_plate.visual(
        Cylinder(radius=0.016, length=0.100),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wrist_barrel",
    )
    wrist_plate.visual(
        Box((0.020, 0.082, 0.020)),
        origin=Origin(xyz=(0.018, 0.0, -0.014)),
        material=steel,
        name="barrel_web",
    )
    wrist_plate.visual(
        Box((0.145, 0.082, 0.016)),
        origin=Origin(xyz=(0.095, 0.0, -0.030)),
        material=steel,
        name="tool_plate",
    )
    for index, (x, y) in enumerate(((0.060, -0.024), (0.060, 0.024), (0.132, -0.024), (0.132, 0.024))):
        wrist_plate.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(x, y, -0.0215)),
            material=black,
            name=f"mount_hole_{index}",
        )

    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=0.0, upper=0.35),
    )
    model.articulation(
        "carriage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_plate,
        origin=Origin(xyz=(0.150, 0.0, 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-0.75, upper=0.90),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    wrist_plate = object_model.get_part("wrist_plate")
    slide = object_model.get_articulation("column_to_carriage")
    wrist = object_model.get_articulation("carriage_to_wrist")

    ctx.allow_overlap(
        carriage,
        wrist_plate,
        elem_a="hinge_pin",
        elem_b="wrist_barrel",
        reason="The steel hinge pin is intentionally captured inside the wrist barrel.",
    )

    for index in (0, 1):
        ctx.expect_gap(
            carriage,
            column,
            axis="x",
            positive_elem=f"bearing_block_{index}",
            negative_elem=f"guide_rail_{index}",
            max_gap=0.002,
            max_penetration=0.0,
            name=f"bearing {index} runs on rail without overlap",
        )
        ctx.expect_overlap(
            carriage,
            column,
            axes="z",
            elem_a=f"bearing_block_{index}",
            elem_b=f"guide_rail_{index}",
            min_overlap=0.18,
            name=f"bearing {index} has retained rail engagement",
        )

    ctx.expect_within(
        carriage,
        wrist_plate,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="wrist_barrel",
        margin=0.001,
        name="hinge pin lies inside wrist barrel section",
    )
    ctx.expect_overlap(
        carriage,
        wrist_plate,
        axes="y",
        elem_a="hinge_pin",
        elem_b="wrist_barrel",
        min_overlap=0.095,
        name="hinge pin spans the barrel",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.35}):
        high_pos = ctx.part_world_position(carriage)
        for index in (0, 1):
            ctx.expect_overlap(
                carriage,
                column,
                axes="z",
                elem_a=f"bearing_block_{index}",
                elem_b=f"guide_rail_{index}",
                min_overlap=0.18,
                name=f"raised bearing {index} remains on rail",
            )

    ctx.check(
        "carriage translates upward on the vertical guide",
        rest_pos is not None and high_pos is not None and high_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, high={high_pos}",
    )

    with ctx.pose({wrist: 0.0}):
        level_aabb = ctx.part_world_aabb(wrist_plate)
    with ctx.pose({wrist: 0.90}):
        raised_aabb = ctx.part_world_aabb(wrist_plate)

    ctx.check(
        "wrist plate hinges upward about the front horizontal axis",
        level_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > level_aabb[1][2] + 0.06,
        details=f"level={level_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
