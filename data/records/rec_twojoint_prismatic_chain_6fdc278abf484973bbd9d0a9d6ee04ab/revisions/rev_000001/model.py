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


GUIDE_RAIL_TOP = 0.085
LARGE_TRAVEL = 0.38
TIP_RAIL_TOP_LOCAL = 0.083
TIP_TRAVEL = 0.15


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_carriage_linear_module")

    dark_anodized = _mat("dark_anodized_aluminum", (0.06, 0.065, 0.07, 1.0))
    black = _mat("black_oxide", (0.01, 0.01, 0.012, 1.0))
    rail_steel = _mat("polished_linear_rail_steel", (0.70, 0.72, 0.74, 1.0))
    blue = _mat("blue_carriage_anodized", (0.05, 0.16, 0.42, 1.0))
    orange = _mat("orange_tip_stage", (0.95, 0.36, 0.08, 1.0))
    screw_dark = _mat("dark_socket_heads", (0.02, 0.02, 0.024, 1.0))

    fixed_guide = model.part("fixed_guide")
    fixed_guide.visual(
        Box((1.00, 0.28, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_anodized,
        name="base_plate",
    )
    for name, y in (("guide_rail_0", -0.075), ("guide_rail_1", 0.075)):
        fixed_guide.visual(
            Box((0.86, 0.035, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0575)),
            material=rail_steel,
            name=name,
        )
    for index, x in enumerate((-0.45, 0.45)):
        fixed_guide.visual(
            Box((0.040, 0.24, 0.085)),
            origin=Origin(xyz=(x, 0.0, 0.0725)),
            material=black,
            name=f"end_stop_{index}",
        )
    fixed_guide.visual(
        Box((0.11, 0.16, 0.11)),
        origin=Origin(xyz=(-0.525, 0.0, 0.085)),
        material=black,
        name="drive_motor",
    )
    fixed_guide.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(-0.470, 0.0, 0.090), rpy=(0.0, 1.57079632679, 0.0)),
        material=rail_steel,
        name="motor_coupler",
    )
    for index, (x, y) in enumerate(
        (
            (-0.34, -0.118),
            (-0.12, -0.118),
            (0.12, -0.118),
            (0.34, -0.118),
            (-0.34, 0.118),
            (-0.12, 0.118),
            (0.12, 0.118),
            (0.34, 0.118),
        )
    ):
        fixed_guide.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, y, 0.032)),
            material=screw_dark,
            name=f"base_bolt_{index}",
        )

    large_stage = model.part("large_stage")
    for name, y in (("lower_shoe_0", -0.075), ("lower_shoe_1", 0.075)):
        large_stage.visual(
            Box((0.28, 0.055, 0.028)),
            origin=Origin(xyz=(0.0, y, 0.014)),
            material=black,
            name=name,
        )
    large_stage.visual(
        Box((0.36, 0.23, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0445)),
        material=blue,
        name="main_carriage_plate",
    )
    for name, y in (("tip_rail_0", -0.045), ("tip_rail_1", 0.045)):
        large_stage.visual(
            Box((0.30, 0.018, 0.022)),
            origin=Origin(xyz=(0.020, y, 0.072)),
            material=rail_steel,
            name=name,
        )
    for index, (x, y) in enumerate(
        (
            (-0.125, -0.090),
            (0.125, -0.090),
            (-0.125, 0.090),
            (0.125, 0.090),
        )
    ):
        large_stage.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(x, y, 0.064)),
            material=screw_dark,
            name=f"carriage_bolt_{index}",
        )
    large_stage.visual(
        Box((0.040, 0.18, 0.024)),
        origin=Origin(xyz=(-0.172, 0.0, 0.040)),
        material=black,
        name="rear_wiper",
    )
    large_stage.visual(
        Box((0.040, 0.18, 0.024)),
        origin=Origin(xyz=(0.172, 0.0, 0.040)),
        material=black,
        name="front_wiper",
    )

    tip_stage = model.part("tip_stage")
    for name, y in (("tip_shoe_0", -0.045), ("tip_shoe_1", 0.045)):
        tip_stage.visual(
            Box((0.14, 0.026, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.009)),
            material=black,
            name=name,
        )
    tip_stage.visual(
        Box((0.22, 0.13, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=orange,
        name="tip_carriage_plate",
    )
    tip_stage.visual(
        Box((0.040, 0.10, 0.050)),
        origin=Origin(xyz=(0.130, 0.0, 0.042)),
        material=orange,
        name="tooling_nose",
    )
    tip_stage.visual(
        Box((0.026, 0.075, 0.032)),
        origin=Origin(xyz=(0.158, 0.0, 0.034)),
        material=rail_steel,
        name="tool_face",
    )
    for index, y in enumerate((-0.035, 0.035)):
        tip_stage.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(0.045, y, 0.0445)),
            material=screw_dark,
            name=f"tip_bolt_{index}",
        )

    model.articulation(
        "guide_to_large",
        ArticulationType.PRISMATIC,
        parent=fixed_guide,
        child=large_stage,
        origin=Origin(xyz=(-0.19, 0.0, GUIDE_RAIL_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=0.0, upper=LARGE_TRAVEL),
    )
    model.articulation(
        "large_to_tip",
        ArticulationType.PRISMATIC,
        parent=large_stage,
        child=tip_stage,
        origin=Origin(xyz=(-0.055, 0.0, TIP_RAIL_TOP_LOCAL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=TIP_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_guide = object_model.get_part("fixed_guide")
    large_stage = object_model.get_part("large_stage")
    tip_stage = object_model.get_part("tip_stage")
    guide_to_large = object_model.get_articulation("guide_to_large")
    large_to_tip = object_model.get_articulation("large_to_tip")

    ctx.expect_gap(
        large_stage,
        fixed_guide,
        axis="z",
        positive_elem="lower_shoe_0",
        negative_elem="guide_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="large shoe rides on fixed guide rail",
    )
    ctx.expect_overlap(
        large_stage,
        fixed_guide,
        axes="xy",
        elem_a="lower_shoe_0",
        elem_b="guide_rail_0",
        min_overlap=0.02,
        name="large carriage retained on guide rail at rest",
    )
    ctx.expect_gap(
        tip_stage,
        large_stage,
        axis="z",
        positive_elem="tip_shoe_0",
        negative_elem="tip_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="tip shoe rides on large-stage rail",
    )
    ctx.expect_overlap(
        tip_stage,
        large_stage,
        axes="xy",
        elem_a="tip_shoe_0",
        elem_b="tip_rail_0",
        min_overlap=0.015,
        name="tip carriage retained on its rail at rest",
    )

    large_rest = ctx.part_world_position(large_stage)
    with ctx.pose({guide_to_large: LARGE_TRAVEL}):
        large_extended = ctx.part_world_position(large_stage)
        ctx.expect_within(
            large_stage,
            fixed_guide,
            axes="x",
            inner_elem="lower_shoe_0",
            outer_elem="guide_rail_0",
            margin=0.002,
            name="large carriage remains supported at full travel",
        )
    ctx.check(
        "large stage translates along guide",
        large_rest is not None
        and large_extended is not None
        and large_extended[0] > large_rest[0] + 0.30,
        details=f"rest={large_rest}, extended={large_extended}",
    )

    tip_rest = ctx.part_world_position(tip_stage)
    with ctx.pose({large_to_tip: TIP_TRAVEL}):
        tip_extended = ctx.part_world_position(tip_stage)
        ctx.expect_within(
            tip_stage,
            large_stage,
            axes="x",
            inner_elem="tip_shoe_0",
            outer_elem="tip_rail_0",
            margin=0.002,
            name="tip carriage remains supported at full travel",
        )
    ctx.check(
        "tip stage translates relative to large stage",
        tip_rest is not None
        and tip_extended is not None
        and tip_extended[0] > tip_rest[0] + 0.12,
        details=f"rest={tip_rest}, extended={tip_extended}",
    )

    return ctx.report()


object_model = build_object_model()
