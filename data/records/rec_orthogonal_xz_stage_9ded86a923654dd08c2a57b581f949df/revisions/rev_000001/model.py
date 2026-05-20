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
    model = ArticulatedObject(name="orthogonal_xz_stage")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    dark = model.material("black_anodized", rgba=(0.03, 0.035, 0.04, 1.0))
    rail_steel = model.material("ground_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    blue = model.material("blue_carriage", rgba=(0.05, 0.18, 0.42, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    def box_visual(part, name: str, size, xyz, material: Material) -> None:
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    def bolt_head(part, name: str, xyz, radius: float = 0.011) -> None:
        part.visual(
            Cylinder(radius=radius, length=0.006),
            origin=Origin(xyz=xyz),
            material=dark,
            name=name,
        )

    def front_bolt_head(part, name: str, xyz, radius: float = 0.008) -> None:
        part.visual(
            Cylinder(radius=radius, length=0.006),
            origin=Origin(xyz=xyz, rpy=(-1.57079632679, 0.0, 0.0)),
            material=dark,
            name=name,
        )

    base = model.part("base")
    box_visual(base, "base_plate", (1.20, 0.55, 0.040), (0.0, 0.0, 0.020), dark)
    box_visual(base, "rail_0", (1.05, 0.045, 0.050), (0.0, -0.18, 0.065), rail_steel)
    box_visual(base, "rail_1", (1.05, 0.045, 0.050), (0.0, 0.18, 0.065), rail_steel)
    box_visual(base, "rail_stop_0", (0.035, 0.14, 0.060), (-0.545, -0.18, 0.070), aluminum)
    box_visual(base, "rail_stop_1", (0.035, 0.14, 0.060), (0.545, -0.18, 0.070), aluminum)
    box_visual(base, "rail_stop_2", (0.035, 0.14, 0.060), (-0.545, 0.18, 0.070), aluminum)
    box_visual(base, "rail_stop_3", (0.035, 0.14, 0.060), (0.545, 0.18, 0.070), aluminum)
    for i, x in enumerate((-0.48, 0.48)):
        for j, y in enumerate((-0.235, 0.235)):
            box_visual(base, f"foot_{i}_{j}", (0.11, 0.08, 0.030), (x, y, -0.015), rubber)
            bolt_head(base, f"base_bolt_{i}_{j}", (x, y, 0.043), radius=0.010)

    x_carriage = model.part("x_carriage")
    for i, x in enumerate((-0.15, 0.15)):
        for j, y in enumerate((-0.18, 0.18)):
            box_visual(
                x_carriage,
                f"x_block_{i}_{j}",
                (0.16, 0.085, 0.055),
                (x, y, 0.1175),
                aluminum,
            )
    box_visual(x_carriage, "saddle_plate", (0.48, 0.47, 0.045), (0.0, 0.0, 0.1675), blue)
    box_visual(x_carriage, "vertical_backplate", (0.38, 0.040, 0.720), (0.0, -0.215, 0.550), aluminum)
    box_visual(x_carriage, "z_rail_0", (0.040, 0.035, 0.620), (-0.11, -0.1775, 0.550), rail_steel)
    box_visual(x_carriage, "z_rail_1", (0.040, 0.035, 0.620), (0.11, -0.1775, 0.550), rail_steel)
    box_visual(x_carriage, "z_stop_0", (0.35, 0.035, 0.030), (0.0, -0.1775, 0.225), dark)
    box_visual(x_carriage, "z_stop_1", (0.35, 0.035, 0.030), (0.0, -0.1775, 0.875), dark)
    box_visual(x_carriage, "side_bracket_0", (0.050, 0.160, 0.260), (-0.215, -0.155, 0.320), dark)
    box_visual(x_carriage, "side_bracket_1", (0.050, 0.160, 0.260), (0.215, -0.155, 0.320), dark)
    for i, x in enumerate((-0.17, 0.17)):
        for j, y in enumerate((-0.14, 0.14)):
            bolt_head(x_carriage, f"saddle_bolt_{i}_{j}", (x, y, 0.193), radius=0.009)

    lift_slide = model.part("lift_slide")
    box_visual(lift_slide, "guide_block_0", (0.075, 0.055, 0.140), (-0.11, -0.1325, 0.360), aluminum)
    box_visual(lift_slide, "guide_block_1", (0.075, 0.055, 0.140), (0.11, -0.1325, 0.360), aluminum)
    box_visual(lift_slide, "tool_plate", (0.34, 0.035, 0.240), (0.0, -0.0875, 0.360), blue)
    box_visual(lift_slide, "work_shelf", (0.30, 0.220, 0.035), (0.0, 0.040, 0.290), aluminum)
    box_visual(lift_slide, "front_lip", (0.34, 0.025, 0.090), (0.0, 0.1625, 0.3175), dark)
    box_visual(lift_slide, "lower_rib_0", (0.035, 0.170, 0.035), (-0.095, 0.035, 0.255), dark)
    box_visual(lift_slide, "lower_rib_1", (0.035, 0.170, 0.035), (0.095, 0.035, 0.255), dark)
    for i, x in enumerate((-0.115, 0.115)):
        for j, z in enumerate((0.305, 0.415)):
            front_bolt_head(lift_slide, f"tool_bolt_{i}_{j}", (x, -0.067, z), radius=0.008)

    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=-0.25, upper=0.25),
    )

    model.articulation(
        "x_carriage_to_lift_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=lift_slide,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.22, lower=0.0, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    lift_slide = object_model.get_part("lift_slide")
    x_joint = object_model.get_articulation("base_to_x_carriage")
    z_joint = object_model.get_articulation("x_carriage_to_lift_slide")

    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        positive_elem="x_block_0_0",
        negative_elem="rail_0",
        max_gap=0.002,
        max_penetration=0.0005,
        name="lower x bearing block rides on rail 0",
    )
    ctx.expect_gap(
        x_carriage,
        base,
        axis="z",
        positive_elem="x_block_0_1",
        negative_elem="rail_1",
        max_gap=0.002,
        max_penetration=0.0005,
        name="lower x bearing block rides on rail 1",
    )
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="xy",
        elem_a="x_block_0_0",
        elem_b="rail_0",
        min_overlap=0.04,
        name="x bearing footprint stays on rail",
    )

    ctx.expect_gap(
        lift_slide,
        x_carriage,
        axis="y",
        positive_elem="guide_block_0",
        negative_elem="z_rail_0",
        max_gap=0.002,
        max_penetration=0.0005,
        name="lift guide block rides on vertical rail",
    )
    ctx.expect_overlap(
        lift_slide,
        x_carriage,
        axes="z",
        elem_a="guide_block_0",
        elem_b="z_rail_0",
        min_overlap=0.12,
        name="lift block is retained on vertical rail at low position",
    )

    rest_x = ctx.part_world_position(x_carriage)
    with ctx.pose({x_joint: 0.25}):
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            elem_a="x_block_1_0",
            elem_b="rail_0",
            min_overlap=0.12,
            name="x stage remains captured at positive travel",
        )
        positive_x = ctx.part_world_position(x_carriage)
    with ctx.pose({x_joint: -0.25}):
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="x",
            elem_a="x_block_0_0",
            elem_b="rail_0",
            min_overlap=0.12,
            name="x stage remains captured at negative travel",
        )
    ctx.check(
        "horizontal stage moves along x",
        rest_x is not None and positive_x is not None and positive_x[0] > rest_x[0] + 0.20,
        details=f"rest={rest_x}, positive={positive_x}",
    )

    rest_lift = ctx.part_world_position(lift_slide)
    with ctx.pose({z_joint: 0.32}):
        ctx.expect_overlap(
            lift_slide,
            x_carriage,
            axes="z",
            elem_a="guide_block_0",
            elem_b="z_rail_0",
            min_overlap=0.12,
            name="lift block is retained on vertical rail at high position",
        )
        high_lift = ctx.part_world_position(lift_slide)
    ctx.check(
        "lift stage moves upward",
        rest_lift is not None and high_lift is not None and high_lift[2] > rest_lift[2] + 0.30,
        details=f"rest={rest_lift}, high={high_lift}",
    )

    return ctx.report()


object_model = build_object_model()
