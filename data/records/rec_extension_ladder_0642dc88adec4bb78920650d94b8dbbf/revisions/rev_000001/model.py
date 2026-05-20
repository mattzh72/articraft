from __future__ import annotations

import math

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


def add_ladder_section(
    part,
    *,
    rail_x: float,
    rail_size: tuple[float, float, float],
    rung_size: tuple[float, float, float],
    rung_z: list[float],
    material: str,
    rail_prefix: str,
    rung_prefix: str,
) -> None:
    rail_height = rail_size[2]
    rail_center_z = rail_height * 0.5

    for index, x_pos in enumerate((-rail_x, rail_x)):
        part.visual(
            Box(rail_size),
            origin=Origin(xyz=(x_pos, 0.0, rail_center_z)),
            material=material,
            name=f"{rail_prefix}_{index}",
        )

    for index, z_pos in enumerate(rung_z):
        part.visual(
            Box(rung_size),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=material,
            name=f"{rung_prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_access_extension_ladder")

    model.material("aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("steel_dark", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    add_ladder_section(
        base,
        rail_x=0.205,
        rail_size=(0.055, 0.028, 4.90),
        rung_size=(0.368, 0.034, 0.040),
        rung_z=[0.42 + 0.33 * index for index in range(13)],
        material="aluminum",
        rail_prefix="base_rail",
        rung_prefix="base_rung",
    )
    for index, x_pos in enumerate((-0.205, 0.205)):
        base.visual(
            Box((0.065, 0.040, 0.100)),
            origin=Origin(xyz=(x_pos, 0.0, 0.050)),
            material="rubber",
            name=f"foot_pad_{index}",
        )
        base.visual(
            Box((0.090, 0.050, 0.120)),
            origin=Origin(xyz=(x_pos, 0.0, 0.160)),
            material="steel_dark",
            name=f"stabilizer_mount_{index}",
        )

    fly = model.part("fly")
    add_ladder_section(
        fly,
        rail_x=0.165,
        rail_size=(0.042, 0.024, 4.25),
        rung_size=(0.302, 0.028, 0.036),
        rung_z=[0.34 + 0.33 * index for index in range(11)],
        material="aluminum",
        rail_prefix="fly_rail",
        rung_prefix="fly_rung",
    )
    fly.visual(
        Box((0.330, 0.024, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 4.20)),
        material="steel_dark",
        name="top_cap",
    )

    roof_hook = model.part("roof_hook")
    roof_hook.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material="steel_dark",
        name="pivot_tube",
    )
    for index, x_pos in enumerate((-0.145, 0.145)):
        roof_hook.visual(
            Box((0.030, 0.022, 0.580)),
            origin=Origin(xyz=(x_pos, 0.0, 0.290)),
            material="steel_dark",
            name=f"hook_leg_{index}",
        )
        roof_hook.visual(
            Box((0.030, 0.280, 0.030)),
            origin=Origin(xyz=(x_pos, 0.140, 0.580)),
            material="steel_dark",
            name=f"hook_crown_{index}",
        )
        roof_hook.visual(
            Box((0.030, 0.022, 0.220)),
            origin=Origin(xyz=(x_pos, 0.270, 0.470)),
            material="steel_dark",
            name=f"hook_tip_{index}",
        )
    roof_hook.visual(
        Cylinder(radius=0.030, length=0.360),
        origin=Origin(xyz=(0.0, 0.295, 0.555), rpy=(0.0, math.pi * 0.5, 0.0)),
        material="rubber",
        name="roller",
    )

    stabilizer = model.part("stabilizer")
    stabilizer.visual(
        Cylinder(radius=0.014, length=0.560),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material="steel_dark",
        name="hinge_tube",
    )
    for index, x_pos in enumerate((-0.240, 0.240)):
        stabilizer.visual(
            Box((0.060, 0.024, 0.250)),
            origin=Origin(xyz=(x_pos, 0.0, 0.125)),
            material="steel_dark",
            name=f"stabilizer_leg_{index}",
        )
        stabilizer.visual(
            Box((0.090, 0.060, 0.060)),
            origin=Origin(xyz=(0.555 if index else -0.555, 0.0, 0.250)),
            material="rubber",
            name=f"stabilizer_foot_{index}",
        )
    stabilizer.visual(
        Box((1.180, 0.050, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material="aluminum",
        name="crossbar",
    )

    fly_slide = model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.031, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=1.55,
        ),
    )
    fly_slide.meta["qc_samples"] = [0.0, 0.80, 1.55]

    roof_hook_hinge = model.articulation(
        "fly_to_roof_hook",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=roof_hook,
        origin=Origin(xyz=(0.0, 0.024, 4.218)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=1.20,
        ),
    )
    roof_hook_hinge.meta["qc_samples"] = [0.0, 0.60, 1.20]

    stabilizer_hinge = model.articulation(
        "base_to_stabilizer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=stabilizer,
        origin=Origin(xyz=(0.0, -0.039, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=0.0,
            upper=1.35,
        ),
    )
    stabilizer_hinge.meta["qc_samples"] = [0.0, 0.80, 1.35]

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return (
            0.5 * (low[0] + high[0]),
            0.5 * (low[1] + high[1]),
            0.5 * (low[2] + high[2]),
        )

    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fly = object_model.get_part("fly")
    roof_hook = object_model.get_part("roof_hook")
    stabilizer = object_model.get_part("stabilizer")
    fly_slide = object_model.get_articulation("base_to_fly")
    roof_hook_hinge = object_model.get_articulation("fly_to_roof_hook")
    stabilizer_hinge = object_model.get_articulation("base_to_stabilizer")

    ctx.expect_within(
        fly,
        base,
        axes="x",
        margin=0.040,
        name="fly stays laterally within the base section",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=2.20,
        name="collapsed fly remains well nested in the base rails",
    )

    rest_pos = ctx.part_world_position(fly)
    fly_upper = fly_slide.motion_limits.upper if fly_slide.motion_limits is not None else 0.0
    with ctx.pose({fly_slide: fly_upper}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=2.00,
            name="extended fly still retains deep insertion in the base section",
        )
        extended_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 1.4,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    folded_roller = aabb_center(ctx.part_element_world_aabb(roof_hook, elem="roller"))
    roof_hook_upper = roof_hook_hinge.motion_limits.upper if roof_hook_hinge.motion_limits is not None else 0.0
    with ctx.pose({roof_hook_hinge: roof_hook_upper}):
        opened_roller = aabb_center(ctx.part_element_world_aabb(roof_hook, elem="roller"))

    ctx.check(
        "roof hook swings out over the roof side",
        folded_roller is not None
        and opened_roller is not None
        and opened_roller[1] > folded_roller[1] + 0.28
        and opened_roller[2] > 4.35,
        details=f"folded={folded_roller}, opened={opened_roller}",
    )

    stowed_bar = aabb_center(ctx.part_element_world_aabb(stabilizer, elem="crossbar"))
    stabilizer_upper = stabilizer_hinge.motion_limits.upper if stabilizer_hinge.motion_limits is not None else 0.0
    with ctx.pose({stabilizer_hinge: stabilizer_upper}):
        deployed_bar = aabb_center(ctx.part_element_world_aabb(stabilizer, elem="crossbar"))

    ctx.check(
        "stabilizer bar folds out from the ladder foot",
        stowed_bar is not None
        and deployed_bar is not None
        and deployed_bar[1] < stowed_bar[1] - 0.20
        and deployed_bar[2] < stowed_bar[2] - 0.14,
        details=f"stowed={stowed_bar}, deployed={deployed_bar}",
    )

    return ctx.report()


object_model = build_object_model()
