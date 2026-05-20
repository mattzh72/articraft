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


BASE_LENGTH = 2.85
FLY_LENGTH = 2.45
BASE_RAIL_CENTER = 0.19
FLY_RAIL_CENTER = 0.169
SUPPORT_HINGE_Z = 0.30
FLY_REST_Z = 0.68
FLY_TRAVEL = 1.05


def _add_base_rail(part, side: float, length: float, material: str, prefix: str) -> None:
    rail_index = 0 if side < 0.0 else 1
    web_y = side * BASE_RAIL_CENTER
    lip_y = side * 0.174
    z_mid = length * 0.5

    part.visual(
        Box((0.028, 0.010, length)),
        origin=Origin(xyz=(0.0, web_y, z_mid)),
        material=material,
        name=f"{prefix}_web_{rail_index}",
    )
    for lip_name, x_pos in (("front", 0.0105), ("back", -0.0105)):
        part.visual(
            Box((0.007, 0.022, length)),
            origin=Origin(xyz=(x_pos, lip_y, z_mid)),
            material=material,
            name=f"{prefix}_{lip_name}_{rail_index}",
        )


def _add_fly_rail(part, side: float, length: float, material: str, prefix: str) -> None:
    rail_index = 0 if side < 0.0 else 1
    part.visual(
        Box((0.012, 0.018, length)),
        origin=Origin(xyz=(0.0, side * FLY_RAIL_CENTER, length * 0.5)),
        material=material,
        name=f"{prefix}_{rail_index}",
    )
    part.visual(
        Box((0.016, 0.022, 0.16)),
        origin=Origin(xyz=(0.0, side * FLY_RAIL_CENTER, length - 0.08)),
        material=material,
        name=f"{prefix}_head_{rail_index}",
    )


def _add_rungs(
    part,
    *,
    count: int,
    start_z: float,
    pitch: float,
    depth_x: float,
    width_y: float,
    x_pos: float,
    material: str,
    prefix: str,
) -> None:
    for index in range(count):
        part.visual(
            Box((depth_x, width_y, 0.028)),
            origin=Origin(xyz=(x_pos, 0.0, start_z + pitch * index)),
            material=material,
            name=f"{prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    rail_material = model.material("rail_aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    rung_material = model.material("rung_aluminum", rgba=(0.70, 0.73, 0.77, 1.0))
    hardware_material = model.material("hardware_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    foot_material = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base_section")
    for side in (-1.0, 1.0):
        _add_base_rail(base, side, BASE_LENGTH, rail_material, "rail")
        base.visual(
            Box((0.034, 0.050, 0.030)),
            origin=Origin(xyz=(0.0, side * 0.179, 0.015)),
            material=foot_material,
            name=f"foot_{0 if side < 0.0 else 1}",
        )
        base.visual(
            Box((0.010, 0.018, 0.070)),
            origin=Origin(xyz=(0.016, side * 0.178, SUPPORT_HINGE_Z)),
            material=hardware_material,
            name=f"hinge_plate_{0 if side < 0.0 else 1}",
        )

    _add_rungs(
        base,
        count=9,
        start_z=0.42,
        pitch=0.28,
        depth_x=0.010,
        width_y=0.338,
        x_pos=-0.011,
        material=rung_material,
        prefix="rung",
    )
    base.visual(
        Box((0.018, 0.370, 0.060)),
        origin=Origin(xyz=(-0.011, 0.0, BASE_LENGTH - 0.03)),
        material=hardware_material,
        name="top_cap",
    )

    fly = model.part("fly_section")
    for side in (-1.0, 1.0):
        _add_fly_rail(fly, side, FLY_LENGTH, rail_material, "rail")
    _add_rungs(
        fly,
        count=8,
        start_z=0.30,
        pitch=0.28,
        depth_x=0.010,
        width_y=0.324,
        x_pos=0.011,
        material=rung_material,
        prefix="rung",
    )
    fly.visual(
        Box((0.018, 0.324, 0.050)),
        origin=Origin(xyz=(0.011, 0.0, 0.065)),
        material=hardware_material,
        name="bottom_stile",
    )

    support = model.part("support_bar")
    for side in (-1.0, 1.0):
        hinge_index = 0 if side < 0.0 else 1
        support.visual(
            Cylinder(radius=0.006, length=0.024),
            origin=Origin(
                xyz=(0.0, side * 0.178, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware_material,
            name=f"hinge_{hinge_index}",
        )
        support.visual(
            Box((0.012, 0.020, 0.720)),
            origin=Origin(xyz=(0.0, side * 0.178, 0.360)),
            material=hardware_material,
            name=f"arm_{hinge_index}",
        )
    support.visual(
        Box((0.012, 0.400, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.710)),
        material=hardware_material,
        name="footbar",
    )
    for side in (-1.0, 1.0):
        support.visual(
            Box((0.030, 0.040, 0.016)),
            origin=Origin(xyz=(0.010, side * 0.180, 0.708)),
            material=foot_material,
            name=f"pad_{0 if side < 0.0 else 1}",
        )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, FLY_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=0.0,
            upper=FLY_TRAVEL,
        ),
    )
    model.articulation(
        "support_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=support,
        origin=Origin(xyz=(0.027, 0.0, SUPPORT_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    support = object_model.get_part("support_bar")
    fly_slide = object_model.get_articulation("fly_slide")
    support_hinge = object_model.get_articulation("support_hinge")

    ctx.expect_within(
        fly,
        base,
        axes="y",
        margin=0.0,
        name="fly stays captured between base rails",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=1.90,
        name="collapsed fly keeps substantial overlap with base section",
    )

    rest_fly_pos = ctx.part_world_position(fly)
    rest_footbar = ctx.part_element_world_aabb(support, elem="footbar")
    with ctx.pose({fly_slide: FLY_TRAVEL, support_hinge: 1.95}):
        ctx.expect_within(
            fly,
            base,
            axes="y",
            margin=0.0,
            name="extended fly remains laterally captured by base rails",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.05,
            name="extended fly still retains insertion in base rails",
        )
        extended_fly_pos = ctx.part_world_position(fly)
        deployed_footbar = ctx.part_element_world_aabb(support, elem="footbar")

    ctx.check(
        "fly extends upward",
        rest_fly_pos is not None
        and extended_fly_pos is not None
        and extended_fly_pos[2] > rest_fly_pos[2] + 1.0,
        details=f"rest={rest_fly_pos}, extended={extended_fly_pos}",
    )
    ctx.check(
        "support bar swings outward and downward",
        rest_footbar is not None
        and deployed_footbar is not None
        and deployed_footbar[1][0] > rest_footbar[1][0] + 0.45
        and deployed_footbar[0][2] < rest_footbar[0][2] - 0.60,
        details=f"rest={rest_footbar}, deployed={deployed_footbar}",
    )

    return ctx.report()


object_model = build_object_model()
