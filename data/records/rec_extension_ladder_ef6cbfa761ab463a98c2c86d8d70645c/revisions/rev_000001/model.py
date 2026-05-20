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


BASE_LENGTH = 3.90
BASE_RAIL_Y = 0.205
BASE_RAIL_SIZE = (0.028, 0.036, BASE_LENGTH)
BASE_RUNG_RADIUS = 0.015
BASE_RUNG_LENGTH = 0.390
BASE_RUNG_START_Z = 0.30
BASE_RUNG_SPACING = 0.30
BASE_RUNG_COUNT = 12

FLY_LENGTH = 3.40
FLY_RAIL_Y = 0.185
FLY_RAIL_SIZE = (0.022, 0.030, FLY_LENGTH)
FLY_OFFSET_X = 0.052
FLY_OFFSET_Z = 0.92
FLY_TRAVEL = 1.25
FLY_RUNG_RADIUS = 0.013
FLY_RUNG_LENGTH = 0.360
FLY_RUNG_START_Z = 0.34
FLY_RUNG_SPACING = 0.30
FLY_RUNG_COUNT = 10

GUIDE_ZS = (0.96, 3.10)
GUIDE_BRIDGE_X = 0.024
GUIDE_BRIDGE_CENTER_X = 0.026
GUIDE_BRIDGE_Y = 0.064
GUIDE_BRIDGE_Z = 0.13
GUIDE_KEEPER_X = 0.012
GUIDE_KEEPER_Y = 0.006
GUIDE_KEEPER_Z = 0.13
GUIDE_KEEPER_CENTER_X = 0.033
GUIDE_SIDE_CLEARANCE = 0.003

FOOT_PIVOT_X = 0.024
FOOT_PIVOT_Z = 0.018
FOOT_LIMITS = MotionLimits(effort=60.0, velocity=1.6, lower=-0.85, upper=0.55)


def add_rungs(
    part,
    *,
    prefix: str,
    count: int,
    start_z: float,
    spacing: float,
    radius: float,
    length: float,
    x: float,
    material: str,
) -> None:
    for index in range(count):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=(x, 0.0, start_z + spacing * index),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_{index}",
        )


def add_base_guides(part, *, material: str) -> None:
    keeper_offset = FLY_RAIL_SIZE[1] / 2.0 + GUIDE_SIDE_CLEARANCE + GUIDE_KEEPER_Y / 2.0
    for rail_index, fly_y in enumerate((-FLY_RAIL_Y, FLY_RAIL_Y)):
        sign = -1.0 if fly_y < 0.0 else 1.0
        for guide_index, guide_z in enumerate(GUIDE_ZS):
            part.visual(
                Box((GUIDE_BRIDGE_X, GUIDE_BRIDGE_Y, GUIDE_BRIDGE_Z)),
                origin=Origin(
                    xyz=(GUIDE_BRIDGE_CENTER_X, fly_y + sign * 0.010, guide_z),
                ),
                material=material,
                name=f"guide_{rail_index}_{guide_index}_bridge",
            )
            part.visual(
                Box((GUIDE_KEEPER_X, GUIDE_KEEPER_Y, GUIDE_KEEPER_Z)),
                origin=Origin(
                    xyz=(GUIDE_KEEPER_CENTER_X, fly_y - sign * keeper_offset, guide_z),
                ),
                material=material,
                name=f"guide_{rail_index}_{guide_index}_inner",
            )
            part.visual(
                Box((GUIDE_KEEPER_X, GUIDE_KEEPER_Y, GUIDE_KEEPER_Z)),
                origin=Origin(
                    xyz=(GUIDE_KEEPER_CENTER_X, fly_y + sign * keeper_offset, guide_z),
                ),
                material=material,
                name=f"guide_{rail_index}_{guide_index}_outer",
            )


def add_foot_mounts(part, *, material: str) -> None:
    cheek_thickness = 0.007
    cheek_size = (0.018, cheek_thickness, 0.058)
    cheek_offset = BASE_RAIL_SIZE[1] / 2.0 + cheek_thickness / 2.0 - 0.002
    for rail_index, rail_y in enumerate((-BASE_RAIL_Y, BASE_RAIL_Y)):
        for side_index, side_sign in enumerate((-1.0, 1.0)):
            part.visual(
                Box(cheek_size),
                origin=Origin(
                    xyz=(
                        0.014,
                        rail_y + side_sign * cheek_offset,
                        0.022,
                    ),
                ),
                material=material,
                name=f"foot_mount_{rail_index}_{side_index}",
            )


def add_foot_geometry(part, *, material: str, tread_material: str) -> None:
    part.visual(
        Cylinder(radius=0.0085, length=0.033),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="pivot_barrel",
    )
    part.visual(
        Box((0.030, 0.020, 0.030)),
        origin=Origin(xyz=(0.015, 0.0, -0.010)),
        material=material,
        name="pivot_web",
    )
    part.visual(
        Box((0.018, 0.034, 0.056)),
        origin=Origin(xyz=(0.036, 0.0, -0.030)),
        material=material,
        name="support_arm",
    )
    part.visual(
        Box((0.120, 0.070, 0.020)),
        origin=Origin(xyz=(0.055, 0.0, -0.066)),
        material=material,
        name="shoe_body",
    )
    part.visual(
        Box((0.108, 0.070, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, -0.073)),
        material=tread_material,
        name="shoe",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    guide_pad = model.material("guide_pad", rgba=(0.18, 0.18, 0.20, 1.0))

    base = model.part("base_section")
    for rail_index, rail_y in enumerate((-BASE_RAIL_Y, BASE_RAIL_Y)):
        base.visual(
            Box(BASE_RAIL_SIZE),
            origin=Origin(xyz=(0.0, rail_y, BASE_LENGTH / 2.0)),
            material=aluminum,
            name=f"rail_{rail_index}",
        )
    add_rungs(
        base,
        prefix="rung",
        count=BASE_RUNG_COUNT,
        start_z=BASE_RUNG_START_Z,
        spacing=BASE_RUNG_SPACING,
        radius=BASE_RUNG_RADIUS,
        length=BASE_RUNG_LENGTH,
        x=-0.004,
        material=steel.name,
    )
    add_base_guides(base, material=guide_pad.name)
    add_foot_mounts(base, material=steel.name)

    fly = model.part("fly_section")
    for rail_index, rail_y in enumerate((-FLY_RAIL_Y, FLY_RAIL_Y)):
        fly.visual(
            Box(FLY_RAIL_SIZE),
            origin=Origin(xyz=(0.0, rail_y, FLY_LENGTH / 2.0)),
            material=aluminum,
            name=f"rail_{rail_index}",
        )
    add_rungs(
        fly,
        prefix="rung",
        count=FLY_RUNG_COUNT,
        start_z=FLY_RUNG_START_Z,
        spacing=FLY_RUNG_SPACING,
        radius=FLY_RUNG_RADIUS,
        length=FLY_RUNG_LENGTH,
        x=0.0,
        material=steel.name,
    )
    for cap_index, rail_y in enumerate((-FLY_RAIL_Y, FLY_RAIL_Y)):
        fly.visual(
            Box((0.024, 0.032, 0.036)),
            origin=Origin(xyz=(0.0, rail_y, FLY_LENGTH - 0.018)),
            material=guide_pad.name,
            name=f"cap_{cap_index}",
        )

    foot_0 = model.part("foot_0")
    add_foot_geometry(foot_0, material=steel.name, tread_material=rubber.name)

    foot_1 = model.part("foot_1")
    add_foot_geometry(foot_1, material=steel.name, tread_material=rubber.name)

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(FLY_OFFSET_X, 0.0, FLY_OFFSET_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.45,
            lower=0.0,
            upper=FLY_TRAVEL,
        ),
    )
    model.articulation(
        "base_to_foot_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=foot_0,
        origin=Origin(xyz=(FOOT_PIVOT_X, -BASE_RAIL_Y, FOOT_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=FOOT_LIMITS,
    )
    model.articulation(
        "base_to_foot_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=foot_1,
        origin=Origin(xyz=(FOOT_PIVOT_X, BASE_RAIL_Y, FOOT_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=FOOT_LIMITS,
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) / 2.0 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    foot_0 = object_model.get_part("foot_0")
    foot_1 = object_model.get_part("foot_1")
    fly_slide = object_model.get_articulation("base_to_fly")

    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=2.9,
        name="collapsed fly remains deeply nested in the base section",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="y",
        min_overlap=0.34,
        name="fly section stays aligned between the base rails",
    )
    ctx.expect_origin_gap(
        fly,
        base,
        axis="x",
        min_gap=0.04,
        max_gap=0.07,
        name="fly section sits ahead of the base rails",
    )

    rest_position = ctx.part_world_position(fly)
    with ctx.pose({fly_slide: FLY_TRAVEL}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.6,
            name="extended fly retains insertion in the guide path",
        )
        extended_position = ctx.part_world_position(fly)

    ctx.check(
        "fly extends upward along the ladder",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 1.0,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    left_rest = _aabb_center(ctx.part_element_world_aabb(foot_0, elem="shoe"))
    right_rest = _aabb_center(ctx.part_element_world_aabb(foot_1, elem="shoe"))
    with ctx.pose(base_to_foot_0=-0.65):
        left_swung = _aabb_center(ctx.part_element_world_aabb(foot_0, elem="shoe"))
        right_still = _aabb_center(ctx.part_element_world_aabb(foot_1, elem="shoe"))
    with ctx.pose(base_to_foot_1=0.40):
        left_still = _aabb_center(ctx.part_element_world_aabb(foot_0, elem="shoe"))
        right_swung = _aabb_center(ctx.part_element_world_aabb(foot_1, elem="shoe"))

    left_shift = None
    right_shift = None
    left_cross_shift = None
    right_cross_shift = None
    if left_rest is not None and left_swung is not None:
        left_shift = abs(left_swung[0] - left_rest[0]) + abs(left_swung[2] - left_rest[2])
    if right_rest is not None and right_swung is not None:
        right_shift = abs(right_swung[0] - right_rest[0]) + abs(right_swung[2] - right_rest[2])
    if left_rest is not None and left_still is not None:
        left_cross_shift = abs(left_still[0] - left_rest[0]) + abs(left_still[2] - left_rest[2])
    if right_rest is not None and right_still is not None:
        right_cross_shift = abs(right_still[0] - right_rest[0]) + abs(right_still[2] - right_rest[2])

    ctx.check(
        "foot 0 pivots without dragging foot 1",
        left_shift is not None
        and right_cross_shift is not None
        and left_shift > 0.02
        and right_cross_shift < 1e-5,
        details=f"left_shift={left_shift}, right_cross_shift={right_cross_shift}",
    )
    ctx.check(
        "foot 1 pivots without dragging foot 0",
        right_shift is not None
        and left_cross_shift is not None
        and right_shift > 0.02
        and left_cross_shift < 1e-5,
        details=f"right_shift={right_shift}, left_cross_shift={left_cross_shift}",
    )

    return ctx.report()


object_model = build_object_model()
