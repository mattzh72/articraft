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


def _add_channel_rail(
    part,
    *,
    prefix: str,
    x_center: float,
    length: float,
    outer_width: float,
    depth: float,
    wall: float,
    material: str,
) -> None:
    flange_offset = outer_width / 2.0 - wall / 2.0
    part.visual(
        Box((outer_width, wall, length)),
        origin=Origin(xyz=(x_center, -depth / 2.0 + wall / 2.0, length / 2.0)),
        material=material,
        name=f"{prefix}_web",
    )
    part.visual(
        Box((wall, depth, length)),
        origin=Origin(xyz=(x_center - flange_offset, 0.0, length / 2.0)),
        material=material,
        name=f"{prefix}_outer_flange",
    )
    part.visual(
        Box((wall, depth, length)),
        origin=Origin(xyz=(x_center + flange_offset, 0.0, length / 2.0)),
        material=material,
        name=f"{prefix}_inner_flange",
    )


def _add_rungs(
    part,
    *,
    prefix: str,
    count: int,
    start_z: float,
    pitch: float,
    radius: float,
    length: float,
    y: float,
    material: str,
) -> None:
    for index in range(count):
        z_pos = start_z + pitch * index
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(0.0, y, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) / 2.0 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.63, 0.68, 1.0))
    guide_gray = model.material("guide_gray", rgba=(0.26, 0.27, 0.30, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base_length = 3.05
    fly_length = 2.78
    rail_centers = (-0.18, 0.18)
    base_rail_width = 0.060
    base_rail_depth = 0.038
    base_wall = 0.004
    fly_rail_width = 0.044
    fly_rail_depth = 0.018

    base = model.part("base")
    for index, x_center in enumerate(rail_centers):
        _add_channel_rail(
            base,
            prefix=f"base_rail_{index}",
            x_center=x_center,
            length=base_length,
            outer_width=base_rail_width,
            depth=base_rail_depth,
            wall=base_wall,
            material=aluminum,
        )
        base.visual(
            Box((0.056, 0.006, 0.028)),
            origin=Origin(xyz=(x_center, -0.006, -0.010)),
            material=steel,
            name=f"paddle_mount_{index}_rear",
        )
        base.visual(
            Box((0.056, 0.006, 0.028)),
            origin=Origin(xyz=(x_center, 0.014, -0.010)),
            material=steel,
            name=f"paddle_mount_{index}_front",
        )
    _add_rungs(
        base,
        prefix="base_rung",
        count=10,
        start_z=0.33,
        pitch=0.28,
        radius=0.013,
        length=0.316,
        y=-0.002,
        material=steel,
    )

    fly = model.part("fly")
    for index, x_center in enumerate(rail_centers):
        fly.visual(
            Box((fly_rail_width, fly_rail_depth, fly_length)),
            origin=Origin(xyz=(x_center, 0.0, fly_length / 2.0)),
            material=aluminum,
            name=f"fly_rail_{index}",
        )
        for pad_index, pad_z in enumerate((0.25, fly_length - 0.25)):
            fly.visual(
                Box((fly_rail_width, 0.004, 0.18)),
                origin=Origin(xyz=(x_center, -0.011, pad_z)),
                material=guide_gray,
                name=f"guide_pad_{index}_{pad_index}",
            )
    _add_rungs(
        fly,
        prefix="fly_rung",
        count=9,
        start_z=0.18,
        pitch=0.28,
        radius=0.011,
        length=0.324,
        y=0.004,
        material=steel,
    )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.004, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.40,
            lower=0.0,
            upper=1.25,
        ),
    )

    for index, x_center in enumerate(rail_centers):
        paddle = model.part(f"paddle_{index}")
        paddle.visual(
            Cylinder(radius=0.007, length=0.056),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name="pivot_barrel",
        )
        paddle.visual(
            Box((0.050, 0.010, 0.038)),
            origin=Origin(xyz=(0.0, 0.001, -0.023)),
            material=steel,
            name="support_arm",
        )
        paddle.visual(
            Box((0.105, 0.130, 0.008)),
            origin=Origin(xyz=(0.0, 0.015, -0.045)),
            material=rubber,
            name="shoe_plate",
        )
        paddle.visual(
            Box((0.080, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.012, -0.036)),
            material=rubber,
            name="tread_bar_0",
        )
        paddle.visual(
            Box((0.080, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.022, -0.036)),
            material=rubber,
            name="tread_bar_1",
        )

        model.articulation(
            f"base_to_paddle_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=paddle,
            origin=Origin(xyz=(x_center, 0.004, -0.006)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=45.0,
                velocity=2.0,
                lower=-0.75,
                upper=0.55,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fly = object_model.get_part("fly")
    left_paddle = object_model.get_part("paddle_0")
    right_paddle = object_model.get_part("paddle_1")
    slide = object_model.get_articulation("base_to_fly")
    left_pivot = object_model.get_articulation("base_to_paddle_0")
    right_pivot = object_model.get_articulation("base_to_paddle_1")

    ctx.expect_origin_distance(
        fly,
        base,
        axes="x",
        max_dist=0.001,
        name="fly section stays centered laterally",
    )
    ctx.expect_origin_distance(
        fly,
        base,
        axes="y",
        max_dist=0.010,
        name="fly section stays close to the guide plane",
    )

    rest_position = ctx.part_world_position(fly)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=1.38,
            name="fly section retains insertion at full extension",
        )
        extended_position = ctx.part_world_position(fly)

    ctx.check(
        "fly section extends upward",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 1.20,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    left_rest = _aabb_center(ctx.part_element_world_aabb(left_paddle, elem="shoe_plate"))
    right_rest = _aabb_center(ctx.part_element_world_aabb(right_paddle, elem="shoe_plate"))
    with ctx.pose({left_pivot: left_pivot.motion_limits.lower}):
        left_low = _aabb_center(ctx.part_element_world_aabb(left_paddle, elem="shoe_plate"))
        right_still = _aabb_center(ctx.part_element_world_aabb(right_paddle, elem="shoe_plate"))
    ctx.check(
        "left paddle rotates without moving right paddle",
        left_rest is not None
        and left_low is not None
        and right_rest is not None
        and right_still is not None
        and left_low[1] < left_rest[1] - 0.020
        and abs(right_still[1] - right_rest[1]) < 0.001,
        details=(
            f"left_rest={left_rest}, left_low={left_low}, "
            f"right_rest={right_rest}, right_still={right_still}"
        ),
    )
    with ctx.pose({right_pivot: right_pivot.motion_limits.upper}):
        left_still = _aabb_center(ctx.part_element_world_aabb(left_paddle, elem="shoe_plate"))
        right_high = _aabb_center(ctx.part_element_world_aabb(right_paddle, elem="shoe_plate"))
    ctx.check(
        "right paddle rotates without moving left paddle",
        left_rest is not None
        and left_still is not None
        and right_rest is not None
        and right_high is not None
        and right_high[1] > right_rest[1] + 0.012
        and abs(left_still[1] - left_rest[1]) < 0.001,
        details=(
            f"left_rest={left_rest}, left_still={left_still}, "
            f"right_rest={right_rest}, right_high={right_high}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
