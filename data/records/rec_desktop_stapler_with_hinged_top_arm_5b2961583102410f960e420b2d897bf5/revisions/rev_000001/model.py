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


HINGE_Z = 0.155
ANVIL_X = 0.420


def _aabb_size(aabb):
    if aabb is None:
        return None
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_desktop_stapler")

    body_paint = model.material("body_paint", rgba=(0.13, 0.14, 0.15, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.470, 0.082, 0.010)),
        origin=Origin(xyz=(0.205, 0.0, 0.005)),
        material=body_paint,
        name="base_plate",
    )
    base.visual(Box((0.370, 0.022, 0.014)), origin=Origin(xyz=(0.215, 0.0, 0.041)), material=body_paint, name="lower_beam")
    base.visual(Box((0.085, 0.040, 0.070)), origin=Origin(xyz=(0.050, 0.0, 0.045)), material=body_paint, name="shoulder")
    base.visual(Box((0.050, 0.030, 0.050)), origin=Origin(xyz=(0.420, 0.0, 0.035)), material=body_paint, name="nose_block")
    base.visual(Box((0.220, 0.010, 0.032)), origin=Origin(xyz=(0.180, 0.026, 0.026)), material=body_paint, name="side_rail_0")
    base.visual(Box((0.220, 0.010, 0.032)), origin=Origin(xyz=(0.180, -0.026, 0.026)), material=body_paint, name="side_rail_1")
    base.visual(Box((0.055, 0.014, 0.170)), origin=Origin(xyz=(0.005, 0.032, 0.085)), material=body_paint, name="tower_cheek_0")
    base.visual(Box((0.055, 0.014, 0.170)), origin=Origin(xyz=(0.005, -0.032, 0.085)), material=body_paint, name="tower_cheek_1")
    base.visual(Box((0.016, 0.050, 0.020)), origin=Origin(xyz=(-0.018, 0.0, 0.158)), material=body_paint, name="tower_bridge")

    lever = model.part("lever")
    lever.visual(
        Box((0.385, 0.046, 0.022)),
        origin=Origin(xyz=(0.2225, 0.0, 0.010)),
        material=dark_steel,
        name="lever_assembly",
    )
    lever.visual(Box((0.355, 0.016, 0.014)), origin=Origin(xyz=(0.2275, 0.0, -0.034)), material=bright_steel, name="staple_rail")
    lever.visual(Box((0.050, 0.040, 0.030)), origin=Origin(xyz=(0.015, 0.0, -0.006)), material=dark_steel, name="rear_block")
    lever.visual(Box((0.050, 0.018, 0.042)), origin=Origin(xyz=(0.045, 0.0, -0.010)), material=dark_steel, name="rear_web")
    lever.visual(Box((0.050, 0.022, 0.044)), origin=Origin(xyz=(0.423, 0.0, -0.012)), material=dark_steel, name="nose_piece")

    follower = model.part("follower")
    follower.visual(
        Box((0.028, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=bright_steel,
        name="follower_slider",
    )
    follower.visual(Box((0.018, 0.012, 0.006)), origin=Origin(xyz=(-0.008, 0.0, 0.012)), material=bright_steel, name="follower_grip")

    anvil = model.part("anvil")
    anvil.visual(
        Box((0.024, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=bright_steel,
        name="anvil_plate",
    )
    anvil.visual(Cylinder(radius=0.0035, length=0.003), origin=Origin(xyz=(0.0, 0.0, 0.0045)), material=bright_steel, name="anvil_pivot")

    model.articulation(
        "base_to_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.4,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "lever_to_follower",
        ArticulationType.PRISMATIC,
        parent=lever,
        child=follower,
        origin=Origin(xyz=(0.078, 0.0, 0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=0.075,
        ),
    )
    model.articulation(
        "base_to_anvil",
        ArticulationType.REVOLUTE,
        parent=base,
        child=anvil,
        origin=Origin(xyz=(ANVIL_X, 0.0, 0.0600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lever = object_model.get_part("lever")
    follower = object_model.get_part("follower")
    anvil = object_model.get_part("anvil")

    lever_hinge = object_model.get_articulation("base_to_lever")
    follower_slide = object_model.get_articulation("lever_to_follower")
    anvil_pivot = object_model.get_articulation("base_to_anvil")

    ctx.expect_origin_gap(
        anvil,
        lever,
        axis="x",
        min_gap=0.40,
        max_gap=0.44,
        name="deep throat reaches far in front of the hinge",
    )

    with ctx.pose({lever_hinge: 0.0, follower_slide: 0.0, anvil_pivot: 0.0}):
        ctx.expect_gap(
            lever,
            anvil,
            axis="z",
            min_gap=0.047,
            max_gap=0.075,
            name="closed lever leaves a realistic stapling throat height",
        )
        ctx.expect_overlap(
            follower,
            lever,
            axes="xy",
            min_overlap=0.012,
            name="rear follower sits on the lever rail at rest",
        )
        closed_lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_assembly")
        closed_follower_pos = ctx.part_world_position(follower)
        closed_anvil_aabb = ctx.part_element_world_aabb(anvil, elem="anvil_plate")

    with ctx.pose({lever_hinge: 0.95}):
        open_lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_assembly")

    ctx.check(
        "lever opens upward around the rear hinge",
        closed_lever_aabb is not None
        and open_lever_aabb is not None
        and open_lever_aabb[1][2] > closed_lever_aabb[1][2] + 0.10,
        details=f"closed={closed_lever_aabb}, open={open_lever_aabb}",
    )

    with ctx.pose({follower_slide: 0.075}):
        extended_follower_pos = ctx.part_world_position(follower)
        ctx.expect_overlap(
            follower,
            lever,
            axes="xy",
            min_overlap=0.010,
            name="follower remains guided over the rail at full loading travel",
        )

    ctx.check(
        "follower slides forward along the rail",
        closed_follower_pos is not None
        and extended_follower_pos is not None
        and extended_follower_pos[0] > closed_follower_pos[0] + 0.05,
        details=f"rest={closed_follower_pos}, extended={extended_follower_pos}",
    )

    with ctx.pose({anvil_pivot: math.pi / 2.0}):
        turned_anvil_aabb = ctx.part_element_world_aabb(anvil, elem="anvil_plate")
        turned_anvil_pos = ctx.part_world_position(anvil)

    anvil_rest_size = _aabb_size(closed_anvil_aabb)
    anvil_turn_size = _aabb_size(turned_anvil_aabb)
    anvil_rest_pos = ctx.part_world_position(anvil)
    ctx.check(
        "anvil rotates in place under the nose",
        anvil_rest_size is not None
        and anvil_turn_size is not None
        and anvil_rest_pos is not None
        and turned_anvil_pos is not None
        and abs(turned_anvil_pos[0] - anvil_rest_pos[0]) < 0.001
        and abs(turned_anvil_pos[1] - anvil_rest_pos[1]) < 0.001
        and abs(anvil_turn_size[0] - anvil_rest_size[1]) < 0.006
        and abs(anvil_turn_size[1] - anvil_rest_size[0]) < 0.006,
        details=(
            f"rest_size={anvil_rest_size}, turned_size={anvil_turn_size}, "
            f"rest_pos={anvil_rest_pos}, turned_pos={turned_anvil_pos}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
