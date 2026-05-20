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
    model = ArticulatedObject(name="drawer_shuttle_inspection_paddle")

    painted_steel = model.material("painted_steel", rgba=(0.12, 0.14, 0.16, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", rgba=(0.05, 0.23, 0.48, 1.0))
    bearing_black = model.material("black_bearing_blocks", rgba=(0.03, 0.035, 0.04, 1.0))
    pivot_steel = model.material("pivot_block_steel", rgba=(0.32, 0.34, 0.36, 1.0))
    link_metal = model.material("brushed_link_metal", rgba=(0.70, 0.71, 0.68, 1.0))
    paddle_orange = model.material("orange_inspection_paddle", rgba=(0.95, 0.46, 0.08, 1.0))
    black_marking = model.material("black_marking", rgba=(0.02, 0.02, 0.02, 1.0))

    guide_frame = model.part("guide_frame")
    guide_frame.visual(
        Box((0.82, 0.24, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=painted_steel,
        name="bed_plate",
    )
    for y, index in [(-0.070, 0), (0.070, 1)]:
        guide_frame.visual(
            Box((0.74, 0.028, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material=painted_steel,
            name=f"rail_pedestal_{index}",
        )
        guide_frame.visual(
            Cylinder(radius=0.014, length=0.72),
            origin=Origin(xyz=(0.0, y, 0.062), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_steel,
            name=f"guide_rail_{index}",
        )
    guide_frame.visual(
        Box((0.028, 0.20, 0.055)),
        origin=Origin(xyz=(-0.386, 0.0, 0.0525)),
        material=painted_steel,
        name="rear_stop",
    )
    guide_frame.visual(
        Box((0.028, 0.20, 0.055)),
        origin=Origin(xyz=(0.386, 0.0, 0.0525)),
        material=painted_steel,
        name="front_stop",
    )
    guide_frame.visual(
        Box((0.66, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.112, 0.031)),
        material=rail_steel,
        name="scale_strip",
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        Box((0.170, 0.170, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=carriage_blue,
        name="shuttle_deck",
    )
    for y, index in [(-0.070, 0), (0.070, 1)]:
        shuttle.visual(
            Box((0.120, 0.034, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.082)),
            material=bearing_black,
            name=f"linear_bearing_{index}",
        )
        shuttle.visual(
            Box((0.120, 0.014, 0.016)),
            origin=Origin(xyz=(0.0, y * 0.74, 0.092)),
            material=carriage_blue,
            name=f"bearing_web_{index}",
        )
    shuttle.visual(
        Box((0.060, 0.052, 0.012)),
        origin=Origin(xyz=(-0.040, 0.0, 0.119)),
        material=rail_steel,
        name="load_pad",
    )
    shuttle.visual(
        Box((0.070, 0.026, 0.055)),
        origin=Origin(xyz=(0.045, 0.095, 0.127)),
        material=pivot_steel,
        name="side_riser",
    )
    shuttle.visual(
        Box((0.060, 0.020, 0.018)),
        origin=Origin(xyz=(0.036, 0.084, 0.106)),
        material=pivot_steel,
        name="riser_foot",
    )
    for y, name in [(0.104, "inner_fork_ear"), (0.152, "outer_fork_ear")]:
        shuttle.visual(
            Box((0.042, 0.012, 0.050)),
            origin=Origin(xyz=(0.055, y, 0.152)),
            material=pivot_steel,
            name=name,
        )
    shuttle.visual(
        Box((0.018, 0.060, 0.036)),
        origin=Origin(xyz=(0.026, 0.128, 0.135)),
        material=pivot_steel,
        name="fork_back_bridge",
    )
    shuttle.visual(
        Cylinder(radius=0.012, length=0.005),
        origin=Origin(xyz=(0.055, 0.0955, 0.152), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="inner_pivot_cap",
    )
    shuttle.visual(
        Cylinder(radius=0.012, length=0.005),
        origin=Origin(xyz=(0.055, 0.1605, 0.152), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="outer_pivot_cap",
    )

    paddle_link = model.part("paddle_link")
    paddle_link.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="hinge_hub",
    )
    paddle_link.visual(
        Box((0.150, 0.018, 0.012)),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material=link_metal,
        name="short_link",
    )
    paddle_link.visual(
        Box((0.072, 0.010, 0.050)),
        origin=Origin(xyz=(0.174, 0.0, 0.0)),
        material=paddle_orange,
        name="paddle_plate",
    )
    paddle_link.visual(
        Box((0.056, 0.003, 0.006)),
        origin=Origin(xyz=(0.174, -0.006, 0.016)),
        material=black_marking,
        name="inspection_mark_top",
    )
    paddle_link.visual(
        Box((0.056, 0.003, 0.006)),
        origin=Origin(xyz=(0.174, -0.006, -0.016)),
        material=black_marking,
        name="inspection_mark_bottom",
    )

    model.articulation(
        "guide_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=guide_frame,
        child=shuttle,
        origin=Origin(xyz=(-0.240, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.300),
    )
    model.articulation(
        "shuttle_to_paddle",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=paddle_link,
        origin=Origin(xyz=(0.055, 0.128, 0.152)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.35, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_frame")
    shuttle = object_model.get_part("shuttle")
    paddle_link = object_model.get_part("paddle_link")
    slide = object_model.get_articulation("guide_to_shuttle")
    hinge = object_model.get_articulation("shuttle_to_paddle")

    ctx.check(
        "shuttle uses prismatic guide joint",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (1.0, 0.0, 0.0)
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper >= 0.25,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check(
        "paddle link uses side pivot revolute joint",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (0.0, -1.0, 0.0)
        and hinge.motion_limits.upper > 1.0,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    for index in (0, 1):
        ctx.expect_gap(
            shuttle,
            guide,
            axis="z",
            positive_elem=f"linear_bearing_{index}",
            negative_elem=f"guide_rail_{index}",
            max_gap=0.002,
            max_penetration=0.0001,
            name=f"bearing_{index} rides on guide rail",
        )
        ctx.expect_overlap(
            shuttle,
            guide,
            axes="x",
            elem_a=f"linear_bearing_{index}",
            elem_b=f"guide_rail_{index}",
            min_overlap=0.10,
            name=f"bearing_{index} has retained rail engagement",
        )

    ctx.expect_contact(
        paddle_link,
        shuttle,
        elem_a="hinge_hub",
        elem_b="inner_fork_ear",
        contact_tol=0.001,
        name="hinge hub bears on inner fork ear",
    )
    ctx.expect_contact(
        paddle_link,
        shuttle,
        elem_a="hinge_hub",
        elem_b="outer_fork_ear",
        contact_tol=0.001,
        name="hinge hub bears on outer fork ear",
    )

    rest_shuttle_pos = ctx.part_world_position(shuttle)
    with ctx.pose({slide: slide.motion_limits.upper}):
        extended_shuttle_pos = ctx.part_world_position(shuttle)
        ctx.expect_overlap(
            shuttle,
            guide,
            axes="x",
            elem_a="linear_bearing_0",
            elem_b="guide_rail_0",
            min_overlap=0.10,
            name="extended shuttle remains engaged on rail",
        )
    ctx.check(
        "shuttle extends along guide axis",
        rest_shuttle_pos is not None
        and extended_shuttle_pos is not None
        and extended_shuttle_pos[0] > rest_shuttle_pos[0] + 0.25,
        details=f"rest={rest_shuttle_pos}, extended={extended_shuttle_pos}",
    )

    rest_paddle_aabb = ctx.part_element_world_aabb(paddle_link, elem="paddle_plate")
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        raised_paddle_aabb = ctx.part_element_world_aabb(paddle_link, elem="paddle_plate")
    rest_paddle_z = None if rest_paddle_aabb is None else (rest_paddle_aabb[0][2] + rest_paddle_aabb[1][2]) * 0.5
    raised_paddle_z = None if raised_paddle_aabb is None else (raised_paddle_aabb[0][2] + raised_paddle_aabb[1][2]) * 0.5
    ctx.check(
        "inspection paddle swings upward from side pivot",
        rest_paddle_z is not None and raised_paddle_z is not None and raised_paddle_z > rest_paddle_z + 0.10,
        details=f"rest_z={rest_paddle_z}, raised_z={raised_paddle_z}",
    )

    return ctx.report()


object_model = build_object_model()
