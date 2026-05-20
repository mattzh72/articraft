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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_hole_punch")

    body_shell = model.material("body_shell", rgba=(0.18, 0.20, 0.22, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    latch_red = model.material("latch_red", rgba=(0.80, 0.18, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.145, 0.102, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=body_shell,
        name="base_plate",
    )
    body.visual(
        Box((0.122, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.031, 0.011)),
        material=body_shell,
        name="front_deck",
    )
    body.visual(
        Box((0.074, 0.054, 0.018)),
        origin=Origin(xyz=(0.0, 0.003, 0.015)),
        material=body_shell,
        name="die_block",
    )
    body.visual(
        Box((0.108, 0.068, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, 0.029)),
        material=body_shell,
        name="housing_cover",
    )
    body.visual(
        Box((0.012, 0.014, 0.010)),
        origin=Origin(xyz=(-0.031, -0.034, 0.039)),
        material=trim_gray,
        name="hinge_tower_0",
    )
    body.visual(
        Box((0.012, 0.014, 0.010)),
        origin=Origin(xyz=(0.031, -0.034, 0.039)),
        material=trim_gray,
        name="hinge_tower_1",
    )
    body.visual(
        Box((0.118, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.035, 0.019)),
        material=trim_gray,
        name="front_guide",
    )
    body.visual(
        Box((0.008, 0.020, 0.012)),
        origin=Origin(xyz=(0.058, -0.002, 0.029)),
        material=trim_gray,
        name="lock_boss",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.0045, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="hinge_barrel",
    )
    lever.visual(
        Box((0.050, 0.124, 0.006)),
        origin=Origin(xyz=(0.0, 0.062, -0.001)),
        material=handle_metal,
        name="handle_plate",
    )
    lever.visual(
        Box((0.010, 0.012, 0.008)),
        origin=Origin(xyz=(-0.016, 0.005, 0.000)),
        material=handle_metal,
        name="rear_cheek_0",
    )
    lever.visual(
        Box((0.010, 0.012, 0.008)),
        origin=Origin(xyz=(0.016, 0.005, 0.000)),
        material=handle_metal,
        name="rear_cheek_1",
    )
    lever.visual(
        Box((0.066, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.104, 0.007)),
        material=handle_metal,
        name="front_grip",
    )

    lock_latch = model.part("lock_latch")
    lock_latch.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=latch_red,
        name="latch_hub",
    )
    lock_latch.visual(
        Box((0.008, 0.012, 0.022)),
        origin=Origin(xyz=(0.006, 0.006, -0.015)),
        material=latch_red,
        name="lock_tab",
    )
    lock_latch.visual(
        Box((0.006, 0.012, 0.006)),
        origin=Origin(xyz=(0.006, 0.014, -0.025)),
        material=latch_red,
        name="lock_hook",
    )

    paper_stop = model.part("paper_stop")
    paper_stop.visual(
        Box((0.028, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_gray,
        name="carriage",
    )
    paper_stop.visual(
        Box((0.006, 0.008, 0.012)),
        origin=Origin(xyz=(-0.010, 0.002, 0.008)),
        material=trim_gray,
        name="post_0",
    )
    paper_stop.visual(
        Box((0.006, 0.008, 0.012)),
        origin=Origin(xyz=(0.010, 0.002, 0.008)),
        material=trim_gray,
        name="post_1",
    )
    paper_stop.visual(
        Box((0.096, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, 0.007)),
        material=handle_metal,
        name="stop_bar",
    )

    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, -0.034, 0.039)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=4.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "body_to_lock_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_latch,
        origin=Origin(xyz=(0.062, -0.002, 0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "body_to_paper_stop",
        ArticulationType.PRISMATIC,
        parent=body,
        child=paper_stop,
        origin=Origin(xyz=(0.0, 0.035, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.15,
            lower=-0.018,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lever = object_model.get_part("lever")
    lock_latch = object_model.get_part("lock_latch")
    paper_stop = object_model.get_part("paper_stop")

    lever_hinge = object_model.get_articulation("body_to_lever")
    latch_pivot = object_model.get_articulation("body_to_lock_latch")
    rail_slide = object_model.get_articulation("body_to_paper_stop")

    ctx.expect_gap(
        lever,
        body,
        axis="z",
        positive_elem="handle_plate",
        negative_elem="housing_cover",
        min_gap=0.0005,
        max_gap=0.008,
        name="closed lever clears the housing cover",
    )
    ctx.expect_overlap(
        lever,
        body,
        axes="xy",
        elem_a="handle_plate",
        elem_b="housing_cover",
        min_overlap=0.045,
        name="lever visually covers the punch housing",
    )
    ctx.expect_gap(
        paper_stop,
        body,
        axis="z",
        positive_elem="carriage",
        negative_elem="front_guide",
        max_gap=0.001,
        max_penetration=0.0,
        name="paper stop carriage rides on the front guide",
    )
    ctx.expect_overlap(
        paper_stop,
        body,
        axes="x",
        elem_a="carriage",
        elem_b="front_guide",
        min_overlap=0.024,
        name="paper stop carriage remains captured on the guide",
    )

    rest_grip_aabb = ctx.part_element_world_aabb(lever, elem="front_grip")
    with ctx.pose({lever_hinge: 1.15}):
        open_grip_aabb = ctx.part_element_world_aabb(lever, elem="front_grip")
    ctx.check(
        "lever opens upward from the rear hinge",
        rest_grip_aabb is not None
        and open_grip_aabb is not None
        and open_grip_aabb[1][2] > rest_grip_aabb[1][2] + 0.040,
        details=f"rest={rest_grip_aabb}, open={open_grip_aabb}",
    )

    rest_latch_aabb = ctx.part_element_world_aabb(lock_latch, elem="lock_hook")
    with ctx.pose({latch_pivot: 1.10}):
        lock_latch_aabb = ctx.part_element_world_aabb(lock_latch, elem="lock_hook")
    ctx.check(
        "side latch rotates upward to a locking position",
        rest_latch_aabb is not None
        and lock_latch_aabb is not None
        and lock_latch_aabb[1][2] > rest_latch_aabb[1][2] + 0.010
        and lock_latch_aabb[1][1] > rest_latch_aabb[1][1] + 0.006,
        details=f"rest={rest_latch_aabb}, locked={lock_latch_aabb}",
    )

    rest_stop_pos = ctx.part_world_position(paper_stop)
    with ctx.pose({rail_slide: 0.018}):
        shifted_stop_pos = ctx.part_world_position(paper_stop)
        ctx.expect_gap(
            paper_stop,
            body,
            axis="z",
            positive_elem="carriage",
            negative_elem="front_guide",
            max_gap=0.001,
            max_penetration=0.0,
            name="paper stop stays seated at full travel",
        )
        ctx.expect_overlap(
            paper_stop,
            body,
            axes="x",
            elem_a="carriage",
            elem_b="front_guide",
            min_overlap=0.024,
            name="paper stop stays retained at full travel",
        )
    ctx.check(
        "paper stop rail slides laterally across the front guide",
        rest_stop_pos is not None
        and shifted_stop_pos is not None
        and shifted_stop_pos[0] > rest_stop_pos[0] + 0.015,
        details=f"rest={rest_stop_pos}, shifted={shifted_stop_pos}",
    )

    return ctx.report()


object_model = build_object_model()
