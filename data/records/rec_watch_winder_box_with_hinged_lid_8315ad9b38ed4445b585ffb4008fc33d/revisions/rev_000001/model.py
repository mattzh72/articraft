from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


W = 0.34
D = 0.26
BODY_H = 0.15
BASE_T = 0.020
WALL_T = 0.022
LID_T = 0.024
LID_W = 0.38
LID_D = 0.30
LID_RAIL = 0.040
HINGE_Y = D / 2.0 + 0.006
HINGE_Z = BODY_H + 0.008
CRADLE_Y = 0.025
CRADLE_Z = 0.085


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("dark_walnut", rgba=(0.20, 0.105, 0.045, 1.0))
    endgrain = model.material("endgrain_walnut", rgba=(0.13, 0.070, 0.032, 1.0))
    velvet = model.material("black_velvet", rgba=(0.008, 0.007, 0.006, 1.0))
    glass = model.material("smoked_glass", rgba=(0.08, 0.10, 0.11, 0.42))
    brass = model.material("brushed_brass", rgba=(0.82, 0.58, 0.26, 1.0))
    leather = model.material("cream_leather", rgba=(0.72, 0.64, 0.51, 1.0))

    body = model.part("body")

    # Thick presentation-box shell: each wall slightly embeds in the base so the
    # root reads as one heavy wooden carcass, not separate planks.
    body.visual(
        Box((W, D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=walnut,
        name="base_slab",
    )
    body.visual(
        Box((W, WALL_T, BODY_H)),
        origin=Origin(xyz=(0.0, D / 2.0 - WALL_T / 2.0, BODY_H / 2.0)),
        material=walnut,
        name="rear_wall",
    )
    body.visual(
        Box((W, WALL_T, BODY_H)),
        origin=Origin(xyz=(0.0, -D / 2.0 + WALL_T / 2.0, BODY_H / 2.0)),
        material=walnut,
        name="front_wall",
    )
    body.visual(
        Box((WALL_T, D, BODY_H)),
        origin=Origin(xyz=(-W / 2.0 + WALL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=walnut,
        name="side_wall_0",
    )
    body.visual(
        Box((WALL_T, D, BODY_H)),
        origin=Origin(xyz=(W / 2.0 - WALL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=walnut,
        name="side_wall_1",
    )
    body.visual(
        Box((W - 2.0 * WALL_T, D - 2.0 * WALL_T, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T + 0.005)),
        material=velvet,
        name="velvet_floor",
    )

    # Rear padded mounting board and a deep, fixed frame around the rotating
    # cradle.  The bars run back into the board, giving the moving cradle a
    # heavy bezel while keeping a clear aperture.
    body.visual(
        Box((0.270, 0.014, 0.125)),
        origin=Origin(xyz=(0.0, 0.092, 0.083)),
        material=velvet,
        name="back_panel",
    )
    frame_y = 0.054
    frame_d = 0.066
    frame_w = 0.220
    frame_h = 0.120
    frame_open_w = 0.145
    frame_open_h = 0.090
    side_bar = (frame_w - frame_open_w) / 2.0
    top_bar = (frame_h - frame_open_h) / 2.0
    body.visual(
        Box((frame_w, frame_d, top_bar)),
        origin=Origin(xyz=(0.0, frame_y, CRADLE_Z + frame_h / 2.0 - top_bar / 2.0)),
        material=endgrain,
        name="top_bezel",
    )
    body.visual(
        Box((frame_w, frame_d, top_bar)),
        origin=Origin(xyz=(0.0, frame_y, CRADLE_Z - frame_h / 2.0 + top_bar / 2.0)),
        material=endgrain,
        name="bottom_bezel",
    )
    body.visual(
        Box((side_bar, frame_d, frame_h)),
        origin=Origin(xyz=(-frame_w / 2.0 + side_bar / 2.0, frame_y, CRADLE_Z)),
        material=endgrain,
        name="side_bezel_0",
    )
    body.visual(
        Box((side_bar, frame_d, frame_h)),
        origin=Origin(xyz=(frame_w / 2.0 - side_bar / 2.0, frame_y, CRADLE_Z)),
        material=endgrain,
        name="side_bezel_1",
    )

    body.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.0, 0.055, CRADLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="spindle",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.084, CRADLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="spindle_boss",
    )

    # External hinge support fixed to the rear wall.  It sits behind the lid's
    # rear edge so the closed lid clears the bracket but the horizontal hinge
    # axis remains visible.
    body.visual(
        Box((0.255, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, D / 2.0 + 0.006, BODY_H - 0.010)),
        material=brass,
        name="hinge_leaf",
    )
    body.visual(
        Box((0.255, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, D / 2.0 + 0.018, BODY_H - 0.004)),
        material=brass,
        name="hinge_stand",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.255),
        origin=Origin(xyz=(0.0, D / 2.0 + 0.024, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="hinge_pin",
    )

    lid = model.part("lid")
    lid_center_z = BODY_H - HINGE_Z + LID_T / 2.0
    lid.visual(
        Box((LID_W, LID_RAIL, LID_T)),
        origin=Origin(xyz=(0.0, -LID_D + LID_RAIL / 2.0, lid_center_z)),
        material=walnut,
        name="front_rail",
    )
    lid.visual(
        Box((LID_W, LID_RAIL, LID_T)),
        origin=Origin(xyz=(0.0, -LID_RAIL / 2.0, lid_center_z)),
        material=walnut,
        name="rear_rail",
    )
    lid.visual(
        Box((LID_RAIL, LID_D, LID_T)),
        origin=Origin(xyz=(-LID_W / 2.0 + LID_RAIL / 2.0, -LID_D / 2.0, lid_center_z)),
        material=walnut,
        name="side_rail_0",
    )
    lid.visual(
        Box((LID_RAIL, LID_D, LID_T)),
        origin=Origin(xyz=(LID_W / 2.0 - LID_RAIL / 2.0, -LID_D / 2.0, lid_center_z)),
        material=walnut,
        name="side_rail_1",
    )
    lid.visual(
        Box((LID_W - 2.0 * LID_RAIL + 0.006, LID_D - 2.0 * LID_RAIL + 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -LID_D / 2.0, lid_center_z - 0.004)),
        material=glass,
        name="glass_panel",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=velvet,
        name="backplate",
    )
    cradle.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hub_cap",
    )
    pillow_shape = (
        cq.Workplane("XY")
        .box(0.062, 0.030, 0.044)
        .edges()
        .fillet(0.006)
    )
    cradle.visual(
        mesh_from_cadquery(pillow_shape, "watch_pillow", tolerance=0.0007),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=leather,
        name="watch_pillow",
    )
    cradle.visual(
        Box((0.013, 0.006, 0.060)),
        origin=Origin(xyz=(-0.034, -0.0405, 0.0)),
        material=brass,
        name="strap_guard_0",
    )
    cradle.visual(
        Box((0.013, 0.006, 0.060)),
        origin=Origin(xyz=(0.034, -0.0405, 0.0)),
        material=brass,
        name="strap_guard_1",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.35),
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, CRADLE_Y, CRADLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_joint = object_model.get_articulation("body_to_lid")
    cradle_joint = object_model.get_articulation("body_to_cradle")

    ctx.check(
        "lid uses rear revolute hinge",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lid_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"type={lid_joint.articulation_type}, axis={lid_joint.axis}",
    )
    ctx.check(
        "cradle uses continuous spindle",
        cradle_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(cradle_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={cradle_joint.articulation_type}, axis={cradle_joint.axis}",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="front_rail",
        negative_elem="front_wall",
        min_gap=0.0,
        max_gap=0.001,
        name="closed lid rests on front wall",
    )
    ctx.expect_gap(
        body,
        cradle,
        axis="y",
        positive_elem="spindle",
        negative_elem="backplate",
        min_gap=0.0,
        max_gap=0.003,
        name="cradle is seated on spindle end",
    )
    ctx.expect_within(
        cradle,
        body,
        axes="xz",
        inner_elem="backplate",
        outer_elem="back_panel",
        margin=0.0,
        name="rotating cradle sits within rear presentation panel",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    rest_pillow = ctx.part_element_world_aabb(cradle, elem="watch_pillow")
    with ctx.pose({lid_joint: 1.10}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    ctx.check(
        "lid opens upward from rear hinge",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.12,
        details=f"closed={closed_front}, open={open_front}",
    )

    with ctx.pose({cradle_joint: math.pi / 2.0}):
        turned_pillow = ctx.part_element_world_aabb(cradle, elem="watch_pillow")
    ctx.check(
        "cradle rotates about spindle",
        rest_pillow is not None
        and turned_pillow is not None
        and (turned_pillow[1][2] - turned_pillow[0][2])
        > (rest_pillow[1][2] - rest_pillow[0][2]) + 0.010,
        details=f"rest={rest_pillow}, turned={turned_pillow}",
    )

    return ctx.report()


object_model = build_object_model()
