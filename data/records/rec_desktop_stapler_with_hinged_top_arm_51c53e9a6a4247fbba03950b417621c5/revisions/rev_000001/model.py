from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.168
BASE_W = 0.041
BASE_T = 0.004
BASE_TOP_Z = -0.018

FRONT_PAD_L = 0.030
FRONT_PAD_W = 0.032
FRONT_PAD_T = 0.0036

PAPER_DECK_L = 0.062
PAPER_DECK_W = 0.020
PAPER_DECK_T = 0.002

REAR_BRIDGE_L = 0.020
REAR_BRIDGE_W = 0.030
REAR_BRIDGE_T = 0.010

POST_L = 0.016
POST_W = 0.006
POST_H = 0.018

HINGE_ROD_R = 0.0025
HINGE_ROD_LEN = 0.031

ARM_W = 0.034
ARM_UPPER_L = 0.118
ARM_UPPER_H = 0.016
ARM_SHOULDER_L = 0.024
ARM_SHOULDER_H = 0.018
MAG_BODY_L = 0.124
MAG_BODY_W = 0.022
MAG_BODY_H = 0.010
NOSE_L = 0.028
NOSE_W = 0.018
NOSE_H = 0.012

HINGE_TUBE_R = 0.0048
HINGE_TUBE_BORE_R = 0.0027
HINGE_TUBE_LEN = 0.027

FOLLOWER_TRAVEL = 0.018
FOLLOWER_JOINT_X = 0.043
FOLLOWER_JOINT_Z = 0.004

CLINCHER_L = 0.018
CLINCHER_W = 0.010
CLINCHER_T = 0.0016
CLINCHER_HOLE_R = 0.00235
CLINCHER_X = 0.147


def _arm_cover_shape() -> cq.Workplane:
    upper = (
        cq.Workplane("XY")
        .box(ARM_UPPER_L, ARM_W, ARM_UPPER_H)
        .edges("|Z")
        .fillet(0.0024)
        .translate((0.076, 0.0, 0.006))
    )
    shoulder = (
        cq.Workplane("XY")
        .box(ARM_SHOULDER_L, ARM_W, ARM_SHOULDER_H)
        .edges("|Z")
        .fillet(0.0018)
        .translate((0.014, 0.0, 0.003))
    )
    magazine = (
        cq.Workplane("XY")
        .box(MAG_BODY_L, MAG_BODY_W, MAG_BODY_H)
        .edges("|Z")
        .fillet(0.0014)
        .translate((0.079, 0.0, -0.006))
    )
    nose = (
        cq.Workplane("XY")
        .box(NOSE_L, NOSE_W, NOSE_H)
        .edges("|Z")
        .fillet(0.0018)
        .translate((0.147, 0.0, -0.006))
    )

    cover = upper.union(shoulder).union(magazine).union(nose)
    cover = cover.cut(cq.Workplane("XY").box(0.094, ARM_W - 0.008, 0.009).translate((0.082, 0.0, 0.006)))
    cover = cover.cut(cq.Workplane("XY").box(0.114, 0.014, 0.0064).translate((0.077, 0.0, -0.0055)))
    cover = cover.cut(cq.Workplane("XY").box(0.056, 0.006, 0.020).translate((0.039, 0.0, 0.006)))
    return cover


def _hinge_tube_shape() -> cq.Workplane:
    tube = cq.Workplane("XZ").circle(HINGE_TUBE_R).extrude(HINGE_TUBE_LEN / 2.0, both=True)
    bore = cq.Workplane("XZ").circle(HINGE_TUBE_BORE_R).extrude((HINGE_TUBE_LEN / 2.0) + 0.001, both=True)
    bridge = cq.Workplane("XY").box(0.010, 0.024, 0.010).translate((0.006, 0.0, -0.001))
    return tube.cut(bore).union(bridge)


def _clincher_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .rect(CLINCHER_L, CLINCHER_W)
        .extrude(CLINCHER_T)
        .edges("|Z")
        .fillet(0.0016)
        .translate((0.0, 0.0, -CLINCHER_T))
    )
    hole = cq.Workplane("XY").circle(CLINCHER_HOLE_R).extrude(CLINCHER_T + 0.001).translate((0.0, 0.0, -CLINCHER_T - 0.0005))
    return plate.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")

    model.material("base_steel", rgba=(0.52, 0.55, 0.58, 1.0))
    model.material("top_graphite", rgba=(0.15, 0.15, 0.17, 1.0))
    model.material("bright_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_L, BASE_W, BASE_T)),
        origin=Origin(xyz=(BASE_L / 2.0, 0.0, BASE_TOP_Z - (BASE_T / 2.0))),
        material="base_steel",
        name="base_plate",
    )
    base.visual(
        Box((FRONT_PAD_L, FRONT_PAD_W, FRONT_PAD_T)),
        origin=Origin(xyz=(BASE_L - (FRONT_PAD_L / 2.0), 0.0, BASE_TOP_Z + (FRONT_PAD_T / 2.0))),
        material="base_steel",
        name="front_pad",
    )
    base.visual(
        Box((PAPER_DECK_L, PAPER_DECK_W, PAPER_DECK_T)),
        origin=Origin(xyz=(0.112, 0.0, BASE_TOP_Z + (PAPER_DECK_T / 2.0))),
        material="bright_steel",
        name="paper_deck",
    )
    base.visual(
        Box((REAR_BRIDGE_L, REAR_BRIDGE_W, REAR_BRIDGE_T)),
        origin=Origin(xyz=(REAR_BRIDGE_L / 2.0, 0.0, (BASE_TOP_Z - BASE_T) + (REAR_BRIDGE_T / 2.0))),
        material="base_steel",
        name="rear_bridge",
    )
    for idx, y in enumerate((-0.0175, 0.0175)):
        base.visual(
            Box((POST_L, POST_W, POST_H)),
            origin=Origin(xyz=(0.004, y, BASE_TOP_Z + (POST_H / 2.0))),
            material="base_steel",
            name=f"hinge_post_{idx}",
        )
    base.visual(
        Cylinder(radius=HINGE_ROD_R, length=HINGE_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="hinge_rod",
    )
    base.visual(
        Cylinder(radius=0.0019, length=0.0022),
        origin=Origin(xyz=(CLINCHER_X, 0.0, -0.0231)),
        material="bright_steel",
        name="pivot_boss",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_cover_shape(), "stapler_arm_cover"),
        material="top_graphite",
        name="arm_cover",
    )
    arm.visual(
        mesh_from_cadquery(_hinge_tube_shape(), "stapler_hinge_tube"),
        material="bright_steel",
        name="hinge_tube",
    )

    follower = model.part("follower")
    follower.visual(
        Box((0.070, 0.0132, 0.0046)),
        origin=Origin(xyz=(0.030, 0.0, -0.007)),
        material="bright_steel",
        name="runner",
    )
    follower.visual(
        Box((0.008, 0.004, 0.0142)),
        origin=Origin(xyz=(0.001, 0.0, 0.0024)),
        material="bright_steel",
        name="stem",
    )
    follower.visual(
        Box((0.014, 0.020, 0.004)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0115)),
        material="bright_steel",
        name="thumb",
    )

    clincher = model.part("clincher")
    clincher.visual(
        mesh_from_cadquery(_clincher_shape(), "stapler_clincher_plate"),
        material="bright_steel",
        name="plate",
    )

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=15.0, velocity=2.5),
    )
    model.articulation(
        "follower_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=follower,
        origin=Origin(xyz=(FOLLOWER_JOINT_X, 0.0, FOLLOWER_JOINT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=FOLLOWER_TRAVEL, effort=8.0, velocity=0.12),
    )
    model.articulation(
        "clincher_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=clincher,
        origin=Origin(xyz=(CLINCHER_X, 0.0, BASE_TOP_Z - BASE_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=(math.pi / 2.0), effort=1.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    follower = object_model.get_part("follower")
    clincher = object_model.get_part("clincher")

    arm_hinge = object_model.get_articulation("arm_hinge")
    follower_slide = object_model.get_articulation("follower_slide")
    clincher_pivot = object_model.get_articulation("clincher_pivot")

    arm_limits = arm_hinge.motion_limits
    follower_limits = follower_slide.motion_limits
    clincher_limits = clincher_pivot.motion_limits

    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="arm_cover",
        negative_elem="base_plate",
        min_gap=0.005,
        max_gap=0.018,
        name="closed stapler throat gap",
    )
    ctx.expect_overlap(
        arm,
        base,
        axes="xy",
        elem_a="arm_cover",
        elem_b="base_plate",
        min_overlap=0.020,
        name="upper arm stays over the base footprint",
    )
    ctx.expect_gap(
        base,
        clincher,
        axis="z",
        positive_elem="base_plate",
        negative_elem="plate",
        min_gap=0.0,
        max_gap=0.0005,
        name="clincher plate sits flush under the base",
    )
    ctx.expect_within(
        clincher,
        base,
        axes="xy",
        inner_elem="plate",
        outer_elem="front_pad",
        margin=0.001,
        name="clincher stays under the stapler nose",
    )
    ctx.expect_within(
        follower,
        arm,
        axes="yz",
        inner_elem="runner",
        outer_elem="arm_cover",
        margin=0.001,
        name="follower runner stays inside the magazine channel",
    )
    ctx.expect_overlap(
        follower,
        arm,
        axes="x",
        elem_a="runner",
        elem_b="arm_cover",
        min_overlap=0.050,
        name="follower runner remains inserted at rest",
    )

    if arm_limits is not None and arm_limits.upper is not None:
        rest_arm_aabb = ctx.part_element_world_aabb(arm, elem="arm_cover")
        with ctx.pose({arm_hinge: arm_limits.upper}):
            open_arm_aabb = ctx.part_element_world_aabb(arm, elem="arm_cover")
            ctx.expect_gap(
                arm,
                base,
                axis="z",
                positive_elem="arm_cover",
                negative_elem="base_plate",
                min_gap=0.015,
                name="opened arm lifts well above the base",
            )
        ctx.check(
            "arm opens upward",
            rest_arm_aabb is not None
            and open_arm_aabb is not None
            and open_arm_aabb[1][2] > rest_arm_aabb[1][2] + 0.040,
            details=f"rest={rest_arm_aabb}, open={open_arm_aabb}",
        )

    if follower_limits is not None and follower_limits.upper is not None:
        rest_follower_pos = ctx.part_world_position(follower)
        with ctx.pose({follower_slide: follower_limits.upper}):
            pulled_follower_pos = ctx.part_world_position(follower)
            ctx.expect_within(
                follower,
                arm,
                axes="yz",
                inner_elem="runner",
                outer_elem="arm_cover",
                margin=0.001,
                name="pulled follower stays guided by the magazine channel",
            )
            ctx.expect_overlap(
                follower,
                arm,
                axes="x",
                elem_a="runner",
                elem_b="arm_cover",
                min_overlap=0.036,
                name="pulled follower still retains magazine insertion",
            )
        ctx.check(
            "follower tab slides rearward",
            rest_follower_pos is not None
            and pulled_follower_pos is not None
            and pulled_follower_pos[0] < rest_follower_pos[0] - 0.012,
            details=f"rest={rest_follower_pos}, pulled={pulled_follower_pos}",
        )

    if clincher_limits is not None and clincher_limits.upper is not None:
        rest_plate_aabb = ctx.part_element_world_aabb(clincher, elem="plate")
        with ctx.pose({clincher_pivot: clincher_limits.upper}):
            turned_plate_aabb = ctx.part_element_world_aabb(clincher, elem="plate")
            ctx.expect_gap(
                base,
                clincher,
                axis="z",
                positive_elem="base_plate",
                negative_elem="plate",
                min_gap=0.0,
                max_gap=0.0005,
                name="rotated clincher stays under the base",
            )
            ctx.expect_within(
                clincher,
                base,
                axes="xy",
                inner_elem="plate",
                outer_elem="front_pad",
                margin=0.001,
                name="rotated clincher remains under the nose pad",
            )
        rest_dx = None if rest_plate_aabb is None else rest_plate_aabb[1][0] - rest_plate_aabb[0][0]
        rest_dy = None if rest_plate_aabb is None else rest_plate_aabb[1][1] - rest_plate_aabb[0][1]
        turned_dx = None if turned_plate_aabb is None else turned_plate_aabb[1][0] - turned_plate_aabb[0][0]
        turned_dy = None if turned_plate_aabb is None else turned_plate_aabb[1][1] - turned_plate_aabb[0][1]
        ctx.check(
            "clincher rotates ninety degrees",
            rest_dx is not None
            and rest_dy is not None
            and turned_dx is not None
            and turned_dy is not None
            and rest_dx > rest_dy
            and turned_dy > turned_dx,
            details=f"rest=({rest_dx}, {rest_dy}), turned=({turned_dx}, {turned_dy})",
        )

    return ctx.report()


object_model = build_object_model()
