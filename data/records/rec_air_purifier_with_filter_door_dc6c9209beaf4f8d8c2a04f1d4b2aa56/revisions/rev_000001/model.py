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


BODY_W = 0.24
BODY_D = 0.19
BODY_H = 0.92
BODY_CORNER_R = 0.026
WALL = 0.014
FRONT_WALL = 0.018
BOTTOM_BASE = 0.075
TOP_SECTION = 0.145

CAVITY_W = BODY_W - 2.0 * WALL
CAVITY_D = BODY_D - FRONT_WALL
CAVITY_H = BODY_H - BOTTOM_BASE - TOP_SECTION
CAVITY_CENTER_Z = BOTTOM_BASE + CAVITY_H / 2.0

HINGE_R = 0.006
HINGE_X = CAVITY_W / 2.0 - 0.014
BODY_KNUCKLE_LEN = 0.17
DOOR_KNUCKLE_LEN = 0.28
DOOR_T = 0.008
DOOR_W = CAVITY_W + 0.016
DOOR_H = CAVITY_H + 0.030
DOOR_LEAF_W = DOOR_W - 0.014

FILTER_W = CAVITY_W - 0.022
FILTER_D = 0.165
FILTER_H = CAVITY_H - 0.040
FILTER_TRAVEL = 0.11

SHOULDER_OUTER_R = 0.080
SHOULDER_INNER_R = 0.060
SHOULDER_H = 0.012
SHOULDER_CENTER_Z = BODY_H + 0.004

RING_OUTER_R = 0.094
RING_INNER_R = 0.080
RING_H = 0.018
RING_CENTER_Z = BODY_H + 0.019

SLOT_CENTER_Z = 0.66
SLOT_CUT_X = 0.030
SLOT_CUT_Y = 0.012
SLOT_CUT_Z = 0.078
SLIDER_TRAVEL = 0.022


def centered_annulus(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def build_body_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(BODY_CORNER_R)
    )

    rear_cavity = (
        cq.Workplane("XY")
        .box(CAVITY_W, CAVITY_D, CAVITY_H)
        .translate((0.0, -FRONT_WALL / 2.0, CAVITY_CENTER_Z))
    )

    top_outlet = (
        cq.Workplane("XY")
        .circle(SHOULDER_INNER_R)
        .extrude(TOP_SECTION + 0.040)
        .translate((0.0, 0.0, BODY_H - TOP_SECTION - 0.010))
    )

    timer_slot = (
        cq.Workplane("XY")
        .box(SLOT_CUT_X, SLOT_CUT_Y, SLOT_CUT_Z)
        .translate((BODY_W / 2.0 - WALL / 2.0, 0.0, SLOT_CENTER_Z))
    )

    return outer.cut(rear_cavity).cut(top_outlet).cut(timer_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_purifier")

    shell_white = model.material("shell_white", rgba=(0.92, 0.93, 0.94, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    charcoal = model.material("charcoal", rgba=(0.24, 0.25, 0.26, 1.0))
    satin = model.material("satin", rgba=(0.70, 0.73, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shell(), "body_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(
            centered_annulus(SHOULDER_OUTER_R, SHOULDER_INNER_R, SHOULDER_H),
            "top_shoulder",
        ),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_CENTER_Z)),
        material=graphite,
        name="top_shoulder",
    )
    body.visual(
        Box((0.120, 0.004, 0.430)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.001, 0.41)),
        material=graphite,
        name="front_grille",
    )
    body.visual(
        Box((0.012, 0.010, BODY_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(HINGE_X - 0.006, -BODY_D / 2.0 - 0.002, CAVITY_CENTER_Z - 0.275),
        ),
        material=graphite,
        name="body_mount_0",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=BODY_KNUCKLE_LEN),
        origin=Origin(
            xyz=(HINGE_X, -BODY_D / 2.0 - 0.002, CAVITY_CENTER_Z - 0.275),
        ),
        material=graphite,
        name="body_knuckle_0",
    )
    body.visual(
        Box((0.012, 0.010, BODY_KNUCKLE_LEN)),
        origin=Origin(
            xyz=(HINGE_X - 0.006, -BODY_D / 2.0 - 0.002, CAVITY_CENTER_Z + 0.275),
        ),
        material=graphite,
        name="body_mount_1",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=BODY_KNUCKLE_LEN),
        origin=Origin(
            xyz=(HINGE_X, -BODY_D / 2.0 - 0.002, CAVITY_CENTER_Z + 0.275),
        ),
        material=graphite,
        name="body_knuckle_1",
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((DOOR_LEAF_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-DOOR_LEAF_W / 2.0 - 0.010, -DOOR_T / 2.0, 0.0)),
        material=shell_white,
        name="door_leaf",
    )
    filter_door.visual(
        Box((0.024, 0.010, 0.120)),
        origin=Origin(xyz=(-DOOR_W + 0.016, -0.009, 0.0)),
        material=graphite,
        name="door_pull",
    )
    filter_door.visual(
        Box((0.010, 0.010, DOOR_KNUCKLE_LEN)),
        origin=Origin(xyz=(-0.007, -0.004, 0.0)),
        material=graphite,
        name="door_mount",
    )
    filter_door.visual(
        Cylinder(radius=HINGE_R, length=DOOR_KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=graphite,
        name="door_knuckle",
    )

    filter_pack = model.part("filter_pack")
    filter_pack.visual(
        Box((FILTER_W, FILTER_D, FILTER_H)),
        origin=Origin(xyz=(0.0, FILTER_D / 2.0, 0.0)),
        material=satin,
        name="filter_frame",
    )
    filter_pack.visual(
        Box((FILTER_W - 0.022, FILTER_D - 0.014, FILTER_H - 0.050)),
        origin=Origin(xyz=(0.0, FILTER_D / 2.0 + 0.004, 0.0)),
        material=charcoal,
        name="filter_media",
    )

    control_ring = model.part("control_ring")
    control_ring.visual(
        mesh_from_cadquery(
            centered_annulus(RING_OUTER_R, RING_INNER_R, RING_H),
            "control_ring",
        ),
        material=satin,
        name="ring_shell",
    )

    timer_slider = model.part("timer_slider")
    timer_slider.visual(
        Box((WALL + 0.012, 0.010, 0.010)),
        origin=Origin(),
        material=graphite,
        name="slider_runner",
    )
    timer_slider.visual(
        Box((0.018, 0.016, 0.026)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=graphite,
        name="slider_thumb",
    )

    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(xyz=(HINGE_X, -BODY_D / 2.0, CAVITY_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=2.1,
        ),
    )

    model.articulation(
        "body_to_filter_pack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_pack,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, CAVITY_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.12,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_control_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=control_ring,
        origin=Origin(xyz=(0.0, 0.0, RING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    model.articulation(
        "body_to_timer_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=timer_slider,
        origin=Origin(xyz=(BODY_W / 2.0 - WALL / 2.0, 0.0, SLOT_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=-SLIDER_TRAVEL,
            upper=SLIDER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    filter_door = object_model.get_part("filter_door")
    filter_pack = object_model.get_part("filter_pack")
    control_ring = object_model.get_part("control_ring")
    timer_slider = object_model.get_part("timer_slider")

    door_hinge = object_model.get_articulation("body_to_filter_door")
    filter_slide = object_model.get_articulation("body_to_filter_pack")
    ring_joint = object_model.get_articulation("body_to_control_ring")
    slider_joint = object_model.get_articulation("body_to_timer_slider")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            filter_door,
            axis="y",
            positive_elem="body_shell",
            negative_elem="door_leaf",
            max_gap=0.003,
            max_penetration=0.0,
            name="rear door closes flush with the shell",
        )
        ctx.expect_overlap(
            body,
            filter_door,
            axes="xz",
            elem_a="body_shell",
            elem_b="door_leaf",
            min_overlap=0.16,
            name="rear door covers the rear opening footprint",
        )

    closed_leaf_aabb = ctx.part_element_world_aabb(filter_door, elem="door_leaf")
    with ctx.pose({door_hinge: 1.2}):
        opened_leaf_aabb = ctx.part_element_world_aabb(filter_door, elem="door_leaf")
    ctx.check(
        "rear door swings out from the rear face",
        closed_leaf_aabb is not None
        and opened_leaf_aabb is not None
        and opened_leaf_aabb[0][1] < closed_leaf_aabb[0][1] - 0.10,
        details=f"closed={closed_leaf_aabb}, opened={opened_leaf_aabb}",
    )

    ctx.expect_gap(
        control_ring,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="ring_shell",
        negative_elem="top_shoulder",
        name="top rotary ring sits just above its shoulder",
    )
    ctx.expect_overlap(
        control_ring,
        body,
        axes="xy",
        elem_a="ring_shell",
        elem_b="top_shoulder",
        min_overlap=0.15,
        name="top rotary ring stays centered over the tower axis",
    )

    rest_pack_pos = ctx.part_world_position(filter_pack)
    filter_limits = filter_slide.motion_limits
    with ctx.pose({filter_slide: filter_limits.upper if filter_limits is not None else FILTER_TRAVEL}):
        ctx.expect_within(
            filter_pack,
            body,
            axes="xz",
            inner_elem="filter_frame",
            outer_elem="body_shell",
            margin=0.014,
            name="filter pack stays aligned in the rear service opening",
        )
        extended_pack_pos = ctx.part_world_position(filter_pack)
        filter_aabb = ctx.part_element_world_aabb(filter_pack, elem="filter_frame")
        body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    ctx.check(
        "filter pack slides rearward for service",
        rest_pack_pos is not None
        and extended_pack_pos is not None
        and extended_pack_pos[1] < rest_pack_pos[1] - 0.09,
        details=f"rest={rest_pack_pos}, extended={extended_pack_pos}",
    )
    retained_depth = None
    if filter_aabb is not None and body_shell_aabb is not None:
        retained_depth = filter_aabb[1][1] - body_shell_aabb[0][1]
    ctx.check(
        "filter pack remains retained at full extension",
        retained_depth is not None and retained_depth >= 0.050,
        details=f"retained_depth={retained_depth}, filter_aabb={filter_aabb}, body_shell_aabb={body_shell_aabb}",
    )

    rest_slider_pos = ctx.part_world_position(timer_slider)
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        upper_slider_pos = ctx.part_world_position(timer_slider)
    ctx.check(
        "timer slider moves upward in its side slot",
        rest_slider_pos is not None
        and upper_slider_pos is not None
        and upper_slider_pos[2] > rest_slider_pos[2] + 0.018,
        details=f"rest={rest_slider_pos}, upper={upper_slider_pos}",
    )

    ctx.check(
        "rotary ring is continuous",
        ring_joint.motion_limits is not None
        and ring_joint.motion_limits.lower is None
        and ring_joint.motion_limits.upper is None,
        details=f"limits={ring_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
