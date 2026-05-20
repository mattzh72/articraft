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


BODY_W = 0.160
BODY_H = 0.190
BODY_D = 0.034
BODY_CORNER_R = 0.017

DIAL_Y = 0.028
DIAL_OUTER_R = 0.060
DIAL_INNER_R = 0.041
DIAL_RECESS_OUTER_R = 0.0615
DIAL_RECESS_INNER_R = 0.0385
DIAL_RING_Z = 0.0308
DIAL_RING_D = 0.0085

ROCKER_Y = 0.016
ROCKER_X = 0.061
ROCKER_W = 0.020
ROCKER_H = 0.048
ROCKER_D = 0.006
ROCKER_AXIS_Z = 0.0362
ROCKER_POCKET_D = 0.0065

DISPLAY_Y = 0.066
DISPLAY_W = 0.050
DISPLAY_H = 0.022

SETTINGS_Y = -0.054
SETTINGS_W = 0.112
SETTINGS_H = 0.056
SETTINGS_FLOOR_Z = 0.0292

COVER_HINGE_Y = -0.083
COVER_HINGE_Z = 0.0326
COVER_W = 0.112
COVER_H = 0.046
COVER_T = 0.0024
COVER_BARREL_R = 0.0032


def _body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_H, BODY_D, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_R)
        .edges(">Z")
        .fillet(0.0045)
    )

    shell = (
        shell.faces(">Z")
        .workplane()
        .center(0.0, DIAL_Y)
        .circle(DIAL_RECESS_OUTER_R)
        .circle(DIAL_RECESS_INNER_R)
        .cutBlind(0.0030)
    )
    shell = (
        shell.faces(">Z")
        .workplane()
        .center(0.0, DISPLAY_Y)
        .rect(DISPLAY_W + 0.008, DISPLAY_H + 0.007)
        .cutBlind(0.0026)
    )
    shell = (
        shell.faces(">Z")
        .workplane()
        .center(-ROCKER_X, ROCKER_Y)
        .rect(ROCKER_W + 0.008, ROCKER_H + 0.010)
        .cutBlind(ROCKER_POCKET_D)
    )
    shell = (
        shell.faces(">Z")
        .workplane()
        .center(ROCKER_X, ROCKER_Y)
        .rect(ROCKER_W + 0.008, ROCKER_H + 0.010)
        .cutBlind(ROCKER_POCKET_D)
    )
    shell = (
        shell.faces(">Z")
        .workplane()
        .center(0.0, SETTINGS_Y)
        .rect(SETTINGS_W + 0.008, SETTINGS_H + 0.008)
        .cutBlind(0.0048)
    )
    shell = shell.cut(
        cq.Workplane("YZ")
        .center(COVER_HINGE_Y, COVER_HINGE_Z)
        .circle(COVER_BARREL_R + 0.0005)
        .extrude(0.018, both=True)
    )

    left_hinge = (
        cq.Workplane("YZ")
        .center(COVER_HINGE_Y, COVER_HINGE_Z)
        .circle(COVER_BARREL_R)
        .extrude(0.009, both=True)
        .translate((-0.027, 0.0, 0.0))
    )
    right_hinge = (
        cq.Workplane("YZ")
        .center(COVER_HINGE_Y, COVER_HINGE_Z)
        .circle(COVER_BARREL_R)
        .extrude(0.009, both=True)
        .translate((0.027, 0.0, 0.0))
    )
    return shell.union(left_hinge).union(right_hinge)


def _dial_ring_shape() -> cq.Workplane:
    rear_band = cq.Workplane("XY").circle(0.0575).circle(DIAL_INNER_R).extrude(0.0045)
    front_band = (
        cq.Workplane("XY")
        .circle(DIAL_OUTER_R)
        .circle(0.044)
        .extrude(0.0040)
        .translate((0.0, 0.0, 0.0045))
    )
    return rear_band.union(front_band)


def _rocker_shape() -> cq.Workplane:
    paddle = (
        cq.Workplane("XY")
        .box(ROCKER_W, ROCKER_H, ROCKER_D, centered=(True, True, True))
        .translate((0.0, 0.0, 0.0032))
        .edges("|Z")
        .fillet(0.0015)
        .edges(">Z")
        .fillet(0.0008)
    )
    axle = cq.Workplane("YZ").circle(0.0018).extrude(0.011, both=True)
    return paddle.union(axle)


def _cover_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(COVER_W, COVER_H, COVER_T, centered=(True, True, True))
        .translate((0.0, 0.0245, 0.0028))
        .edges("|Z")
        .fillet(0.003)
    )
    pull_lip = (
        cq.Workplane("XY")
        .box(0.036, 0.006, 0.004, centered=(True, True, True))
        .translate((0.0, 0.048, 0.0042))
        .edges(">Y")
        .fillet(0.001)
    )
    return panel.union(pull_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    housing_mat = model.material("housing", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_mat = model.material("trim", rgba=(0.18, 0.20, 0.23, 1.0))
    rocker_mat = model.material("rocker", rgba=(0.26, 0.28, 0.31, 1.0))
    display_mat = model.material("display", rgba=(0.09, 0.11, 0.13, 1.0))
    setting_mat = model.material("setting", rgba=(0.72, 0.75, 0.79, 1.0))
    clear_mat = model.material("clear_smoke", rgba=(0.58, 0.65, 0.74, 0.30))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "thermostat_body"),
        material=housing_mat,
        name="housing",
    )

    center_hub = model.part("center_hub")
    center_hub.visual(
        Box((0.072, 0.072, 0.0030)),
        origin=Origin(xyz=(0.0, DIAL_Y, 0.0320)),
        material=trim_mat,
        name="center_pad",
    )

    display_window = model.part("display_window")
    display_window.visual(
        Box((DISPLAY_W, DISPLAY_H, 0.0024)),
        origin=Origin(xyz=(0.0, DISPLAY_Y, BODY_D - 0.0014)),
        material=display_mat,
        name="display",
    )

    settings_panel = model.part("settings_panel")
    settings_panel.visual(
        Box((SETTINGS_W - 0.012, SETTINGS_H - 0.012, 0.0012)),
        origin=Origin(xyz=(0.0, SETTINGS_Y, SETTINGS_FLOOR_Z)),
        material=trim_mat,
        name="settings_floor",
    )
    settings_panel.visual(
        Box((SETTINGS_W + 0.008, 0.050, 0.0060)),
        origin=Origin(xyz=(0.0, SETTINGS_Y, 0.0290)),
        material=trim_mat,
        name="settings_backer",
    )
    for idx, x_pos in enumerate((-0.028, 0.0, 0.028)):
        settings_panel.visual(
            Box((0.016, 0.010, 0.0022)),
            origin=Origin(xyz=(x_pos, SETTINGS_Y - 0.001, SETTINGS_FLOOR_Z + 0.0012)),
            material=setting_mat,
            name=f"setting_{idx}",
        )

    dial_ring = model.part("dial_ring")
    dial_ring.visual(
        mesh_from_cadquery(_dial_ring_shape(), "thermostat_dial_ring"),
        material=trim_mat,
        name="dial_ring",
    )

    rocker_left = model.part("rocker_0")
    rocker_left.visual(
        mesh_from_cadquery(_rocker_shape(), "thermostat_rocker_left"),
        material=rocker_mat,
        name="rocker",
    )

    rocker_right = model.part("rocker_1")
    rocker_right.visual(
        mesh_from_cadquery(_rocker_shape(), "thermostat_rocker_right"),
        material=rocker_mat,
        name="rocker",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_shape(), "thermostat_cover"),
        material=clear_mat,
        name="cover_panel",
    )
    cover.visual(
        Cylinder(radius=COVER_BARREL_R, length=0.030),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_mat,
        name="cover_barrel",
    )
    cover.visual(
        Box((COVER_W, COVER_H, COVER_T)),
        origin=Origin(xyz=(0.0, 0.0245, 0.0028)),
        material=clear_mat,
        name="cover_glass",
    )

    model.articulation(
        "body_to_center_hub",
        ArticulationType.FIXED,
        parent=body,
        child=center_hub,
        origin=Origin(),
    )
    model.articulation(
        "body_to_display_window",
        ArticulationType.FIXED,
        parent=body,
        child=display_window,
        origin=Origin(),
    )
    model.articulation(
        "body_to_settings_panel",
        ArticulationType.FIXED,
        parent=body,
        child=settings_panel,
        origin=Origin(),
    )
    model.articulation(
        "body_to_dial_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial_ring,
        origin=Origin(xyz=(0.0, DIAL_Y, DIAL_RING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=10.0),
    )
    model.articulation(
        "body_to_rocker_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker_left,
        origin=Origin(xyz=(-ROCKER_X, ROCKER_Y, ROCKER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "body_to_rocker_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker_right,
        origin=Origin(xyz=(ROCKER_X, ROCKER_Y, ROCKER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, COVER_HINGE_Y, COVER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    center_hub = object_model.get_part("center_hub")
    settings_panel = object_model.get_part("settings_panel")
    dial_ring = object_model.get_part("dial_ring")
    rocker_left = object_model.get_part("rocker_0")
    rocker_right = object_model.get_part("rocker_1")
    cover = object_model.get_part("cover")

    dial_joint = object_model.get_articulation("body_to_dial_ring")
    rocker_left_joint = object_model.get_articulation("body_to_rocker_0")
    rocker_right_joint = object_model.get_articulation("body_to_rocker_1")
    cover_joint = object_model.get_articulation("body_to_cover")

    ctx.check(
        "dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"type={dial_joint.articulation_type}, limits={dial_joint.motion_limits}",
    )
    ctx.expect_origin_distance(
        dial_ring,
        body,
        axes="x",
        min_dist=0.0,
        max_dist=0.001,
        name="dial ring stays centered left to right on the thermostat face",
    )
    ctx.expect_origin_gap(
        dial_ring,
        body,
        axis="y",
        min_gap=0.024,
        max_gap=0.032,
        name="dial ring sits above the housing midline",
    )
    ctx.expect_overlap(
        dial_ring,
        center_hub,
        axes="xy",
        elem_a="dial_ring",
        elem_b="center_pad",
        min_overlap=0.070,
        name="dial ring surrounds the center pad",
    )

    ctx.expect_within(
        rocker_left,
        body,
        axes="xy",
        margin=0.004,
        elem_a="rocker",
        elem_b="housing",
        name="left rocker stays within the body footprint",
    )
    ctx.expect_within(
        rocker_right,
        body,
        axes="xy",
        margin=0.004,
        elem_a="rocker",
        elem_b="housing",
        name="right rocker stays within the body footprint",
    )
    ctx.expect_overlap(
        cover,
        settings_panel,
        axes="xy",
        elem_a="cover_glass",
        elem_b="settings_floor",
        min_overlap=0.035,
        name="cover spans the lower settings area",
    )
    ctx.expect_gap(
        cover,
        settings_panel,
        axis="z",
        positive_elem="cover_glass",
        negative_elem="settings_floor",
        min_gap=0.002,
        max_gap=0.008,
        name="closed cover sits just above the recessed settings",
    )

    left_rest = ctx.part_element_world_aabb(rocker_left, elem="rocker")
    right_rest = ctx.part_element_world_aabb(rocker_right, elem="rocker")
    cover_rest = ctx.part_element_world_aabb(cover, elem="cover_glass")

    with ctx.pose({rocker_left_joint: rocker_left_joint.motion_limits.upper}):
        left_forward = ctx.part_element_world_aabb(rocker_left, elem="rocker")
    with ctx.pose({rocker_right_joint: rocker_right_joint.motion_limits.upper}):
        right_forward = ctx.part_element_world_aabb(rocker_right, elem="rocker")
    with ctx.pose({cover_joint: cover_joint.motion_limits.upper}):
        cover_open = ctx.part_element_world_aabb(cover, elem="cover_glass")

    ctx.check(
        "left rocker tips outward",
        left_rest is not None
        and left_forward is not None
        and left_forward[1][2] > left_rest[1][2] + 0.002,
        details=f"rest={left_rest}, forward={left_forward}",
    )
    ctx.check(
        "right rocker tips outward",
        right_rest is not None
        and right_forward is not None
        and right_forward[1][2] > right_rest[1][2] + 0.002,
        details=f"rest={right_rest}, forward={right_forward}",
    )
    ctx.check(
        "cover rotates down and outward",
        cover_rest is not None
        and cover_open is not None
        and cover_open[1][2] > cover_rest[1][2] + 0.020
        and cover_open[1][1] < cover_rest[1][1] - 0.020,
        details=f"closed={cover_rest}, open={cover_open}",
    )

    return ctx.report()


object_model = build_object_model()
