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


BODY_WIDTH = 0.170
BODY_DEPTH = 0.255
BODY_HEIGHT = 0.335
BODY_WALL = 0.0025
BODY_FLOOR = 0.004
BODY_RADIUS = 0.020

LID_WIDTH = BODY_WIDTH - 0.008
LID_DEPTH = BODY_DEPTH - 0.006
LID_HEIGHT = 0.034
LID_WALL = 0.0022
LID_TOP = 0.0080

PEDAL_PIVOT_Y = BODY_DEPTH / 2.0 + 0.001
PEDAL_PIVOT_Z = 0.030
DAMPER_X = 0.064


def _rounded_open_shell(
    width: float,
    depth: float,
    height: float,
    wall: float,
    floor: float,
    radius: float,
) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(radius)
    )
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - floor)
        .translate((0.0, 0.0, (height + floor) / 2.0))
        .edges("|Z")
        .fillet(max(radius - wall, wall))
    )
    return outer.cut(inner)


def _domed_lid_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LID_WIDTH, LID_DEPTH, LID_HEIGHT)
        .translate((0.0, LID_DEPTH / 2.0, LID_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.014)
        .faces(">Z")
        .edges()
        .fillet(0.015)
    )
    inner = (
        cq.Workplane("XY")
        .box(LID_WIDTH - 2.0 * LID_WALL, LID_DEPTH - 2.0 * LID_WALL, LID_HEIGHT - LID_WALL - LID_TOP)
        .translate((0.0, LID_DEPTH / 2.0, (LID_HEIGHT + LID_WALL - LID_TOP) / 2.0))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_step_bin")

    body_color = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    trim_color = model.material("trim_gray", rgba=(0.44, 0.46, 0.48, 1.0))
    pedal_color = model.material("pedal_gray", rgba=(0.63, 0.65, 0.67, 1.0))
    rubber_color = model.material("rubber_black", rgba=(0.16, 0.16, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(
            _rounded_open_shell(
                width=BODY_WIDTH,
                depth=BODY_DEPTH,
                height=BODY_HEIGHT,
                wall=BODY_WALL,
                floor=BODY_FLOOR,
                radius=BODY_RADIUS,
            ),
            "body_shell",
        ),
        material=body_color,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.007, length=BODY_WIDTH - 0.022),
        origin=Origin(
            xyz=(0.0, -BODY_DEPTH / 2.0 + 0.004, BODY_HEIGHT - 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_color,
        name="hinge_band",
    )
    body.visual(
        Box((0.014, 0.020, 0.026)),
        origin=Origin(xyz=(-0.092, BODY_DEPTH / 2.0 - 0.010, 0.013)),
        material=trim_color,
        name="pedal_bracket_0",
    )
    body.visual(
        Box((0.014, 0.020, 0.026)),
        origin=Origin(xyz=(0.092, BODY_DEPTH / 2.0 - 0.010, 0.013)),
        material=trim_color,
        name="pedal_bracket_1",
    )
    body.visual(
        Box((0.050, 0.040, 0.014)),
        origin=Origin(xyz=(DAMPER_X, -BODY_DEPTH / 2.0 + 0.028, BODY_HEIGHT - 0.007)),
        material=trim_color,
        name="damper_base",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_domed_lid_shell(), "lid_shell"),
        material=body_color,
        name="lid_shell",
    )
    lid.visual(
        Box((0.074, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.016, 0.008)),
        material=trim_color,
        name="lift_tab",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0 + 0.001, BODY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.20),
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.004, length=0.188),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pedal_color,
        name="pedal_axle",
    )
    pedal.visual(
        Box((0.108, 0.022, 0.006)),
        origin=Origin(xyz=(0.0, 0.021, -0.011)),
        material=rubber_color,
        name="pedal_tread",
    )
    pedal.visual(
        Box((0.108, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.009, -0.005)),
        material=pedal_color,
        name="pedal_bridge",
    )
    pedal.visual(
        Box((0.008, 0.266, 0.008)),
        origin=Origin(xyz=(-0.092, -0.133, 0.002)),
        material=pedal_color,
        name="side_link_0",
    )
    pedal.visual(
        Box((0.008, 0.266, 0.008)),
        origin=Origin(xyz=(0.092, -0.133, 0.002)),
        material=pedal_color,
        name="side_link_1",
    )
    pedal.visual(
        Box((0.188, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.266, 0.004)),
        material=pedal_color,
        name="rear_crossbar",
    )
    pedal.visual(
        Box((0.010, 0.010, 0.140)),
        origin=Origin(xyz=(-0.092, -0.266, 0.074)),
        material=pedal_color,
        name="lift_link_0",
    )
    pedal.visual(
        Box((0.010, 0.010, 0.140)),
        origin=Origin(xyz=(0.092, -0.266, 0.074)),
        material=pedal_color,
        name="lift_link_1",
    )

    model.articulation(
        "pedal_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, PEDAL_PIVOT_Y, PEDAL_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=0.42),
    )

    damper_cover = model.part("damper_cover")
    damper_cover.visual(
        Cylinder(radius=0.003, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_color,
        name="damper_hinge_barrel",
    )
    damper_cover.visual(
        Box((0.056, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.020, 0.020)),
        material=trim_color,
        name="damper_top",
    )
    damper_cover.visual(
        Box((0.056, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.038, 0.009)),
        material=trim_color,
        name="damper_front",
    )
    damper_cover.visual(
        Box((0.056, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.002, 0.009)),
        material=trim_color,
        name="damper_rear",
    )
    damper_cover.visual(
        Box((0.004, 0.034, 0.018)),
        origin=Origin(xyz=(-0.026, 0.019, 0.009)),
        material=trim_color,
        name="damper_side_0",
    )
    damper_cover.visual(
        Box((0.004, 0.034, 0.018)),
        origin=Origin(xyz=(0.026, 0.019, 0.009)),
        material=trim_color,
        name="damper_side_1",
    )

    model.articulation(
        "damper_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=damper_cover,
        origin=Origin(xyz=(DAMPER_X, -BODY_DEPTH / 2.0 - 0.004, BODY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal = object_model.get_part("pedal")
    pedal_hinge = object_model.get_articulation("pedal_hinge")
    damper_cover = object_model.get_part("damper_cover")
    damper_hinge = object_model.get_articulation("damper_hinge")

    with ctx.pose({lid_hinge: 0.0, pedal_hinge: 0.0, damper_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.010,
            max_penetration=0.0,
            name="closed lid sits just above the body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.140,
            name="closed lid covers the narrow body opening",
        )
        ctx.expect_gap(
            pedal,
            body,
            axis="z",
            positive_elem="pedal_axle",
            negative_elem="pedal_bracket_0",
            max_gap=0.001,
            max_penetration=0.0001,
            name="pedal axle sits on the front pivot bracket",
        )
        ctx.expect_overlap(
            pedal,
            body,
            axes="y",
            min_overlap=0.220,
            name="pedal linkage runs back along most of the bin depth",
        )
        ctx.expect_overlap(
            damper_cover,
            body,
            axes="xy",
            elem_a="damper_top",
            elem_b="damper_base",
            min_overlap=0.025,
            name="damper cover stays over the rear hinge-side base",
        )
        ctx.expect_gap(
            damper_cover,
            body,
            axis="z",
            positive_elem="damper_rear",
            negative_elem="damper_base",
            max_gap=0.002,
            max_penetration=0.0,
            name="damper cover sits down onto its rear base",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)
        rest_tread_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_tread")
        rest_link_aabb = ctx.part_element_world_aabb(pedal, elem="lift_link_1")
        closed_damper_top = ctx.part_element_world_aabb(damper_cover, elem="damper_top")

    limits = lid_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({lid_hinge: limits.upper, pedal_hinge: 0.0, damper_hinge: 0.0}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward from the rear hinge band",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.080,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    pedal_limits = pedal_hinge.motion_limits
    if pedal_limits is not None and pedal_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.0, pedal_hinge: pedal_limits.upper, damper_hinge: 0.0}):
            pressed_tread_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_tread")
            raised_link_aabb = ctx.part_element_world_aabb(pedal, elem="lift_link_1")
        ctx.check(
            "pedal tread presses down while the rear linkage rises",
            rest_tread_aabb is not None
            and rest_link_aabb is not None
            and pressed_tread_aabb is not None
            and raised_link_aabb is not None
            and pressed_tread_aabb[0][2] < rest_tread_aabb[0][2] - 0.006
            and raised_link_aabb[1][2] > rest_link_aabb[1][2] + 0.040,
            details=(
                f"rest_tread={rest_tread_aabb}, pressed_tread={pressed_tread_aabb}, "
                f"rest_link={rest_link_aabb}, raised_link={raised_link_aabb}"
            ),
        )

    damper_limits = damper_hinge.motion_limits
    if damper_limits is not None and damper_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.0, pedal_hinge: 0.0, damper_hinge: damper_limits.upper}):
            open_damper_top = ctx.part_element_world_aabb(damper_cover, elem="damper_top")
        ctx.check(
            "damper cover flips upward on its short rear hinge",
            closed_damper_top is not None
            and open_damper_top is not None
            and open_damper_top[1][2] > closed_damper_top[1][2] + 0.020,
            details=f"closed={closed_damper_top}, open={open_damper_top}",
        )

    return ctx.report()


object_model = build_object_model()
