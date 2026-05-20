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


BODY_W = 0.62
BODY_D = 0.39
BODY_H = 0.42
BODY_BOTTOM = 0.025
BODY_WALL = 0.014

WHEEL_RADIUS = 0.085
WHEEL_WIDTH = 0.054
WHEEL_X = 0.268
AXLE_Y = 0.295
AXLE_Z = WHEEL_RADIUS
AXLE_LEN = 0.4766

GUIDE_X_SPACING = 0.300
GUIDE_OUTER_X = 0.032
GUIDE_OUTER_Y = 0.024
GUIDE_WALL = 0.003
GUIDE_BOTTOM = 0.120
GUIDE_LEN = 0.210
GUIDE_Y = BODY_D * 0.5 + GUIDE_OUTER_Y * 0.5
GUIDE_TOP = GUIDE_BOTTOM + GUIDE_LEN

LOWER_ROD_X = 0.020
LOWER_ROD_Y = 0.012
LOWER_INSERT = 0.180
LOWER_VISIBLE = 0.012
LOWER_TRAVEL = 0.110
MIDDLE_SLEEVE_X = 0.026
MIDDLE_SLEEVE_Y = 0.018
MIDDLE_SLEEVE_WALL = 0.0025
MIDDLE_SLEEVE_LEN = 0.185

UPPER_ROD_X = 0.016
UPPER_ROD_Y = 0.008
UPPER_INSERT = 0.155
UPPER_VISIBLE = 0.070
UPPER_TRAVEL = 0.100

LID_W = BODY_W - 2.0 * BODY_WALL - 0.006
LID_D = BODY_D - 2.0 * BODY_WALL - 0.006
LID_H = 0.034
LID_WALL = 0.006
LID_TOP = 0.010
LID_BOTTOM_Z = -0.006
LID_REAR_OFFSET = 0.004
LID_HINGE_Y = BODY_D * 0.5 - BODY_WALL - 0.004
LID_HINGE_Z = BODY_BOTTOM + BODY_H + 0.006


def _rect_sleeve(
    *,
    outer_x: float,
    outer_y: float,
    length: float,
    wall: float,
    close_bottom: bool = True,
):
    outer = cq.Workplane("XY").box(outer_x, outer_y, length, centered=(True, True, False))
    inner_length = length - wall if close_bottom else length
    inner_z = wall if close_bottom else 0.0
    inner = (
        cq.Workplane("XY")
        .box(outer_x - 2.0 * wall, outer_y - 2.0 * wall, inner_length, centered=(True, True, False))
        .translate((0.0, 0.0, inner_z))
    )
    return outer.cut(inner)


def _body_shell_shape():
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(0.018)
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_H - BODY_WALL, centered=(True, True, False))
        .translate((0.0, 0.0, BODY_WALL))
    )
    shell = outer.cut(inner).translate((0.0, 0.0, BODY_BOTTOM))

    front_foot_y = -BODY_D * 0.5 + 0.070
    foot = cq.Workplane("XY").box(0.078, 0.090, BODY_BOTTOM + 0.020, centered=(True, True, False))
    left_foot = foot.translate((0.205, front_foot_y, 0.0))
    right_foot = foot.translate((-0.205, front_foot_y, 0.0))

    bracket = cq.Workplane("XY").box(0.040, 0.150, 0.112, centered=(True, True, False))
    left_bracket = bracket.translate((0.218, 0.237, BODY_BOTTOM))
    right_bracket = bracket.translate((-0.218, 0.237, BODY_BOTTOM))

    return shell.union(left_foot).union(right_foot).union(left_bracket).union(right_bracket)


def _guide_sleeves_shape():
    sleeve = _rect_sleeve(
        outer_x=GUIDE_OUTER_X,
        outer_y=GUIDE_OUTER_Y,
        length=GUIDE_LEN,
        wall=GUIDE_WALL,
        close_bottom=True,
    )
    left = sleeve.translate((GUIDE_X_SPACING * 0.5, 0.0, 0.0))
    right = sleeve.translate((-GUIDE_X_SPACING * 0.5, 0.0, 0.0))
    bridge = (
        cq.Workplane("XY")
        .box(GUIDE_X_SPACING + GUIDE_OUTER_X, GUIDE_OUTER_Y, 0.014, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0))
    )
    return left.union(right).union(bridge)


def _lower_rods_shape():
    rod_length = LOWER_INSERT + LOWER_VISIBLE
    rod = cq.Workplane("XY").box(LOWER_ROD_X, LOWER_ROD_Y, rod_length, centered=(True, True, False))
    left = rod.translate((GUIDE_X_SPACING * 0.5, 0.0, -LOWER_INSERT))
    right = rod.translate((-GUIDE_X_SPACING * 0.5, 0.0, -LOWER_INSERT))
    bridge = (
        cq.Workplane("XY")
        .box(GUIDE_X_SPACING + LOWER_ROD_X, 0.016, 0.018, centered=(True, True, False))
        .translate((0.0, 0.0, 0.004))
    )
    return left.union(right).union(bridge)


def _middle_sleeves_shape():
    sleeve = _rect_sleeve(
        outer_x=MIDDLE_SLEEVE_X,
        outer_y=MIDDLE_SLEEVE_Y,
        length=MIDDLE_SLEEVE_LEN,
        wall=MIDDLE_SLEEVE_WALL,
        close_bottom=True,
    )
    left = sleeve.translate((GUIDE_X_SPACING * 0.5, 0.0, 0.0))
    right = sleeve.translate((-GUIDE_X_SPACING * 0.5, 0.0, 0.0))
    bridge = (
        cq.Workplane("XY")
        .box(GUIDE_X_SPACING + MIDDLE_SLEEVE_X, MIDDLE_SLEEVE_Y, 0.015, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0))
    )
    return left.union(right).union(bridge)


def _upper_rods_shape():
    rod_length = UPPER_INSERT + UPPER_VISIBLE
    rod = cq.Workplane("XY").box(UPPER_ROD_X, UPPER_ROD_Y, rod_length, centered=(True, True, False))
    left = rod.translate((GUIDE_X_SPACING * 0.5, 0.0, -UPPER_INSERT))
    right = rod.translate((-GUIDE_X_SPACING * 0.5, 0.0, -UPPER_INSERT))
    crossbar = (
        cq.Workplane("XY")
        .box(GUIDE_X_SPACING + UPPER_ROD_X, 0.012, 0.014, centered=(True, True, False))
        .translate((0.0, 0.0, 0.050))
    )
    return left.union(right).union(crossbar)


def _lid_panel_shape():
    outer = cq.Workplane("XY").box(LID_W, LID_D, LID_H, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(0.008)
    outer = outer.translate((0.0, -LID_D * 0.5 + LID_REAR_OFFSET, LID_BOTTOM_Z))
    inner = (
        cq.Workplane("XY")
        .box(LID_W - 2.0 * LID_WALL, LID_D - 2.0 * LID_WALL, LID_H - LID_TOP, centered=(True, True, False))
        .translate((0.0, -LID_D * 0.5 + LID_REAR_OFFSET, LID_BOTTOM_Z))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="contractor_rolling_toolbox")

    body_charcoal = model.material("body_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    lid_black = model.material("lid_black", rgba=(0.10, 0.11, 0.12, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    latch_red = model.material("latch_red", rgba=(0.67, 0.15, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    hub_grey = model.material("hub_grey", rgba=(0.46, 0.48, 0.51, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.28, 0.29, 0.31, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "toolbox_body_shell"),
        material=body_charcoal,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_guide_sleeves_shape(), "toolbox_outer_sleeves"),
        origin=Origin(xyz=(0.0, GUIDE_Y, GUIDE_BOTTOM)),
        material=handle_steel,
        name="outer_sleeves",
    )
    body.visual(
        Cylinder(radius=0.014, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, AXLE_Y, AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="axle",
    )
    body.visual(
        Box((0.136, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + 0.010, BODY_BOTTOM + BODY_H - 0.020)),
        material=latch_red,
        name="front_latch",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_panel_shape(), "toolbox_lid_panel"),
        material=lid_black,
        name="lid_panel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=0.0, upper=1.30),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_lower_rods_shape(), "toolbox_lower_handle_rods"),
        material=handle_steel,
        name="lower_rods",
    )
    lower_stage.visual(
        mesh_from_cadquery(_middle_sleeves_shape(), "toolbox_middle_handle_sleeves"),
        material=handle_steel,
        name="middle_sleeves",
    )

    model.articulation(
        "body_to_lower_stage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_stage,
        origin=Origin(xyz=(0.0, GUIDE_Y, GUIDE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.22, lower=0.0, upper=LOWER_TRAVEL),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(_upper_rods_shape(), "toolbox_upper_handle_rods"),
        material=handle_steel,
        name="upper_rods",
    )
    upper_stage.visual(
        Box((GUIDE_X_SPACING + 0.060, 0.024, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel_dark,
        name="stop_collar",
    )
    upper_stage.visual(
        Box((GUIDE_X_SPACING + 0.080, 0.028, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=steel_dark,
        name="grip",
    )

    model.articulation(
        "lower_to_upper_stage",
        ArticulationType.PRISMATIC,
        parent=lower_stage,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_SLEEVE_LEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.22, lower=0.0, upper=UPPER_TRAVEL),
    )

    for suffix, sign in (("0", 1.0), ("1", -1.0)):
        wheel = model.part(f"rear_wheel_{suffix}")
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.050, length=WHEEL_WIDTH * 0.86),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hub_grey,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.020, length=WHEEL_WIDTH * 1.10),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel_dark,
            name="hub",
        )
        model.articulation(
            f"body_to_rear_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(sign * WHEEL_X, AXLE_Y, AXLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=30.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")

    lid_hinge = object_model.get_articulation("body_to_lid")
    lower_slide = object_model.get_articulation("body_to_lower_stage")
    upper_slide = object_model.get_articulation("lower_to_upper_stage")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="shell",
        max_gap=0.008,
        max_penetration=0.002,
        name="lid panel sits on the body opening",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="shell",
        min_overlap=0.24,
        name="lid panel covers the toolbox opening",
    )

    ctx.expect_within(
        lower_stage,
        body,
        axes="xy",
        inner_elem="lower_rods",
        outer_elem="outer_sleeves",
        margin=0.0015,
        name="lower handle rods stay centered in the body guide sleeves",
    )
    ctx.expect_overlap(
        lower_stage,
        body,
        axes="z",
        elem_a="lower_rods",
        elem_b="outer_sleeves",
        min_overlap=0.17,
        name="lower handle stage remains deeply inserted when collapsed",
    )
    ctx.expect_within(
        upper_stage,
        lower_stage,
        axes="xy",
        inner_elem="upper_rods",
        outer_elem="middle_sleeves",
        margin=0.0015,
        name="upper handle rods stay centered in the lower stage sleeves",
    )
    ctx.expect_overlap(
        upper_stage,
        lower_stage,
        axes="z",
        elem_a="upper_rods",
        elem_b="middle_sleeves",
        min_overlap=0.14,
        name="upper handle stage remains deeply inserted when collapsed",
    )

    rest_upper_pos = ctx.part_world_position(upper_stage)
    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lower_slide: LOWER_TRAVEL, upper_slide: UPPER_TRAVEL}):
        ctx.expect_within(
            lower_stage,
            body,
            axes="xy",
            inner_elem="lower_rods",
            outer_elem="outer_sleeves",
            margin=0.0015,
            name="extended lower handle rods stay centered in the body guide sleeves",
        )
        ctx.expect_overlap(
            lower_stage,
            body,
            axes="z",
            elem_a="lower_rods",
            elem_b="outer_sleeves",
            min_overlap=0.07,
            name="extended lower handle stage still retains insertion",
        )
        ctx.expect_within(
            upper_stage,
            lower_stage,
            axes="xy",
            inner_elem="upper_rods",
            outer_elem="middle_sleeves",
            margin=0.0015,
            name="extended upper handle rods stay centered in the lower stage sleeves",
        )
        ctx.expect_overlap(
            upper_stage,
            lower_stage,
            axes="z",
            elem_a="upper_rods",
            elem_b="middle_sleeves",
            min_overlap=0.055,
            name="extended upper handle stage still retains insertion",
        )
        extended_upper_pos = ctx.part_world_position(upper_stage)

    ctx.check(
        "handle extends upward from the storage position",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.18,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )

    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.14,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
