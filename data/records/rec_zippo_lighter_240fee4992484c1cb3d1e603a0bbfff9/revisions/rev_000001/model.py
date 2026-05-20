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


CASE_W = 0.038
CASE_D = 0.013
LOWER_H = 0.039
LID_H = 0.019
WALL = 0.0012
HINGE_R = 0.0017
HINGE_X = CASE_W / 2.0 + HINGE_R
LID_OPEN_ANGLE = 1.75

INSERT_W = 0.032
INSERT_D = 0.0090
INSERT_BODY_H = 0.039
INSERT_BOTTOM_Z = WALL
CHIMNEY_H = 0.017
WHEEL_X = 0.010
WHEEL_Z = INSERT_BOTTOM_Z + INSERT_BODY_H + 0.0087


def _front_groove_cutters(width: float, depth: float, z_values: list[float], *, height: float) -> cq.Workplane:
    cutters = None
    for z in z_values:
        for y in (-depth / 2.0, depth / 2.0):
            cutter = (
                cq.Workplane("XY")
                .box(width, 0.00045, height, centered=(True, True, True))
                .translate((0.0, y, z))
            )
            cutters = cutter if cutters is None else cutters.union(cutter)
    return cutters


def _hollow_case_cup() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, LOWER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0013)
    )
    inner = (
        cq.Workplane("XY")
        .box(CASE_W - 2.0 * WALL, CASE_D - 2.0 * WALL, LOWER_H + 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, WALL))
    )
    cup = outer.cut(inner)
    grooves = _front_groove_cutters(
        CASE_W - 0.007,
        CASE_D + 0.00005,
        [0.007, 0.012, 0.017, 0.022, 0.027, 0.032],
        height=0.00010,
    )
    return cup.cut(grooves)


def _hollow_lid_cap() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, LID_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0013)
    )
    inner = (
        cq.Workplane("XY")
        .box(CASE_W - 2.0 * WALL, CASE_D - 2.0 * WALL, LID_H + 0.002, centered=(True, True, False))
        .translate((0.0, 0.0, -0.001))
    )
    cap = outer.cut(inner)
    grooves = _front_groove_cutters(
        CASE_W - 0.007,
        CASE_D + 0.00005,
        [0.004, 0.009, 0.014],
        height=0.00010,
    )
    return cap.cut(grooves)


def _insert_body_and_chimney() -> cq.Workplane:
    body_top = INSERT_BOTTOM_Z + INSERT_BODY_H
    body = (
        cq.Workplane("XY")
        .box(INSERT_W, INSERT_D, INSERT_BODY_H, centered=(True, True, False))
        .translate((0.0, 0.0, INSERT_BOTTOM_Z))
        .edges("|Z")
        .fillet(0.0007)
    )

    chimney_w = 0.018
    plate_t = 0.00065
    plates = body
    for y in (INSERT_D / 2.0 - plate_t / 2.0, -INSERT_D / 2.0 + plate_t / 2.0):
        plate = (
            cq.Workplane("XY")
            .box(chimney_w, plate_t, CHIMNEY_H, centered=(True, True, False))
            .translate((0.0, y, body_top - 0.0002))
        )
        for hx in (-0.0055, 0.0, 0.0055):
            for hz in (body_top + 0.0045, body_top + 0.0105):
                slot = (
                    cq.Workplane("XY")
                    .box(0.0022, 0.0020, 0.0020, centered=(True, True, True))
                    .translate((hx, y, hz))
                )
                plate = plate.cut(slot)
        plates = plates.union(plate)

    side_bridge = (
        cq.Workplane("XY")
        .box(0.0010, INSERT_D, CHIMNEY_H * 0.85, centered=(True, True, False))
        .translate((-0.0090, 0.0, body_top - 0.0002))
    )
    top_bridge = (
        cq.Workplane("XY")
        .box(chimney_w, INSERT_D, 0.00075, centered=(True, True, True))
        .translate((0.0, 0.0, body_top + CHIMNEY_H - 0.0010))
    )
    plates = plates.union(side_bridge).union(top_bridge)

    tab_t = 0.00075
    tab_y = 0.0029
    for y in (-tab_y, tab_y):
        tab = (
            cq.Workplane("XY")
            .box(0.0062, tab_t, 0.0178, centered=(True, True, True))
            .translate((WHEEL_X, y, WHEEL_Z))
        )
        plates = plates.union(tab)

    axle_hole = (
        cq.Workplane("XZ")
        .center(WHEEL_X, WHEEL_Z)
        .circle(0.00105)
        .extrude(0.020)
        .translate((0.0, -0.010, 0.0))
    )
    return plates.cut(axle_hole)


def _spark_wheel() -> cq.Workplane:
    radius = 0.0032
    thickness = 0.0033
    wheel = (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    for i in range(18):
        angle = 2.0 * math.pi * i / 18
        tooth = (
            cq.Workplane("XY")
            .box(0.0010, 0.00105, thickness, centered=(True, True, True))
            .translate((0.0, radius + 0.00035, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(angle))
        )
        wheel = wheel.union(tooth)
    axle = (
        cq.Workplane("XY")
        .circle(0.00078)
        .extrude(0.0074)
        .translate((0.0, 0.0, -0.0037))
    )
    return wheel.union(axle)


def _rotated_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    return (x * math.cos(angle) - y * math.sin(angle), x * math.sin(angle) + y * math.cos(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brushed_metal_zippo_lighter")

    brushed = Material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_steel = Material("dark_knurled_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    shadow = Material("dark_openings", rgba=(0.04, 0.04, 0.035, 1.0))
    wick_mat = Material("charred_wick", rgba=(0.08, 0.065, 0.045, 1.0))
    brass = Material("warm_brass", rgba=(0.78, 0.55, 0.23, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        Box((CASE_W, WALL, LOWER_H)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - WALL / 2.0, LOWER_H / 2.0)),
        material=brushed,
        name="lower_front",
    )
    lower_shell.visual(
        Box((CASE_W, WALL, LOWER_H)),
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 + WALL / 2.0, LOWER_H / 2.0)),
        material=brushed,
        name="lower_back",
    )
    lower_shell.visual(
        Box((WALL, CASE_D, LOWER_H)),
        origin=Origin(xyz=(CASE_W / 2.0 - WALL / 2.0, 0.0, LOWER_H / 2.0)),
        material=brushed,
        name="lower_hinge_wall",
    )
    lower_shell.visual(
        Box((WALL, CASE_D, LOWER_H)),
        origin=Origin(xyz=(-CASE_W / 2.0 + WALL / 2.0, 0.0, LOWER_H / 2.0)),
        material=brushed,
        name="lower_far_wall",
    )
    lower_shell.visual(
        Box((CASE_W, CASE_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=brushed,
        name="lower_floor",
    )
    lower_shell.visual(
        Box((0.0018, 0.0065, 0.027)),
        origin=Origin(xyz=(HINGE_X - 0.00085, 0.0, 0.020)),
        material=brushed,
        name="lower_hinge_leaf",
    )
    lower_shell.visual(
        Cylinder(radius=HINGE_R, length=0.024),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.017)),
        material=dark_steel,
        name="lower_hinge_barrel",
    )

    lid = model.part("lid")
    def lid_box_origin(x: float, y: float, z: float) -> Origin:
        open_x, open_y = _rotated_xy(x, y, LID_OPEN_ANGLE)
        return Origin(xyz=(open_x, open_y, z), rpy=(0.0, 0.0, LID_OPEN_ANGLE))

    lid.visual(
        Box((CASE_W, WALL, LID_H)),
        origin=lid_box_origin(-HINGE_X, CASE_D / 2.0 - WALL / 2.0, LID_H / 2.0),
        material=brushed,
        name="lid_front",
    )
    lid.visual(
        Box((CASE_W, WALL, LID_H)),
        origin=lid_box_origin(-HINGE_X, -CASE_D / 2.0 + WALL / 2.0, LID_H / 2.0),
        material=brushed,
        name="lid_back",
    )
    lid.visual(
        Box((WALL, CASE_D, LID_H)),
        origin=lid_box_origin(-HINGE_X + CASE_W / 2.0 - WALL / 2.0, 0.0, LID_H / 2.0),
        material=brushed,
        name="lid_hinge_wall",
    )
    lid.visual(
        Box((WALL, CASE_D, LID_H)),
        origin=lid_box_origin(-HINGE_X - CASE_W / 2.0 + WALL / 2.0, 0.0, LID_H / 2.0),
        material=brushed,
        name="lid_far_wall",
    )
    lid.visual(
        Box((CASE_W, CASE_D, WALL)),
        origin=lid_box_origin(-HINGE_X, 0.0, LID_H - WALL / 2.0),
        material=brushed,
        name="lid_top",
    )
    leaf_x, leaf_y = _rotated_xy(-0.0010, 0.0, LID_OPEN_ANGLE)
    lid.visual(
        Box((0.0020, 0.0065, 0.014)),
        origin=Origin(xyz=(leaf_x, leaf_y, 0.010), rpy=(0.0, 0.0, LID_OPEN_ANGLE)),
        material=brushed,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_steel,
        name="lid_hinge_barrel",
    )
    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-LID_OPEN_ANGLE, upper=0.0),
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_insert_body_and_chimney(), "insert_chimney_body", tolerance=0.0003, angular_tolerance=0.12),
        material=brushed,
        name="insert_chimney_body",
    )
    insert.visual(
        Cylinder(radius=0.0009, length=0.012),
        origin=Origin(xyz=(-0.0045, 0.0, INSERT_BOTTOM_Z + INSERT_BODY_H + 0.0055)),
        material=wick_mat,
        name="wick",
    )
    insert.visual(
        Cylinder(radius=0.0010, length=0.011),
        origin=Origin(xyz=(WHEEL_X + 0.0038, 0.0, INSERT_BOTTOM_Z + INSERT_BODY_H + 0.0040)),
        material=brass,
        name="flint_tube",
    )
    model.articulation(
        "shell_to_insert",
        ArticulationType.PRISMATIC,
        parent=lower_shell,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.35, lower=0.0, upper=0.026),
    )

    spark_wheel = model.part("spark_wheel")
    spark_wheel.visual(
        mesh_from_cadquery(_spark_wheel(), "spark_wheel", tolerance=0.00018, angular_tolerance=0.08),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="spark_wheel",
    )
    model.articulation(
        "insert_to_spark_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=spark_wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    spark_wheel = object_model.get_part("spark_wheel")
    lid_joint = object_model.get_articulation("shell_to_lid")
    insert_joint = object_model.get_articulation("shell_to_insert")
    wheel_joint = object_model.get_articulation("insert_to_spark_wheel")

    ctx.expect_within(
        insert,
        lower_shell,
        axes="xy",
        inner_elem="insert_chimney_body",
        margin=0.001,
        name="insert is centered inside the lower shell opening",
    )
    ctx.expect_overlap(
        insert,
        lower_shell,
        axes="z",
        elem_a="insert_chimney_body",
        min_overlap=0.020,
        name="seated insert remains captured by the lower case",
    )
    ctx.expect_overlap(
        spark_wheel,
        insert,
        axes="xz",
        elem_a="spark_wheel",
        elem_b="insert_chimney_body",
        min_overlap=0.002,
        name="spark wheel sits between the insert support tabs",
    )

    seated_pos = ctx.part_world_position(insert)
    with ctx.pose({insert_joint: 0.026}):
        ctx.expect_overlap(
            insert,
            lower_shell,
            axes="z",
            elem_a="insert_chimney_body",
            min_overlap=0.010,
            name="lifted insert still has retained insertion",
        )
        lifted_pos = ctx.part_world_position(insert)
    ctx.check(
        "insert slides upward out of the case",
        seated_pos is not None and lifted_pos is not None and lifted_pos[2] > seated_pos[2] + 0.020,
        details=f"seated={seated_pos}, lifted={lifted_pos}",
    )

    open_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")
    with ctx.pose({lid_joint: -LID_OPEN_ANGLE}):
        ctx.expect_gap(
            lid,
            lower_shell,
            axis="z",
            positive_elem="lid_front",
            negative_elem="lower_front",
            max_gap=0.0015,
            max_penetration=0.0002,
            name="closed lid seats on the lower shell rim",
        )
        ctx.expect_overlap(
            lid,
            lower_shell,
            axes="x",
            elem_a="lid_front",
            elem_b="lower_front",
            min_overlap=0.035,
            name="closed lid spans the case width",
        )
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")
    ctx.check(
        "lid default pose is swung open to reveal the chimney",
        open_aabb is not None
        and closed_aabb is not None
        and (open_aabb[0][1] + open_aabb[1][1]) * 0.5 < (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5 - 0.010,
        details=f"open={open_aabb}, closed={closed_aabb}",
    )
    ctx.check(
        "spark wheel is a continuous rotary joint",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, -1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
