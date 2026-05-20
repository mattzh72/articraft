from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_W = 0.038
CASE_D = 0.013
CASE_H = 0.038
LID_H = 0.019
OUTER_R = 0.0030
CASE_WALL = 0.00085
LID_WALL = 0.00070
EPS = 0.0002

INSERT_BODY_W = 0.0344
INSERT_BODY_D = 0.0103
INSERT_BODY_H = 0.0310
INSERT_SHOULDER_W = 0.0358
INSERT_SHOULDER_D = 0.0115
INSERT_SHOULDER_T = 0.0008
INSERT_TRAVEL = 0.020
INSERT_SEAT_DROP = 0.00040

CHIMNEY_W = 0.0148
CHIMNEY_D = 0.0086
CHIMNEY_H = 0.0142
CHIMNEY_WALL = 0.00055
CHIMNEY_Y = -0.00145

HINGE_R = 0.00125
HINGE_Z = CASE_H - 0.0038
HINGE_X = (CASE_W / 2.0) + (HINGE_R * 0.72)
CASE_BARREL_LEN = 0.0040
LID_BARREL_LEN = 0.0041
BARREL_GAP = 0.00045
CASE_BARREL_Y = (LID_BARREL_LEN / 2.0) + BARREL_GAP + (CASE_BARREL_LEN / 2.0)

LID_CLOSED_GAP = 0.00025
LID_RIGHT_FACE_FROM_AXIS = HINGE_R * 0.58

WHEEL_R = 0.00325
WHEEL_LEN = 0.0047
AXLE_R = 0.00052
WHEEL_Y = 0.00275
WHEEL_Z = 0.0099
EAR_T = 0.00095
EAR_D = 0.0038
EAR_H = 0.0132
EAR_X = (WHEEL_LEN / 2.0) + (EAR_T / 2.0) + 0.00018
AXLE_HOLE_R = AXLE_R + 0.00002


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    usable_r = max(0.0002, min(radius, width * 0.45, depth * 0.45, height * 0.45))
    return cq.Workplane("XY").box(width, depth, height, centered=(True, True, False)).edges().fillet(usable_r)


def _open_top_shell(
    width: float,
    depth: float,
    height: float,
    wall: float,
    radius: float,
) -> cq.Workplane:
    outer = _rounded_box(width, depth, height, radius)
    inner = _rounded_box(width - (2.0 * wall), depth - (2.0 * wall), height - wall + EPS, max(radius - wall, 0.00035))
    inner = inner.translate((0.0, 0.0, wall))
    return outer.cut(inner)


def _open_bottom_shell(
    width: float,
    depth: float,
    height: float,
    wall: float,
    radius: float,
) -> cq.Workplane:
    outer = _rounded_box(width, depth, height, radius)
    inner = _rounded_box(width - (2.0 * wall), depth - (2.0 * wall), height - wall + EPS, max(radius - wall, 0.00035))
    inner = inner.translate((0.0, 0.0, -EPS))
    return outer.cut(inner)


def _case_shape() -> cq.Workplane:
    return _open_top_shell(CASE_W, CASE_D, CASE_H, CASE_WALL, OUTER_R)


def _lid_shape() -> cq.Workplane:
    return _open_bottom_shell(CASE_W + 0.0008, CASE_D + 0.0008, LID_H, LID_WALL, OUTER_R)


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _x_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )


def _insert_shape() -> cq.Workplane:
    body = _rounded_box(INSERT_BODY_W, INSERT_BODY_D, INSERT_BODY_H, 0.0022).translate(
        (0.0, 0.0, -INSERT_BODY_H)
    )

    shoulder = _rounded_box(
        INSERT_SHOULDER_W,
        INSERT_SHOULDER_D,
        INSERT_SHOULDER_T,
        0.0018,
    )

    chimney = _open_top_shell(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H, CHIMNEY_WALL, 0.0010).translate(
        (0.0, CHIMNEY_Y, INSERT_SHOULDER_T)
    )

    insert_shape = body.union(shoulder).union(chimney)

    hole_rows = [0.0031, 0.0058, 0.0085, 0.0112]
    hole_xs = (-0.0034, 0.0034)
    hole_cutters = None
    for hole_z in hole_rows:
        for hole_x in hole_xs:
            cutter = _y_cylinder(radius=0.00085, length=CHIMNEY_D * 2.6).translate(
                (hole_x, CHIMNEY_Y, INSERT_SHOULDER_T + hole_z)
            )
            hole_cutters = cutter if hole_cutters is None else hole_cutters.union(cutter)
    if hole_cutters is not None:
        insert_shape = insert_shape.cut(hole_cutters)

    left_ear = _rounded_box(EAR_T, EAR_D, EAR_H, 0.00038).translate(
        (-EAR_X, WHEEL_Y, 0.0)
    )
    right_ear = _rounded_box(EAR_T, EAR_D, EAR_H, 0.00038).translate(
        (EAR_X, WHEEL_Y, 0.0)
    )
    insert_shape = insert_shape.union(left_ear).union(right_ear)

    axle_cutters = (
        _x_cylinder(AXLE_HOLE_R, EAR_T + 0.0008).translate((-EAR_X, WHEEL_Y, WHEEL_Z))
        .union(_x_cylinder(AXLE_HOLE_R, EAR_T + 0.0008).translate((EAR_X, WHEEL_Y, WHEEL_Z)))
    )
    insert_shape = insert_shape.cut(axle_cutters)

    wheel_relief = _x_cylinder(WHEEL_R + 0.00028, WHEEL_LEN + 0.0011).translate((0.0, WHEEL_Y, WHEEL_Z))
    insert_shape = insert_shape.cut(wheel_relief)

    wick_slot = _rounded_box(0.0032, 0.0024, 0.0042, 0.0006).translate(
        (0.0, CHIMNEY_Y - 0.0004, INSERT_SHOULDER_T + CHIMNEY_H - 0.0042)
    )
    insert_shape = insert_shape.cut(wick_slot)

    return insert_shape


def _wheel_shape() -> cq.Workplane:
    wheel = cq.Workplane("YZ").polygon(18, WHEEL_R * 2.05).extrude(WHEEL_LEN / 2.0, both=True)
    axle = cq.Workplane("YZ").circle(AXLE_R).extrude((WHEEL_LEN + 0.0022) / 2.0, both=True)
    return wheel.union(axle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_lighter")

    brass = model.material("aged_brass", rgba=(0.72, 0.60, 0.31, 1.0))
    steel = model.material("insert_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_case_shape(), "lighter_case"),
        material=brass,
        name="shell",
    )
    case.visual(
        Cylinder(radius=HINGE_R, length=CASE_BARREL_LEN),
        origin=Origin(xyz=(HINGE_X, CASE_BARREL_Y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="barrel_0",
    )
    case.visual(
        Cylinder(radius=HINGE_R, length=CASE_BARREL_LEN),
        origin=Origin(xyz=(HINGE_X, -CASE_BARREL_Y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="barrel_1",
    )
    case.visual(
        mesh_from_cadquery(
            _rounded_box(0.0015, CASE_D * 0.42, 0.0105, 0.00045).translate((HINGE_X - 0.00065, 0.0, HINGE_Z - 0.0062)),
            "case_hinge_leaf",
        ),
        material=brass,
        name="hinge_leaf",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "lighter_lid"),
        origin=Origin(
            xyz=(
                -(CASE_W / 2.0) - LID_RIGHT_FACE_FROM_AXIS,
                0.0,
                HINGE_R + 0.0030,
            )
        ),
        material=brass,
        name="shell",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=LID_BARREL_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="barrel",
    )
    lid.visual(
        mesh_from_cadquery(
            _rounded_box(0.0013, LID_BARREL_LEN, 0.0112, 0.0004).translate((-0.00075, 0.0, -0.0057)),
            "lid_hinge_leaf",
        ),
        material=brass,
        name="hinge_leaf",
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_insert_shape(), "lighter_insert"),
        material=steel,
        name="body",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_wheel_shape(), "lighter_wheel"),
        material=dark_steel,
        name="wheel",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.95,
            effort=2.0,
            velocity=6.0,
        ),
    )
    model.articulation(
        "insert_slide",
        ArticulationType.PRISMATIC,
        parent=case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, CASE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INSERT_TRAVEL,
            effort=12.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=30.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    wheel = object_model.get_part("wheel")

    lid_hinge = object_model.get_articulation("lid_hinge")
    insert_slide = object_model.get_articulation("insert_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.allow_isolated_part(
        insert,
        reason="The lift-out insert is modeled with a visible removal clearance inside the case so it reads as removable rather than fused.",
    )
    ctx.allow_isolated_part(
        wheel,
        reason="The striker wheel rides in a tiny axle clearance inside the insert ears so it can rotate freely.",
    )

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="shell",
        negative_elem="shell",
        min_gap=0.0,
        max_gap=0.0016,
        name="closed lid sits tightly over the case seam",
    )
    ctx.expect_overlap(
        lid,
        case,
        axes="xy",
        elem_a="shell",
        elem_b="shell",
        min_overlap=0.010,
        name="closed lid stays aligned over the body footprint",
    )
    ctx.expect_within(
        insert,
        case,
        axes="xy",
        inner_elem="body",
        outer_elem="shell",
        margin=0.002,
        name="insert can remains centered inside the outer case",
    )
    ctx.expect_overlap(
        insert,
        case,
        axes="z",
        elem_a="body",
        elem_b="shell",
        min_overlap=0.028,
        name="collapsed insert remains deeply nested in the lower shell",
    )

    insert_rest = ctx.part_world_position(insert)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="shell")
        case_aabb = ctx.part_element_world_aabb(case, elem="shell")

    with ctx.pose({insert_slide: insert_slide.motion_limits.upper}):
        ctx.expect_overlap(
            insert,
            case,
            axes="z",
            elem_a="body",
            elem_b="shell",
            min_overlap=0.010,
            name="extended insert still retains visible insertion in the case",
        )
        insert_extended = ctx.part_world_position(insert)

    ctx.check(
        "insert lifts upward along the case height axis",
        insert_rest is not None
        and insert_extended is not None
        and insert_extended[2] > insert_rest[2] + 0.015,
        details=f"rest={insert_rest}, extended={insert_extended}",
    )
    ctx.check(
        "lid hinge is side-mounted and opens upward",
        tuple(lid_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "opened lid swings clear of the case",
        lid_open_aabb is not None
        and case_aabb is not None
        and lid_open_aabb[0][0] > case_aabb[1][0] + 0.004,
        details=f"lid_open_aabb={lid_open_aabb}, case_aabb={case_aabb}",
    )
    ctx.check(
        "striker wheel uses continuous local axle rotation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.expect_overlap(
        wheel,
        insert,
        axes="yz",
        elem_a="wheel",
        elem_b="body",
        min_overlap=0.002,
        name="wheel stays captured between the insert ear supports",
    )

    return ctx.report()


object_model = build_object_model()
