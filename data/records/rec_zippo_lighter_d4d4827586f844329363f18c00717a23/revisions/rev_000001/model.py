from __future__ import annotations

from math import pi

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


CASE_W = 0.038
CASE_D = 0.013
BODY_H = 0.039
LID_H = 0.0175
WALL = 0.0008
CORNER_R = 0.0020

HINGE_SIDE_GAP = 0.0015
HINGE_X = -(CASE_W * 0.5 + HINGE_SIDE_GAP)
LID_CENTER_X = CASE_W * 0.5 + HINGE_SIDE_GAP
HINGE_BARREL_R = 0.0011
HINGE_LEAF_T = 0.0007
BODY_KNUCKLE_LEN = 0.0052
LID_KNUCKLE_LEN = 0.0084
LID_OPEN_ANGLE = 2.05

INSERT_W = 0.0344
INSERT_D = 0.0106
INSERT_H = 0.0382
INSERT_TRAVEL = 0.029
TOP_DECK_H = 0.0016
CHIMNEY_W = 0.0152
CHIMNEY_D = 0.0066
CHIMNEY_H = 0.0140
CHIMNEY_Y = -0.0023
CHIMNEY_WALL = 0.0005
HOLE_R = 0.0008
WICK_R = 0.0011

WHEEL_R = 0.0026
WHEEL_LEN = 0.0044
WHEEL_Y = 0.0027
WHEEL_Z = 0.0054

MESH_TOL = 0.00015
MESH_ANGULAR_TOL = 0.05


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(
        shape,
        name,
        tolerance=MESH_TOL,
        angular_tolerance=MESH_ANGULAR_TOL,
    )


def _open_shell(width: float, depth: float, height: float, wall: float) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(CORNER_R)
    )
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - wall, centered=(True, True, False))
        .translate((0.0, 0.0, wall))
    )
    return outer.cut(inner)


def _lid_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, LID_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(CORNER_R)
    )
    inner = cq.Workplane("XY").box(
        CASE_W - 2.0 * WALL,
        CASE_D - 2.0 * WALL,
        LID_H - WALL,
        centered=(True, True, False),
    )
    return outer.cut(inner)


def _body_hinge() -> cq.Workplane:
    leaf = cq.Workplane("XY").box(HINGE_SIDE_GAP + HINGE_LEAF_T, 0.0058, 0.0210).translate(
        ((HINGE_SIDE_GAP + HINGE_LEAF_T) * 0.5, 0.0, BODY_H + 0.0045)
    )
    lower = (
        cq.Workplane("XY")
        .circle(HINGE_BARREL_R)
        .extrude(BODY_KNUCKLE_LEN)
        .translate((0.0, 0.0, BODY_H - 0.0048))
    )
    upper = (
        cq.Workplane("XY")
        .circle(HINGE_BARREL_R)
        .extrude(BODY_KNUCKLE_LEN)
        .translate((0.0, 0.0, BODY_H + 0.0105))
    )
    return leaf.union(lower).union(upper)


def _lid_hinge() -> cq.Workplane:
    leaf = cq.Workplane("XY").box(HINGE_SIDE_GAP + HINGE_LEAF_T, 0.0054, 0.0100).translate(
        ((HINGE_SIDE_GAP + HINGE_LEAF_T) * 0.5, 0.0, 0.0051)
    )
    knuckle = (
        cq.Workplane("XY")
        .circle(HINGE_BARREL_R)
        .extrude(LID_KNUCKLE_LEN)
        .translate((0.0, 0.0, 0.0043))
    )
    return leaf.union(knuckle)


def _insert_can() -> cq.Workplane:
    can = (
        cq.Workplane("XY")
        .box(INSERT_W, INSERT_D, INSERT_H, centered=(True, True, True))
        .translate((0.0, 0.0, -INSERT_H * 0.5))
        .edges("|Z")
        .fillet(0.0013)
    )
    deck = cq.Workplane("XY").box(0.022, INSERT_D - 0.0008, TOP_DECK_H, centered=(True, True, False)).translate(
        (0.0, -0.0004, 0.0)
    )
    flint_tube = (
        cq.Workplane("XY")
        .circle(0.00125)
        .extrude(0.024)
        .translate((0.0, WHEEL_Y + 0.0001, -0.018))
    )
    yoke_left = cq.Workplane("XY").box(0.00115, 0.0016, 0.0076, centered=(True, True, False)).translate(
        (0.0031, WHEEL_Y, -0.0001)
    )
    yoke_right = cq.Workplane("XY").box(0.00115, 0.0016, 0.0076, centered=(True, True, False)).translate(
        (-0.0031, WHEEL_Y, -0.0001)
    )
    bridge = cq.Workplane("XY").box(0.0105, 0.0012, 0.0012, centered=(True, True, False)).translate(
        (0.0, WHEEL_Y - 0.0012, 0.0022)
    )
    return can.union(deck).union(flint_tube).union(yoke_left).union(yoke_right).union(bridge)


def _chimney() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0008)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            CHIMNEY_W - 2.0 * CHIMNEY_WALL,
            CHIMNEY_D - 2.0 * CHIMNEY_WALL,
            CHIMNEY_H - CHIMNEY_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CHIMNEY_WALL))
    )
    notch = (
        cq.Workplane("YZ")
        .circle(WHEEL_R * 0.9)
        .extrude(CHIMNEY_W + 0.002, both=True)
        .translate((0.0, CHIMNEY_Y + CHIMNEY_D * 0.58, WHEEL_Z))
    )
    chimney = outer.cut(inner).cut(notch).translate((0.0, CHIMNEY_Y, -0.0002))
    face_points = [
        (-0.0040, 0.0046),
        (0.0, 0.0046),
        (0.0040, 0.0046),
        (-0.0040, 0.0078),
        (0.0, 0.0078),
        (0.0040, 0.0078),
        (-0.0040, 0.0110),
        (0.0, 0.0110),
        (0.0040, 0.0110),
    ]
    side_points = [(-0.0028, 0.0054), (0.0028, 0.0054), (-0.0028, 0.0089), (0.0028, 0.0089)]
    chimney = chimney.faces(">Y").workplane().pushPoints(face_points).circle(HOLE_R).cutBlind(0.0014)
    chimney = chimney.faces("<Y").workplane().pushPoints(face_points).circle(HOLE_R).cutBlind(0.0014)
    chimney = chimney.faces(">X").workplane().pushPoints(side_points).circle(HOLE_R * 0.92).cutBlind(0.0012)
    chimney = chimney.faces("<X").workplane().pushPoints(side_points).circle(HOLE_R * 0.92).cutBlind(0.0012)
    return chimney


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brass_lighter")

    model.material("brass", rgba=(0.78, 0.64, 0.25, 1.0))
    model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("dark_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("wick", rgba=(0.84, 0.80, 0.65, 1.0))

    case = model.part("case")
    case.visual(_mesh(_open_shell(CASE_W, CASE_D, BODY_H, WALL), "case_shell"), material="brass", name="case_shell")
    case.visual(
        _mesh(_body_hinge(), "case_hinge"),
        origin=Origin(xyz=(HINGE_X, 0.0, 0.0)),
        material="brass",
        name="case_hinge",
    )

    lid = model.part("lid")
    lid.visual(
        _mesh(_lid_shell(), "lid_shell"),
        origin=Origin(xyz=(LID_CENTER_X, 0.0, 0.0)),
        material="brass",
        name="lid_shell",
    )
    lid.visual(_mesh(_lid_hinge(), "lid_hinge_mesh"), material="brass", name="lid_hinge")

    insert = model.part("insert")
    insert.visual(_mesh(_insert_can(), "insert_can"), material="steel", name="insert_can")
    insert.visual(_mesh(_chimney(), "chimney"), material="steel", name="chimney")
    insert.visual(
        Cylinder(radius=WICK_R, length=CHIMNEY_H * 0.9),
        origin=Origin(xyz=(0.0, CHIMNEY_Y, CHIMNEY_H * 0.45), rpy=(0.0, 0.0, 0.0)),
        material="wick",
        name="wick",
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_LEN),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="wheel_rim",
    )
    striker_wheel.visual(
        Cylinder(radius=WHEEL_R * 0.48, length=WHEEL_LEN * 1.04),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="steel",
        name="wheel_hub",
    )
    striker_wheel.visual(
        Box((WHEEL_LEN * 0.86, 0.0008, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_R * 0.78)),
        material="steel",
        name="wheel_marker",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_OPEN_ANGLE, effort=1.2, velocity=5.0),
    )
    model.articulation(
        "insert_slide",
        ArticulationType.PRISMATIC,
        parent=case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=INSERT_TRAVEL, effort=4.0, velocity=0.25),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=20.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    striker_wheel = object_model.get_part("striker_wheel")

    lid_hinge = object_model.get_articulation("lid_hinge")
    insert_slide = object_model.get_articulation("insert_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="case_shell",
        max_gap=0.0008,
        max_penetration=0.0,
        name="lid seam closes flush",
    )
    ctx.expect_within(
        insert,
        case,
        axes="xy",
        inner_elem="insert_can",
        outer_elem="case_shell",
        margin=0.0012,
        name="insert body stays centered in the shell",
    )
    ctx.expect_overlap(
        insert,
        case,
        axes="z",
        elem_a="insert_can",
        elem_b="case_shell",
        min_overlap=0.030,
        name="seated insert remains deeply engaged",
    )

    case_aabb = ctx.part_element_world_aabb(case, elem="case_shell")
    chimney_aabb = ctx.part_element_world_aabb(insert, elem="chimney")
    chimney_projects = (
        case_aabb is not None
        and chimney_aabb is not None
        and chimney_aabb[0][2] < case_aabb[1][2] + 0.001
        and chimney_aabb[1][2] > case_aabb[1][2] + 0.010
    )
    ctx.check(
        "chimney stays seated while projecting above the opening",
        chimney_projects,
        details=f"case_aabb={case_aabb}, chimney_aabb={chimney_aabb}",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: LID_OPEN_ANGLE}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    lid_opens_outward = (
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][1] > closed_lid_aabb[1][1] + 0.010
    )
    ctx.check(
        "lid swings outward from the side hinge",
        lid_opens_outward,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_insert_pos = ctx.part_world_position(insert)
    with ctx.pose({insert_slide: INSERT_TRAVEL}):
        ctx.expect_within(
            insert,
            case,
            axes="xy",
            inner_elem="insert_can",
            outer_elem="case_shell",
            margin=0.0012,
            name="extended insert stays aligned with the shell",
        )
        ctx.expect_overlap(
            insert,
            case,
            axes="z",
            elem_a="insert_can",
            elem_b="case_shell",
            min_overlap=0.006,
            name="extended insert still retains insertion",
        )
        extended_insert_pos = ctx.part_world_position(insert)
    ctx.check(
        "insert lifts upward for removal",
        rest_insert_pos is not None
        and extended_insert_pos is not None
        and extended_insert_pos[2] > rest_insert_pos[2] + 0.020,
        details=f"rest={rest_insert_pos}, extended={extended_insert_pos}",
    )

    marker_rest = _aabb_center(ctx.part_element_world_aabb(striker_wheel, elem="wheel_marker"))
    with ctx.pose({wheel_spin: pi * 0.5}):
        marker_quarter = _aabb_center(ctx.part_element_world_aabb(striker_wheel, elem="wheel_marker"))
    ctx.check(
        "striker wheel rotates around its transverse axle",
        marker_rest is not None
        and marker_quarter is not None
        and marker_quarter[1] < marker_rest[1] - 0.001
        and marker_quarter[2] < marker_rest[2] - 0.001,
        details=f"rest={marker_rest}, quarter_turn={marker_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
