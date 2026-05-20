from __future__ import annotations

from math import isfinite

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.0368
BODY_D = 0.0122
BODY_H = 0.0410
BODY_WALL = 0.00085
BODY_FLOOR = 0.00115

CAP_W = 0.0392
CAP_D = 0.0144
CAP_H = 0.0160
CAP_WALL = 0.00080
CAP_TOP = 0.00080

CORNER_R = 0.00115

HINGE_AXIS_Y = -BODY_D / 2.0 - 0.00065
HINGE_AXIS_Z = BODY_H + 0.00255
CAP_AXIS_TO_REAR = 0.00065
CAP_AXIS_TO_BOTTOM = 0.00275

HINGE_X = -0.0142
HINGE_LEAF_W = 0.0042
HINGE_LEAF_T = 0.00070
BODY_KNUCKLE_LEN = 0.00265
CAP_KNUCKLE_LEN = 0.00195
KNUCKLE_R = 0.00105

INSERT_W = 0.0328
INSERT_D = 0.0099
INSERT_TOP_Z = BODY_H - 0.0011

CHIMNEY_W = 0.0138
CHIMNEY_D = 0.0087
CHIMNEY_H = 0.0140
CHIMNEY_WALL = 0.00070
CHIMNEY_Y = -0.00055
CHIMNEY_BASE_Z = BODY_H - 0.0013
CHIMNEY_HOLE_D = 0.00155
CHIMNEY_HOLE_X = 0.0032
CHIMNEY_HOLE_ZS = (-0.0042, -0.0015, 0.0012, 0.0039)

WHEEL_R = 0.00315
WHEEL_LEN = 0.0048
SHAFT_R = 0.00062
SHAFT_LEN = 0.0108
WHEEL_Y = 0.00215
WHEEL_Z = BODY_H + 0.00515
SUPPORT_X = 0.00355
SUPPORT_T = 0.00080
SUPPORT_D = 0.00270
SUPPORT_H = 0.00660
SUPPORT_Z = BODY_H + SUPPORT_H / 2.0 - 0.00015
SUPPORT_Y = 0.00085


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.00018, angular_tolerance=0.06)


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _body_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(CORNER_R)
    )
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_H + 0.010)
        .translate((0.0, 0.0, BODY_FLOOR + (BODY_H + 0.010) / 2.0))
    )
    return outer.cut(inner)


def _body_hinge() -> cq.Workplane:
    leaf = cq.Workplane("XY").box(HINGE_LEAF_W, HINGE_LEAF_T, 0.0092).translate(
        (HINGE_X, HINGE_AXIS_Y + HINGE_LEAF_T / 2.0, BODY_H - 0.0033)
    )
    upper_knuckle = _x_cylinder(
        KNUCKLE_R,
        BODY_KNUCKLE_LEN,
        (HINGE_X - 0.00155, HINGE_AXIS_Y, HINGE_AXIS_Z),
    )
    lower_knuckle = _x_cylinder(
        KNUCKLE_R,
        BODY_KNUCKLE_LEN,
        (HINGE_X + 0.00150, HINGE_AXIS_Y, HINGE_AXIS_Z),
    )
    web = cq.Workplane("XY").box(0.0016, 0.0010, 0.0048).translate(
        (HINGE_X, HINGE_AXIS_Y + 0.00045, BODY_H + 0.0009)
    )
    return leaf.union(web).union(upper_knuckle).union(lower_knuckle)


def _insert() -> cq.Workplane:
    insert_bottom_z = BODY_FLOOR - 0.00015
    insert_h = INSERT_TOP_Z - insert_bottom_z
    insert_center_z = insert_bottom_z + insert_h / 2.0
    base = cq.Workplane("XY").box(INSERT_W, INSERT_D, insert_h).translate((0.0, 0.0, insert_center_z))
    bridge = cq.Workplane("XY").box(0.0125, 0.0050, 0.0032).translate((0.0, 0.0004, BODY_H + 0.00025))
    left_support = cq.Workplane("XY").box(SUPPORT_T, SUPPORT_D, SUPPORT_H).translate(
        (SUPPORT_X, SUPPORT_Y, SUPPORT_Z)
    )
    right_support = cq.Workplane("XY").box(SUPPORT_T, SUPPORT_D, SUPPORT_H).translate(
        (-SUPPORT_X, SUPPORT_Y, SUPPORT_Z)
    )
    left_bore = _x_cylinder(0.00088, SUPPORT_T + 0.0020, (SUPPORT_X, WHEEL_Y, WHEEL_Z))
    right_bore = _x_cylinder(0.00088, SUPPORT_T + 0.0020, (-SUPPORT_X, WHEEL_Y, WHEEL_Z))
    supports = left_support.union(right_support).cut(left_bore).cut(right_bore)
    return base.union(bridge).union(supports)


def _chimney() -> cq.Workplane:
    outer = cq.Workplane("XY").box(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H).translate(
        (0.0, CHIMNEY_Y, CHIMNEY_BASE_Z + CHIMNEY_H / 2.0)
    )
    inner = cq.Workplane("XY").box(
        CHIMNEY_W - 2.0 * CHIMNEY_WALL,
        CHIMNEY_D - 2.0 * CHIMNEY_WALL,
        CHIMNEY_H + 0.004,
    ).translate((0.0, CHIMNEY_Y, CHIMNEY_BASE_Z + CHIMNEY_H / 2.0))
    chimney = outer.cut(inner)
    wheel_window = cq.Workplane("XY").box(0.0088, 0.0062, 0.0080).translate(
        (0.0, CHIMNEY_Y + 0.0010, CHIMNEY_BASE_Z + 0.00355)
    )
    chimney = chimney.cut(wheel_window)
    hole_points = [(sx, sz) for sz in CHIMNEY_HOLE_ZS for sx in (-CHIMNEY_HOLE_X, CHIMNEY_HOLE_X)]
    chimney = chimney.faces(">Y").workplane().pushPoints(hole_points).hole(CHIMNEY_HOLE_D)
    chimney = chimney.faces("<Y").workplane().pushPoints(hole_points).hole(CHIMNEY_HOLE_D)
    return chimney


def _cap_shell() -> cq.Workplane:
    center_y = CAP_AXIS_TO_REAR + CAP_D / 2.0
    center_z = CAP_H / 2.0 - CAP_AXIS_TO_BOTTOM
    outer = (
        cq.Workplane("XY")
        .box(CAP_W, CAP_D, CAP_H)
        .translate((0.0, center_y, center_z))
        .edges("|Z")
        .fillet(CORNER_R)
    )
    inner = cq.Workplane("XY").box(
        CAP_W - 2.0 * CAP_WALL,
        CAP_D - 2.0 * CAP_WALL,
        CAP_H + 0.010,
    ).translate((0.0, center_y, CAP_H - CAP_TOP - (CAP_H + 0.010) / 2.0 - CAP_AXIS_TO_BOTTOM))
    shell = outer.cut(inner)
    leaf = cq.Workplane("XY").box(HINGE_LEAF_W, HINGE_LEAF_T, 0.0080).translate(
        (HINGE_X, CAP_AXIS_TO_REAR - HINGE_LEAF_T / 2.0, 0.0044)
    )
    knuckle = _x_cylinder(KNUCKLE_R, CAP_KNUCKLE_LEN, (HINGE_X, 0.0, 0.0))
    return shell.union(leaf).union(knuckle)


def _thumb_wheel() -> cq.Workplane:
    wheel = _x_cylinder(WHEEL_R, WHEEL_LEN, (0.0, 0.0, 0.0))
    shaft = _x_cylinder(SHAFT_R, SHAFT_LEN, (0.0, 0.0, 0.0))
    left_coller = _x_cylinder(0.0010, 0.0007, (-(WHEEL_LEN / 2.0 + 0.0013), 0.0, 0.0))
    right_coller = _x_cylinder(0.0010, 0.0007, ((WHEEL_LEN / 2.0 + 0.0013), 0.0, 0.0))
    return shaft.union(wheel).union(left_coller).union(right_coller)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windproof_lighter")

    chrome = model.material("chrome", rgba=(0.77, 0.78, 0.80, 1.0))
    insert_steel = model.material("insert_steel", rgba=(0.58, 0.59, 0.61, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.42, 0.43, 0.45, 1.0))

    case = model.part("case")
    case.visual(_mesh(_body_shell(), "lighter_case_shell"), material=chrome, name="case_shell")
    case.visual(_mesh(_body_hinge(), "lighter_body_hinge"), material=chrome, name="body_hinge")
    case.visual(_mesh(_insert(), "lighter_insert"), material=insert_steel, name="insert")
    case.visual(_mesh(_chimney(), "lighter_chimney"), material=insert_steel, name="chimney")

    cap = model.part("cap")
    cap.visual(_mesh(_cap_shell(), "lighter_cap"), material=chrome, name="cap_shell")

    wheel = model.part("wheel")
    wheel.visual(_mesh(_thumb_wheel(), "lighter_thumb_wheel"), material=wheel_steel, name="thumb_wheel")

    model.articulation(
        "case_to_cap",
        ArticulationType.REVOLUTE,
        parent=case,
        child=cap,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=7.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "case_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=35.0),
    )

    return model


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    cap = object_model.get_part("cap")
    wheel = object_model.get_part("wheel")
    cap_hinge = object_model.get_articulation("case_to_cap")

    with ctx.pose({cap_hinge: 0.0}):
        ctx.expect_overlap(
            cap,
            case,
            axes="xy",
            min_overlap=0.012,
            name="closed cap covers the lighter body footprint",
        )

        wheel_aabb = ctx.part_element_world_aabb(wheel, elem="thumb_wheel")
        chimney_aabb = ctx.part_element_world_aabb(case, elem="chimney")
        wheel_center = _center_from_aabb(wheel_aabb)
        chimney_center = _center_from_aabb(chimney_aabb)
        ctx.check(
            "thumb wheel sits ahead of the chimney",
            wheel_center is not None
            and chimney_center is not None
            and wheel_center[1] > chimney_center[1] + 0.0015
            and wheel_center[2] < chimney_aabb[1][2]
            and abs(wheel_center[0] - chimney_center[0]) < 0.0008,
            details=f"wheel_center={wheel_center}, chimney_center={chimney_center}, chimney_aabb={chimney_aabb}",
        )

    closed_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_shell")
    with ctx.pose({cap_hinge: 1.75}):
        open_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_shell")

    open_ok = False
    if closed_cap_aabb is not None and open_cap_aabb is not None:
        open_ok = (
            isfinite(open_cap_aabb[1][2])
            and open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.001
            and open_cap_aabb[0][1] < closed_cap_aabb[0][1] - 0.010
        )
    ctx.check(
        "cap flips up and behind the body",
        open_ok,
        details=f"closed_cap_aabb={closed_cap_aabb}, open_cap_aabb={open_cap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
