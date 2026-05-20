from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_W = 0.038
CASE_D = 0.014
BODY_H = 0.041
LID_H = 0.019

CASE_CORNER_R = 0.0033
BODY_WALL = 0.0011
BODY_FLOOR = 0.0014
LID_WALL = 0.0008
LID_TOP = 0.0010

HINGE_AXIS_OFFSET = 0.0011
HINGE_AXIS_Z = BODY_H + 0.0036
HINGE_RADIUS = 0.00125
BODY_KNUCKLE_LEN = 0.0052
LID_KNUCKLE_LEN = 0.0030
BODY_KNUCKLE_Y = 0.0044

INSERT_SKIRT_W = 0.0345
INSERT_SKIRT_D = 0.0106
INSERT_SKIRT_H = 0.0395
INSERT_Z0 = 0.0020

INSERT_COLLAR_W = 0.0354
INSERT_COLLAR_D = 0.0112
INSERT_COLLAR_H = 0.0026
INSERT_COLLAR_Z = BODY_H - 0.0012

CHIMNEY_W = 0.0260
CHIMNEY_D = 0.0094
CHIMNEY_H = 0.0158
CHIMNEY_WALL = 0.0010
CHIMNEY_Z = BODY_H + 0.0002

WHEEL_RADIUS = 0.0028
WHEEL_TREAD_LEN = 0.0050
WHEEL_TOTAL_LEN = 0.0112
WHEEL_AXLE_RADIUS = 0.0011
WHEEL_Y = 0.0032
WHEEL_Z = BODY_H + 0.0060

LEVER_PIVOT = (-0.0048, -0.0031, 0.0018)


def _base_box(size_x: float, size_y: float, size_z: float, *, center_x: float = 0.0, center_y: float = 0.0, z0: float = 0.0):
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z, centered=(True, True, False))
        .translate((center_x, center_y, z0))
    )


def _centered_box(size_x: float, size_y: float, size_z: float, *, center_xyz: tuple[float, float, float]):
    cx, cy, cz = center_xyz
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate((cx, cy, cz))


def _x_cylinder(radius: float, length: float, *, center_xyz: tuple[float, float, float]):
    cx, cy, cz = center_xyz
    return (
        cq.Workplane("YZ")
        .center(cy, cz)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((cx, 0.0, 0.0))
    )


def _y_cylinder(radius: float, length: float, *, center_xyz: tuple[float, float, float]):
    cx, cy, cz = center_xyz
    return (
        cq.Workplane("XZ")
        .center(cx, cz)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, cy, 0.0))
    )


def _make_case_shell() -> cq.Workplane:
    body = _base_box(CASE_W, CASE_D, BODY_H).edges("|Z").fillet(CASE_CORNER_R)
    cavity = _base_box(
        CASE_W - (2.0 * BODY_WALL),
        CASE_D - (2.0 * BODY_WALL),
        BODY_H - BODY_FLOOR + 0.003,
        z0=BODY_FLOOR,
    )
    return body.cut(cavity)


def _make_lid_shell() -> cq.Workplane:
    lid_bottom_local_z = BODY_H - HINGE_AXIS_Z
    shell = (
        _base_box(
            CASE_W,
            CASE_D,
            LID_H,
            center_x=-(CASE_W / 2.0 + HINGE_AXIS_OFFSET),
            z0=lid_bottom_local_z,
        )
        .edges("|Z")
        .fillet(CASE_CORNER_R * 0.85)
    )
    cavity = _base_box(
        CASE_W - (2.0 * LID_WALL),
        CASE_D - (2.0 * LID_WALL),
        LID_H - LID_TOP,
        center_x=-(CASE_W / 2.0 + HINGE_AXIS_OFFSET),
        z0=lid_bottom_local_z - 0.0001,
    )
    shell = shell.cut(cavity)

    lever_rib = _base_box(
        0.0056,
        0.0018,
        0.0056,
        center_x=-0.0030,
        center_y=LEVER_PIVOT[1],
        z0=-0.0016,
    )
    lever_boss = _y_cylinder(0.00105, 0.0018, center_xyz=LEVER_PIVOT)
    return shell.union(lever_rib).union(lever_boss)


def _make_insert_shell() -> cq.Workplane:
    insert = _base_box(
        INSERT_SKIRT_W,
        INSERT_SKIRT_D,
        INSERT_SKIRT_H,
        z0=INSERT_Z0,
    ).edges("|Z").fillet(0.0024)

    collar = _base_box(
        INSERT_COLLAR_W,
        INSERT_COLLAR_D,
        INSERT_COLLAR_H,
        z0=INSERT_COLLAR_Z,
    )
    chimney = _base_box(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H, z0=CHIMNEY_Z)

    insert = insert.union(collar).union(chimney)

    flue = _base_box(
        CHIMNEY_W - (2.0 * CHIMNEY_WALL),
        CHIMNEY_D - (2.0 * CHIMNEY_WALL),
        CHIMNEY_H - 0.001,
        z0=CHIMNEY_Z + 0.0006,
    )
    insert = insert.cut(flue)
    insert = insert.cut(
        _centered_box(
            0.0135,
            0.0048,
            0.0074,
            center_xyz=(0.0, (CHIMNEY_D / 2.0) + 0.0004, WHEEL_Z),
        )
    )

    for hole_y in (-0.0021, 0.0021):
        for hole_z in (BODY_H + 0.0042, BODY_H + 0.0080, BODY_H + 0.0118):
            insert = insert.cut(
                _x_cylinder(
                    0.00082,
                    CHIMNEY_W + 0.008,
                    center_xyz=(0.0, hole_y, hole_z),
                )
            )

    tab_x = (WHEEL_TOTAL_LEN / 2.0) + 0.0013
    tab_depth = 0.0032
    tab_height = 0.0072
    tab_z0 = WHEEL_Z - (tab_height / 2.0)
    left_tab = _base_box(0.0022, tab_depth, tab_height, center_x=-tab_x, center_y=WHEEL_Y, z0=tab_z0)
    right_tab = _base_box(0.0022, tab_depth, tab_height, center_x=tab_x, center_y=WHEEL_Y, z0=tab_z0)
    axle_seat_x = (WHEEL_TOTAL_LEN / 2.0) + 0.0001
    left_seat = _centered_box(0.0002, 0.0018, 0.0018, center_xyz=(-axle_seat_x, WHEEL_Y, WHEEL_Z))
    right_seat = _centered_box(0.0002, 0.0018, 0.0018, center_xyz=(axle_seat_x, WHEEL_Y, WHEEL_Z))
    insert = insert.union(left_tab).union(right_tab).union(left_seat).union(right_seat)

    return insert


def _make_wheel() -> cq.Workplane:
    axle = _x_cylinder(WHEEL_AXLE_RADIUS, WHEEL_TOTAL_LEN, center_xyz=(0.0, 0.0, 0.0))
    hub = _x_cylinder(WHEEL_RADIUS * 0.8, WHEEL_TOTAL_LEN - 0.0012, center_xyz=(0.0, 0.0, 0.0))
    tread = (
        cq.Workplane("YZ")
        .polygon(16, 2.0 * WHEEL_RADIUS)
        .extrude(WHEEL_TREAD_LEN / 2.0, both=True)
    )
    return axle.union(hub).union(tread)


def _make_cam_lever() -> cq.Workplane:
    hub = _y_cylinder(0.0010, 0.0016, center_xyz=(0.0, 0.0, 0.0))
    arm = _centered_box(0.0070, 0.0013, 0.0021, center_xyz=(-0.0034, 0.0, -0.0005))
    toe = _centered_box(0.0012, 0.0013, 0.0030, center_xyz=(-0.0066, 0.0, 0.0002))
    return hub.union(arm).union(toe)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_lighter")

    case_finish = model.material("case_finish", rgba=(0.77, 0.69, 0.45, 1.0))
    insert_finish = model.material("insert_finish", rgba=(0.73, 0.74, 0.76, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.42, 0.44, 0.46, 1.0))
    lever_finish = model.material("lever_finish", rgba=(0.56, 0.58, 0.60, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_make_case_shell(), "lighter_case_shell"),
        material=case_finish,
        name="case_shell",
    )
    case.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LEN),
        origin=Origin(
            xyz=(CASE_W / 2.0 + HINGE_AXIS_OFFSET, -BODY_KNUCKLE_Y, HINGE_AXIS_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=case_finish,
        name="hinge_knuckle_0",
    )
    case.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LEN),
        origin=Origin(
            xyz=(CASE_W / 2.0 + HINGE_AXIS_OFFSET, BODY_KNUCKLE_Y, HINGE_AXIS_Z),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=case_finish,
        name="hinge_knuckle_1",
    )
    case.visual(
        mesh_from_cadquery(
            _centered_box(
                0.0024,
                BODY_KNUCKLE_LEN,
                0.0032,
                center_xyz=(CASE_W / 2.0 + 0.00055, -BODY_KNUCKLE_Y, HINGE_AXIS_Z - 0.0022),
            ),
            "lighter_hinge_leaf_0",
        ),
        material=case_finish,
        name="hinge_leaf_0",
    )
    case.visual(
        mesh_from_cadquery(
            _centered_box(
                0.0024,
                BODY_KNUCKLE_LEN,
                0.0032,
                center_xyz=(CASE_W / 2.0 + 0.00055, BODY_KNUCKLE_Y, HINGE_AXIS_Z - 0.0022),
            ),
            "lighter_hinge_leaf_1",
        ),
        material=case_finish,
        name="hinge_leaf_1",
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_make_insert_shell(), "lighter_insert"),
        material=insert_finish,
        name="insert_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shell(), "lighter_lid_shell"),
        material=case_finish,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=LID_KNUCKLE_LEN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=case_finish,
        name="lid_knuckle",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_make_wheel(), "lighter_wheel"),
        material=wheel_finish,
        name="striker_wheel",
    )

    cam = model.part("cam")
    cam.visual(
        mesh_from_cadquery(_make_cam_lever(), "lighter_cam"),
        material=lever_finish,
        name="cam_arm",
    )

    model.articulation(
        "case_to_insert",
        ArticulationType.FIXED,
        parent=case,
        child=insert,
        origin=Origin(),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(CASE_W / 2.0 + HINGE_AXIS_OFFSET, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.05, effort=0.6, velocity=5.0),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=20.0),
    )
    model.articulation(
        "lid_cam",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cam,
        origin=Origin(xyz=LEVER_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.7, effort=0.05, velocity=8.0),
        mimic=Mimic(joint="lid_hinge", multiplier=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    cam = object_model.get_part("cam")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("wheel_spin")
    lid_hinge = object_model.get_articulation("lid_hinge")
    lid_limits = lid_hinge.motion_limits

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="case_shell",
        max_gap=0.0008,
        max_penetration=0.0,
        name="lid sits flush on the case seam",
    )
    ctx.expect_overlap(
        lid,
        case,
        axes="xy",
        elem_a="lid_shell",
        elem_b="case_shell",
        min_overlap=0.010,
        name="lid covers the case footprint when closed",
    )
    ctx.expect_within(
        insert,
        case,
        axes="xy",
        inner_elem="insert_shell",
        outer_elem="case_shell",
        margin=0.0016,
        name="insert remains nested inside the case footprint",
    )
    ctx.allow_overlap(
        insert,
        wheel,
        elem_a="insert_shell",
        elem_b="striker_wheel",
        reason="The striker wheel axle is intentionally represented as captured inside simplified insert-side sockets.",
    )

    case_shell_aabb = ctx.part_element_world_aabb(case, elem="case_shell")
    insert_shell_aabb = ctx.part_element_world_aabb(insert, elem="insert_shell")
    ctx.check(
        "chimney rises above the lower shell",
        case_shell_aabb is not None
        and insert_shell_aabb is not None
        and insert_shell_aabb[1][2] > case_shell_aabb[1][2] + 0.012,
        details=f"case={case_shell_aabb}, insert={insert_shell_aabb}",
    )

    wheel_limits = wheel_joint.motion_limits
    ctx.check(
        "striker wheel uses a continuous axle joint",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_limits is not None
        and wheel_limits.lower is None
        and wheel_limits.upper is None,
        details=f"type={wheel_joint.articulation_type}, limits={wheel_limits}",
    )

    if lid_limits is not None and lid_limits.upper is not None:
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        closed_cam_aabb = ctx.part_element_world_aabb(cam, elem="cam_arm")
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
            open_cam_aabb = ctx.part_element_world_aabb(cam, elem="cam_arm")

        ctx.check(
            "lid opens upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and ((open_lid_aabb[0][2] + open_lid_aabb[1][2]) / 2.0) > ((closed_lid_aabb[0][2] + closed_lid_aabb[1][2]) / 2.0) + 0.007,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )
        ctx.check(
            "cam lever follows the lid on its own pivot",
            closed_cam_aabb is not None
            and open_cam_aabb is not None
            and ((open_cam_aabb[0][2] + open_cam_aabb[1][2]) / 2.0) > ((closed_cam_aabb[0][2] + closed_cam_aabb[1][2]) / 2.0) + 0.003,
            details=f"closed={closed_cam_aabb}, open={open_cam_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
