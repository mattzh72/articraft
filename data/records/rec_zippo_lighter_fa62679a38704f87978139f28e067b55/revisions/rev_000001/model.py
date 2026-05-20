from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.038
DEPTH = 0.014
BODY_H = 0.040
LID_H = 0.020
WALL = 0.0016
HINGE_OFFSET = 0.0022
HINGE_R = 0.00155
HINGE_X = -WIDTH / 2.0 - HINGE_OFFSET
OPEN_ANGLE = math.radians(108.0)

INSERT_W = 0.029
INSERT_D = 0.0090
INSERT_BASE_Z = WALL
INSERT_BODY_H = 0.042
INSERT_TOP_Z = INSERT_BASE_Z + INSERT_BODY_H
WHEEL_CENTER = (0.0075, 0.0, 0.0535)
CAM_PIVOT = (-0.0155, -0.0022, 0.0490)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _safe_fillet(shape: cq.Workplane, selector: str, radius: float) -> cq.Workplane:
    try:
        return shape.edges(selector).fillet(radius)
    except Exception:
        return shape


def _body_shell_cq() -> cq.Workplane:
    outer = cq.Workplane("XY").box(WIDTH, DEPTH, BODY_H, centered=(True, True, False))
    outer = _safe_fillet(outer, "|Z", 0.0018)

    inner = (
        cq.Workplane("XY")
        .box(WIDTH - 2.0 * WALL, DEPTH - 2.0 * WALL, BODY_H + 0.006, centered=(True, True, False))
        .translate((0.0, 0.0, WALL))
    )
    shell = outer.cut(inner)

    leaf = _cq_box(
        (HINGE_OFFSET + 1.7 * HINGE_R, 0.0048, BODY_H * 0.82),
        (-WIDTH / 2.0 - HINGE_OFFSET * 0.52, 0.0, BODY_H * 0.41 + 0.002),
    )
    barrel = (
        cq.Workplane("XY")
        .cylinder(BODY_H * 0.84, HINGE_R)
        .translate((HINGE_X, 0.0, BODY_H * 0.42 + 0.002))
    )
    return shell.union(leaf).union(barrel)


def _lid_shell_cq() -> cq.Workplane:
    closed_center_x = -HINGE_X
    outer = (
        cq.Workplane("XY")
        .box(WIDTH, DEPTH, LID_H, centered=(True, True, False))
        .translate((closed_center_x, 0.0, 0.0))
    )
    outer = _safe_fillet(outer, "|Z", 0.0015)

    inner = (
        cq.Workplane("XY")
        .box(WIDTH - 2.0 * WALL, DEPTH - 2.0 * WALL, LID_H, centered=(True, True, False))
        .translate((closed_center_x, 0.0, -WALL))
    )
    cap = outer.cut(inner)

    leaf = _cq_box(
        (HINGE_OFFSET + 1.7 * HINGE_R, 0.0042, LID_H * 0.74),
        (HINGE_OFFSET * 0.50, 0.0, LID_H * 0.42),
    )
    barrel = (
        cq.Workplane("XY")
        .cylinder(LID_H * 0.86, HINGE_R)
        .translate((0.0, 0.0, LID_H * 0.43))
    )
    cam_rub = _cq_box(
        (0.004, 0.0012, 0.006),
        (HINGE_OFFSET + 0.001, -DEPTH / 2.0 + WALL * 1.2, LID_H * 0.40),
    )
    cap = cap.union(leaf).union(barrel).union(cam_rub)
    return cap.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(OPEN_ANGLE))


def _insert_cq() -> cq.Workplane:
    body = _cq_box(
        (INSERT_W, INSERT_D, INSERT_BODY_H),
        (0.0, 0.0, INSERT_BASE_Z + INSERT_BODY_H / 2.0),
    )
    body = _safe_fillet(body, "|Z", 0.0009)

    deck = _cq_box((INSERT_W + 0.0010, INSERT_D + 0.0007, 0.0022), (0.0, 0.0, INSERT_TOP_Z + 0.00075))
    insert = body.union(deck)

    chimney_z = INSERT_TOP_Z + 0.0068
    chimney_h = 0.0140
    insert = insert.union(_cq_box((0.0110, 0.00075, chimney_h), (-0.0045, INSERT_D / 2.0 - 0.00035, chimney_z)))
    insert = insert.union(_cq_box((0.0110, 0.00075, chimney_h), (-0.0045, -INSERT_D / 2.0 + 0.00035, chimney_z)))
    insert = insert.union(_cq_box((0.0009, INSERT_D, chimney_h * 0.72), (-0.0100, 0.0, chimney_z - 0.0012)))

    lug_h = 0.0145
    lug_z = WHEEL_CENTER[2] - 0.0015
    for y in (-INSERT_D / 2.0 + 0.00045, INSERT_D / 2.0 - 0.00045):
        insert = insert.union(_cq_box((0.0030, 0.0009, lug_h), (WHEEL_CENTER[0], y, lug_z)))

    cam_pedestal = _cq_box((0.0038, 0.0054, 0.0082), (CAM_PIVOT[0] + 0.0007, CAM_PIVOT[1], CAM_PIVOT[2] - 0.0036))
    insert = insert.union(cam_pedestal)
    for y in (CAM_PIVOT[1] - 0.0020, CAM_PIVOT[1] + 0.0020):
        insert = insert.union(_cq_box((0.0024, 0.0008, 0.0034), (CAM_PIVOT[0], y, CAM_PIVOT[2])))

    return insert


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="matte_pocket_lighter")

    matte_graphite = model.material("matte_graphite", rgba=(0.055, 0.058, 0.060, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.68, 0.68, 0.64, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.11, 0.11, 0.105, 1.0))
    char_black = model.material("char_black", rgba=(0.010, 0.009, 0.007, 1.0))
    wick_fiber = model.material("wick_fiber", rgba=(0.70, 0.55, 0.34, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.38, 0.39, 0.37, 1.0))

    outer_case = model.part("outer_case")
    outer_case.visual(
        Box((WIDTH, DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=matte_graphite,
        name="case_floor",
    )
    outer_case.visual(
        Box((WIDTH, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - WALL / 2.0, BODY_H / 2.0)),
        material=matte_graphite,
        name="front_wall",
    )
    outer_case.visual(
        Box((WIDTH, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + WALL / 2.0, BODY_H / 2.0)),
        material=matte_graphite,
        name="rear_wall",
    )
    outer_case.visual(
        Box((WALL, DEPTH, BODY_H)),
        origin=Origin(xyz=(WIDTH / 2.0 - WALL / 2.0, 0.0, BODY_H / 2.0)),
        material=matte_graphite,
        name="free_side_wall",
    )
    outer_case.visual(
        Box((WALL, DEPTH, BODY_H)),
        origin=Origin(xyz=(-WIDTH / 2.0 + WALL / 2.0, 0.0, BODY_H / 2.0)),
        material=matte_graphite,
        name="hinge_side_wall",
    )
    outer_case.visual(
        Box((HINGE_OFFSET + 1.7 * HINGE_R, 0.0048, BODY_H * 0.82)),
        origin=Origin(xyz=(-WIDTH / 2.0 - HINGE_OFFSET * 0.52, 0.0, BODY_H * 0.41 + 0.002)),
        material=matte_graphite,
        name="hinge_leaf",
    )
    outer_case.visual(
        Cylinder(radius=HINGE_R, length=BODY_H * 0.96),
        origin=Origin(xyz=(HINGE_X, 0.0, BODY_H * 0.48)),
        material=matte_graphite,
        name="body_hinge_barrel",
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_insert_cq(), "insert_metal", tolerance=0.00035, angular_tolerance=0.08),
        material=satin_steel,
        name="insert_metal",
    )
    insert.visual(
        Cylinder(radius=0.0022, length=0.00045),
        origin=Origin(xyz=(-0.0046, 0.0, INSERT_TOP_Z + 0.002075)),
        material=char_black,
        name="wick_opening",
    )
    insert.visual(
        Cylinder(radius=0.0010, length=0.0060),
        origin=Origin(xyz=(-0.0046, 0.0, INSERT_TOP_Z + 0.0051)),
        material=wick_fiber,
        name="wick",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_cq(), "lid_shell", tolerance=0.00045, angular_tolerance=0.08),
        material=matte_graphite,
        name="lid_shell",
    )

    striker_wheel = model.part("striker_wheel")
    wheel_mesh = mesh_from_geometry(
        KnobGeometry(
            0.0082,
            0.0072,
            body_style="cylindrical",
            grip=KnobGrip(style="knurled", count=28, depth=0.00045, helix_angle_deg=18.0),
            edge_radius=0.00025,
        ),
        "striker_wheel_knurled",
    )
    striker_wheel.visual(
        wheel_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_disk",
    )

    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        Cylinder(radius=0.00105, length=0.0026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spring_steel,
        name="cam_pivot",
    )
    cam_lever.visual(
        Box((0.0072, 0.0010, 0.00135)),
        origin=Origin(xyz=(0.0039, 0.0, 0.00125)),
        material=spring_steel,
        name="cam_arm",
    )
    cam_lever.visual(
        Cylinder(radius=0.00072, length=0.0012),
        origin=Origin(xyz=(0.0076, 0.0, 0.00125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spring_steel,
        name="cam_tip",
    )

    model.articulation(
        "case_to_insert",
        ArticulationType.FIXED,
        parent=outer_case,
        child=insert,
        origin=Origin(),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=outer_case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-OPEN_ANGLE, upper=0.0),
    )
    model.articulation(
        "insert_to_striker_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=WHEEL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=30.0),
    )
    model.articulation(
        "insert_to_cam_lever",
        ArticulationType.REVOLUTE,
        parent=insert,
        child=cam_lever,
        origin=Origin(xyz=CAM_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0, lower=0.0, upper=0.75),
        mimic=Mimic("case_to_lid", multiplier=-0.34, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_case = object_model.get_part("outer_case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("striker_wheel")
    cam = object_model.get_part("cam_lever")
    lid_joint = object_model.get_articulation("case_to_lid")
    wheel_joint = object_model.get_articulation("insert_to_striker_wheel")

    ctx.expect_contact(
        insert,
        outer_case,
        elem_a="insert_metal",
        elem_b="case_floor",
        contact_tol=0.0008,
        name="insert seats on the lower case floor",
    )
    ctx.expect_within(
        insert,
        outer_case,
        axes="xy",
        inner_elem="insert_metal",
        margin=0.0005,
        name="insert remains a distinct piece inside the outer case footprint",
    )
    ctx.expect_origin_distance(
        lid,
        insert,
        axes="xy",
        min_dist=0.010,
        name="display pose keeps lid swung away from visible insert",
    )
    ctx.expect_overlap(
        wheel,
        insert,
        axes="xy",
        elem_a="wheel_disk",
        elem_b="insert_metal",
        min_overlap=0.0035,
        name="striker wheel sits across the top of the insert",
    )
    ctx.expect_origin_distance(
        cam,
        insert,
        axes="xy",
        max_dist=0.020,
        name="cam lever pivot is supported beside the insert hinge side",
    )
    ctx.check(
        "striker wheel uses continuous rotation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulation_type={wheel_joint.articulation_type!r}",
    )

    open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    open_cam_aabb = ctx.part_element_world_aabb(cam, elem="cam_arm")
    with ctx.pose({lid_joint: -OPEN_ANGLE}):
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        closed_cam_aabb = ctx.part_element_world_aabb(cam, elem="cam_arm")

    def _center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) * 0.5

    def _center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) * 0.5

    ctx.check(
        "side hinge swings the lid away from the case",
        open_lid_aabb is not None
        and closed_lid_aabb is not None
        and _center_y(open_lid_aabb) > _center_y(closed_lid_aabb) + 0.010,
        details=f"open={open_lid_aabb}, closed={closed_lid_aabb}",
    )
    ctx.check(
        "cam lever follows the lid on its local pivot",
        open_cam_aabb is not None
        and closed_cam_aabb is not None
        and abs(_center_z(open_cam_aabb) - _center_z(closed_cam_aabb)) > 0.0008,
        details=f"open={open_cam_aabb}, closed={closed_cam_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
