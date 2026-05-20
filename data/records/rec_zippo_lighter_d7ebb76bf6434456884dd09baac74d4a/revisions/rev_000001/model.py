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


CASE_WIDTH = 0.038
CASE_DEPTH = 0.016
BODY_HEIGHT = 0.055
CAP_HEIGHT = 0.026
CASE_WALL = 0.00125
SEAM_GAP = 0.0008
HINGE_RADIUS = 0.00155
HINGE_X = -CASE_WIDTH / 2.0 - 0.0026

CHIMNEY_WIDTH = 0.019
CHIMNEY_DEPTH = 0.0105
CHIMNEY_HEIGHT = 0.024
WHEEL_CENTER = (0.0072, 0.0, 0.0165)


def _cylinder_between_y(x: float, y0: float, z: float, length: float, radius: float) -> cq.Shape:
    return cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(x, y0, z),
        cq.Vector(0.0, 1.0, 0.0),
    )


def _cylinder_between_x(x0: float, y: float, z: float, length: float, radius: float) -> cq.Shape:
    return cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(x0, y, z),
        cq.Vector(1.0, 0.0, 0.0),
    )


def _case_body_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(CASE_WIDTH, CASE_DEPTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.0022)
    )


def _chimney_shape() -> cq.Workplane:
    w = CHIMNEY_WIDTH
    d = CHIMNEY_DEPTH
    h = CHIMNEY_HEIGHT
    wall = 0.00085

    outer = (
        cq.Workplane("XY")
        .box(w, d, h)
        .translate((0.0, 0.0, h / 2.0))
        .edges("|Z")
        .fillet(0.0007)
    )
    inner = (
        cq.Workplane("XY")
        .box(w - 2.0 * wall, d - 2.0 * wall, h + 0.002)
        .translate((0.0, 0.0, h / 2.0 + 0.001))
    )
    chimney = outer.cut(inner)

    base_flange = (
        cq.Workplane("XY")
        .box(w + 0.0035, d + 0.0014, 0.0012)
        .translate((0.0, 0.0, 0.0006))
    )
    chimney = chimney.union(base_flange)

    hole_radius = 0.00105
    y_cutter_len = d + 0.006
    for z in (0.0055, 0.0096, 0.0137, 0.0178, 0.0212):
        row_shift = 0.0022 if int(round(z * 10000)) % 2 else 0.0
        for x in (-0.0053 + row_shift, 0.0 + row_shift, 0.0053 + row_shift):
            if abs(x) < w / 2.0 - 0.0022:
                chimney = chimney.cut(_cylinder_between_y(x, -y_cutter_len / 2.0, z, y_cutter_len, hole_radius))

    x_cutter_len = w + 0.006
    for z in (0.0072, 0.0118, 0.0164, 0.0206):
        for y in (-0.0024, 0.0024):
            chimney = chimney.cut(_cylinder_between_x(-x_cutter_len / 2.0, y, z, x_cutter_len, 0.00082))

    # Fork cheeks and a real clearance shaft for the continuous striker wheel.
    wx, wy, wz = WHEEL_CENTER
    cheek_y = CHIMNEY_DEPTH / 2.0 - 0.00045
    for y in (-cheek_y, cheek_y):
        cheek = (
            cq.Workplane("XY")
            .box(0.0024, 0.0012, 0.011)
            .translate((wx, y, wz - 0.001))
        )
        chimney = chimney.union(cheek)

    shaft = cq.Workplane("XY").add(_cylinder_between_y(wx, -0.005, wz, 0.010, 0.00058))
    chimney = chimney.union(shaft)

    wick = (
        cq.Workplane("XY")
        .circle(0.00115)
        .extrude(0.011)
        .translate((-0.0032, 0.0, 0.0012))
    )
    return chimney.union(wick)


def _striker_wheel_shape() -> cq.Workplane:
    teeth = 28
    r_outer = 0.0041
    r_inner = 0.00355
    points = []
    for i in range(teeth * 2):
        a = 2.0 * math.pi * i / (teeth * 2)
        r = r_outer if i % 2 == 0 else r_inner
        points.append((r * math.cos(a), r * math.sin(a)))

    wheel = (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(0.0042)
        .translate((0.0, -0.0021, 0.0))
    )
    bore = (
        cq.Workplane("XZ")
        .circle(0.00115)
        .extrude(0.0054)
        .translate((0.0, -0.0027, 0.0))
    )
    side_relief = (
        cq.Workplane("XZ")
        .circle(0.0023)
        .extrude(0.0048)
        .translate((0.0, -0.0024, 0.0))
    )
    return wheel.cut(bore).cut(side_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windproof_flip_top_lighter")

    chrome = model.material("polished_chrome", color=(0.82, 0.84, 0.86, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.58, 0.60, 0.58, 1.0))
    dark_knurl = model.material("dark_knurled_steel", color=(0.12, 0.12, 0.11, 1.0))
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_case_body_shape(), "chrome_body", tolerance=0.00035),
        material=chrome,
        name="chrome_body",
    )
    body.visual(
        Box((0.0044, 0.0066, 0.024)),
        origin=Origin(xyz=(HINGE_X + 0.0018, 0.0, BODY_HEIGHT - 0.012)),
        material=chrome,
        name="body_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.018),
        origin=Origin(xyz=(HINGE_X, 0.0, BODY_HEIGHT - 0.014)),
        material=chrome,
        name="lower_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.012),
        origin=Origin(xyz=(HINGE_X, 0.0, BODY_HEIGHT + 0.001)),
        material=chrome,
        name="upper_hinge_barrel",
    )

    cap = model.part("cap")
    cap_center_x = -HINGE_X
    cap.visual(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_WALL)),
        origin=Origin(xyz=(cap_center_x, 0.0, SEAM_GAP + CAP_HEIGHT - CASE_WALL / 2.0)),
        material=chrome,
        name="cap_roof",
    )
    cap.visual(
        Box((CASE_WIDTH, CASE_WALL, CAP_HEIGHT)),
        origin=Origin(xyz=(cap_center_x, -CASE_DEPTH / 2.0 + CASE_WALL / 2.0, SEAM_GAP + CAP_HEIGHT / 2.0)),
        material=chrome,
        name="front_wall",
    )
    cap.visual(
        Box((CASE_WIDTH, CASE_WALL, CAP_HEIGHT)),
        origin=Origin(xyz=(cap_center_x, CASE_DEPTH / 2.0 - CASE_WALL / 2.0, SEAM_GAP + CAP_HEIGHT / 2.0)),
        material=chrome,
        name="rear_wall",
    )
    cap.visual(
        Box((CASE_WALL, CASE_DEPTH, CAP_HEIGHT)),
        origin=Origin(xyz=(cap_center_x - CASE_WIDTH / 2.0 + CASE_WALL / 2.0, 0.0, SEAM_GAP + CAP_HEIGHT / 2.0)),
        material=chrome,
        name="hinge_wall",
    )
    cap.visual(
        Box((CASE_WALL, CASE_DEPTH, CAP_HEIGHT)),
        origin=Origin(xyz=(cap_center_x + CASE_WIDTH / 2.0 - CASE_WALL / 2.0, 0.0, SEAM_GAP + CAP_HEIGHT / 2.0)),
        material=chrome,
        name="free_wall",
    )
    cap.visual(
        Box((0.0048, 0.0066, 0.018)),
        origin=Origin(xyz=(0.0022, 0.0, SEAM_GAP + 0.0190)),
        material=chrome,
        name="hinge_leaf",
    )
    cap.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, SEAM_GAP + 0.0152)),
        material=chrome,
        name="hinge_barrel",
    )

    chimney = model.part("chimney")
    chimney.visual(
        mesh_from_cadquery(_chimney_shape(), "perforated_chimney", tolerance=0.00025),
        material=brushed_steel,
        name="chimney_insert",
    )

    wheel = model.part("striker_wheel")
    wheel.visual(
        mesh_from_cadquery(_striker_wheel_shape(), "striker_wheel", tolerance=0.00018),
        material=dark_knurl,
        name="knurled_wheel",
    )

    model.articulation(
        "body_to_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(HINGE_X, 0.0, BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=2.05),
    )

    model.articulation(
        "body_to_chimney",
        ArticulationType.FIXED,
        parent=body,
        child=chimney,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
    )

    model.articulation(
        "chimney_to_striker_wheel",
        ArticulationType.CONTINUOUS,
        parent=chimney,
        child=wheel,
        origin=Origin(xyz=WHEEL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=24.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    chimney = object_model.get_part("chimney")
    wheel = object_model.get_part("striker_wheel")
    cap_hinge = object_model.get_articulation("body_to_cap")
    wheel_joint = object_model.get_articulation("chimney_to_striker_wheel")

    with ctx.pose({cap_hinge: 0.0}):
        cap_wall_aabb = ctx.part_element_world_aabb(cap, elem="front_wall")
        seam_gap = None if cap_wall_aabb is None else cap_wall_aabb[0][2] - BODY_HEIGHT
        ctx.check(
            "closed cap skirt has realistic seam clearance",
            seam_gap is not None and 0.0004 <= seam_gap <= 0.0012,
            details=f"seam_gap={seam_gap}, cap_wall_aabb={cap_wall_aabb}",
        )
        ctx.expect_contact(
            cap,
            body,
            elem_a="hinge_barrel",
            elem_b="upper_hinge_barrel",
            contact_tol=0.0003,
            name="cap hinge barrel bears on the body hinge knuckle",
        )
        ctx.expect_overlap(
            cap,
            body,
            axes="xy",
            min_overlap=0.012,
            name="closed cap footprint overlaps the rectangular body",
        )
        ctx.expect_within(
            chimney,
            cap,
            axes="xy",
            margin=0.002,
            name="chimney and striker assembly fit inside the closed cap outline",
        )

    closed_aabb = ctx.part_world_aabb(cap)
    with ctx.pose({cap_hinge: 2.05}):
        open_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "cap swings out around the side hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.015,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        spun_wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "striker wheel spins continuously on a fixed shaft center",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_wheel_pos, spun_wheel_pos)),
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
