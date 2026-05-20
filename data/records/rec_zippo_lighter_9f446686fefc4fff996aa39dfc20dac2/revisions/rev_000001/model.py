from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.0375
BODY_D = 0.0126
BODY_H = 0.0420
BODY_WALL = 0.00050
BODY_FLOOR = 0.00070

CAP_W = 0.0383
CAP_D = 0.0134
CAP_H = 0.0165
CAP_WALL = 0.00045
CAP_TOP = 0.00065
CAP_OVERLAP = 0.00250

CASE_FILLET = 0.00100
HINGE_R = 0.00105
HINGE_EMBED = 0.00015
HINGE_GAP = 0.00035
BODY_KNUCKLE_H = 0.00420
CAP_KNUCKLE_H = 0.00550
HINGE_Z0 = BODY_H - CAP_OVERLAP + 0.00080

INSERT_W = BODY_W - 2.0 * BODY_WALL - 0.00080
INSERT_D = BODY_D - 2.0 * BODY_WALL - 0.00070
INSERT_Z0 = BODY_FLOOR + 0.00030
INSERT_H = BODY_H - INSERT_Z0

CHIMNEY_W = 0.0138
CHIMNEY_D = 0.0080
CHIMNEY_H = 0.0142
CHIMNEY_WALL = 0.00045
CHIMNEY_Y = 0.00020
CHIMNEY_EMBED = 0.00025

WHEEL_R = 0.00305
WHEEL_W = CHIMNEY_W - 2.0 * CHIMNEY_WALL - 0.00140
AXLE_R = 0.00055
WHEEL_BORE_R = 0.00085
WHEEL_Y = CHIMNEY_Y + CHIMNEY_D / 2.0 - CHIMNEY_WALL - WHEEL_R - 0.00020
WHEEL_Z = BODY_H + 0.00185

HINGE_X = -BODY_W / 2.0 - HINGE_R + HINGE_EMBED
CAP_X0 = HINGE_R - HINGE_EMBED

BODY_KNUCKLE_0_Z = HINGE_Z0
CAP_KNUCKLE_Z = BODY_KNUCKLE_0_Z + BODY_KNUCKLE_H + HINGE_GAP - (BODY_H - CAP_OVERLAP)
BODY_KNUCKLE_1_Z = BODY_KNUCKLE_0_Z + BODY_KNUCKLE_H + HINGE_GAP + CAP_KNUCKLE_H + HINGE_GAP


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _z_cylinder(radius: float, height: float, x: float, y: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z0))


def _x_cylinder(radius: float, length: float, x0: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, y, z))


def make_body_shell() -> cq.Workplane:
    outer = _box_at((BODY_W, BODY_D, BODY_H), (0.0, 0.0, BODY_H / 2.0))
    outer = outer.edges("|Z").fillet(CASE_FILLET)
    cavity = _box_at(
        (BODY_W - 2.0 * BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_H),
        (0.0, 0.0, BODY_FLOOR + BODY_H / 2.0),
    )
    return outer.cut(cavity)


def make_body_hinge() -> cq.Workplane:
    lower = _z_cylinder(HINGE_R, BODY_KNUCKLE_H, HINGE_X, 0.0, BODY_KNUCKLE_0_Z)
    upper = _z_cylinder(HINGE_R, BODY_KNUCKLE_H, HINGE_X, 0.0, BODY_KNUCKLE_1_Z)
    lower_pad = _box_at(
        (0.00120, 0.00360, BODY_KNUCKLE_H),
        (-BODY_W / 2.0 - 0.00025, 0.0, BODY_KNUCKLE_0_Z + BODY_KNUCKLE_H / 2.0),
    )
    upper_pad = _box_at(
        (0.00120, 0.00360, BODY_KNUCKLE_H),
        (-BODY_W / 2.0 - 0.00025, 0.0, BODY_KNUCKLE_1_Z + BODY_KNUCKLE_H / 2.0),
    )
    strap_height = BODY_KNUCKLE_1_Z + BODY_KNUCKLE_H - BODY_KNUCKLE_0_Z
    strap = _box_at(
        (0.00100, 0.00060, strap_height),
        (-BODY_W / 2.0 - 0.00030, -0.00145, BODY_KNUCKLE_0_Z + strap_height / 2.0),
    )
    return lower.union(upper).union(lower_pad).union(upper_pad).union(strap)


def make_cap_hinge() -> cq.Workplane:
    knuckle = _z_cylinder(HINGE_R, CAP_KNUCKLE_H, 0.0, 0.0, CAP_KNUCKLE_Z)
    tab = _box_at(
        (0.00080, 0.00260, CAP_KNUCKLE_H),
        (0.00070, 0.00165, CAP_KNUCKLE_Z + CAP_KNUCKLE_H / 2.0),
    )
    return knuckle.union(tab)


def make_insert_core() -> cq.Workplane:
    core = _box_at(
        (INSERT_W, INSERT_D, INSERT_H),
        (0.0, 0.0, INSERT_Z0 + INSERT_H / 2.0),
    )
    return core.edges("|Z").fillet(0.00060)


def make_chimney() -> cq.Workplane:
    chimney_center_z = BODY_H - CHIMNEY_EMBED + CHIMNEY_H / 2.0
    outer = _box_at((CHIMNEY_W, CHIMNEY_D, CHIMNEY_H), (0.0, CHIMNEY_Y, chimney_center_z))
    inner = _box_at(
        (CHIMNEY_W - 2.0 * CHIMNEY_WALL, CHIMNEY_D - 2.0 * CHIMNEY_WALL, CHIMNEY_H + 0.0020),
        (0.0, CHIMNEY_Y, chimney_center_z),
    )
    chimney = outer.cut(inner)

    hole_radius = 0.00105
    x_positions = (-0.00325, 0.00325)
    z_positions = (
        BODY_H + 0.0025,
        BODY_H + 0.0051,
        BODY_H + 0.0077,
        BODY_H + 0.0103,
    )
    for x_pos in x_positions:
        for z_pos in z_positions:
            cutter = (
                cq.Workplane("XZ", origin=(x_pos, CHIMNEY_Y, z_pos))
                .circle(hole_radius)
                .extrude(CHIMNEY_D + 0.0040, both=True)
            )
            chimney = chimney.cut(cutter)

    shaft = _x_cylinder(AXLE_R, CHIMNEY_W + 0.0010, -(CHIMNEY_W + 0.0010) / 2.0, WHEEL_Y, WHEEL_Z)
    return chimney.union(shaft)


def make_wheel() -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .polygon(18, 2.0 * WHEEL_R)
        .extrude(WHEEL_W)
        .translate((-WHEEL_W / 2.0, 0.0, 0.0))
    )
    bore = _x_cylinder(WHEEL_BORE_R, WHEEL_W + 0.0020, -WHEEL_W / 2.0 - 0.0010, 0.0, 0.0)
    wheel = outer.cut(bore)
    return wheel.edges("|X").fillet(0.00018)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windproof_lighter")

    chrome = model.material("chrome", rgba=(0.82, 0.83, 0.85, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.71, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.43, 0.45, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(make_body_shell(), "body_shell"), material=chrome, name="body_shell")
    body.visual(mesh_from_cadquery(make_body_hinge(), "body_hinge"), material=chrome, name="body_hinge")

    insert = model.part("insert")
    insert.visual(mesh_from_cadquery(make_insert_core(), "insert_core"), material=steel, name="insert_core")
    insert.visual(mesh_from_cadquery(make_chimney(), "chimney"), material=steel, name="chimney")

    cap = model.part("cap")
    shell_height = CAP_H - CAP_TOP
    cap.visual(
        Box((CAP_W, CAP_D, CAP_TOP)),
        origin=Origin(xyz=(CAP_X0 + CAP_W / 2.0, 0.0, CAP_H - CAP_TOP / 2.0)),
        material=chrome,
        name="cap_top",
    )
    cap.visual(
        Box((CAP_W, CAP_WALL, shell_height)),
        origin=Origin(xyz=(CAP_X0 + CAP_W / 2.0, CAP_D / 2.0 - CAP_WALL / 2.0, shell_height / 2.0)),
        material=chrome,
        name="cap_front",
    )
    cap.visual(
        Box((CAP_W, CAP_WALL, shell_height)),
        origin=Origin(xyz=(CAP_X0 + CAP_W / 2.0, -CAP_D / 2.0 + CAP_WALL / 2.0, shell_height / 2.0)),
        material=chrome,
        name="cap_back",
    )
    cap.visual(
        Box((CAP_WALL, 0.00395, shell_height)),
        origin=Origin(xyz=(CAP_X0 + CAP_WALL / 2.0, 0.004275, shell_height / 2.0)),
        material=chrome,
        name="cap_side_front",
    )
    cap.visual(
        Box((CAP_WALL, 0.00395, shell_height)),
        origin=Origin(xyz=(CAP_X0 + CAP_WALL / 2.0, -0.004275, shell_height / 2.0)),
        material=chrome,
        name="cap_side_back",
    )
    cap.visual(
        Box((CAP_WALL, CAP_D - 2.0 * CAP_WALL, shell_height)),
        origin=Origin(xyz=(CAP_X0 + CAP_W - CAP_WALL / 2.0, 0.0, shell_height / 2.0)),
        material=chrome,
        name="cap_side_free",
    )
    cap.visual(mesh_from_cadquery(make_cap_hinge(), "cap_hinge"), material=chrome, name="cap_hinge")

    wheel = model.part("wheel")
    wheel.visual(mesh_from_cadquery(make_wheel(), "wheel"), material=dark_steel, name="wheel")

    model.articulation(
        "body_to_insert",
        ArticulationType.FIXED,
        parent=body,
        child=insert,
        origin=Origin(),
    )
    model.articulation(
        "body_to_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(HINGE_X, 0.0, BODY_H - CAP_OVERLAP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.0, effort=0.3, velocity=8.0),
    )
    model.articulation(
        "insert_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    insert = object_model.get_part("insert")
    cap = object_model.get_part("cap")
    wheel = object_model.get_part("wheel")
    cap_hinge = object_model.get_articulation("body_to_cap")

    with ctx.pose({cap_hinge: 0.0}):
        ctx.expect_overlap(
            cap,
            body,
            axes="xy",
            min_overlap=0.010,
            name="closed cap stays aligned over the case",
        )
        ctx.expect_overlap(
            cap,
            body,
            axes="z",
            min_overlap=CAP_OVERLAP - 0.0002,
            name="closed cap overlaps the body seam",
        )
        ctx.expect_within(
            insert,
            cap,
            axes="xy",
            inner_elem="chimney",
            margin=0.004,
            name="closed cap encloses the chimney footprint",
        )
        ctx.expect_within(
            wheel,
            insert,
            axes="x",
            inner_elem="wheel",
            outer_elem="chimney",
            margin=0.0010,
            name="striker wheel remains between the chimney walls",
        )

    limits = cap_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        closed_aabb = ctx.part_world_aabb(cap)
        with ctx.pose({cap_hinge: limits.upper}):
            open_aabb = ctx.part_world_aabb(cap)
            ctx.expect_gap(
                insert,
                cap,
                axis="x",
                min_gap=0.0005,
                name="open cap clears the insert to the hinge side",
            )
        closed_center_y = None
        open_center_x = None
        if closed_aabb is not None:
            closed_center_x = 0.5 * (closed_aabb[0][0] + closed_aabb[1][0])
        else:
            closed_center_x = None
        if open_aabb is not None:
            open_center_x = 0.5 * (open_aabb[0][0] + open_aabb[1][0])
        ctx.check(
            "cap swings outward from the body",
            closed_center_x is not None and open_center_x is not None and open_center_x < closed_center_x - 0.020,
            details=f"closed_center_x={closed_center_x}, open_center_x={open_center_x}",
        )

    return ctx.report()


object_model = build_object_model()
