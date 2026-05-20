from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
LOWER_H = 0.040
LID_H = 0.018
WALL = 0.00135
CORNER_R = 0.0036
HINGE_X = -CASE_W / 2.0 - 0.0021
HINGE_R = 0.00135


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _vertical_cylinder_at(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return cq.Workplane("XY").cylinder(length, radius).translate(center)


def _y_cylinder_at(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate(center)
    )


def _rounded_rect_prism(
    width: float,
    depth: float,
    height: float,
    radius: float,
    z0: float = 0.0,
) -> cq.Workplane:
    """CadQuery build without relying on optional roundedRect plugin methods."""
    radius = max(0.0, min(radius, width * 0.49, depth * 0.49))
    solid = cq.Workplane("XY").rect(width, depth).extrude(height)
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    if z0:
        solid = solid.translate((0.0, 0.0, z0))
    return solid


def _hollow_lower_shell() -> cq.Workplane:
    """Rounded open cup with visible Zippo-scale side hinge knuckles."""
    outer = _rounded_rect_prism(CASE_W, CASE_D, LOWER_H, CORNER_R)
    inner = _rounded_rect_prism(
        CASE_W - 2.0 * WALL,
        CASE_D - 2.0 * WALL,
        LOWER_H + 0.002,
        CORNER_R - WALL,
        WALL,
    )
    shell = outer.cut(inner)

    # The hinge leaf is buried into the left wall and makes the barrels read as
    # mounted instead of ornamental floating cylinders.
    for zc, length in ((0.0105, 0.013), (0.0305, 0.013)):
        shell = shell.union(_box_at((0.0025, 0.0046, length), (HINGE_X + 0.0012, 0.0, zc)))
        shell = shell.union(_vertical_cylinder_at(HINGE_R, length, (HINGE_X, 0.0, zc)))

    return shell


def _cam_pivot_support() -> cq.Workplane:
    support = _box_at((0.0050, 0.00055, 0.0100), (-0.0170, -0.00205, 0.0415))
    support = support.union(_box_at((0.0050, 0.00055, 0.0100), (-0.0170, 0.00205, 0.0415)))
    support = support.union(_box_at((0.0020, 0.0046, 0.0020), (-0.0185, 0.0, 0.0375)))
    return support


def _hollow_lid() -> cq.Workplane:
    """Open-bottom fitted lid in the child frame whose origin is the hinge axis."""
    body_x = CASE_W / 2.0 - HINGE_X
    z0 = LOWER_H + 0.00055
    outer = _rounded_rect_prism(CASE_W, CASE_D, LID_H, CORNER_R, z0).translate((body_x, 0.0, 0.0))
    inner = _rounded_rect_prism(
        CASE_W - 2.0 * WALL,
        CASE_D - 2.0 * WALL,
        LID_H - WALL + 0.001,
        CORNER_R - WALL,
        z0 - 0.001,
    ).translate((body_x, 0.0, 0.0))
    lid = outer.cut(inner)

    # Upper hinge leaf and knuckle on the moving lid.
    hinge_zc = LOWER_H + 0.0095
    hinge_len = 0.0150
    lid = lid.union(_box_at((0.0230, 0.0044, hinge_len), (0.0107, 0.0, hinge_zc)))
    lid = lid.union(_vertical_cylinder_at(HINGE_R, hinge_len, (0.0, 0.0, hinge_zc)))

    # A tiny internal cam strike pad along the hinged lid edge.
    lid = lid.union(_box_at((0.0030, 0.0032, 0.0065), (0.0226, 0.0, LOWER_H + 0.0058)))
    return lid


def _rounded_solid(width: float, depth: float, height: float, z0: float, radius: float) -> cq.Workplane:
    return _rounded_rect_prism(width, depth, height, radius, z0)


def _chimney_insert() -> cq.Workplane:
    """Nested insert body with a perforated windscreen, striker supports, and wick."""
    insert = _rounded_solid(0.0280, 0.0090, LOWER_H - WALL, WALL, 0.0018)
    insert = insert.union(_rounded_solid(0.0310, 0.0100, 0.0018, LOWER_H - 0.0010, 0.0020))

    # Tall perforated chimney.  Its front/back plates are joined by side posts
    # and top/bottom bridge bands so the insert remains one manufactured piece.
    cw = 0.0190
    cd = 0.0084
    ch = 0.0152
    t = 0.00070
    z0 = LOWER_H + 0.00045
    chimney = _box_at((cw, t, ch), (0.0, -cd / 2.0 + t / 2.0, z0 + ch / 2.0))
    chimney = chimney.union(_box_at((cw, t, ch), (0.0, cd / 2.0 - t / 2.0, z0 + ch / 2.0)))
    chimney = chimney.union(_box_at((t, cd, ch), (-cw / 2.0 + t / 2.0, 0.0, z0 + ch / 2.0)))
    chimney = chimney.union(_box_at((t, cd, ch), (cw / 2.0 - t / 2.0, 0.0, z0 + ch / 2.0)))
    chimney = chimney.union(_box_at((cw, cd, 0.0010), (0.0, 0.0, z0 + 0.0005)))
    chimney = chimney.union(_box_at((cw, cd, 0.0010), (0.0, 0.0, z0 + ch - 0.0005)))
    for row_z in (z0 + 0.0032, z0 + 0.0064, z0 + 0.0096, z0 + 0.0128):
        for hx in (-0.0058, -0.0029, 0.0, 0.0029, 0.0058):
            hole = _y_cylinder_at(0.00075, cd * 3.0, (hx, 0.0, row_z))
            chimney = chimney.cut(hole)
    insert = insert.union(chimney)

    # Striker-wheel fork and flint tube are part of the fixed insert.  The wheel
    # itself is a separate continuously rotating child part.
    wx = 0.0132
    wz = LOWER_H + 0.0080
    insert = insert.union(_box_at((0.0044, 0.00055, 0.0100), (wx, -0.00355, wz)))
    insert = insert.union(_box_at((0.0044, 0.00055, 0.0100), (wx, 0.00355, wz)))
    insert = insert.union(_box_at((0.0024, 0.0066, 0.0050), (wx - 0.0021, 0.0, LOWER_H + 0.0027)))
    insert = insert.union(_vertical_cylinder_at(0.00115, 0.0160, (wx, 0.0, LOWER_H + 0.0015)))

    # Central dark wick, rooted into the insert body so it is not a floating
    # decorative island even though it protrudes inside the chimney.
    insert = insert.union(_box_at((0.0016, 0.0016, 0.0140), (-0.0025, 0.0, LOWER_H + 0.0063)))
    return insert


def _seam_shadow() -> cq.Workplane:
    outer = _rounded_rect_prism(0.0344, 0.0108, 0.00045, 0.0023)
    inner = _rounded_rect_prism(0.0302, 0.0066, 0.0007, 0.0014, -0.0001)
    return outer.cut(inner).translate((0.0, 0.0, LOWER_H + 0.00005))


def _striker_wheel() -> cq.Workplane:
    radius = 0.00305
    length = 0.0060
    wheel = _y_cylinder_at(radius, length, (0.0, 0.0, 0.0))
    for i in range(18):
        angle = 360.0 * i / 18.0
        tooth = (
            cq.Workplane("XY")
            .box(0.00055, length * 1.04, 0.00115)
            .translate((radius + 0.00023, 0.0, 0.0))
            .rotate((0, 0, 0), (0, 1, 0), angle)
        )
        wheel = wheel.union(tooth)
    return wheel


def _cam_lever() -> cq.Workplane:
    hub = _y_cylinder_at(0.00122, 0.00355, (0.0, 0.0, 0.0))
    arm = (
        cq.Workplane("XY")
        .box(0.0019, 0.0020, 0.0115)
        .translate((0.0008, 0.0, 0.0050))
    )
    nose = cq.Workplane("XY").sphere(0.00125).translate((0.0014, 0.0, 0.0110))
    return hub.union(arm).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_vintage_lighter")

    brass = model.material("aged_brass", rgba=(0.78, 0.58, 0.24, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.73, 0.74, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    shadow = model.material("shadow_line", rgba=(0.025, 0.022, 0.018, 1.0))
    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        mesh_from_cadquery(_hollow_lower_shell(), "lower_case_shell", tolerance=0.00035),
        material=brass,
        name="lower_case_shell",
    )
    lower_shell.visual(
        mesh_from_cadquery(_cam_pivot_support(), "cam_pivot_support", tolerance=0.00018),
        material=dark_steel,
        name="cam_pivot_support",
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_chimney_insert(), "chimney_insert", tolerance=0.00025),
        material=bright_steel,
        name="chimney_insert",
    )
    insert.visual(
        mesh_from_cadquery(_seam_shadow(), "nesting_seam", tolerance=0.00025),
        material=shadow,
        name="nesting_seam",
    )
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_hollow_lid(), "fitted_lid", tolerance=0.00035),
        material=brass,
        name="lid_shell",
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        mesh_from_cadquery(_striker_wheel(), "knurled_striker_wheel", tolerance=0.00018),
        material=dark_steel,
        name="knurled_wheel",
    )
    striker_wheel.visual(
        Cylinder(radius=0.00055, length=0.0068),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="wheel_axle_stub",
    )

    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        mesh_from_cadquery(_cam_lever(), "cam_lever", tolerance=0.00018),
        material=dark_steel,
        name="cam_lever_body",
    )

    model.articulation(
        "insert_mount",
        ArticulationType.FIXED,
        parent=lower_shell,
        child=insert,
        origin=Origin(),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=5.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "striker_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=(0.0132, 0.0, LOWER_H + 0.0080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=40.0),
    )
    model.articulation(
        "cam_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=cam_lever,
        origin=Origin(xyz=(-0.01555, 0.0, LOWER_H + 0.0040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0, lower=0.0, upper=0.95),
        mimic=Mimic(joint="lid_hinge", multiplier=0.42, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    striker_wheel = object_model.get_part("striker_wheel")
    cam_lever = object_model.get_part("cam_lever")
    lid_hinge = object_model.get_articulation("lid_hinge")
    striker_spin = object_model.get_articulation("striker_spin")

    ctx.check(
        "handheld lighter scale",
        0.034 <= CASE_W <= 0.045 and 0.052 <= LOWER_H + LID_H <= 0.065 and CASE_D < 0.020,
        details=f"case=({CASE_W}, {CASE_D}, {LOWER_H + LID_H})",
    )
    ctx.check(
        "striker wheel uses continuous axle joint",
        getattr(striker_spin, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"type={getattr(striker_spin, 'articulation_type', None)}",
    )
    ctx.allow_overlap(
        insert,
        lower_shell,
        elem_a="chimney_insert",
        elem_b="lower_case_shell",
        reason="The metal insert is intentionally nested inside the hollow lower case shell.",
    )
    ctx.allow_overlap(
        insert,
        lid,
        elem_a="chimney_insert",
        elem_b="lid_shell",
        reason="In the closed pose the hollow lid surrounds the chimney insert.",
    )
    ctx.expect_within(
        insert,
        lower_shell,
        axes="xy",
        margin=0.001,
        name="insert nests within lower case footprint",
    )
    ctx.expect_overlap(
        insert,
        lower_shell,
        axes="z",
        min_overlap=0.020,
        elem_a="chimney_insert",
        elem_b="lower_case_shell",
        name="insert remains deeply seated in lower case",
    )
    ctx.expect_within(
        insert,
        lid,
        axes="xy",
        margin=0.006,
        elem_a="chimney_insert",
        elem_b="lid_shell",
        name="closed lid surrounds chimney in plan",
    )
    ctx.expect_overlap(
        lid,
        insert,
        axes="z",
        min_overlap=0.010,
        elem_a="lid_shell",
        elem_b="chimney_insert",
        name="closed lid covers chimney height",
    )
    ctx.expect_gap(
        lid,
        lower_shell,
        axis="z",
        min_gap=0.0002,
        max_gap=0.0012,
        positive_elem="lid_shell",
        negative_elem="lower_case_shell",
        name="closed lid has a fine case seam",
    )
    ctx.expect_overlap(
        lid,
        lower_shell,
        axes="xy",
        min_overlap=0.010,
        elem_a="lid_shell",
        elem_b="lower_case_shell",
        name="fitted lid covers lower shell footprint",
    )
    ctx.expect_contact(
        insert,
        lower_shell,
        contact_tol=0.0010,
        elem_a="chimney_insert",
        elem_b="lower_case_shell",
        name="insert is seated in lower shell",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    closed_cam_aabb = ctx.part_world_aabb(cam_lever)
    with ctx.pose({lid_hinge: 1.35}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        open_cam_aabb = ctx.part_world_aabb(cam_lever)

    def _center_y(aabb):
        return None if aabb is None else 0.5 * (aabb[0][1] + aabb[1][1])

    def _center_x(aabb):
        return None if aabb is None else 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "lid swings outward on side hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and _center_y(open_lid_aabb) > _center_y(closed_lid_aabb) + 0.010,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "cam lever follows lid opening",
        closed_cam_aabb is not None
        and open_cam_aabb is not None
        and abs(_center_x(open_cam_aabb) - _center_x(closed_cam_aabb)) > 0.001,
        details=f"closed={closed_cam_aabb}, open={open_cam_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
