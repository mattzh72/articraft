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


WIDTH = 0.82
DEPTH = 0.56
BODY_HEIGHT = 0.75
WALL = 0.045
FLOOR = 0.055

LID_OVERHANG = 0.018
LID_DEPTH = DEPTH + 2.0 * LID_OVERHANG
LID_WIDTH = WIDTH + 2.0 * LID_OVERHANG
LID_THICKNESS = 0.075
GASKET_THICKNESS = 0.012
HINGE_OFFSET = 0.020
HINGE_Y = DEPTH / 2.0 + LID_OVERHANG + HINGE_OFFSET
HINGE_Z = BODY_HEIGHT + GASKET_THICKNESS + LID_THICKNESS / 2.0

HANDLE_WIDTH = 0.34
HANDLE_DEPTH = 0.034
HANDLE_HEIGHT = 0.036

HINGE_RADIUS = 0.014
HINGE_SEGMENT = 0.038
HINGE_CENTER_SEGMENT = 0.044
HINGE_CLEARANCE = 0.004
HINGE_KNUCKLE_OFFSET = HINGE_CENTER_SEGMENT / 2.0 + HINGE_CLEARANCE + HINGE_SEGMENT / 2.0
HINGE_SPAN = 2.0 * HINGE_SEGMENT + HINGE_CENTER_SEGMENT + 2.0 * HINGE_CLEARANCE
HINGE_XS = (-0.25, 0.25)


def _body_shell() -> cq.Workplane:
    """One-piece open top cabinet tub with thick insulated walls."""
    outer = (
        cq.Workplane("XY")
        .box(WIDTH, DEPTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
    )
    # A slightly over-tall cutter leaves a real open cavity instead of a solid block.
    void = (
        cq.Workplane("XY")
        .box(WIDTH - 2.0 * WALL, DEPTH - 2.0 * WALL, BODY_HEIGHT)
        .translate((0.0, 0.0, FLOOR + BODY_HEIGHT / 2.0))
    )
    shell = outer.cut(void)
    try:
        shell = shell.edges("|Z").fillet(0.018).edges(">Z").fillet(0.006)
    except Exception:
        # Fillets are visual refinement; keep the valid hollow shell if CQ rejects
        # an edge selection on a particular kernel build.
        pass
    return shell


def _lid_shell() -> cq.Workplane:
    """Flat lid with a molded front scoop cut into the lower lip."""
    center_y = -HINGE_OFFSET - LID_DEPTH / 2.0
    lid = (
        cq.Workplane("XY")
        .box(LID_WIDTH, LID_DEPTH, LID_THICKNESS)
        .translate((0.0, center_y, 0.0))
    )
    recess = (
        cq.Workplane("XY")
        .box(HANDLE_WIDTH, HANDLE_DEPTH, HANDLE_HEIGHT)
        .translate(
            (
                0.0,
                -HINGE_OFFSET - LID_DEPTH + HANDLE_DEPTH / 2.0,
                -0.004,
            )
        )
    )
    lid = lid.cut(recess)
    try:
        lid = lid.edges("|Z").fillet(0.012).edges(">Z").fillet(0.006)
    except Exception:
        pass
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_chest_freezer")

    white = model.material("slightly_warm_white", rgba=(0.94, 0.95, 0.94, 1.0))
    lid_white = model.material("smooth_lid_white", rgba=(0.98, 0.985, 0.98, 1.0))
    liner = model.material("pale_liner", rgba=(0.86, 0.90, 0.92, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.03, 0.035, 0.04, 1.0))
    shadow = model.material("handle_shadow", rgba=(0.10, 0.11, 0.12, 1.0))
    metal = model.material("brushed_hinge_metal", rgba=(0.62, 0.64, 0.65, 1.0))
    foot = model.material("dark_leveling_feet", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "freezer_body_shell", tolerance=0.002),
        material=white,
        name="body_shell",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL - 0.010, DEPTH - 2.0 * WALL - 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR + 0.004)),
        material=liner,
        name="inner_floor_liner",
    )

    gasket_width = 0.018
    body.visual(
        Box((WIDTH - 0.018, gasket_width, GASKET_THICKNESS)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + WALL / 2.0, BODY_HEIGHT + GASKET_THICKNESS / 2.0)),
        material=rubber,
        name="gasket_front",
    )
    body.visual(
        Box((WIDTH - 0.018, gasket_width, GASKET_THICKNESS)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - WALL / 2.0, BODY_HEIGHT + GASKET_THICKNESS / 2.0)),
        material=rubber,
        name="gasket_rear",
    )
    body.visual(
        Box((gasket_width, DEPTH - 2.0 * WALL, GASKET_THICKNESS)),
        origin=Origin(xyz=(-WIDTH / 2.0 + WALL / 2.0, 0.0, BODY_HEIGHT + GASKET_THICKNESS / 2.0)),
        material=rubber,
        name="gasket_side_0",
    )
    body.visual(
        Box((gasket_width, DEPTH - 2.0 * WALL, GASKET_THICKNESS)),
        origin=Origin(xyz=(WIDTH / 2.0 - WALL / 2.0, 0.0, BODY_HEIGHT + GASKET_THICKNESS / 2.0)),
        material=rubber,
        name="gasket_side_1",
    )

    for i, x in enumerate((-WIDTH / 2.0 + 0.095, WIDTH / 2.0 - 0.095)):
        for j, y in enumerate((-DEPTH / 2.0 + 0.075, DEPTH / 2.0 - 0.075)):
            body.visual(
                Cylinder(radius=0.028, length=0.022),
                origin=Origin(xyz=(x, y, -0.011)),
                material=foot,
                name=f"foot_{i}_{j}",
            )

    for hinge_i, x0 in enumerate(HINGE_XS):
        body_knuckle_names = (
            ("body_hinge_knuckle_0_0", "body_hinge_knuckle_0_1"),
            ("body_hinge_knuckle_1_0", "body_hinge_knuckle_1_1"),
        )
        for side, dx in enumerate((-HINGE_KNUCKLE_OFFSET, HINGE_KNUCKLE_OFFSET)):
            body.visual(
                Cylinder(radius=HINGE_RADIUS, length=HINGE_SEGMENT),
                origin=Origin(
                    xyz=(x0 + dx, HINGE_Y, HINGE_Z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=metal,
                name=body_knuckle_names[hinge_i][side],
            )
            body.visual(
                Box((HINGE_SEGMENT, 0.010, 0.108)),
                origin=Origin(
                    xyz=(
                        x0 + dx,
                        HINGE_Y - HINGE_RADIUS - 0.001,
                        HINGE_Z - 0.056,
                    )
                ),
                material=metal,
                name=f"body_hinge_plate_{hinge_i}_{side}",
            )
            body.visual(
                Box((HINGE_SEGMENT, HINGE_Y - DEPTH / 2.0 - HINGE_RADIUS, 0.020)),
                origin=Origin(
                    xyz=(
                        x0 + dx,
                        (DEPTH / 2.0 + HINGE_Y - HINGE_RADIUS) / 2.0,
                        BODY_HEIGHT - 0.035,
                    )
                ),
                material=metal,
                name=f"body_hinge_standoff_{hinge_i}_{side}",
            )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "flat_lid_with_recessed_handle", tolerance=0.0015),
        material=lid_white,
        name="lid_shell",
    )
    lid.visual(
        Box((HANDLE_WIDTH - 0.030, 0.004, HANDLE_HEIGHT - 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                -HINGE_OFFSET - LID_DEPTH + HANDLE_DEPTH - 0.002,
                -0.004,
            )
        ),
        material=shadow,
        name="handle_pocket",
    )

    for hinge_i, x0 in enumerate(HINGE_XS):
        lid_knuckle_names = ("lid_hinge_knuckle_0", "lid_hinge_knuckle_1")
        lid.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_CENTER_SEGMENT),
            origin=Origin(
                xyz=(x0, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=metal,
            name=lid_knuckle_names[hinge_i],
        )
        lid.visual(
            Box((HINGE_CENTER_SEGMENT, 0.028, 0.008)),
            origin=Origin(xyz=(x0, -HINGE_OFFSET + 0.004, -HINGE_RADIUS - 0.002)),
            material=metal,
            name=f"lid_hinge_leaf_{hinge_i}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # The closed lid extends forward along local -Y from the rear hinge line.
        # Negating X makes positive joint motion lift the front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=45.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="gasket_front",
        max_gap=0.0015,
        max_penetration=0.0,
        name="closed lid sits on front gasket",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="body_shell",
        min_overlap=0.48,
        name="lid covers the freezer body footprint",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        elem_a="lid_hinge_knuckle_0",
        elem_b="body_hinge_knuckle_0_0",
        min_overlap=0.020,
        name="first hinge barrels align on common pin axis",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        positive_elem="lid_hinge_knuckle_0",
        negative_elem="body_hinge_knuckle_0_0",
        min_gap=0.002,
        max_gap=0.006,
        name="first hinge knuckles keep barrel clearance",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({hinge: 1.15}):
        opened_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="handle_pocket",
            negative_elem="gasket_front",
            min_gap=0.20,
            name="opened recessed handle clears the front gasket",
        )

    ctx.check(
        "lid opens upward from rear barrel hinges",
        closed_aabb is not None
        and opened_aabb is not None
        and float(opened_aabb[1][2] - closed_aabb[1][2]) > 0.18,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
