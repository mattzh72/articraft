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


WIDTH = 1.20
DEPTH = 0.55
HEIGHT = 0.72
SIDE_THICKNESS = 0.04
FRONT_Y = -DEPTH / 2.0
HINGE_Y = FRONT_Y - 0.035
DOOR_BOTTOM_Z = 0.10
DOOR_HEIGHT = 0.50
DOOR_THICKNESS = 0.026
DOOR_INNER_X = 0.54
DOOR_HINGE_CLEARANCE = 0.035


def _stepped_door_shape(sign: float) -> cq.Workplane:
    """One offset leaf: full top/bottom rails with a larger central sink cutout."""
    x0 = sign * DOOR_HINGE_CLEARANCE
    x_full = sign * DOOR_INNER_X
    x_cut = sign * 0.43
    z0 = 0.0
    z1 = 0.13
    z2 = 0.37
    z3 = DOOR_HEIGHT

    if sign > 0.0:
        pts = [
            (x0, z0),
            (x_full, z0),
            (x_full, z1),
            (x_cut, z1),
            (x_cut, z2),
            (x_full, z2),
            (x_full, z3),
            (x0, z3),
        ]
    else:
        pts = [
            (x0, z0),
            (x_full, z0),
            (x_full, z1),
            (x_cut, z1),
            (x_cut, z2),
            (x_full, z2),
            (x_full, z3),
            (x0, z3),
        ]

    # Author the front outline in CadQuery XY, extrude through local +Z, then
    # rotate so the extrusion becomes cabinet depth (-Y) and sketch-Y becomes Z.
    return (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(DOOR_THICKNESS)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _door_visuals(door, sign: float, panel_material: Material, trim_material: Material, metal: Material) -> None:
    door.visual(
        mesh_from_cadquery(_stepped_door_shape(sign), f"door_{0 if sign > 0 else 1}_panel"),
        material=panel_material,
        name="door_panel",
    )

    # Raised rails and stiles make the offset leaves read as cabinet doors while
    # staying mounted on the solid stepped slab.
    front_y = -DOOR_THICKNESS - 0.004
    trim_depth = 0.008
    rail_h = 0.030
    stile_w = 0.030
    mid_low = 0.13
    mid_high = 0.37

    def add_box(name: str, x_center: float, z_center: float, sx: float, sz: float) -> None:
        door.visual(
            Box((sx, trim_depth, sz)),
            origin=Origin(xyz=(x_center, front_y, z_center)),
            material=trim_material,
            name=name,
        )

    x_outer = sign * (DOOR_HINGE_CLEARANCE + stile_w / 2.0)
    x_full_center = sign * ((DOOR_HINGE_CLEARANCE + DOOR_INNER_X) / 2.0)
    full_span = DOOR_INNER_X - DOOR_HINGE_CLEARANCE
    x_cut_edge = sign * (0.43 - stile_w / 2.0)
    x_full_edge = sign * (DOOR_INNER_X - stile_w / 2.0)

    add_box("outer_stile", x_outer, DOOR_HEIGHT / 2.0, stile_w, DOOR_HEIGHT)
    add_box("top_rail", x_full_center, DOOR_HEIGHT - rail_h / 2.0, full_span, rail_h)
    add_box("bottom_rail", x_full_center, rail_h / 2.0, full_span, rail_h)
    add_box("cutout_stile", x_cut_edge, (mid_low + mid_high) / 2.0, stile_w, mid_high - mid_low)
    add_box("inner_top_stile", x_full_edge, (mid_high + DOOR_HEIGHT) / 2.0, stile_w, DOOR_HEIGHT - mid_high)
    add_box("inner_bottom_stile", x_full_edge, mid_low / 2.0, stile_w, mid_low)

    # The moving knuckle and leaf strap are centered on the joint axis, so the
    # visible leaf is mechanically clipped to the side hinge rather than merely
    # floating in front of the carcass.
    door.visual(
        Cylinder(radius=0.014, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=metal,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.070, 0.010, 0.22)),
        origin=Origin(xyz=(sign * 0.035, -0.010, 0.25)),
        material=metal,
        name="hinge_leaf",
    )

    knob_x = sign * 0.38
    door.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(knob_x, -DOOR_THICKNESS - 0.017, 0.265), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pull_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_sink_cabinet")

    painted = model.material("warm_painted_wood", color=(0.82, 0.78, 0.69, 1.0))
    door_paint = model.material("soft_sage_door", color=(0.55, 0.67, 0.62, 1.0))
    raised_trim = model.material("raised_sage_trim", color=(0.48, 0.60, 0.56, 1.0))
    dark_interior = model.material("shadowed_interior", color=(0.05, 0.045, 0.04, 1.0))
    metal = model.material("brushed_steel", color=(0.70, 0.70, 0.66, 1.0))

    carcass = model.part("carcass")
    # Low rectangular cabinet box: side panels, bottom, back, and a short false
    # rail above the offset under-sink doors.
    carcass.visual(
        Box((SIDE_THICKNESS, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + SIDE_THICKNESS / 2.0, 0.0, HEIGHT / 2.0)),
        material=painted,
        name="side_panel_0",
    )
    carcass.visual(
        Box((SIDE_THICKNESS, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - SIDE_THICKNESS / 2.0, 0.0, HEIGHT / 2.0)),
        material=painted,
        name="side_panel_1",
    )
    carcass.visual(
        Box((WIDTH, DEPTH, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=painted,
        name="bottom_panel",
    )
    carcass.visual(
        Box((WIDTH, 0.030, 0.62)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0 - 0.015, 0.345)),
        material=dark_interior,
        name="back_panel",
    )
    carcass.visual(
        Box((WIDTH - 0.06, 0.045, 0.085)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.020, 0.655)),
        material=painted,
        name="false_rail",
    )
    carcass.visual(
        Box((WIDTH - 0.04, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.014, 0.065)),
        material=painted,
        name="front_sill",
    )

    # Alternating fixed hinge knuckles on each outer side.  They share the
    # revolute axis with the moving barrel but occupy different Z bands, so the
    # door is visually captured without an interpenetrating solid pin.
    left_hinge_x = -WIDTH / 2.0
    carcass.visual(
        Cylinder(radius=0.014, length=0.12),
        origin=Origin(xyz=(left_hinge_x, HINGE_Y, DOOR_BOTTOM_Z + 0.08)),
        material=metal,
        name="hinge_0_lower",
    )
    carcass.visual(
        Cylinder(radius=0.014, length=0.12),
        origin=Origin(xyz=(left_hinge_x, HINGE_Y, DOOR_BOTTOM_Z + 0.42)),
        material=metal,
        name="hinge_0_upper",
    )
    carcass.visual(
        Box((0.060, 0.032, 0.10)),
        origin=Origin(xyz=(left_hinge_x + 0.030, FRONT_Y - 0.012, DOOR_BOTTOM_Z + 0.08)),
        material=metal,
        name="hinge_0_lower_leaf",
    )
    carcass.visual(
        Box((0.060, 0.032, 0.10)),
        origin=Origin(xyz=(left_hinge_x + 0.030, FRONT_Y - 0.012, DOOR_BOTTOM_Z + 0.42)),
        material=metal,
        name="hinge_0_upper_leaf",
    )

    right_hinge_x = WIDTH / 2.0
    carcass.visual(
        Cylinder(radius=0.014, length=0.12),
        origin=Origin(xyz=(right_hinge_x, HINGE_Y, DOOR_BOTTOM_Z + 0.08)),
        material=metal,
        name="hinge_1_lower",
    )
    carcass.visual(
        Cylinder(radius=0.014, length=0.12),
        origin=Origin(xyz=(right_hinge_x, HINGE_Y, DOOR_BOTTOM_Z + 0.42)),
        material=metal,
        name="hinge_1_upper",
    )
    carcass.visual(
        Box((0.060, 0.032, 0.10)),
        origin=Origin(xyz=(right_hinge_x - 0.030, FRONT_Y - 0.012, DOOR_BOTTOM_Z + 0.08)),
        material=metal,
        name="hinge_1_lower_leaf",
    )
    carcass.visual(
        Box((0.060, 0.032, 0.10)),
        origin=Origin(xyz=(right_hinge_x - 0.030, FRONT_Y - 0.012, DOOR_BOTTOM_Z + 0.42)),
        material=metal,
        name="hinge_1_upper_leaf",
    )

    door_0 = model.part("door_0")
    _door_visuals(door_0, 1.0, door_paint, raised_trim, metal)

    door_1 = model.part("door_1")
    _door_visuals(door_1, -1.0, door_paint, raised_trim, metal)

    model.articulation(
        "side_hinge_0",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door_0,
        origin=Origin(xyz=(-WIDTH / 2.0, HINGE_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "side_hinge_1",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door_1,
        origin=Origin(xyz=(WIDTH / 2.0, HINGE_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    hinge_0 = object_model.get_articulation("side_hinge_0")
    hinge_1 = object_model.get_articulation("side_hinge_1")

    # The two offset leaves deliberately leave a larger central under-sink
    # opening instead of meeting as a plain pair of rectangular doors.
    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        positive_elem="door_panel",
        negative_elem="door_panel",
        min_gap=0.10,
        max_gap=0.14,
        name="offset leaves leave center cutout",
    )
    for door in (door_0, door_1):
        ctx.expect_gap(
            carcass,
            door,
            axis="z",
            positive_elem="false_rail",
            negative_elem="door_panel",
            min_gap=0.005,
            max_gap=0.025,
            name=f"false rail spans above {door.name}",
        )

    # Alternating hinge knuckles should touch along Z so each moving leaf is
    # captured on the carcass rather than floating in front of it.
    ctx.expect_gap(
        door_0,
        carcass,
        axis="z",
        positive_elem="hinge_barrel",
        negative_elem="hinge_0_lower",
        max_gap=0.001,
        max_penetration=0.0,
        name="door_0 captured by lower side hinge",
    )
    ctx.expect_gap(
        carcass,
        door_0,
        axis="z",
        positive_elem="hinge_0_upper",
        negative_elem="hinge_barrel",
        max_gap=0.001,
        max_penetration=0.0,
        name="door_0 captured by upper side hinge",
    )
    ctx.expect_gap(
        door_1,
        carcass,
        axis="z",
        positive_elem="hinge_barrel",
        negative_elem="hinge_1_lower",
        max_gap=0.001,
        max_penetration=0.0,
        name="door_1 captured by lower side hinge",
    )
    ctx.expect_gap(
        carcass,
        door_1,
        axis="z",
        positive_elem="hinge_1_upper",
        negative_elem="hinge_barrel",
        max_gap=0.001,
        max_penetration=0.0,
        name="door_1 captured by upper side hinge",
    )

    origin_0 = ctx.part_world_position(door_0)
    origin_1 = ctx.part_world_position(door_1)
    ctx.check(
        "hinge axes are on outer side panels",
        origin_0 is not None
        and origin_1 is not None
        and abs(origin_0[0] + WIDTH / 2.0) < 1e-6
        and abs(origin_1[0] - WIDTH / 2.0) < 1e-6,
        details=f"door_0_origin={origin_0}, door_1_origin={origin_1}",
    )

    rest_0 = ctx.part_element_world_aabb(door_0, elem="door_panel")
    rest_1 = ctx.part_element_world_aabb(door_1, elem="door_panel")
    with ctx.pose({hinge_0: 1.2, hinge_1: 1.2}):
        open_0 = ctx.part_element_world_aabb(door_0, elem="door_panel")
        open_1 = ctx.part_element_world_aabb(door_1, elem="door_panel")
    ctx.check(
        "both doors swing outward from front",
        rest_0 is not None
        and rest_1 is not None
        and open_0 is not None
        and open_1 is not None
        and open_0[0][1] < rest_0[0][1] - 0.15
        and open_1[0][1] < rest_1[0][1] - 0.15,
        details=f"rest_0={rest_0}, open_0={open_0}, rest_1={rest_1}, open_1={open_1}",
    )

    return ctx.report()


object_model = build_object_model()
