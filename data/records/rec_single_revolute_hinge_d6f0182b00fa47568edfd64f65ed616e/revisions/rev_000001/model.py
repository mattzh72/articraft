from __future__ import annotations

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


LEAF_THICKNESS = 0.006
LEAF_HEIGHT = 0.160
BARREL_HALF_LENGTH = 0.090
BARREL_OUTER_RADIUS = 0.016
BARREL_BORE_RADIUS = 0.0068
PIN_RADIUS = 0.0046
HINGE_EDGE_GAP = 0.023


def _tube_z(outer_radius: float, inner_radius: float, z_min: float, z_max: float) -> cq.Workplane:
    """Hollow round knuckle tube along the hinge pin axis."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Solid:
    """Cylindrical cutter or liner with its axis running through the leaf thickness."""
    return cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(center[0], center[1] - length / 2.0, center[2]),
        cq.Vector(0.0, 1.0, 0.0),
    )


def _round_hole(body: cq.Workplane, x: float, z: float, radius: float) -> cq.Workplane:
    return body.cut(_cylinder_y(radius, LEAF_THICKNESS * 4.0, (x, 0.0, z)))


def _slot_hole(body: cq.Workplane, x: float, z: float, length_z: float, radius: float) -> cq.Workplane:
    through = LEAF_THICKNESS * 4.0
    straight = max(length_z - 2.0 * radius, 0.001)
    cutter = (
        cq.Workplane("XY")
        .box(2.0 * radius, through, straight)
        .translate((x, 0.0, z))
        .union(_cylinder_y(radius, through, (x, 0.0, z - straight / 2.0)))
        .union(_cylinder_y(radius, through, (x, 0.0, z + straight / 2.0)))
    )
    return body.cut(cutter)


def _annular_disc_z(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    z_center: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, z_center - thickness / 2.0))
    )


def _annular_disc_y(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _leaf_plate(
    *,
    side: str,
    width: float,
    x_inner: float,
    hole_style: str,
) -> cq.Workplane:
    x_center = x_inner - width / 2.0 if side == "grounded" else x_inner + width / 2.0
    plate = (
        cq.Workplane("XY")
        .box(width, LEAF_THICKNESS, LEAF_HEIGHT)
        .translate((x_center, 0.0, 0.0))
        .edges()
        .chamfer(0.0012)
    )

    if hole_style == "round":
        for x in (x_inner - 0.030, x_inner - 0.070):
            for z in (-0.044, 0.044):
                plate = _round_hole(plate, x, z, 0.0054)
        # Small clearance holes closer to the barrel emphasize the grounded mounting.
        for z in (-0.066, 0.066):
            plate = _round_hole(plate, x_inner - 0.012, z, 0.0032)
    else:
        for z in (-0.050, 0.0, 0.050):
            plate = _slot_hole(plate, x_inner + 0.045, z, 0.027, 0.0042)
        # A short witness relief along the hinge-side land, different from the fixed leaf.
        for z in (-0.064, 0.064):
            plate = _slot_hole(plate, x_inner + 0.013, z, 0.018, 0.0026)

    return plate


def _grounded_body() -> cq.Workplane:
    x_inner = -HINGE_EDGE_GAP
    plate = _leaf_plate(side="grounded", width=0.105, x_inner=x_inner, hole_style="round")

    # A raised outer rail gives the grounded leaf a beefier machined datum edge.
    rail = (
        cq.Workplane("XY")
        .box(0.006, 0.0022, LEAF_HEIGHT - 0.018)
        .translate((x_inner - 0.105 + 0.004, LEAF_THICKNESS / 2.0 + 0.00085, 0.0))
    )
    return plate.union(rail)


def _grounded_knuckles() -> cq.Workplane:
    body = cq.Workplane("XY")
    parent_segments = [(-0.090, -0.056), (-0.022, 0.022), (0.056, 0.090)]
    for z_min, z_max in parent_segments:
        z_center = (z_min + z_max) / 2.0
        length = z_max - z_min
        strap = (
            cq.Workplane("XY")
            .box(0.016, 0.012, length)
            .translate((-0.016, 0.0, z_center))
        )
        body = body.union(_tube_z(BARREL_OUTER_RADIUS, BARREL_BORE_RADIUS, z_min, z_max)).union(strap)

    return body


def _carried_body() -> cq.Workplane:
    x_inner = HINGE_EDGE_GAP
    plate = _leaf_plate(side="carried", width=0.095, x_inner=x_inner, hole_style="slot")

    # A thin recessed-looking hinge-side land differentiates this moving leaf from the fixed rail.
    land = (
        cq.Workplane("XY")
        .box(0.005, 0.0018, LEAF_HEIGHT - 0.026)
        .translate((x_inner + 0.006, LEAF_THICKNESS / 2.0 + 0.00065, 0.0))
    )
    return plate.union(land)


def _carried_knuckles() -> cq.Workplane:
    body = cq.Workplane("XY")
    child_segments = [(-0.053, -0.025), (0.025, 0.053)]
    for z_min, z_max in child_segments:
        z_center = (z_min + z_max) / 2.0
        length = z_max - z_min
        strap = (
            cq.Workplane("XY")
            .box(0.016, 0.012, length)
            .translate((0.016, 0.0, z_center))
        )
        body = body.union(_tube_z(BARREL_OUTER_RADIUS, BARREL_BORE_RADIUS, z_min, z_max)).union(strap)

    return body


def _pin_core_and_caps() -> cq.Workplane:
    pin = cq.Workplane("XY").circle(PIN_RADIUS).extrude(0.196).translate((0.0, 0.0, -0.098))
    top_cap = _annular_disc_z(0.0125, 0.0, 0.006, 0.094)
    bottom_cap = _annular_disc_z(0.0125, 0.0, 0.006, -0.094)
    return pin.union(top_cap).union(bottom_cap)


def _shoulder_washers() -> cq.Workplane:
    body = cq.Workplane("XY")
    # Bronze shoulder washers occupy the visible axial clearances between knuckles.
    for z in (-0.0545, -0.0235, 0.0235, 0.0545):
        body = body.union(_annular_disc_z(0.0174, PIN_RADIUS * 0.92, 0.0030, z))
    return body


def _round_counterbore_rings() -> cq.Workplane:
    y = LEAF_THICKNESS / 2.0 + 0.00045
    rings = cq.Workplane("XY")
    for x in (-HINGE_EDGE_GAP - 0.030, -HINGE_EDGE_GAP - 0.070):
        for z in (-0.044, 0.044):
            ring = _annular_disc_y(0.0105, 0.0057, 0.0012, (x, y, z))
            rings = rings.union(ring)
    for z in (-0.066, 0.066):
        ring = _annular_disc_y(0.0064, 0.0035, 0.0011, (-HINGE_EDGE_GAP - 0.012, y, z))
        rings = rings.union(ring)
    return rings


def _slot_liners() -> cq.Workplane:
    y = LEAF_THICKNESS / 2.0 + 0.00045
    liners = cq.Workplane("XY")
    for z in (-0.050, 0.0, 0.050):
        # Three dark shouldered slotted bushings on the moving leaf.
        top = _annular_disc_y(0.0064, 0.00435, 0.0011, (HINGE_EDGE_GAP + 0.045, y, z + 0.0093))
        bottom = _annular_disc_y(0.0064, 0.00435, 0.0011, (HINGE_EDGE_GAP + 0.045, y, z - 0.0093))
        bar_a = (
            cq.Workplane("XY")
            .box(0.0019, 0.0011, 0.0186)
            .translate((HINGE_EDGE_GAP + 0.045 - 0.0052, y, z))
        )
        bar_b = (
            cq.Workplane("XY")
            .box(0.0019, 0.0011, 0.0186)
            .translate((HINGE_EDGE_GAP + 0.045 + 0.0052, y, z))
        )
        liners = liners.union(top).union(bottom).union(bar_a).union(bar_b)
    return liners


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_industrial_barrel_hinge")
    dark_steel = model.material("dark_phosphate_steel", rgba=(0.20, 0.22, 0.23, 1.0))
    satin_steel = model.material("satin_carried_steel", rgba=(0.48, 0.50, 0.48, 1.0))
    burnished_edges = model.material("dark_burnished_edges", rgba=(0.06, 0.065, 0.07, 1.0))
    bronze = model.material("oiled_bronze_washers", rgba=(0.75, 0.52, 0.24, 1.0))

    grounded_leaf = model.part("grounded_leaf")
    grounded_leaf.visual(
        mesh_from_cadquery(_grounded_body(), "grounded_leaf_plate", tolerance=0.00045),
        material=dark_steel,
        name="grounded_plate",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(_grounded_knuckles(), "grounded_leaf_knuckles", tolerance=0.00045),
        material=dark_steel,
        name="grounded_knuckles",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(_round_counterbore_rings(), "grounded_counterbore_rings", tolerance=0.00035),
        material=burnished_edges,
        name="counterbore_rings",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(_pin_core_and_caps(), "machined_pin_core", tolerance=0.00035),
        material=burnished_edges,
        name="pin_core",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(_shoulder_washers(), "bronze_shoulder_washers", tolerance=0.00035),
        material=bronze,
        name="shoulder_washers",
    )

    carried_leaf = model.part("carried_leaf")
    carried_leaf.visual(
        mesh_from_cadquery(_carried_body(), "carried_leaf_plate", tolerance=0.00045),
        material=satin_steel,
        name="carried_plate",
    )
    carried_leaf.visual(
        mesh_from_cadquery(_carried_knuckles(), "carried_leaf_knuckles", tolerance=0.00045),
        material=satin_steel,
        name="carried_knuckles",
    )
    carried_leaf.visual(
        mesh_from_cadquery(_slot_liners(), "carried_slot_liners", tolerance=0.00035),
        material=burnished_edges,
        name="slot_liners",
    )

    model.articulation(
        "pin_axis",
        ArticulationType.REVOLUTE,
        parent=grounded_leaf,
        child=carried_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0, lower=0.0, upper=1.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    grounded = object_model.get_part("grounded_leaf")
    carried = object_model.get_part("carried_leaf")
    joint = object_model.get_articulation("pin_axis")

    ctx.allow_overlap(
        carried,
        grounded,
        elem_a="carried_knuckles",
        elem_b="pin_core",
        reason=(
            "The moving barrel knuckles are represented as captured around the grounded hinge pin; "
            "the pin/bore support is intentionally nested on the revolute axis."
        ),
    )

    ctx.check(
        "single supported revolute stage",
        len(object_model.articulations) == 1 and joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({joint: 0.0}):
        ctx.expect_gap(
            carried,
            grounded,
            axis="x",
            positive_elem="carried_plate",
            negative_elem="grounded_plate",
            min_gap=0.030,
            name="opposed leaves clear the hinge-line hardware when flat",
        )
        ctx.expect_overlap(
            carried,
            grounded,
            axes="xy",
            elem_a="carried_knuckles",
            elem_b="grounded_knuckles",
            min_overlap=0.010,
            name="interleaved knuckles share the pin-axis footprint",
        )
        ctx.expect_within(
            grounded,
            carried,
            axes="xy",
            inner_elem="pin_core",
            outer_elem="carried_knuckles",
            margin=0.001,
            name="grounded pin sits inside the moving barrel footprint",
        )
        ctx.expect_overlap(
            grounded,
            carried,
            axes="z",
            elem_a="pin_core",
            elem_b="carried_knuckles",
            min_overlap=0.025,
            name="pin spans the carried knuckle bearing length",
        )

    rest_aabb = ctx.part_element_world_aabb(carried, elem="carried_plate")
    with ctx.pose({joint: 1.80}):
        open_aabb = ctx.part_element_world_aabb(carried, elem="carried_plate")
        ctx.expect_gap(
            carried,
            grounded,
            axis="y",
            positive_elem="carried_plate",
            negative_elem="grounded_plate",
            min_gap=0.010,
            name="opened carried leaf swings clear of grounded leaf",
        )

    if rest_aabb is not None and open_aabb is not None:
        rest_y_center = (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
        open_y_center = (open_aabb[0][1] + open_aabb[1][1]) / 2.0
        ctx.check(
            "positive rotation carries the moving leaf outward",
            open_y_center > rest_y_center + 0.040,
            details=f"rest_y={rest_y_center:.4f}, open_y={open_y_center:.4f}",
        )
    else:
        ctx.fail("moving leaf aabb available", "could not measure carried leaf body")

    return ctx.report()


object_model = build_object_model()
