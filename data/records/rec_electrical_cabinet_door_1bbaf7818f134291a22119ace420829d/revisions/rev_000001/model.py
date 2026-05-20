from __future__ import annotations

from math import pi

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


def _rounded_rect_prism(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """A robust cast-looking rounded rectangular prism, standing on z=0."""
    solid = cq.Workplane("XY").rect(width, depth).extrude(height)
    if radius > 0:
        solid = solid.edges("|Z").fillet(radius)
    return solid


def _body_shell() -> cq.Workplane:
    """Heavy open cast enclosure with a raised flame-path cover land."""
    outer_w, outer_d, outer_h = 0.36, 0.54, 0.18
    wall = 0.032
    flange_w, flange_d, flange_h = 0.45, 0.63, 0.025

    body = _rounded_rect_prism(outer_w, outer_d, outer_h, 0.032)
    cavity = (
        cq.Workplane("XY")
        .rect(outer_w - 2.0 * wall, outer_d - 2.0 * wall)
        .extrude(outer_h + 0.04)
        .translate((0.0, 0.0, wall))
    )
    body = body.cut(cavity)

    flange = _rounded_rect_prism(flange_w, flange_d, flange_h, 0.026).translate(
        (0.0, 0.0, outer_h)
    )
    flange_opening = (
        cq.Workplane("XY")
        .rect(outer_w - 2.0 * wall, outer_d - 2.0 * wall)
        .extrude(flange_h + 0.03)
        .translate((0.0, 0.0, outer_h - 0.005))
    )
    body = body.union(flange.cut(flange_opening))

    return body


def _gasket_ring() -> cq.Workplane:
    """Black raised gasket seated on the body land."""
    outer = cq.Workplane("XY").rect(0.405, 0.585).extrude(0.005)
    inner = cq.Workplane("XY").rect(0.292, 0.472).extrude(0.008).translate((0, 0, -0.001))
    return outer.cut(inner).translate((0.0, 0.0, 0.205))


def _cover_plate() -> cq.Workplane:
    """Thick cast cover plate in the cover joint frame; hinge pin is local y axis at x=0,z=0."""
    plate = _rounded_rect_prism(0.465, 0.64, 0.035, 0.018).translate((0.2725, 0.0, -0.025))

    raised_land_outer = cq.Workplane("XY").rect(0.370, 0.535).extrude(0.010)
    raised_land_inner = (
        cq.Workplane("XY").rect(0.275, 0.440).extrude(0.014).translate((0.0, 0.0, -0.002))
    )
    raised_land = raised_land_outer.cut(raised_land_inner).translate((0.275, 0.0, 0.010))
    plate = plate.union(raised_land)

    # Four cast bolt pads on the cover corners.
    for x in (0.105, 0.445):
        for y in (-0.245, 0.245):
            pad = cq.Workplane("XY").circle(0.035).extrude(0.011).translate((x, y, 0.010))
            recess = cq.Workplane("XY").circle(0.015).extrude(0.014).translate((x, y, 0.014))
            plate = plate.union(pad.cut(recess))

    # Through boss for the captive bolt ring, left as a real clearance hole.
    boss_outer = cq.Workplane("XY").circle(0.046).extrude(0.018).translate((0.470, 0.0, 0.010))
    boss_hole = cq.Workplane("XY").circle(0.018).extrude(0.080).translate((0.470, 0.0, -0.035))
    plate = plate.union(boss_outer).cut(boss_hole)
    return plate


def _bolt_ring() -> cq.Workplane:
    """Rotating captive bolt ring/handwheel around the bolt shank."""
    outer = cq.Workplane("XY").circle(0.062).circle(0.039).extrude(0.012).translate((0, 0, 0.032))
    hub = cq.Workplane("XY").circle(0.018).extrude(0.044)
    short_shank = cq.Workplane("XY").circle(0.013).extrude(0.010).translate((0, 0, -0.010))
    spoke_x = cq.Workplane("XY").box(0.096, 0.014, 0.010).translate((0, 0, 0.038))
    spoke_y = cq.Workplane("XY").box(0.014, 0.092, 0.010).translate((0, 0, 0.038))
    dog = cq.Workplane("XY").box(0.020, 0.034, 0.010).translate((0.0, 0.073, 0.038))
    return outer.union(hub).union(short_shank).union(spoke_x).union(spoke_y).union(dog)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flameproof_junction_box")

    cast_iron = model.material("dark_cast_iron", rgba=(0.22, 0.24, 0.24, 1.0))
    worn_edges = model.material("worn_machined_edges", rgba=(0.46, 0.48, 0.47, 1.0))
    black_rubber = model.material("black_neoprene_gasket", rgba=(0.01, 0.012, 0.01, 1.0))
    parkerized = model.material("parkerized_bolt_ring", rgba=(0.06, 0.065, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "cast_body", tolerance=0.0015),
        material=cast_iron,
        name="cast_body",
    )
    body.visual(
        mesh_from_cadquery(_gasket_ring(), "gasket", tolerance=0.001),
        material=black_rubber,
        name="gasket",
    )
    # Cast conduit entries on the short sides with dark recessed openings.
    for index, y in enumerate((-0.295, 0.295)):
        body.visual(
            Cylinder(radius=0.055, length=0.050),
            origin=Origin(xyz=(0.0, y, 0.095), rpy=(pi / 2.0, 0.0, 0.0)),
            material=cast_iron,
            name=f"conduit_hub_{index}",
        )
        body.visual(
            Cylinder(radius=0.030, length=0.006),
            origin=Origin(
                xyz=(0.0, y + (0.028 if y > 0 else -0.028), 0.095),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=black_rubber,
            name=f"conduit_bore_{index}",
        )
    # Receiver boss under the cover hole, giving the captive bolt something to draw against.
    body.visual(
        Cylinder(radius=0.025, length=0.028),
        origin=Origin(xyz=(0.215, 0.0, 0.194)),
        material=worn_edges,
        name="bolt_socket",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=(0.215, 0.0, 0.209)),
        material=black_rubber,
        name="threaded_hole",
    )
    # Two fixed hinge knuckles and their cast ears on the long edge.
    for index, y in enumerate((-0.225, 0.225)):
        body.visual(
            Box((0.052, 0.120, 0.055)),
            origin=Origin(xyz=(-0.249, y, 0.210)),
            material=cast_iron,
            name=f"hinge_ear_{index}",
        )
        body.visual(
            Cylinder(radius=0.021, length=0.120),
            origin=Origin(xyz=(-0.255, y, 0.235), rpy=(pi / 2.0, 0.0, 0.0)),
            material=worn_edges,
            name=f"hinge_knuckle_{index}",
        )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_plate(), "cover_plate", tolerance=0.0012),
        material=cast_iron,
        name="cover_plate",
    )
    cover.visual(
        Box((0.070, 0.250, 0.035)),
        origin=Origin(xyz=(0.035, 0.0, -0.008)),
        material=cast_iron,
        name="hinge_tongue",
    )
    cover.visual(
        Cylinder(radius=0.020, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=worn_edges,
        name="hinge_knuckle",
    )

    bolt_ring = model.part("bolt_ring")
    bolt_ring.visual(
        mesh_from_cadquery(_bolt_ring(), "bolt_ring", tolerance=0.0008),
        material=parkerized,
        name="bolt_ring",
    )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        # The cover plate extends in local +X from this hinge line; -Y makes
        # positive motion lift the free edge upward.
        origin=Origin(xyz=(-0.255, 0.0, 0.235)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    model.articulation(
        "bolt_ring_turn",
        ArticulationType.REVOLUTE,
        parent=cover,
        child=bolt_ring,
        origin=Origin(xyz=(0.470, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0, lower=-1.75, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    bolt_ring = object_model.get_part("bolt_ring")
    cover_hinge = object_model.get_articulation("cover_hinge")
    ring_turn = object_model.get_articulation("bolt_ring_turn")

    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="gasket",
        max_gap=0.001,
        max_penetration=0.0005,
        name="cover seats on raised gasket",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_plate",
        elem_b="gasket",
        min_overlap=0.25,
        name="cover overlaps the flame-path land",
    )
    ctx.expect_within(
        bolt_ring,
        cover,
        axes="xy",
        inner_elem="bolt_ring",
        outer_elem="cover_plate",
        margin=0.02,
        name="captive bolt ring is mounted on cover boss",
    )

    rest_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
    with ctx.pose({cover_hinge: 1.1}):
        opened_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
    ctx.check(
        "cover hinge lifts the latch edge",
        rest_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > rest_aabb[1][2] + 0.18,
        details=f"rest={rest_aabb}, opened={opened_aabb}",
    )

    rest_ring = ctx.part_element_world_aabb(bolt_ring, elem="bolt_ring")
    with ctx.pose({ring_turn: 1.2}):
        turned_ring = ctx.part_element_world_aabb(bolt_ring, elem="bolt_ring")
    ctx.check(
        "bolt ring turns about its captive shank",
        rest_ring is not None
        and turned_ring is not None
        and abs((turned_ring[1][0] - turned_ring[0][0]) - (rest_ring[1][0] - rest_ring[0][0]))
        > 0.006,
        details=f"rest={rest_ring}, turned={turned_ring}",
    )

    return ctx.report()


object_model = build_object_model()
