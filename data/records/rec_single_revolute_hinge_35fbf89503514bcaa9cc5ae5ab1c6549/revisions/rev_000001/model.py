from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LEAF_WIDTH = 0.042
PLATE_THICKNESS = 0.004
BACK_LEAF_LENGTH = 0.055
STRAP_LENGTH = 0.155

PIN_RADIUS = 0.0022
BORE_RADIUS = 0.0027
BARREL_RADIUS = 0.0058
PLATE_ROOT_X = 0.0048

SEGMENT_LENGTH = 0.0068
SEGMENT_GAP = 0.0015
SEGMENT_MARGIN = (LEAF_WIDTH - 5.0 * SEGMENT_LENGTH - 4.0 * SEGMENT_GAP) / 2.0

PIN_HEAD_RADIUS = 0.0045
PIN_HEAD_THICKNESS = 0.0015
PIN_HEAD_OVERLAP = 0.0005
PIN_LENGTH = LEAF_WIDTH + 0.0020

SCREW_HOLE_RADIUS = 0.0022


def _knuckle_ranges() -> list[tuple[float, float]]:
    start = -LEAF_WIDTH / 2.0 + SEGMENT_MARGIN
    return [
        (
            start + i * (SEGMENT_LENGTH + SEGMENT_GAP),
            start + i * (SEGMENT_LENGTH + SEGMENT_GAP) + SEGMENT_LENGTH,
        )
        for i in range(5)
    ]


def _barrel_segment(y0: float, y1: float, *, bore_radius: float) -> cq.Workplane:
    outer = cq.Solid.makeCylinder(
        BARREL_RADIUS,
        y1 - y0,
        cq.Vector(0.0, y0, 0.0),
        cq.Vector(0.0, 1.0, 0.0),
    )
    bore = cq.Solid.makeCylinder(
        bore_radius,
        y1 - y0 + 0.001,
        cq.Vector(0.0, y0 - 0.0005, 0.0),
        cq.Vector(0.0, 1.0, 0.0),
    )
    return cq.Workplane(obj=outer.cut(bore))


def _back_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BACK_LEAF_LENGTH, LEAF_WIDTH, PLATE_THICKNESS)
        .translate((-(BACK_LEAF_LENGTH / 2.0 + PLATE_ROOT_X), 0.0, 0.0))
    )
    plate = plate.edges("|Z").fillet(0.0028)

    hole_cuts = (
        cq.Workplane("XY")
        .pushPoints([(-0.018, 0.0), (-0.040, 0.0)])
        .circle(SCREW_HOLE_RADIUS)
        .extrude(PLATE_THICKNESS * 3.0, both=True)
    )
    return plate.cut(hole_cuts)


def _strap_plate_shape() -> cq.Workplane:
    tip_x = PLATE_ROOT_X + STRAP_LENGTH
    arc_start_x = tip_x - LEAF_WIDTH / 2.0
    profile = (
        cq.Workplane("XY")
        .moveTo(PLATE_ROOT_X, -LEAF_WIDTH / 2.0)
        .lineTo(arc_start_x, -LEAF_WIDTH / 2.0)
        .threePointArc((tip_x, 0.0), (arc_start_x, LEAF_WIDTH / 2.0))
        .lineTo(PLATE_ROOT_X, LEAF_WIDTH / 2.0)
        .close()
    )
    plate = profile.extrude(PLATE_THICKNESS / 2.0, both=True)
    plate = plate.edges("|Z").fillet(0.0020)

    hole_cuts = (
        cq.Workplane("XY")
        .pushPoints([(0.028, 0.0), (0.068, 0.0), (0.110, 0.0)])
        .circle(SCREW_HOLE_RADIUS)
        .extrude(PLATE_THICKNESS * 3.0, both=True)
    )
    return plate.cut(hole_cuts)


def _back_knuckles_shape() -> cq.Workplane:
    ranges = _knuckle_ranges()
    shape: cq.Workplane | None = None
    for idx in (0, 2, 4):
        y0, y1 = ranges[idx]
        segment = _barrel_segment(y0, y1, bore_radius=BORE_RADIUS)
        shape = segment if shape is None else shape.union(segment)
    assert shape is not None
    return shape


def _strap_knuckles_shape() -> cq.Workplane:
    ranges = _knuckle_ranges()
    shape: cq.Workplane | None = None
    for idx in (1, 3):
        y0, y1 = ranges[idx]
        segment = _barrel_segment(y0, y1, bore_radius=BORE_RADIUS)
        shape = segment if shape is None else shape.union(segment)
    assert shape is not None
    return shape


def _hinge_pin_shape() -> cq.Workplane:
    shaft = cq.Solid.makeCylinder(
        PIN_RADIUS,
        PIN_LENGTH,
        cq.Vector(0.0, -PIN_LENGTH / 2.0, 0.0),
        cq.Vector(0.0, 1.0, 0.0),
    )

    first_start, _ = _knuckle_ranges()[0]
    _, last_end = _knuckle_ranges()[-1]
    left_head = cq.Solid.makeCylinder(
        PIN_HEAD_RADIUS,
        PIN_HEAD_THICKNESS,
        cq.Vector(0.0, first_start - (PIN_HEAD_THICKNESS - PIN_HEAD_OVERLAP), 0.0),
        cq.Vector(0.0, 1.0, 0.0),
    )
    right_head = cq.Solid.makeCylinder(
        PIN_HEAD_RADIUS,
        PIN_HEAD_THICKNESS,
        cq.Vector(0.0, last_end - PIN_HEAD_OVERLAP, 0.0),
        cq.Vector(0.0, 1.0, 0.0),
    )

    return cq.Workplane(obj=shaft.fuse(left_head).fuse(right_head))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="strap_and_leaf_hatch_hinge")

    blackened_steel = model.material("blackened_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    zinc_pin = model.material("zinc_pin", rgba=(0.74, 0.75, 0.78, 1.0))

    back_leaf = model.part("back_leaf")
    back_leaf.visual(
        mesh_from_cadquery(_back_plate_shape(), "back_leaf_plate"),
        material=blackened_steel,
        name="back_plate",
    )
    back_leaf.visual(
        mesh_from_cadquery(_back_knuckles_shape(), "back_leaf_knuckles"),
        material=blackened_steel,
        name="back_knuckles",
    )
    back_leaf.visual(
        mesh_from_cadquery(_hinge_pin_shape(), "hinge_pin"),
        material=zinc_pin,
        name="hinge_pin",
    )
    back_leaf.inertial = Inertial.from_geometry(
        Box((BACK_LEAF_LENGTH + 0.012, LEAF_WIDTH, 0.012)),
        mass=0.22,
        origin=Origin(xyz=(-0.028, 0.0, 0.0)),
    )

    strap_leaf = model.part("strap_leaf")
    strap_leaf.visual(
        mesh_from_cadquery(_strap_plate_shape(), "strap_leaf_plate"),
        material=blackened_steel,
        name="strap_plate",
    )
    strap_leaf.visual(
        mesh_from_cadquery(_strap_knuckles_shape(), "strap_leaf_knuckles"),
        material=blackened_steel,
        name="strap_knuckles",
    )
    strap_leaf.inertial = Inertial.from_geometry(
        Box((STRAP_LENGTH + 0.012, LEAF_WIDTH, 0.012)),
        mass=0.31,
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
    )

    model.articulation(
        "back_to_strap",
        ArticulationType.REVOLUTE,
        parent=back_leaf,
        child=strap_leaf,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.2, effort=6.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_leaf = object_model.get_part("back_leaf")
    strap_leaf = object_model.get_part("strap_leaf")
    hinge = object_model.get_articulation("back_to_strap")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            strap_leaf,
            back_leaf,
            axes="yz",
            elem_a="strap_knuckles",
            elem_b="hinge_pin",
            min_overlap=0.004,
            name="strap knuckles align with the hinge pin at rest",
        )
        ctx.expect_within(
            strap_leaf,
            back_leaf,
            axes="y",
            inner_elem="strap_knuckles",
            outer_elem="hinge_pin",
            margin=0.0015,
            name="hinge pin spans the full moving knuckle stack",
        )

    def visual_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    closed_center = None
    opened_center = None
    with ctx.pose({hinge: 0.0}):
        closed_center = visual_center("strap_leaf", "strap_plate")
    with ctx.pose({hinge: 1.2}):
        opened_center = visual_center("strap_leaf", "strap_plate")

    ctx.check(
        "strap leaf opens upward about the pin",
        closed_center is not None
        and opened_center is not None
        and opened_center[2] > closed_center[2] + 0.05
        and opened_center[0] < closed_center[0] - 0.02,
        details=f"closed_center={closed_center}, opened_center={opened_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
