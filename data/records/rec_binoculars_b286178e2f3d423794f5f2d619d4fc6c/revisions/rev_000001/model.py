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


OPTICAL_AXIS = (1.0, 0.0, 0.0)
TUBE_OFFSET_Y = 0.047
RING_X = -0.102
CYLINDER_X_RPY = (0.0, math.pi / 2.0, 0.0)


def _cylinder_x(length: float, radius: float, *, x0: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    """CadQuery cylinder with its axis along +X."""
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, y, z))


def _tube_x(
    length: float,
    outer_radius: float,
    inner_radius: float,
    *,
    x0: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    """Open cylindrical shell with its axis along +X."""
    outer = _cylinder_x(length, outer_radius, x0=x0, y=y, z=z)
    cutter = _cylinder_x(length + 0.006, inner_radius, x0=x0 - 0.003, y=y, z=z)
    return outer.cut(cutter)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _add_cylinder_x(part, *, name: str, radius: float, length: float, center: tuple[float, float, float], material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=CYLINDER_X_RPY),
        material=material,
        name=name,
    )


def _add_primitive_body(part, *, side: int, material) -> None:
    """Rubber-armored binocular half using robust primitive barrels and bridge blocks."""
    y = side * TUBE_OFFSET_Y
    _add_cylinder_x(part, name="main_barrel", radius=0.0275, length=0.160, center=(0.002, y, 0.0), material=material)
    _add_cylinder_x(part, name="objective_tube", radius=0.0370, length=0.105, center=(0.0975, y, 0.0), material=material)
    _add_cylinder_x(part, name="objective_rim", radius=0.0400, length=0.014, center=(0.146, y, 0.0), material=material)
    _add_cylinder_x(part, name="front_retainer", radius=0.0218, length=0.009, center=(-0.0825, y, 0.0), material=material)

    # The central rubber bridge halves meet on the hinge plane at y=0 without
    # interpenetrating.  Their broad molded webs visibly support each tube.
    part.visual(
        Box((0.150, 0.037, 0.018)),
        origin=Origin(xyz=(0.020, side * 0.0205, 0.0)),
        material=material,
        name="bridge_web",
    )
    part.visual(
        Box((0.220, 0.010, 0.022)),
        origin=Origin(xyz=(0.010, side * 0.005, 0.0)),
        material=material,
        name="hinge_leaf",
    )

    for index, x_center in enumerate((-0.025, 0.028, 0.094)):
        part.visual(
            Box((0.030, 0.006, 0.026)),
            origin=Origin(xyz=(x_center, y + side * 0.0285, 0.0)),
            material=material,
            name=f"armor_pad_{index}",
        )


def _ribbed_focus_ring() -> cq.Workplane:
    """A hollow, rubberized individual-focus ring centered on the local X axis."""
    length = 0.030
    inner_r = 0.0185
    base_r = 0.0255
    ring = _tube_x(length, base_r, inner_r, x0=-length / 2.0)

    # Raised axial grip ribs around the ring.  They are shallow so the ring still
    # looks like a knurled rubber sleeve rather than a gear.
    rib_count = 24
    for i in range(rib_count):
        angle = 2.0 * math.pi * i / rib_count
        radial_y = math.cos(angle)
        radial_z = math.sin(angle)
        rib = (
            cq.Workplane("XY")
            .box(length * 0.92, 0.0024, 0.0040)
            .translate((0.0, 0.0, base_r + 0.0018))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), math.degrees(angle) - 90.0)
        )
        rib = rib.translate((0.0, radial_y * 0.0002, radial_z * 0.0002))
        ring = ring.union(rib)
    return ring


def _armored_body(side: int, *, root_side: bool) -> cq.Workplane:
    """One binocular half: objective tube, eyepiece carrier, bridge arm, and hinge knuckles."""
    y = side * TUBE_OFFSET_Y

    # Main rubber-armored optical body and the larger 50 mm class objective tube.
    body = _cylinder_x(0.160, 0.0275, x0=-0.078, y=y)
    objective = _tube_x(0.105, 0.0370, 0.0282, x0=0.045, y=y)
    front_rim = _tube_x(0.014, 0.0400, 0.0280, x0=0.139, y=y)
    eyepiece_neck = _cylinder_x(0.060, 0.0185, x0=-0.132, y=y)
    rear_retainer = _cylinder_x(0.007, 0.0218, x0=-0.126, y=y)
    front_retainer = _cylinder_x(0.007, 0.0218, x0=-0.083, y=y)

    body = body.union(objective).union(front_rim).union(eyepiece_neck)
    body = body.union(rear_retainer).union(front_retainer)

    # Molded bridge webs between each tube and the central hinge.  Interleaving
    # the knuckles along X keeps the hinge mechanically readable without broad
    # part-on-part overlap.
    bridge_y_center = side * 0.0205
    if root_side:
        bridge_segments = (
            (0.050, (-0.069, bridge_y_center, 0.0)),
            (0.080, (0.085, bridge_y_center, 0.0)),
        )
        for sx, center in bridge_segments:
            body = body.union(_box((sx, 0.041, 0.018), center))
        body = body.union(_cylinder_x(0.050, 0.0145, x0=-0.095))
        body = body.union(_cylinder_x(0.080, 0.0145, x0=0.045))
    else:
        body = body.union(_box((0.070, 0.041, 0.018), (-0.000, bridge_y_center, 0.0)))
        body = body.union(_tube_x(0.070, 0.0145, 0.0082, x0=-0.035))

    # Low rubber armor pads on the tube sides break up the cylindrical silhouette
    # and make the housing read as aviation field equipment.
    for x_center in (-0.025, 0.028, 0.094):
        pad = _box((0.030, 0.006, 0.026), (x_center, y + side * 0.0285, 0.0))
        body = body.union(pad)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aviation_pilot_binoculars")

    rubber = model.material("matte_black_rubber", rgba=(0.010, 0.012, 0.011, 1.0))
    dark_rubber = model.material("slightly_worn_rubber", rgba=(0.030, 0.034, 0.032, 1.0))
    glass = model.material("deep_coated_glass", rgba=(0.05, 0.16, 0.22, 0.62))

    tube_0 = model.part("tube_0")
    _add_primitive_body(tube_0, side=+1, material=rubber)
    tube_0.visual(
        Cylinder(radius=0.0295, length=0.004),
        origin=Origin(xyz=(0.1535, TUBE_OFFSET_Y, 0.0), rpy=CYLINDER_X_RPY),
        material=glass,
        name="objective_glass",
    )
    tube_1 = model.part("tube_1")
    _add_primitive_body(tube_1, side=-1, material=rubber)
    tube_1.visual(
        Cylinder(radius=0.0295, length=0.004),
        origin=Origin(xyz=(0.1535, -TUBE_OFFSET_Y, 0.0), rpy=CYLINDER_X_RPY),
        material=glass,
        name="objective_glass",
    )
    focus_ring_0 = model.part("focus_ring_0")
    focus_ring_0.visual(
        mesh_from_cadquery(_ribbed_focus_ring(), "focus_ring_0", tolerance=0.0005),
        material=dark_rubber,
        name="ribbed_ring",
    )
    _add_cylinder_x(focus_ring_0, name="eyecup", radius=0.0185, length=0.025, center=(-0.0275, 0.0, 0.0), material=dark_rubber)
    _add_cylinder_x(focus_ring_0, name="eyepiece_glass", radius=0.0155, length=0.003, center=(-0.0415, 0.0, 0.0), material=glass)

    focus_ring_1 = model.part("focus_ring_1")
    focus_ring_1.visual(
        mesh_from_cadquery(_ribbed_focus_ring(), "focus_ring_1", tolerance=0.0005),
        material=dark_rubber,
        name="ribbed_ring",
    )
    _add_cylinder_x(focus_ring_1, name="eyecup", radius=0.0185, length=0.025, center=(-0.0275, 0.0, 0.0), material=dark_rubber)
    _add_cylinder_x(focus_ring_1, name="eyepiece_glass", radius=0.0155, length=0.003, center=(-0.0415, 0.0, 0.0), material=glass)

    model.articulation(
        "bridge_hinge",
        ArticulationType.REVOLUTE,
        parent=tube_0,
        child=tube_1,
        origin=Origin(),
        axis=OPTICAL_AXIS,
        motion_limits=MotionLimits(lower=-0.22, upper=0.22, effort=3.0, velocity=0.8),
    )
    model.articulation(
        "focus_0_spin",
        ArticulationType.CONTINUOUS,
        parent=tube_0,
        child=focus_ring_0,
        origin=Origin(xyz=(RING_X, TUBE_OFFSET_Y, 0.0)),
        axis=OPTICAL_AXIS,
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )
    model.articulation(
        "focus_1_spin",
        ArticulationType.CONTINUOUS,
        parent=tube_1,
        child=focus_ring_1,
        origin=Origin(xyz=(RING_X, -TUBE_OFFSET_Y, 0.0)),
        axis=OPTICAL_AXIS,
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tube_0 = object_model.get_part("tube_0")
    tube_1 = object_model.get_part("tube_1")
    focus_ring_0 = object_model.get_part("focus_ring_0")
    focus_ring_1 = object_model.get_part("focus_ring_1")
    hinge = object_model.get_articulation("bridge_hinge")
    focus_0 = object_model.get_articulation("focus_0_spin")
    focus_1 = object_model.get_articulation("focus_1_spin")

    ctx.check("bridge hinge is revolute", hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("focus rings rotate continuously", focus_0.articulation_type == ArticulationType.CONTINUOUS and focus_1.articulation_type == ArticulationType.CONTINUOUS)
    ctx.expect_gap(
        tube_0,
        tube_1,
        axis="y",
        min_gap=0.030,
        max_gap=0.038,
        positive_elem="objective_glass",
        negative_elem="objective_glass",
        name="objective tubes are wide and separated",
    )
    ctx.expect_overlap(
        tube_0,
        tube_1,
        axes="x",
        min_overlap=0.003,
        elem_a="objective_glass",
        elem_b="objective_glass",
        name="objective tubes share a common forward axis length",
    )
    ctx.expect_overlap(focus_ring_0, tube_0, axes="x", min_overlap=0.006, name="focus ring 0 is retained by the eyepiece shoulder")
    ctx.expect_overlap(focus_ring_1, tube_1, axes="x", min_overlap=0.006, name="focus ring 1 is retained by the eyepiece shoulder")
    ctx.expect_contact(focus_ring_0, tube_0, contact_tol=0.0015, name="focus ring 0 is supported by its eyepiece sleeve")
    ctx.expect_contact(focus_ring_1, tube_1, contact_tol=0.0015, name="focus ring 1 is supported by its eyepiece sleeve")

    rest_aabb = ctx.part_element_world_aabb(tube_1, elem="objective_glass")
    with ctx.pose({hinge: 0.18, focus_0: math.pi, focus_1: -math.pi / 2.0}):
        adjusted_aabb = ctx.part_element_world_aabb(tube_1, elem="objective_glass")
        ctx.expect_gap(
            tube_0,
            tube_1,
            axis="y",
            min_gap=0.028,
            max_gap=0.040,
            positive_elem="objective_glass",
            negative_elem="objective_glass",
            name="hinge keeps objective tubes separated during IPD adjustment",
        )
    ctx.check(
        "bridge hinge changes interpupillary pose",
        rest_aabb is not None
        and adjusted_aabb is not None
        and abs(((adjusted_aabb[0][2] + adjusted_aabb[1][2]) / 2.0) - ((rest_aabb[0][2] + rest_aabb[1][2]) / 2.0)) > 0.006,
        details=f"rest={rest_aabb}, adjusted={adjusted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
