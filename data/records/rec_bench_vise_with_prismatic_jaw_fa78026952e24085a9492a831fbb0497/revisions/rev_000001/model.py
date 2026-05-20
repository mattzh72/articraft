from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _trapezoid_prism_mesh(
    *,
    length: float,
    bottom_width: float,
    top_width: float,
    height: float,
    name: str,
):
    """A simple centered dovetail prism: length along X, trapezoid in Y/Z."""
    geom = MeshGeometry()
    x0 = -length / 2.0
    x1 = length / 2.0
    b = bottom_width / 2.0
    t = top_width / 2.0
    pts = [
        (x0, -b, 0.0),
        (x0, b, 0.0),
        (x0, t, height),
        (x0, -t, height),
        (x1, -b, 0.0),
        (x1, b, 0.0),
        (x1, t, height),
        (x1, -t, height),
    ]
    for p in pts:
        geom.add_vertex(*p)
    for tri in (
        (0, 1, 2),
        (0, 2, 3),
        (4, 7, 6),
        (4, 6, 5),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ):
        geom.add_face(*tri)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="welding_positioner_vise")

    paint = model.material("hammered_blue_paint", rgba=(0.05, 0.18, 0.34, 1.0))
    dark_paint = model.material("dark_machined_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    bright_steel = model.material("polished_leadscrew_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    jaw_steel = model.material("hardened_jaw_steel", rgba=(0.30, 0.31, 0.30, 1.0))
    black = model.material("blackened_handle", rgba=(0.02, 0.02, 0.018, 1.0))

    rail_mesh = _trapezoid_prism_mesh(
        length=0.36,
        bottom_width=0.032,
        top_width=0.052,
        height=0.020,
        name="dovetail_rail",
    )
    thread_ring = mesh_from_geometry(
        TorusGeometry(0.0108, 0.0014, radial_segments=10, tubular_segments=24),
        "leadscrew_thread_ring",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.22, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_paint,
        name="floor_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.075, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=paint,
        name="center_column",
    )
    pedestal.visual(
        Cylinder(radius=0.16, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1275)),
        material=dark_paint,
        name="turntable_base",
    )

    base_plate = model.part("base_plate")
    base_plate.visual(
        Cylinder(radius=0.158, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_paint,
        name="turntable_top",
    )
    base_plate.visual(
        Box((0.52, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0415)),
        material=paint,
        name="plate",
    )
    base_plate.visual(
        rail_mesh,
        origin=Origin(xyz=(0.0, -0.055, 0.059)),
        material=dark_paint,
        name="rail_0",
    )
    base_plate.visual(
        rail_mesh,
        origin=Origin(xyz=(0.0, 0.055, 0.059)),
        material=dark_paint,
        name="rail_1",
    )
    base_plate.visual(
        Box((0.13, 0.205, 0.022)),
        origin=Origin(xyz=(0.14, 0.0, 0.069)),
        material=paint,
        name="fixed_foot",
    )
    base_plate.visual(
        Box((0.032, 0.050, 0.024)),
        origin=Origin(xyz=(0.078, 0.0, 0.091)),
        material=dark_paint,
        name="leadnut_support",
    )
    base_plate.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.078, 0.0, 0.103), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="leadnut",
    )
    base_plate.visual(
        Box((0.080, 0.180, 0.112)),
        origin=Origin(xyz=(0.14, 0.0, 0.116)),
        material=paint,
        name="fixed_block",
    )
    base_plate.visual(
        Box((0.006, 0.162, 0.060)),
        origin=Origin(xyz=(0.097, 0.0, 0.122)),
        material=jaw_steel,
        name="fixed_jaw_face",
    )
    for iz, z in enumerate((0.104, 0.122, 0.140)):
        base_plate.visual(
            Box((0.007, 0.150, 0.004)),
            origin=Origin(xyz=(0.093, 0.0, z)),
            material=bright_steel,
            name=f"fixed_tooth_{iz}",
        )
    for i, (x, y) in enumerate(((-0.205, -0.085), (-0.205, 0.085), (0.205, -0.085), (0.205, 0.085))):
        base_plate.visual(
            Cylinder(radius=0.014, length=0.007),
            origin=Origin(xyz=(x, y, 0.0625)),
            material=dark_paint,
            name=f"bolt_{i}",
        )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.100, 0.064, 0.014)),
        origin=Origin(xyz=(0.0, -0.055, 0.007)),
        material=dark_paint,
        name="shoe_0",
    )
    moving_jaw.visual(
        Box((0.100, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.089, -0.006)),
        material=dark_paint,
        name="shoe_lip_0_0",
    )
    moving_jaw.visual(
        Box((0.100, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.021, -0.006)),
        material=dark_paint,
        name="shoe_lip_0_1",
    )
    moving_jaw.visual(
        Box((0.100, 0.064, 0.014)),
        origin=Origin(xyz=(0.0, 0.055, 0.007)),
        material=dark_paint,
        name="shoe_1",
    )
    moving_jaw.visual(
        Box((0.100, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.021, -0.006)),
        material=dark_paint,
        name="shoe_lip_1_0",
    )
    moving_jaw.visual(
        Box((0.100, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.089, -0.006)),
        material=dark_paint,
        name="shoe_lip_1_1",
    )
    moving_jaw.visual(
        Box((0.086, 0.190, 0.112)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=paint,
        name="jaw_block",
    )
    moving_jaw.visual(
        Box((0.006, 0.164, 0.062)),
        origin=Origin(xyz=(0.046, 0.0, 0.076)),
        material=jaw_steel,
        name="jaw_face",
    )
    for iz, z in enumerate((0.058, 0.076, 0.094)):
        moving_jaw.visual(
            Box((0.007, 0.152, 0.004)),
            origin=Origin(xyz=(0.050, 0.0, z)),
            material=bright_steel,
            name=f"jaw_tooth_{iz}",
        )
    moving_jaw.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(-0.047, 0.0, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_paint,
        name="screw_boss",
    )

    leadscrew = model.part("leadscrew")
    leadscrew.visual(
        Cylinder(radius=0.010, length=0.396),
        origin=Origin(xyz=(-0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="threaded_shaft",
    )
    for i, x in enumerate((-0.176, -0.132, -0.088, -0.044, 0.000, 0.044, 0.088, 0.132, 0.176)):
        leadscrew.visual(
            thread_ring,
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bright_steel,
            name=f"thread_{i}",
        )
    leadscrew.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(-0.222, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="handle_hub",
    )
    leadscrew.visual(
        Cylinder(radius=0.007, length=0.170),
        origin=Origin(xyz=(-0.222, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="handle_bar",
    )
    for i, y in enumerate((-0.092, 0.092)):
        leadscrew.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(-0.222, y, 0.0)),
            material=black,
            name=f"handle_knob_{i}",
        )

    model.articulation(
        "pedestal_to_plate",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=base_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0),
    )
    model.articulation(
        "plate_to_jaw",
        ArticulationType.PRISMATIC,
        parent=base_plate,
        child=moving_jaw,
        origin=Origin(xyz=(-0.130, 0.0, 0.079)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.04, lower=0.0, upper=0.120),
    )
    model.articulation(
        "jaw_to_leadscrew",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=leadscrew,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    base_plate = object_model.get_part("base_plate")
    moving_jaw = object_model.get_part("moving_jaw")
    leadscrew = object_model.get_part("leadscrew")
    swivel = object_model.get_articulation("pedestal_to_plate")
    slide = object_model.get_articulation("plate_to_jaw")
    screw_turn = object_model.get_articulation("jaw_to_leadscrew")

    ctx.allow_overlap(
        moving_jaw,
        leadscrew,
        elem_a="jaw_block",
        elem_b="threaded_shaft",
        reason="The leadscrew shaft intentionally passes through the drilled moving-jaw bore.",
    )
    ctx.allow_overlap(
        leadscrew,
        moving_jaw,
        elem_a="threaded_shaft",
        elem_b="screw_boss",
        reason="The screw shaft is intentionally captured inside the front boss bushing.",
    )
    ctx.expect_within(
        leadscrew,
        moving_jaw,
        axes="yz",
        inner_elem="threaded_shaft",
        outer_elem="jaw_block",
        margin=0.002,
        name="leadscrew centered in jaw bore",
    )
    ctx.expect_overlap(
        moving_jaw,
        leadscrew,
        axes="x",
        elem_a="jaw_block",
        elem_b="threaded_shaft",
        min_overlap=0.030,
        name="leadscrew retained in moving jaw",
    )
    ctx.expect_within(
        leadscrew,
        moving_jaw,
        axes="yz",
        inner_elem="threaded_shaft",
        outer_elem="screw_boss",
        margin=0.002,
        name="leadscrew centered in front bushing",
    )
    ctx.expect_overlap(
        leadscrew,
        moving_jaw,
        axes="x",
        elem_a="threaded_shaft",
        elem_b="screw_boss",
        min_overlap=0.018,
        name="front bushing captures leadscrew",
    )
    ctx.expect_contact(
        base_plate,
        pedestal,
        elem_a="turntable_top",
        elem_b="turntable_base",
        contact_tol=0.0005,
        name="turntable plates are seated",
    )
    ctx.expect_contact(
        moving_jaw,
        base_plate,
        elem_a="shoe_0",
        elem_b="rail_0",
        contact_tol=0.0005,
        name="moving jaw rides first dovetail rail",
    )
    ctx.expect_contact(
        moving_jaw,
        base_plate,
        elem_a="shoe_1",
        elem_b="rail_1",
        contact_tol=0.0005,
        name="moving jaw rides second dovetail rail",
    )
    ctx.expect_overlap(
        moving_jaw,
        base_plate,
        axes="x",
        elem_a="shoe_0",
        elem_b="rail_0",
        min_overlap=0.090,
        name="front jaw shoe remains on rail at rest",
    )

    rest_jaw = ctx.part_world_position(moving_jaw)
    with ctx.pose({slide: 0.120}):
        advanced_jaw = ctx.part_world_position(moving_jaw)
        ctx.expect_gap(
            base_plate,
            moving_jaw,
            axis="x",
            min_gap=0.035,
            max_gap=0.090,
            positive_elem="fixed_jaw_face",
            negative_elem="jaw_face",
            name="jaws close with usable work gap",
        )
        ctx.expect_overlap(
            moving_jaw,
            base_plate,
            axes="x",
            elem_a="shoe_0",
            elem_b="rail_0",
            min_overlap=0.090,
            name="front jaw shoe remains on rail when advanced",
        )
    ctx.check(
        "front jaw advances toward rear jaw",
        rest_jaw is not None
        and advanced_jaw is not None
        and advanced_jaw[0] > rest_jaw[0] + 0.11,
        details=f"rest={rest_jaw}, advanced={advanced_jaw}",
    )

    plate0 = ctx.part_element_world_aabb(base_plate, elem="plate")
    with ctx.pose({swivel: math.pi / 2.0}):
        plate90 = ctx.part_element_world_aabb(base_plate, elem="plate")
    if plate0 is not None and plate90 is not None:
        dx0 = plate0[1][0] - plate0[0][0]
        dy0 = plate0[1][1] - plate0[0][1]
        dx90 = plate90[1][0] - plate90[0][0]
        dy90 = plate90[1][1] - plate90[0][1]
        ctx.check(
            "base plate swivels continuously",
            dx0 > dy0 * 1.8 and dy90 > dx90 * 1.8,
            details=f"rest=({dx0:.3f},{dy0:.3f}), quarter_turn=({dx90:.3f},{dy90:.3f})",
        )
    else:
        ctx.fail("base plate swivel aabb", "Could not measure rectangular plate before and after swivel.")

    bar0 = ctx.part_element_world_aabb(leadscrew, elem="handle_bar")
    with ctx.pose({screw_turn: math.pi / 2.0}):
        bar90 = ctx.part_element_world_aabb(leadscrew, elem="handle_bar")
    if bar0 is not None and bar90 is not None:
        y0 = bar0[1][1] - bar0[0][1]
        z0 = bar0[1][2] - bar0[0][2]
        y90 = bar90[1][1] - bar90[0][1]
        z90 = bar90[1][2] - bar90[0][2]
        ctx.check(
            "bar handle rotates screw",
            y0 > z0 * 5.0 and z90 > y90 * 5.0,
            details=f"rest_yz=({y0:.3f},{z0:.3f}), turn_yz=({y90:.3f},{z90:.3f})",
        )
    else:
        ctx.fail("bar handle rotation aabb", "Could not measure handle bar before and after screw rotation.")

    return ctx.report()


object_model = build_object_model()
