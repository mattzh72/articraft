from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _dovetail_prism(length: float, bottom_width: float, top_width: float, height: float) -> MeshGeometry:
    """Trapezoidal rail prism running along local X, with bottom on z=0."""
    x0, x1 = -length / 2.0, length / 2.0
    by, ty = bottom_width / 2.0, top_width / 2.0
    verts = [
        (x0, -by, 0.0),
        (x0, by, 0.0),
        (x0, ty, height),
        (x0, -ty, height),
        (x1, -by, 0.0),
        (x1, by, 0.0),
        (x1, ty, height),
        (x1, -ty, height),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 7, 6),
        (4, 6, 5),
        (0, 4, 5),
        (0, 5, 1),
        (3, 2, 6),
        (3, 6, 7),
        (1, 5, 6),
        (1, 6, 2),
        (0, 3, 7),
        (0, 7, 4),
    ]
    return MeshGeometry(vertices=verts, faces=faces)


def _handwheel_geometry() -> MeshGeometry:
    """Small metal handwheel whose rotation/screw axis is local +Z."""
    geom = TorusGeometry(radius=0.033, tube=0.0042, radial_segments=18, tubular_segments=48)
    geom.translate(0.0, 0.0, 0.037)

    hub = CylinderGeometry(0.011, 0.056, radial_segments=32)
    hub.translate(0.0, 0.0, 0.028)
    geom.merge(hub)

    # Raised collars cut across the shaft surface so the screw end reads as threaded.
    for z in (0.006, 0.012, 0.018):
        thread_ring = TorusGeometry(radius=0.0112, tube=0.0008, radial_segments=12, tubular_segments=28)
        thread_ring.translate(0.0, 0.0, z)
        geom.merge(thread_ring)

    # Two solid spokes cross the wheel and deliberately overlap the hub and rim.
    spoke_x = CylinderGeometry(0.0028, 0.060, radial_segments=12)
    spoke_x.rotate_y(math.pi / 2.0).translate(0.0, 0.0, 0.037)
    geom.merge(spoke_x)
    spoke_y = CylinderGeometry(0.0028, 0.060, radial_segments=12)
    spoke_y.rotate_x(math.pi / 2.0).translate(0.0, 0.0, 0.037)
    geom.merge(spoke_y)

    crank = CylinderGeometry(0.0054, 0.030, radial_segments=18)
    crank.translate(0.024, 0.024, 0.055)
    geom.merge(crank)
    return geom

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drill_press_cross_slide_vise")

    cast_iron = model.material("dark_cast_iron", rgba=(0.22, 0.23, 0.23, 1.0))
    machined = model.material("machined_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    black = model.material("blackened_handwheel", rgba=(0.02, 0.02, 0.02, 1.0))
    dark_slot = model.material("shadowed_slots", rgba=(0.015, 0.015, 0.014, 1.0))

    # One-piece-looking base casting with broad drill-press vise proportions.
    base = model.part("base")
    base_casting = (
        cq.Workplane("XY")
        .box(0.46, 0.24, 0.045)
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.003)
    )
    base.visual(
        mesh_from_cadquery(base_casting, "base_casting", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast_iron,
        name="base_casting",
    )
    base.visual(
        mesh_from_geometry(_dovetail_prism(0.36, 0.124, 0.074, 0.028), "base_dovetail"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=cast_iron,
        name="base_dovetail",
    )
    for i, x in enumerate((-0.165, 0.165)):
        base.visual(
            Box((0.070, 0.030, 0.0014)),
            origin=Origin(xyz=(x, 0.087, 0.0455)),
            material=dark_slot,
            name=f"mount_slot_{i}",
        )
        base.visual(
            Box((0.070, 0.030, 0.0014)),
            origin=Origin(xyz=(x, -0.087, 0.0455)),
            material=dark_slot,
            name=f"mount_slot_{i + 2}",
        )
    base.visual(
        Box((0.018, 0.045, 0.040)),
        origin=Origin(xyz=(0.224, -0.105, 0.064)),
        material=cast_iron,
        name="x_bearing_block",
    )

    x_stage = model.part("x_stage")
    x_stage.visual(
        Box((0.250, 0.162, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="saddle_body",
    )
    # Exposed sloped strips echo the underside of the dovetail saddle without
    # hiding the base rail's trapezoid on either long edge.
    for side, y in enumerate((-0.064, 0.064)):
        x_stage.visual(
            mesh_from_geometry(_dovetail_prism(0.235, 0.020, 0.010, 0.018), f"saddle_gib_{side}"),
            origin=Origin(xyz=(0.0, y, 0.000), rpy=(0.0, 0.0, math.pi)),
            material=machined,
            name=f"saddle_gib_{side}",
        )
    for i, x in enumerate((-0.058, 0.058)):
        x_stage.visual(
            Box((0.026, 0.172, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.046)),
            material=machined,
            name=f"flat_rail_{i}",
        )
    x_stage.visual(
        Box((0.174, 0.034, 0.080)),
        origin=Origin(xyz=(0.0, 0.066, 0.092)),
        material=cast_iron,
        name="fixed_jaw_block",
    )
    x_stage.visual(
        Box((0.160, 0.004, 0.045)),
        origin=Origin(xyz=(0.0, 0.047, 0.106)),
        material=machined,
        name="fixed_jaw_face",
    )
    for i, z in enumerate((0.090, 0.097, 0.104, 0.111, 0.118, 0.125)):
        x_stage.visual(
            Box((0.155, 0.0014, 0.0017)),
            origin=Origin(xyz=(0.0, 0.0447, z)),
            material=dark_slot,
            name=f"fixed_serration_{i}",
        )
    x_stage.visual(
        Box((0.052, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.091, 0.026)),
        material=cast_iron,
        name="y_bearing_foot",
    )
    x_stage.visual(
        Box((0.050, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, -0.101, 0.044)),
        material=cast_iron,
        name="y_bearing_block",
    )

    y_jaw = model.part("y_jaw")
    y_jaw.visual(
        Box((0.188, 0.110, 0.020)),
        origin=Origin(xyz=(0.0, -0.020, 0.010)),
        material=cast_iron,
        name="slide_plate",
    )
    y_jaw.visual(
        Box((0.072, 0.092, 0.008)),
        origin=Origin(xyz=(0.0, -0.020, 0.024)),
        material=machined,
        name="rail_tongue",
    )
    y_jaw.visual(
        Box((0.174, 0.035, 0.060)),
        origin=Origin(xyz=(0.0, -0.060, 0.052)),
        material=cast_iron,
        name="moving_jaw_block",
    )
    y_jaw.visual(
        Box((0.160, 0.004, 0.045)),
        origin=Origin(xyz=(0.0, -0.0405, 0.054)),
        material=machined,
        name="moving_jaw_face",
    )
    for i, z in enumerate((0.037, 0.044, 0.051, 0.058, 0.065, 0.072)):
        y_jaw.visual(
            Box((0.155, 0.0014, 0.0017)),
            origin=Origin(xyz=(0.0, -0.0389, z)),
            material=dark_slot,
            name=f"moving_serration_{i}",
        )

    wheel_mesh = _handwheel_geometry()
    x_handwheel = model.part("x_handwheel")
    x_handwheel.visual(
        mesh_from_geometry(wheel_mesh.copy(), "x_handwheel"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="wheel",
    )
    y_handwheel = model.part("y_handwheel")
    y_handwheel.visual(
        mesh_from_geometry(wheel_mesh.copy(), "y_handwheel"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="wheel",
    )

    model.articulation(
        "base_to_x_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.08, lower=-0.045, upper=0.045),
    )
    model.articulation(
        "x_stage_to_y_jaw",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.06, lower=0.0, upper=0.040),
    )
    model.articulation(
        "base_to_x_handwheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=x_handwheel,
        origin=Origin(xyz=(0.233, -0.105, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
    )
    model.articulation(
        "x_stage_to_y_handwheel",
        ArticulationType.CONTINUOUS,
        parent=x_stage,
        child=y_handwheel,
        origin=Origin(xyz=(0.0, -0.110, 0.044)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_stage = object_model.get_part("x_stage")
    y_jaw = object_model.get_part("y_jaw")
    x_handwheel = object_model.get_part("x_handwheel")
    y_handwheel = object_model.get_part("y_handwheel")
    x_slide = object_model.get_articulation("base_to_x_stage")
    y_slide = object_model.get_articulation("x_stage_to_y_jaw")

    ctx.expect_gap(
        x_stage,
        base,
        axis="z",
        positive_elem="saddle_body",
        negative_elem="base_dovetail",
        max_gap=0.002,
        max_penetration=0.0,
        name="x saddle rides on top of base dovetail",
    )
    ctx.expect_overlap(
        x_stage,
        base,
        axes="xy",
        elem_a="saddle_body",
        elem_b="base_dovetail",
        min_overlap=0.070,
        name="x saddle overlaps the dovetail footprint",
    )
    ctx.expect_gap(
        y_jaw,
        x_stage,
        axis="z",
        positive_elem="slide_plate",
        negative_elem="flat_rail_0",
        max_gap=0.002,
        max_penetration=0.0,
        name="y jaw sits on the flat rail",
    )
    ctx.expect_overlap(
        y_jaw,
        x_stage,
        axes="xz",
        elem_a="moving_jaw_face",
        elem_b="fixed_jaw_face",
        min_overlap=0.035,
        name="opposing serrated jaw faces align",
    )
    ctx.expect_gap(
        x_stage,
        y_jaw,
        axis="y",
        positive_elem="fixed_jaw_face",
        negative_elem="moving_jaw_face",
        min_gap=0.040,
        max_gap=0.100,
        name="jaw faces have a realistic vise opening",
    )
    ctx.expect_contact(
        x_handwheel,
        base,
        elem_a="wheel",
        elem_b="x_bearing_block",
        contact_tol=0.004,
        name="x handwheel bears on base screw support",
    )
    ctx.expect_contact(
        y_handwheel,
        x_stage,
        elem_a="wheel",
        elem_b="y_bearing_block",
        contact_tol=0.004,
        name="y handwheel bears on top-stage screw support",
    )

    x_rest = ctx.part_world_position(x_stage)
    with ctx.pose({x_slide: 0.040}):
        x_shifted = ctx.part_world_position(x_stage)
        ctx.expect_gap(
            x_stage,
            base,
            axis="z",
            positive_elem="saddle_body",
            negative_elem="base_dovetail",
            max_gap=0.002,
            max_penetration=0.0,
            name="x saddle remains seated when advanced",
        )
    ctx.check(
        "x stage translates along the dovetail",
        x_rest is not None and x_shifted is not None and x_shifted[0] > x_rest[0] + 0.030,
        details=f"rest={x_rest}, shifted={x_shifted}",
    )

    with ctx.pose({y_slide: 0.040}):
        ctx.expect_gap(
            x_stage,
            y_jaw,
            axis="y",
            positive_elem="fixed_jaw_face",
            negative_elem="moving_jaw_face",
            min_gap=0.001,
            max_gap=0.060,
            name="y jaw slide closes the vise opening",
        )

    return ctx.report()


object_model = build_object_model()
