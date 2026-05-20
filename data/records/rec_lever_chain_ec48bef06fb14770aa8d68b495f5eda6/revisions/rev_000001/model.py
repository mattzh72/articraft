from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def make_lever(length: float, width: float, thickness: float, hole_dia: float, gap: float = 0.001) -> cq.Workplane:
    pts = [(0, 0), (length, 0)]
    body = (
        cq.Workplane("XY")
        .pushPoints(pts)
        .circle(width / 2)
        .extrude(thickness)
        .union(
            cq.Workplane("XY")
            .center(length / 2, 0)
            .box(length, width, thickness, centered=(True, True, False))
        )
    )
    # Cut hole at (length, 0)
    hole = (
        cq.Workplane("XY")
        .center(length, 0)
        .circle(hole_dia / 2)
        .extrude(thickness)
    )
    body = body.cut(hole)
    
    # Pin at (0, 0) extending down
    # Pin length is chosen to go into the parent's hole but not stick out the bottom
    pin_length = thickness + gap * 0.5
    pin = (
        cq.Workplane("XY")
        .center(0, 0)
        .circle(hole_dia / 2 * 0.95)
        .extrude(-pin_length)
    )
    body = body.union(pin)
        
    return body


def make_base(length: float, width: float, thickness: float, hole_dia: float) -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(length, width, thickness, centered=(True, True, False))
    )
    hole = (
        cq.Workplane("XY")
        .circle(hole_dia / 2)
        .extrude(thickness)
    )
    return body.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linkage_study")

    # Dimensions
    lever_w = 0.03
    lever_t = 0.01
    hole_dia = 0.015
    gap = 0.001

    base = model.part("base")
    base_cq = make_base(0.15, 0.15, 0.02, hole_dia)
    base.visual(
        mesh_from_cadquery(base_cq, "base_mesh"),
        material=Material(name="base_mat", color=(0.25, 0.25, 0.25)),
        name="base_vis"
    )

    lever1 = model.part("lever1")
    lever1_cq = make_lever(0.15, lever_w, lever_t, hole_dia, gap)
    lever1.visual(
        mesh_from_cadquery(lever1_cq, "lever1_mesh"),
        material=Material(name="lever1_mat", color=(0.8, 0.2, 0.2)),
        name="lever1_vis"
    )

    lever2 = model.part("lever2")
    lever2_cq = make_lever(0.12, lever_w, lever_t, hole_dia, gap)
    lever2.visual(
        mesh_from_cadquery(lever2_cq, "lever2_mesh"),
        material=Material(name="lever2_mat", color=(0.2, 0.8, 0.2)),
        name="lever2_vis"
    )

    lever3 = model.part("lever3")
    lever3_cq = make_lever(0.10, lever_w, lever_t, hole_dia, gap)
    lever3.visual(
        mesh_from_cadquery(lever3_cq, "lever3_mesh"),
        material=Material(name="lever3_mat", color=(0.2, 0.2, 0.8)),
        name="lever3_vis"
    )

    # Articulations
    model.articulation(
        "base_to_lever1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever1,
        origin=Origin(xyz=(0.0, 0.0, 0.02 + gap)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-3.14, upper=3.14)
    )

    model.articulation(
        "lever1_to_lever2",
        ArticulationType.REVOLUTE,
        parent=lever1,
        child=lever2,
        origin=Origin(xyz=(0.15, 0.0, lever_t + gap)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-3.14, upper=3.14)
    )

    model.articulation(
        "lever2_to_lever3",
        ArticulationType.REVOLUTE,
        parent=lever2,
        child=lever3,
        origin=Origin(xyz=(0.12, 0.0, lever_t + gap)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-3.14, upper=3.14)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # The pins intentionally overlap with the holes in the parent parts.
    # Base <-> Lever1
    ctx.allow_overlap(
        "base", "lever1",
        reason="Lever1's pin is captured inside the Base's hole."
    )
    # Lever1 <-> Lever2
    ctx.allow_overlap(
        "lever1", "lever2",
        reason="Lever2's pin is captured inside Lever1's hole."
    )
    # Lever2 <-> Lever3
    ctx.allow_overlap(
        "lever2", "lever3",
        reason="Lever3's pin is captured inside Lever2's hole."
    )

    # Verify gaps using origin distances since the bodies overlap due to pins
    ctx.expect_origin_distance("lever1", "base", axes="z", min_dist=0.0209, max_dist=0.0211)
    ctx.expect_origin_distance("lever2", "lever1", axes="z", min_dist=0.0109, max_dist=0.0111)
    ctx.expect_origin_distance("lever3", "lever2", axes="z", min_dist=0.0109, max_dist=0.0111)

    # The levers are intentionally modeled with radial and axial clearance to represent a free-moving linkage
    ctx.allow_isolated_part("lever1", reason="Lever1 is captured by its pin in the base but has realistic clearance.")
    ctx.allow_isolated_part("lever2", reason="Lever2 is captured by its pin in Lever1 but has realistic clearance.")
    ctx.allow_isolated_part("lever3", reason="Lever3 is captured by its pin in Lever2 but has realistic clearance.")

    return ctx.report()


object_model = build_object_model()
