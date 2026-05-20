from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate(center)
    )


def _rounded_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    fillet: float,
    selector: str = "|Z",
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).edges(selector).fillet(fillet).translate(center)


def _make_fork_crown_and_blades() -> cq.Workplane:
    """One connected crown/blade casting around the separate solid steerer visual."""

    crown_socket = cq.Workplane("XY").cylinder(0.070, 0.030).translate((0.0, 0.0, -0.010))
    crown_bridge = _rounded_box((0.160, 0.056, 0.042), (0.0, 0.0, -0.036), fillet=0.010)

    body = crown_socket.union(crown_bridge)
    for x in (-0.056, 0.056):
        shoulder = _rounded_box((0.046, 0.058, 0.090), (x, 0.0, -0.070), fillet=0.010)
        blade = _rounded_box((0.021, 0.035, 0.310), (x, 0.012, -0.220), fillet=0.007)
        dropout = _rounded_box((0.044, 0.014, 0.055), (x, 0.016, -0.390), fillet=0.006)
        body = body.union(shoulder).union(blade).union(dropout)

    return body


def _make_stem_body() -> cq.Workplane:
    """Hollow BMX riser stem with a steerer clamp and center handlebar pinch collar."""

    steerer_collar = cq.Workplane("XY").cylinder(0.086, 0.027).translate((0.0, 0.0, 0.0))
    steerer_bore = cq.Workplane("XY").cylinder(0.100, 0.0140).translate((0.0, 0.0, 0.0))

    handlebar_collar = _cylinder_x(0.030, 0.118, (0.0, 0.102, 0.042))
    handlebar_bore = _cylinder_x(0.0108, 0.132, (0.0, 0.102, 0.042))

    bridge = _rounded_box((0.060, 0.116, 0.038), (0.0, 0.052, 0.022), fillet=0.008, selector="|Z")
    face_plate = _rounded_box((0.122, 0.014, 0.064), (0.0, 0.139, 0.042), fillet=0.005, selector="|Y")
    rear_split_lug = _rounded_box((0.062, 0.018, 0.064), (0.0, -0.031, 0.0), fillet=0.004, selector="|Z")

    body = (
        steerer_collar.union(bridge)
        .union(handlebar_collar)
        .union(face_plate)
        .union(rear_split_lug)
        .cut(steerer_bore)
        .cut(handlebar_bore)
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_fork_stem_bar")

    satin_black = model.material("satin_black", rgba=(0.01, 0.012, 0.012, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.085, 0.09, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.78, 0.78, 0.73, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.55, 0.55, 0.52, 1.0))

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.0143, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=satin_black,
        name="steerer",
    )
    fork.visual(
        mesh_from_cadquery(_make_fork_crown_and_blades(), "fork_crown_blades"),
        material=satin_black,
        name="crown_blades",
    )

    stem = model.part("stem")
    stem.visual(
        mesh_from_cadquery(_make_stem_body(), "stem_body"),
        material=dark_anodized,
        name="stem_body",
    )
    # Visible pinch hardware: two steerer-clamp bolts and four handlebar faceplate bolts.
    for z in (-0.024, 0.024):
        stem.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(xyz=(0.0, -0.056, z), rpy=(math.pi / 2, 0.0, 0.0)),
            material=bolt_steel,
            name=f"steerer_bolt_{0 if z < 0 else 1}",
        )
    for x in (-0.040, 0.040):
        for z in (0.018, 0.066):
            stem.visual(
                Cylinder(radius=0.0055, length=0.018),
                origin=Origin(xyz=(x, 0.150, z), rpy=(math.pi / 2, 0.0, 0.0)),
                material=bolt_steel,
                name=f"face_bolt_{0 if x < 0 else 1}_{0 if z < 0.04 else 1}",
            )

    handlebar = model.part("handlebar")
    tube_radius = 0.0111
    handlebar.visual(
        Cylinder(radius=tube_radius, length=0.340),
        origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
        material=chrome,
        name="clamp_tube",
    )
    for x in (-0.160, 0.160):
        handlebar.visual(
            Cylinder(radius=tube_radius, length=0.220),
            origin=Origin(xyz=(x, 0.0, 0.110)),
            material=chrome,
            name=f"upright_{0 if x < 0 else 1}",
        )
        handlebar.visual(
            Sphere(radius=tube_radius * 1.05),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=chrome,
            name=f"lower_weld_{0 if x < 0 else 1}",
        )
        handlebar.visual(
            Sphere(radius=tube_radius * 1.05),
            origin=Origin(xyz=(x, 0.0, 0.220)),
            material=chrome,
            name=f"top_weld_{0 if x < 0 else 1}",
        )
        handlebar.visual(
            Sphere(radius=0.010),
            origin=Origin(xyz=(x, 0.006, 0.120)),
            material=chrome,
            name=f"cross_weld_{0 if x < 0 else 1}",
        )
    handlebar.visual(
        Cylinder(radius=tube_radius, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.220), rpy=(0.0, math.pi / 2, 0.0)),
        material=chrome,
        name="top_tube",
    )
    handlebar.visual(
        Cylinder(radius=0.0083, length=0.340),
        origin=Origin(xyz=(0.0, 0.006, 0.120), rpy=(0.0, math.pi / 2, 0.0)),
        material=chrome,
        name="crossbar",
    )
    for x in (-0.340, 0.340):
        handlebar.visual(
            Cylinder(radius=0.0145, length=0.100),
            origin=Origin(xyz=(x, 0.0, 0.220), rpy=(0.0, math.pi / 2, 0.0)),
            material=rubber,
            name=f"grip_{0 if x < 0 else 1}",
        )

    model.articulation(
        "fork_to_stem",
        ArticulationType.FIXED,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.FIXED,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.102, 0.042)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")

    ctx.allow_overlap(
        fork,
        stem,
        elem_a="steerer",
        elem_b="stem_body",
        reason="The riser stem steerer clamp is modeled with slight compression around the solid unthreaded steerer.",
    )
    ctx.allow_overlap(
        stem,
        handlebar,
        elem_a="stem_body",
        elem_b="clamp_tube",
        reason="The center pinch collar is modeled as a small clamped interference on the handlebar tube.",
    )

    ctx.expect_overlap(
        fork,
        stem,
        axes="z",
        elem_a="steerer",
        elem_b="stem_body",
        min_overlap=0.070,
        name="steerer passes through stem clamp height",
    )
    ctx.expect_within(
        fork,
        stem,
        axes="xy",
        inner_elem="steerer",
        outer_elem="stem_body",
        margin=0.004,
        name="steerer centered in stem clamp footprint",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="x",
        elem_a="clamp_tube",
        elem_b="stem_body",
        min_overlap=0.100,
        name="handlebar tube retained by wide pinch collar",
    )
    ctx.expect_within(
        handlebar,
        stem,
        axes="yz",
        inner_elem="clamp_tube",
        outer_elem="stem_body",
        margin=0.004,
        name="handlebar tube centered inside collar bore",
    )

    bar_aabb = ctx.part_world_aabb(handlebar)
    ctx.check(
        "wide freestyle handlebar span",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.72,
        details=f"handlebar_aabb={bar_aabb}",
    )
    ctx.check(
        "riser bar height above stem",
        bar_aabb is not None and (bar_aabb[1][2] - bar_aabb[0][2]) > 0.22,
        details=f"handlebar_aabb={bar_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
