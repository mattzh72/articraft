from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BOX_LENGTH = 0.42
BOX_DEPTH = 0.26
BODY_HEIGHT = 0.16
HINGE_Y = -BOX_DEPTH / 2.0 - 0.009
HINGE_Z = 0.181
HINGE_RADIUS = 0.006


def _rounded_open_body() -> cq.Workplane:
    """Painted metal storage tub: rounded outside, open top, true hollow interior."""
    wall = 0.010
    shell = (
        cq.Workplane("XY")
        .box(BOX_LENGTH, BOX_DEPTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.026)
        .faces(">Z")
        .shell(-wall)
    )
    return shell.translate((0.0, 0.0, BODY_HEIGHT / 2.0))


def _rounded_frame(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
    radius: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(outer_x, outer_y, height).edges("|Z").fillet(radius)
    inner_radius = max(0.002, radius - (outer_x - inner_x) * 0.5)
    cutter = (
        cq.Workplane("XY")
        .box(inner_x, inner_y, height + 0.004)
        .edges("|Z")
        .fillet(inner_radius)
    )
    return outer.cut(cutter)


def _organizer_tray() -> cq.Workplane:
    """Connected polymer insert with a base and low dividers for sewing notions."""
    base = cq.Workplane("XY").box(0.356, 0.194, 0.006)
    # Low partition ribs are fused to the tray base so the insert is one supported piece.
    base = base.union(cq.Workplane("XY").box(0.010, 0.188, 0.032).translate((-0.060, 0.0, 0.013)))
    base = base.union(cq.Workplane("XY").box(0.010, 0.188, 0.032).translate((0.072, 0.0, 0.013)))
    base = base.union(cq.Workplane("XY").box(0.348, 0.010, 0.032).translate((0.0, -0.045, 0.013)))
    base = base.union(cq.Workplane("XY").box(0.132, 0.010, 0.028).translate((-0.126, 0.050, 0.011)))
    return base.edges("|Z").fillet(0.003).translate((0.0, 0.010, 0.013))


def _lid_shell() -> cq.Workplane:
    """Thin stamped lid cap, open on the underside rather than a solid block."""
    lid = (
        cq.Workplane("XY")
        .box(0.436, 0.265, 0.022)
        .edges("|Z")
        .fillet(0.024)
        .faces("<Z")
        .shell(-0.0045)
    )
    return lid


def _rounded_pad(length: float, depth: float, height: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, depth, height).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sewing_box")

    painted_metal = model.material("warm_painted_metal", rgba=(0.72, 0.66, 0.58, 1.0))
    soft_ivory = model.material("soft_ivory_lid", rgba=(0.86, 0.82, 0.74, 1.0))
    dark_polymer = model.material("charcoal_polymer", rgba=(0.075, 0.078, 0.082, 1.0))
    elastomer = model.material("matte_elastomer", rgba=(0.025, 0.025, 0.027, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.70, 0.66, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_open_body(), "body_shell", tolerance=0.0008),
        material=painted_metal,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(
            _rounded_frame(0.414, 0.254, 0.360, 0.200, 0.008, 0.024),
            "rim_gasket",
            tolerance=0.0008,
        ),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + 0.004)),
        material=dark_polymer,
        name="rim_gasket",
    )
    body.visual(
        mesh_from_cadquery(_organizer_tray(), "organizer_tray", tolerance=0.0008),
        material=dark_polymer,
        name="organizer_tray",
    )
    body.visual(
        Box((0.070, 0.0025, 0.026)),
        origin=Origin(xyz=(0.0, BOX_DEPTH / 2.0 + 0.00125, 0.113)),
        material=brushed_steel,
        name="front_catch_plate",
    )
    body.visual(
        Box((0.335, 0.0050, 0.044)),
        origin=Origin(xyz=(0.0, -BOX_DEPTH / 2.0 - 0.0015, 0.154)),
        material=brushed_steel,
        name="rear_hinge_leaf",
    )
    # Two fixed outer knuckles on the body side of a three-knuckle rear hinge.
    for knuckle_name, x in (
        ("body_hinge_knuckle_0", -0.145),
        ("body_hinge_knuckle_1", 0.145),
    ):
        body.visual(
            Box((0.024, 0.010, 0.014)),
            origin=Origin(xyz=(x, HINGE_Y + 0.005, HINGE_Z - 0.002)),
            material=brushed_steel,
            name=knuckle_name.replace("knuckle", "saddle"),
        )
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.110),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=knuckle_name,
        )
    for index, x in enumerate((-0.158, 0.158)):
        for y in (-0.082, 0.082):
            body.visual(
                Cylinder(radius=0.017, length=0.010),
                origin=Origin(xyz=(x, y, -0.005)),
                material=elastomer,
                name=f"foot_{index}_{0 if y < 0 else 1}",
            )
    body.inertial = Inertial.from_geometry(
        Box((BOX_LENGTH, BOX_DEPTH, BODY_HEIGHT)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.006 + 0.265 / 2.0, 0.0)),
        material=soft_ivory,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_rounded_pad(0.238, 0.052, 0.004, 0.014), "lid_grip_pad", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.143, 0.013)),
        material=elastomer,
        name="lid_grip_pad",
    )
    lid.visual(
        Box((0.072, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.270, -0.004)),
        material=dark_polymer,
        name="front_pull_lip",
    )
    lid.visual(
        Box((0.150, 0.016, 0.0030)),
        origin=Origin(xyz=(0.0, 0.008, -0.0065)),
        material=brushed_steel,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="lid_hinge_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.436, 0.265, 0.028)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.138, 0.002)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.2, lower=0.0, upper=1.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    ctx.check(
        "hinge has consumer lid limits",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and 1.7 <= hinge.motion_limits.upper <= 1.95,
        details=f"limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="rim_gasket",
            min_gap=0.001,
            max_gap=0.008,
            name="closed lid has a tight gasket reveal",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="rim_gasket",
            min_overlap=0.18,
            name="lid covers the storage rim",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem="lid_hinge_knuckle",
            negative_elem="body_hinge_knuckle_0",
            min_gap=0.006,
            max_gap=0.014,
            name="left hinge knuckle clearance",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="x",
            positive_elem="body_hinge_knuckle_1",
            negative_elem="lid_hinge_knuckle",
            min_gap=0.006,
            max_gap=0.014,
            name="right hinge knuckle clearance",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    with ctx.pose({hinge: 1.45}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    ctx.check(
        "hinged lid opens upward about rear barrel",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
