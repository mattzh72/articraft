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


def _rounded_open_box(width: float, depth: float, height: float, wall: float, bottom: float) -> cq.Workplane:
    """One-piece rounded hollow storage body with an open top."""
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.014)
        .edges(">Z")
        .fillet(0.004)
    )
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, bottom + height / 2.0))
    )
    return outer.cut(inner)


def _rounded_slab(width: float, depth: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, thickness)
        .edges("|Z")
        .fillet(0.016)
        .edges(">Z")
        .fillet(0.004)
    )


def _rectangular_frame(width: float, depth: float, rail: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height)
    cut = cq.Workplane("XY").box(width - 2.0 * rail, depth - 2.0 * rail, height * 3.0)
    return outer.cut(cut).edges("|Z").fillet(0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_hinged_sewing_box")

    # Real compact sewing-box proportions in meters.
    width = 0.38
    depth = 0.24
    body_height = 0.150
    wall = 0.014
    bottom = 0.018
    lid_width = 0.392
    lid_depth = 0.258
    lid_thickness = 0.020
    hinge_y = depth / 2.0 + 0.028
    hinge_z = body_height + 0.016
    lid_center_y = (depth / 2.0 + 0.006) - hinge_y - lid_depth / 2.0
    lid_center_z = (body_height + 0.012) - hinge_z

    matte_walnut = Material("matte_dark_walnut", rgba=(0.28, 0.15, 0.075, 1.0))
    satin_walnut = Material("satin_brown_walnut", rgba=(0.36, 0.20, 0.10, 1.0))
    end_grain = Material("subtle_end_grain", rgba=(0.20, 0.105, 0.055, 1.0))
    satin_brass = Material("satin_brass", rgba=(0.86, 0.64, 0.30, 1.0))
    dark_shadow = Material("tight_shadow_seam", rgba=(0.025, 0.023, 0.020, 1.0))
    charcoal_felt = Material("charcoal_wool_felt", rgba=(0.045, 0.055, 0.060, 1.0))
    blush_pincushion = Material("muted_blush_fabric", rgba=(0.62, 0.25, 0.23, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_open_box(width, depth, body_height, wall, bottom), "rounded_hollow_body"),
        material=matte_walnut,
        name="body_shell",
    )

    # Clean shadow break just below the lid seam; each strip is seated into the shell.
    body.visual(
        Box((width - 0.052, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.001, body_height - 0.020)),
        material=dark_shadow,
        name="front_seam",
    )
    body.visual(
        Box((width - 0.052, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.001, body_height - 0.020)),
        material=dark_shadow,
        name="rear_seam",
    )
    for x, name in ((-width / 2.0 - 0.001, "side_seam_0"), (width / 2.0 + 0.001, "side_seam_1")):
        body.visual(
            Box((0.003, depth - 0.055, 0.004)),
            origin=Origin(xyz=(x, 0.0, body_height - 0.020)),
            material=dark_shadow,
            name=name,
        )

    # Satin top lip frames the open storage mouth without capping it.
    lip_z = body_height + 0.0015
    body.visual(
        Box((width - 0.025, 0.010, 0.005)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.006, lip_z)),
        material=satin_walnut,
        name="front_lip",
    )
    body.visual(
        Box((width - 0.025, 0.010, 0.005)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.006, lip_z)),
        material=satin_walnut,
        name="rear_lip",
    )
    for x, name in ((-width / 2.0 + 0.006, "side_lip_0"), (width / 2.0 - 0.006, "side_lip_1")):
        body.visual(
            Box((0.010, depth - 0.028, 0.005)),
            origin=Origin(xyz=(x, 0.0, lip_z)),
            material=satin_walnut,
            name=name,
        )

    # Interior sewing storage details are fixed and supported by the box floor.
    inner_width = width - 2.0 * wall - 0.018
    inner_depth = depth - 2.0 * wall - 0.018
    body.visual(
        Box((inner_width, inner_depth, 0.004)),
        origin=Origin(xyz=(0.0, -0.004, bottom + 0.002)),
        material=charcoal_felt,
        name="felt_floor",
    )
    body.visual(
        Box((inner_width, 0.005, 0.034)),
        origin=Origin(xyz=(0.0, -0.028, bottom + 0.017)),
        material=end_grain,
        name="cross_divider",
    )
    body.visual(
        Box((0.005, inner_depth * 0.48, 0.034)),
        origin=Origin(xyz=(0.060, -0.074, bottom + 0.017)),
        material=end_grain,
        name="front_divider",
    )
    body.visual(
        Box((0.070, 0.045, 0.012)),
        origin=Origin(xyz=(-0.090, -0.070, bottom + 0.008)),
        material=blush_pincushion,
        name="pincushion_pad",
    )
    for x, name in ((0.112, "spool_peg_0"), (0.140, "spool_peg_1"), (0.168, "spool_peg_2")):
        body.visual(
            Cylinder(radius=0.004, length=0.046),
            origin=Origin(xyz=(x, 0.045, bottom + 0.023)),
            material=satin_brass,
            name=name,
        )

    # Simple rear hinge supports: two rear leaves, saddles, and outer barrels.
    body.visual(
        Box((0.086, 0.018, 0.006)),
        origin=Origin(xyz=(-0.132, depth / 2.0 + 0.008, body_height - 0.002)),
        material=satin_brass,
        name="rear_hinge_0_shelf",
    )
    body.visual(
        Box((0.086, 0.008, 0.026)),
        origin=Origin(xyz=(-0.132, hinge_y - 0.011, body_height + 0.003)),
        material=satin_brass,
        name="rear_hinge_0_leaf",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.076),
        origin=Origin(xyz=(-0.132, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_brass,
        name="rear_hinge_0_barrel",
    )
    body.visual(
        Box((0.086, 0.018, 0.006)),
        origin=Origin(xyz=(0.132, depth / 2.0 + 0.008, body_height - 0.002)),
        material=satin_brass,
        name="rear_hinge_1_shelf",
    )
    body.visual(
        Box((0.086, 0.008, 0.026)),
        origin=Origin(xyz=(0.132, hinge_y - 0.011, body_height + 0.003)),
        material=satin_brass,
        name="rear_hinge_1_leaf",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.076),
        origin=Origin(xyz=(0.132, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_brass,
        name="rear_hinge_1_barrel",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_rounded_slab(lid_width, lid_depth, lid_thickness), "rounded_lid_slab"),
        origin=Origin(xyz=(0.0, lid_center_y, lid_center_z)),
        material=satin_walnut,
        name="lid_slab",
    )
    lid.visual(
        mesh_from_cadquery(_rectangular_frame(lid_width - 0.026, lid_depth - 0.026, 0.025, 0.007), "raised_lid_frame"),
        origin=Origin(xyz=(0.0, lid_center_y, lid_center_z + lid_thickness / 2.0 + 0.0035)),
        material=matte_walnut,
        name="lid_frame",
    )
    lid.visual(
        Box((lid_width - 0.092, lid_depth - 0.092, 0.004)),
        origin=Origin(xyz=(0.0, lid_center_y - 0.004, lid_center_z + lid_thickness / 2.0 + 0.002)),
        material=charcoal_felt,
        name="recessed_panel",
    )
    lid.visual(
        Box((0.116, 0.028, 0.005)),
        origin=Origin(xyz=(0.0, lid_center_y - lid_depth / 2.0 - 0.001, lid_center_z - 0.001)),
        material=satin_brass,
        name="front_pull_plate",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(
            xyz=(0.0, lid_center_y - lid_depth / 2.0 - 0.007, lid_center_z - 0.001),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_brass,
        name="front_pull_roll",
    )
    lid.visual(
        Box((0.128, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, -0.011, -0.008)),
        material=satin_brass,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.104),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_brass,
        name="center_barrel",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="lid_slab",
        negative_elem="body_shell",
        name="closed lid keeps a tight shadow seam above the body",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.18,
        elem_a="lid_slab",
        elem_b="body_shell",
        name="closed lid covers the framed storage opening",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        min_overlap=0.012,
        elem_a="center_barrel",
        elem_b="rear_hinge_0_barrel",
        name="left hinge knuckle shares the pivot line",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        min_overlap=0.012,
        elem_a="center_barrel",
        elem_b="rear_hinge_1_barrel",
        name="right hinge knuckle shares the pivot line",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_slab")
    with ctx.pose({hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_slab")

    ctx.check(
        "hinged lid opens upward around the rear pivot",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10
        and open_aabb[0][1] > closed_aabb[0][1] - 0.01,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
